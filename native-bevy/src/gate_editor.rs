//! Top-down 2D gate-path editor (egui) — ported from `src/path-editor.js`.
//!
//! A Placement-phase modal that edits a closed-loop chain of gate control points on the world's
//! horizontal plane (height on the up-axis via Z/X), previews the closed Catmull-Rom curve, and on
//! Accept rebuilds the live [`RaceCourse`], respawns the 3D frames (via `GateVisualsDirty`), and
//! persists the path. Draws a downsampled top-down cloud backdrop (height-band filtered) from
//! `SplatScene::cloud_points`; live per-gate clearance still needs the octree (Phase 7) → deferred.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use std::path::Path;

use crate::app_state::{AppState, CurrentSceneConfig, WorldUp};
use crate::gate_plugin::{world_up_vec, GateVisualsDirty, RaceCourse};
use crate::persistence::{self, SceneRecord};
use crate::spline;
use crate::splat_plugin::SplatScene;

const MIN_GATES: usize = 3;
const ZOOM_STEP: f32 = 1.15;
const PICK_PX: f32 = 9.0;
const DOT_PX: f32 = 9.0;
const CANVAS_PX: f32 = 560.0;

/// Editor state. `active` is read by the placement systems + `menu::handle_esc` so they yield while
/// the editor is open (camera/spawn editing + normal Esc pause).
#[derive(Resource, Default)]
pub struct GateEditor {
    pub active: bool,
    initialized: bool,
    points: Vec<Vec3>,
    selected: Option<usize>,
    view_a: f32,
    view_b: f32,
    zoom: f32,
    h_min: f32,
    h_max: f32,
    h_lo: f32,
    h_hi: f32,
    gate_size: f32,
    dragging: bool,
}

pub struct GateEditorPlugin;

impl Plugin for GateEditorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GateEditor>();
        app.add_systems(
            EguiPrimaryContextPass,
            gate_editor_ui.run_if(in_state(AppState::Placement)),
        );
        app.add_systems(OnExit(AppState::Placement), close_editor);
    }
}

fn close_editor(mut editor: ResMut<GateEditor>) {
    editor.active = false;
    editor.initialized = false;
}

/// World <-> top-down editor-plane mapping: `(a, b, h)` where `a` -> screen-right, `b` -> screen-down
/// (egui y-down), `h` = height along the world up-axis.
///
/// Z-up: plane = XY, height = +Z. The `b` axis is NEGATED (`b = -Y`) so world +Y points UP on screen,
/// matching the 3D orbit view (a top-down `compute_orbit_camera` for Z-up has screen-right=+X,
/// screen-down=-Y). Without the negation the editor was vertically flipped — the reported "仰视图".
///
/// COLMAP (Y-down): plane = XZ. Height is `h = -Y` (altitude): `+Y` is gravity/down, so the world
/// up-axis is `-Y` and `h` must be `-Y` for Z/X (lower/raise) and the height readout to move gates UP
/// toward the sky — using `+Y` inverted them. The `b` axis is also NEGATED (`b = -Z`) for the same
/// reason as Z-up: the up-axis is -Y, so a top-down view looks DOWN along +Y, and a non-mirrored
/// top-down projection needs `b = -Z` (`right=+X, up=+Z` → `X×Z = -Y = -forward`). Without it the map
/// was a mirrored bottom-up projection — the reported "仰视图" — the same flip already fixed for Z-up.
fn to_plane(p: Vec3, wu: WorldUp) -> (f32, f32, f32) {
    match wu {
        WorldUp::Zup => (p.x, -p.y, p.z),
        WorldUp::Colmap => (p.x, -p.z, -p.y),
    }
}

fn from_plane(a: f32, b: f32, h: f32, wu: WorldUp) -> Vec3 {
    match wu {
        WorldUp::Zup => Vec3::new(a, -b, h),
        WorldUp::Colmap => Vec3::new(a, -h, -b),
    }
}

fn w2c(a: f32, b: f32, va: f32, vb: f32, zoom: f32, c: egui::Pos2) -> egui::Pos2 {
    egui::pos2(c.x + (a - va) * zoom, c.y + (b - vb) * zoom)
}

fn c2w(p: egui::Pos2, va: f32, vb: f32, zoom: f32, c: egui::Pos2) -> (f32, f32) {
    ((p.x - c.x) / zoom + va, (p.y - c.y) / zoom + vb)
}

fn hit_test(pts: &[Vec3], wu: WorldUp, mp: egui::Pos2, va: f32, vb: f32, zoom: f32, c: egui::Pos2) -> Option<usize> {
    for i in (0..pts.len()).rev() {
        let (a, b, _) = to_plane(pts[i], wu);
        let cp = w2c(a, b, va, vb, zoom, c);
        if (mp.x - cp.x).abs() <= PICK_PX && (mp.y - cp.y).abs() <= PICK_PX {
            return Some(i);
        }
    }
    None
}

fn init_editor(ed: &mut GateEditor, course: &RaceCourse, config: &CurrentSceneConfig) {
    let wu = config.0.world_up;
    ed.points = course.shared().gates.iter().map(|g| Vec3::new(g.pos.x, g.pos.y, g.pos.z)).collect();
    ed.gate_size = course.shared().gate_size;
    ed.selected = None;
    ed.dragging = false;
    let (sa, sb, sh) = to_plane(Vec3::from(config.0.spawn), wu);
    ed.h_min = sh - 3.0;
    ed.h_max = sh + 5.0;
    ed.h_lo = sh - 50.0;
    ed.h_hi = sh + 50.0;
    if ed.points.is_empty() {
        ed.view_a = sa;
        ed.view_b = sb;
        ed.zoom = CANVAS_PX / 60.0;
    } else {
        let (mut mna, mut mxa, mut mnb, mut mxb) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
        for p in &ed.points {
            let (a, b, _) = to_plane(*p, wu);
            mna = mna.min(a); mxa = mxa.max(a); mnb = mnb.min(b); mxb = mxb.max(b);
        }
        ed.view_a = (mna + mxa) * 0.5;
        ed.view_b = (mnb + mxb) * 0.5;
        let span = (mxa - mna).max(mxb - mnb).max(1.0);
        ed.zoom = (CANVAS_PX / (span * 1.4)).clamp(CANVAS_PX / 160.0, 60.0);
    }
    ed.initialized = true;
}

fn gate_editor_ui(
    mut contexts: EguiContexts,
    mut editor: ResMut<GateEditor>,
    course: Option<ResMut<RaceCourse>>,
    config: Option<Res<CurrentSceneConfig>>,
    splat: Res<SplatScene>,
    mut dirty: ResMut<GateVisualsDirty>,
) -> Result {
    if !editor.active {
        return Ok(());
    }
    let (Some(mut course), Some(config)) = (course, config) else {
        editor.active = false;
        return Ok(());
    };
    let wu = config.0.world_up;
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    if !editor.initialized {
        init_editor(&mut editor, &course, &config);
    }

    let screen = ctx.viewport_rect();
    ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::new("gate_dim")))
        .rect_filled(screen, 0.0, egui::Color32::from_black_alpha(220));

    let spawn = Vec3::from(config.0.spawn);
    let mut accept = false;
    let mut cancel = false;

    egui::Window::new("Edit Gate Path")
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            ui.label(
                egui::RichText::new(
                    "L-click add/select · drag move · Z/X lower/raise · Del remove · Backspace undo · wheel zoom · right-drag pan · Enter accept · Esc cancel",
                )
                .size(11.0)
                .color(egui::Color32::from_rgb(150, 170, 200)),
            );
            ui.add_space(6.0);

            let (rect, response) =
                ui.allocate_exact_size(egui::vec2(CANVAS_PX, CANVAS_PX), egui::Sense::click_and_drag());
            let c = rect.center();
            let mut va = editor.view_a;
            let mut vb = editor.view_b;
            let mut zoom = editor.zoom;

            // --- Interaction ---
            if response.hovered() {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    if let Some(mp) = response.hover_pos() {
                        let (pa, pb) = c2w(mp, va, vb, zoom, c);
                        zoom = (zoom * if scroll > 0.0 { ZOOM_STEP } else { 1.0 / ZOOM_STEP }).clamp(0.05, 500.0);
                        let (qa, qb) = c2w(mp, va, vb, zoom, c);
                        va += pa - qa;
                        vb += pb - qb;
                    }
                }
            }
            if response.dragged_by(egui::PointerButton::Secondary) || response.dragged_by(egui::PointerButton::Middle) {
                let d = response.drag_delta();
                va -= d.x / zoom;
                vb -= d.y / zoom;
            }
            if response.drag_started_by(egui::PointerButton::Primary) {
                editor.dragging = false;
                // Hit-test the PRESS ORIGIN (where the button went down), NOT interact_pointer_pos:
                // by the time `drag_started` fires the pointer has already moved past egui's drag
                // threshold (~6px), so testing the moved position made near-edge grabs miss — the
                // reported "selected but won't drag" bug.
                let press = ui
                    .input(|i| i.pointer.press_origin())
                    .or_else(|| response.interact_pointer_pos());
                if let Some(mp) = press {
                    if let Some(i) = hit_test(&editor.points, wu, mp, va, vb, zoom, c) {
                        editor.selected = Some(i);
                        editor.dragging = true;
                    } else if let Some(sel) = editor.selected {
                        // No precise hit, but a gate is already selected: grab it with a more
                        // forgiving radius so a slightly-off press still drags the lit-up point.
                        let (a, b, _) = to_plane(editor.points[sel], wu);
                        if (mp - w2c(a, b, va, vb, zoom, c)).length() <= PICK_PX * 2.5 {
                            editor.dragging = true;
                        }
                    }
                }
            }
            if editor.dragging && response.dragged_by(egui::PointerButton::Primary) {
                if let Some(i) = editor.selected {
                    let d = response.drag_delta();
                    let (a, b, h) = to_plane(editor.points[i], wu);
                    editor.points[i] = from_plane(a + d.x / zoom, b + d.y / zoom, h, wu);
                }
            }
            if response.drag_stopped() {
                editor.dragging = false;
            }
            if response.clicked() {
                if let Some(mp) = response.interact_pointer_pos() {
                    match hit_test(&editor.points, wu, mp, va, vb, zoom, c) {
                        Some(i) => editor.selected = Some(i),
                        None => {
                            let (a, b) = c2w(mp, va, vb, zoom, c);
                            let h = 0.5 * (editor.h_min + editor.h_max);
                            editor.points.push(from_plane(a, b, h, wu));
                            editor.selected = Some(editor.points.len() - 1);
                        }
                    }
                }
            }

            // --- Keyboard ---
            let (kz, kx, shift, kdel, kback, kenter, kesc) = ui.input(|i| (
                i.key_pressed(egui::Key::Z),
                i.key_pressed(egui::Key::X),
                i.modifiers.shift,
                i.key_pressed(egui::Key::Delete),
                i.key_pressed(egui::Key::Backspace),
                i.key_pressed(egui::Key::Enter),
                i.key_pressed(egui::Key::Escape),
            ));
            if let Some(sel) = editor.selected {
                let step = if shift { 1.0 } else { 0.1 };
                let dh = (if kx { step } else { 0.0 }) - (if kz { step } else { 0.0 });
                if dh != 0.0 {
                    let (a, b, h) = to_plane(editor.points[sel], wu);
                    editor.points[sel] = from_plane(a, b, (h + dh).clamp(editor.h_min, editor.h_max), wu);
                }
            }
            if kdel || kback {
                if let Some(sel) = editor.selected {
                    editor.points.remove(sel);
                    editor.selected = None;
                } else if kback && !editor.points.is_empty() {
                    editor.points.pop();
                }
            }
            if kesc {
                cancel = true;
            }
            if kenter && editor.points.len() >= MIN_GATES {
                accept = true;
            }

            editor.view_a = va;
            editor.view_b = vb;
            editor.zoom = zoom;

            // --- Draw ---
            let p = ui.painter_at(rect);
            p.rect_filled(rect, 0.0, egui::Color32::from_rgb(10, 10, 18));
            // Grid.
            let step = if zoom > 8.0 { 10.0 } else if zoom > 2.0 { 50.0 } else { 200.0 };
            let grid = egui::Stroke::new(1.0, egui::Color32::from_rgb(26, 26, 40));
            let (a0, _) = c2w(rect.left_top(), va, vb, zoom, c);
            let (a1, _) = c2w(rect.right_bottom(), va, vb, zoom, c);
            let (_, b0) = c2w(rect.left_top(), va, vb, zoom, c);
            let (_, b1) = c2w(rect.right_bottom(), va, vb, zoom, c);
            let mut gx = (a0 / step).floor() * step;
            while gx <= a1 {
                let x = w2c(gx, 0.0, va, vb, zoom, c).x;
                p.line_segment([egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], grid);
                gx += step;
            }
            let mut gz = (b0 / step).floor() * step;
            while gz <= b1 {
                let y = w2c(0.0, gz, va, vb, zoom, c).y;
                p.line_segment([egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)], grid);
                gz += step;
            }
            // Scene backdrop: faint top-down dots of the splat cloud, culled to the current height
            // band [h_min, h_max] so you see the structures at gate height (ported from path-editor.js).
            if !splat.cloud_points.is_empty() {
                let dot = egui::Color32::from_rgba_unmultiplied(150, 190, 225, 140);
                let uv = egui::epaint::WHITE_UV;
                let r = 0.75;
                let (hmn, hmx) = (editor.h_min, editor.h_max);
                let mut mesh = egui::epaint::Mesh::default();
                for pt in &splat.cloud_points {
                    let (a, b, hgt) = to_plane(Vec3::from(*pt), wu);
                    if hgt < hmn || hgt > hmx {
                        continue;
                    }
                    let cp = w2c(a, b, va, vb, zoom, c);
                    if !rect.contains(cp) {
                        continue;
                    }
                    let i = mesh.vertices.len() as u32;
                    for off in [(-r, -r), (r, -r), (r, r), (-r, r)] {
                        mesh.vertices.push(egui::epaint::Vertex { pos: egui::pos2(cp.x + off.0, cp.y + off.1), uv, color: dot });
                    }
                    mesh.indices.extend_from_slice(&[i, i + 1, i + 2, i, i + 2, i + 3]);
                    if mesh.vertices.len() >= 60000 {
                        p.add(egui::Shape::mesh(std::mem::take(&mut mesh)));
                    }
                }
                if !mesh.indices.is_empty() {
                    p.add(egui::Shape::mesh(mesh));
                }
            }
            // Spawn.
            let (ssa, ssb, _) = to_plane(spawn, wu);
            let scp = w2c(ssa, ssb, va, vb, zoom, c);
            if rect.contains(scp) {
                p.circle_filled(scp, 5.0, egui::Color32::from_rgb(70, 170, 255));
                p.circle_stroke(scp, 5.0, egui::Stroke::new(1.0, egui::Color32::WHITE));
                p.text(scp + egui::vec2(7.0, -10.0), egui::Align2::LEFT_TOP, "spawn", egui::FontId::proportional(11.0), egui::Color32::from_rgb(70, 170, 255));
            }
            // Spline preview.
            let n = editor.points.len();
            if n >= MIN_GATES {
                let cg: Vec<cgmath::Vector3<f32>> = editor.points.iter().map(|q| cgmath::Vector3::new(q.x, q.y, q.z)).collect();
                let samples = spline::sample_closed(&cg, 16);
                let stroke = egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 220, 120));
                for w in samples.windows(2) {
                    let (a0p, b0p, _) = to_plane(Vec3::new(w[0].pos.x, w[0].pos.y, w[0].pos.z), wu);
                    let (a1p, b1p, _) = to_plane(Vec3::new(w[1].pos.x, w[1].pos.y, w[1].pos.z), wu);
                    p.line_segment([w2c(a0p, b0p, va, vb, zoom, c), w2c(a1p, b1p, va, vb, zoom, c)], stroke);
                }
                if let (Some(f), Some(l)) = (samples.first(), samples.last()) {
                    let (fa, fb, _) = to_plane(Vec3::new(f.pos.x, f.pos.y, f.pos.z), wu);
                    let (la, lb, _) = to_plane(Vec3::new(l.pos.x, l.pos.y, l.pos.z), wu);
                    p.line_segment([w2c(la, lb, va, vb, zoom, c), w2c(fa, fb, va, vb, zoom, c)], stroke);
                }
            }
            // Gates.
            let cg_tan: Vec<cgmath::Vector3<f32>> = editor.points.iter().map(|q| cgmath::Vector3::new(q.x, q.y, q.z)).collect();
            for i in 0..n {
                let (a, b, h) = to_plane(editor.points[i], wu);
                let cp = w2c(a, b, va, vb, zoom, c);
                let sel = editor.selected == Some(i);
                let fill = egui::Color32::from_rgba_unmultiplied(80, 220, 120, 217);
                let half = DOT_PX * 0.5;
                let gr = egui::Rect::from_center_size(cp, egui::vec2(DOT_PX, DOT_PX));
                p.rect_filled(gr, 0.0, fill);
                let border = if sel { egui::Color32::from_rgb(255, 208, 0) } else { egui::Color32::WHITE };
                let bw = if sel { 2.5 } else { 1.2 };
                p.line_segment([gr.left_top(), gr.right_top()], egui::Stroke::new(bw, border));
                p.line_segment([gr.right_top(), gr.right_bottom()], egui::Stroke::new(bw, border));
                p.line_segment([gr.right_bottom(), gr.left_bottom()], egui::Stroke::new(bw, border));
                p.line_segment([gr.left_bottom(), gr.left_top()], egui::Stroke::new(bw, border));
                p.text(cp + egui::vec2(half + 1.0, -half - 11.0), egui::Align2::LEFT_TOP, format!("{i}"), egui::FontId::proportional(11.0), egui::Color32::WHITE);
                if n >= MIN_GATES {
                    let td = spline::tangent_at_point(&cg_tan, i);
                    let (ta, tb, _) = to_plane(Vec3::new(td.x, td.y, td.z), wu);
                    let mag = (ta * ta + tb * tb).sqrt().max(1e-3);
                    let tip = egui::pos2(cp.x + ta / mag * 16.0, cp.y + tb / mag * 16.0);
                    let acol = if sel { egui::Color32::from_rgb(255, 208, 0) } else { egui::Color32::from_rgb(200, 200, 255) };
                    p.line_segment([cp, tip], egui::Stroke::new(1.5, acol));
                }
                if sel {
                    p.text(cp + egui::vec2(half + 1.0, half + 1.0), egui::Align2::LEFT_TOP, format!("h = {h:.2} m"), egui::FontId::proportional(11.0), egui::Color32::from_rgb(255, 208, 0));
                }
            }

            // --- Controls ---
            ui.add_space(6.0);
            let (mut hmin, mut hmax, hlo, hhi) = (editor.h_min, editor.h_max, editor.h_lo, editor.h_hi);
            ui.horizontal(|ui| {
                ui.label("height min");
                ui.add(egui::Slider::new(&mut hmin, hlo..=hhi).fixed_decimals(1));
                ui.label("max");
                ui.add(egui::Slider::new(&mut hmax, hlo..=hhi).fixed_decimals(1));
            });
            ui.horizontal(|ui| {
                ui.label("gate size");
                ui.add(egui::Slider::new(&mut editor.gate_size, 0.4..=5.0).fixed_decimals(1));
            });
            editor.h_min = hmin.min(hmax);
            editor.h_max = hmin.max(hmax);
            let (hmn, hmx) = (editor.h_min, editor.h_max);
            for q in editor.points.iter_mut() {
                let (a, b, h) = to_plane(*q, wu);
                let ch = h.clamp(hmn, hmx);
                if ch != h {
                    *q = from_plane(a, b, ch, wu);
                }
            }

            ui.add_space(4.0);
            let n = editor.points.len();
            let status = if n == 0 {
                "click to drop the first gate".to_string()
            } else if n < MIN_GATES {
                format!("{n} / {MIN_GATES} gates — need {} more to close the loop", MIN_GATES - n)
            } else {
                format!("{n} gates")
            };
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(status).size(12.0).color(egui::Color32::GRAY));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let can = n >= MIN_GATES;
                    if ui.add_enabled(can, egui::Button::new("Accept")).clicked() {
                        accept = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
        });

    // --- Apply / cancel (after the window so the resource borrows are free) ---
    if accept && editor.points.len() >= MIN_GATES {
        let pts: Vec<[f32; 3]> = editor.points.iter().map(|q| [q.x, q.y, q.z]).collect();
        let cg: Vec<cgmath::Vector3<f32>> = editor.points.iter().map(|q| cgmath::Vector3::new(q.x, q.y, q.z)).collect();
        let best = course.shared().best_lap_ms;
        let wuv = world_up_vec(wu);
        // Rebuild every player's tracker on the new path (identical geometry, fresh pass state).
        for c in &mut course.players {
            c.gate_size = editor.gate_size;
            c.rebuild(&cg, wuv);
            c.visible = true;
        }
        if let Some(path) = &splat.ply_path {
            let key = persistence::scene_key(Path::new(path));
            let rec = SceneRecord { points: pts, gate_size: editor.gate_size, best_lap_ms: best };
            if let Err(e) = persistence::save_scene_record(&key, &rec) {
                warn!("[GateEditor] failed to save path: {e}");
            }
        }
        dirty.0 = true;
        info!("[GateEditor] accepted {} gates", editor.points.len());
        editor.active = false;
        editor.initialized = false;
    } else if cancel {
        info!("[GateEditor] cancelled");
        editor.active = false;
        editor.initialized = false;
    }

    Ok(())
}
