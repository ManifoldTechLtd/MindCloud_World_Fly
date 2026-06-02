/// HUD overlay — fighter-jet style OSD + race panel.
/// Ported from src/hud.js + src/osd.js, rendered via egui custom paint.

use egui::{Color32, FontId, Pos2, Rect, RichText, Stroke, Vec2};

use crate::drone::{Drone, FlightMode};
use crate::gates::{self, GateCourse};

const GREEN: Color32 = Color32::from_rgb(0, 255, 100);
const DIM: Color32 = Color32::from_rgb(100, 160, 100);
const WARN: Color32 = Color32::from_rgb(255, 80, 80);
const BG: Color32 = Color32::from_rgba_premultiplied(0, 0, 0, 120);

/// Draw the complete HUD for one player inside the given viewport rectangle.
pub fn draw_hud(
    ctx: &egui::Context,
    drone: &Drone,
    course: Option<&GateCourse>,
    armed: bool,
    fps: f32,
    player_label: Option<&str>,
    viewport: Option<Rect>,
) {
    let vp = viewport.unwrap_or_else(|| ctx.screen_rect());
    let area_id = player_label.unwrap_or("sp");

    // ---- Info panel (top-left corner) ----
    egui::Area::new(egui::Id::new(format!("info_{}", area_id)))
        .fixed_pos(vp.min + Vec2::new(8.0, 8.0))
        .show(ctx, |ui| {
            ui.set_max_width(160.0);
            egui::Frame::none()
                .fill(Color32::from_black_alpha(160))
                .rounding(4.0)
                .inner_margin(6.0)
                .show(ui, |ui| {
                    if let Some(l) = player_label {
                        ui.label(RichText::new(l).color(Color32::LIGHT_BLUE).size(13.0).strong());
                    }
                    ui.label(RichText::new(format!("{:.0} FPS", fps)).color(Color32::GRAY).size(10.0));
                    let mode = match drone.flight_mode { FlightMode::Fpv => "FPV", FlightMode::Drone => "DRONE" };
                    let arm_c = if armed { GREEN } else { WARN };
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(mode).color(Color32::WHITE).size(12.0).strong());
                        ui.label(RichText::new(if armed { "ARMED" } else { "DISARMED" }).color(arm_c).size(12.0).strong());
                    });
                });
        });

    // ---- Race panel (top-left, below info) ----
    if let Some(course) = course {
        if course.visible && !course.gates.is_empty() {
            egui::Area::new(egui::Id::new(format!("race_{}", area_id)))
                .fixed_pos(vp.min + Vec2::new(8.0, 80.0))
                .show(ctx, |ui| {
                    ui.set_max_width(160.0);
                    egui::Frame::none()
                        .fill(Color32::from_black_alpha(180))
                        .rounding(4.0)
                        .inner_margin(6.0)
                        .show(ui, |ui| {
                            ui.label(RichText::new("FPV RACE").color(DIM).size(12.0).strong().italics());
                            let passed = course.passed_count();
                            let total = course.gates.len();
                            let next = if passed < total { passed + 1 } else { total };
                            ui.label(RichText::new(format!("GATE {:>2} / {}", next, total)).color(Color32::WHITE).size(12.0));
                            let lap_str = gates::format_lap(course.current_lap_ms);
                            let lap_c = if course.lap_start.is_some() { Color32::from_rgb(77, 220, 255) } else { DIM };
                            ui.label(RichText::new(&lap_str).color(lap_c).size(16.0).strong());
                            let best = course.best_lap_ms.map(|ms| gates::format_lap(ms)).unwrap_or("--:--.---".into());
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("BEST").color(DIM).size(10.0));
                                ui.label(RichText::new(&best).color(Color32::from_rgb(192, 132, 252)).size(11.0));
                            });
                        });
                });
        }
    }

    // ---- Fighter-jet OSD (custom paint directly on screen layer) ----
    {
        let painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Middle,
            egui::Id::new(format!("osd_{}", area_id)),
        ));
        let cx = vp.center().x;
        let cy = vp.center().y;
        let w = vp.width();
        let h = vp.height();

        draw_horizon(&painter, cx, cy, w, h, drone.body_pitch, drone.body_roll);
        draw_speed_tape(&painter, w, h, cx, cy, drone.speed);
        let altitude = match drone.world_up {
            crate::app_state::WorldUp::Zup => drone.z,
            crate::app_state::WorldUp::Colmap => -drone.y,
        };
        draw_alt_tape(&painter, w, h, cx, cy, altitude);
        draw_heading(&painter, w, h, cx, cy, drone.yaw);
        draw_vsi(&painter, w, h, cx, cy, drone.vertical_speed);

        // Debug: show world position XYZ
        let pos_text = format!("X:{:.2} Y:{:.2} Z:{:.2}", drone.x, drone.y, drone.z);
        painter.text(
            egui::Pos2::new(cx, vp.min.y + 20.0),
            egui::Align2::CENTER_CENTER,
            pos_text,
            egui::FontId::monospace(14.0),
            Color32::YELLOW,
        );

    }
}

fn draw_horizon(p: &egui::Painter, cx: f32, cy: f32, w: f32, h: f32, pitch: f32, roll: f32) {
    let horizon_w = w * 0.22;
    let px_per_deg = h / 60.0;
    let roll_rad = roll.to_radians();
    let cos_r = roll_rad.cos();
    let sin_r = roll_rad.sin();
    let pitch_off = pitch * px_per_deg;

    // Rotated horizon line (split with gap in center)
    let rotate = |dx: f32, dy: f32| -> Pos2 {
        Pos2::new(
            cx + dx * cos_r - dy * sin_r,
            cy + dx * sin_r + dy * cos_r,
        )
    };

    // Horizon line left
    p.line_segment([rotate(-horizon_w, pitch_off), rotate(-30.0, pitch_off)], Stroke::new(2.0, GREEN));
    // Horizon line right
    p.line_segment([rotate(30.0, pitch_off), rotate(horizon_w, pitch_off)], Stroke::new(2.0, GREEN));

    // Pitch ladder
    let ladder_angles: &[f32] = &[-30.0, -20.0, -10.0, -5.0, 5.0, 10.0, 20.0, 30.0];
    for &deg in ladder_angles {
        let y = pitch_off - deg * px_per_deg;
        if y.abs() > h * 0.4 { continue; }
        let lw = if (deg.abs() as i32) % 10 == 0 { 50.0 } else { 25.0 };
        let col = if deg > 0.0 { GREEN } else { DIM };
        p.line_segment([rotate(-lw, y), rotate(lw, y)], Stroke::new(1.0, col));
        // Labels
        let font = FontId::monospace(9.0);
        let txt = format!("{}", deg as i32);
        p.text(rotate(-lw - 18.0, y), egui::Align2::CENTER_CENTER, &txt, font.clone(), DIM);
        p.text(rotate(lw + 18.0, y), egui::Align2::CENTER_CENTER, &txt, font, DIM);
    }

    // Center reticle (fixed, not rotated)
    p.line_segment([Pos2::new(cx - 50.0, cy), Pos2::new(cx - 20.0, cy)], Stroke::new(2.0, GREEN));
    p.line_segment([Pos2::new(cx - 20.0, cy), Pos2::new(cx - 20.0, cy + 6.0)], Stroke::new(2.0, GREEN));
    p.line_segment([Pos2::new(cx + 50.0, cy), Pos2::new(cx + 20.0, cy)], Stroke::new(2.0, GREEN));
    p.line_segment([Pos2::new(cx + 20.0, cy), Pos2::new(cx + 20.0, cy + 6.0)], Stroke::new(2.0, GREEN));
    p.circle_stroke(Pos2::new(cx, cy), 3.0, Stroke::new(2.0, GREEN));
}

fn draw_speed_tape(p: &egui::Painter, w: f32, h: f32, cx: f32, cy: f32, speed: f32) {
    let tape_x = w * 0.22 + (cx - w / 2.0);
    let tape_h = h * 0.4;
    let tape_w = 50.0;
    let px_per_unit = tape_h / 20.0;
    let top = cy - tape_h / 2.0;

    // Background
    let rect = Rect::from_min_size(Pos2::new(tape_x - tape_w / 2.0, top), Vec2::new(tape_w, tape_h));
    p.rect_filled(rect, 0.0, BG);

    // Ticks
    let start = (speed - 10.0).floor() as i32;
    let end = (speed + 10.0).ceil() as i32;
    for v in start..=end {
        if v < 0 { continue; }
        let y = cy - (v as f32 - speed) * px_per_unit;
        if y < top || y > top + tape_h { continue; }
        if v % 5 == 0 {
            p.line_segment(
                [Pos2::new(tape_x + tape_w / 2.0, y), Pos2::new(tape_x + tape_w / 2.0 - 10.0, y)],
                Stroke::new(1.0, DIM),
            );
            p.text(Pos2::new(tape_x + tape_w / 2.0 - 14.0, y), egui::Align2::RIGHT_CENTER, format!("{}", v), FontId::monospace(9.0), DIM);
        } else {
            p.line_segment(
                [Pos2::new(tape_x + tape_w / 2.0, y), Pos2::new(tape_x + tape_w / 2.0 - 5.0, y)],
                Stroke::new(1.0, DIM),
            );
        }
    }

    // Current value box
    let box_r = Rect::from_min_size(Pos2::new(tape_x - tape_w / 2.0, cy - 10.0), Vec2::new(tape_w, 20.0));
    p.rect_filled(box_r, 0.0, Color32::from_black_alpha(180));
    p.line_segment([box_r.left_top(), box_r.right_top()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.right_top(), box_r.right_bottom()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.right_bottom(), box_r.left_bottom()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.left_bottom(), box_r.left_top()], Stroke::new(1.0, GREEN));
    p.text(box_r.center(), egui::Align2::CENTER_CENTER, format!("{:.1}", speed), FontId::monospace(11.0), GREEN);
    p.text(Pos2::new(tape_x, top - 10.0), egui::Align2::CENTER_CENTER, "SPD", FontId::monospace(9.0), DIM);
}

fn draw_alt_tape(p: &egui::Painter, w: f32, h: f32, cx: f32, cy: f32, alt: f32) {
    let tape_x = w * 0.78 + (cx - w / 2.0);
    let tape_h = h * 0.4;
    let tape_w = 50.0;
    let px_per_unit = tape_h / 40.0;
    let top = cy - tape_h / 2.0;

    let rect = Rect::from_min_size(Pos2::new(tape_x - tape_w / 2.0, top), Vec2::new(tape_w, tape_h));
    p.rect_filled(rect, 0.0, BG);

    let start = (alt - 20.0).floor() as i32;
    let end = (alt + 20.0).ceil() as i32;
    for v in start..=end {
        let y = cy - (v as f32 - alt) * px_per_unit;
        if y < top || y > top + tape_h { continue; }
        if v % 5 == 0 {
            p.line_segment(
                [Pos2::new(tape_x - tape_w / 2.0, y), Pos2::new(tape_x - tape_w / 2.0 + 10.0, y)],
                Stroke::new(1.0, DIM),
            );
            p.text(Pos2::new(tape_x - tape_w / 2.0 + 14.0, y), egui::Align2::LEFT_CENTER, format!("{}", v), FontId::monospace(9.0), DIM);
        } else {
            p.line_segment(
                [Pos2::new(tape_x - tape_w / 2.0, y), Pos2::new(tape_x - tape_w / 2.0 + 5.0, y)],
                Stroke::new(1.0, DIM),
            );
        }
    }

    let box_r = Rect::from_min_size(Pos2::new(tape_x - tape_w / 2.0, cy - 10.0), Vec2::new(tape_w, 20.0));
    p.rect_filled(box_r, 0.0, Color32::from_black_alpha(180));
    p.line_segment([box_r.left_top(), box_r.right_top()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.right_top(), box_r.right_bottom()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.right_bottom(), box_r.left_bottom()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.left_bottom(), box_r.left_top()], Stroke::new(1.0, GREEN));
    p.text(box_r.center(), egui::Align2::CENTER_CENTER, format!("{:.1}", alt), FontId::monospace(11.0), GREEN);
    p.text(Pos2::new(tape_x, top - 10.0), egui::Align2::CENTER_CENTER, "ALT", FontId::monospace(9.0), DIM);
}

fn draw_heading(p: &egui::Painter, w: f32, h: f32, cx: f32, cy: f32, yaw: f32) {
    let bar_y = cy - h / 2.0 + h * 0.08;
    let bar_w = w * 0.22;
    let bar_h = 20.0;
    let px_per_deg = bar_w / 90.0;
    let heading = ((yaw % 360.0) + 360.0) % 360.0;

    let rect = Rect::from_min_size(Pos2::new(cx - bar_w / 2.0, bar_y - bar_h / 2.0), Vec2::new(bar_w, bar_h));
    p.rect_filled(rect, 0.0, BG);

    let cardinals: &[(f32, &str)] = &[
        (0.0, "N"), (45.0, "NE"), (90.0, "E"), (135.0, "SE"),
        (180.0, "S"), (225.0, "SW"), (270.0, "W"), (315.0, "NW"),
    ];

    for d in (-180..=540).step_by(5) {
        let deg = ((d % 360) + 360) % 360;
        let mut diff = d as f32 - heading;
        if diff > 180.0 { diff -= 360.0; }
        if diff < -180.0 { diff += 360.0; }
        let x = cx + diff * px_per_deg;
        if x < cx - bar_w / 2.0 - 5.0 || x > cx + bar_w / 2.0 + 5.0 { continue; }

        if d % 10 == 0 {
            p.line_segment(
                [Pos2::new(x, bar_y + bar_h / 2.0), Pos2::new(x, bar_y + bar_h / 2.0 - 5.0)],
                Stroke::new(1.0, DIM),
            );
        }

        if let Some((_, label)) = cardinals.iter().find(|(a, _)| *a as i32 == deg) {
            let col = if deg == 0 { WARN } else { GREEN };
            p.text(Pos2::new(x, bar_y - bar_h / 2.0 + 6.0), egui::Align2::CENTER_CENTER, *label, FontId::monospace(9.0), col);
        } else if d % 30 == 0 {
            p.text(Pos2::new(x, bar_y - bar_h / 2.0 + 6.0), egui::Align2::CENTER_CENTER, format!("{}", deg), FontId::monospace(8.0), DIM);
        }
    }

    // Center triangle
    p.line_segment([Pos2::new(cx, bar_y + bar_h / 2.0 + 2.0), Pos2::new(cx - 4.0, bar_y + bar_h / 2.0 + 7.0)], Stroke::new(1.0, GREEN));
    p.line_segment([Pos2::new(cx, bar_y + bar_h / 2.0 + 2.0), Pos2::new(cx + 4.0, bar_y + bar_h / 2.0 + 7.0)], Stroke::new(1.0, GREEN));

    // Heading value
    let box_r = Rect::from_min_size(Pos2::new(cx - 20.0, bar_y + bar_h / 2.0 + 8.0), Vec2::new(40.0, 14.0));
    p.rect_filled(box_r, 0.0, Color32::from_black_alpha(180));
    p.line_segment([box_r.left_top(), box_r.right_top()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.right_top(), box_r.right_bottom()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.right_bottom(), box_r.left_bottom()], Stroke::new(1.0, GREEN));
    p.line_segment([box_r.left_bottom(), box_r.left_top()], Stroke::new(1.0, GREEN));
    p.text(box_r.center(), egui::Align2::CENTER_CENTER, format!("{}°", heading.round() as i32), FontId::monospace(10.0), GREEN);
}

fn draw_vsi(p: &egui::Painter, w: f32, h: f32, cx: f32, cy: f32, vs: f32) {
    let x = w * 0.85 + (cx - w / 2.0);
    let bar_h = h * 0.3;
    let bar_w = 6.0;
    let max_vs = 10.0;

    let rect = Rect::from_min_size(Pos2::new(x - bar_w / 2.0, cy - bar_h / 2.0), Vec2::new(bar_w, bar_h));
    p.rect_filled(rect, 0.0, BG);
    p.line_segment([rect.left_top(), rect.right_top()], Stroke::new(1.0, DIM));
    p.line_segment([rect.right_top(), rect.right_bottom()], Stroke::new(1.0, DIM));
    p.line_segment([rect.right_bottom(), rect.left_bottom()], Stroke::new(1.0, DIM));
    p.line_segment([rect.left_bottom(), rect.left_top()], Stroke::new(1.0, DIM));

    // Center line
    p.line_segment([Pos2::new(x - bar_w / 2.0 - 3.0, cy), Pos2::new(x + bar_w / 2.0 + 3.0, cy)], Stroke::new(1.0, DIM));

    // VS bar
    let clamped = vs.clamp(-max_vs, max_vs);
    let bar_len = (clamped / max_vs) * (bar_h / 2.0);
    let col = if vs > 0.0 { GREEN } else { WARN };
    if bar_len.abs() > 0.5 {
        let (min_y, max_y) = if bar_len > 0.0 { (cy - bar_len, cy) } else { (cy, cy - bar_len) };
        p.rect_filled(Rect::from_min_max(Pos2::new(x - bar_w / 2.0 + 1.0, min_y), Pos2::new(x + bar_w / 2.0 - 1.0, max_y)), 0.0, col);
    }

    p.text(Pos2::new(x, cy - bar_h / 2.0 - 8.0), egui::Align2::CENTER_CENTER, "VS", FontId::monospace(9.0), DIM);
    let vs_col = if vs < -2.0 { WARN } else { GREEN };
    p.text(Pos2::new(x, cy + bar_h / 2.0 + 8.0), egui::Align2::CENTER_CENTER, format!("{:.1}", vs), FontId::monospace(9.0), vs_col);
}
