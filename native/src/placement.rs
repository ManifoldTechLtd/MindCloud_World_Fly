/// Placement phase: orbit camera around spawn point, WASD/QE to move spawn,
/// world-up selector, heading recording, start flight / back actions.

use std::collections::HashSet;
use cgmath::{InnerSpace, Point3, Quaternion, Rad, Rotation, Rotation3, Vector3};
use winit::keyboard::KeyCode;

use crate::app_state::{self, SceneConfig, WorldUp};
use crate::scene_mesh;

/// Orbit camera state.
pub struct OrbitState {
    pub yaw: f32,       // degrees
    pub pitch: f32,     // degrees, clamped to (-89, 89)
    pub dist: f32,      // distance from spawn
    pub dragging: bool,
    pub last_mouse: [f32; 2],
    pub keys_down: HashSet<KeyCode>,
}

impl Default for OrbitState {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: -20.0,
            dist: 10.0,
            dragging: false,
            last_mouse: [0.0; 2],
            keys_down: HashSet::new(),
        }
    }
}

/// Result of a placement frame update.
pub enum PlacementAction {
    None,
    StartFlight,
    GoBack,
}

/// Update spawn point based on WASD/QE keys relative to orbit yaw.
pub fn update_spawn(config: &mut SceneConfig, orbit: &OrbitState, dt: f32) {
    let move_speed = 3.0 * dt;
    let yaw_rad = orbit.yaw * std::f32::consts::PI / 180.0;
    let sin_y = yaw_rad.sin();
    let cos_y = yaw_rad.cos();

    match config.world_up {
        WorldUp::Zup => {
            if orbit.keys_down.contains(&KeyCode::KeyW) { config.spawn[0] -= sin_y * move_speed; config.spawn[1] += cos_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyS) { config.spawn[0] += sin_y * move_speed; config.spawn[1] -= cos_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyA) { config.spawn[0] -= cos_y * move_speed; config.spawn[1] -= sin_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyD) { config.spawn[0] += cos_y * move_speed; config.spawn[1] += sin_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyE) { config.spawn[2] += move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyQ) { config.spawn[2] -= move_speed; }
        }
        WorldUp::Colmap => {
            // Forward=+X at yaw=0, right=+Z→-Z (screen right) at yaw=0
            if orbit.keys_down.contains(&KeyCode::KeyW) { config.spawn[0] += cos_y * move_speed; config.spawn[2] += sin_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyS) { config.spawn[0] -= cos_y * move_speed; config.spawn[2] -= sin_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyD) { config.spawn[0] += sin_y * move_speed; config.spawn[2] -= cos_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyA) { config.spawn[0] -= sin_y * move_speed; config.spawn[2] += cos_y * move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyE) { config.spawn[1] -= move_speed; }
            if orbit.keys_down.contains(&KeyCode::KeyQ) { config.spawn[1] += move_speed; }
        }
    }

    // Record heading (wrap 0..360)
    config.heading_deg = ((orbit.yaw % 360.0) + 360.0) % 360.0;
}

/// Compute orbit camera position and rotation quaternion.
/// Returns (position, rotation) for web-splat camera.
pub fn compute_orbit_camera(config: &SceneConfig, orbit: &OrbitState) -> (Point3<f32>, Quaternion<f32>) {
    let sp = config.spawn;
    let yaw_rad = orbit.yaw * std::f32::consts::PI / 180.0;
    let pitch_rad = orbit.pitch * std::f32::consts::PI / 180.0;
    let d = orbit.dist;
    let cos_p = pitch_rad.cos();
    let sin_p = pitch_rad.sin();

    let cam_offset = match config.world_up {
        WorldUp::Zup => Vector3::new(
            -yaw_rad.sin() * cos_p * d,
            -yaw_rad.cos() * cos_p * d,
            -sin_p * d,
        ),
        WorldUp::Colmap => Vector3::new(
            -yaw_rad.cos() * cos_p * d,
            sin_p * d,
            -yaw_rad.sin() * cos_p * d,
        ),
    };

    let cam_pos = Point3::new(sp[0] + cam_offset.x, sp[1] + cam_offset.y, sp[2] + cam_offset.z);
    let look_dir = Vector3::new(sp[0] - cam_pos.x, sp[1] - cam_pos.y, sp[2] - cam_pos.z).normalize();
    let up = match config.world_up {
        WorldUp::Zup => Vector3::new(0.0, 0.0, -1.0),
        WorldUp::Colmap => Vector3::new(0.0, 1.0, 0.0),
    };
    let cam_rot = Quaternion::look_at(look_dir, up);

    (cam_pos, cam_rot)
}

/// Handle mouse drag for orbit rotation.
pub fn on_mouse_move(orbit: &mut OrbitState, mx: f32, my: f32) {
    if orbit.dragging {
        let dx = mx - orbit.last_mouse[0];
        let dy = my - orbit.last_mouse[1];
        orbit.yaw += dx * 0.3;
        orbit.pitch = (orbit.pitch - dy * 0.3).clamp(-89.0, 89.0);
    }
    orbit.last_mouse = [mx, my];
}

/// Handle mouse scroll for orbit zoom.
pub fn on_scroll(orbit: &mut OrbitState, delta_y: f32) {
    orbit.dist *= 1.0 - delta_y * 0.1;
    orbit.dist = orbit.dist.clamp(0.5, 500.0);
}

/// Temporary input state for spawn/heading text fields (not yet applied).
pub struct PlacementInputState {
    pub spawn_text: [String; 3],
    pub heading_text: String,
}

impl PlacementInputState {
    pub fn from_config(config: &SceneConfig) -> Self {
        Self {
            spawn_text: [
                format!("{:.2}", config.spawn[0]),
                format!("{:.2}", config.spawn[1]),
                format!("{:.2}", config.spawn[2]),
            ],
            heading_text: format!("{:.1}", config.heading_deg),
        }
    }
}

/// Draw placement overlay UI. Returns action.
/// `scene_min`/`scene_max` are the scene AABB bounds for clamping input.
pub fn draw_overlay(
    ctx: &egui::Context,
    config: &mut SceneConfig,
    input_state: &mut PlacementInputState,
    orbit: &mut OrbitState,
    scene_min: [f32; 3],
    scene_max: [f32; 3],
) -> PlacementAction {
    let mut action = PlacementAction::None;

    // Sync text fields to live config values unless a text field currently has focus
    let any_focused = ctx.memory(|m| m.focused()).is_some_and(|id| {
        let spawn_ids: [egui::Id; 3] = std::array::from_fn(|i| egui::Id::new(format!("spawn_{}", i)));
        let heading_id = egui::Id::new("heading_input");
        spawn_ids.contains(&id) || id == heading_id
    });
    if !any_focused {
        input_state.spawn_text = [
            format!("{:.2}", config.spawn[0]),
            format!("{:.2}", config.spawn[1]),
            format!("{:.2}", config.spawn[2]),
        ];
        input_state.heading_text = format!("{:.1}", config.heading_deg);
    }

    let panel_frame = egui::Frame::NONE
        .fill(egui::Color32::from_black_alpha(230))
        .corner_radius(8.0)
        .inner_margin(egui::Margin::same(16));

    egui::Area::new(egui::Id::new("placement_ui"))
        .anchor(egui::Align2::CENTER_TOP, [0.0, 10.0])
        .show(ctx, |ui| {
            panel_frame.show(ui, |ui| {
                ui.set_min_width(420.0);
                ui.vertical_centered(|ui| {
                    ui.label(egui::RichText::new("Spawn Placement").size(22.0).color(egui::Color32::WHITE).strong());
                    ui.add_space(4.0);
                    ui.label(egui::RichText::new("WASD = move | QE = up/down | Mouse drag = orbit | Scroll = zoom")
                        .size(11.0).color(egui::Color32::from_rgb(180, 190, 200)));
                    ui.add_space(8.0);

                    // Spawn input fields
                    let field_width = 65.0;
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Spawn:").size(13.0).color(egui::Color32::WHITE));
                        ui.label(egui::RichText::new("X").size(11.0).color(egui::Color32::from_rgb(255, 120, 120)));
                        ui.add(egui::TextEdit::singleline(&mut input_state.spawn_text[0]).id(egui::Id::new("spawn_0")).desired_width(field_width).font(egui::TextStyle::Monospace));
                        ui.label(egui::RichText::new("Y").size(11.0).color(egui::Color32::from_rgb(120, 255, 120)));
                        ui.add(egui::TextEdit::singleline(&mut input_state.spawn_text[1]).id(egui::Id::new("spawn_1")).desired_width(field_width).font(egui::TextStyle::Monospace));
                        ui.label(egui::RichText::new("Z").size(11.0).color(egui::Color32::from_rgb(120, 160, 255)));
                        ui.add(egui::TextEdit::singleline(&mut input_state.spawn_text[2]).id(egui::Id::new("spawn_2")).desired_width(field_width).font(egui::TextStyle::Monospace));
                    });
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Heading:").size(13.0).color(egui::Color32::WHITE));
                        ui.add(egui::TextEdit::singleline(&mut input_state.heading_text).id(egui::Id::new("heading_input")).desired_width(field_width).font(egui::TextStyle::Monospace));
                        ui.label(egui::RichText::new("°").size(13.0).color(egui::Color32::WHITE));
                        ui.add_space(10.0);
                        if ui.button(egui::RichText::new(" Set ").size(13.0).color(egui::Color32::WHITE)).clicked() {
                            // Parse and clamp spawn
                            for i in 0..3 {
                                if let Ok(v) = input_state.spawn_text[i].trim().parse::<f32>() {
                                    config.spawn[i] = v.clamp(scene_min[i], scene_max[i]);
                                }
                            }
                            // Parse and clamp heading, also sync orbit yaw
                            if let Ok(v) = input_state.heading_text.trim().parse::<f32>() {
                                config.heading_deg = ((v % 360.0) + 360.0) % 360.0;
                                orbit.yaw = config.heading_deg;
                            }
                            // Update text fields to reflect clamped values
                            for i in 0..3 {
                                input_state.spawn_text[i] = format!("{:.2}", config.spawn[i]);
                            }
                            input_state.heading_text = format!("{:.1}", config.heading_deg);
                        }
                    });

                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("World Up:").size(13.0).color(egui::Color32::WHITE));
                        if ui.selectable_label(config.world_up == WorldUp::Zup, egui::RichText::new("Z-up").size(13.0)).clicked() {
                            config.world_up = WorldUp::Zup;
                        }
                        if ui.selectable_label(config.world_up == WorldUp::Colmap, egui::RichText::new("COLMAP (Y-down)").size(13.0)).clicked() {
                            config.world_up = WorldUp::Colmap;
                        }
                    });
                    ui.add_space(12.0);
                    ui.horizontal(|ui| {
                        if ui.button(egui::RichText::new("  Start Flight (Enter)  ").size(15.0).color(egui::Color32::WHITE)).clicked() {
                            action = PlacementAction::StartFlight;
                        }
                        ui.add_space(15.0);
                        if ui.button(egui::RichText::new("  Back (Esc)  ").size(13.0).color(egui::Color32::from_rgb(150, 150, 180))).clicked() {
                            action = PlacementAction::GoBack;
                        }
                    });
                });
            });
        });

    action
}
