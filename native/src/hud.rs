/// HUD overlay — telemetry + race panel.
/// Ported from src/hud.js + src/osd.js, rendered via egui.

use egui::{Color32, Pos2, Rect, RichText, Stroke, Vec2};

use crate::drone::Drone;
use crate::gates::{self, GateCourse};

/// Draw the full HUD for one player.
pub fn draw_hud(
    ctx: &egui::Context,
    drone: &Drone,
    course: Option<&GateCourse>,
    armed: bool,
    fps: f32,
    player_label: Option<&str>, // e.g. "P1" or None for single player
    viewport_rect: Option<Rect>, // None = full screen
) {
    let area_id = match player_label {
        Some(l) => format!("hud_{}", l),
        None => "hud".into(),
    };

    egui::Area::new(egui::Id::new(&area_id))
        .fixed_pos(viewport_rect.map_or(Pos2::new(10.0, 10.0), |r| r.min + Vec2::new(10.0, 10.0)))
        .show(ctx, |ui| {
            ui.set_max_width(viewport_rect.map_or(300.0, |r| r.width().min(300.0)));

            // ---- Flight telemetry ----
            egui::Frame::none()
                .fill(Color32::from_black_alpha(180))
                .rounding(6.0)
                .inner_margin(8.0)
                .show(ui, |ui| {
                    if let Some(label) = player_label {
                        ui.label(RichText::new(label).color(Color32::LIGHT_BLUE).size(14.0).strong());
                    }

                    ui.label(RichText::new(format!("{:.0} FPS", fps)).color(Color32::GRAY).size(11.0));

                    ui.add_space(4.0);

                    let mode_str = match drone.flight_mode {
                        crate::drone::FlightMode::Fpv => "FPV",
                        crate::drone::FlightMode::Drone => "DRONE",
                    };
                    let armed_str = if armed { "ARMED" } else { "DISARMED" };
                    let armed_color = if armed { Color32::from_rgb(50, 255, 50) } else { Color32::from_rgb(255, 80, 80) };

                    ui.horizontal(|ui| {
                        ui.label(RichText::new(mode_str).color(Color32::WHITE).size(13.0).strong());
                        ui.label(RichText::new(" | ").color(Color32::GRAY).size(13.0));
                        ui.label(RichText::new(armed_str).color(armed_color).size(13.0).strong());
                    });

                    ui.add_space(4.0);

                    // Speed / altitude / vspeed
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(format!("SPD {:.1} m/s", drone.speed)).color(Color32::from_rgb(120, 200, 255)).size(12.0));
                        ui.label(RichText::new(format!("ALT {:.1} m", drone.y)).color(Color32::from_rgb(120, 255, 120)).size(12.0));
                    });
                    ui.label(RichText::new(format!("VS {:.1} m/s", drone.vertical_speed)).color(Color32::from_rgb(200, 200, 120)).size(12.0));

                    // Attitude
                    ui.label(RichText::new(format!(
                        "P {:.0}° R {:.0}° Y {:.0}°",
                        drone.body_pitch, drone.body_roll, drone.yaw
                    )).color(Color32::GRAY).size(11.0));
                });

            // ---- Race panel ----
            if let Some(course) = course {
                if course.visible && !course.gates.is_empty() {
                    ui.add_space(6.0);
                    egui::Frame::none()
                        .fill(Color32::from_black_alpha(200))
                        .rounding(6.0)
                        .inner_margin(8.0)
                        .show(ui, |ui| {
                            ui.label(RichText::new("FPV RACE").color(Color32::from_rgb(152, 164, 178)).size(14.0).strong().italics());

                            ui.add_space(4.0);

                            // Gate counter
                            let passed = course.passed_count();
                            let total = course.gates.len();
                            let next = if passed < total { passed + 1 } else { total };
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("GATE").color(Color32::from_rgb(152, 164, 178)).size(12.0).strong());
                                ui.label(RichText::new(format!("{:>2} / {}", next, total)).color(Color32::WHITE).size(14.0).strong());
                            });

                            // Lap clock
                            let lap_str = gates::format_lap(course.current_lap_ms);
                            let lap_color = if course.lap_start.is_some() {
                                Color32::from_rgb(77, 220, 255)
                            } else {
                                Color32::from_rgb(106, 122, 136)
                            };
                            ui.label(RichText::new(&lap_str).color(lap_color).size(18.0).strong());

                            // Best lap
                            let best_str = match course.best_lap_ms {
                                Some(ms) => gates::format_lap(ms),
                                None => "--:--.---".into(),
                            };
                            let best_color = if course.best_lap_ms.is_some() {
                                Color32::from_rgb(192, 132, 252)
                            } else {
                                Color32::from_rgb(85, 85, 102)
                            };
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("BEST").color(Color32::from_rgb(122, 140, 158)).size(10.0).strong());
                                ui.label(RichText::new(&best_str).color(best_color).size(12.0).strong());
                            });
                        });
                }
            }
        });
}

/// Draw a simple artificial horizon (custom egui paint).
pub fn draw_horizon(
    ui: &mut egui::Ui,
    pitch: f32,
    roll: f32,
    center: Pos2,
    size: f32,
) {
    let painter = ui.painter();
    let r = size * 0.5;

    // Clip circle
    let px_per_deg = size / 60.0;
    let roll_rad = roll * std::f32::consts::PI / 180.0;
    let cos_r = roll_rad.cos();
    let sin_r = roll_rad.sin();
    let offset_y = pitch * px_per_deg;

    // Horizon line
    let dx = cos_r * r;
    let dy = sin_r * r;
    let mid_y = center.y + offset_y * cos_r;
    let mid_x = center.x - offset_y * sin_r;

    painter.line_segment(
        [
            Pos2::new(mid_x - dx, mid_y - dy),
            Pos2::new(mid_x + dx, mid_y + dy),
        ],
        Stroke::new(2.0, Color32::from_rgb(0, 200, 0)),
    );

    // Center crosshair
    painter.line_segment(
        [Pos2::new(center.x - 8.0, center.y), Pos2::new(center.x + 8.0, center.y)],
        Stroke::new(1.5, Color32::WHITE),
    );
    painter.line_segment(
        [Pos2::new(center.x, center.y - 8.0), Pos2::new(center.x, center.y + 8.0)],
        Stroke::new(1.5, Color32::WHITE),
    );
}
