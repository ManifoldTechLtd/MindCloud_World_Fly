//! Settings panel — egui UI for drone physics / rates / PID params. Ported from native
//! `src/settings.rs`, **drone-only** sections (Flight Mode, Physics, FPV Rates, Drone Mode PID).
//! The Channel-Mapping + HID-Input sections need the HID `Controller`, which isn't ported yet →
//! deferred to Phase 5. Pure egui (changes persisted via `persistence::save_drone_settings`); the
//! Bevy glue (F1 toggle) lives in `flight`.

use bevy_egui::egui::{self, Color32, RichText};

use crate::drone::{Drone, FlightMode};

/// Draw the settings window. `open` is bound to the window's `[x]` close button. Drone-param edits
/// are persisted per-section (matching native).
pub fn draw_settings(ctx: &egui::Context, open: &mut bool, drone: &mut Drone) {
    if !*open {
        return;
    }

    egui::Window::new("⚙ Settings")
        .open(open)
        .resizable(false)
        .default_size([480.0, 520.0])
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .order(egui::Order::Tooltip)
        .show(ctx, |ui| {
            ui.set_width(460.0);
            ui.set_height(500.0);
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                egui::CollapsingHeader::new("Flight Mode")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.selectable_value(&mut drone.flight_mode, FlightMode::Fpv, "FPV (Acro)");
                            ui.selectable_value(&mut drone.flight_mode, FlightMode::Drone, "Drone (Stabilized)");
                        });
                    });

                egui::CollapsingHeader::new("Physics")
                    .default_open(true)
                    .show(ui, |ui| {
                        let mut changed = false;
                        ui.horizontal(|ui| { ui.label("Mass (g):"); changed |= ui.add(egui::DragValue::new(&mut drone.mass).range(50.0..=5000.0).speed(10.0)).changed(); });
                        ui.horizontal(|ui| { ui.label("Max Thrust (gf):"); changed |= ui.add(egui::DragValue::new(&mut drone.max_thrust).range(100.0..=10000.0).speed(50.0)).changed(); });
                        ui.horizontal(|ui| { ui.label("Drag Cd:"); changed |= ui.add(egui::DragValue::new(&mut drone.drag_cd).range(0.0..=5.0).speed(0.05)).changed(); });
                        ui.horizontal(|ui| { ui.label("Drag Area (m²):"); changed |= ui.add(egui::DragValue::new(&mut drone.drag_area).range(0.001..=0.5).speed(0.001)).changed(); });
                        ui.horizontal(|ui| { ui.label("Drone Size (m):"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_size).range(0.05..=2.0).speed(0.01)).changed(); });
                        ui.horizontal(|ui| { ui.label("Collision Radius:"); changed |= ui.add(egui::DragValue::new(&mut drone.collision_radius).range(0.05..=2.0).speed(0.01)).changed(); });
                        if changed { let _ = crate::persistence::save_drone_settings(drone); }
                    });

                egui::CollapsingHeader::new("FPV Rates")
                    .default_open(false)
                    .show(ui, |ui| {
                        let mut changed = false;
                        ui.horizontal(|ui| { ui.label("Max Pitch Rate (°/s):"); changed |= ui.add(egui::DragValue::new(&mut drone.max_pitch_rate).range(50.0..=1200.0).speed(10.0)).changed(); });
                        ui.horizontal(|ui| { ui.label("Max Roll Rate (°/s):"); changed |= ui.add(egui::DragValue::new(&mut drone.max_roll_rate).range(50.0..=1200.0).speed(10.0)).changed(); });
                        ui.horizontal(|ui| { ui.label("Max Yaw Rate (°/s):"); changed |= ui.add(egui::DragValue::new(&mut drone.max_yaw_rate).range(30.0..=600.0).speed(5.0)).changed(); });
                        ui.horizontal(|ui| { ui.label("Camera Mount Angle (°):"); changed |= ui.add(egui::DragValue::new(&mut drone.camera_mount_angle).range(0.0..=60.0).speed(1.0)).changed(); });
                        if changed { let _ = crate::persistence::save_drone_settings(drone); }
                    });

                egui::CollapsingHeader::new("Drone Mode PID")
                    .default_open(false)
                    .show(ui, |ui| {
                        let mut changed = false;
                        ui.horizontal(|ui| { ui.label("Pos Kp:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_pos_kp).range(0.0..=20.0).speed(0.1)).changed(); });
                        ui.horizontal(|ui| { ui.label("Pos Ki:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_pos_ki).range(0.0..=10.0).speed(0.05)).changed(); });
                        ui.horizontal(|ui| { ui.label("Pos Kd:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_pos_kd).range(0.0..=5.0).speed(0.01)).changed(); });
                        ui.separator();
                        ui.horizontal(|ui| { ui.label("Vel Kp:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_vel_kp).range(0.0..=20.0).speed(0.1)).changed(); });
                        ui.horizontal(|ui| { ui.label("Vel Ki:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_vel_ki).range(0.0..=10.0).speed(0.05)).changed(); });
                        ui.horizontal(|ui| { ui.label("Vel Kd:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_vel_kd).range(0.0..=5.0).speed(0.01)).changed(); });
                        ui.separator();
                        ui.horizontal(|ui| { ui.label("Alt Kp:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_alt_kp).range(0.0..=20.0).speed(0.1)).changed(); });
                        ui.horizontal(|ui| { ui.label("Alt Ki:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_alt_ki).range(0.0..=10.0).speed(0.05)).changed(); });
                        ui.horizontal(|ui| { ui.label("Alt Kd:"); changed |= ui.add(egui::DragValue::new(&mut drone.drone_alt_kd).range(0.0..=5.0).speed(0.01)).changed(); });
                        if changed { let _ = crate::persistence::save_drone_settings(drone); }
                    });

                // Channel-Mapping + HID-Input sections need the HID `Controller` (Phase 5).
                ui.separator();
                ui.label(
                    RichText::new("Channel Mapping & HID Input — arrives with controller support (Phase 5)")
                        .size(10.0)
                        .color(Color32::GRAY),
                );
            }); // end ScrollArea
        });
}
