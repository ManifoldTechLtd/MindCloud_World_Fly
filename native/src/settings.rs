/// Settings panel — egui UI for drone physics + controller config.

use egui::{Color32, RichText};

use crate::drone::{Drone, FlightMode};
use crate::input::Controller;

/// Draw the settings panel. Returns true if it should be visible.
pub fn draw_settings(
    ctx: &egui::Context,
    open: &mut bool,
    drone: &mut Drone,
    controller: &mut Controller,
    armed: &mut bool,
) {
    if !*open { return; }

    egui::Window::new("⚙ Settings")
        .open(open)
        .default_width(450.0)
        .min_width(400.0)
        .order(egui::Order::Tooltip)
        .show(ctx, |ui| {
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
                    ui.horizontal(|ui| {
                        ui.label("Mass (g):");
                        ui.add(egui::DragValue::new(&mut drone.mass).range(50.0..=5000.0).speed(10.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Max Thrust (gf):");
                        ui.add(egui::DragValue::new(&mut drone.max_thrust).range(100.0..=10000.0).speed(50.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Drag Cd:");
                        ui.add(egui::DragValue::new(&mut drone.drag_cd).range(0.0..=5.0).speed(0.05));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Drag Area (m²):");
                        ui.add(egui::DragValue::new(&mut drone.drag_area).range(0.001..=0.5).speed(0.001));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Drone Size (m):");
                        ui.add(egui::DragValue::new(&mut drone.drone_size).range(0.05..=2.0).speed(0.01));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Collision Radius:");
                        ui.add(egui::DragValue::new(&mut drone.collision_radius).range(0.05..=2.0).speed(0.01));
                    });
                });

            egui::CollapsingHeader::new("FPV Rates")
                .default_open(false)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Max Pitch Rate (°/s):");
                        ui.add(egui::DragValue::new(&mut drone.max_pitch_rate).range(50.0..=1200.0).speed(10.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Max Roll Rate (°/s):");
                        ui.add(egui::DragValue::new(&mut drone.max_roll_rate).range(50.0..=1200.0).speed(10.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Max Yaw Rate (°/s):");
                        ui.add(egui::DragValue::new(&mut drone.max_yaw_rate).range(30.0..=600.0).speed(5.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Camera Mount Angle (°):");
                        ui.add(egui::DragValue::new(&mut drone.camera_mount_angle).range(0.0..=60.0).speed(1.0));
                    });
                });

            egui::CollapsingHeader::new("Drone Mode PID")
                .default_open(false)
                .show(ui, |ui| {
                    ui.horizontal(|ui| { ui.label("Pos Kp:"); ui.add(egui::DragValue::new(&mut drone.drone_pos_kp).range(0.0..=20.0).speed(0.1)); });
                    ui.horizontal(|ui| { ui.label("Pos Ki:"); ui.add(egui::DragValue::new(&mut drone.drone_pos_ki).range(0.0..=10.0).speed(0.05)); });
                    ui.horizontal(|ui| { ui.label("Pos Kd:"); ui.add(egui::DragValue::new(&mut drone.drone_pos_kd).range(0.0..=5.0).speed(0.01)); });
                    ui.separator();
                    ui.horizontal(|ui| { ui.label("Vel Kp:"); ui.add(egui::DragValue::new(&mut drone.drone_vel_kp).range(0.0..=20.0).speed(0.1)); });
                    ui.horizontal(|ui| { ui.label("Vel Ki:"); ui.add(egui::DragValue::new(&mut drone.drone_vel_ki).range(0.0..=10.0).speed(0.05)); });
                    ui.horizontal(|ui| { ui.label("Vel Kd:"); ui.add(egui::DragValue::new(&mut drone.drone_vel_kd).range(0.0..=5.0).speed(0.01)); });
                    ui.separator();
                    ui.horizontal(|ui| { ui.label("Alt Kp:"); ui.add(egui::DragValue::new(&mut drone.drone_alt_kp).range(0.0..=20.0).speed(0.1)); });
                    ui.horizontal(|ui| { ui.label("Alt Ki:"); ui.add(egui::DragValue::new(&mut drone.drone_alt_ki).range(0.0..=10.0).speed(0.05)); });
                    ui.horizontal(|ui| { ui.label("Alt Kd:"); ui.add(egui::DragValue::new(&mut drone.drone_alt_kd).range(0.0..=5.0).speed(0.01)); });
                });

            egui::CollapsingHeader::new("Channel Mapping")
                .default_open(false)
                .show(ui, |ui| {
                    ui.label(RichText::new("Click channel button, then move the stick to assign").size(10.0).color(Color32::GRAY));
                    ui.add_space(4.0);
                    // Axes
                    for i in 0..4 {
                        let name = crate::input::AXIS_NAMES[i];
                        let ch = controller.axis_map[i].channel;
                        let ch_label = if ch >= 0 { format!("CH{}", ch + 1) } else { "---".into() };
                        let is_listening = controller.listening == Some(i);
                        ui.horizontal(|ui| {
                            ui.label(format!("{:>9}:", name));
                            let btn_text = if is_listening { "[ ... ]".into() } else { ch_label };
                            let btn_color = if is_listening { Color32::from_rgb(255, 180, 0) } else { Color32::from_rgb(100, 180, 255) };
                            if ui.button(RichText::new(&btn_text).color(btn_color).size(12.0)).clicked() {
                                controller.start_listen(i);
                            }
                            ui.checkbox(&mut controller.axis_map[i].inverted, "Inv");
                            ui.label("Expo:");
                            ui.add(egui::DragValue::new(&mut controller.axis_map[i].expo).range(0.0..=1.0).speed(0.05).max_decimals(2));
                        });
                    }
                    ui.add_space(6.0);
                    // Switches
                    for i in 0..2 {
                        let name = crate::input::SWITCH_NAMES[i];
                        let ch = controller.switch_channels[i];
                        let ch_label = if ch >= 0 { format!("CH{}", ch + 1) } else { "---".into() };
                        let target_id = 10 + i;
                        let is_listening = controller.listening == Some(target_id);
                        ui.horizontal(|ui| {
                            ui.label(format!("{:>9}:", name));
                            let btn_text = if is_listening { "[ ... ]".into() } else { ch_label };
                            let btn_color = if is_listening { Color32::from_rgb(255, 180, 0) } else { Color32::from_rgb(100, 200, 100) };
                            if ui.button(RichText::new(&btn_text).color(btn_color).size(12.0)).clicked() {
                                controller.start_listen(target_id);
                            }
                            ui.checkbox(&mut controller.switch_inverted[i], "Inv");
                        });
                    }
                });

            if controller.hid_connected {
                egui::CollapsingHeader::new("HID Channels (live)")
                    .default_open(true)
                    .show(ui, |ui| {
                        for i in 0..8 {
                            let v = controller.hid_axes[i];
                            ui.horizontal(|ui| {
                                ui.label(format!("CH{:02}", i + 1));
                                let bar_w = 200.0 * ((v + 1.0) / 2.0).clamp(0.0, 1.0);
                                let (rect, _) = ui.allocate_exact_size(egui::Vec2::new(200.0, 14.0), egui::Sense::hover());
                                ui.painter().rect_filled(rect, 0.0, Color32::from_gray(40));
                                // Center line
                                let cx = rect.min.x + 100.0;
                                ui.painter().line_segment([egui::Pos2::new(cx, rect.min.y), egui::Pos2::new(cx, rect.max.y)], egui::Stroke::new(1.0, Color32::from_gray(80)));
                                let filled = egui::Rect::from_min_size(rect.min, egui::Vec2::new(bar_w, 14.0));
                                ui.painter().rect_filled(filled, 0.0, Color32::from_rgb(0, 180, 100));
                                ui.label(format!("{:+.2}", v));
                            });
                        }
                        ui.add_space(5.0);
                        let is_calibrated = controller.calibration[0].is_calibrated() && !controller.calibrating;

                        if controller.calibrating {
                            ui.label(RichText::new("⏺ Recording... move all sticks to extremes").color(Color32::from_rgb(255, 180, 0)));
                            if ui.button(RichText::new("    Finish Calibration    ").size(13.0)).clicked() {
                                controller.calibrating = false;
                                let _ = crate::persistence::save_calibration(&controller.calibration);
                                log::info!("Calibration saved!");
                            }
                        } else if is_calibrated {
                            ui.label(RichText::new("✓ Calibrated").size(11.0).color(Color32::from_rgb(80, 255, 80)));
                            if ui.button(RichText::new("    Re-calibrate    ").size(12.0)).clicked() {
                                for cal in controller.calibration.iter_mut() {
                                    *cal = crate::input::ChannelCalibration::default();
                                }
                                controller.calibrating = true;
                                log::info!("Calibration reset — move sticks to extremes");
                            }
                        } else {
                            if ui.button(RichText::new("    Start Calibration    ").size(13.0)).clicked() {
                                for cal in controller.calibration.iter_mut() {
                                    *cal = crate::input::ChannelCalibration::default();
                                }
                                controller.calibrating = true;
                                log::info!("Calibration started — move sticks to extremes");
                            }
                        }
                    });
            }

            ui.separator();
            ui.horizontal(|ui| {
                let arm_text = if *armed { "DISARM" } else { "ARM" };
                let arm_color = if *armed { Color32::from_rgb(255, 80, 80) } else { Color32::from_rgb(80, 255, 80) };
                if ui.button(RichText::new(arm_text).color(arm_color).strong()).clicked() {
                    *armed = !*armed;
                }
            });
        });
}
