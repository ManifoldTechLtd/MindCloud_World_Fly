/// Settings panel — egui UI for drone physics + controller config.

use egui::{Color32, RichText};
use std::sync::Mutex;

use crate::drone::{Drone, FlightMode};
use crate::input::Controller;

// Static state for HID device picker
static DETECTED_DEVICES: Mutex<Option<Vec<crate::input::HidDeviceInfo>>> = Mutex::new(None);
static SELECTED_HID_PATH: Mutex<Option<std::ffi::CString>> = Mutex::new(None);

/// Check if the user selected a HID device from the settings panel.
/// Returns the path if one was selected (caller should open it).
pub fn take_selected_hid_path() -> Option<std::ffi::CString> {
    SELECTED_HID_PATH.lock().unwrap().take()
}

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
        .resizable(false)
        .default_size([480.0, 520.0])
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .order(egui::Order::Tooltip)
        .show(ctx, |ui| {
            ui.set_width(460.0);
            ui.set_height(500.0);
            egui::ScrollArea::vertical().show(ui, |ui| {
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
                    let m = controller.current_mode.min(1);
                    let mode_label = if m == 0 { "FPV" } else { "Drone" };
                    ui.label(RichText::new(format!("Mode: {} (rate/expo below apply to this mode)", mode_label)).size(10.0).color(Color32::from_rgb(100, 200, 150)));
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
                            if ui.checkbox(&mut controller.axis_map[i].inverted, "Inv").changed() {
                                let _ = crate::persistence::save_controller_mapping(controller);
                            }
                            ui.label("Rate:");
                            if ui.add(egui::DragValue::new(&mut controller.mode_rate[m][i]).range(0.1..=3.0).speed(0.05).max_decimals(2)).changed() {
                                let _ = crate::persistence::save_controller_mapping(controller);
                            }
                            ui.label("Expo:");
                            if ui.add(egui::DragValue::new(&mut controller.mode_expo[m][i]).range(0.0..=1.0).speed(0.05).max_decimals(2)).changed() {
                                let _ = crate::persistence::save_controller_mapping(controller);
                            }
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
                            if ui.checkbox(&mut controller.switch_inverted[i], "Inv").changed() {
                                let _ = crate::persistence::save_controller_mapping(controller);
                            }
                            let mode_label = if controller.switch_level_mode[i] { "Level" } else { "Toggle" };
                            if ui.button(RichText::new(mode_label).size(10.0)).clicked() {
                                controller.switch_level_mode[i] = !controller.switch_level_mode[i];
                                let _ = crate::persistence::save_controller_mapping(controller);
                            }
                        });
                    }
                });

            egui::CollapsingHeader::new("HID Input")
                .default_open(true)
                .show(ui, |ui| {
                    if controller.hid_connected {
                        ui.label(RichText::new("✓ Device connected").size(11.0).color(Color32::from_rgb(80, 255, 80)));
                        ui.add_space(4.0);
                        for i in 0..8 {
                            let v = controller.hid_axes[i];
                            ui.horizontal(|ui| {
                                ui.label(format!("CH{:02}", i + 1));
                                let bar_w = 200.0 * ((v + 1.0) / 2.0).clamp(0.0, 1.0);
                                let (rect, _) = ui.allocate_exact_size(egui::Vec2::new(200.0, 14.0), egui::Sense::hover());
                                ui.painter().rect_filled(rect, 0.0, Color32::from_gray(40));
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
                            }
                        } else if is_calibrated {
                            ui.label(RichText::new("✓ Calibrated").size(11.0).color(Color32::from_rgb(80, 255, 80)));
                            if ui.button(RichText::new("    Re-calibrate    ").size(12.0)).clicked() {
                                for cal in controller.calibration.iter_mut() { *cal = crate::input::ChannelCalibration::default(); }
                                controller.calibrating = true;
                            }
                        } else {
                            if ui.button(RichText::new("    Start Calibration    ").size(13.0)).clicked() {
                                for cal in controller.calibration.iter_mut() { *cal = crate::input::ChannelCalibration::default(); }
                                controller.calibrating = true;
                            }
                        }
                        ui.add_space(5.0);
                        if ui.button("Disconnect / Change Device").clicked() {
                            controller.hid_connected = false;
                        }
                    } else {
                        ui.label(RichText::new("No HID device connected").size(12.0).color(Color32::from_rgb(200, 150, 50)));
                        ui.add_space(5.0);
                        if ui.button(RichText::new("    Detect HID Devices    ").size(13.0)).clicked() {
                            // List devices and store for selection
                            let devices = crate::input::list_hid_devices();
                            DETECTED_DEVICES.lock().unwrap().replace(devices);
                        }
                        // Show detected device list if available
                        let mut selected_path: Option<std::ffi::CString> = None;
                        if let Some(ref devices) = *DETECTED_DEVICES.lock().unwrap() {
                            if devices.is_empty() {
                                ui.label(RichText::new("No HID devices found").size(11.0).color(Color32::GRAY));
                            } else {
                                ui.label(RichText::new("Select a device:").size(11.0));
                                for dev in devices {
                                    let label = format!("{} ({:04X}:{:04X})", dev.product_name, dev.vendor_id, dev.product_id);
                                    if ui.button(&label).clicked() {
                                        selected_path = Some(dev.path.clone());
                                    }
                                }
                            }
                        }
                        if let Some(path) = selected_path {
                            SELECTED_HID_PATH.lock().unwrap().replace(path);
                            DETECTED_DEVICES.lock().unwrap().take();
                        }
                    }
                });

            }); // end ScrollArea
        });
}
