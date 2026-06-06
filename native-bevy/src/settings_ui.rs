//! Settings panel — egui UI for drone physics / rates / PID + HID controller config. Ported from
//! native `src/settings.rs` (Flight Mode, Physics, FPV Rates, Drone Mode PID) plus the HID section
//! (device connect, axis/switch mapping, calibration, per-mode expo/rate). Pure egui (changes
//! persisted via `persistence`); the Bevy glue (F1 toggle, device connect/disconnect) lives in `flight`.

use bevy_egui::egui::{self, Color32, RichText};

use crate::drone::{Drone, FlightMode};
use crate::input::{
    Controller, HidDeviceInfo, AXIS_NAMES, LISTEN_ARM, LISTEN_MODE, MAX_CHANNELS, SWITCH_NAMES,
};

/// Device action requested by the HID section, applied by `flight::settings_ui_system`.
pub enum HidUiAction {
    /// Re-enumerate HID devices.
    Scan,
    /// Connect to `devices[idx]`.
    Connect(usize),
    /// Disconnect the current device.
    Disconnect,
}

/// Format a calibration bound for display: the raw value, or `—` if not yet learned.
fn fmt_cal(v: Option<u16>) -> String {
    match v {
        Some(n) => n.to_string(),
        None => "—".to_string(),
    }
}

/// Draw the settings window. `open` is bound to the window's `[x]` close button. The upper sections
/// (Flight Mode / Physics / FPV Rates / Drone Mode PID) edit `drone` — a single shared config for
/// all players (the caller mirrors it onto the others). The HID section is per-player, picked by the
/// P1/P2 switch inside it (`sel` / `num_players`). Drone-param edits persist per-section.
pub fn draw_settings(
    ctx: &egui::Context,
    open: &mut bool,
    sel: &mut usize,
    num_players: usize,
    drone: &mut Drone,
    ctrl: &mut Controller,
    devices: &[HidDeviceInfo],
    connected: Option<&str>,
) -> Option<HidUiAction> {
    if !*open {
        return None;
    }
    let mut action: Option<HidUiAction> = None;

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

                // HID controller (per-player): device, mapping, switches, calibration, expo/rate.
                ui.separator();
                action = draw_hid_section(ui, sel, num_players, ctrl, devices, connected);
            }); // end ScrollArea
        });

    action
}

/// HID controller section of the settings panel. Mutates + persists `ctrl` (mapping / calibration /
/// expo / rate) and returns a device action (scan / connect / disconnect) for the caller to apply.
fn draw_hid_section(
    ui: &mut egui::Ui,
    sel: &mut usize,
    num_players: usize,
    ctrl: &mut Controller,
    devices: &[HidDeviceInfo],
    connected: Option<&str>,
) -> Option<HidUiAction> {
    let mut action: Option<HidUiAction> = None;

    egui::CollapsingHeader::new("HID Device")
        .default_open(true)
        .show(ui, |ui| {
            // Split-screen: the controller config below is per-player — pick whose device/mapping/
            // calibration this edits. The upper drone params stay shared across both players.
            if num_players > 1 {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("Player:").strong());
                    for p in 0..num_players {
                        ui.selectable_value(sel, p, format!("P{}", p + 1));
                    }
                });
                ui.separator();
            }
            ui.horizontal(|ui| {
                ui.label("Status:");
                match connected {
                    Some(name) => {
                        ui.colored_label(Color32::from_rgb(80, 220, 120), format!("● {name}"))
                    }
                    None => ui.colored_label(Color32::GRAY, "○ not connected"),
                };
            });
            ui.horizontal(|ui| {
                if ui.button("Scan").clicked() {
                    action = Some(HidUiAction::Scan);
                }
                if connected.is_some() && ui.button("Disconnect").clicked() {
                    action = Some(HidUiAction::Disconnect);
                }
            });
            for (i, d) in devices.iter().enumerate() {
                ui.horizontal(|ui| {
                    if ui.button("Connect").clicked() {
                        action = Some(HidUiAction::Connect(i));
                    }
                    ui.label(format!(
                        "{} ({:04x}:{:04x})",
                        d.product_name, d.vendor_id, d.product_id
                    ));
                });
            }
            if devices.is_empty() {
                ui.label(
                    RichText::new("Press Scan to list devices.")
                        .size(10.0)
                        .color(Color32::GRAY),
                );
            }
        });

    // The mapping/calibration/expo sections only make sense with a live device.
    if connected.is_none() {
        return action;
    }

    egui::CollapsingHeader::new("Axis Mapping")
        .default_open(true)
        .show(ui, |ui| {
            let mut changed = false;
            for i in 0..4 {
                ui.horizontal(|ui| {
                    ui.label(format!("{:<9}", AXIS_NAMES[i]));
                    ui.label(format!("ch {:>2}", ctrl.axis_map[i].channel));
                    let listening = ctrl.listening == Some(i);
                    if ui.button(if listening { "…move…" } else { "Listen" }).clicked() {
                        ctrl.start_listen(i);
                    }
                    changed |= ui.checkbox(&mut ctrl.axis_map[i].inverted, "Inv").changed();
                });
            }
            if changed {
                let _ = crate::persistence::save_controller_mapping(ctrl);
            }
        });

    egui::CollapsingHeader::new("Switches (Arm / Mode)")
        .default_open(true)
        .show(ui, |ui| {
            let mut changed = false;
            for s in 0..2 {
                ui.horizontal(|ui| {
                    ui.label(format!("{:<5}", SWITCH_NAMES[s]));
                    ui.label(format!("ch {:>2}", ctrl.switch_channels[s]));
                    let target = if s == 0 { LISTEN_ARM } else { LISTEN_MODE };
                    let listening = ctrl.listening == Some(target);
                    if ui.button(if listening { "…flip…" } else { "Listen" }).clicked() {
                        ctrl.start_listen(target);
                    }
                    changed |= ui.checkbox(&mut ctrl.switch_inverted[s], "Inv").changed();
                    changed |= ui
                        .selectable_value(&mut ctrl.switch_level_mode[s], false, "Toggle")
                        .changed();
                    changed |= ui
                        .selectable_value(&mut ctrl.switch_level_mode[s], true, "Level")
                        .changed();
                });
            }
            ui.horizontal(|ui| {
                ui.label("Threshold:");
                changed |= ui
                    .add(egui::DragValue::new(&mut ctrl.switch_threshold).range(0.0..=1.0).speed(0.02))
                    .changed();
            });
            if changed {
                let _ = crate::persistence::save_controller_mapping(ctrl);
            }
        });

    egui::CollapsingHeader::new("Calibration")
        .default_open(false)
        .show(ui, |ui| {
            let label = if ctrl.calibrating { "Stop & Save Calibration" } else { "Start Calibration" };
            if ui.button(label).clicked() {
                if ctrl.calibrating {
                    // Stopping → persist the learned min/max for this player.
                    ctrl.calibrating = false;
                    let _ = crate::persistence::save_calibration(ctrl.player, &ctrl.calibration);
                } else {
                    // Starting → wipe old min/max so we re-learn this device from scratch.
                    ctrl.reset_calibration();
                    ctrl.calibrating = true;
                }
            }
            if ctrl.calibrating {
                ui.colored_label(Color32::YELLOW, "Move every stick + switch to its extremes, then Stop.");
            }
            // Per-channel raw HID input (not the 4 logical axes): list every live channel, at least
            // 8, so any stick/switch can be identified and its learned [min, max] verified.
            let n = ctrl.hid_channel_count.max(8).min(MAX_CHANNELS);
            for ch in 0..n {
                let (cmin, cmax, calibrated) = {
                    let c = &ctrl.calibration[ch];
                    (c.min, c.max, c.is_calibrated())
                };
                let raw = ctrl.hid_raw[ch];
                let v = ctrl.hid_axes[ch];
                let mark = if calibrated { "✓" } else { "—" };
                ui.horizontal(|ui| {
                    ui.label(format!("{mark} ch{ch:>2}"));
                    ui.add(
                        egui::ProgressBar::new((v + 1.0) / 2.0)
                            .desired_width(150.0)
                            .text(format!("{raw}")),
                    );
                    ui.label(format!("[{}, {}]", fmt_cal(cmin), fmt_cal(cmax)));
                });
            }
        });

    egui::CollapsingHeader::new("Expo / Rate (per mode)")
        .default_open(false)
        .show(ui, |ui| {
            let mut changed = false;
            for (mi, mname) in ["FPV", "Drone"].into_iter().enumerate() {
                ui.label(RichText::new(mname).strong());
                for i in 0..4 {
                    ui.horizontal(|ui| {
                        ui.label(format!("{:<9}", AXIS_NAMES[i]));
                        ui.label("expo");
                        changed |= ui
                            .add(egui::DragValue::new(&mut ctrl.mode_expo[mi][i]).range(0.0..=1.0).speed(0.02))
                            .changed();
                        ui.label("rate");
                        changed |= ui
                            .add(egui::DragValue::new(&mut ctrl.mode_rate[mi][i]).range(0.0..=2.0).speed(0.02))
                            .changed();
                    });
                }
            }
            if changed {
                let _ = crate::persistence::save_controller_mapping(ctrl);
            }
        });

    action
}
