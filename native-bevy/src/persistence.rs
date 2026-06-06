//! Config persistence — JSON files under `~/.config/mindcloud-fly/`.
//!
//! Ported from `native/src/persistence.rs`. **Subset for this slice**: only the per-scene config
//! (`world_up` / `spawn` / `heading`). The other persisted blobs (drone settings, HID calibration,
//! controller mapping, gate-path records) will be ported alongside their feature slices (drone /
//! input / gates) so each lands with the type it serializes.

use crate::app_state::SceneConfig;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Config directory (stable regardless of CWD): `~/.config/mindcloud-fly`, else `.`.
/// Same location native uses — this is shared user data, not a code dependency on `native/`.
fn config_dir() -> PathBuf {
    if let Some(home) = std::env::var_os("HOME") {
        PathBuf::from(home).join(".config").join("mindcloud-fly")
    } else {
        PathBuf::from(".")
    }
}

fn ensure_config_dir() {
    let _ = std::fs::create_dir_all(config_dir());
}

/// Per-player config filename: player 0 keeps the original (native-shared) name; player 1+ get a
/// numeric suffix (`calibration.json` → `calibration2.json`), so P1 + P2 store independent HID
/// calibration / mapping / device-path without clobbering each other.
fn player_file(stem: &str, ext: &str, player: usize) -> String {
    if player == 0 {
        format!("{stem}.{ext}")
    } else {
        format!("{stem}{}.{ext}", player + 1)
    }
}

/// Storage key for a scene = `"{filename}_{filesize}"` (so identically-named files differ).
/// Ported from `native/src/persistence.rs::scene_key`.
pub fn scene_key(file_path: &Path) -> String {
    let name = file_path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let size = std::fs::metadata(file_path).map(|m| m.len()).unwrap_or(0);
    format!("{}_{}", name, size)
}

/// Load per-scene config from `scenes.json` (returns `Default` if absent / unparseable).
pub fn load_scene_config(scene_key: &str) -> SceneConfig {
    let path = config_dir().join("scenes.json");
    std::fs::read_to_string(path)
        .ok()
        .and_then(|s| serde_json::from_str::<HashMap<String, SceneConfig>>(&s).ok())
        .and_then(|map| map.get(scene_key).cloned())
        .unwrap_or_default()
}

/// Save per-scene config into `scenes.json` (merging with existing entries).
/// Ported from `native/src/persistence.rs::save_scene_config`.
pub fn save_scene_config(scene_key: &str, config: &SceneConfig) -> std::io::Result<()> {
    ensure_config_dir();
    let path = config_dir().join("scenes.json");
    let mut map: HashMap<String, SceneConfig> = std::fs::read_to_string(&path)
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default();
    map.insert(scene_key.to_string(), config.clone());
    let json = serde_json::to_string_pretty(&map)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    std::fs::write(path, json)
}

/// Per-scene gate-path record (stored as JSON). Ported from `native/src/persistence.rs`.
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct SceneRecord {
    pub points: Vec<[f32; 3]>,
    pub gate_size: f32,
    pub best_lap_ms: Option<f64>,
}

/// Gate-path directory: `~/.config/mindcloud-fly/gate-paths`. (native used a CWD-relative
/// `asset/gate-paths`; native-bevy is standalone + CWD-independent, so it lives under the config dir.)
fn gate_paths_dir() -> PathBuf {
    config_dir().join("gate-paths")
}

/// Save a scene's gate-path record to `gate-paths/{scene_key}.json`.
pub fn save_scene_record(scene_key: &str, record: &SceneRecord) -> std::io::Result<()> {
    let dir = gate_paths_dir();
    std::fs::create_dir_all(&dir)?;
    let json = serde_json::to_string_pretty(record)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    std::fs::write(dir.join(format!("{}.json", scene_key)), json)
}

/// Load a scene's gate-path record (returns `None` if absent / unparseable).
pub fn load_scene_record(scene_key: &str) -> Option<SceneRecord> {
    let path = gate_paths_dir().join(format!("{}.json", scene_key));
    let data = std::fs::read_to_string(path).ok()?;
    serde_json::from_str(&data).ok()
}

/// Drone physics settings (persisted to `drone.json`). Ported from `native/src/persistence.rs`.
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct DroneSettings {
    pub mass: f32,
    pub max_thrust: f32,
    pub drag_cd: f32,
    pub drag_area: f32,
    pub collision_radius: f32,
    pub drone_size: f32,
    pub camera_mount_angle: f32,
    pub max_pitch_rate: f32,
    pub max_roll_rate: f32,
    pub max_yaw_rate: f32,
    // Drone mode PID
    pub drone_pos_kp: f32,
    pub drone_pos_ki: f32,
    pub drone_pos_kd: f32,
    pub drone_vel_kp: f32,
    pub drone_vel_ki: f32,
    pub drone_vel_kd: f32,
    pub drone_alt_kp: f32,
    pub drone_alt_ki: f32,
    pub drone_alt_kd: f32,
}

/// Save the drone's tunable physics params to `drone.json`. Ported from native.
pub fn save_drone_settings(drone: &crate::drone::Drone) -> std::io::Result<()> {
    ensure_config_dir();
    let s = DroneSettings {
        mass: drone.mass, max_thrust: drone.max_thrust,
        drag_cd: drone.drag_cd, drag_area: drone.drag_area,
        collision_radius: drone.collision_radius, drone_size: drone.drone_size,
        camera_mount_angle: drone.camera_mount_angle,
        max_pitch_rate: drone.max_pitch_rate, max_roll_rate: drone.max_roll_rate,
        max_yaw_rate: drone.max_yaw_rate,
        drone_pos_kp: drone.drone_pos_kp, drone_pos_ki: drone.drone_pos_ki, drone_pos_kd: drone.drone_pos_kd,
        drone_vel_kp: drone.drone_vel_kp, drone_vel_ki: drone.drone_vel_ki, drone_vel_kd: drone.drone_vel_kd,
        drone_alt_kp: drone.drone_alt_kp, drone_alt_ki: drone.drone_alt_ki, drone_alt_kd: drone.drone_alt_kd,
    };
    let json = serde_json::to_string_pretty(&s)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    std::fs::write(config_dir().join("drone.json"), json)
}

/// Load drone physics params from `drone.json` into `drone` (no-op if absent). Ported from native.
pub fn load_drone_settings(drone: &mut crate::drone::Drone) {
    if let Ok(json) = std::fs::read_to_string(config_dir().join("drone.json")) {
        if let Ok(s) = serde_json::from_str::<DroneSettings>(&json) {
            drone.mass = s.mass; drone.max_thrust = s.max_thrust;
            drone.drag_cd = s.drag_cd; drone.drag_area = s.drag_area;
            drone.collision_radius = s.collision_radius; drone.drone_size = s.drone_size;
            drone.camera_mount_angle = s.camera_mount_angle;
            drone.max_pitch_rate = s.max_pitch_rate; drone.max_roll_rate = s.max_roll_rate;
            drone.max_yaw_rate = s.max_yaw_rate;
            drone.drone_pos_kp = s.drone_pos_kp; drone.drone_pos_ki = s.drone_pos_ki; drone.drone_pos_kd = s.drone_pos_kd;
            drone.drone_vel_kp = s.drone_vel_kp; drone.drone_vel_ki = s.drone_vel_ki; drone.drone_vel_kd = s.drone_vel_kd;
            drone.drone_alt_kp = s.drone_alt_kp; drone.drone_alt_ki = s.drone_alt_ki; drone.drone_alt_kd = s.drone_alt_kd;
        }
    }
}

// ---- HID controller persistence (calibration.json / mapping.json / hid_device.txt) ----
// Same files + format as native/src/persistence.rs, in the shared config dir, so a calibration or
// mapping created in `native/` is reused by native-bevy (and vice-versa).

fn json_io_err(e: serde_json::Error) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::Other, e)
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, Default)]
struct CalibrationData {
    /// `[min, max]` per channel.
    channels: Vec<[Option<u16>; 2]>,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
struct ControllerMapping {
    axis_channels: [i32; 4],   // roll, pitch, throttle, yaw
    axis_inverted: [bool; 4],
    axis_expo_fpv: [f32; 4],
    axis_expo_drone: [f32; 4],
    axis_rate_fpv: [f32; 4],
    axis_rate_drone: [f32; 4],
    switch_channels: [i32; 2], // arm, mode
    switch_inverted: [bool; 2],
    switch_level_mode: [bool; 2],
}

/// Save HID per-channel calibration to `calibration.json`.
pub fn save_calibration(player: usize, calibration: &[crate::input::ChannelCalibration]) -> std::io::Result<()> {
    ensure_config_dir();
    let data = CalibrationData {
        channels: calibration.iter().map(|c| [c.min, c.max]).collect(),
    };
    let json = serde_json::to_string_pretty(&data).map_err(json_io_err)?;
    std::fs::write(config_dir().join(player_file("calibration", "json", player)), json)
}

/// Load HID per-channel calibration from `calibration.json` (no-op if absent).
pub fn load_calibration(player: usize, calibration: &mut [crate::input::ChannelCalibration]) {
    if let Ok(json) = std::fs::read_to_string(config_dir().join(player_file("calibration", "json", player))) {
        if let Ok(data) = serde_json::from_str::<CalibrationData>(&json) {
            for (i, ch) in data.channels.iter().enumerate() {
                if i < calibration.len() {
                    calibration[i].min = ch[0];
                    calibration[i].max = ch[1];
                    calibration[i].center = None;
                }
            }
        }
    }
}

/// Save the controller axis/switch mapping + per-mode expo/rate to `mapping.json`.
pub fn save_controller_mapping(ctrl: &crate::input::Controller) -> std::io::Result<()> {
    ensure_config_dir();
    let map = ControllerMapping {
        axis_channels: std::array::from_fn(|i| ctrl.axis_map[i].channel),
        axis_inverted: std::array::from_fn(|i| ctrl.axis_map[i].inverted),
        axis_expo_fpv: ctrl.mode_expo[0],
        axis_expo_drone: ctrl.mode_expo[1],
        axis_rate_fpv: ctrl.mode_rate[0],
        axis_rate_drone: ctrl.mode_rate[1],
        switch_channels: ctrl.switch_channels,
        switch_inverted: ctrl.switch_inverted,
        switch_level_mode: ctrl.switch_level_mode,
    };
    let json = serde_json::to_string_pretty(&map).map_err(json_io_err)?;
    std::fs::write(config_dir().join(player_file("mapping", "json", ctrl.player)), json)
}

/// Load the controller mapping from `mapping.json` into `ctrl` (no-op if absent).
pub fn load_controller_mapping(ctrl: &mut crate::input::Controller) {
    if let Ok(json) = std::fs::read_to_string(config_dir().join(player_file("mapping", "json", ctrl.player))) {
        if let Ok(map) = serde_json::from_str::<ControllerMapping>(&json) {
            for i in 0..4 {
                ctrl.axis_map[i].channel = map.axis_channels[i];
                ctrl.axis_map[i].inverted = map.axis_inverted[i];
            }
            ctrl.mode_expo[0] = map.axis_expo_fpv;
            ctrl.mode_expo[1] = map.axis_expo_drone;
            ctrl.mode_rate[0] = map.axis_rate_fpv;
            ctrl.mode_rate[1] = map.axis_rate_drone;
            ctrl.switch_channels = map.switch_channels;
            ctrl.switch_inverted = map.switch_inverted;
            ctrl.switch_level_mode = map.switch_level_mode;
        }
    }
}

/// Save player `player`'s last-used HID device path to `hid_device[N].txt` (for auto-connect).
pub fn save_hid_device_path(player: usize, path: &std::ffi::CStr) -> std::io::Result<()> {
    ensure_config_dir();
    std::fs::write(config_dir().join(player_file("hid_device", "txt", player)), path.to_string_lossy().as_bytes())
}

/// Load player `player`'s last-used HID device path (`None` if not saved / empty).
pub fn load_hid_device_path(player: usize) -> Option<std::ffi::CString> {
    let s = std::fs::read_to_string(config_dir().join(player_file("hid_device", "txt", player))).ok()?;
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    std::ffi::CString::new(s).ok()
}

/// Clear player `player`'s saved HID device path (on explicit disconnect).
pub fn clear_hid_device_path(player: usize) {
    let _ = std::fs::remove_file(config_dir().join(player_file("hid_device", "txt", player)));
}
