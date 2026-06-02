/// Settings + persistence — JSON config files.
/// Ported from src/path-store.js + controller.js settings logic.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// Per-scene gate path record (stored as JSON).
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SceneRecord {
    pub points: Vec<[f32; 3]>,
    pub gate_size: f32,
    pub best_lap_ms: Option<f64>,
}

/// Drone physics settings (persisted).
#[derive(Serialize, Deserialize, Debug, Clone)]
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

impl Default for DroneSettings {
    fn default() -> Self {
        Self {
            mass: 500.0, max_thrust: 1000.0, drag_cd: 1.0, drag_area: 0.01,
            collision_radius: 0.3, drone_size: 0.3, camera_mount_angle: 30.0,
            max_pitch_rate: 220.0, max_roll_rate: 220.0, max_yaw_rate: 120.0,
            drone_pos_kp: 2.0, drone_pos_ki: 0.3, drone_pos_kd: 0.1,
            drone_vel_kp: 3.0, drone_vel_ki: 1.0, drone_vel_kd: 0.05,
            drone_alt_kp: 4.0, drone_alt_ki: 2.0, drone_alt_kd: 0.1,
        }
    }
}

pub fn save_drone_settings(drone: &crate::drone::Drone) -> anyhow::Result<()> {
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
    let json = serde_json::to_string_pretty(&s)?;
    std::fs::write(config_dir().join("drone.json"), json)?;
    Ok(())
}

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
            log::info!("Loaded drone settings");
        }
    }
}

// Keep old AppSettings for backward compat (unused now)
pub type AppSettings = DroneSettings;

/// Generate a storage key from scene filename + size.
pub fn scene_key(file_path: &Path) -> String {
    let name = file_path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let size = std::fs::metadata(file_path)
        .map(|m| m.len())
        .unwrap_or(0);
    format!("{}_{}", name, size)
}

/// Directory for storing per-scene gate paths.
fn gate_paths_dir() -> PathBuf {
    PathBuf::from("asset/gate-paths")
}

/// Save a scene record to disk.
pub fn save_scene_record(file_path: &Path, record: &SceneRecord) -> anyhow::Result<()> {
    let dir = gate_paths_dir();
    std::fs::create_dir_all(&dir)?;
    let key = scene_key(file_path);
    let path = dir.join(format!("{}.json", key));
    let json = serde_json::to_string_pretty(record)?;
    std::fs::write(path, json)?;
    Ok(())
}

/// Load a scene record from disk (returns None if not found).
pub fn load_scene_record(file_path: &Path) -> Option<SceneRecord> {
    let key = scene_key(file_path);
    let path = gate_paths_dir().join(format!("{}.json", key));
    let data = std::fs::read_to_string(path).ok()?;
    serde_json::from_str(&data).ok()
}

/// Save app settings.
pub fn save_settings(settings: &AppSettings) -> anyhow::Result<()> {
    let json = serde_json::to_string_pretty(settings)?;
    std::fs::write("mindcloud-fly-settings.json", json)?;
    Ok(())
}

/// Load app settings.
pub fn load_settings() -> AppSettings {
    std::fs::read_to_string("mindcloud-fly-settings.json")
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default()
}

/// Calibration data (serializable).
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CalibrationData {
    pub channels: Vec<[Option<u16>; 2]>, // [min, max] per channel
}

/// Controller mapping data (serializable).
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControllerMapping {
    pub axis_channels: [i32; 4],       // roll, pitch, throttle, yaw
    pub axis_inverted: [bool; 4],
    pub axis_expo_fpv: [f32; 4],       // expo per axis for FPV mode
    pub axis_expo_drone: [f32; 4],     // expo per axis for Drone mode
    pub axis_rate_fpv: [f32; 4],       // rate per axis for FPV mode
    pub axis_rate_drone: [f32; 4],     // rate per axis for Drone mode
    pub switch_channels: [i32; 2],     // arm, mode
    pub switch_inverted: [bool; 2],
    pub switch_level_mode: [bool; 2],
}

/// Get config directory path (always the same regardless of CWD).
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

/// Save HID calibration to disk.
pub fn save_calibration(calibration: &[crate::input::ChannelCalibration]) -> anyhow::Result<()> {
    ensure_config_dir();
    let data = CalibrationData {
        channels: calibration.iter().map(|c| [c.min, c.max]).collect(),
    };
    let json = serde_json::to_string_pretty(&data)?;
    std::fs::write(config_dir().join("calibration.json"), json)?;
    Ok(())
}

/// Load HID calibration from disk.
pub fn load_calibration(calibration: &mut [crate::input::ChannelCalibration]) {
    if let Ok(json) = std::fs::read_to_string(config_dir().join("calibration.json")) {
        if let Ok(data) = serde_json::from_str::<CalibrationData>(&json) {
            for (i, ch) in data.channels.iter().enumerate() {
                if i < calibration.len() {
                    calibration[i].min = ch[0];
                    calibration[i].max = ch[1];
                    calibration[i].center = None;
                }
            }
            log::info!("Loaded calibration for {} channels", data.channels.len());
        }
    }
}

/// Save controller mapping to disk.
pub fn save_controller_mapping(ctrl: &crate::input::Controller) -> anyhow::Result<()> {
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
    let json = serde_json::to_string_pretty(&map)?;
    std::fs::write(config_dir().join("mapping.json"), json)?;
    Ok(())
}

/// Save per-scene config (world_up, spawn, heading).
pub fn save_scene_config(scene_key: &str, config: &crate::app_state::SceneConfig) -> anyhow::Result<()> {
    ensure_config_dir();
    let path = config_dir().join("scenes.json");
    let mut map: std::collections::HashMap<String, crate::app_state::SceneConfig> =
        std::fs::read_to_string(&path).ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
    map.insert(scene_key.to_string(), config.clone());
    let json = serde_json::to_string_pretty(&map)?;
    std::fs::write(path, json)?;
    Ok(())
}

/// Load per-scene config (returns default if not found).
pub fn load_scene_config(scene_key: &str) -> crate::app_state::SceneConfig {
    let path = config_dir().join("scenes.json");
    std::fs::read_to_string(path).ok()
        .and_then(|s| serde_json::from_str::<std::collections::HashMap<String, crate::app_state::SceneConfig>>(&s).ok())
        .and_then(|map| map.get(scene_key).cloned())
        .unwrap_or_default()
}

/// Load controller mapping from disk.
pub fn load_controller_mapping(ctrl: &mut crate::input::Controller) {
    if let Ok(json) = std::fs::read_to_string(config_dir().join("mapping.json")) {
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
            log::info!("Loaded controller mapping");
        }
    }
}
