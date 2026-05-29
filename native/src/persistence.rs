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

/// Global app settings.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AppSettings {
    // Drone physics
    pub mass: f32,
    pub max_thrust: f32,
    pub drag_cd: f32,
    pub drag_area: f32,
    pub collision_radius: f32,
    pub drone_size: f32,
    pub camera_mount_angle: f32,

    // Controller axis mapping
    pub axis_channels: [i32; 5], // roll, pitch, throttle, yaw, cameraTilt
    pub axis_inverted: [bool; 5],
    pub axis_deadzone: [f32; 5],
    pub axis_rate: [f32; 5],
    pub axis_expo: [f32; 5],
}

impl Default for AppSettings {
    fn default() -> Self {
        Self {
            mass: 500.0,
            max_thrust: 1000.0,
            drag_cd: 1.0,
            drag_area: 0.01,
            collision_radius: 0.3,
            drone_size: 0.3,
            camera_mount_angle: 30.0,
            axis_channels: [0, 1, 2, 3, -1],
            axis_inverted: [false; 5],
            axis_deadzone: [0.0; 5],
            axis_rate: [1.0; 5],
            axis_expo: [0.0; 5],
        }
    }
}

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
    pub axis_expo: [f32; 4],
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
        axis_expo: std::array::from_fn(|i| ctrl.axis_map[i].expo),
        switch_channels: ctrl.switch_channels,
        switch_inverted: ctrl.switch_inverted,
        switch_level_mode: ctrl.switch_level_mode,
    };
    let json = serde_json::to_string_pretty(&map)?;
    std::fs::write(config_dir().join("mapping.json"), json)?;
    Ok(())
}

/// Load controller mapping from disk.
pub fn load_controller_mapping(ctrl: &mut crate::input::Controller) {
    if let Ok(json) = std::fs::read_to_string(config_dir().join("mapping.json")) {
        if let Ok(map) = serde_json::from_str::<ControllerMapping>(&json) {
            for i in 0..4 {
                ctrl.axis_map[i].channel = map.axis_channels[i];
                ctrl.axis_map[i].inverted = map.axis_inverted[i];
                ctrl.axis_map[i].expo = map.axis_expo[i];
            }
            ctrl.switch_channels = map.switch_channels;
            ctrl.switch_inverted = map.switch_inverted;
            ctrl.switch_level_mode = map.switch_level_mode;
            log::info!("Loaded controller mapping");
        }
    }
}
