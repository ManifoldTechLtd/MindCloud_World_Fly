//! Config persistence — JSON files in a `config/` folder beside the app's `assets/` (dev:
//! `native-bevy/config/` via `CARGO_MANIFEST_DIR`; packaged: a `config/` next to the binary).
//!
//! Ported from `native/src/persistence.rs`. **Subset for this slice**: only the per-scene config
//! (`world_up` / `spawn` / `heading`). The other persisted blobs (drone settings, HID calibration,
//! controller mapping, gate-path records) will be ported alongside their feature slices (drone /
//! input / gates) so each lands with the type it serializes.

use crate::app_state::SceneConfig;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Persistence directory: a `config/` folder that lives NEXT TO the app's `assets/`, so the whole
/// app (binary + assets + saved settings) is self-contained and can be packaged or copied as one
/// unit. The base path mirrors how Bevy resolves `assets/`, so `config/` is always its sibling:
///   - `BEVY_ASSET_ROOT` if set (matches Bevy; used by tests / advanced setups),
///   - else `CARGO_MANIFEST_DIR` (set by `cargo run` in dev) -> `native-bevy/config`,
///   - else the executable's own directory (a packaged build) -> `<app_dir>/config`.
fn config_dir() -> PathBuf {
    let base = std::env::var_os("BEVY_ASSET_ROOT")
        .or_else(|| std::env::var_os("CARGO_MANIFEST_DIR"))
        .map(PathBuf::from)
        .or_else(|| std::env::current_exe().ok().and_then(|e| e.parent().map(Path::to_path_buf)))
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("config")
}

fn ensure_config_dir() {
    let _ = std::fs::create_dir_all(config_dir());
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

/// Save audio volumes + mute to `audio.json` (written when a settings slider changes).
pub fn save_audio_settings(s: &crate::audio::AudioSettings) -> std::io::Result<()> {
    ensure_config_dir();
    let json = serde_json::to_string_pretty(s)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    std::fs::write(config_dir().join("audio.json"), json)
}

/// Load audio settings from `audio.json` (`None` if absent / unparseable → caller uses defaults).
pub fn load_audio_settings() -> Option<crate::audio::AudioSettings> {
    let json = std::fs::read_to_string(config_dir().join("audio.json")).ok()?;
    serde_json::from_str(&json).ok()
}

// ---- HID controller persistence (keyed by device NAME) ----
// A controller's config (axis/switch mapping, per-mode expo/rate, calibration) is keyed by its
// product name, so a transmitter keeps ITS config no matter which player slot / game mode drives it.
// One file per controller (`controllers/<sanitized-name>.json`), overwritten on every save, so the
// config dir never grows unbounded no matter how often it is re-calibrated.

fn json_io_err(e: serde_json::Error) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::Other, e)
}

/// Full per-controller config: axis/switch mapping, per-mode expo + rate, and per-channel
/// calibration. Serialized to `controllers/<name>.json`.
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct ControllerConfig {
    pub axis_channels: [i32; 4],   // roll, pitch, throttle, yaw
    pub axis_inverted: [bool; 4],
    pub axis_expo_fpv: [f32; 4],
    pub axis_expo_drone: [f32; 4],
    pub axis_rate_fpv: [f32; 4],
    pub axis_rate_drone: [f32; 4],
    pub switch_channels: [i32; 2], // arm, mode
    pub switch_inverted: [bool; 2],
    pub switch_level_mode: [bool; 2],
    pub switch_threshold: f32,
    /// `[min, max]` per channel (`None` until learned during calibration).
    pub calibration: Vec<[Option<u16>; 2]>,
}

/// `controllers/` subdir holding one JSON per known transmitter.
fn controllers_dir() -> PathBuf {
    config_dir().join("controllers")
}

/// Filesystem-safe stem for a controller's product name (lowercased; non-alnum → `_`).
fn sanitize_name(name: &str) -> String {
    let s: String = name
        .trim()
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '_' })
        .collect();
    let s = s.trim_matches('_').to_string();
    if s.is_empty() { "controller".to_string() } else { s }
}

fn controller_config_path(name: &str) -> PathBuf {
    controllers_dir().join(format!("{}.json", sanitize_name(name)))
}

/// Save a controller's full config to `controllers/<name>.json` (creates or OVERWRITES).
pub fn save_controller_config(name: &str, cfg: &ControllerConfig) -> std::io::Result<()> {
    let dir = controllers_dir();
    std::fs::create_dir_all(&dir)?;
    let json = serde_json::to_string_pretty(cfg).map_err(json_io_err)?;
    std::fs::write(controller_config_path(name), json)
}

/// Load a controller's saved config by name (`None` if absent / unparseable).
pub fn load_controller_config(name: &str) -> Option<ControllerConfig> {
    let json = std::fs::read_to_string(controller_config_path(name)).ok()?;
    serde_json::from_str(&json).ok()
}

// ---- Last-used transmitter per (mode, slot) ----
// Each logical slot ("single", "dual_p1", "dual_p2") remembers the NAME of the transmitter last
// connected there, so the right device auto-connects on the next visit. Names (not paths) survive
// replugging. Stored together in `hid_last_used.json`.

fn last_used_path() -> PathBuf {
    config_dir().join("hid_last_used.json")
}

fn load_last_used_map() -> HashMap<String, String> {
    std::fs::read_to_string(last_used_path())
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default()
}

/// Remember `name` as the last-used transmitter for `slot` ("single" / "dual_p1" / "dual_p2").
pub fn save_last_device(slot: &str, name: &str) {
    ensure_config_dir();
    let mut map = load_last_used_map();
    map.insert(slot.to_string(), name.to_string());
    if let Ok(json) = serde_json::to_string_pretty(&map) {
        let _ = std::fs::write(last_used_path(), json);
    }
}

/// Last-used transmitter name for `slot` (`None` if never set).
pub fn load_last_device(slot: &str) -> Option<String> {
    load_last_used_map().get(slot).cloned()
}

/// Forget `slot`'s last-used transmitter (on explicit disconnect).
pub fn clear_last_device(slot: &str) {
    let mut map = load_last_used_map();
    if map.remove(slot).is_some() {
        if let Ok(json) = serde_json::to_string_pretty(&map) {
            let _ = std::fs::write(last_used_path(), json);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitize_name_is_filesystem_safe_and_stable() {
        // A given controller always maps to the SAME single filename → re-calibration overwrites
        // one file (no bloat); non-alnum chars are folded so the path is always valid.
        assert_eq!(sanitize_name("RadioMaster Zorro"), "radiomaster_zorro");
        assert_eq!(sanitize_name("  TX16S (EdgeTX)  "), "tx16s__edgetx");
        assert_eq!(sanitize_name("RadioMaster Zorro"), sanitize_name("radiomaster zorro"));
        // Degenerate names never produce an empty (invalid) filename.
        assert_eq!(sanitize_name("///"), "controller");
        assert_eq!(sanitize_name(""), "controller");
    }

    #[test]
    fn config_dir_sits_next_to_assets_never_in_home() {
        // Mirrors Bevy's asset base path. Under `cargo test`, CARGO_MANIFEST_DIR is set, so config
        // resolves to `<crate>/config` (sibling of `<crate>/assets`) — never `$HOME/.config`.
        let dir = config_dir();
        assert_eq!(dir.file_name().unwrap(), "config");
        if std::env::var_os("BEVY_ASSET_ROOT").is_none() {
            let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap();
            assert_eq!(dir, Path::new(&manifest).join("config"));
        }
    }
}
