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
