//! Application state machine — Bevy port of `native/src/app_state.rs`.
//!
//! Flow: `ModeSelect → SceneSelect → Loading → Placement → Playing`
//! ```text
//!   ModeSelect ─▶ SceneSelect ─▶ Loading ─▶ Placement ─▶ Playing
//!                     ▲                                      │
//!                     └────────────── (Esc in game) ─────────┘
//! ```
//!
//! The native version drove a hand-rolled `Phase` enum inside a winit event loop. Here it is
//! reimplemented with Bevy [`States`] so each phase's behaviour is expressed as systems gated by
//! `OnEnter` / `in_state`. Pure data types (`GameMode`, and later `WorldUp` / `SceneConfig`) are
//! copied verbatim from native so the ported physics / persistence logic stays identical.

use bevy::prelude::*;

/// Top-level application state. Drives which systems run (menus, loading, placement, flight).
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AppState {
    /// Choose single-player vs split-screen. (rich menu UI: later slice — Phase 8)
    #[default]
    ModeSelect,
    /// Pick a `.ply` scene file. (rich menu UI: later slice — Phase 8)
    SceneSelect,
    /// Background PLY load in progress (loading screen: later slice — Phase 1.4).
    Loading,
    /// Scene loaded; orbit camera to set spawn position / heading / world-up (Phase 1.5/1.6).
    Placement,
    /// Drone flight (Phase 4).
    Playing,
}

/// Game mode: one drone + full-window view, or two split-screen halves.
/// Ported from `native/src/app_state.rs`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Resource)]
pub enum GameMode {
    #[default]
    SinglePlayer,
    SplitScreen,
}

/// World coordinate system (scene-dependent). Ported verbatim from `native/src/app_state.rs`.
/// Determines the gravity direction and the horizontal plane used for spawn placement + flight.
#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum WorldUp {
    /// Z-up: +Z = sky, gravity = (0, 0, -G). Horizontal plane = XY.
    Zup,
    /// COLMAP: +Y = gravity direction (down), gravity = (0, +G, 0). Horizontal plane = XZ.
    Colmap,
}

impl WorldUp {
    /// Parse a CLI/string value (`"zup"` / `"colmap"`); anything else falls back to `Zup`.
    pub fn parse(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "colmap" | "y" | "ydown" | "y-down" => WorldUp::Colmap,
            _ => WorldUp::Zup,
        }
    }
}

/// Per-scene persistent config (world up, spawn point, heading). Ported verbatim from
/// `native/src/app_state.rs`; serialized to `~/.config/mindcloud-fly/scenes.json` keyed by scene.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SceneConfig {
    pub world_up: WorldUp,
    pub spawn: [f32; 3],
    pub heading_deg: f32,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            world_up: WorldUp::Zup,
            spawn: [0.0, 0.0, 2.0],
            heading_deg: 0.0,
        }
    }
}

/// The active scene's config, loaded on entering `Placement` and consumed by placement/flight
/// (e.g. the drone reads `world_up` for its gravity direction and `spawn`/`heading_deg` to spawn).
#[derive(Resource, Clone, Debug, Default)]
pub struct CurrentSceneConfig(pub SceneConfig);
