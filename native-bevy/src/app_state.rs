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
