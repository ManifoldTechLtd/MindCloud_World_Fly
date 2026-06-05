//! Shared scene/camera plumbing used across phases: the scene-entity marker (for cleanup), the
//! split-screen camera marker + per-frame viewport assignment, and the window title/FPS readout.

use bevy::camera::Viewport;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;

use crate::app_state::AppState;
use crate::splat_plugin;

/// Marks entities spawned for the active scene (cameras, meshes, lights) so they can be despawned
/// when returning to the menu (see `menu::cleanup_scene`).
#[derive(Component)]
pub(crate) struct SceneEntity;

/// Marks a camera as one half of the split screen. `index` 0 = top, 1 = bottom.
#[derive(Component)]
pub(crate) struct SplitCamera {
    pub(crate) index: u8,
}

/// Registers the always-on scene utilities (viewport layout + window title).
pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, (set_split_viewports, update_window_title));
    }
}

/// Lays out the split-screen cameras. The split is active **only during flight** (`Playing`): each
/// camera gets its top/bottom-half viewport. In every other state (e.g. `Placement`) the scene is a
/// single full-window view — camera `index 0` fills the window, the rest are disabled — so placement
/// is not split. Runs every frame but only writes when something actually changes (resize-safe).
fn set_split_viewports(
    state: Res<State<AppState>>,
    windows: Query<&Window>,
    mut cameras: Query<(&SplitCamera, &mut Camera)>,
) {
    let Ok(window) = windows.single() else {
        return;
    };
    let w = window.physical_width();
    let h = window.physical_height();
    if w == 0 || h == 0 {
        return;
    }
    let split = *state.get() == AppState::Playing;
    let half = h / 2;
    for (sc, mut cam) in &mut cameras {
        if !split {
            // Single full-window view: only camera 0 renders, full-screen (no viewport rect).
            let active = sc.index == 0;
            if cam.is_active != active {
                cam.is_active = active;
            }
            if active && cam.viewport.is_some() {
                cam.viewport = None;
            }
            continue;
        }
        if !cam.is_active {
            cam.is_active = true;
        }
        let (pos, size) = match sc.index {
            0 => (UVec2::new(0, 0), UVec2::new(w, half)),
            _ => (UVec2::new(0, half), UVec2::new(w, h.saturating_sub(half))),
        };
        let needs_update = match &cam.viewport {
            Some(v) => v.physical_position != pos || v.physical_size != size,
            None => true,
        };
        if needs_update {
            cam.viewport = Some(Viewport {
                physical_position: pos,
                physical_size: size,
                depth: 0.0..1.0,
            });
        }
    }
}

/// Shows the smoothed FPS + splat load status in the window title bar.
fn update_window_title(
    diagnostics: Res<DiagnosticsStore>,
    mut windows: Query<&mut Window>,
    splat_scene: Res<splat_plugin::SplatScene>,
) {
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    let status = if splat_scene.loaded {
        "splat loaded"
    } else if splat_scene.ply_path.is_some() {
        "loading PLY..."
    } else {
        "no scene"
    };

    if let Ok(mut window) = windows.single_mut() {
        window.title = format!("{:.0} FPS | {}", fps, status);
    }
}
