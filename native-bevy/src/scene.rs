//! Shared scene/camera plumbing used across phases: the scene-entity marker (for cleanup), the
//! split-screen camera marker + per-frame viewport assignment, and the window title/FPS readout.

use bevy::camera::Viewport;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;

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

/// Assigns each split-screen camera its viewport rect (top/bottom half) based on the current
/// window size. Runs every frame but only writes when the rect actually changes (so window
/// resizes are handled without spamming change detection).
fn set_split_viewports(windows: Query<&Window>, mut cameras: Query<(&SplitCamera, &mut Camera)>) {
    let Ok(window) = windows.single() else {
        return;
    };
    let w = window.physical_width();
    let h = window.physical_height();
    if w == 0 || h == 0 {
        return;
    }
    let half = h / 2;
    for (split, mut cam) in &mut cameras {
        let (pos, size) = match split.index {
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
