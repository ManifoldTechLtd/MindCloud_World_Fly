//! Flight phase: the player drone resource + the systems that drive it (keyboard → physics, then
//! physics → camera). Registered by [`FlightPlugin`]; active only in `AppState::Playing`.

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass};

use crate::app_state::{AppState, CurrentSceneConfig};
use crate::drone::{Drone, FlightMode, KeyState};
use crate::hud;
use crate::persistence;
use crate::placement::SpawnMarker;
use crate::settings_ui;
use crate::splat_plugin::SplatCamera;

/// The single-player drone + its keyboard state + armed flag (inserted on entering `Playing`).
#[derive(Resource)]
pub struct PlayerDrone {
    pub drone: Drone,
    pub keys: KeyState,
    pub armed: bool,
}

/// Whether the in-flight settings panel (toggled with F1) is open.
#[derive(Resource, Default)]
pub struct SettingsOpen(pub bool);

/// Wires the flight systems: drone setup on entering `Playing`, then input → physics → camera each
/// frame while playing.
pub struct FlightPlugin;

impl Plugin for FlightPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SettingsOpen>();
        app.add_systems(OnEnter(AppState::Playing), setup_flight);
        app.add_systems(
            Update,
            (settings_toggle_system, drone_input_system, drone_camera_system)
                .chain()
                .run_if(in_state(AppState::Playing)),
        );
        // HUD + settings panel draw in the egui pass while flying.
        app.add_systems(
            EguiPrimaryContextPass,
            (hud_system, settings_ui_system).run_if(in_state(AppState::Playing)),
        );
    }
}

/// `OnEnter(Playing)`: build the drone from the scene config and hide the placement marker. The
/// orbit cameras persist; `placement_orbit_system` stops (run_if Placement) and
/// `drone_camera_system` now drives their transforms as FPV cameras.
fn setup_flight(
    mut commands: Commands,
    config: Res<CurrentSceneConfig>,
    markers: Query<Entity, With<SpawnMarker>>,
) {
    let mut drone = Drone::new();
    drone.world_up = config.0.world_up;
    drone.spawn_heading = config.0.heading_deg;
    persistence::load_drone_settings(&mut drone);
    let s = config.0.spawn;
    drone.reset(s[0], s[1], s[2]);
    // Stabilized (Drone) mode is far easier to fly on a keyboard than FPV acro; default to it.
    drone.flight_mode = FlightMode::Drone;
    info!(
        "[Playing] drone spawned at {:?} heading {:.1} mode {:?}",
        s, config.0.heading_deg, drone.flight_mode
    );
    commands.insert_resource(PlayerDrone { drone, keys: KeyState::default(), armed: false });
    commands.insert_resource(SettingsOpen(false));
    // The placement spawn marker is not shown during flight.
    for e in &markers {
        commands.entity(e).despawn();
    }
}

/// `Update` (Playing): keyboard → `DroneInput` → physics step. Space = arm/disarm, R = reset to
/// spawn, M = toggle FPV(acro)/stabilized. Arrows = roll/pitch, W/S = throttle, A/D = yaw.
fn drone_input_system(
    time: Res<Time>,
    keys_in: Res<ButtonInput<KeyCode>>,
    mut pd: ResMut<PlayerDrone>,
) {
    {
        let k = &mut pd.keys;
        k.w = keys_in.pressed(KeyCode::KeyW);
        k.s = keys_in.pressed(KeyCode::KeyS);
        k.a = keys_in.pressed(KeyCode::KeyA);
        k.d = keys_in.pressed(KeyCode::KeyD);
        k.up = keys_in.pressed(KeyCode::ArrowUp);
        k.down = keys_in.pressed(KeyCode::ArrowDown);
        k.left = keys_in.pressed(KeyCode::ArrowLeft);
        k.right = keys_in.pressed(KeyCode::ArrowRight);
    }
    if keys_in.just_pressed(KeyCode::Space) {
        pd.armed = !pd.armed;
        info!("[Playing] armed = {}", pd.armed);
    }
    if keys_in.just_pressed(KeyCode::KeyR) {
        pd.drone.reset_to_spawn();
    }
    if keys_in.just_pressed(KeyCode::KeyM) {
        pd.drone.flight_mode = match pd.drone.flight_mode {
            FlightMode::Fpv => FlightMode::Drone,
            FlightMode::Drone => FlightMode::Fpv,
        };
        info!("[Playing] flight mode = {:?}", pd.drone.flight_mode);
    }
    let armed = pd.armed;
    let input = pd.keys.to_input(armed);
    let dt = time.delta_secs();
    pd.drone.update(dt, &input);
}

/// `Update` (Playing): drive every SplatCamera from the drone's camera transform. The drone yields
/// (pos, q_wb) in web-splat convention; the splat node re-applies the Bevy→COLMAP rotation
/// (180° about X), so the Bevy camera rotation is `cam_orient * flip_x` (flip_x is its own inverse).
fn drone_camera_system(pd: Res<PlayerDrone>, mut cams: Query<&mut Transform, With<SplatCamera>>) {
    let (pos, cam_orient) = pd.drone.camera_transform();
    let flip_x = cgmath::Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let q = cam_orient * flip_x;
    let bevy_pos = Vec3::new(pos.x, pos.y, pos.z);
    let bevy_rot = Quat::from_xyzw(q.v.x, q.v.y, q.v.z, q.s);
    for mut t in &mut cams {
        t.translation = bevy_pos;
        t.rotation = bevy_rot;
    }
}

/// `EguiPrimaryContextPass` (Playing): draw the flight HUD/OSD over the splat render. Single-player
/// (full screen, no label); split-screen per-player HUDs land with the second drone instance.
fn hud_system(
    mut contexts: EguiContexts,
    pd: Res<PlayerDrone>,
    diagnostics: Res<DiagnosticsStore>,
) -> Result {
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0) as f32;
    hud::draw_hud(ctx, &pd.drone, pd.armed, fps, None, None);
    Ok(())
}

/// `Update` (Playing): F1 toggles the settings panel (native uses F1). Esc still toggles the exit
/// dialog (`menu::handle_esc`); close the panel with F1 or its `[x]`.
fn settings_toggle_system(keys: Res<ButtonInput<KeyCode>>, mut settings_open: ResMut<SettingsOpen>) {
    if keys.just_pressed(KeyCode::F1) {
        settings_open.0 = !settings_open.0;
    }
}

/// `EguiPrimaryContextPass` (Playing): draw the settings panel (drone physics/rates/PID) when open.
/// Param edits persist per-section inside `settings_ui::draw_settings`.
fn settings_ui_system(
    mut contexts: EguiContexts,
    mut settings_open: ResMut<SettingsOpen>,
    mut pd: ResMut<PlayerDrone>,
) -> Result {
    if !settings_open.0 {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    settings_ui::draw_settings(ctx, &mut settings_open.0, &mut pd.drone);
    Ok(())
}
