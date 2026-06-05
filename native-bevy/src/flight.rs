//! Flight phase: the player drone resource + the systems that drive it (keyboard → physics, then
//! physics → camera). Registered by [`FlightPlugin`]; active only in `AppState::Playing`.

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};

use crate::app_state::{AppState, CurrentSceneConfig, GameMode};
use crate::drone::{Drone, FlightMode, KeyState};
use crate::gate_plugin::RaceCourse;
use crate::hud;
use crate::persistence;
use crate::placement::SpawnMarker;
use crate::scene::SplitCamera;
use crate::settings_ui;
use crate::splat_plugin::SplatCamera;

/// One player's drone + its keyboard state + armed flag.
pub struct PlayerState {
    pub drone: Drone,
    pub keys: KeyState,
    pub armed: bool,
}

/// All active players: 1 in single-player, 2 in split-screen. The index matches `SplitCamera.index`
/// (0 = top half, 1 = bottom half). Inserted on entering `Playing` by [`setup_flight`].
#[derive(Resource, Default)]
pub struct Players(pub Vec<PlayerState>);

/// Whether the in-flight settings panel (toggled with F1) is open.
#[derive(Resource, Default)]
pub struct SettingsOpen(pub bool);

/// Wires the flight systems: drone setup on entering `Playing`, then input → physics → camera each
/// frame while playing.
pub struct FlightPlugin;

impl Plugin for FlightPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SettingsOpen>();
        app.init_resource::<Players>();
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

/// `OnEnter(Playing)`: build the player drone(s) from the scene config and hide the placement
/// marker. The orbit cameras persist; `placement_orbit_system` stops (run_if Placement) and
/// `drone_camera_system` now drives their transforms as FPV cameras (one per `SplitCamera`).
///
/// SinglePlayer = 1 drone (disarmed). SplitScreen = 2 drones, P2 offset +2 on X so they don't
/// overlap, both armed at start (ported from native).
fn setup_flight(
    mut commands: Commands,
    config: Res<CurrentSceneConfig>,
    mode: Res<GameMode>,
    markers: Query<Entity, With<SpawnMarker>>,
) {
    let s = config.0.spawn;
    let make = |offset_x: f32| -> Drone {
        let mut drone = Drone::new();
        drone.world_up = config.0.world_up;
        drone.spawn_heading = config.0.heading_deg;
        persistence::load_drone_settings(&mut drone);
        drone.reset(s[0] + offset_x, s[1], s[2]);
        // Stabilized (Drone) mode is far easier to fly on a keyboard than FPV acro; default to it.
        drone.flight_mode = FlightMode::Drone;
        drone
    };

    let players = match *mode {
        GameMode::SinglePlayer => vec![PlayerState {
            drone: make(0.0),
            keys: KeyState::default(),
            armed: false,
        }],
        GameMode::SplitScreen => vec![
            PlayerState { drone: make(0.0), keys: KeyState::default(), armed: true },
            PlayerState { drone: make(2.0), keys: KeyState::default(), armed: true },
        ],
    };
    info!(
        "[Playing] {} drone(s) spawned at {:?} heading {:.1} ({:?})",
        players.len(), s, config.0.heading_deg, *mode
    );
    commands.insert_resource(Players(players));
    commands.insert_resource(SettingsOpen(false));
    // The placement spawn marker is not shown during flight.
    for e in &markers {
        commands.entity(e).despawn();
    }
}

/// `Update` (Playing): keyboard → `DroneInput` → physics step, per player.
/// P1: arrows = roll/pitch, W/S = throttle, A/D = yaw, Space = arm, R = reset, M = mode.
/// P2 (split): I/K = pitch, J/L = roll, T/G = throttle, F/H = yaw, Enter = arm, Backspace = reset,
/// N = mode.
fn drone_input_system(
    time: Res<Time>,
    keys_in: Res<ButtonInput<KeyCode>>,
    mut players: ResMut<Players>,
) {
    if players.0.is_empty() {
        return;
    }
    let dt = time.delta_secs();

    // --- Player 1 ---
    {
        let p = &mut players.0[0];
        let k = &mut p.keys;
        k.w = keys_in.pressed(KeyCode::KeyW);
        k.s = keys_in.pressed(KeyCode::KeyS);
        k.a = keys_in.pressed(KeyCode::KeyA);
        k.d = keys_in.pressed(KeyCode::KeyD);
        k.up = keys_in.pressed(KeyCode::ArrowUp);
        k.down = keys_in.pressed(KeyCode::ArrowDown);
        k.left = keys_in.pressed(KeyCode::ArrowLeft);
        k.right = keys_in.pressed(KeyCode::ArrowRight);
        if keys_in.just_pressed(KeyCode::Space) {
            p.armed = !p.armed;
            info!("[P1] armed = {}", p.armed);
        }
        if keys_in.just_pressed(KeyCode::KeyR) {
            p.drone.reset_to_spawn();
        }
        if keys_in.just_pressed(KeyCode::KeyM) {
            p.drone.flight_mode = toggle_mode(p.drone.flight_mode);
            info!("[P1] flight mode = {:?}", p.drone.flight_mode);
        }
        let armed = p.armed;
        let input = p.keys.to_input(armed);
        p.drone.update(dt, &input);
    }

    // --- Player 2 (split-screen only) ---
    // P2's physical keys map onto the same `KeyState` fields so `KeyState::to_input` reproduces
    // native's `KeyStateP2::to_input`: I/K → up/down (pitch), J/L → left/right (roll), T/G → w/s
    // (throttle), F/H → a/d (yaw).
    if players.0.len() > 1 {
        let p = &mut players.0[1];
        let k = &mut p.keys;
        k.up = keys_in.pressed(KeyCode::KeyI);
        k.down = keys_in.pressed(KeyCode::KeyK);
        k.left = keys_in.pressed(KeyCode::KeyJ);
        k.right = keys_in.pressed(KeyCode::KeyL);
        k.w = keys_in.pressed(KeyCode::KeyT);
        k.s = keys_in.pressed(KeyCode::KeyG);
        k.a = keys_in.pressed(KeyCode::KeyF);
        k.d = keys_in.pressed(KeyCode::KeyH);
        if keys_in.just_pressed(KeyCode::Enter) {
            p.armed = !p.armed;
            info!("[P2] armed = {}", p.armed);
        }
        if keys_in.just_pressed(KeyCode::Backspace) {
            p.drone.reset_to_spawn();
        }
        if keys_in.just_pressed(KeyCode::KeyN) {
            p.drone.flight_mode = toggle_mode(p.drone.flight_mode);
            info!("[P2] flight mode = {:?}", p.drone.flight_mode);
        }
        let armed = p.armed;
        let input = p.keys.to_input(armed);
        p.drone.update(dt, &input);
    }
}

fn toggle_mode(m: FlightMode) -> FlightMode {
    match m {
        FlightMode::Fpv => FlightMode::Drone,
        FlightMode::Drone => FlightMode::Fpv,
    }
}

/// `Update` (Playing): drive each SplatCamera from its player's drone camera transform. The camera
/// for `SplitCamera{index}` follows player `index`; the single-player camera (no `SplitCamera`)
/// follows player 0. The drone yields (pos, q_wb) in web-splat convention. The Bevy camera that
/// renders the SAME view as web-splat(q_wb) is `q_wb.conjugate() * flip_x` (180° about X); the splat
/// node re-derives `(bevy_q*flip_x).conjugate()` = q_wb, so meshes (gates/markers) stay locked to the
/// splat under all rotations. Verified in tests/splat_camera_align.rs.
fn drone_camera_system(
    players: Res<Players>,
    mut cams: Query<(&mut Transform, Option<&SplitCamera>), With<SplatCamera>>,
) {
    if players.0.is_empty() {
        return;
    }
    let flip_x = cgmath::Quaternion::new(0.0, 1.0, 0.0, 0.0);
    for (mut t, split) in &mut cams {
        let idx = split
            .map(|s| s.index as usize)
            .unwrap_or(0)
            .min(players.0.len() - 1);
        let (pos, cam_orient) = players.0[idx].drone.camera_transform();
        let q = cam_orient.conjugate() * flip_x;
        t.translation = Vec3::new(pos.x, pos.y, pos.z);
        t.rotation = Quat::from_xyzw(q.v.x, q.v.y, q.v.z, q.s);
    }
}

/// `EguiPrimaryContextPass` (Playing): draw the flight HUD/OSD over the splat render. Single-player
/// draws one full-screen HUD; split-screen draws one HUD per viewport half (top = P1, bottom = P2),
/// each labelled and clipped to its half via `hud::draw_hud`'s viewport argument.
fn hud_system(
    mut contexts: EguiContexts,
    players: Res<Players>,
    windows: Query<&Window>,
    course: Option<Res<RaceCourse>>,
    diagnostics: Res<DiagnosticsStore>,
) -> Result {
    if players.0.is_empty() {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0) as f32;

    if players.0.len() == 1 {
        let p = &players.0[0];
        hud::draw_hud(ctx, &p.drone, p.armed, fps, None, None);
        if let Some(course) = &course {
            hud::draw_race(ctx, &course.0, None, None);
        }
    } else {
        // Window uses scale_factor_override(1.0) → egui logical points == physical px == the camera
        // viewports, so the top/bottom halves line up with the split-screen render.
        let Ok(win) = windows.single() else {
            return Ok(());
        };
        let (w, h) = (win.width(), win.height());
        let half = h * 0.5;
        let labels = ["P1", "P2"];
        for (i, p) in players.0.iter().enumerate().take(2) {
            let min_y = if i == 0 { 0.0 } else { half };
            let rect = egui::Rect::from_min_size(egui::pos2(0.0, min_y), egui::vec2(w, half));
            hud::draw_hud(ctx, &p.drone, p.armed, fps, Some(labels[i]), Some(rect));
        }
        // Race panel on P1 (top half) only — the single course tracks player 0.
        if let Some(course) = &course {
            let rect = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(w, half));
            hud::draw_race(ctx, &course.0, Some("P1"), Some(rect));
        }
    }
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
    mut players: ResMut<Players>,
) -> Result {
    if !settings_open.0 || players.0.is_empty() {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    settings_ui::draw_settings(ctx, &mut settings_open.0, &mut players.0[0].drone);
    Ok(())
}
