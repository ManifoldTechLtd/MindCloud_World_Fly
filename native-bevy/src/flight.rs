//! Flight phase: the player drone resource + the systems that drive it (keyboard → physics, then
//! physics → camera). Registered by [`FlightPlugin`]; active only in `AppState::Playing`.

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};

use crate::app_state::{AppState, CurrentSceneConfig, GameMode, WorldUp};
use crate::drone::{Drone, DroneInput, FlightMode, KeyState};
use crate::gate_plugin::RaceCourse;
use crate::gates;
use crate::hud;
use crate::input_plugin::{self, ControllerRes, HidConnections, HidDevices};
use crate::persistence;
use crate::placement::SpawnMarker;
use crate::scene::SplitCamera;
use crate::settings_ui;
use crate::splat_plugin::{SplatCamera, SplatScene};

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

/// Which player the settings panel is currently configuring (the split-screen P1/P2 selector).
#[derive(Resource, Default)]
pub struct SettingsPlayer(pub usize);

/// Split-screen race phase. Single-player skips straight to `Racing` (free flight, no countdown).
#[derive(Resource, Clone, Copy, PartialEq, Debug)]
pub enum RaceState {
    /// Drones pinned at spawn; waiting for `P` to start the countdown.
    Ready,
    /// 3-2-1 countdown; drones still pinned (attitude adjustable). `remaining` counts down to 0.
    Countdown { remaining: f32 },
    /// Race underway; while `go_flash` > 0 the brief "GO!" banner stays up.
    Racing { go_flash: f32 },
}

impl Default for RaceState {
    fn default() -> Self {
        RaceState::Racing { go_flash: 0.0 }
    }
}

impl RaceState {
    /// Drones are pinned at spawn during `Ready` + `Countdown` (attitude still adjustable).
    fn is_locked(self) -> bool {
        matches!(self, RaceState::Ready | RaceState::Countdown { .. })
    }
}

/// Race countdown length (seconds) and the post-GO banner flash duration.
const COUNTDOWN_SECS: f32 = 3.0;
const GO_FLASH_SECS: f32 = 0.9;

/// Wires the flight systems: drone setup on entering `Playing`, then input → physics → camera each
/// frame while playing.
pub struct FlightPlugin;

impl Plugin for FlightPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SettingsOpen>();
        app.init_resource::<SettingsPlayer>();
        app.init_resource::<RaceState>();
        app.init_resource::<Players>();
        app.add_systems(OnEnter(AppState::Playing), setup_flight);
        app.add_systems(
            Update,
            (settings_toggle_system, race_system, drone_input_system, drone_camera_system)
                .chain()
                .run_if(in_state(AppState::Playing)),
        );
        // HUD + countdown overlay + settings panel + pause curtain draw in the egui pass.
        app.add_systems(
            EguiPrimaryContextPass,
            (
                hud_system,
                race_overlay_system,
                race_finish_overlay_system,
                settings_ui_system,
                pause_curtain_system,
            )
                .run_if(in_state(AppState::Playing)),
        );
    }
}

/// `OnEnter(Playing)`: build the player drone(s) from the scene config and hide the placement
/// marker. The orbit cameras persist; `placement_orbit_system` stops (run_if Placement) and
/// `drone_camera_system` now drives their transforms as FPV cameras (one per `SplitCamera`).
///
/// SinglePlayer = 1 drone (disarmed). SplitScreen = 2 drones started abreast — offset left (P1) and
/// right (P2) along the heading's perpendicular so they're side by side and equidistant to a gate
/// dead ahead — both armed at start.
fn setup_flight(
    mut commands: Commands,
    config: Res<CurrentSceneConfig>,
    mode: Res<GameMode>,
    markers: Query<Entity, With<SpawnMarker>>,
) {
    let s = config.0.spawn;
    let make = |offset: Vec3| -> Drone {
        let mut drone = Drone::new();
        drone.world_up = config.0.world_up;
        drone.spawn_heading = config.0.heading_deg;
        persistence::load_drone_settings(&mut drone);
        drone.reset(s[0] + offset.x, s[1] + offset.y, s[2] + offset.z);
        // Stabilized (Drone) mode is far easier to fly on a keyboard than FPV acro; default to it.
        drone.flight_mode = FlightMode::Drone;
        drone
    };

    let players = match *mode {
        GameMode::SinglePlayer => vec![PlayerState {
            drone: make(Vec3::ZERO),
            keys: KeyState::default(),
            armed: false,
        }],
        GameMode::SplitScreen => {
            // Start the drones abreast: offset along the heading's perpendicular ("right") axis so
            // they're side by side and equidistant to a gate dead ahead. P1 = left, P2 = right.
            // right = fwd rotated -90° on the horizontal plane (Zup fwd=(sin h,cos h,0); Colmap
            // fwd=(cos h,0,sin h)).
            let h = config.0.heading_deg.to_radians();
            let right = match config.0.world_up {
                WorldUp::Zup => Vec3::new(h.cos(), -h.sin(), 0.0),
                WorldUp::Colmap => Vec3::new(h.sin(), 0.0, -h.cos()),
            };
            let d = 2.5;
            vec![
                PlayerState { drone: make(-right * d), keys: KeyState::default(), armed: true },
                PlayerState { drone: make(right * d), keys: KeyState::default(), armed: true },
            ]
        }
    };
    info!(
        "[Playing] {} drone(s) spawned at {:?} heading {:.1} ({:?})",
        players.len(), s, config.0.heading_deg, *mode
    );
    commands.insert_resource(Players(players));
    commands.insert_resource(SettingsOpen(false));
    // Split-screen is a race: drones stay pinned at spawn until P starts the 3-2-1 countdown.
    // Single-player is free flight (no countdown / no lock).
    commands.insert_resource(match *mode {
        GameMode::SplitScreen => RaceState::Ready,
        GameMode::SinglePlayer => RaceState::Racing { go_flash: 0.0 },
    });
    // The placement spawn marker is not shown during flight.
    for e in &markers {
        commands.entity(e).despawn();
    }
}

/// `Update` (Playing): keyboard → `DroneInput` → physics step, per player.
/// P1: arrows = roll/pitch, W/S = throttle, A/D = yaw, Space = arm, M = mode.
/// P2 (split): I/K = pitch, J/L = roll, T/G = throttle, F/H = yaw, Enter = arm, N = mode.
/// `R` (or P1's controller reset switch) is a *master* reset: it returns every drone to spawn for a
/// single fair restart — P2 has no separate reset key.
fn drone_input_system(
    time: Res<Time>,
    keys_in: Res<ButtonInput<KeyCode>>,
    mut players: ResMut<Players>,
    mut ctrl: Option<ResMut<ControllerRes>>,
    mut race: ResMut<RaceState>,
    settings_open: Res<SettingsOpen>,
    menu: Res<crate::menu::MenuState>,
    splat: Res<SplatScene>,
) {
    if players.0.is_empty() {
        return;
    }
    // Pause physics while the settings panel or exit dialog is open: the drone freezes in place
    // (matches native's `paused = settings_open || show_exit`). HID polling/listen continues in
    // `input_plugin::hid_poll_system`, so channel mapping still works while the panel is up.
    if settings_open.0 || menu.show_exit {
        return;
    }
    let dt = time.delta_secs();
    let octree = splat.collision_octree.as_ref();
    // During Ready/Countdown the drones are pinned at spawn (attitude still adjustable).
    let locked = race.is_locked();

    // --- Player 1: keyboard (WASD/arrows + Space/R/M) + optional HID controller index 0 ---
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
        // Keyboard arm/reset/mode stay available even when a controller is connected.
        if keys_in.just_pressed(KeyCode::Space) {
            p.armed = !p.armed;
            info!("[P1] armed = {}", p.armed);
        }
        if keys_in.just_pressed(KeyCode::KeyM) {
            p.drone.flight_mode = toggle_mode(p.drone.flight_mode);
            info!("[P1] flight mode = {:?}", p.drone.flight_mode);
        }
        let kbd = p.keys.to_input(p.armed);
        let c = ctrl.as_deref_mut().map(|c| &mut c.0[0]);
        drive_player(p, c, kbd, "P1", dt, octree, locked);
    }

    // --- Player 2 (split-screen only): keyboard IJKL/TGFH + Enter/N, or HID index 1 ---
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
        if keys_in.just_pressed(KeyCode::KeyN) {
            p.drone.flight_mode = toggle_mode(p.drone.flight_mode);
            info!("[P2] flight mode = {:?}", p.drone.flight_mode);
        }
        let kbd = p.keys.to_input(p.armed);
        let c = ctrl.as_deref_mut().map(|c| &mut c.0[1]);
        drive_player(p, c, kbd, "P2", dt, octree, locked);
    }

    // Master reset: R (keyboard) or P1's controller reset switch returns EVERY drone to spawn — one
    // fair restart for the whole race. P2 has no independent reset.
    let p1_hid_reset = ctrl.as_ref().is_some_and(|c| c.0[0].reset_triggered);
    if keys_in.just_pressed(KeyCode::KeyR) || p1_hid_reset {
        for p in players.0.iter_mut() {
            p.drone.reset_to_spawn();
        }
        // Split-screen: a reset returns to the start line, ready for a fresh countdown.
        if players.0.len() > 1 {
            *race = RaceState::Ready;
        }
        info!("[Reset] all drones reset to spawn");
    }
}

/// Drive one player's drone for this frame: a connected HID controller (`ctrl`) supplies sticks +
/// arm/mode; otherwise the keyboard fallback `kbd` is used. Steps physics, then either pins the
/// drone at spawn (`locked`, during the countdown) or runs collision. (Reset is a master action
/// handled by `drone_input_system`, not per-player.)
fn drive_player(
    p: &mut PlayerState,
    ctrl: Option<&mut crate::input::Controller>,
    kbd: DroneInput,
    label: &str,
    dt: f32,
    octree: Option<&crate::collision::Octree>,
    locked: bool,
) {
    let input = match ctrl {
        Some(c) if c.hid_connected => {
            c.current_mode = mode_idx(p.drone.flight_mode);
            c.armed = p.armed; // sync keyboard/arm state into the controller
            let mut hid = c.poll_input();
            if c.mode_switch_triggered {
                p.drone.flight_mode = toggle_mode(p.drone.flight_mode);
                info!("[{}] flight mode = {:?} (HID)", label, p.drone.flight_mode);
            }
            p.armed = c.armed; // arm switch may have toggled it
            hid.armed = p.armed;
            hid
        }
        _ => kbd,
    };
    p.drone.update(dt, &input);
    if locked {
        // Countdown: hold the drone on the start line; the attitude from `update` is kept.
        p.drone.lock_position();
    } else if let Some(octree) = octree {
        check_collision(&mut p.drone, octree);
    }
}

/// Point-cloud collision for one drone after its physics step (mirrors native's check_collision):
/// query the octree at the drone position, then push out + bounce-reflect, or clear if no contact.
fn check_collision(drone: &mut Drone, octree: &crate::collision::Octree) {
    let hits = octree.query_sphere(drone.x, drone.y, drone.z, drone.collision_radius);
    match crate::collision::compute_collision_response(
        drone.x, drone.y, drone.z, drone.collision_radius, &hits,
    ) {
        Some(r) => drone.apply_collision(r.normal, r.penetration),
        None => drone.clear_collision(),
    }
}

fn toggle_mode(m: FlightMode) -> FlightMode {
    match m {
        FlightMode::Fpv => FlightMode::Drone,
        FlightMode::Drone => FlightMode::Fpv,
    }
}

/// Controller mode index: `0 = FPV`, `1 = Drone` (matches `Controller::mode_expo/rate` layout).
fn mode_idx(m: FlightMode) -> usize {
    match m {
        FlightMode::Fpv => 0,
        FlightMode::Drone => 1,
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
            if let Some(c) = course.players.first() {
                hud::draw_race(ctx, c, None, None);
            }
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
        // Race panel per half — each player's own course (independent timing + gate count).
        if let Some(course) = &course {
            for (i, c) in course.players.iter().enumerate().take(2) {
                let min_y = if i == 0 { 0.0 } else { half };
                let rect = egui::Rect::from_min_size(egui::pos2(0.0, min_y), egui::vec2(w, half));
                hud::draw_race(ctx, c, Some(labels[i]), Some(rect));
            }
        }
    }
    Ok(())
}

/// `Update` (Playing): F1 toggles the settings panel (native uses F1). Esc closes the panel if open
/// (`menu::handle_esc`), else opens the exit dialog; both pause flight. Close with F1 / Esc / `[x]`.
fn settings_toggle_system(keys: Res<ButtonInput<KeyCode>>, mut settings_open: ResMut<SettingsOpen>) {
    if keys.just_pressed(KeyCode::F1) {
        settings_open.0 = !settings_open.0;
    }
}

/// `Update` (Playing): drives the split-screen race phase. `P` (while `Ready`) snaps both drones to
/// the start line, clears the gate trackers, and begins the 3-2-1 countdown; at GO the linear race
/// clock starts (per-player) and a brief banner flashes. Single-player stays in `Racing` (free
/// flight) so `P` is a no-op. Frozen while paused.
fn race_system(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut race: ResMut<RaceState>,
    mut players: ResMut<Players>,
    mut course: Option<ResMut<RaceCourse>>,
    mut beep: MessageWriter<crate::audio::CountdownBeep>,
    settings_open: Res<SettingsOpen>,
    menu: Res<crate::menu::MenuState>,
) {
    if settings_open.0 || menu.show_exit {
        return; // paused: hold the countdown where it is
    }
    let dt = time.delta_secs();
    match *race {
        RaceState::Ready => {
            if keys.just_pressed(KeyCode::KeyP) {
                for p in players.0.iter_mut() {
                    p.drone.reset_to_spawn();
                }
                // Clear any prior finish / progress so the new race starts fresh.
                if let Some(course) = course.as_mut() {
                    for c in &mut course.players {
                        c.reset_lap();
                    }
                }
                *race = RaceState::Countdown { remaining: COUNTDOWN_SECS };
                beep.write(crate::audio::CountdownBeep { go: false }); // first pip ("3")
                info!("[Race] countdown started");
            }
        }
        RaceState::Countdown { remaining } => {
            let next = remaining - dt;
            if next <= 0.0 {
                *race = RaceState::Racing { go_flash: GO_FLASH_SECS };
                // Linear (dual) race: the clock starts NOW, at GO — not on the first gate.
                let now_ms = time.elapsed_secs_f64() * 1000.0;
                if let Some(course) = course.as_mut() {
                    for c in &mut course.players {
                        c.start_race_timer(now_ms);
                    }
                }
                beep.write(crate::audio::CountdownBeep { go: true });
                info!("[Race] GO!");
            } else {
                // A new pip each time the displayed integer drops: "2" at 2.0 s, "1" at 1.0 s.
                if next.ceil() < remaining.ceil() {
                    beep.write(crate::audio::CountdownBeep { go: false });
                }
                *race = RaceState::Countdown { remaining: next };
            }
        }
        RaceState::Racing { go_flash } if go_flash > 0.0 => {
            *race = RaceState::Racing { go_flash: (go_flash - dt).max(0.0) };
        }
        RaceState::Racing { .. } => {}
    }
}

/// `EguiPrimaryContextPass` (Playing): big centered countdown / GO! banner in each viewport half,
/// plus a "Press P to start" hint while `Ready`. Nothing draws in single-player (always `Racing`).
fn race_overlay_system(
    mut contexts: EguiContexts,
    race: Res<RaceState>,
    players: Res<Players>,
    windows: Query<&Window>,
) -> Result {
    if players.0.is_empty() {
        return Ok(());
    }
    // Banner text + style for this frame; bail if there's nothing to show (free-flight Racing).
    let (text, color, size) = match *race {
        RaceState::Ready => ("Press  P  to start".to_string(), egui::Color32::WHITE, 30.0),
        RaceState::Countdown { remaining } => (
            (remaining.ceil().max(1.0) as i32).to_string(),
            egui::Color32::from_rgb(255, 215, 70),
            150.0,
        ),
        RaceState::Racing { go_flash } if go_flash > 0.0 => {
            ("GO!".to_string(), egui::Color32::from_rgb(90, 230, 120), 150.0)
        }
        RaceState::Racing { .. } => return Ok(()),
    };
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let Ok(win) = windows.single() else {
        return Ok(());
    };
    // scale_factor_override(1.0) → egui logical points == physical px == the camera viewports.
    let (w, h) = (win.width(), win.height());
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("race_overlay"),
    ));
    // Split-screen: one banner centered in each half. Single-player would be one centered banner,
    // but it never leaves free-flight `Racing` so only the split path is reached in practice.
    if players.0.len() <= 1 {
        painter.text(
            egui::pos2(w * 0.5, h * 0.5),
            egui::Align2::CENTER_CENTER,
            &text,
            egui::FontId::proportional(size),
            color,
        );
    } else {
        let half = h * 0.5;
        for i in 0..2 {
            painter.text(
                egui::pos2(w * 0.5, half * i as f32 + half * 0.5),
                egui::Align2::CENTER_CENTER,
                &text,
                egui::FontId::proportional(size),
                color,
            );
        }
    }
    Ok(())
}

/// `EguiPrimaryContextPass` (Playing): once a split-screen pilot crosses the final gate (linear
/// race), cover their viewport half with a translucent black curtain + "FINISH!", their place
/// (No.1 = gold / No.2 = silver) and time, plus a "press R to reset" hint. The other half stays
/// clear so that pilot keeps racing. Single-player (lap mode) never finishes → nothing draws.
fn race_finish_overlay_system(
    mut contexts: EguiContexts,
    race: Res<RaceState>,
    players: Res<Players>,
    course: Option<Res<RaceCourse>>,
    windows: Query<&Window>,
) -> Result {
    // Only during the race proper (Ready/Countdown have cleared finishes); split-screen only.
    if !matches!(*race, RaceState::Racing { .. }) || players.0.len() <= 1 {
        return Ok(());
    }
    let Some(course) = course else {
        return Ok(());
    };
    if !course.players.iter().any(|c| c.finish_rank.is_some()) {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let Ok(win) = windows.single() else {
        return Ok(());
    };
    let (w, h) = (win.width(), win.height());
    let half = h * 0.5;
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("race_finish"),
    ));
    for (i, c) in course.players.iter().enumerate().take(2) {
        let Some(rank) = c.finish_rank else {
            continue;
        };
        let top = half * i as f32;
        let rect = egui::Rect::from_min_size(egui::pos2(0.0, top), egui::vec2(w, half));
        // Translucent black curtain over just this half (like the pause curtain).
        painter.rect_filled(rect, 0.0, egui::Color32::from_black_alpha(190));
        let cx = w * 0.5;
        let cy = top + half * 0.5;
        let place_col = if rank == 1 {
            egui::Color32::from_rgb(255, 215, 70)
        } else {
            egui::Color32::from_rgb(205, 205, 215)
        };
        painter.text(
            egui::pos2(cx, cy - 48.0),
            egui::Align2::CENTER_CENTER,
            "FINISH!",
            egui::FontId::proportional(38.0),
            egui::Color32::WHITE,
        );
        painter.text(
            egui::pos2(cx, cy + 6.0),
            egui::Align2::CENTER_CENTER,
            format!("No. {}", rank),
            egui::FontId::proportional(60.0),
            place_col,
        );
        painter.text(
            egui::pos2(cx, cy + 48.0),
            egui::Align2::CENTER_CENTER,
            gates::format_lap(c.current_lap_ms),
            egui::FontId::proportional(22.0),
            egui::Color32::from_rgb(77, 220, 255),
        );
        painter.text(
            egui::pos2(cx, cy + 80.0),
            egui::Align2::CENTER_CENTER,
            "press  R  to reset",
            egui::FontId::proportional(14.0),
            egui::Color32::from_gray(170),
        );
    }
    Ok(())
}

/// `EguiPrimaryContextPass` (Playing): draw the settings panel (drone physics/rates/PID) when open.
/// Param edits persist per-section inside `settings_ui::draw_settings`.
fn settings_ui_system(
    mut contexts: EguiContexts,
    mut settings_open: ResMut<SettingsOpen>,
    mut settings_player: ResMut<SettingsPlayer>,
    mut players: ResMut<Players>,
    mut ctrl: ResMut<ControllerRes>,
    mut conns: ResMut<HidConnections>,
    mut devices: ResMut<HidDevices>,
    mut audio: ResMut<crate::audio::AudioSettings>,
    mode: Res<GameMode>,
) -> Result {
    if !settings_open.0 || players.0.is_empty() {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let num = players.0.len();
    let mut sel = settings_player.0.min(num - 1);
    // `idx` = the HID-section player (per-player controller); the upper drone params edit P1's drone
    // (index 0) as the shared config. `&mut sel` is the selector the HID section may change.
    let idx = sel;
    // Clone the name so the immutable borrow of `conns` ends before the action handler mutates it.
    let connected = conns.0[idx].as_ref().map(|c| c.product_name.clone());
    let action = settings_ui::draw_settings(
        ctx,
        &mut settings_open.0,
        &mut sel,
        num,
        &mut players.0[0].drone,
        &mut audio,
        &mut ctrl.0[idx],
        &devices.0,
        connected.as_deref(),
    );
    settings_player.0 = sel.min(num - 1);
    let sel = settings_player.0;
    match action {
        Some(settings_ui::HidUiAction::Scan) => {
            // RC-only list: mice/keyboards are filtered out so a pointing device can't be bound.
            devices.0 = crate::input::list_rc_devices();
            info!("[HID] scan found {} RC device(s)", devices.0.len());
        }
        Some(settings_ui::HidUiAction::Connect(idx)) => {
            if let Some(d) = devices.0.get(idx) {
                let (path, name) = (d.path.clone(), d.product_name.clone());
                // `num` = active players in this mode: in single-player a transmitter left bound to
                // the inactive P2 slot is freed + rebound to the player. connect_hid loads the
                // device's saved config (by name) so its mapping/calibration take effect.
                match input_plugin::connect_hid(&mut conns, &mut ctrl.0[sel], sel, num, path, name.clone()) {
                    Ok(()) => {
                        // Remember this transmitter as this mode+slot's last-used (auto-reconnect).
                        persistence::save_last_device(input_plugin::slot_key(*mode, sel), &name);
                    }
                    Err(e) => warn!("[HID] connect failed: {}", e),
                }
            }
        }
        Some(settings_ui::HidUiAction::Disconnect) => {
            input_plugin::disconnect_hid(&mut conns, &mut ctrl.0[sel], sel);
            // Forget the last-used device for this slot so it does not auto-reconnect next time.
            persistence::clear_last_device(input_plugin::slot_key(*mode, sel));
        }
        None => {}
    }
    // The upper sections edit P1's drone as the single shared config; mirror those tunable settings
    // onto every other player so all drones fly identically (only the HID config differs per player).
    if num > 1 {
        if let Some((p1, rest)) = players.0.split_first_mut() {
            for p in rest {
                p.drone.copy_settings_from(&p1.drone);
            }
        }
    }
    Ok(())
}

/// `EguiPrimaryContextPass` (Playing): when the settings panel or exit dialog is open, paint a
/// full-screen dim curtain on the `Foreground` layer — above the HUD (`Middle`) but below the panel
/// / exit dialog (both `Tooltip`). Matches native's `Color32::from_black_alpha(230)` pause overlay.
fn pause_curtain_system(
    mut contexts: EguiContexts,
    settings_open: Res<SettingsOpen>,
    menu: Res<crate::menu::MenuState>,
) -> Result {
    if !(settings_open.0 || menu.show_exit) {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let screen = ctx.viewport_rect();
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("pause_dim"),
    ));
    painter.rect_filled(screen, 0.0, egui::Color32::from_black_alpha(230));
    Ok(())
}
