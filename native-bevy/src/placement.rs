//! Placement phase: build the scene, orbit a world-up-aware camera around the spawn point, edit the
//! spawn/heading (WASD/QE + overlay text fields), then Start Flight / Back. The orbit math is ported
//! verbatim from native `placement.rs` (`compute_orbit_camera` / `update_spawn`); the resulting
//! web-splat camera is converted to a Bevy transform with the same `* flip_x` the splat node applies.

use bevy::camera::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass};
use cgmath::{InnerSpace, Rotation};
use std::path::Path;

use crate::app_state::{AppState, CurrentSceneConfig, GameMode, SceneConfig, WorldUp};
use crate::menu::MenuState;
use crate::persistence;
use crate::placement_ui::{self, PlacementAction, PlacementUiState};
use crate::scene::{SceneEntity, SplitCamera};
use crate::splat_plugin::{self, SplatCamera};
use crate::SceneInput;

/// Marks the placement spawn marker (bright sphere + heading arrow) so `placement_update` can keep
/// it on the configured spawn point + heading. Despawned by `flight::setup_flight`.
#[derive(Component)]
pub(crate) struct SpawnMarker;

/// Set by the placement overlay each frame: whether egui currently wants keyboard input (a text
/// field is focused → suppress WASD spawn movement) and/or pointer input (cursor over the panel →
/// suppress mouse orbit).
#[derive(Resource, Default)]
struct PlacementUiFocus {
    wants_keyboard: bool,
    wants_pointer: bool,
}

/// Orbit camera state for the placement phase (ported from native `placement::OrbitState`).
/// `yaw`/`pitch` in degrees; `dist` is the distance from the spawn point. The orbit is world-up
/// aware (see `compute_orbit_camera`), unlike `bevy_panorbit_camera` which assumes Y-up.
#[derive(Resource)]
struct OrbitState {
    yaw: f32,
    pitch: f32,
    dist: f32,
}

impl Default for OrbitState {
    fn default() -> Self {
        Self { yaw: 0.0, pitch: -20.0, dist: 45.0 }
    }
}

/// Wires the placement phase: scene/config/orbit setup on enter, orbit + spawn editing each frame,
/// and the egui overlay.
pub struct PlacementPlugin;

impl Plugin for PlacementPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PlacementUiFocus>();
        // init_scene_config loads/persists the per-scene config; init_placement_orbit seeds the
        // orbit so setup_scene can place the camera correctly.
        app.add_systems(
            OnEnter(AppState::Placement),
            (init_scene_config, init_placement_orbit, setup_scene).chain(),
        );
        app.add_systems(
            Update,
            (placement_orbit_system, placement_update)
                .chain()
                .run_if(in_state(AppState::Placement)),
        );
        app.add_systems(
            EguiPrimaryContextPass,
            placement_overlay_system.run_if(in_state(AppState::Placement)),
        );
    }
}

/// `OnEnter(Placement)`: load this scene's persisted config (`world_up`/`spawn`/`heading`), apply
/// any `--world-up` CLI override, store it as `CurrentSceneConfig`, and write it back (a round-trip
/// that also creates `scenes.json` on first run). Placement + flight read this resource.
fn init_scene_config(
    mut commands: Commands,
    scene_input: Res<SceneInput>,
    splat_scene: Res<splat_plugin::SplatScene>,
) {
    let mut config = match &scene_input.path {
        Some(p) => persistence::load_scene_config(&persistence::scene_key(Path::new(p))),
        None => SceneConfig::default(),
    };
    if let Some(wu) = scene_input.world_up_override {
        config.world_up = wu;
    }
    if let Some(p) = &scene_input.path {
        if let Err(e) = persistence::save_scene_config(&persistence::scene_key(Path::new(p)), &config) {
            warn!("[Placement] failed to save scene config: {e}");
        }
    }
    // Anchor spawn to the scene center on first use (no saved spawn). Bevy world coords == PLY
    // coords for positions (the splat node passes camera position through unchanged), so the
    // point-cloud center is directly usable as a Bevy-space spawn / orbit focus.
    if config.spawn == SceneConfig::default().spawn {
        if let Some(c) = splat_scene.scene_center {
            config.spawn = c;
        }
    }
    info!(
        "[Placement] scene config: world_up={:?} spawn={:?} heading={:.1} deg",
        config.world_up, config.spawn, config.heading_deg
    );
    commands.insert_resource(PlacementUiState::from_config(&config));
    commands.insert_resource(CurrentSceneConfig(config));
}

/// `OnEnter(Placement)` (after `init_scene_config`): seed the orbit from the persisted heading so
/// the very first frame — including `setup_scene`'s camera spawn — is already aimed correctly.
fn init_placement_orbit(mut commands: Commands, config: Res<CurrentSceneConfig>) {
    commands.insert_resource(OrbitState { yaw: config.0.heading_deg, ..default() });
}

/// `OnEnter(Placement)`: build the scene (demo PBR objects + lighting + spawn marker) and spawn the
/// camera(s) for the current `GameMode` (SinglePlayer = one full-window camera; SplitScreen = two
/// stacked halves).
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mode: Res<GameMode>,
    config: Res<CurrentSceneConfig>,
    orbit: Res<OrbitState>,
) {
    // Orbit focus + demo-object anchor = the configured spawn point (scene center on first use).
    let focus = Vec3::from(config.0.spawn);

    // --- PBR 3D objects placed inside the gaussian splat scene ---
    let cube_mesh = meshes.add(Cuboid::new(8.0, 8.0, 8.0));
    let sphere_mesh = meshes.add(Sphere::new(5.0));

    // Red cube offset along +Y (kept off the spawn point so the drone doesn't start inside it)
    commands.spawn((
        Mesh3d(cube_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.15, 0.15),
            perceptual_roughness: 0.5,
            ..default()
        })),
        Transform::from_translation(focus + Vec3::new(0.0, 16.0, 0.0)),
        SceneEntity,
    ));
    // Blue sphere offset along +X
    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.45, 0.95),
            metallic: 0.3,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(focus + Vec3::new(16.0, 0.0, 0.0)),
        SceneEntity,
    ));
    // Green cube offset along -X
    commands.spawn((
        Mesh3d(cube_mesh),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.85, 0.3),
            perceptual_roughness: 0.6,
            ..default()
        })),
        Transform::from_translation(focus + Vec3::new(-16.0, 0.0, 0.0)),
        SceneEntity,
    ));

    // --- Lighting for the PBR meshes ---
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_translation(focus + Vec3::new(30.0, 60.0, 40.0)).looking_at(focus, Vec3::Z),
        SceneEntity,
    ));

    // --- Spawn marker: bright unlit sphere + forward arrow at the configured spawn point. ---
    // `placement_update` moves/rotates this each frame to follow `spawn` + `heading`, so editing
    // the spawn (text fields or WASD) is visible; the orbit camera also recenters on `spawn`.
    let marker_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.85, 0.1),
        emissive: LinearRgba::rgb(1.5, 1.2, 0.0),
        unlit: true,
        ..default()
    });
    commands
        .spawn((
            Transform::from_translation(focus),
            Visibility::default(),
            SpawnMarker,
            SceneEntity,
        ))
        .with_children(|p| {
            p.spawn((
                Mesh3d(meshes.add(Sphere::new(1.5))),
                MeshMaterial3d(marker_mat.clone()),
                Transform::default(),
            ));
            // Arrow modeled along +Y; the parent rotation aims it along the heading direction.
            p.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.4, 6.0, 0.4))),
                MeshMaterial3d(marker_mat),
                Transform::from_xyz(0.0, 3.5, 0.0),
            ));
        });

    // --- Camera(s): each renders the full scene (splat + PBR meshes). In split mode the two stacked
    // cameras share the window target and `scene::set_split_viewports` assigns their viewport rects.
    // The placement camera(s) start aimed by `placement_orbit_system`; `init_placement_orbit` seeded
    // `OrbitState`, so compute the initial transform now.
    //
    // No MSAA: the splat rasterize uses a single-sample intermediate + the single-sample mesh depth
    // buffer; MSAA would multisample depth and mismatch the splat pass.
    let cam_xf = orbit_camera_transform(&config.0, &orbit);
    let cams: &[(u8, isize)] = match *mode {
        GameMode::SinglePlayer => &[(0, 0)],
        GameMode::SplitScreen => &[(0, 0), (1, 1)],
    };
    for &(index, order) in cams {
        let mut cam = commands.spawn((
            Camera3d::default(),
            Camera {
                order,
                // Only the first camera (order 0) clears the shared window; later split-screen
                // cameras must LOAD, else each re-clears the whole window and erases the previous
                // camera's viewport (which blacked out the top half in split-screen).
                clear_color: if order == 0 {
                    ClearColorConfig::Default
                } else {
                    ClearColorConfig::None
                },
                ..default()
            },
            Tonemapping::None,
            Msaa::Off,
            AmbientLight {
                color: Color::WHITE,
                brightness: 400.0,
                ..default()
            },
            cam_xf,
            SplatCamera,
            SceneEntity,
        ));
        // Only split-screen cameras get a viewport rect; the single-player camera stays full-window.
        if matches!(*mode, GameMode::SplitScreen) {
            cam.insert(SplitCamera { index });
        }
    }

    info!("[Placement] scene ready ({:?}). Orbit: Left-drag=orbit, Scroll=zoom, WASD/QE=move spawn", *mode);
}

/// `Update` while in `Placement` (runs first): mouse-drag orbits / scroll zooms the camera around
/// the spawn point, world-up aware (ported from native `placement::compute_orbit_camera`). The
/// orbit yaw drives `heading_deg`; an external heading edit (the overlay "Set" button) snaps yaw
/// back. The resulting web-splat camera is converted to a Bevy transform on every `SplatCamera`.
fn placement_orbit_system(
    ui_focus: Res<PlacementUiFocus>,
    menu: Res<MenuState>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    mut orbit: ResMut<OrbitState>,
    mut config: ResMut<CurrentSceneConfig>,
    mut cams: Query<&mut Transform, With<SplatCamera>>,
) {
    let interacting = !ui_focus.wants_pointer && !menu.show_exit;
    // Left-drag → orbit (native: yaw += dx*0.3, pitch -= dy*0.3, clamped).
    let mut dragged = false;
    if interacting && mouse_buttons.pressed(MouseButton::Left) {
        let d = mouse_motion.delta;
        if d != Vec2::ZERO {
            orbit.yaw += d.x * 0.3;
            orbit.pitch = (orbit.pitch - d.y * 0.3).clamp(-89.0, 89.0);
            dragged = true;
        }
    }
    // Scroll → zoom (native: dist *= 1 - dy*0.1).
    if interacting {
        let dz = mouse_scroll.delta.y.clamp(-5.0, 5.0);
        if dz != 0.0 {
            orbit.dist = (orbit.dist * (1.0 - dz * 0.1)).clamp(1.0, 1000.0);
        }
    }
    // Heading ↔ yaw link: dragging writes heading; otherwise honor external heading edits.
    let wrapped = ((orbit.yaw % 360.0) + 360.0) % 360.0;
    if dragged {
        config.0.heading_deg = wrapped;
    } else if (config.0.heading_deg - wrapped).abs() > 0.01 {
        orbit.yaw = config.0.heading_deg;
    }
    let xf = orbit_camera_transform(&config.0, &orbit);
    for mut t in &mut cams {
        *t = xf;
    }
}

/// `Update` while in `Placement` (after `placement_orbit_system`): WASD/QE move the spawn point
/// relative to the orbit yaw (ported from native `placement::update_spawn`), and the spawn marker
/// is kept on the spawn point + heading so any edit (text fields or WASD) is immediately visible.
fn placement_update(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    ui_focus: Res<PlacementUiFocus>,
    menu: Res<MenuState>,
    orbit: Res<OrbitState>,
    mut config: ResMut<CurrentSceneConfig>,
    mut markers: Query<&mut Transform, With<SpawnMarker>>,
) {
    if !ui_focus.wants_keyboard && !menu.show_exit {
        update_spawn(&mut config.0, &orbit, time.delta_secs(), &keys);
    }
    // Keep the marker (position + heading) on the spawn point.
    let spawn = Vec3::from(config.0.spawn);
    let h = config.0.heading_deg.to_radians();
    let fwd_dir = match config.0.world_up {
        WorldUp::Zup => Vec3::new(-h.sin(), h.cos(), 0.0),
        WorldUp::Colmap => Vec3::new(h.cos(), 0.0, h.sin()),
    };
    let rot = Quat::from_rotation_arc(Vec3::Y, fwd_dir);
    for mut t in &mut markers {
        t.translation = spawn;
        t.rotation = rot;
    }
}

/// Move the spawn point with WASD (yaw-relative horizontal) / QE (world-up axis), ported from
/// native `placement::update_spawn` (Shift = faster is a native-bevy QoL addition).
fn update_spawn(config: &mut SceneConfig, orbit: &OrbitState, dt: f32, keys: &ButtonInput<KeyCode>) {
    let mut s = 8.0 * dt;
    if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        s *= 5.0;
    }
    let yaw = orbit.yaw.to_radians();
    let (sin_y, cos_y) = (yaw.sin(), yaw.cos());
    match config.world_up {
        WorldUp::Zup => {
            if keys.pressed(KeyCode::KeyW) { config.spawn[0] -= sin_y * s; config.spawn[1] += cos_y * s; }
            if keys.pressed(KeyCode::KeyS) { config.spawn[0] += sin_y * s; config.spawn[1] -= cos_y * s; }
            if keys.pressed(KeyCode::KeyA) { config.spawn[0] -= cos_y * s; config.spawn[1] -= sin_y * s; }
            if keys.pressed(KeyCode::KeyD) { config.spawn[0] += cos_y * s; config.spawn[1] += sin_y * s; }
            if keys.pressed(KeyCode::KeyE) { config.spawn[2] += s; }
            if keys.pressed(KeyCode::KeyQ) { config.spawn[2] -= s; }
        }
        WorldUp::Colmap => {
            if keys.pressed(KeyCode::KeyW) { config.spawn[0] += cos_y * s; config.spawn[2] += sin_y * s; }
            if keys.pressed(KeyCode::KeyS) { config.spawn[0] -= cos_y * s; config.spawn[2] -= sin_y * s; }
            if keys.pressed(KeyCode::KeyD) { config.spawn[0] += sin_y * s; config.spawn[2] -= cos_y * s; }
            if keys.pressed(KeyCode::KeyA) { config.spawn[0] -= sin_y * s; config.spawn[2] += cos_y * s; }
            if keys.pressed(KeyCode::KeyE) { config.spawn[1] -= s; }
            if keys.pressed(KeyCode::KeyQ) { config.spawn[1] += s; }
        }
    }
}

/// Orbit camera position + rotation in web-splat convention, ported verbatim from native
/// `placement::compute_orbit_camera`. World-up aware: Z-up uses a Z-vertical orbit, COLMAP a
/// Y-vertical (Y-down) orbit. The caller converts to a Bevy transform.
fn compute_orbit_camera(
    config: &SceneConfig,
    orbit: &OrbitState,
) -> (cgmath::Point3<f32>, cgmath::Quaternion<f32>) {
    use cgmath::{Point3, Vector3};
    let sp = config.spawn;
    let yaw = orbit.yaw.to_radians();
    let pitch = orbit.pitch.to_radians();
    let d = orbit.dist;
    let (cos_p, sin_p) = (pitch.cos(), pitch.sin());
    let cam_offset = match config.world_up {
        WorldUp::Zup => Vector3::new(-yaw.sin() * cos_p * d, -yaw.cos() * cos_p * d, -sin_p * d),
        WorldUp::Colmap => Vector3::new(-yaw.cos() * cos_p * d, sin_p * d, -yaw.sin() * cos_p * d),
    };
    let cam_pos = Point3::new(sp[0] + cam_offset.x, sp[1] + cam_offset.y, sp[2] + cam_offset.z);
    let look_dir =
        Vector3::new(sp[0] - cam_pos.x, sp[1] - cam_pos.y, sp[2] - cam_pos.z).normalize();
    let up = match config.world_up {
        WorldUp::Zup => Vector3::new(0.0, 0.0, -1.0),
        WorldUp::Colmap => Vector3::new(0.0, 1.0, 0.0),
    };
    (cam_pos, cgmath::Quaternion::look_at(look_dir, up))
}

/// Bevy transform for the placement orbit: `compute_orbit_camera` (web-splat convention) converted
/// via the same `* flip_x` (180° about X) the splat node applies, matching `flight::drone_camera_system`.
fn orbit_camera_transform(config: &SceneConfig, orbit: &OrbitState) -> Transform {
    let (pos, rot) = compute_orbit_camera(config, orbit);
    let flip_x = cgmath::Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let q = rot * flip_x;
    Transform {
        translation: Vec3::new(pos.x, pos.y, pos.z),
        rotation: Quat::from_xyzw(q.v.x, q.v.y, q.v.z, q.s),
        scale: Vec3::ONE,
    }
}

/// Placement overlay: edit spawn / heading / world-up, then Start Flight (Enter) or Back (Esc).
fn placement_overlay_system(
    mut contexts: EguiContexts,
    mut config: ResMut<CurrentSceneConfig>,
    mut ui_state: ResMut<PlacementUiState>,
    mut ui_focus: ResMut<PlacementUiFocus>,
    scene_input: Res<SceneInput>,
    menu: Res<MenuState>,
    keys: Res<ButtonInput<KeyCode>>,
    mut next: ResMut<NextState<AppState>>,
) -> Result {
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let wants_kb = ctx.wants_keyboard_input();
    ui_focus.wants_keyboard = wants_kb;
    ui_focus.wants_pointer = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
    // Generous bounds for now; scene-AABB clamping ports with the spawn-marker/drone slice.
    let b = 1.0e4_f32;
    let mut action =
        placement_ui::draw_overlay(ctx, &mut config.0, &mut ui_state, [-b, -b, -b], [b, b, b]);
    // Enter = Start Flight (unless a text field is focused). Esc is handled by `menu::handle_esc`.
    if matches!(action, PlacementAction::None) && !wants_kb && keys.just_pressed(KeyCode::Enter) {
        action = PlacementAction::StartFlight;
    }
    // Ignore overlay interaction while the exit dialog is up (it draws on top).
    if menu.show_exit {
        return Ok(());
    }
    match action {
        PlacementAction::StartFlight => {
            if let Some(p) = &scene_input.path {
                if let Err(e) =
                    persistence::save_scene_config(&persistence::scene_key(Path::new(p)), &config.0)
                {
                    warn!("[Placement] failed to save scene config: {e}");
                }
            }
            info!("[Placement] Start Flight -> Playing");
            next.set(AppState::Playing);
        }
        PlacementAction::GoBack => {
            info!("[Placement] Back -> ModeSelect");
            next.set(AppState::ModeSelect);
        }
        PlacementAction::None => {}
    }
    Ok(())
}
