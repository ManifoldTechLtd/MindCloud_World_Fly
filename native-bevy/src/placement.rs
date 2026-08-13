//! Placement phase: build the scene, orbit a world-up-aware camera around the spawn point, edit the
//! spawn/heading (WASD/QE + overlay text fields), then Start Flight / Back. The orbit math is ported
//! verbatim from native `placement.rs` (`compute_orbit_camera` / `update_spawn`); the resulting
//! web-splat camera is converted to a Bevy transform with the same `* flip_x` the splat node applies.

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::gltf::GltfAssetLabel;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass};
use cgmath::{InnerSpace, Rotation};
use std::path::Path;

use crate::app_state::{AppState, CurrentSceneConfig, GameMode, SceneConfig, WorldUp};
use crate::gate_editor::GateEditor;
use crate::menu::MenuState;
use crate::persistence;
use crate::placement_ui::{self, PlacementAction, PlacementUiState};
use crate::scene::{SceneEntity, SplitCamera};
use crate::splat_plugin::{self, SplatCamera};
use crate::SceneInput;

/// Marks the placement spawn marker (a flat dark-yellow arrow) so `placement_update` can keep it on
/// the configured spawn point + heading. Despawned by `flight::setup_flight`.
#[derive(Component)]
pub(crate) struct SpawnMarker;

/// The drone model asset (glTF) shown at the spawn point and flown in `Playing`. Shared by
/// `placement` + `flight` so both load the same model. NOTE: this Sketchfab DJI FPV model has 0
/// animations (propellers stay static) AND bakes an arbitrary rotation into its glTF root node, so it
/// renders tilted; `DRONE_TILT_FIX` (below) cancels that so it sits level.
pub(crate) const DRONE_MODEL_ASSET: &str = "model/dji_fvp_-_sdc_performance_edition.glb";

/// Uniform scale applied to `DRONE_MODEL_ASSET`. This model spans ~6.3 units, so ~0.08 → ~0.5 m
/// across; bump up/down if it loads too small/large. (Orientation fix = the +90° X rotation
/// applied to the model children below.)
pub(crate) const DRONE_MODEL_SCALE: f32 = 0.1;

/// Per-model orientation correction, right-multiplied onto the drone's world rotation in BOTH
/// placement (`drone_child`) and flight (`drone_model_system`). The DJI FPV glTF bakes an arbitrary
/// rotation `R_root` into its `Sketchfab_model` node (so it renders tilted) AND its body is modelled
/// facing +Z, i.e. 180° off our forward convention. This quaternion is `Ry(180°) · Rx(-90°) · R_root⁻¹`:
/// it cancels the baked rotation, restores the standard Sketchfab Z-up→Y-up orientation (level), then
/// yaws 180° about the up axis so the nose faces forward. Precomputed for THIS model (det(R_root)=1,
/// verified); recompute if `DRONE_MODEL_ASSET` changes.
pub(crate) const DRONE_TILT_FIX: Quat = Quat::from_xyzw(0.176037, 0.976306, 0.061927, 0.109556);

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
        Self { yaw: 0.0, pitch: -20.0, dist: 8.0 }
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
/// camera(s) for the current `GameMode` (SinglePlayer = one full-window camera; DualPlayer = two
/// stacked halves).
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mode: Res<GameMode>,
    game_type: Res<crate::app_state::GameType>,
    config: Res<CurrentSceneConfig>,
    orbit: Res<OrbitState>,
    asset_server: Res<AssetServer>,
) {
    // Orbit focus + demo-object anchor = the configured spawn point (scene center on first use).
    let focus = Vec3::from(config.0.spawn);

    // (Demo PBR cubes/sphere removed: the gate frames are now the in-scene mesh reference, and the
    // old blue demo sphere clashed with the blue spawn marker below.)

    // --- Lighting (kept for any future lit PBR meshes; the marker + gates are unlit). ---
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_translation(focus + Vec3::new(30.0, 60.0, 40.0)).looking_at(focus, Vec3::Z),
        SceneEntity,
    ));

    // --- Spawn marker(s): a transform-only `SpawnMarker` parent driven by `placement_update`
    // (local +Y = heading, +Z = world up, +X = right); `flight::setup_flight` despawns it recursively.
    // Children depend on the mode:
    //   SinglePlayer: just the drone model on the spawn point (no arrow — the drone IS the marker).
    //   DualPlayer:  the flat yellow heading arrow at the centre + a static drone at each player's
    //     start (P1 left / P2 right along local X = the heading's "right", offset ±SPLIT_SPAWN_OFFSET
    //     so they sit exactly where `setup_flight` places the two flight drones).
    let drone_model = asset_server.load(GltfAssetLabel::Scene(0).from_asset(DRONE_MODEL_ASSET));
    // Child transform that stands the glTF model (native +Y up, -Z forward) upright facing the
    // parent's local +Y (heading): +90° about X, then DRONE_TILT_FIX to level this model's baked tilt,
    // scaled by DRONE_MODEL_SCALE; `offset` is along local X, `yaw` spins it about local Z (up) —
    // battle previews face outward (±90°) to mirror the back-to-back duel spawn in `setup_flight`.
    let drone_child = |offset: f32, yaw: f32| {
        Transform::from_xyz(offset, 0.0, 0.0)
            .with_rotation(
                Quat::from_rotation_z(yaw)
                    * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)
                    * DRONE_TILT_FIX,
            )
            .with_scale(Vec3::splat(DRONE_MODEL_SCALE))
    };
    // Heading arrow (split-screen only): a flat unlit double-sided isosceles prism + its mesh.
    let arrow = matches!(*mode, GameMode::DualPlayer).then(|| {
        let mat = materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.95, 0.0), // dark / muted yellow
            unlit: true,
            cull_mode: None,
            ..default()
        });
        (meshes.add(arrow_prism_mesh(1.4, 0.4, 0.12)), mat) // length, width, height
    });
    commands
        .spawn((
            Transform::from_translation(focus),
            Visibility::default(),
            SpawnMarker,
            SceneEntity,
        ))
        .with_children(|p| match *mode {
            GameMode::SinglePlayer => {
                p.spawn((SceneRoot(drone_model.clone()), drone_child(0.0, 0.0)));
            }
            GameMode::DualPlayer => {
                let (mesh, mat) = arrow.clone().unwrap();
                p.spawn((Mesh3d(mesh), MeshMaterial3d(mat), Transform::default()));
                let d = crate::flight::SPLIT_SPAWN_OFFSET;
                // Battle: back-to-back duel preview — P1 faces local -X, P2 local +X (matching
                // setup_flight's outward headings). Race: both face the arrow (+Y, yaw 0).
                let battle = matches!(*game_type, crate::app_state::GameType::Battle);
                let half = std::f32::consts::FRAC_PI_2;
                let (y1, y2) = if battle { (half, -half) } else { (0.0, 0.0) };
                p.spawn((SceneRoot(drone_model.clone()), drone_child(-d, y1))); // P1 (left)
                p.spawn((SceneRoot(drone_model.clone()), drone_child(d, y2))); // P2 (right)
            }
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
        GameMode::DualPlayer => &[(0, 0), (1, 1)],
    };
    for &(index, order) in cams {
        // Layer 0 = shared (splat-driven meshes, the spawn marker). Camera i also sees layer i+1 (its
        // OWN gate-frame set, so each split view pulses its own next gate). In split-screen each camera
        // additionally sees the OTHER player's in-flight drone layer (DRONE_VIS_LAYER_BASE + other), so
        // pilots see the opponent's aircraft but never their own FPV body.
        let mut layer_ids = vec![0usize, index as usize + 1];
        if matches!(*mode, GameMode::DualPlayer) {
            // also see the OTHER player's in-flight drone (player 1-index), never our own FPV body.
            layer_ids.push(crate::flight::DRONE_VIS_LAYER_BASE + (1 - index as usize));
        }
        // In battle mode, also render your OWN drone model (unlike race mode where FPV pilots
        // shouldn't see their own body — battle mode needs visual feedback of the drone), plus
        // the OPPONENT's hit-sphere shell (never your own — the camera sits inside it and the
        // double-sided translucent material would tint the whole view).
        if matches!(*game_type, crate::app_state::GameType::Battle) {
            layer_ids.push(crate::flight::DRONE_VIS_LAYER_BASE + index as usize);
            if matches!(*mode, GameMode::DualPlayer) {
                layer_ids.push(crate::battle_plugin::HIT_SPHERE_LAYER_BASE + (1 - index as usize));
            }
        }
        let render_layers = RenderLayers::from_layers(&layer_ids);
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
            render_layers.clone(),
        ));
        // Only split-screen cameras get a viewport rect; the single-player camera stays full-window.
        if matches!(*mode, GameMode::DualPlayer) {
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
    editor: Option<Res<GateEditor>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    mut orbit: ResMut<OrbitState>,
    mut config: ResMut<CurrentSceneConfig>,
    mut cams: Query<&mut Transform, With<SplatCamera>>,
) {
    let editing = editor.map_or(false, |e| e.active);
    let interacting = !ui_focus.wants_pointer && !menu.show_exit && !editing;
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
    editor: Option<Res<GateEditor>>,
    orbit: Res<OrbitState>,
    mut config: ResMut<CurrentSceneConfig>,
    mut markers: Query<&mut Transform, With<SpawnMarker>>,
) {
    let editing = editor.map_or(false, |e| e.active);
    if !ui_focus.wants_keyboard && !menu.show_exit && !editing {
        update_spawn(&mut config.0, &orbit, time.delta_secs(), &keys);
    }
    // Keep the marker (position + heading) on the spawn point. Build an orthonormal basis that maps
    // local +Y → the horizontal heading (= the drone spawn forward, matching the orbit camera's look
    // direction) and local +Z → world up, so the flat triangular prism lies on the ground pointing
    // where the drone will face. Zup heading vector is (sin h, cos h, 0) (CW from above), matching
    // compute_orbit_camera's look_dir; Colmap up is -Y (gravity +Y).
    let spawn = Vec3::from(config.0.spawn);
    let h = config.0.heading_deg.to_radians();
    let (up, fwd) = match config.0.world_up {
        WorldUp::Zup => (Vec3::Z, Vec3::new(h.sin(), h.cos(), 0.0)),
        WorldUp::Colmap => (-Vec3::Y, Vec3::new(h.cos(), 0.0, h.sin())),
    };
    let right = fwd.cross(up).normalize();
    let rot = Quat::from_mat3(&Mat3::from_cols(right, fwd, up));
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
            // Forward (W) = camera horizontal look dir (sin y, cos y); Right (D) = forward × up(+Z)
            // = (cos y, -sin y). The old code negated the sin terms (mirroring the yaw), so W moved
            // backwards and A/D were swapped at non-zero yaw. (COLMAP below was already correct.)
            if keys.pressed(KeyCode::KeyW) { config.spawn[0] += sin_y * s; config.spawn[1] += cos_y * s; }
            if keys.pressed(KeyCode::KeyS) { config.spawn[0] -= sin_y * s; config.spawn[1] -= cos_y * s; }
            if keys.pressed(KeyCode::KeyA) { config.spawn[0] -= cos_y * s; config.spawn[1] += sin_y * s; }
            if keys.pressed(KeyCode::KeyD) { config.spawn[0] += cos_y * s; config.spawn[1] -= sin_y * s; }
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

/// Bevy transform for the placement orbit. `compute_orbit_camera` yields q_wb (web-splat convention);
/// the Bevy camera that renders the SAME view is `q_wb.conjugate() * flip_x` (180° about X), matching
/// `flight::drone_camera_system`. The splat node re-derives `(bevy_q*flip_x).conjugate()` = q_wb, so
/// gates/markers stay locked to the splat. Verified in tests/splat_camera_align.rs.
fn orbit_camera_transform(config: &SceneConfig, orbit: &OrbitState) -> Transform {
    let (pos, rot) = compute_orbit_camera(config, orbit);
    let flip_x = cgmath::Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let q = rot.conjugate() * flip_x;
    Transform {
        translation: Vec3::new(pos.x, pos.y, pos.z),
        rotation: Quat::from_xyzw(q.v.x, q.v.y, q.v.z, q.s),
        scale: Vec3::ONE,
    }
}

/// Build the spawn-arrow mesh: a flat isosceles triangular prism modeled in local XY with the apex
/// toward +Y and flat through local Z, its centroid at the origin (so it stays centred on the spawn
/// point). `length` = apex→base distance (长), `width` = base-edge width (宽), `height` = prism
/// thickness (高) — all independent. Carries per-face vertex colours used as brightness multipliers —
/// the top cap full-bright, the sides dimmer, the underside dimmest — so the unlit dark-yellow
/// material reads as a 3D arrow. The material is double-sided, so triangle winding here is cosmetic.
fn arrow_prism_mesh(length: f32, width: f32, height: f32) -> Mesh {
    // Centroid of a triangle sits 1/3 of the way from base to apex, so place the apex at +2/3·length
    // and the base edge at -1/3·length to keep the centroid (hence the arrow) on the spawn point.
    let (ay, by) = (length * 2.0 / 3.0, -length / 3.0);
    let (hw, hz) = (width * 0.5, height * 0.5);
    // Triangle corners on the top (+Z) and bottom (-Z) caps.
    let apex_t = [0.0, ay, hz];
    let bl_t = [-hw, by, hz];
    let br_t = [hw, by, hz];
    let apex_b = [0.0, ay, -hz];
    let bl_b = [-hw, by, -hz];
    let br_b = [hw, by, -hz];
    const TOP: f32 = 1.0;
    const SIDE: f32 = 0.6;
    const BOTTOM: f32 = 0.45;
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut colors: Vec<[f32; 4]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    // Add a polygon as a triangle fan (handles the 3-vert caps and 4-vert side quads), with one flat
    // normal + brightness `b` baked into every vertex colour.
    let mut add = |verts: &[[f32; 3]], n: [f32; 3], b: f32| {
        let base = positions.len() as u32;
        for &p in verts {
            positions.push(p);
            normals.push(n);
            colors.push([b, b, b, 1.0]);
        }
        for i in 1..(verts.len() as u32 - 1) {
            indices.extend_from_slice(&[base, base + i, base + i + 1]);
        }
    };
    // Top + bottom caps.
    add(&[apex_t, bl_t, br_t], [0.0, 0.0, 1.0], TOP);
    add(&[apex_b, br_b, bl_b], [0.0, 0.0, -1.0], BOTTOM);
    // Three side faces (outward normals approximate; cosmetic only under the double-sided material).
    add(&[apex_t, bl_t, bl_b, apex_b], [-0.866, 0.5, 0.0], SIDE);
    add(&[bl_t, br_t, br_b, bl_b], [0.0, -1.0, 0.0], SIDE);
    add(&[br_t, apex_t, apex_b, br_b], [0.866, 0.5, 0.0], SIDE);
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::RENDER_WORLD)
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
        .with_inserted_indices(Indices::U32(indices))
}

/// Placement overlay: edit spawn / heading / world-up, then Start Flight (Enter) or Back (Esc).
fn placement_overlay_system(
    mut contexts: EguiContexts,
    mut config: ResMut<CurrentSceneConfig>,
    mut ui_state: ResMut<PlacementUiState>,
    mut ui_focus: ResMut<PlacementUiFocus>,
    mut editor: ResMut<GateEditor>,
    scene_input: Res<SceneInput>,
    menu: Res<MenuState>,
    keys: Res<ButtonInput<KeyCode>>,
    mut next: ResMut<NextState<AppState>>,
) -> Result {
    // While the gate editor is open it draws instead of this overlay (and owns Enter/Esc).
    if editor.active {
        return Ok(());
    }
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
    if matches!(action, PlacementAction::EditGates) {
        editor.active = true;
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
        PlacementAction::EditGates | PlacementAction::None => {}
    }
    Ok(())
}

#[cfg(test)]
mod spawn_move_tests {
    use super::*;

    fn cfg(world_up: WorldUp) -> SceneConfig {
        SceneConfig { world_up, spawn: [1.0, 2.0, 3.0], heading_deg: 0.0 }
    }

    /// Horizontal (ground-plane) component of the orbit camera's look direction.
    fn cam_forward_horiz(config: &SceneConfig, orbit: &OrbitState) -> Vec3 {
        let (cam_pos, _q) = compute_orbit_camera(config, orbit);
        let look = Vec3::new(
            config.spawn[0] - cam_pos.x,
            config.spawn[1] - cam_pos.y,
            config.spawn[2] - cam_pos.z,
        );
        match config.world_up {
            WorldUp::Zup => Vec3::new(look.x, look.y, 0.0),
            WorldUp::Colmap => Vec3::new(look.x, 0.0, look.z),
        }
        .normalize()
    }

    /// Normalised spawn displacement after holding `key` for one 60 FPS tick at the given yaw.
    fn moved(world_up: WorldUp, yaw: f32, key: KeyCode) -> Vec3 {
        let mut c = cfg(world_up);
        let before = Vec3::from(c.spawn);
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(key);
        update_spawn(&mut c, &OrbitState { yaw, pitch: -20.0, dist: 8.0 }, 1.0 / 60.0, &keys);
        (Vec3::from(c.spawn) - before).normalize()
    }

    /// W/S/A/D must move the spawn along the orbit camera's basis (kept in sync with the spawn
    /// heading) in BOTH world-ups. The Z-up branch used to mirror the yaw, so W went backwards (and
    /// A/D swapped) at non-zero yaw; COLMAP was already correct.
    #[test]
    fn wasd_moves_relative_to_camera_in_both_world_ups() {
        for &world_up in &[WorldUp::Zup, WorldUp::Colmap] {
            let up = match world_up {
                WorldUp::Zup => Vec3::Z,
                WorldUp::Colmap => -Vec3::Y,
            };
            for &yaw in &[0.0_f32, 30.0, 90.0, 135.0, 200.0, -60.0] {
                let fwd = cam_forward_horiz(&cfg(world_up), &OrbitState { yaw, pitch: -20.0, dist: 8.0 });
                let right = fwd.cross(up).normalize();
                for (key, expected, name) in [
                    (KeyCode::KeyW, fwd, "W"),
                    (KeyCode::KeyS, -fwd, "S"),
                    (KeyCode::KeyD, right, "D"),
                    (KeyCode::KeyA, -right, "A"),
                ] {
                    let dot = moved(world_up, yaw, key).dot(expected);
                    assert!(
                        dot > 0.999,
                        "{world_up:?} yaw {yaw}: {name} should move along {expected:?} (dot={dot})"
                    );
                }
            }
        }
    }
}
