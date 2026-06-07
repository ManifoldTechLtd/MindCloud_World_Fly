//! Gate rendering (Bevy glue around the pure-logic [`crate::gates::GateCourse`]).
//!
//! Builds the course from the scene's saved gate path (or a demo ring when none exists), spawns a
//! square-frame mesh per gate oriented along the Catmull-Rom travel direction, and each frame:
//! recolours by state (start gate = black/white checker, next = green + pulse, others = blue) and,
//! while flying, shows only the next gate + the two after it. `G` shows/hides the whole course.

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::RenderLayers;
use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::prelude::*;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use std::path::Path;

use crate::app_state::{AppState, CurrentSceneConfig, GameMode, WorldUp};
use crate::flight::Players;
use crate::gates::{self, GateCourse, GateEvent};
use crate::persistence;
use crate::scene::SceneEntity;
use crate::splat_plugin::SplatScene;

/// How many gates are shown while flying: the next gate + the two after it (3 total). Single-player
/// (lap mode) wraps around to preview the next lap; linear (dual) races don't wrap and hide every
/// gate once finished. Placement always shows the whole course.
const GATE_WINDOW: usize = 3;

/// The active race course (pure logic), with **one progress tracker per player**. Every entry
/// shares identical geometry (gates / size / world-up / visibility, all built from the same control
/// points); only the live pass / lap / next-gate state differs, so each split-screen pilot races
/// independently. Inserted once the scene config is ready (in `Placement`), removed on return to the
/// menu so the next scene rebuilds fresh.
#[derive(Resource)]
pub struct RaceCourse {
    pub players: Vec<GateCourse>,
}

impl RaceCourse {
    /// Shared geometry / visibility lives on every entry; player 0 is the canonical copy used for
    /// geometry reads (gate editor, world-up check, gate count, visibility toggle).
    pub fn shared(&self) -> &GateCourse {
        &self.players[0]
    }
}

/// Set by the gate editor on Accept (after it rebuilds [`RaceCourse`]) to request a fresh respawn
/// of the 3D gate frames. `respawn_gate_visuals` consumes it.
#[derive(Resource, Default)]
pub struct GateVisualsDirty(pub bool);

/// Parent of all gate-frame entities (also a `SceneEntity` so `menu::cleanup_scene` despawns it +
/// its children recursively on return to the menu).
#[derive(Component)]
struct GateRoot;

/// One gate frame: its course index, which player's set it belongs to (split-screen renders one set
/// per player on its own `RenderLayers`, so each view pulses its OWN next gate), and the
/// (shared-by-its-4-bars) emissive material to recolour.
#[derive(Component)]
struct GateVisual {
    index: usize,
    player: usize,
    material: Handle<StandardMaterial>,
}

pub struct GatePlugin;

impl Plugin for GatePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GateVisualsDirty>();
        app.add_systems(
            Update,
            (
                rebuild_gates_on_world_up_change,
                ensure_gates_built,
                respawn_gate_visuals,
            )
                .chain()
                .run_if(in_state(AppState::Placement)),
        );
        app.add_systems(
            Update,
            (toggle_gates, update_gate_appearance).run_if(in_placement_or_playing),
        );
        app.add_systems(
            Update,
            gate_race_system.run_if(in_state(AppState::Playing)),
        );
        app.add_systems(OnEnter(AppState::ModeSelect), gate_cleanup);
    }
}

fn in_placement_or_playing(state: Res<State<AppState>>) -> bool {
    matches!(*state.get(), AppState::Placement | AppState::Playing)
}

/// World up axis as a vector (gate frames orient "up" along this). Z-up = +Z; COLMAP = -Y (gravity +Y).
pub(crate) fn world_up_vec(wu: WorldUp) -> cgmath::Vector3<f32> {
    match wu {
        WorldUp::Zup => cgmath::Vector3::new(0.0, 0.0, 1.0),
        WorldUp::Colmap => cgmath::Vector3::new(0.0, -1.0, 0.0),
    }
}

/// `Update` (Placement): build the course + spawn its frames exactly once (when the scene config is
/// ready and no course exists yet). Loads the saved gate path for this scene, else a demo ring.
fn ensure_gates_built(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    existing: Option<Res<RaceCourse>>,
    config: Option<Res<CurrentSceneConfig>>,
    splat: Res<SplatScene>,
    mode: Res<GameMode>,
) {
    if existing.is_some() {
        return;
    }
    let Some(config) = config else {
        return;
    };

    let world_up = world_up_vec(config.0.world_up);
    let record = splat
        .ply_path
        .as_ref()
        .map(|p| persistence::scene_key(Path::new(p)))
        .and_then(|k| persistence::load_scene_record(&k));

    let mut course = GateCourse::new();
    let points: Vec<cgmath::Vector3<f32>> = if let Some(rec) = &record {
        course.gate_size = rec.gate_size;
        course.best_lap_ms = rec.best_lap_ms;
        rec.points
            .iter()
            .map(|p| cgmath::Vector3::new(p[0], p[1], p[2]))
            .collect()
    } else {
        // Demo course sits AHEAD of the spawn along the heading (not centered on it): a ring
        // centered on the spawn surrounds the drone, so climbing straight up its axis makes it
        // shrink-in-place and look "attached" to the plane. Offsetting it forward makes the gates
        // read as clearly world-fixed (you fly toward them).
        // fwd = drone spawn forward = orbit look_dir = the spawn arrow's direction, so the ring sits
        // AHEAD of the spawn in the fly-forward direction (Zup: (sin h, cos h, 0); Colmap unchanged).
        let h = config.0.heading_deg.to_radians();
        let fwd = match config.0.world_up {
            WorldUp::Zup => [h.sin(), h.cos(), 0.0],
            WorldUp::Colmap => [h.cos(), 0.0, h.sin()],
        };
        let r = 12.0_f32;
        let s = config.0.spawn;
        let center = [s[0] + fwd[0] * r * 1.5, s[1] + fwd[1] * r * 1.5, s[2] + fwd[2] * r * 1.5];
        demo_ring(center, world_up, r)
    };
    course.rebuild(&points, world_up);
    course.visible = true;
    // Dual (split) races are linear: clock from GO, finish on the last gate. Single-player keeps
    // lap racing (clock from the first gate-0 crossing, laps loop forever).
    course.linear = matches!(*mode, GameMode::SplitScreen);

    let n = course.gates.len();
    // One independent progress tracker per player (identical geometry, separate pass/lap state).
    let n_players = match *mode {
        GameMode::SinglePlayer => 1,
        GameMode::SplitScreen => 2,
    };
    let race = RaceCourse { players: vec![course; n_players] };
    spawn_gate_visuals(&mut commands, &mut meshes, &mut materials, &mut images, &race);
    commands.insert_resource(race);
    info!(
        "[Gates] built course: {} gates x {} player(s) ({}); press G to toggle",
        n, n_players,
        if record.is_some() { "saved path" } else { "demo ring" }
    );
}

/// `Update` (Placement, before `ensure_gates_built`): if the user flips World Up in the overlay,
/// the course (built for the old up-axis) is now stale — the camera/spawn-marker/WASD already track
/// the new axis, but the gates keep their old plane + orientation. Drop the course + its frames so
/// `ensure_gates_built` regenerates them on the new horizontal plane (demo ring) or reloads the
/// saved path, re-oriented for the new up-axis.
fn rebuild_gates_on_world_up_change(
    mut commands: Commands,
    config: Option<Res<CurrentSceneConfig>>,
    course: Option<Res<RaceCourse>>,
    roots: Query<Entity, With<GateRoot>>,
) {
    let (Some(config), Some(course)) = (config, course) else {
        return;
    };
    if course.shared().world_up == world_up_vec(config.0.world_up) {
        return;
    }
    for e in &roots {
        commands.entity(e).despawn();
    }
    commands.remove_resource::<RaceCourse>();
    info!("[Gates] world-up changed -> rebuilding course");
}

/// `OnEnter(ModeSelect)`: drop the course resource (visual entities are despawned by
/// `menu::cleanup_scene` via the `SceneEntity` root) so the next scene builds fresh.
fn gate_cleanup(mut commands: Commands) {
    commands.remove_resource::<RaceCourse>();
}

/// `Update` (Placement): when the gate editor commits a new path it updates [`RaceCourse`] and sets
/// `GateVisualsDirty`; here we despawn the old `GateRoot` (+ its frames) and respawn from the
/// now-current course, then clear the flag.
fn respawn_gate_visuals(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut dirty: ResMut<GateVisualsDirty>,
    course: Option<Res<RaceCourse>>,
    roots: Query<Entity, With<GateRoot>>,
) {
    if !dirty.0 {
        return;
    }
    dirty.0 = false;
    let Some(course) = course else {
        return;
    };
    for e in &roots {
        commands.entity(e).despawn();
    }
    spawn_gate_visuals(&mut commands, &mut meshes, &mut materials, &mut images, &course);
    info!("[Gates] respawned {} gate frames after edit", course.shared().gates.len());
}

/// `Update` (Playing): drive pass detection + lap timing **per player** — each `players[i]` tracker
/// follows player i's drone, so timing + gate count are fully independent across the split. Courses
/// only react while visible (press G). R resets every player's lap (the master reset).
fn gate_race_system(
    course: Option<ResMut<RaceCourse>>,
    players: Option<Res<Players>>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut sfx: MessageWriter<crate::audio::GatePassed>,
) {
    let Some(mut course) = course else {
        return;
    };
    let Some(players) = players else {
        return;
    };
    if players.0.is_empty() {
        return;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        for c in &mut course.players {
            c.reset_lap();
        }
    }
    let now_ms = time.elapsed_secs_f64() * 1000.0;
    let dt = time.delta_secs();
    // Each player's tracker follows their own drone, so timing + gate count are independent.
    let mut newly_finished: Vec<usize> = Vec::new();
    for (i, c) in course.players.iter_mut().enumerate() {
        let Some(p) = players.0.get(i) else {
            break;
        };
        let d = &p.drone;
        let pos = cgmath::Vector3::new(d.x, d.y, d.z);
        for ev in c.update(dt, pos, now_ms) {
            // Every crossing (gate pass, lap, or finish) fires one gate-ding.
            sfx.write(crate::audio::GatePassed);
            match ev {
                GateEvent::Passed { index, total } => {
                    info!("[Race][P{}] passed gate {} / {}", i + 1, index + 1, total);
                }
                GateEvent::LapComplete { lap_ms, is_best } => {
                    info!(
                        "[Race][P{}] lap {}{}",
                        i + 1,
                        gates::format_lap(lap_ms),
                        if is_best { " — NEW BEST" } else { "" }
                    );
                }
                GateEvent::Finished { total_ms, is_best } => {
                    info!(
                        "[Race][P{}] FINISHED {}{}",
                        i + 1,
                        gates::format_lap(total_ms),
                        if is_best { " — NEW BEST" } else { "" }
                    );
                    newly_finished.push(i);
                }
            }
        }
    }
    // Assign finishing places in the order players crossed the final gate (No.1, No.2, …).
    for i in newly_finished {
        let rank = course.players.iter().filter(|c| c.finish_rank.is_some()).count() as u8 + 1;
        course.players[i].finish_rank = Some(rank);
        info!("[Race][P{}] place No.{}", i + 1, rank);
    }
}

/// `Update` (Placement/Playing): `G` toggles the whole course visibility.
fn toggle_gates(keys: Res<ButtonInput<KeyCode>>, course: Option<ResMut<RaceCourse>>) {
    if let Some(mut course) = course {
        if keys.just_pressed(KeyCode::KeyG) {
            let vis = !course.shared().visible;
            for c in &mut course.players {
                c.visible = vis;
            }
            info!("[Gates] visible = {}", vis);
        }
    }
}

/// `Update` (Placement/Playing): recolour by state, pulse the next gate, apply horizon visibility.
fn update_gate_appearance(
    course: Option<Res<RaceCourse>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut q: Query<(&GateVisual, &mut Transform, &mut Visibility)>,
    state: Res<State<AppState>>,
) {
    let Some(course) = course else {
        return;
    };
    if course.players.is_empty() {
        return;
    }
    // Horizon culling only while flying; in placement show the whole course for editing/overview.
    let flying = *state.get() == AppState::Playing;
    for (gv, mut tf, mut vis) in &mut q {
        // Each frame belongs to one player's set and reflects THAT player's progress, so each split
        // view pulses its own next gate (the set is shown only to its camera's RenderLayers).
        let Some(c) = course.players.get(gv.player) else {
            continue;
        };
        let n = c.gates.len();
        if n == 0 {
            continue;
        }
        // --- Visibility: placement shows the whole course (overview/editing); flight shows only the
        // next gate + the two after it (`GATE_WINDOW`). Single-player (lap) wraps around to preview
        // the next lap; linear (dual) races don't wrap and hide every gate once finished.
        let in_window = if !flying {
            true
        } else if c.linear {
            let d = gv.index as i32 - c.next_gate_idx as i32;
            c.finished_at.is_none() && (0..GATE_WINDOW as i32).contains(&d)
        } else {
            (gv.index + n - c.next_gate_idx) % n < GATE_WINDOW
        };
        *vis = if c.visible && in_window {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };

        // --- Colour: set `base_color` (unlit materials ignore `emissive` — that's why the old
        // emissive-only scheme rendered black). Gate 0 keeps its checker-flag texture (set at spawn).
        // The next gate is green while flying (unless it IS the start gate); every other gate is blue.
        let is_next = flying && gv.index == c.next_gate_idx;
        if gv.index != 0 {
            let col = if is_next {
                Color::srgb(0.1, 0.9, 0.2) // green — fly here next
            } else {
                Color::srgb(0.0, 0.45, 1.0) // blue — upcoming
            };
            if let Some(m) = materials.get_mut(&gv.material) {
                m.base_color = col;
            }
        }

        // The next gate pulses (the start gate pulses too — it just stays checker, not green).
        tf.scale = if is_next {
            Vec3::splat(c.pulse_scale())
        } else {
            Vec3::ONE
        };
    }
}

/// Quaternion that maps gate-local axes (X = right, Y = up, Z = travel dir) into world space.
/// Mirrors the basis built in `gates::GateCourse::rebuild`.
fn gate_basis_quat(travel_dir: cgmath::Vector3<f32>, world_up: cgmath::Vector3<f32>) -> Quat {
    let fwd = Vec3::new(travel_dir.x, travel_dir.y, travel_dir.z).normalize_or_zero();
    let wup = Vec3::new(world_up.x, world_up.y, world_up.z);
    let mut right = wup.cross(fwd);
    if right.length_squared() < 1e-6 {
        let alt = if fwd.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        right = alt - fwd * alt.dot(fwd);
    }
    right = right.normalize_or_zero();
    let up = fwd.cross(right).normalize_or_zero();
    Quat::from_mat3(&Mat3::from_cols(right, up, fwd))
}

/// Spawn the gate-frame entities under a single `GateRoot`. Each gate = 4 thin cuboid bars forming a
/// square ring in its local XY plane (local +Z = travel direction), sharing one emissive material.
fn spawn_gate_visuals(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    images: &mut Assets<Image>,
    course: &RaceCourse,
) {
    let geom = course.shared();
    let size = geom.gate_size;
    let t = (size * 0.10).max(0.06);
    let half = size * 0.5;
    // Faced box meshes: per-face UVs (so the checker squares stay uniform on EVERY face, including
    // the thin sides) + per-face vertex colours (front/back bright, sides dim) baked in for a 3D
    // "directional-lighting" cue that survives recolouring. Local +Z = travel = the front/back axis.
    let h_bar = meshes.add(faced_box_mesh(size + t, t, t));
    let v_bar = meshes.add(faced_box_mesh(t, size - t, t));
    // Shared black/white checker texture for every player's start gate (gate 0).
    let checker = images.add(make_checker_image());

    let root = commands
        .spawn((GateRoot, SceneEntity, Transform::default(), Visibility::default()))
        .id();

    // One frame set per player. Player p's set lives on RenderLayers `p + 1` so the matching split
    // camera (layers {0, p+1}) sees ONLY its own set, letting each view pulse its own next gate.
    // Layer 0 stays shared (splat-driven meshes, spawn marker).
    for player in 0..course.players.len() {
        let layer = RenderLayers::layer(player + 1);
        for (i, gate) in geom.gates.iter().enumerate() {
            // Gate 0 (start/finish) wears a black/white checker flag; the rest start blue and are
            // recoloured per-frame by `update_gate_appearance`. All unlit so the colour/texture shows
            // directly (a lit material would be black with no scene lights). The bar mesh carries
            // per-face vertex colours (front/back bright, sides dim), which the unlit material
            // multiplies into base_color/texture — so both the solid colour AND the checker get the
            // same 3D face shading, and recolouring (which only changes base_color) preserves it.
            let material = materials.add(if i == 0 {
                StandardMaterial {
                    base_color: Color::WHITE,
                    base_color_texture: Some(checker.clone()),
                    unlit: true,
                    ..default()
                }
            } else {
                StandardMaterial {
                    base_color: Color::srgb(0.0, 0.45, 1.0),
                    unlit: true,
                    ..default()
                }
            });
            let pos = Vec3::new(gate.pos.x, gate.pos.y, gate.pos.z);
            let rot = gate_basis_quat(gate.travel_dir, geom.world_up);
            commands.entity(root).with_children(|p| {
                p.spawn((
                    GateVisual { index: i, player, material: material.clone() },
                    Transform::from_translation(pos).with_rotation(rot),
                    Visibility::default(),
                    layer.clone(),
                ))
                .with_children(|g| {
                    g.spawn((
                        Mesh3d(h_bar.clone()),
                        MeshMaterial3d(material.clone()),
                        Transform::from_xyz(0.0, half, 0.0),
                        layer.clone(),
                    ));
                    g.spawn((
                        Mesh3d(h_bar.clone()),
                        MeshMaterial3d(material.clone()),
                        Transform::from_xyz(0.0, -half, 0.0),
                        layer.clone(),
                    ));
                    g.spawn((
                        Mesh3d(v_bar.clone()),
                        MeshMaterial3d(material.clone()),
                        Transform::from_xyz(-half, 0.0, 0.0),
                        layer.clone(),
                    ));
                    g.spawn((
                        Mesh3d(v_bar.clone()),
                        MeshMaterial3d(material.clone()),
                        Transform::from_xyz(half, 0.0, 0.0),
                        layer.clone(),
                    ));
                });
            });
        }
    }
}

/// Build the 2×2 black/white checker texture worn by every start gate (gate 0), mirroring JS
/// `getCheckerTexture`. REPEAT + NEAREST keeps the squares crisp; `faced_box_mesh` bakes per-face UVs
/// scaled to world size so this 2×2 tiles into uniform squares on every bar face (sides included).
fn make_checker_image() -> Image {
    // One pixel per square: (white, black / black, white).
    let data = vec![
        255, 255, 255, 255, 0, 0, 0, 255, // row 0
        0, 0, 0, 255, 255, 255, 255, 255, // row 1
    ];
    let mut image = Image::new(
        Extent3d { width: 2, height: 2, depth_or_array_layers: 1 },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    // NEAREST (crisp squares) + REPEAT (so the per-face UVs > 1 from `faced_box_mesh` wrap/tile).
    let mut desc = ImageSamplerDescriptor::nearest();
    desc.address_mode_u = ImageAddressMode::Repeat;
    desc.address_mode_v = ImageAddressMode::Repeat;
    desc.address_mode_w = ImageAddressMode::Repeat;
    image.sampler = ImageSampler::Descriptor(desc);
    image
}

/// Build a flat-shaded box mesh (dims `sx,sy,sz`, centred at origin) for one gate bar, carrying:
/// - **per-face UVs** scaled to each face's world size, so a tiled checker renders UNIFORM squares
///   on *every* face including the thin sides (a single `uv_transform` could only make ONE face's
///   squares square — the others stretched into stripes), and
/// - **per-face vertex colours**: the ±Z faces (the gate front/back the pilot sees) full-bright, all
///   other faces dimmed. The unlit material multiplies these in, baking a directional-lighting cue
///   into both solid-colour and checker gates that survives per-frame recolouring (mirrors JS
///   `FACE_BRIGHTNESS`: front/back 1.0, outer/inner/caps 0.4).
fn faced_box_mesh(sx: f32, sy: f32, sz: f32) -> Mesh {
    const SQUARE: f32 = 0.11; // world size of one checker square (gate-0 only; ignored when untextured)
    const FRONT: f32 = 1.0; // ±Z brightness (gate front/back)
    const SIDE: f32 = 0.5; // every other face (outer/inner rim + end caps)
    let (hx, hy, hz) = (sx * 0.5, sy * 0.5, sz * 0.5);
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(24);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(24);
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(24);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(24);
    let mut indices: Vec<u32> = Vec::with_capacity(36);
    // Append one outward-facing quad (CCW p0→p1→p2→p3). `u_size`/`v_size` are the face's world dims
    // along p0→p1 and p1→p2; UVs span 0..size/(2*SQUARE) (×2 because the checker texture is 2×2).
    let mut add = |p0: [f32; 3], p1: [f32; 3], p2: [f32; 3], p3: [f32; 3], n: [f32; 3], u_size: f32, v_size: f32, b: f32| {
        let base = positions.len() as u32;
        let (u, v) = (u_size / (2.0 * SQUARE), v_size / (2.0 * SQUARE));
        positions.extend_from_slice(&[p0, p1, p2, p3]);
        normals.extend_from_slice(&[n; 4]);
        uvs.extend_from_slice(&[[0.0, 0.0], [u, 0.0], [u, v], [0.0, v]]);
        colors.extend_from_slice(&[[b, b, b, 1.0]; 4]);
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    };
    // Front / back (±Z): bright. U along X, V along Y.
    add([-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz], [0.0, 0.0, 1.0], sx, sy, FRONT);
    add([hx, -hy, -hz], [-hx, -hy, -hz], [-hx, hy, -hz], [hx, hy, -hz], [0.0, 0.0, -1.0], sx, sy, FRONT);
    // Inner / outer rim (±X): dim. U along Z, V along Y.
    add([hx, -hy, hz], [hx, -hy, -hz], [hx, hy, -hz], [hx, hy, hz], [1.0, 0.0, 0.0], sz, sy, SIDE);
    add([-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, hz], [-hx, hy, -hz], [-1.0, 0.0, 0.0], sz, sy, SIDE);
    // Rim / caps (±Y): dim. U along X, V along Z.
    add([-hx, hy, hz], [hx, hy, hz], [hx, hy, -hz], [-hx, hy, -hz], [0.0, 1.0, 0.0], sx, sz, SIDE);
    add([-hx, -hy, -hz], [hx, -hy, -hz], [hx, -hy, hz], [-hx, -hy, hz], [0.0, -1.0, 0.0], sx, sz, SIDE);
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::RENDER_WORLD)
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
        .with_inserted_indices(Indices::U32(indices))
}

/// A default closed loop of 6 gates ringing the spawn on the world-up horizontal plane (radius m),
/// used when a scene has no saved gate path yet so the course is visible before the editor exists.
fn demo_ring(center: [f32; 3], world_up: cgmath::Vector3<f32>, radius: f32) -> Vec<cgmath::Vector3<f32>> {
    let c = Vec3::from_array(center);
    let up = Vec3::new(world_up.x, world_up.y, world_up.z).normalize_or_zero();
    let seed = if up.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let e1 = up.cross(seed).normalize_or_zero();
    let e2 = up.cross(e1).normalize_or_zero();
    let n = 6;
    (0..n)
        .map(|i| {
            let a = i as f32 / n as f32 * std::f32::consts::TAU;
            let p = c + (e1 * a.cos() + e2 * a.sin()) * radius;
            cgmath::Vector3::new(p.x, p.y, p.z)
        })
        .collect()
}
