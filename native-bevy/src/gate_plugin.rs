//! Gate rendering (Bevy glue around the pure-logic [`crate::gates::GateCourse`]).
//!
//! Builds the course from the scene's saved gate path (or a demo ring when none exists), spawns a
//! square-frame mesh per gate oriented along the Catmull-Rom travel direction, and each frame:
//! recolours by state (start = white, next = yellow + pulse, others = cyan) and toggles visibility.
//! `G` shows/hides the whole course. Pass detection + lap timing land in a later slice (race).

use bevy::prelude::*;
use std::path::Path;

use crate::app_state::{AppState, CurrentSceneConfig, WorldUp};
use crate::flight::Players;
use crate::gates::{self, GateCourse, GateEvent};
use crate::persistence;
use crate::scene::SceneEntity;
use crate::splat_plugin::SplatScene;

/// How many gates ahead of `next` stay lit (matches native `HORIZON`). Applied only while flying.
const HORIZON: usize = 5;

/// The active race course (pure logic). Inserted once the scene config is ready (in `Placement`),
/// removed on return to the menu so the next scene rebuilds fresh.
#[derive(Resource)]
pub struct RaceCourse(pub GateCourse);

/// Parent of all gate-frame entities (also a `SceneEntity` so `menu::cleanup_scene` despawns it +
/// its children recursively on return to the menu).
#[derive(Component)]
struct GateRoot;

/// One gate frame: its course index + the (shared-by-its-4-bars) emissive material to recolour.
#[derive(Component)]
struct GateVisual {
    index: usize,
    material: Handle<StandardMaterial>,
}

pub struct GatePlugin;

impl Plugin for GatePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            ensure_gates_built.run_if(in_state(AppState::Placement)),
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
fn world_up_vec(wu: WorldUp) -> cgmath::Vector3<f32> {
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
    existing: Option<Res<RaceCourse>>,
    config: Option<Res<CurrentSceneConfig>>,
    splat: Res<SplatScene>,
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
        demo_ring(config.0.spawn, world_up, 12.0)
    };
    course.rebuild(&points, world_up);
    course.visible = true;

    spawn_gate_visuals(&mut commands, &mut meshes, &mut materials, &course);
    let n = course.gates.len();
    commands.insert_resource(RaceCourse(course));
    info!(
        "[Gates] built course: {} gates ({}); press G to toggle",
        n,
        if record.is_some() { "saved path" } else { "demo ring" }
    );
}

/// `OnEnter(ModeSelect)`: drop the course resource (visual entities are despawned by
/// `menu::cleanup_scene` via the `SceneEntity` root) so the next scene builds fresh.
fn gate_cleanup(mut commands: Commands) {
    commands.remove_resource::<RaceCourse>();
}

/// `Update` (Playing): drive pass detection + lap timing from player 0's (P1) drone position.
/// The course only reacts while visible (press G). R resets the current lap (matches native).
/// Single-course: in split-screen the timer tracks P1 only (native races P1 via the controller).
fn gate_race_system(
    course: Option<ResMut<RaceCourse>>,
    players: Option<Res<Players>>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
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
        course.0.reset_lap();
    }
    let d = &players.0[0].drone;
    let pos = cgmath::Vector3::new(d.x, d.y, d.z);
    let now_ms = time.elapsed_secs_f64() * 1000.0;
    let dt = time.delta_secs();
    for ev in course.0.update(dt, pos, now_ms) {
        match ev {
            GateEvent::Passed { index, total } => {
                info!("[Race] passed gate {} / {}", index + 1, total);
            }
            GateEvent::LapComplete { lap_ms, is_best } => {
                info!(
                    "[Race] lap {}{}",
                    gates::format_lap(lap_ms),
                    if is_best { " — NEW BEST" } else { "" }
                );
            }
        }
    }
}

/// `Update` (Placement/Playing): `G` toggles the whole course visibility.
fn toggle_gates(keys: Res<ButtonInput<KeyCode>>, course: Option<ResMut<RaceCourse>>) {
    if let Some(mut course) = course {
        if keys.just_pressed(KeyCode::KeyG) {
            course.0.visible = !course.0.visible;
            info!("[Gates] visible = {}", course.0.visible);
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
    let c = &course.0;
    let n = c.gates.len();
    if n == 0 {
        return;
    }
    let pulse = c.pulse_scale();
    // Horizon culling only while flying; in placement show the whole course for editing/overview.
    let flying = *state.get() == AppState::Playing;
    for (gv, mut tf, mut vis) in &mut q {
        let dist = (gv.index + n - c.next_gate_idx) % n;
        let in_horizon = !flying || dist < HORIZON;
        *vis = if c.visible && in_horizon {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };

        // Emissive colour (unlit material): start = white, next = yellow, others = cyan. Scaled > 1
        // so the frames punch through bright 3DGS backgrounds.
        let col = if gv.index == 0 {
            LinearRgba::rgb(2.0, 2.0, 2.0)
        } else if gv.index == c.next_gate_idx {
            LinearRgba::rgb(2.2, 1.85, 0.0)
        } else {
            LinearRgba::rgb(0.0, 1.5, 2.0)
        };
        if let Some(m) = materials.get_mut(&gv.material) {
            m.emissive = col;
        }

        tf.scale = if gv.index == c.next_gate_idx {
            Vec3::splat(pulse)
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
    course: &GateCourse,
) {
    let size = course.gate_size;
    let t = (size * 0.10).max(0.06);
    let half = size * 0.5;
    let h_bar = meshes.add(Cuboid::new(size + t, t, t));
    let v_bar = meshes.add(Cuboid::new(t, size - t, t));

    let root = commands
        .spawn((GateRoot, SceneEntity, Transform::default(), Visibility::default()))
        .id();

    for (i, gate) in course.gates.iter().enumerate() {
        let material = materials.add(StandardMaterial {
            base_color: Color::BLACK,
            emissive: LinearRgba::rgb(0.0, 1.5, 2.0),
            unlit: true,
            ..default()
        });
        let pos = Vec3::new(gate.pos.x, gate.pos.y, gate.pos.z);
        let rot = gate_basis_quat(gate.travel_dir, course.world_up);
        commands.entity(root).with_children(|p| {
            p.spawn((
                GateVisual { index: i, material: material.clone() },
                Transform::from_translation(pos).with_rotation(rot),
                Visibility::default(),
            ))
            .with_children(|g| {
                g.spawn((
                    Mesh3d(h_bar.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::from_xyz(0.0, half, 0.0),
                ));
                g.spawn((
                    Mesh3d(h_bar.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::from_xyz(0.0, -half, 0.0),
                ));
                g.spawn((
                    Mesh3d(v_bar.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::from_xyz(-half, 0.0, 0.0),
                ));
                g.spawn((
                    Mesh3d(v_bar.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::from_xyz(half, 0.0, 0.0),
                ));
            });
        });
    }
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
