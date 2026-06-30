//! Bevy plugin for battle mode: spawns projectiles, updates their physics,
//! syncs visual entities, and handles firing input.
//!
//! Active only when `GameType::Battle` + `AppState::Playing`. The core logic
//! (projectile physics, weapon state) lives in `battle.rs`; this plugin is the
//! Bevy glue: spawning meshes, reading keyboard for fire, syncing transforms.

use bevy::camera::visibility::RenderLayers;
use bevy::prelude::*;
use cgmath::Matrix3;

use crate::app_state::{AppState, GameType};
use crate::battle::{BattleState, WeaponState};
use crate::flight::Players;
use crate::scene::SceneEntity;
use crate::splat_plugin::SplatScene;

/// RenderLayers for projectiles: visible to all cameras (layer 0 = shared).
const PROJECTILE_LAYER: usize = 0;

/// Marker for projectile visual entities (all despawned + respawned each frame).
#[derive(Component)]
struct ProjectileVisual;

pub struct BattlePlugin;

impl Plugin for BattlePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppState::Playing), setup_battle);
        app.add_systems(
            Update,
            (
                battle_fire_system,
                battle_projectile_physics_system,
                battle_visual_sync_system,
            )
                .chain()
                .run_if(in_state(AppState::Playing))
                .run_if(resource_exists::<BattleState>),
        );
    }
}

/// `OnEnter(Playing)`: if GameType::Battle, create the BattleState resource (empty —
/// weapons are lazily initialized in `battle_fire_system` once `Players` exists).
fn setup_battle(
    mut commands: Commands,
    game_type: Res<GameType>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if *game_type != GameType::Battle {
        return;
    }
    commands.insert_resource(BattleState::default());
    // Cache the beam mesh + material so we don't rebuild per projectile.
    let mesh = build_beam_mesh(&mut meshes);
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.1, 0.1),
        emissive: Color::srgb(0.8, 0.0, 0.0).into(),
        unlit: true,
        ..default()
    });
    commands.insert_resource(BeamMeshRes(mesh));
    commands.insert_resource(BeamMatRes(mat));
    info!("[Battle] setup: initialized (weapons lazy-init on first frame)");
}

/// Read fire input (LeftShift=P1, RightShift=P2) and spawn projectiles.
/// Also lazily initializes weapons when `Players` first becomes available.
fn battle_fire_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut battle: ResMut<BattleState>,
    players: Res<Players>,
    settings_open: Res<crate::flight::SettingsOpen>,
    menu: Res<crate::menu::MenuState>,
) {
    if settings_open.0 || menu.show_exit {
        return;
    }
    // Lazy-init: ensure we have one weapon per player.
    if battle.weapons.len() < players.0.len() {
        battle.weapons.resize_with(players.0.len(), WeaponState::default);
        info!("[Battle] initialized {} weapon(s)", players.0.len());
    }
    let fire_keys = [KeyCode::ShiftLeft, KeyCode::ShiftRight];
    for (i, p) in players.0.iter().enumerate() {
        if !p.armed {
            continue;
        }
        if !keys.pressed(fire_keys[i]) {
            continue;
        }
        let d = &p.drone;
        let rot = Matrix3::from(d.orientation);
        let forward = cgmath::Vector3::new(rot.x.x, rot.x.y, rot.x.z);
        let half_size = d.drone_size * 0.5;
        let nose_pos = cgmath::Vector3::new(
            d.x + forward.x * half_size,
            d.y + forward.y * half_size,
            d.z + forward.z * half_size,
        );
        battle.try_fire(i, nose_pos, forward);
    }
}

/// Step projectile physics + remove expired/collided ones.
fn battle_projectile_physics_system(
    time: Res<Time>,
    splat: Res<SplatScene>,
    mut battle: ResMut<BattleState>,
) {
    let dt = time.delta_secs();
    battle.tick_cooldowns(dt);
    let octree = splat.collision_octree.as_ref();
    let to_remove = battle.update_projectiles(dt, octree);
    if !to_remove.is_empty() {
        battle.remove_projectiles(&to_remove);
    }
}

/// Sync visual entities to match BattleState.projectiles.
/// Simple approach: despawn all old visuals, spawn one per live projectile each frame.
fn battle_visual_sync_system(
    mut commands: Commands,
    battle: Res<BattleState>,
    mut visuals: Query<Entity, With<ProjectileVisual>>,
    beam_mesh: Option<Res<BeamMeshRes>>,
    beam_mat: Option<Res<BeamMatRes>>,
) {
    // Despawn all existing projectile visuals.
    for e in &mut visuals {
        commands.entity(e).despawn();
    }

    let (Some(beam_mesh), Some(beam_mat)) = (beam_mesh.as_deref(), beam_mat.as_deref()) else {
        return;
    };

    // Spawn one visual per live projectile.
    for p in &battle.projectiles {
        let vel = Vec3::new(p.vel.x, p.vel.y, p.vel.z);
        let rot = if vel.length_squared() > 0.001 {
            Quat::from_rotation_arc(Vec3::Y, vel.normalize())
        } else {
            Quat::IDENTITY
        };
        commands.spawn((
            Mesh3d(beam_mesh.0.clone()),
            MeshMaterial3d(beam_mat.0.clone()),
            Transform::from_translation(Vec3::new(p.pos.x, p.pos.y, p.pos.z))
                .with_rotation(rot),
            Visibility::default(),
            ProjectileVisual,
            RenderLayers::layer(PROJECTILE_LAYER),
            SceneEntity,
        ));
    }
}

/// Cached beam mesh handle (built once at setup_battle, reused for all projectiles).
#[derive(Resource)]
struct BeamMeshRes(Handle<Mesh>);

/// Cached beam material handle (cyan, unlit, emissive).
#[derive(Resource)]
struct BeamMatRes(Handle<StandardMaterial>);

/// Build the beam mesh (cylinder: radius 0.05, height 1.0, local Y = travel axis).
/// Bevy's Cylinder is oriented along Y, so the visual sync uses from_rotation_arc
/// with Vec3::Y as the forward reference.
fn build_beam_mesh(meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    meshes.add(Cylinder::new(0.05, 1.0))
}
