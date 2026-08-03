//! Bevy plugin for battle mode: spawns projectiles, updates their physics,
//! syncs visual entities, and handles firing input.
//!
//! Active only when `GameType::Battle` + `AppState::Playing`. The core logic
//! (projectile physics, weapon state) lives in `battle.rs`; this plugin is the
//! Bevy glue: spawning meshes, reading keyboard for fire, syncing transforms.

use bevy::camera::visibility::RenderLayers;
use bevy::prelude::*;
use bevy_egui::egui;
use cgmath::Matrix3;

use crate::app_state::{AppState, CurrentSceneConfig, GameType, WorldUp};
use crate::battle::{BattleState, Health, WeaponState};
use crate::flight::Players;
use crate::scene::SceneEntity;
use crate::splat_plugin::SplatScene;

/// RenderLayers for projectiles: visible to all cameras (layer 0 = shared).
const PROJECTILE_LAYER: usize = 0;

/// Muzzle offset below the camera/screen centre (m, along body +Z = down): the shot spawns
/// slightly under the view centre so the tracer is visible leaving the screen.
const MUZZLE_DROP: f32 = 0.2;

/// Fixed battle hit-sphere radius per drone (m). Intentionally generous (vs the tunable
/// physics `collision_radius`) so shots connect more often; scene collision is unaffected.
const BATTLE_HIT_RADIUS: f32 = 0.3;

/// Gravitational acceleration applied to projectiles (m/s²) — ballistic drop.
const PROJECTILE_G: f32 = 9.81;

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
                battle_reset_system,
            )
                .chain()
                .run_if(in_state(AppState::Playing))
                .run_if(resource_exists::<BattleState>),
        );
        app.add_systems(
            bevy_egui::EguiPrimaryContextPass,
            battle_finish_overlay_system
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
    // Cache the projectile mesh + material so we don't rebuild per shot.
    let mesh = build_projectile_mesh(&mut meshes);
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 1.0, 0.3),
        emissive: Color::srgb(0.0, 0.8, 0.2).into(),
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
    // Lazy-init: ensure we have one weapon + health per player.
    if battle.weapons.len() < players.0.len() {
        battle.weapons.resize_with(players.0.len(), WeaponState::default);
        while battle.player_health.len() < players.0.len() {
            battle.player_health.push(Health::new(100.0));
        }
        info!("[Battle] initialized {} weapon(s) + health", players.0.len());
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
        // Body +Z = down (NED). The muzzle sits MUZZLE_DROP below the camera/screen centre so the
        // tracer is visible as it leaves the view instead of spawning at the exact eye point.
        let down = cgmath::Vector3::new(rot.z.x, rot.z.y, rot.z.z);
        let half_size = d.drone_size * 0.5;
        let muzzle_pos = cgmath::Vector3::new(
            d.x + forward.x * half_size + down.x * MUZZLE_DROP,
            d.y + forward.y * half_size + down.y * MUZZLE_DROP,
            d.z + forward.z * half_size + down.z * MUZZLE_DROP,
        );
        battle.try_fire(i, muzzle_pos, forward);
    }
}

/// Step projectile physics (ballistic, world-up-aware gravity) + remove expired/collided
/// ones, then check hits against the fixed battle hit spheres.
fn battle_projectile_physics_system(
    time: Res<Time>,
    splat: Res<SplatScene>,
    mut battle: ResMut<BattleState>,
    players: Res<Players>,
    config: Res<CurrentSceneConfig>,
) {
    if battle.winner.is_some() {
        return; // match over — freeze projectiles
    }
    let dt = time.delta_secs();
    battle.tick_cooldowns(dt);
    let octree = splat.collision_octree.as_ref();
    // World-frame gravity for the ballistic arc (Zup: -Z is down; Colmap: +Y is down).
    let gravity = match config.0.world_up {
        WorldUp::Zup => cgmath::Vector3::new(0.0, 0.0, -PROJECTILE_G),
        WorldUp::Colmap => cgmath::Vector3::new(0.0, PROJECTILE_G, 0.0),
    };
    let to_remove = battle.update_projectiles(dt, gravity, octree);
    if !to_remove.is_empty() {
        battle.remove_projectiles(&to_remove);
    }
    // Check projectile-vs-drone hits with the FIXED battle hit radius (not the tunable
    // physics collision_radius) so hit probability stays consistent.
    let drone_positions: Vec<(cgmath::Vector3<f32>, f32)> = players.0.iter()
        .map(|p| {
            (cgmath::Vector3::new(p.drone.x, p.drone.y, p.drone.z), BATTLE_HIT_RADIUS)
        })
        .collect();
    let (hit_remove, events) = battle.check_hits(&drone_positions);
    if !hit_remove.is_empty() {
        battle.remove_projectiles(&hit_remove);
    }
    for ev in &events {
        let killed = battle.apply_hit(ev);
        if killed {
            let killer = ev.killer.map(|k| format!("P{}", k + 1)).unwrap_or_else(|| "NPC".into());
            info!("[Battle] P{} killed by {}", ev.target_player + 1, killer);
        } else {
            info!("[Battle] P{} hit for {} damage", ev.target_player + 1, ev.damage);
        }
    }
    battle.check_win_condition();
    if let Some(crate::battle::BattleWinner::Player(winner)) = battle.winner {
        info!("[Battle] MATCH OVER — P{} WINS!", winner + 1);
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

    // Spawn one visual per live projectile. Spheres are orientation-free — no velocity
    // alignment needed (the old cylinder beam aligned its Y axis to the velocity here).
    for p in &battle.projectiles {
        commands.spawn((
            Mesh3d(beam_mesh.0.clone()),
            MeshMaterial3d(beam_mat.0.clone()),
            Transform::from_translation(Vec3::new(p.pos.x, p.pos.y, p.pos.z)),
            Visibility::default(),
            ProjectileVisual,
            RenderLayers::layer(PROJECTILE_LAYER),
            SceneEntity,
        ));
    }
}

/// Cached projectile mesh handle (built once at setup_battle, reused for all shots).
#[derive(Resource)]
struct BeamMeshRes(Handle<Mesh>);

/// Cached projectile material handle (green tracer, unlit, emissive).
#[derive(Resource)]
struct BeamMatRes(Handle<StandardMaterial>);

/// Build the projectile mesh: a small green ball (radius 0.1 m). Replaced the earlier
/// red cylinder beam — spheres need no orientation and read cleanly as ballistic tracers.
fn build_projectile_mesh(meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    meshes.add(Sphere::new(0.1))
}

/// Handle R-reset in battle mode: reset health, clear projectiles, clear winner.
fn battle_reset_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut battle: ResMut<BattleState>,
) {
    if !keys.just_pressed(KeyCode::KeyR) {
        return;
    }
    // Reset all player health to full.
    for h in &mut battle.player_health {
        h.current = h.max;
    }
    // Clear projectiles + winner.
    battle.projectiles.clear();
    battle.winner = None;
    info!("[Battle] reset — all health restored, match restarted");
}

/// Battle finish overlay: when `battle.winner` is set, show VICTORY/DEFEATED to each player.
fn battle_finish_overlay_system(
    mut contexts: bevy_egui::EguiContexts,
    battle: Res<BattleState>,
    players: Res<Players>,
    windows: Query<&Window>,
) -> Result {
    let Some(winner) = battle.winner else {
        return Ok(());
    };
    if players.0.is_empty() {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let Ok(win) = windows.single() else {
        return Ok(());
    };
    let (w, h) = (win.width(), win.height());
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("battle_finish"),
    ));
    let winner_idx = match winner {
        crate::battle::BattleWinner::Player(i) => i,
    };

    if players.0.len() <= 1 {
        // Single player: just show result.
        let (text, color) = if winner_idx == 0 {
            ("VICTORY", egui::Color32::from_rgb(90, 230, 120))
        } else {
            ("DEFEATED", egui::Color32::from_rgb(255, 80, 80))
        };
        let curtain = egui::Color32::from_black_alpha(190);
        painter.rect_filled(
            egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(w, h)),
            0.0,
            curtain,
        );
        painter.text(
            egui::pos2(w * 0.5, h * 0.5 - 20.0),
            egui::Align2::CENTER_CENTER,
            text,
            egui::FontId::proportional(60.0),
            color,
        );
        painter.text(
            egui::pos2(w * 0.5, h * 0.5 + 40.0),
            egui::Align2::CENTER_CENTER,
            "press  R  to restart",
            egui::FontId::proportional(16.0),
            egui::Color32::from_gray(170),
        );
    } else {
        // Dual player: show VICTORY/DEFEATED per half.
        let half = h * 0.5;
        let curtain = egui::Color32::from_black_alpha(190);
        for i in 0..2 {
            let top = half * i as f32;
            let rect = egui::Rect::from_min_size(
                egui::pos2(0.0, top),
                egui::vec2(w, half),
            );
            painter.rect_filled(rect, 0.0, curtain);
            let (text, color) = if i == winner_idx {
                ("VICTORY", egui::Color32::from_rgb(255, 215, 70))
            } else {
                ("DEFEATED", egui::Color32::from_rgb(180, 180, 190))
            };
            let cx = w * 0.5;
            let cy = top + half * 0.5;
            painter.text(
                egui::pos2(cx, cy - 20.0),
                egui::Align2::CENTER_CENTER,
                text,
                egui::FontId::proportional(50.0),
                egui::Color32::WHITE,
            );
            painter.text(
                egui::pos2(cx, cy + 30.0),
                egui::Align2::CENTER_CENTER,
                format!("P{} {}", i + 1, if i == winner_idx { "WINS" } else { "LOST" }),
                egui::FontId::proportional(36.0),
                color,
            );
            painter.text(
                egui::pos2(cx, cy + 70.0),
                egui::Align2::CENTER_CENTER,
                "press  R  to restart",
                egui::FontId::proportional(14.0),
                egui::Color32::from_gray(170),
            );
        }
    }
    Ok(())
}
