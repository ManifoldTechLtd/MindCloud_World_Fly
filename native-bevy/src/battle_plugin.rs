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

/// RenderLayers base for the per-player hit-spheres: player `p`'s shell lives on layer
/// `HIT_SPHERE_LAYER_BASE + p`. Each battle camera renders only the OPPONENT's layer — never
/// its own — because the camera sits INSIDE its own 0.5 m shell (double-sided translucent
/// material), which otherwise tints the whole view cyan (the reported "green bullets look
/// cyan / opponent discoloured" bug). Drone models use `DRONE_VIS_LAYER_BASE` (3) + {0,1},
/// so the shells start right after at 5.
pub const HIT_SPHERE_LAYER_BASE: usize = crate::flight::DRONE_VIS_LAYER_BASE + 2;

/// Muzzle offset below the camera/screen centre (m, along body +Z = down): the shot spawns
/// slightly under the view centre so the tracer is visible leaving the screen.
const MUZZLE_DROP: f32 = 0.2;

/// Fixed battle hit-sphere radius per drone (m). Intentionally generous (vs the tunable
/// physics `collision_radius`) so shots connect more often; scene collision is unaffected.
const BATTLE_HIT_RADIUS: f32 = 0.3;

/// Hit-sphere centre lift above the drone ORIGIN (m, along world up): the glTF model's origin
/// sits at its underside, so the ball is raised to wrap the model's geometric centre. Applied
/// identically to the hit TEST and the shield VISUAL so they always coincide.
const BATTLE_HIT_CENTER_LIFT: f32 = 0.1;

/// Gravitational acceleration applied to projectiles (m/s²) — ballistic drop.
const PROJECTILE_G: f32 = 9.81;

/// Hit-marker flash duration (s): how long the white X shows after landing a shot.
const HIT_MARKER_SECS: f32 = 0.3;

/// Alive shields' world spheres `(owner player, centre, radius incl. pulse margin)`, refreshed
/// every frame by `battle_hit_sphere_system` and mirrored into the render world: the splat
/// depth-only pass (shield-vs-3DGS occlusion) scissors to these projected screen rects — and
/// skips entirely when none apply to a view — so its cost tracks the shield's screen size
/// instead of the whole viewport.
#[derive(Resource, Clone, Default, bevy::render::extract_resource::ExtractResource)]
pub struct ShieldWorldSpheres(pub Vec<(usize, Vec3, f32)>);

/// "Got hit" hit-sphere flash duration (s): colour/opacity pulse when a shot lands on you.
const HURT_FLASH_SECS: f32 = 0.35;

/// Marker for projectile visual entities (all despawned + respawned each frame).
#[derive(Component)]
struct ProjectileVisual;

/// Translucent hit-sphere visual following player `player`'s drone (battle mode only): shows
/// the fixed BATTLE_HIT_RADIUS judgement ball; flashes red/opaque briefly when hit.
#[derive(Component)]
struct HitSphereVisual {
    player: usize,
}

pub struct BattlePlugin;

impl Plugin for BattlePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppState::Playing), setup_battle);
        app.init_resource::<ShieldWorldSpheres>();
        app.add_plugins(
            bevy::render::extract_resource::ExtractResourcePlugin::<ShieldWorldSpheres>::default(),
        );
        app.add_systems(
            Update,
            (
                battle_fire_system,
                battle_projectile_physics_system,
                battle_visual_sync_system,
                battle_hit_sphere_system,
                battle_reset_system,
            )
                .chain()
                .run_if(in_state(AppState::Playing))
                .run_if(resource_exists::<BattleState>),
        );
        app.add_systems(
            bevy_egui::EguiPrimaryContextPass,
            (battle_hud_system, battle_finish_overlay_system)
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
    mut shield_spheres: ResMut<ShieldWorldSpheres>,
) {
    // Fresh session: drop any shield rects left over from a previous match (they gate the
    // splat depth-only pass in the render world).
    shield_spheres.0.clear();
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
    // Per-player energy-shield shell (the fixed judgement ball, battle mode only): a smooth
    // translucent bubble. True alpha-blending over the 3DGS background works because the splat
    // pass now composites BEFORE the transparent mesh pass (see splat_plugin's render-graph
    // edges); each camera renders only the OPPONENT's shell (see HIT_SPHERE_LAYER_BASE), so
    // the pilot never sits inside their own bubble tinting the whole view.
    let sphere_mesh = meshes.add(Sphere::new(BATTLE_HIT_RADIUS));
    for player in 0..crate::input_plugin::NUM_PLAYERS {
        let sphere_mat = materials.add(StandardMaterial {
            // Idle: faint cyan, alpha 0.3. Front faces only (default back-face culling) — a
            // single tint layer reads cleaner than front+back double-blend.
            base_color: Color::srgba(0.3, 0.8, 1.0, 0.3),
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            ..default()
        });
        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(sphere_mat),
            Transform::default(),
            Visibility::Hidden,
            HitSphereVisual { player },
            RenderLayers::layer(HIT_SPHERE_LAYER_BASE + player),
            SceneEntity,
        ));
    }
    info!("[Battle] setup: initialized (weapons lazy-init on first frame)");
}

/// Read fire input — keyboard (LeftShift=P1, RightShift=P2) OR the transmitter's mapped Fire
/// switch — and spawn projectiles. Also lazily initializes weapons when `Players` first exists.
fn battle_fire_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut battle: ResMut<BattleState>,
    players: Res<Players>,
    ctrl: Res<crate::input_plugin::ControllerRes>,
    settings_open: Res<crate::flight::SettingsOpen>,
    menu: Res<crate::menu::MenuState>,
) {
    if settings_open.0 || menu.show_exit {
        return;
    }
    // Lazy-init: ensure we have one weapon + health + hit/hurt flash per player.
    if battle.weapons.len() < players.0.len() {
        battle.weapons.resize_with(players.0.len(), WeaponState::default);
        while battle.player_health.len() < players.0.len() {
            battle.player_health.push(Health::new(100.0));
        }
        battle.hit_markers.resize(players.0.len(), 0.0);
        battle.hurt_flashes.resize(players.0.len(), 0.0);
        info!("[Battle] initialized {} weapon(s) + health", players.0.len());
    }
    let fire_keys = [KeyCode::ShiftLeft, KeyCode::ShiftRight];
    for (i, p) in players.0.iter().enumerate() {
        if !p.armed {
            continue;
        }
        // Keyboard OR the transmitter's Fire switch (mapped in F1 → Switches; level = hold).
        let hid_fire = ctrl.0.get(i).is_some_and(|c| c.hid_connected && c.fire_active);
        if !keys.pressed(fire_keys[i]) && !hid_fire {
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
    // physics collision_radius) so hit probability stays consistent. The centre is lifted
    // BATTLE_HIT_CENTER_LIFT above the drone origin (model origin sits at its underside).
    let lift = match config.0.world_up {
        WorldUp::Zup => cgmath::Vector3::new(0.0, 0.0, BATTLE_HIT_CENTER_LIFT),
        WorldUp::Colmap => cgmath::Vector3::new(0.0, -BATTLE_HIT_CENTER_LIFT, 0.0),
    };
    let drone_positions: Vec<(cgmath::Vector3<f32>, f32)> = players.0.iter()
        .map(|p| {
            (cgmath::Vector3::new(p.drone.x, p.drone.y, p.drone.z) + lift, BATTLE_HIT_RADIUS)
        })
        .collect();
    let (hit_remove, events) = battle.check_hits(&drone_positions);
    if !hit_remove.is_empty() {
        battle.remove_projectiles(&hit_remove);
    }
    // Tick down hit-marker + hurt flashes, then re-arm on each landed hit.
    for m in &mut battle.hit_markers {
        *m = (*m - dt).max(0.0);
    }
    for f in &mut battle.hurt_flashes {
        *f = (*f - dt).max(0.0);
    }
    for ev in &events {
        if let Some(k) = ev.killer {
            if let Some(m) = battle.hit_markers.get_mut(k) {
                *m = HIT_MARKER_SECS;
            }
        }
        if let Some(f) = battle.hurt_flashes.get_mut(ev.target_player) {
            *f = HURT_FLASH_SECS;
        }
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

/// Snap each player's translucent hit-sphere onto their drone and drive the "got hit" flash:
/// idle = faint cyan shell; hit = red + much more opaque, fading back over HURT_FLASH_SECS.
/// Spheres for player slots beyond the active player count stay hidden. Battle mode only
/// (these entities are only ever spawned by `setup_battle`).
fn battle_hit_sphere_system(
    battle: Res<BattleState>,
    players: Res<Players>,
    config: Res<CurrentSceneConfig>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut shield_spheres: ResMut<ShieldWorldSpheres>,
    mut spheres: Query<(&HitSphereVisual, &mut Transform, &mut Visibility, &MeshMaterial3d<StandardMaterial>)>,
) {
    // Same world-up lift as the hit test, so the visual shell IS the judgement volume.
    let lift = match config.0.world_up {
        WorldUp::Zup => Vec3::new(0.0, 0.0, BATTLE_HIT_CENTER_LIFT),
        WorldUp::Colmap => Vec3::new(0.0, -BATTLE_HIT_CENTER_LIFT, 0.0),
    };
    shield_spheres.0.clear();
    for (hs, mut tf, mut vis, mat) in &mut spheres {
        let Some(p) = players.0.get(hs.player) else {
            *vis = Visibility::Hidden; // inactive slot (e.g. P2 in single player)
            continue;
        };
        // Dead players' spheres disappear with them.
        let alive = battle.player_health.get(hs.player).map_or(true, |h| h.is_alive());
        *vis = if alive { Visibility::Inherited } else { Visibility::Hidden };
        let d = &p.drone;
        tf.translation = Vec3::new(d.x, d.y, d.z) + lift;
        if alive {
            // 1.15 margin covers the 12% hit pulse; consumed by the splat depth-pass scissor.
            shield_spheres.0.push((hs.player, tf.translation, BATTLE_HIT_RADIUS * 1.15));
        }
        // "Got hit" feedback: bubble blends cyan→red, opacity 0.3→0.6, and pops out ~12%
        // (impact pulse), all easing back over HURT_FLASH_SECS.
        let t = battle
            .hurt_flashes
            .get(hs.player)
            .map_or(0.0, |f| (f / HURT_FLASH_SECS).clamp(0.0, 1.0));
        tf.scale = Vec3::splat(1.0 + 0.12 * t);
        if let Some(m) = materials.get_mut(&mat.0) {
            let idle = (0.3, 0.8, 1.0, 0.3); // rgba
            let hit = (1.0, 0.15, 0.1, 0.6);
            m.base_color = Color::srgba(
                idle.0 + (hit.0 - idle.0) * t,
                idle.1 + (hit.1 - idle.1) * t,
                idle.2 + (hit.2 - idle.2) * t,
                idle.3 + (hit.3 - idle.3) * t,
            );
        }
    }
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

/// Battle HUD: each player's own health bar + hit-marker flash, drawn in their viewport half
/// (full window in single player). Hidden once the match is over (finish overlay takes over).
fn battle_hud_system(
    mut contexts: bevy_egui::EguiContexts,
    battle: Res<BattleState>,
    players: Res<Players>,
    windows: Query<&Window>,
) -> Result {
    if battle.winner.is_some() || players.0.is_empty() || battle.player_health.is_empty() {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    if players.0.len() == 1 {
        let h = &battle.player_health[0];
        let marker = battle.hit_markers.first().copied().unwrap_or(0.0);
        let hurt = battle.hurt_flashes.first().copied().unwrap_or(0.0) / HURT_FLASH_SECS;
        crate::hud::draw_battle_hud(ctx, h.current, h.pct(), marker, hurt, None, None);
    } else {
        // Window uses scale_factor_override(1.0) → egui points == physical px == the viewports.
        let Ok(win) = windows.single() else {
            return Ok(());
        };
        let (w, hh) = (win.width(), win.height());
        let half = hh * 0.5;
        let labels = ["P1", "P2"];
        for (i, _) in players.0.iter().enumerate().take(2) {
            let Some(h) = battle.player_health.get(i) else {
                continue;
            };
            let marker = battle.hit_markers.get(i).copied().unwrap_or(0.0);
            let hurt = battle.hurt_flashes.get(i).copied().unwrap_or(0.0) / HURT_FLASH_SECS;
            let rect = egui::Rect::from_min_size(
                egui::pos2(0.0, half * i as f32),
                egui::vec2(w, half),
            );
            crate::hud::draw_battle_hud(ctx, h.current, h.pct(), marker, hurt, Some(labels[i]), Some(rect));
        }
    }
    Ok(())
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
