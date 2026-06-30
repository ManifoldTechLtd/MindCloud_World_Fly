//! Battle mode core logic: beam projectiles, weapon state, and battle state.
//!
//! Pure data structures + logic (no Bevy deps except for the Resource derive on
//! BattleState, which is used by battle_plugin). The Bevy integration (spawning,
//! system scheduling, rendering) lives in battle_plugin.rs.

use bevy::prelude::Resource;
use cgmath::{InnerSpace, Vector3};

/// Who fired a projectile — used for hit detection (don't hit the shooter).
#[derive(Clone, Copy, Debug)]
pub enum ProjectileOwner {
    Player(usize),
    Npc,
}

/// A flying beam projectile. Visualized as a stretched ellipsoid by the plugin.
#[derive(Clone, Debug)]
pub struct Projectile {
    pub pos: Vector3<f32>,
    /// Constant velocity (direction × speed). No gravity.
    pub vel: Vector3<f32>,
    pub owner: ProjectileOwner,
    /// Seconds remaining before auto-despawn.
    pub lifetime: f32,
    pub damage: f32,
    /// Collision radius for hit detection (world units).
    pub radius: f32,
}

/// Per-player weapon state: cooldown + projectile params.
#[derive(Clone, Debug)]
pub struct WeaponState {
    pub cooldown_remaining: f32,
    pub cooldown_duration: f32,
    pub projectile_speed: f32,
    pub projectile_damage: f32,
    pub projectile_lifetime: f32,
    pub projectile_radius: f32,
}

impl Default for WeaponState {
    fn default() -> Self {
        Self {
            cooldown_remaining: 0.0,
            cooldown_duration: 0.15,     // 300 RPM
            projectile_speed: 70.0,      // m/s
            projectile_damage: 15.0,
            projectile_lifetime: 3.0,   // seconds
            projectile_radius: 0.2,     // collision radius
        }
    }
}

/// Global battle state resource. Created on entering Playing when GameType::Battle.
#[derive(Resource, Default)]
pub struct BattleState {
    /// All live projectiles in flight.
    pub projectiles: Vec<Projectile>,
    /// One weapon per player (P1=0, P2=1).
    pub weapons: Vec<WeaponState>,
}

impl BattleState {
    /// Advance projectile physics: move, check lifetime, check octree collision.
    /// Returns indices of projectiles to remove (expired or collided).
    pub fn update_projectiles(
        &mut self,
        dt: f32,
        octree: Option<&crate::collision::Octree>,
    ) -> Vec<usize> {
        let mut to_remove = Vec::new();
        for (i, p) in self.projectiles.iter_mut().enumerate() {
            // Move
            p.pos.x += p.vel.x * dt;
            p.pos.y += p.vel.y * dt;
            p.pos.z += p.vel.z * dt;
            // Lifetime
            p.lifetime -= dt;
            if p.lifetime <= 0.0 {
                to_remove.push(i);
                continue;
            }
            // Octree collision
            if let Some(octree) = octree {
                let hits = octree.query_sphere(p.pos.x, p.pos.y, p.pos.z, p.radius);
                if !hits.is_empty() {
                    to_remove.push(i);
                }
            }
        }
        to_remove
    }

    /// Remove projectiles by indices (descending order to avoid shift issues).
    pub fn remove_projectiles(&mut self, indices: &[usize]) {
        for &i in indices.iter().rev() {
            self.projectiles.swap_remove(i);
        }
    }

    /// Try to fire a projectile from the given position/direction for player `player_idx`.
    /// Returns true if fired, false if on cooldown.
    pub fn try_fire(
        &mut self,
        player_idx: usize,
        pos: Vector3<f32>,
        forward_dir: Vector3<f32>,
    ) -> bool {
        let weapon = &mut self.weapons[player_idx];
        if weapon.cooldown_remaining > 0.0 {
            return false;
        }
        weapon.cooldown_remaining = weapon.cooldown_duration;
        let dir = forward_dir.normalize();
        let speed = weapon.projectile_speed;
        self.projectiles.push(Projectile {
            pos,
            vel: Vector3::new(dir.x * speed, dir.y * speed, dir.z * speed),
            owner: ProjectileOwner::Player(player_idx),
            lifetime: weapon.projectile_lifetime,
            damage: weapon.projectile_damage,
            radius: weapon.projectile_radius,
        });
        true
    }

    /// Tick weapon cooldowns.
    pub fn tick_cooldowns(&mut self, dt: f32) {
        for w in &mut self.weapons {
            if w.cooldown_remaining > 0.0 {
                w.cooldown_remaining = (w.cooldown_remaining - dt).max(0.0);
            }
        }
    }
}
