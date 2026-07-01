//! Battle mode core logic: beam projectiles, weapon state, and battle state.
//!
//! Pure data structures + logic (no Bevy deps except for the Resource derive on
//! BattleState, which is used by battle_plugin). The Bevy integration (spawning,
//! system scheduling, rendering) lives in battle_plugin.rs.

use bevy::prelude::Resource;
use cgmath::{InnerSpace, Vector3};

/// Who fired a projectile — used for hit detection (don't hit the shooter).
#[derive(Clone, Copy, Debug, PartialEq)]
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

/// Health state for a drone or NPC.
#[derive(Clone, Debug)]
pub struct Health {
    pub current: f32,
    pub max: f32,
}

impl Health {
    pub fn new(max: f32) -> Self {
        Self { current: max, max }
    }
    pub fn is_alive(&self) -> bool {
        self.current > 0.0
    }
    pub fn take_damage(&mut self, amount: f32) {
        self.current = (self.current - amount).max(0.0);
    }
    pub fn pct(&self) -> f32 {
        if self.max > 0.0 { self.current / self.max } else { 0.0 }
    }
}

/// Battle winner — set when the match ends.
#[derive(Clone, Copy, Debug)]
pub enum BattleWinner {
    Player(usize),
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
    /// One health per player (P1=0, P2=1). Lazy-init in battle_fire_system.
    pub player_health: Vec<Health>,
    /// Set when the match ends (winner determined).
    pub winner: Option<BattleWinner>,
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

    /// Check projectile-vs-drone hits. `drone_positions` is (pos, collision_radius) per player.
    /// Returns (projectile indices to remove, hit events) — caller logs/scores.
    pub fn check_hits(
        &mut self,
        drone_positions: &[(Vector3<f32>, f32)], // (pos, collision_radius) per player
    ) -> (Vec<usize>, Vec<HitEvent>) {
        let mut to_remove = Vec::new();
        let mut events = Vec::new();
        for (i, proj) in self.projectiles.iter().enumerate() {
            for (player_idx, (pos, radius)) in drone_positions.iter().enumerate() {
                // Don't hit the shooter.
                if proj.owner == ProjectileOwner::Player(player_idx) {
                    continue;
                }
                // Skip dead players.
                if let Some(h) = self.player_health.get(player_idx) {
                    if !h.is_alive() {
                        continue;
                    }
                }
                let dx = proj.pos.x - pos.x;
                let dy = proj.pos.y - pos.y;
                let dz = proj.pos.z - pos.z;
                let dist_sq = dx * dx + dy * dy + dz * dz;
                let hit_dist = proj.radius + radius;
                if dist_sq < hit_dist * hit_dist {
                    to_remove.push(i);
                    events.push(HitEvent {
                        target_player: player_idx,
                        damage: proj.damage,
                        killer: match proj.owner {
                            ProjectileOwner::Player(p) => Some(p),
                            ProjectileOwner::Npc => None,
                        },
                    });
                    break; // one projectile hits one target
                }
            }
        }
        (to_remove, events)
    }

    /// Apply a hit: reduce target health. Returns true if the target died this hit.
    pub fn apply_hit(&mut self, event: &HitEvent) -> bool {
        if event.target_player >= self.player_health.len() {
            return false;
        }
        let health = &mut self.player_health[event.target_player];
        let was_alive = health.is_alive();
        health.take_damage(event.damage);
        was_alive && !health.is_alive() // true if this hit killed them
    }

    /// Check if the match has a winner. PvP: first to kill the other wins.
    pub fn check_win_condition(&mut self) {
        if self.winner.is_some() {
            return;
        }
        for i in 0..self.player_health.len() {
            if !self.player_health[i].is_alive() {
                for j in 0..self.player_health.len() {
                    if j != i && self.player_health[j].is_alive() {
                        self.winner = Some(BattleWinner::Player(j));
                        return;
                    }
                }
            }
        }
    }
}

/// A hit event: projectile hit a player.
#[derive(Clone, Debug)]
pub struct HitEvent {
    pub target_player: usize,
    pub damage: f32,
    pub killer: Option<usize>,
}
