//! Gate course with lap timing — ported from `native/src/gates.rs` (itself from `src/gates.js`).
//! Handles gate layout (Catmull-Rom tangent orientation), pass-through detection, and lap timing.
//! Rendering lives in `gate_render` (Bevy meshes); this module is pure logic (cgmath only).
//!
//! Two deviations from native: (1) `world_up` is configurable (native hardcoded Y-up) so the gate
//! frames orient correctly in our Z-up world; (2) the `FnMut` callbacks are replaced by a returned
//! `Vec<GateEvent>` from `update`, which fits Bevy systems (the caller reacts to events for SFX/HUD).

use cgmath::{InnerSpace, Matrix4, SquareMatrix, Vector3, Vector4};

use crate::spline;

/// Format milliseconds as MM:SS.mmm
pub fn format_lap(ms: f64) -> String {
    if !ms.is_finite() || ms < 0.0 {
        return "--:--.---".into();
    }
    let total = ms.round() as u64;
    let minutes = total / 60000;
    let seconds = (total % 60000) / 1000;
    let millis = total % 1000;
    format!("{:02}:{:02}.{:03}", minutes, seconds, millis)
}

/// What happened to the course during one `update` (returned for the caller to react to: SFX, HUD).
#[derive(Debug, Clone, Copy)]
pub enum GateEvent {
    /// Drone cleanly crossed gate `index` (of `total`) in order.
    Passed { index: usize, total: usize },
    /// A lap finished (crossing gate 0). `is_best` if it beat the previous best.
    LapComplete { lap_ms: f64, is_best: bool },
}

#[derive(Clone)]
pub struct Gate {
    pub pos: Vector3<f32>,
    pub travel_dir: Vector3<f32>,
    pub passed: bool,
    /// Inverse world transform for the plane-crossing test (gate-local space).
    pub inv_world: Matrix4<f32>,
}

#[derive(Clone)]
pub struct GateCourse {
    pub gates: Vec<Gate>,
    pub next_gate_idx: usize,
    pub gate_size: f32,
    /// World up axis used to build each gate's frame basis (Z-up = `(0,0,1)` here; native used Y-up).
    pub world_up: Vector3<f32>,

    // Lap timing
    pub lap_start: Option<f64>, // ms timestamp
    pub lap_count: u32,
    pub best_lap_ms: Option<f64>,
    pub current_lap_ms: f64,

    // State
    prev_drone_pos: Option<Vector3<f32>>,
    pulse_time: f32,
    pub visible: bool,
}

impl GateCourse {
    pub fn new() -> Self {
        Self {
            gates: Vec::new(),
            next_gate_idx: 0,
            gate_size: 1.5,
            world_up: Vector3::new(0.0, 0.0, 1.0),
            lap_start: None,
            lap_count: 0,
            best_lap_ms: None,
            current_lap_ms: 0.0,
            prev_drone_pos: None,
            pulse_time: 0.0,
            visible: false,
        }
    }

    /// Rebuild the course from control points (calls spline for tangents). `world_up` orients the
    /// gate frames (perpendicular to travel direction, "up" in the world's up plane).
    pub fn rebuild(&mut self, points: &[Vector3<f32>], world_up: Vector3<f32>) {
        self.gates.clear();
        self.next_gate_idx = 0;
        self.prev_drone_pos = None;
        self.pulse_time = 0.0;
        self.world_up = world_up;

        if points.len() < 3 {
            return;
        }

        for i in 0..points.len() {
            let pos = points[i];
            let td = spline::tangent_at_point(points, i);

            // Build gate orientation: fwd = travel_dir, right = cross(worldUp, fwd), up = cross(fwd, right)
            let fwd = td;
            let world_up = self.world_up;
            let mut right = world_up.cross(fwd);
            if right.magnitude2() < 1e-6 {
                right = if fwd.x.abs() < 0.9 {
                    Vector3::unit_x()
                } else {
                    Vector3::unit_y()
                };
                let dot = right.dot(fwd);
                right = right - fwd * dot;
            }
            right = right.normalize();
            let up = fwd.cross(right).normalize();

            // Column-major world transform
            let world = Matrix4::new(
                right.x, right.y, right.z, 0.0,
                up.x, up.y, up.z, 0.0,
                fwd.x, fwd.y, fwd.z, 0.0,
                pos.x, pos.y, pos.z, 1.0,
            );
            let inv_world = world.invert().unwrap_or(Matrix4::identity());

            self.gates.push(Gate {
                pos,
                travel_dir: td,
                passed: false,
                inv_world,
            });
        }

        self.reset_lap();
    }

    /// Main per-frame update — call with drone position and current time (ms). Returns the events
    /// (gate passes / lap completions) that occurred this frame.
    pub fn update(&mut self, dt: f32, drone_pos: Vector3<f32>, now_ms: f64) -> Vec<GateEvent> {
        let mut events = Vec::new();
        if self.gates.is_empty() || !self.visible {
            self.current_lap_ms = 0.0;
            return events;
        }

        self.pulse_time += dt;

        // Live lap clock
        if let Some(start) = self.lap_start {
            self.current_lap_ms = now_ms - start;
        }

        if let Some(prev) = self.prev_drone_pos {
            // Check all gates for crossing
            let crossings: Vec<usize> = (0..self.gates.len())
                .filter(|&i| self.check_cross(i, prev, drone_pos))
                .collect();
            for i in crossings {
                self.handle_crossing(i, now_ms, &mut events);
            }
        }
        self.prev_drone_pos = Some(drone_pos);
        events
    }

    fn check_cross(&self, gate_idx: usize, prev: Vector3<f32>, curr: Vector3<f32>) -> bool {
        let gate = &self.gates[gate_idx];
        let inv = &gate.inv_world;

        // Transform to gate-local space
        let p0 = inv * Vector4::new(prev.x, prev.y, prev.z, 1.0);
        let p1 = inv * Vector4::new(curr.x, curr.y, curr.z, 1.0);

        // Check Z-plane crossing
        if p0.z * p1.z > 0.0 {
            return false;
        }
        let denom = p0.z - p1.z;
        if denom.abs() < 1e-9 {
            return false;
        }

        let t = p0.z / denom;
        let hx = p0.x + (p1.x - p0.x) * t;
        let hy = p0.y + (p1.y - p0.y) * t;
        let half = self.gate_size * 0.5;
        hx.abs() <= half && hy.abs() <= half
    }

    fn handle_crossing(&mut self, i: usize, now: f64, events: &mut Vec<GateEvent>) {
        if i != self.next_gate_idx {
            return;
        }

        if i == 0 {
            // Gate 0 is the lap boundary
            if let Some(start) = self.lap_start {
                let lap_ms = now - start;
                self.lap_count += 1;
                let is_best = self.best_lap_ms.map_or(true, |b| lap_ms < b);
                if is_best {
                    self.best_lap_ms = Some(lap_ms);
                }
                events.push(GateEvent::LapComplete { lap_ms, is_best });
            }
            self.begin_lap(now);
        }

        self.mark_passed(i, events);
    }

    fn mark_passed(&mut self, i: usize, events: &mut Vec<GateEvent>) {
        if self.gates[i].passed {
            return;
        }
        self.gates[i].passed = true;

        let n = self.gates.len();
        let mut k = (i + 1) % n;
        for _ in 0..n {
            if !self.gates[k].passed {
                break;
            }
            k = (k + 1) % n;
        }
        self.next_gate_idx = k;

        events.push(GateEvent::Passed { index: i, total: n });
    }

    fn begin_lap(&mut self, now: f64) {
        for g in &mut self.gates {
            g.passed = false;
        }
        self.next_gate_idx = 0;
        self.lap_start = Some(now);
        self.current_lap_ms = 0.0;
    }

    pub fn reset_lap(&mut self) {
        for g in &mut self.gates {
            g.passed = false;
        }
        self.next_gate_idx = 0;
        self.lap_start = None;
        self.current_lap_ms = 0.0;
        self.prev_drone_pos = None;
    }

    pub fn passed_count(&self) -> usize {
        self.gates.iter().filter(|g| g.passed).count()
    }

    pub fn status_text(&self) -> String {
        if self.gates.is_empty() {
            return "no path drawn".into();
        }
        let gates = format!("{} gates", self.gates.len());
        match self.best_lap_ms {
            Some(ms) => format!("{} · best {}", gates, format_lap(ms)),
            None => gates,
        }
    }

    /// Pulse scale for the next gate (for rendering).
    pub fn pulse_scale(&self) -> f32 {
        1.0 + 0.08 * (self.pulse_time * 0.8 * std::f32::consts::PI * 2.0).sin()
    }
}
