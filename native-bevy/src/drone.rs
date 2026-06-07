//! Drone physics v4 — NED body frame + quaternion rate integration.
//! Ported VERBATIM from `native/src/drone.rs` (pure `cgmath`, no winit/wgpu deps). The only
//! changes: `crate::app_state::WorldUp` → imported `WorldUp`, native's per-frame `log` debug spam
//! removed, the NaN guard uses `bevy::log::warn!`, and the keyboard `KeyState` (native had it in
//! main.rs) lives here. The Bevy integration (spawning, camera, input) is in `main.rs`.
//!
//! Body coordinate system (NED):
//!   +X = Forward (camera optical axis when tilt=0), +Y = Right, +Z = Down. Thrust = body -Z (up).
//! World systems: Z-up (gravity -Z, horiz XY) / COLMAP (+Y = down, gravity +Y, horiz XZ).
//! Convention: `orientation` = q_bw (body→world). `Matrix3::from(orientation)` COLUMNS are body
//! axes in world. Kinematics: q̇ = ½ q ⊗ ω (right-multiply by body-frame angular velocity).

use cgmath::{InnerSpace, Matrix3, Quaternion, Rad, Rotation3, Vector3};

use crate::app_state::WorldUp;

const DEG2RAD: f32 = std::f32::consts::PI / 180.0;
const RAD2DEG: f32 = 180.0 / std::f32::consts::PI;
const G: f32 = 9.81;
const AIR_DENSITY: f32 = 1.225;

fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    v.max(lo).min(hi)
}

/// Returns the spawn orientation quaternion (q_bw: body→world) for the given world coordinate system.
fn ned_identity(world_up: WorldUp) -> Quaternion<f32> {
    match world_up {
        WorldUp::Zup => {
            // body+X→world+Y, body+Y→world+X, body+Z→world-Z. Thrust (body -Z) → world +Z (up).
            Quaternion::new(0.0, -std::f32::consts::FRAC_1_SQRT_2, -std::f32::consts::FRAC_1_SQRT_2, 0.0)
        }
        WorldUp::Colmap => {
            // body+X→world+X, body+Y→world(0,0,-1), body+Z→world(0,1,0). Thrust → world -Y (up).
            Quaternion::new(std::f32::consts::FRAC_1_SQRT_2, -std::f32::consts::FRAC_1_SQRT_2, 0.0, 0.0)
        }
    }
}

// ---- Flight mode ----
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FlightMode {
    Fpv,
    Drone,
}

// ---- Input ----
pub struct DroneInput {
    pub roll: f32,      // -1..1
    pub pitch: f32,     // -1..1
    pub throttle: f32,  // -1..1
    pub yaw: f32,       // -1..1
    pub armed: bool,
    pub boost: bool,
    /// Per-axis rate multipliers (from controller settings): [roll, pitch, yaw]
    pub rates: [f32; 3],
}

impl Default for DroneInput {
    fn default() -> Self {
        Self { roll: 0.0, pitch: 0.0, throttle: -1.0, yaw: 0.0, armed: false, boost: false, rates: [1.0, 1.0, 1.0] }
    }
}

/// Keyboard fallback for player 1 (ported from native `main.rs::KeyState`).
/// Arrows = roll/pitch, W/S = throttle, A/D = yaw.
#[derive(Default)]
pub struct KeyState {
    pub w: bool, pub s: bool, pub a: bool, pub d: bool,
    pub up: bool, pub down: bool, pub left: bool, pub right: bool,
}

impl KeyState {
    pub fn to_input(&self, armed: bool) -> DroneInput {
        // Convention: right key = roll right = negative (body Z+ = roll left)
        DroneInput {
            roll: if self.right { -1. } else if self.left { 1. } else { 0. },
            pitch: if self.up { -1. } else if self.down { 1. } else { 0. },
            throttle: if self.w { 0.5 } else if self.s { -1. } else { -0.2 },
            yaw: if self.d { 1. } else if self.a { -1. } else { 0. },
            armed, boost: false, rates: [1., 1., 1.],
        }
    }
}

// ---- Drone ----
pub struct Drone {
    // ---- Geometry ----
    pub drone_size: f32,

    // ---- State ----
    pub x: f32, pub y: f32, pub z: f32,
    pub vx: f32, pub vy: f32, pub vz: f32,

    /// Quaternion orientation (single source of truth)
    pub orientation: Quaternion<f32>,

    /// Angular velocity in body frame (deg/s)
    pub pitch_rate: f32,
    pub roll_rate: f32,
    pub yaw_rate: f32,

    /// Euler angles (derived from orientation each frame, for HUD)
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    /// Yaw-independent body tilt for OSD artificial horizon
    pub body_pitch: f32,
    pub body_roll: f32,

    // ---- World coordinate system ----
    pub world_up: WorldUp,

    // ---- Tunable parameters ----
    pub flight_mode: FlightMode,
    prev_flight_mode: FlightMode,
    pub mass: f32,           // grams
    pub max_thrust: f32,     // grams-force
    pub drag_cd: f32,        // drag coefficient
    pub drag_area: f32,      // frontal area (m²)

    pub max_pitch_rate: f32, // deg/s
    pub max_roll_rate: f32,
    pub max_yaw_rate: f32,
    pub drone_max_yaw_rate: f32,

    pub drone_max_angle: f32,
    pub drone_angle_rate: f32,
    pub drone_max_vspeed: f32,
    pub drone_max_speed: f32,

    // Cascaded PID gains
    pub drone_pos_kp: f32, pub drone_pos_ki: f32, pub drone_pos_kd: f32,
    pub drone_vel_kp: f32, pub drone_vel_ki: f32, pub drone_vel_kd: f32,
    pub drone_alt_kp: f32, pub drone_alt_ki: f32, pub drone_alt_kd: f32,

    // Position-hold setpoints
    target_x: f32, target_y: f32, target_z: f32,
    // Smoothed attitude targets
    smooth_target_pitch: f32, smooth_target_roll: f32,
    // Integral accumulators (position loop)
    pos_int_x: f32, pos_int_y: f32, pos_int_z: f32,
    // Integral accumulators (velocity loop)
    vel_int_x: f32, vel_int_y: f32, vel_int_z: f32,
    // Previous errors for derivative
    prev_pos_err_x: f32, prev_pos_err_y: f32, prev_pos_err_z: f32,
    prev_vel_err_x: f32, prev_vel_err_y: f32, prev_vel_err_z: f32,
    // Filtered derivatives
    filt_pos_derr_x: f32, filt_pos_derr_y: f32, filt_pos_derr_z: f32,
    filt_vel_derr_x: f32, filt_vel_derr_y: f32, filt_vel_derr_z: f32,
    // Anti-windup limits
    pos_int_max: f32,
    vel_int_max: f32,

    pub angular_drag: f32,
    pub collision_radius: f32,
    pub bounce_damping: f32,

    /// FPV position lock: true after arm until throttle exceeds 20%
    pub fpv_pos_locked: bool,

    // ---- Output state ----
    pub is_colliding: bool,
    pub collision_intensity: f32,
    pub speed: f32,
    pub vertical_speed: f32,
    pub thrust_output: f32,

    /// Camera mount angle (degrees). FPV: fixed; Drone: live tilt.
    pub camera_mount_angle: f32,
    pub camera_tilt_angle: f32,

    // Spawn
    spawn_x: f32, spawn_y: f32, spawn_z: f32,
    pub spawn_heading: f32,
}

impl Drone {
    pub fn new() -> Self {
        Self {
            drone_size: 0.3,
            x: 0.0, y: 2.0, z: 0.0,
            vx: 0.0, vy: 0.0, vz: 0.0,
            orientation: ned_identity(WorldUp::Zup),
            pitch_rate: 0.0, roll_rate: 0.0, yaw_rate: 0.0,
            pitch: 0.0, roll: 0.0, yaw: 0.0,
            body_pitch: 0.0, body_roll: 0.0,

            world_up: WorldUp::Zup,

            flight_mode: FlightMode::Fpv,
            prev_flight_mode: FlightMode::Fpv,
            mass: 500.0,
            max_thrust: 1000.0,
            drag_cd: 1.0,
            drag_area: 0.01,

            max_pitch_rate: 220.0,
            max_roll_rate: 220.0,
            max_yaw_rate: 120.0,
            drone_max_yaw_rate: 60.0,

            drone_max_angle: 30.0,
            drone_angle_rate: 150.0,
            drone_max_vspeed: 3.0,
            drone_max_speed: 5.0,

            drone_pos_kp: 2.0, drone_pos_ki: 0.3, drone_pos_kd: 0.1,
            drone_vel_kp: 3.0, drone_vel_ki: 1.0, drone_vel_kd: 0.05,
            drone_alt_kp: 4.0, drone_alt_ki: 2.0, drone_alt_kd: 0.1,

            target_x: 0.0, target_y: 2.0, target_z: 0.0,
            smooth_target_pitch: 0.0, smooth_target_roll: 0.0,
            pos_int_x: 0.0, pos_int_y: 0.0, pos_int_z: 0.0,
            vel_int_x: 0.0, vel_int_y: 0.0, vel_int_z: 0.0,
            prev_pos_err_x: 0.0, prev_pos_err_y: 0.0, prev_pos_err_z: 0.0,
            prev_vel_err_x: 0.0, prev_vel_err_y: 0.0, prev_vel_err_z: 0.0,
            filt_pos_derr_x: 0.0, filt_pos_derr_y: 0.0, filt_pos_derr_z: 0.0,
            filt_vel_derr_x: 0.0, filt_vel_derr_y: 0.0, filt_vel_derr_z: 0.0,
            pos_int_max: 5.0,
            vel_int_max: 15.0,

            angular_drag: 8.0,
            collision_radius: 0.3,
            bounce_damping: 0.3,
            fpv_pos_locked: true,

            is_colliding: false,
            collision_intensity: 0.0,
            speed: 0.0,
            vertical_speed: 0.0,
            thrust_output: 0.0,

            camera_mount_angle: 30.0,
            camera_tilt_angle: 0.0,

            spawn_x: 0.0, spawn_y: 2.0, spawn_z: 0.0,
            spawn_heading: 0.0,
        }
    }

    pub fn set_spawn_point(&mut self, x: f32, y: f32, z: f32) {
        self.spawn_x = x; self.spawn_y = y; self.spawn_z = z;
        self.reset_to_spawn();
    }

    pub fn reset_to_spawn(&mut self) {
        self.x = self.spawn_x; self.y = self.spawn_y; self.z = self.spawn_z;
        self.vx = 0.0; self.vy = 0.0; self.vz = 0.0;
        self.orientation = ned_identity(self.world_up);
        self.pitch_rate = 0.0; self.roll_rate = 0.0; self.yaw_rate = 0.0;
        self.pitch = 0.0; self.roll = 0.0; self.yaw = 0.0;
        self.is_colliding = false;
        self.collision_intensity = 0.0;
        self.thrust_output = 0.0;
        self.fpv_pos_locked = true;
        self.target_x = self.spawn_x; self.target_y = self.spawn_y; self.target_z = self.spawn_z;
        self.clear_pid_state();
        self.apply_spawn_heading(self.spawn_heading);
    }

    /// Rotate the current orientation by heading_deg around the world up axis.
    pub fn apply_spawn_heading(&mut self, heading_deg: f32) {
        if heading_deg.abs() < 0.01 { return; }
        // Heading = yaw = rotation around body+Z (down axis in NED).
        //   Z-up:   body+Z → world-Z, RH around world-Z is CW from above → +angle matches orbit.
        //   COLMAP: body+Z → world+Y, RH around world+Y is CCW from above(-Y) → negate angle.
        let angle = match self.world_up {
            WorldUp::Zup => heading_deg * DEG2RAD,
            WorldUp::Colmap => -heading_deg * DEG2RAD,
        };
        let yaw_q = Quaternion::from_axis_angle(Vector3::unit_z(), Rad(angle));
        self.orientation = (self.orientation * yaw_q).normalize();
    }

    pub fn reset(&mut self, x: f32, y: f32, z: f32) {
        self.spawn_x = x; self.spawn_y = y; self.spawn_z = z;
        self.reset_to_spawn();
    }

    /// Pin position + linear velocity to the spawn point while leaving attitude (orientation /
    /// angular rates) untouched — used during the race countdown so a pilot can aim their drone in
    /// place but cannot leave the start line until GO.
    pub fn lock_position(&mut self) {
        self.x = self.spawn_x; self.y = self.spawn_y; self.z = self.spawn_z;
        self.vx = 0.0; self.vy = 0.0; self.vz = 0.0;
    }

    /// Copy the *tunable settings* (not live flight state) from `src` into `self`. Used by the
    /// split-screen settings panel to keep every player's drone on one shared config. The field set
    /// mirrors `persistence::DroneSettings` plus `flight_mode` (the panel's upper sections).
    pub fn copy_settings_from(&mut self, src: &Drone) {
        self.flight_mode = src.flight_mode;
        self.mass = src.mass;
        self.max_thrust = src.max_thrust;
        self.drag_cd = src.drag_cd;
        self.drag_area = src.drag_area;
        self.collision_radius = src.collision_radius;
        self.drone_size = src.drone_size;
        self.camera_mount_angle = src.camera_mount_angle;
        self.max_pitch_rate = src.max_pitch_rate;
        self.max_roll_rate = src.max_roll_rate;
        self.max_yaw_rate = src.max_yaw_rate;
        self.drone_pos_kp = src.drone_pos_kp;
        self.drone_pos_ki = src.drone_pos_ki;
        self.drone_pos_kd = src.drone_pos_kd;
        self.drone_vel_kp = src.drone_vel_kp;
        self.drone_vel_ki = src.drone_vel_ki;
        self.drone_vel_kd = src.drone_vel_kd;
        self.drone_alt_kp = src.drone_alt_kp;
        self.drone_alt_ki = src.drone_alt_ki;
        self.drone_alt_kd = src.drone_alt_kd;
    }

    fn clear_pid_state(&mut self) {
        self.pos_int_x = 0.0; self.pos_int_y = 0.0; self.pos_int_z = 0.0;
        self.vel_int_x = 0.0; self.vel_int_y = 0.0; self.vel_int_z = 0.0;
        self.prev_pos_err_x = 0.0; self.prev_pos_err_y = 0.0; self.prev_pos_err_z = 0.0;
        self.prev_vel_err_x = 0.0; self.prev_vel_err_y = 0.0; self.prev_vel_err_z = 0.0;
        self.filt_pos_derr_x = 0.0; self.filt_pos_derr_y = 0.0; self.filt_pos_derr_z = 0.0;
        self.filt_vel_derr_x = 0.0; self.filt_vel_derr_y = 0.0; self.filt_vel_derr_z = 0.0;
        self.smooth_target_pitch = 0.0;
        self.smooth_target_roll = 0.0;
    }

    pub fn adjust_camera_tilt(&mut self, delta: f32) {
        self.camera_tilt_angle = clamp(self.camera_tilt_angle + delta, -90.0, 0.0);
    }

    // ---- Main update ----

    pub fn update(&mut self, dt: f32, input: &DroneInput) {
        let dt = dt.min(0.05);

        // Flight-mode transition
        if self.flight_mode != self.prev_flight_mode {
            self.on_flight_mode_changed();
            self.prev_flight_mode = self.flight_mode;
        }

        // Control law
        if !input.armed {
            self.update_disarmed(dt);
            self.fpv_pos_locked = true; // reset lock for next arm
        } else if self.flight_mode == FlightMode::Drone {
            self.control_drone(dt, input);
            self.fpv_pos_locked = false;
        } else {
            // FPV mode: unlock position when throttle > 20% (-1..1 scale, 20% = -0.6)
            if self.fpv_pos_locked && input.throttle > -0.6 {
                self.fpv_pos_locked = false;
            }
            self.control_fpv(dt, input);
        }

        // Extract rotation matrix from orientation (q_bw: body→world).
        // Columns of Matrix3::from(q_bw) are body axes in world.
        let rot = Matrix3::from(self.orientation);
        // Thrust direction = body -Z in world = -col2 = -rot.z
        let thrust_dir = Vector3::new(-rot.z.x, -rot.z.y, -rot.z.z);

        // Skip physics when position-locked (disarmed or FPV pre-throttle)
        let pos_frozen = !input.armed || self.fpv_pos_locked;
        if !pos_frozen {
            // Forces: thrust along body -Z + gravity + quadratic drag
            let mass_g = self.mass.max(1.0);
            let mass_kg = mass_g / 1000.0;
            let thrust_accel = (self.thrust_output / mass_g) * G;
            let mut ax = thrust_dir.x * thrust_accel;
            let mut ay = thrust_dir.y * thrust_accel;
            let mut az = thrust_dir.z * thrust_accel;
            // Gravity along world down axis
            match self.world_up {
                WorldUp::Zup => { az -= G; }     // gravity = (0, 0, -G)
                WorldUp::Colmap => { ay += G; }  // gravity = (0, +G, 0), +Y = down
            }

            // Quadratic drag
            let spd = (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt();
            if spd > 0.001 {
                let drag_force = 0.5 * self.drag_cd * self.drag_area * AIR_DENSITY * spd * spd;
                let drag_accel = drag_force / mass_kg;
                ax -= (self.vx / spd) * drag_accel;
                ay -= (self.vy / spd) * drag_accel;
                az -= (self.vz / spd) * drag_accel;
            }

            // Integrate velocity & position
            self.vx += ax * dt;
            self.vy += ay * dt;
            self.vz += az * dt;
            self.x += self.vx * dt;
            self.y += self.vy * dt;
            self.z += self.vz * dt;
        }

        // NaN guard
        if self.x.is_nan() || self.y.is_nan() || self.z.is_nan() {
            bevy::log::warn!("NaN in drone state, resetting");
            self.reset_to_spawn();
            return;
        }

        // Derive euler angles for HUD
        self.update_euler_from_quat();
        match self.world_up {
            WorldUp::Zup => {
                self.speed = (self.vx * self.vx + self.vy * self.vy).sqrt();
                self.vertical_speed = self.vz;
            }
            WorldUp::Colmap => {
                self.speed = (self.vx * self.vx + self.vz * self.vz).sqrt();
                self.vertical_speed = -self.vy; // -vy because +Y=down
            }
        }
    }

    /// Apply collision response from external collision system.
    pub fn apply_collision(&mut self, normal: Vector3<f32>, penetration: f32) {
        if penetration <= 0.0 { return; }
        self.is_colliding = true;
        self.collision_intensity = (penetration / self.collision_radius).min(1.0);

        let push = penetration + 0.01;
        self.x += normal.x * push;
        self.y += normal.y * push;
        self.z += normal.z * push;

        let v_dot_n = self.vx * normal.x + self.vy * normal.y + self.vz * normal.z;
        if v_dot_n < 0.0 {
            let factor = 1.0 + self.bounce_damping;
            self.vx -= normal.x * v_dot_n * factor;
            self.vy -= normal.y * v_dot_n * factor;
            self.vz -= normal.z * v_dot_n * factor;
        }
        self.vx *= 0.8;
        self.vy *= 0.8;
        self.vz *= 0.8;
    }

    pub fn clear_collision(&mut self) {
        self.is_colliding = false;
        self.collision_intensity = 0.0;
    }

    // ---- Camera ----

    /// body→cam rotation constant (for the camera pipeline which uses q_wb convention).
    /// Maps: body+X→cam+Z, body+Y→cam+X, body+Z→cam+Y
    const CAM_TO_NED: Quaternion<f32> = Quaternion::new(0.5, -0.5, -0.5, -0.5);

    /// Returns the camera (position, q_wb rotation) in web-splat camera convention. `main.rs`
    /// converts this to a Bevy camera transform (the splat node applies the inverse).
    pub fn camera_transform(&self) -> (Vector3<f32>, Quaternion<f32>) {
        let pos = Vector3::new(self.x, self.y, self.z);
        let rot = Matrix3::from(self.orientation);
        // body +X (forward) in world = col0 = rot.x
        let forward = Vector3::new(rot.x.x, rot.x.y, rot.x.z);
        let half_size = self.drone_size * 0.5;

        // Camera mount pitch about the BODY pitch axis (+Y in NED). Positive angle tilts the view
        // UP (FPV cameras mount upward); Drone mode's negative tilt looks down.
        let mount_deg = match self.flight_mode {
            FlightMode::Fpv => self.camera_mount_angle,
            FlightMode::Drone => self.camera_tilt_angle,
        };
        let mount_rad = mount_deg * DEG2RAD;
        // Negate: a right-handed rotation about body +Y pitches forward→down, so we flip the sign to
        // make a positive mount angle look up.
        let tilt_q = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(-mount_rad));
        // Camera pipeline expects q_wb. Convert q_bw→q_wb via conjugate. The tilt is applied in the
        // BODY frame (left of the world→body rotation) so a yawed-but-level drone pitches the view
        // without rolling; CAM_TO_NED then maps the tilted body axes into the camera optical frame.
        let cam_orient = Self::CAM_TO_NED * tilt_q * self.orientation.conjugate();

        let cam_pos = pos + forward * half_size;
        (cam_pos, cam_orient)
    }

    // ---- Orientation helpers ----

    /// Quaternion rate integration: q̇ = ½ q ⊗ ω, q_{k+1} = normalize(q + q̇·dt).
    /// `omega` is body-frame angular velocity in rad/s: [ω_x(roll), ω_y(pitch), ω_z(yaw)].
    fn integrate_orientation(&mut self, omega_x: f32, omega_y: f32, omega_z: f32, dt: f32) {
        let q = self.orientation;
        let omega_q = Quaternion::new(0.0, omega_x, omega_y, omega_z);
        let q_dot = q * omega_q * 0.5;
        let q_new = Quaternion::new(
            q.s + q_dot.s * dt,
            q.v.x + q_dot.v.x * dt,
            q.v.y + q_dot.v.y * dt,
            q.v.z + q_dot.v.z * dt,
        );
        self.orientation = q_new.normalize();
    }

    /// Decompose orientation into yaw, pitch, roll using gravity reference.
    fn decompose_orientation(&self) -> (f32, f32, f32) {
        let rot = Matrix3::from(self.orientation);
        // With body→world convention, COLUMNS are body axes in world:
        let fwd = Vector3::new(rot.x.x, rot.x.y, rot.x.z);   // col0 = body+X (forward)
        let right = Vector3::new(rot.y.x, rot.y.y, rot.y.z); // col1 = body+Y (right)
        let down = Vector3::new(rot.z.x, rot.z.y, rot.z.z);  // col2 = body+Z (down)

        match self.world_up {
            WorldUp::Zup => {
                // Horizontal = XY, vertical = Z. At spawn fwd=(0,1,0).
                let yaw_deg = fwd.x.atan2(fwd.y) * RAD2DEG;
                let body_pitch = fwd.z.asin() * RAD2DEG;
                let body_roll = right.z.atan2(-down.z) * RAD2DEG;
                (yaw_deg, body_pitch, body_roll)
            }
            WorldUp::Colmap => {
                // Horizontal = XZ, vertical = -Y (+Y=down). At spawn fwd=(1,0,0).
                let yaw_deg = fwd.z.atan2(fwd.x) * RAD2DEG;
                let body_pitch = (-fwd.y).asin() * RAD2DEG; // -fwd.y because +Y=down
                let body_roll = (-right.y).atan2(down.y) * RAD2DEG;
                (yaw_deg, body_pitch, body_roll)
            }
        }
    }

    fn update_euler_from_quat(&mut self) {
        let (yaw, bp, br) = self.decompose_orientation();
        self.yaw = yaw;
        self.body_pitch = bp;
        self.body_roll = br;
        self.pitch = bp;
        self.roll = br;
    }

    // ---- Control laws ----

    fn on_flight_mode_changed(&mut self) {
        // Z-up world: target_x = world X, target_z = world Y (horiz), target_y = world Z (alt)
        self.target_x = self.x;
        self.target_z = self.y; // horizontal Y hold
        self.target_y = self.z; // altitude Z hold
        self.clear_pid_state();
    }

    fn update_disarmed(&mut self, dt: f32) {
        self.thrust_output = 0.0;
        self.vx = 0.0; self.vy = 0.0; self.vz = 0.0;

        // Auto-level using same P-controller as Drone mode (target = 0°)
        let (_, body_pitch_deg, body_roll_deg) = self.decompose_orientation();
        let max_rate = 60.0;
        self.pitch_rate = clamp(-body_pitch_deg * 3.0, -max_rate, max_rate);
        self.roll_rate = clamp(body_roll_deg * 3.0, -max_rate, max_rate);

        // Damp yaw
        self.yaw_rate *= (-self.angular_drag * dt).exp();

        let omega_x = self.roll_rate * DEG2RAD;
        let omega_y = self.pitch_rate * DEG2RAD;
        let omega_z = self.yaw_rate * DEG2RAD;
        self.integrate_orientation(omega_x, omega_y, omega_z, dt);
    }

    fn control_fpv(&mut self, dt: f32, input: &DroneInput) {
        let boost = if input.boost { 1.5 } else { 1.0 };
        let [rate_r, rate_p, rate_y] = input.rates;

        // Sticks → target angular rates (deg/s)
        let t_rr = input.roll * self.max_roll_rate * rate_r * boost;
        let t_pr = input.pitch * self.max_pitch_rate * rate_p * boost;
        let t_yr = input.yaw * self.max_yaw_rate * rate_y * boost;

        // Smooth rate tracking
        let s = 1.0 - (-15.0 * dt).exp();
        self.roll_rate += (t_rr - self.roll_rate) * s;
        self.pitch_rate += (t_pr - self.pitch_rate) * s;
        self.yaw_rate += (t_yr - self.yaw_rate) * s;

        // Damp when centered
        let ad = (-self.angular_drag * dt).exp();
        if input.roll.abs() < 0.05 { self.roll_rate *= ad; }
        if input.pitch.abs() < 0.05 { self.pitch_rate *= ad; }
        if input.yaw.abs() < 0.05 { self.yaw_rate *= ad; }

        // Quaternion rate integration: ω in body NED frame (rad/s)
        let omega_x = self.roll_rate * DEG2RAD;
        let omega_y = self.pitch_rate * DEG2RAD;
        let omega_z = self.yaw_rate * DEG2RAD;
        self.integrate_orientation(omega_x, omega_y, omega_z, dt);

        // Throttle → thrust (grams-force)
        if self.fpv_pos_locked {
            self.thrust_output = 0.0;
        } else {
            self.thrust_output = ((input.throttle + 1.0) * 0.5) * self.max_thrust * boost;
        }
    }

    /// Stabilize mode: roll/pitch sticks → target angle, yaw stick → angular rate.
    fn control_drone(&mut self, dt: f32, input: &DroneInput) {
        let boost = if input.boost { 1.5 } else { 1.0 };
        let [rate_r, rate_p, rate_y] = input.rates;
        let max_angle = self.drone_max_angle;
        let yaw_active = input.yaw.abs() > 0.05;

        // ---- 1. Sticks → target tilt angles (degrees) ----
        let target_pitch = input.pitch * max_angle * rate_p;
        let target_roll = -input.roll * max_angle * rate_r;

        // ---- 2. Attitude P-controller ----
        let (_, body_pitch_deg, body_roll_deg) = self.decompose_orientation();
        let pitch_err = target_pitch - body_pitch_deg;
        let roll_err = body_roll_deg - target_roll;

        let max_rate = 60.0; // deg/s max correction rate
        self.pitch_rate = clamp(pitch_err * 3.0, -max_rate, max_rate);
        self.roll_rate = clamp(roll_err * 3.0, -max_rate, max_rate);

        // ---- 3. Yaw: pure rate control ----
        let drone_yaw_max = self.drone_max_yaw_rate * rate_y * boost;
        let t_yr = input.yaw * drone_yaw_max;
        let ys = 1.0 - (-15.0 * dt).exp();
        self.yaw_rate += (t_yr - self.yaw_rate) * ys;
        if !yaw_active {
            self.yaw_rate *= (-self.angular_drag * dt).exp();
        }

        // ---- 4. Integrate orientation ----
        let omega_x = self.roll_rate * DEG2RAD;
        let omega_y = self.pitch_rate * DEG2RAD;
        let omega_z = self.yaw_rate * DEG2RAD;
        self.integrate_orientation(omega_x, omega_y, omega_z, dt);

        // ---- 5. Throttle → thrust with tilt compensation ----
        let thrust_base = ((input.throttle + 1.0) * 0.5) * self.max_thrust * boost;
        let rot = Matrix3::from(self.orientation);
        let cos_t = match self.world_up {
            WorldUp::Zup => (-rot.z.z).max(0.1),
            WorldUp::Colmap => (rot.z.y).max(0.1),
        };
        self.thrust_output = clamp(thrust_base / cos_t, 0.0, self.max_thrust * boost);
    }
}

#[cfg(test)]
mod camera_tilt_tests {
    use super::*;

    /// Level drone (Z-up) yawed by `heading_deg`, FPV camera mounted at `mount_deg`.
    fn fpv_level(heading_deg: f32, mount_deg: f32) -> Drone {
        let mut d = Drone::new();
        d.world_up = WorldUp::Zup;
        d.orientation = ned_identity(WorldUp::Zup);
        d.apply_spawn_heading(heading_deg);
        d.flight_mode = FlightMode::Fpv;
        d.camera_mount_angle = mount_deg;
        d
    }

    /// |a ⋅ b| for two unit quaternions (≈ 1 iff equal up to double-cover sign).
    fn quat_align(a: Quaternion<f32>, b: Quaternion<f32>) -> f32 {
        (a.s * b.s + a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z).abs()
    }

    /// Issue 1: a positive FPV mount angle tilts the *view* UP. The camera forward (`q_wb * +Z`, the
    /// web-splat view axis) is horizontal at mount 0 and gains a world-up (+Z) component once tilted.
    #[test]
    fn positive_mount_looks_up() {
        let fwd0 = fpv_level(0.0, 0.0).camera_transform().1 * Vector3::unit_z();
        assert!(fwd0.z.abs() < 1e-3, "mount 0 should look horizontal, got forward.z={}", fwd0.z);
        let fwd30 = fpv_level(0.0, 30.0).camera_transform().1 * Vector3::unit_z();
        assert!(fwd30.z > 0.1, "mount +30 should look UP (forward.z>0), got forward.z={}", fwd30.z);
    }

    /// Issue 2: the mount tilt is applied in the BODY frame, so the camera rotation it induces
    /// (relative to mount 0) is identical at every heading. The old world-frame tilt made this
    /// heading-dependent, which appeared as image roll when the drone was yawed.
    #[test]
    fn mount_tilt_is_body_frame_so_independent_of_heading() {
        let theta = 30.0;
        let delta_at = |heading: f32| {
            let c0 = fpv_level(heading, 0.0).camera_transform().1;
            let ct = fpv_level(heading, theta).camera_transform().1;
            ct * c0.conjugate()
        };
        let reference = delta_at(0.0);
        for &heading in &[15.0_f32, 45.0, 90.0, 180.0, -60.0, 270.0] {
            let align = quat_align(delta_at(heading), reference);
            assert!(
                align > 1.0 - 1e-4,
                "tilt induced a different camera rotation at heading {heading} (align={align}); \
                 tilt is not body-frame → it rolls the view when yawed"
            );
        }
    }
}
