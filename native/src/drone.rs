/// Drone physics v3 — quaternion-based orientation.
///
/// Faithfully ported from src/drone.js.
///
/// All rotations are applied in the drone's BODY frame via quaternion
/// multiplication.  This eliminates Euler-angle cross-coupling: roll is
/// always around the drone's nose-to-tail axis regardless of heading.
///
/// FPV:   sticks → body-frame angular rates, throttle → thrust, no self-leveling
/// Drone: sticks → velocity command → position setpoint, cascaded PID

use cgmath::{InnerSpace, Matrix3, Quaternion, Rad, Rotation, Rotation3, SquareMatrix, Vector3};

const DEG2RAD: f32 = std::f32::consts::PI / 180.0;
const RAD2DEG: f32 = 180.0 / std::f32::consts::PI;
const G: f32 = 9.81;
const AIR_DENSITY: f32 = 1.225;

fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    v.max(lo).min(hi)
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
    /// Per-axis rate multipliers (from controller settings)
    pub rates: [f32; 3], // [roll, pitch, yaw]
}

impl Default for DroneInput {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            throttle: -1.0,
            yaw: 0.0,
            armed: false,
            boost: false,
            rates: [1.0, 1.0, 1.0],
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
}

impl Drone {
    pub fn new() -> Self {
        Self {
            drone_size: 0.3,
            x: 0.0, y: 2.0, z: 0.0,
            vx: 0.0, vy: 0.0, vz: 0.0,
            orientation: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            pitch_rate: 0.0, roll_rate: 0.0, yaw_rate: 0.0,
            pitch: 0.0, roll: 0.0, yaw: 0.0,
            body_pitch: 0.0, body_roll: 0.0,

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

            is_colliding: false,
            collision_intensity: 0.0,
            speed: 0.0,
            vertical_speed: 0.0,
            thrust_output: 0.0,

            camera_mount_angle: 30.0,
            camera_tilt_angle: 0.0,

            spawn_x: 0.0, spawn_y: 2.0, spawn_z: 0.0,
        }
    }

    pub fn set_spawn_point(&mut self, x: f32, y: f32, z: f32) {
        self.spawn_x = x; self.spawn_y = y; self.spawn_z = z;
        self.reset_to_spawn();
    }

    pub fn reset_to_spawn(&mut self) {
        self.x = self.spawn_x; self.y = self.spawn_y; self.z = self.spawn_z;
        self.vx = 0.0; self.vy = 0.0; self.vz = 0.0;
        self.orientation = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        self.pitch_rate = 0.0; self.roll_rate = 0.0; self.yaw_rate = 0.0;
        self.pitch = 0.0; self.roll = 0.0; self.yaw = 0.0;
        self.is_colliding = false;
        self.collision_intensity = 0.0;
        self.thrust_output = 0.0;
        self.target_x = self.spawn_x; self.target_y = self.spawn_y; self.target_z = self.spawn_z;
        self.clear_pid_state();
    }

    pub fn reset(&mut self, x: f32, y: f32, z: f32) {
        self.spawn_x = x; self.spawn_y = y; self.spawn_z = z;
        self.reset_to_spawn();
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
        } else if self.flight_mode == FlightMode::Drone {
            self.control_drone(dt, input);
        } else {
            self.control_fpv(dt, input);
        }

        // Extract rotation matrix from orientation
        let rot = Matrix3::from(self.orientation);
        let up = rot.y; // local +Y column

        // Forces: thrust along local up + gravity + quadratic drag
        let mass_g = self.mass.max(1.0);
        let mass_kg = mass_g / 1000.0;
        let thrust_accel = (self.thrust_output / mass_g) * G;
        let mut ax = up.x * thrust_accel;
        let mut ay = up.y * thrust_accel - G;
        let mut az = up.z * thrust_accel;

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

        // NaN guard
        if self.x.is_nan() || self.y.is_nan() || self.z.is_nan() {
            log::warn!("NaN in drone state, resetting");
            self.reset_to_spawn();
            return;
        }

        // Derive euler angles for HUD
        self.update_euler_from_quat();
        self.speed = (self.vx * self.vx + self.vz * self.vz).sqrt();
        self.vertical_speed = self.vy;
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

    pub fn camera_transform(&self) -> (Vector3<f32>, Quaternion<f32>) {
        let pos = Vector3::new(self.x, self.y, self.z);
        let rot = Matrix3::from(self.orientation);
        let forward = -rot.z; // local -Z = forward
        let half_size = self.drone_size * 0.5;

        // Camera mount pitch offset (body-frame X rotation)
        let mount_deg = match self.flight_mode {
            FlightMode::Fpv => self.camera_mount_angle,
            FlightMode::Drone => self.camera_tilt_angle,
        };
        let mount_rad = mount_deg * DEG2RAD;
        let tilt_q = Quaternion::from_axis_angle(Vector3::unit_x(), Rad(mount_rad));
        let cam_orient = self.orientation * tilt_q;

        let cam_pos = pos + forward * half_size;
        (cam_pos, cam_orient)
    }

    // ---- Orientation helpers ----

    fn apply_body_rotation(&mut self, axis: Vector3<f32>, angle_deg: f32) {
        if angle_deg.abs() < 1e-8 { return; }
        let q = Quaternion::from_axis_angle(axis, Rad(angle_deg * DEG2RAD));
        self.orientation = (self.orientation * q).normalize();
    }

    /// Decompose orientation into yaw (world Y) + body tilt (pitch, roll).
    fn decompose_orientation(&self) -> (f32, f32, f32) {
        let rot = Matrix3::from(self.orientation);
        let local_z = rot.z; // local +Z in world

        let yaw_rad = local_z.x.atan2(local_z.z);
        let yaw_deg = yaw_rad * RAD2DEG;

        // Yaw-only quaternion
        let yaw_q = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(yaw_rad));
        // Body tilt = inverse(yawQuat) * orientation
        let tilt_q = yaw_q.invert() * self.orientation;

        // Extract pitch (X) and roll (Z) from tilt quaternion using euler approximation
        let tilt_rot = Matrix3::from(tilt_q);
        let body_pitch = (-tilt_rot.z.y).asin() * RAD2DEG;
        let body_roll = tilt_rot.z.x.atan2(tilt_rot.z.z) * RAD2DEG;

        (yaw_deg, body_pitch, body_roll)
    }

    fn update_euler_from_quat(&mut self) {
        // Standard euler extraction from quaternion
        let rot = Matrix3::from(self.orientation);
        self.pitch = (-rot.z.y).asin() * RAD2DEG;
        self.yaw = rot.z.x.atan2(rot.z.z) * RAD2DEG;
        self.roll = rot.x.y.atan2(rot.y.y) * RAD2DEG;

        let (_, bp, br) = self.decompose_orientation();
        self.body_pitch = bp;
        self.body_roll = br;
    }

    // ---- Control laws ----

    fn on_flight_mode_changed(&mut self) {
        self.target_x = self.x;
        self.target_y = self.y;
        self.target_z = self.z;
        self.clear_pid_state();
    }

    fn update_disarmed(&mut self, dt: f32) {
        self.thrust_output = 0.0;
        let damp = (-self.angular_drag * dt).exp();
        self.pitch_rate *= damp;
        self.roll_rate *= damp;
        self.yaw_rate *= damp;

        // Auto-level toward identity tilt (keep current yaw)
        let (_, body_pitch, body_roll) = self.decompose_orientation();
        let level_speed = 60.0; // deg/s
        let pitch_step = (level_speed * dt).min(body_pitch.abs());
        let roll_step = (level_speed * dt).min(body_roll.abs());

        if pitch_step > 0.01 {
            self.apply_body_rotation(Vector3::unit_x(), -body_pitch.signum() * pitch_step);
        }
        if roll_step > 0.01 {
            self.apply_body_rotation(Vector3::unit_z(), -body_roll.signum() * roll_step);
        }
    }

    fn control_fpv(&mut self, dt: f32, input: &DroneInput) {
        let boost = if input.boost { 1.5 } else { 1.0 };
        let [rate_r, rate_p, rate_y] = input.rates;

        // Sticks → target angular rates
        let t_pr = input.pitch * self.max_pitch_rate * rate_p * boost;
        let t_rr = input.roll * self.max_roll_rate * rate_r * boost;
        let t_yr = input.yaw * self.max_yaw_rate * rate_y * boost;

        // Smooth rate tracking
        let s = 1.0 - (-15.0 * dt).exp();
        self.pitch_rate += (t_pr - self.pitch_rate) * s;
        self.roll_rate += (t_rr - self.roll_rate) * s;
        self.yaw_rate += (t_yr - self.yaw_rate) * s;

        // Damp when centered
        let ad = (-self.angular_drag * dt).exp();
        if input.pitch.abs() < 0.05 { self.pitch_rate *= ad; }
        if input.roll.abs() < 0.05 { self.roll_rate *= ad; }
        if input.yaw.abs() < 0.05 { self.yaw_rate *= ad; }

        // Apply body-frame rotations
        self.apply_body_rotation(Vector3::unit_x(), self.pitch_rate * dt);
        self.apply_body_rotation(Vector3::unit_z(), self.roll_rate * dt);
        self.apply_body_rotation(Vector3::unit_y(), self.yaw_rate * dt);

        // Throttle → thrust (grams-force)
        self.thrust_output = ((input.throttle + 1.0) * 0.5) * self.max_thrust * boost;
    }

    fn control_drone(&mut self, dt: f32, input: &DroneInput) {
        let boost = if input.boost { 1.5 } else { 1.0 };
        let [rate_r, rate_p, rate_y] = input.rates;
        let max_spd = self.drone_max_speed * boost;

        // Body-frame forward (-Z) and right (+X) in world XZ plane
        let rot = Matrix3::from(self.orientation);
        let fwd_x = -rot.z.x;
        let fwd_z = -rot.z.z;
        let right_x = rot.x.x;
        let right_z = rot.x.z;

        let horiz_active = input.pitch.abs() > 0.05 || input.roll.abs() > 0.05;
        let vert_active = input.throttle.abs() > 0.05;
        let yaw_active = input.yaw.abs() > 0.05;

        // ---- Horizontal ----
        let (v_des_x, v_des_z);
        if horiz_active {
            let cmd_fwd = -input.pitch * max_spd * rate_p;
            let cmd_right = input.roll * max_spd * rate_r;
            v_des_x = cmd_fwd * fwd_x + cmd_right * right_x;
            v_des_z = cmd_fwd * fwd_z + cmd_right * right_z;
            self.target_x = self.x;
            self.target_z = self.z;
            self.pos_int_x = 0.0; self.pos_int_z = 0.0;
            self.filt_pos_derr_x = 0.0; self.filt_pos_derr_z = 0.0;
            self.prev_pos_err_x = 0.0; self.prev_pos_err_z = 0.0;
        } else {
            let pos_err_x = self.target_x - self.x;
            let pos_err_z = self.target_z - self.z;
            let pi_max = self.pos_int_max;
            self.pos_int_x = clamp(self.pos_int_x + pos_err_x * dt, -pi_max, pi_max);
            self.pos_int_z = clamp(self.pos_int_z + pos_err_z * dt, -pi_max, pi_max);
            let d_alpha = 1.0 - (-20.0 * dt).exp();
            let raw_x = if dt > 0.0 { (pos_err_x - self.prev_pos_err_x) / dt } else { 0.0 };
            let raw_z = if dt > 0.0 { (pos_err_z - self.prev_pos_err_z) / dt } else { 0.0 };
            self.filt_pos_derr_x += (raw_x - self.filt_pos_derr_x) * d_alpha;
            self.filt_pos_derr_z += (raw_z - self.filt_pos_derr_z) * d_alpha;
            self.prev_pos_err_x = pos_err_x;
            self.prev_pos_err_z = pos_err_z;
            v_des_x = self.drone_pos_kp * pos_err_x + self.drone_pos_ki * self.pos_int_x + self.drone_pos_kd * self.filt_pos_derr_x;
            v_des_z = self.drone_pos_kp * pos_err_z + self.drone_pos_ki * self.pos_int_z + self.drone_pos_kd * self.filt_pos_derr_z;
        }

        // ---- Vertical ----
        let v_des_y;
        if vert_active {
            v_des_y = input.throttle * self.drone_max_vspeed * boost;
            self.target_y = self.y;
            self.pos_int_y = 0.0;
            self.filt_pos_derr_y = 0.0;
            self.prev_pos_err_y = 0.0;
        } else {
            let pos_err_y = self.target_y - self.y;
            let pi_max = self.pos_int_max;
            self.pos_int_y = clamp(self.pos_int_y + pos_err_y * dt, -pi_max, pi_max);
            let d_alpha = 1.0 - (-20.0 * dt).exp();
            let raw_y = if dt > 0.0 { (pos_err_y - self.prev_pos_err_y) / dt } else { 0.0 };
            self.filt_pos_derr_y += (raw_y - self.filt_pos_derr_y) * d_alpha;
            self.prev_pos_err_y = pos_err_y;
            v_des_y = self.drone_alt_kp * pos_err_y + self.drone_alt_ki * self.pos_int_y + self.drone_alt_kd * self.filt_pos_derr_y;
        }

        // Clamp desired velocity
        let v_des_h = (v_des_x * v_des_x + v_des_z * v_des_z).sqrt();
        let (v_des_x, v_des_z) = if v_des_h > max_spd {
            let s = max_spd / v_des_h;
            (v_des_x * s, v_des_z * s)
        } else {
            (v_des_x, v_des_z)
        };
        let v_des_y = clamp(v_des_y, -self.drone_max_vspeed * boost, self.drone_max_vspeed * boost);

        // ---- Inner loop: velocity PID → tilt angles ----
        let max_angle = self.drone_max_angle;
        let a_max_horiz = G * (max_angle * DEG2RAD).tan();
        let vel_err_clamp = a_max_horiz / self.drone_vel_kp;

        let vel_err_x = clamp(v_des_x - self.vx, -vel_err_clamp, vel_err_clamp);
        let vel_err_y = v_des_y - self.vy;
        let vel_err_z = clamp(v_des_z - self.vz, -vel_err_clamp, vel_err_clamp);

        let vi_max = self.vel_int_max;
        self.vel_int_x = clamp(self.vel_int_x + vel_err_x * dt, -vi_max, vi_max);
        self.vel_int_y = clamp(self.vel_int_y + vel_err_y * dt, -vi_max, vi_max);
        self.vel_int_z = clamp(self.vel_int_z + vel_err_z * dt, -vi_max, vi_max);

        let vd_alpha = 1.0 - (-15.0 * dt).exp();
        let raw_vx = if dt > 0.0 { (vel_err_x - self.prev_vel_err_x) / dt } else { 0.0 };
        let raw_vy = if dt > 0.0 { (vel_err_y - self.prev_vel_err_y) / dt } else { 0.0 };
        let raw_vz = if dt > 0.0 { (vel_err_z - self.prev_vel_err_z) / dt } else { 0.0 };
        self.filt_vel_derr_x += (raw_vx - self.filt_vel_derr_x) * vd_alpha;
        self.filt_vel_derr_y += (raw_vy - self.filt_vel_derr_y) * vd_alpha;
        self.filt_vel_derr_z += (raw_vz - self.filt_vel_derr_z) * vd_alpha;
        self.prev_vel_err_x = vel_err_x;
        self.prev_vel_err_y = vel_err_y;
        self.prev_vel_err_z = vel_err_z;

        // Desired world-frame horizontal acceleration
        let a_des_x = self.drone_vel_kp * vel_err_x + self.drone_vel_ki * self.vel_int_x + self.drone_vel_kd * self.filt_vel_derr_x;
        let a_des_z = self.drone_vel_kp * vel_err_z + self.drone_vel_ki * self.vel_int_z + self.drone_vel_kd * self.filt_vel_derr_z;

        // Project onto body axes
        let a_fwd = a_des_x * fwd_x + a_des_z * fwd_z;
        let a_right = a_des_x * right_x + a_des_z * right_z;

        let target_pitch = clamp(-a_fwd / G * RAD2DEG, -max_angle, max_angle);
        let target_roll = clamp(-a_right / G * RAD2DEG, -max_angle, max_angle);

        let smooth_factor = 1.0 - (-10.0 * dt).exp();
        self.smooth_target_pitch += (target_pitch - self.smooth_target_pitch) * smooth_factor;
        self.smooth_target_roll += (target_roll - self.smooth_target_roll) * smooth_factor;

        // Attitude P-controller
        let (_, body_pitch_deg, body_roll_deg) = self.decompose_orientation();
        let pitch_err = self.smooth_target_pitch - body_pitch_deg;
        let roll_err = self.smooth_target_roll - body_roll_deg;
        let max_step = self.drone_angle_rate * dt;
        let dpitch = clamp(pitch_err, -max_step, max_step);
        let droll = clamp(roll_err, -max_step, max_step);

        self.apply_body_rotation(Vector3::unit_x(), dpitch);
        self.apply_body_rotation(Vector3::unit_z(), droll);
        self.pitch_rate = pitch_err * 5.0;
        self.roll_rate = roll_err * 5.0;

        // Yaw: pure rate control
        let drone_yaw_max = self.drone_max_yaw_rate * rate_y * boost;
        let t_yr = input.yaw * drone_yaw_max;
        let ys = 1.0 - (-15.0 * dt).exp();
        self.yaw_rate += (t_yr - self.yaw_rate) * ys;
        if !yaw_active {
            self.yaw_rate *= (-self.angular_drag * dt).exp();
        }
        self.apply_body_rotation(Vector3::unit_y(), self.yaw_rate * dt);

        // Altitude PID → thrust (grams-force)
        let a_des_y = self.drone_vel_kp * vel_err_y + self.drone_vel_ki * self.vel_int_y + self.drone_vel_kd * self.filt_vel_derr_y;
        let mut cmd_gf = self.mass * (G + a_des_y) / G;

        // Tilt compensation
        let rot2 = Matrix3::from(self.orientation);
        let cos_t = rot2.y.y.max(0.1);
        cmd_gf /= cos_t;

        self.thrust_output = clamp(cmd_gf, 0.0, self.max_thrust * boost);
    }
}
