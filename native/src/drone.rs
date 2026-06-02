/// Drone physics v4 — NED body frame + quaternion rate integration.
///
/// Body coordinate system (NED):
///   +X = Forward (camera optical axis when tilt=0)
///   +Y = Right
///   +Z = Down
///   Thrust direction = body -Z (upward)
///
/// World coordinate systems:
///   Z-up: +Z = Up, gravity along -Z, horizontal plane = XY
///   COLMAP: +Y = Down, gravity along +Y, horizontal plane = XZ
///
/// `orientation` quaternion: with integration formula q̇ = ½ (-ω) ⊗ q (left-multiply),
/// Matrix3::from(orientation) ROWS are body axes expressed in world coordinates.
///
/// Initial orientation (spawn, see `ned_identity()`):
///   Z-up:   body+X→world+Y, body+Y→world+X, body+Z→world-Z
///   COLMAP: body+X→world+X, body+Y→world(0,0,-1), body+Z→world+Y
///
/// Heading rotation uses RIGHT-multiply `orientation * yaw_q` (body-frame yaw),
/// which preserves the thrust direction (body+Z in world).
///
/// Camera transform: cam_orient = CAM_TO_NED * orientation * tilt_q
///   CAM_TO_NED maps body axes to COLMAP camera axes (X-right, Y-down, Z-front).
///
/// FPV:   sticks → body-frame angular rates, throttle → thrust, no self-leveling
/// Drone: sticks → target angle, P-controller, tilt compensation

use cgmath::{InnerSpace, Matrix3, Quaternion, Rad, Rotation, Rotation3, Vector3};

const DEG2RAD: f32 = std::f32::consts::PI / 180.0;
const RAD2DEG: f32 = 180.0 / std::f32::consts::PI;
const G: f32 = 9.81;
const AIR_DENSITY: f32 = 1.225;

fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    v.max(lo).min(hi)
}

/// Returns the spawn orientation quaternion for the given world coordinate system.
fn ned_identity(world_up: crate::app_state::WorldUp) -> Quaternion<f32> {
    match world_up {
        crate::app_state::WorldUp::Zup => {
            // body+X→world+Y, body+Y→world+X, body+Z→world-Z
            // Thrust (body -Z) → world +Z (up)
            Quaternion::new(0.0, std::f32::consts::FRAC_1_SQRT_2, std::f32::consts::FRAC_1_SQRT_2, 0.0)
        }
        crate::app_state::WorldUp::Colmap => {
            // body+X→world(1,0,0), body+Y→world(0,0,-1), body+Z→world(0,1,0)
            // Thrust (body -Z) → world -Y (up in COLMAP where +Y=down)
            // Forward at heading=0 is world +X
            Quaternion::new(std::f32::consts::FRAC_1_SQRT_2, std::f32::consts::FRAC_1_SQRT_2, 0.0, 0.0)
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

    // ---- World coordinate system ----
    pub world_up: crate::app_state::WorldUp,

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
    pub spawn_heading: f32,
}

impl Drone {
    pub fn new() -> Self {
        Self {
            drone_size: 0.3,
            x: 0.0, y: 2.0, z: 0.0,
            vx: 0.0, vy: 0.0, vz: 0.0,
            orientation: ned_identity(crate::app_state::WorldUp::Zup),
            pitch_rate: 0.0, roll_rate: 0.0, yaw_rate: 0.0,
            pitch: 0.0, roll: 0.0, yaw: 0.0,
            body_pitch: 0.0, body_roll: 0.0,

            world_up: crate::app_state::WorldUp::Zup,

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
        self.target_x = self.spawn_x; self.target_y = self.spawn_y; self.target_z = self.spawn_z;
        self.clear_pid_state();
        self.apply_spawn_heading(self.spawn_heading);
    }

    /// Rotate the current orientation by heading_deg around the world up axis.
    /// Called after reset_to_spawn to set the initial facing direction.
    pub fn apply_spawn_heading(&mut self, heading_deg: f32) {
        if heading_deg.abs() < 0.01 { return; }
        // Right-multiply: body-frame rotation that keeps thrust axis unchanged.
        // Axis is the body-local "down" axis (body+Z in NED), which maps to world up.
        // For both Z-up and COLMAP, from_axis_angle uses the NED body+Z direction
        // projected into the appropriate world axis.
        let yaw_q = match self.world_up {
            // Z-up: body+Z → world-Z. Rotate around +Z in body = world-Z direction.
            // Positive heading = CW from above = right-hand around +Z in body frame.
            crate::app_state::WorldUp::Zup =>
                Quaternion::from_axis_angle(Vector3::unit_z(), Rad(heading_deg * DEG2RAD)),
            // COLMAP: body+Z → world+Y. Rotate around body+Y maps to world heading rotation.
            crate::app_state::WorldUp::Colmap =>
                Quaternion::from_axis_angle(Vector3::unit_y(), Rad(heading_deg * DEG2RAD)),
        };
        self.orientation = (self.orientation * yaw_q).normalize();
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
        // With left-multiply integration (q_dot = omega*q*0.5), Matrix3 ROWS are body axes in world.
        let rot = Matrix3::from(self.orientation);
        // Thrust direction = body -Z in world = -(row 2) = -(rot.x.z, rot.y.z, rot.z.z)
        let thrust_dir = Vector3::new(-rot.x.z, -rot.y.z, -rot.z.z);

        // Debug: log thrust direction on first few frames
        static DEBUG_COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let c = DEBUG_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if c < 5 || (c % 300 == 0 && self.thrust_output > 0.1) {
            log::info!("thrust_dir=({:.2},{:.2},{:.2}) thrust_out={:.1} pos=({:.2},{:.2},{:.2}) vy={:.2}",
                thrust_dir.x, thrust_dir.y, thrust_dir.z, self.thrust_output,
                self.x, self.y, self.z, self.vy);
        }

        // Forces: thrust along body -Z + gravity + quadratic drag
        let mass_g = self.mass.max(1.0);
        let mass_kg = mass_g / 1000.0;
        let thrust_accel = (self.thrust_output / mass_g) * G;
        let mut ax = thrust_dir.x * thrust_accel;
        let mut ay = thrust_dir.y * thrust_accel;
        let mut az = thrust_dir.z * thrust_accel;
        // Gravity along world down axis
        match self.world_up {
            crate::app_state::WorldUp::Zup => { az -= G; }     // gravity = (0, 0, -G)
            crate::app_state::WorldUp::Colmap => { ay += G; }  // gravity = (0, +G, 0), +Y = down
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

        // NaN guard
        if self.x.is_nan() || self.y.is_nan() || self.z.is_nan() {
            log::warn!("NaN in drone state, resetting");
            self.reset_to_spawn();
            return;
        }

        // Derive euler angles for HUD
        self.update_euler_from_quat();
        match self.world_up {
            crate::app_state::WorldUp::Zup => {
                self.speed = (self.vx * self.vx + self.vy * self.vy).sqrt();
                self.vertical_speed = self.vz;
            }
            crate::app_state::WorldUp::Colmap => {
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

    /// body→cam rotation (applied as LEFT multiply).
    /// Maps: body+X→cam+Z, body+Y→cam+X, body+Z→cam+Y
    /// Used as: cam_orient = CAM_TO_NED * orientation
    const CAM_TO_NED: Quaternion<f32> = Quaternion::new(0.5, -0.5, -0.5, -0.5);

    pub fn camera_transform(&self) -> (Vector3<f32>, Quaternion<f32>) {
        let pos = Vector3::new(self.x, self.y, self.z);
        let rot = Matrix3::from(self.orientation);
        // With left-multiply integration, rows are body axes in world.
        // body +X (forward) in world = row 0 = (rot.x.x, rot.y.x, rot.z.x)
        let forward = Vector3::new(rot.x.x, rot.y.x, rot.z.x);
        let half_size = self.drone_size * 0.5;

        // Camera mount pitch offset (body-frame Y rotation in NED = pitch axis)
        let mount_deg = match self.flight_mode {
            FlightMode::Fpv => self.camera_mount_angle,
            FlightMode::Drone => self.camera_tilt_angle,
        };
        let mount_rad = mount_deg * DEG2RAD;
        // In NED, pitch is around body Y axis. Positive mount angle tilts camera up (nose up).
        let tilt_q = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(mount_rad));
        // Apply tilt in body frame, then left-multiply by body→cam rotation
        let cam_orient = Self::CAM_TO_NED * self.orientation * tilt_q;

        let cam_pos = pos + forward * half_size;
        (cam_pos, cam_orient)
    }

    // ---- Orientation helpers ----

    /// Quaternion rate integration: q̇ = ½ (-ω) ⊗ q, q_{k+1} = normalize(q + q̇·dt)
    /// `omega` is body-frame angular velocity in rad/s: [ω_x(roll), ω_y(pitch), ω_z(yaw)]
    /// Empirically verified directions (with left-multiply and negation):
    ///   ω_x > 0 → roll right (body+Y wing down)
    ///   ω_y > 0 → pitch up (nose up, body_pitch increases)
    ///   ω_z > 0 → yaw right (nose toward body+Y)
    fn integrate_orientation(&mut self, omega_x: f32, omega_y: f32, omega_z: f32, dt: f32) {
        let q = self.orientation;
        // q̇ = ½ (-ω) ⊗ q (left-multiply with negated omega for correct body-frame rotation)
        let omega_q = Quaternion::new(0.0, -omega_x, -omega_y, -omega_z);
        let q_dot = omega_q * q * 0.5;
        // First-order Euler integration
        let q_new = Quaternion::new(
            q.s + q_dot.s * dt,
            q.v.x + q_dot.v.x * dt,
            q.v.y + q_dot.v.y * dt,
            q.v.z + q_dot.v.z * dt,
        );
        self.orientation = q_new.normalize();
    }

    /// Legacy: apply a single-axis body-frame rotation (kept for potential future use)
    #[allow(dead_code)]
    fn apply_body_rotation(&mut self, axis: Vector3<f32>, angle_deg: f32) {
        if angle_deg.abs() < 1e-8 { return; }
        let q = Quaternion::from_axis_angle(axis, Rad(angle_deg * DEG2RAD));
        self.orientation = (self.orientation * q).normalize();
    }

    /// Decompose orientation into yaw, pitch, roll using gravity reference.
    /// Uses body axes in world (rows of Matrix3) to compute attitude relative to gravity.
    /// This method works correctly regardless of yaw angle.
    fn decompose_orientation(&self) -> (f32, f32, f32) {
        let rot = Matrix3::from(self.orientation);
        // With left-multiply integration, ROWS are body axes in world:
        let fwd = Vector3::new(rot.x.x, rot.y.x, rot.z.x);   // row 0 = body+X (forward)
        let right = Vector3::new(rot.x.y, rot.y.y, rot.z.y);  // row 1 = body+Y (right)
        let down = Vector3::new(rot.x.z, rot.y.z, rot.z.z);   // row 2 = body+Z (down)

        match self.world_up {
            crate::app_state::WorldUp::Zup => {
                // Horizontal = XY, vertical = Z. At spawn fwd=(0,1,0).
                let yaw_deg = fwd.x.atan2(fwd.y) * RAD2DEG;
                let body_pitch = fwd.z.asin() * RAD2DEG;
                let body_roll = right.z.atan2(-down.z) * RAD2DEG;
                (yaw_deg, body_pitch, body_roll)
            }
            crate::app_state::WorldUp::Colmap => {
                // Horizontal = XZ, vertical = -Y (+Y=down). At spawn fwd=(1,0,0).
                let yaw_deg = fwd.z.atan2(fwd.x) * RAD2DEG;
                let body_pitch = (-fwd.y).asin() * RAD2DEG;  // -fwd.y because +Y=down
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
        let damp = (-self.angular_drag * dt).exp();
        self.pitch_rate *= damp;
        self.roll_rate *= damp;
        self.yaw_rate *= damp;

        // Auto-level toward zero tilt (keep current yaw)
        let (_, body_pitch, body_roll) = self.decompose_orientation();
        let level_speed = 60.0; // deg/s
        let pitch_step = (level_speed * dt).min(body_pitch.abs());
        let roll_step = (level_speed * dt).min(body_roll.abs());

        // From Drone mode P-controller (empirically verified):
        //   pitch_rate<0 → nose down (reduces positive body_pitch)
        //   roll_rate>0 → reduces positive body_roll (rolls back left)
        let level_pitch_rate = if pitch_step > 0.01 { -body_pitch.signum() * level_speed } else { 0.0 };
        let level_roll_rate = if roll_step > 0.01 { body_roll.signum() * level_speed } else { 0.0 };

        // NED: ω_x = roll, ω_y = pitch, ω_z = yaw
        let omega_x = level_roll_rate * DEG2RAD;
        let omega_y = level_pitch_rate * DEG2RAD;
        let omega_z = self.yaw_rate * DEG2RAD;
        self.integrate_orientation(omega_x, omega_y, omega_z, dt);
    }

    fn control_fpv(&mut self, dt: f32, input: &DroneInput) {
        let boost = if input.boost { 1.5 } else { 1.0 };
        let [rate_r, rate_p, rate_y] = input.rates;

        // Sticks → target angular rates (deg/s)
        // NED: roll=+1 → right roll (ω_x>0), pitch=+1 → nose up (ω_y>0), yaw=+1 → right (ω_z>0)
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
        // NED body axes: ω_x = roll (around body+X/forward), ω_y = pitch (around body+Y/right), ω_z = yaw (around body+Z/down)
        let omega_x = self.roll_rate * DEG2RAD;
        let omega_y = self.pitch_rate * DEG2RAD;
        let omega_z = self.yaw_rate * DEG2RAD;
        self.integrate_orientation(omega_x, omega_y, omega_z, dt);

        // Throttle → thrust (grams-force)
        self.thrust_output = ((input.throttle + 1.0) * 0.5) * self.max_thrust * boost;
    }

    /// Stabilize mode: roll/pitch sticks → target angle, yaw stick → angular rate.
    /// input=0 → level flight. Throttle directly controls thrust.
    fn control_drone(&mut self, dt: f32, input: &DroneInput) {
        let boost = if input.boost { 1.5 } else { 1.0 };
        let [rate_r, rate_p, rate_y] = input.rates;
        let max_angle = self.drone_max_angle;
        let yaw_active = input.yaw.abs() > 0.05;

        // ---- 1. Sticks → target tilt angles (degrees) ----
        // Positive input.pitch → nose up (same direction as FPV)
        let target_pitch = input.pitch * max_angle * rate_p;
        // Positive input.roll → roll right (same direction as FPV)
        let target_roll = -input.roll * max_angle * rate_r;

        // ---- 2. Attitude P-controller ----
        let (_, body_pitch_deg, body_roll_deg) = self.decompose_orientation();
        // pitch_rate>0 → nose up → body_pitch increases. So err = target-current for convergence.
        let pitch_err = target_pitch - body_pitch_deg;
        // roll_rate>0 → body_roll decreases. So err = current-target for convergence.
        let roll_err = body_roll_deg - target_roll;

        let max_rate = 60.0; // deg/s max correction rate (conservative for stability)
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
        // Same as FPV: pass rates directly, integrate_orientation handles negation internally
        let omega_x = self.roll_rate * DEG2RAD;
        let omega_y = self.pitch_rate * DEG2RAD;
        let omega_z = self.yaw_rate * DEG2RAD;
        self.integrate_orientation(omega_x, omega_y, omega_z, dt);

        // ---- 5. Throttle → thrust with tilt compensation ----
        let thrust_base = ((input.throttle + 1.0) * 0.5) * self.max_thrust * boost;
        let rot = Matrix3::from(self.orientation);
        // cos(tilt) = dot(thrust_dir, world_up)
        // thrust_dir = -(row 2) = -(rot.x.z, rot.y.z, rot.z.z)
        let cos_t = match self.world_up {
            crate::app_state::WorldUp::Zup => (-rot.z.z).max(0.1),     // world_up=(0,0,1)
            crate::app_state::WorldUp::Colmap => (rot.y.z).max(0.1),   // world_up=(0,-1,0), dot with -(rot.x.z,rot.y.z,rot.z.z) = rot.y.z
        };
        self.thrust_output = clamp(thrust_base / cos_t, 0.0, self.max_thrust * boost);
    }
}
