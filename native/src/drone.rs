/// Drone physics — quaternion-based FPV flight model.
/// Ported from src/drone.js (simplified for Phase 2: keyboard-only, no collision yet).
use cgmath::{Deg, InnerSpace, Quaternion, Rad, Rotation, Rotation3, Vector3, Zero};

const G: f32 = 9.81;
const DEG2RAD: f32 = std::f32::consts::PI / 180.0;

pub struct Drone {
    // Position
    pub x: f32,
    pub y: f32,
    pub z: f32,

    // Velocity
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,

    // Orientation (quaternion)
    pub orientation: Quaternion<f32>,

    // Camera
    pub camera_tilt_angle: f32, // degrees, negative = looking down

    // Config
    pub max_thrust: f32,
    pub mass: f32,
    pub drag_coeff: f32,
    pub max_rate: f32,    // deg/s for roll/pitch/yaw
    pub drone_size: f32,

    // Outputs (for HUD/audio)
    pub thrust_output: f32,
    pub armed: bool,
}

pub struct DroneInput {
    pub roll: f32,     // -1..1
    pub pitch: f32,    // -1..1
    pub throttle: f32, // -1..1 (maps to 0..1 thrust)
    pub yaw: f32,      // -1..1
}

impl Default for DroneInput {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            throttle: -1.0, // idle = no thrust
            yaw: 0.0,
        }
    }
}

impl Drone {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 2.0,
            z: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            orientation: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            camera_tilt_angle: -10.0,
            max_thrust: 25.0,
            mass: 0.8,
            drag_coeff: 0.3,
            max_rate: 600.0,
            drone_size: 0.3,
            thrust_output: 0.0,
            armed: false,
        }
    }

    pub fn reset(&mut self, x: f32, y: f32, z: f32) {
        self.x = x;
        self.y = y;
        self.z = z;
        self.vx = 0.0;
        self.vy = 0.0;
        self.vz = 0.0;
        self.orientation = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        self.thrust_output = 0.0;
    }

    pub fn update(&mut self, dt: f32, input: &DroneInput) {
        if !self.armed {
            self.thrust_output = 0.0;
            return;
        }

        let dt = dt.min(0.05); // Cap dt to avoid instability

        // Angular rates (body frame)
        let roll_rate = input.roll * self.max_rate * DEG2RAD * dt;
        let pitch_rate = input.pitch * self.max_rate * DEG2RAD * dt;
        let yaw_rate = input.yaw * self.max_rate * DEG2RAD * dt;

        // Apply rotations in body frame
        let q_roll = Quaternion::from_axis_angle(Vector3::unit_z(), Rad(-roll_rate));
        let q_pitch = Quaternion::from_axis_angle(Vector3::unit_x(), Rad(pitch_rate));
        let q_yaw = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(-yaw_rate));

        self.orientation = self.orientation * q_roll * q_pitch * q_yaw;
        self.orientation = self.orientation.normalize();

        // Thrust (0..1 mapped from throttle -1..1)
        let throttle_01 = (input.throttle + 1.0) * 0.5;
        let thrust_magnitude = throttle_01 * self.max_thrust;
        self.thrust_output = thrust_magnitude;

        // Thrust vector in world frame (local +Y)
        let local_up = self.orientation.rotate_vector(Vector3::unit_y());
        let thrust_world = local_up * thrust_magnitude;

        // Gravity
        let gravity = Vector3::new(0.0, -G * self.mass, 0.0);

        // Drag (simple linear)
        let vel = Vector3::new(self.vx, self.vy, self.vz);
        let speed = vel.magnitude();
        let drag = if speed > 0.001 {
            -vel.normalize() * speed * speed * self.drag_coeff
        } else {
            Vector3::zero()
        };

        // Net acceleration
        let accel = (thrust_world + gravity + drag) / self.mass;

        // Integrate
        self.vx += accel.x * dt;
        self.vy += accel.y * dt;
        self.vz += accel.z * dt;

        self.x += self.vx * dt;
        self.y += self.vy * dt;
        self.z += self.vz * dt;

        // Ground clamp
        if self.y < 0.1 {
            self.y = 0.1;
            self.vy = self.vy.max(0.0);
        }
    }

    /// Returns (position, forward, up) for camera placement.
    /// Camera is at the front edge of the drone, tilted by camera_tilt_angle.
    pub fn camera_transform(&self) -> (Vector3<f32>, Quaternion<f32>) {
        let pos = Vector3::new(self.x, self.y, self.z);

        // Camera tilt around local X axis
        let tilt = Quaternion::from_axis_angle(
            Vector3::unit_x(),
            Rad(self.camera_tilt_angle * DEG2RAD),
        );
        let cam_orientation = self.orientation * tilt;

        // Offset camera to front of drone
        let forward = self.orientation.rotate_vector(-Vector3::unit_z());
        let cam_pos = pos + forward * (self.drone_size * 0.5);

        (cam_pos, cam_orientation)
    }
}
