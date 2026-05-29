//! Numerical drone PID stability test — no GPU, no window.
//! Simulates 5 seconds of Drone mode with a pitch input pulse.
//! Prints position/velocity/pitch each 0.1s to check for divergence.

#[path = "../src/drone.rs"]
mod drone;

use drone::{Drone, DroneInput, FlightMode};

#[test]
fn drone_mode_stability() {
    let mut d = Drone::new();
    d.flight_mode = FlightMode::Drone;
    d.reset(0.0, 10.0, 0.0); // start at Y=10 (altitude)

    let dt = 0.01f32; // 100 Hz
    let mut t = 0.0f32;
    let mut frame = 0u32;

    println!("\n=== Drone PID stability test ===");
    println!("t     | x      y      z     | vx     vy     vz    | pitch  roll   | thrust");

    for _ in 0..500 { // 5 seconds
        // First 1s: no stick input (hover). Then 1s: pitch forward. Then 3s: release.
        let pitch_input = if t > 1.0 && t < 2.0 { -0.3 } else { 0.0 }; // push forward
        let input = DroneInput {
            roll: 0.0,
            pitch: pitch_input,
            throttle: 0.0, // middle = hover in drone mode
            yaw: 0.0,
            armed: true,
            boost: false,
            rates: [1.0, 1.0, 1.0],
        };

        d.update(dt, &input);
        t += dt;
        frame += 1;

        // Print every 0.1s
        if frame % 10 == 0 {
            println!("{:.2}  | {:.2} {:.2} {:.2} | {:.2} {:.2} {:.2} | {:.1} {:.1} | {:.0}",
                t, d.x, d.y, d.z, d.vx, d.vy, d.vz,
                d.body_pitch, d.body_roll, d.thrust_output);
        }

        // Check for NaN or extreme values (divergence)
        if d.x.is_nan() || d.y.abs() > 1000.0 || d.vx.abs() > 100.0 {
            println!("!!! DIVERGED at t={:.2} !!!", t);
            panic!("Drone PID diverged!");
        }
    }

    println!("\n=== Test passed: no divergence ===");
    println!("Final pos: ({:.2}, {:.2}, {:.2})", d.x, d.y, d.z);
}
