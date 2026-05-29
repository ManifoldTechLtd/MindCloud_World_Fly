//! Standalone quaternion convention test — no GPU, no window, just math.
//! Run: cargo test --test quat_check -- --nocapture

use cgmath::{InnerSpace, Matrix3, Quaternion, Rad, Rotation, Rotation3, Vector3};
use std::f32::consts::PI;

fn deg(d: f32) -> Rad<f32> { Rad(d * PI / 180.0) }

#[test]
fn check_conventions() {
    println!("\n=== cgmath quaternion convention test ===\n");

    // 1. Identity
    let q_id: Quaternion<f32> = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(0.0));
    let up = q_id.rotate_vector(Vector3::unit_y());
    let fwd = q_id.rotate_vector(-Vector3::unit_z());
    println!("Identity:");
    println!("  rotate_vector(+Y) = ({:.3}, {:.3}, {:.3}) [expect (0,1,0)]", up.x, up.y, up.z);
    println!("  rotate_vector(-Z) = ({:.3}, {:.3}, {:.3}) [expect (0,0,-1)]", fwd.x, fwd.y, fwd.z);

    // 2. Matrix3 columns vs rotate_vector
    let rot = Matrix3::from(q_id);
    println!("\n  Matrix3::from(identity).y = ({:.3}, {:.3}, {:.3})", rot.y.x, rot.y.y, rot.y.z);
    println!("  (should match rotate_vector(+Y) above)\n");

    // 3. Yaw 90 right (positive Y rotation)
    let q_yaw90 = Quaternion::from_axis_angle(Vector3::unit_y(), deg(90.0));
    let fwd_y90 = q_yaw90.rotate_vector(-Vector3::unit_z());
    let right_y90 = q_yaw90.rotate_vector(Vector3::unit_x());
    let up_y90 = q_yaw90.rotate_vector(Vector3::unit_y());
    println!("Yaw +90 (around +Y):");
    println!("  forward(-Z): ({:.3}, {:.3}, {:.3})", fwd_y90.x, fwd_y90.y, fwd_y90.z);
    println!("  right(+X):   ({:.3}, {:.3}, {:.3})", right_y90.x, right_y90.y, right_y90.z);
    println!("  up(+Y):      ({:.3}, {:.3}, {:.3})", up_y90.x, up_y90.y, up_y90.z);

    // Also check Matrix3 column
    let rot_y90 = Matrix3::from(q_yaw90);
    println!("  Matrix3.y (col1) = ({:.3}, {:.3}, {:.3})", rot_y90.y.x, rot_y90.y.y, rot_y90.y.z);
    println!("  Matrix3.z (col2) = ({:.3}, {:.3}, {:.3})", rot_y90.z.x, rot_y90.z.y, rot_y90.z.z);

    // 4. Pitch nose-down test (positive X rotation)
    let q_pitch30 = Quaternion::from_axis_angle(Vector3::unit_x(), deg(30.0));
    let up_p30 = q_pitch30.rotate_vector(Vector3::unit_y());
    let fwd_p30 = q_pitch30.rotate_vector(-Vector3::unit_z());
    println!("\nPitch +30 (around +X):");
    println!("  up(+Y):  ({:.3}, {:.3}, {:.3})", up_p30.x, up_p30.y, up_p30.z);
    println!("  fwd(-Z): ({:.3}, {:.3}, {:.3})", fwd_p30.x, fwd_p30.y, fwd_p30.z);
    println!("  fwd.y={:.3}: negative means nose points downward", fwd_p30.y);
    println!("  up.z={:.3}: positive means up-vector tilts backward (nose down)", up_p30.z);

    // 5. Body pitch after yaw (the critical test)
    let q_yaw90_then_pitch = q_yaw90 * q_pitch30; // body frame pitch after yaw
    let up_yp = q_yaw90_then_pitch.rotate_vector(Vector3::unit_y());
    let fwd_yp = q_yaw90_then_pitch.rotate_vector(-Vector3::unit_z());
    println!("\nYaw90 then body-pitch +30:");
    println!("  up:  ({:.3}, {:.3}, {:.3})", up_yp.x, up_yp.y, up_yp.z);
    println!("  fwd: ({:.3}, {:.3}, {:.3})", fwd_yp.x, fwd_yp.y, fwd_yp.z);
    println!("  fwd.y={:.3}: should still be <0 (nose down in world)", fwd_yp.y);
    println!("  BODY PITCH WORKS CORRECTLY: {}", fwd_yp.y < -0.1);

    // 6. Compare Matrix3 columns vs rotate_vector for yaw90
    println!("\n=== Matrix3 vs rotate_vector comparison ===");
    let m = Matrix3::from(q_yaw90);
    let rv_x = q_yaw90.rotate_vector(Vector3::unit_x());
    let rv_y = q_yaw90.rotate_vector(Vector3::unit_y());
    let rv_z = q_yaw90.rotate_vector(Vector3::unit_z());
    println!("  Matrix3.x (col0) = ({:.3}, {:.3}, {:.3})", m.x.x, m.x.y, m.x.z);
    println!("  rotate(+X)       = ({:.3}, {:.3}, {:.3})", rv_x.x, rv_x.y, rv_x.z);
    println!("  MATCH: {}", (m.x - rv_x).magnitude() < 0.01);
    println!("  Matrix3.y (col1) = ({:.3}, {:.3}, {:.3})", m.y.x, m.y.y, m.y.z);
    println!("  rotate(+Y)       = ({:.3}, {:.3}, {:.3})", rv_y.x, rv_y.y, rv_y.z);
    println!("  MATCH: {}", (m.y - rv_y).magnitude() < 0.01);
    println!("  Matrix3.z (col2) = ({:.3}, {:.3}, {:.3})", m.z.x, m.z.y, m.z.z);
    println!("  rotate(+Z)       = ({:.3}, {:.3}, {:.3})", rv_z.x, rv_z.y, rv_z.z);
    println!("  MATCH: {}", (m.z - rv_z).magnitude() < 0.01);

    println!("\n=== DONE ===");
}
