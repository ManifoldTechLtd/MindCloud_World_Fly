use cgmath::{Quaternion, Rad, Rotation, Rotation3, Vector3};
use std::f32::consts::PI;

#[test]
fn test_identity_orientation() {
    let q = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(0.0f32));
    let up = q.rotate_vector(Vector3::unit_y());
    let fwd = q.rotate_vector(-Vector3::unit_z());
    let right = q.rotate_vector(Vector3::unit_x());
    
    println!("Identity: up={:?} fwd={:?} right={:?}", up, fwd, right);
    assert!((up.y - 1.0).abs() < 0.01, "up should be (0,1,0)");
    assert!((fwd.z + 1.0).abs() < 0.01, "fwd should be (0,0,-1)");
    assert!((right.x - 1.0).abs() < 0.01, "right should be (1,0,0)");
}

#[test]
fn test_yaw_90_right() {
    // Yaw 90 degrees right (around Y axis, positive = counter-clockwise from above)
    let q = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(PI / 2.0));
    let up = q.rotate_vector(Vector3::unit_y());
    let fwd = q.rotate_vector(-Vector3::unit_z());
    let right = q.rotate_vector(Vector3::unit_x());
    
    println!("Yaw 90: up={:?} fwd={:?} right={:?}", up, fwd, right);
    // After yaw 90 right: forward should point along +X or -X?
    // Positive Y rotation (right-hand rule): +X → +Z, +Z → -X
    // So local -Z (forward) maps to: -(-X) = +X
    println!("  fwd.x={:.3} fwd.z={:.3}", fwd.x, fwd.z);
}

#[test]
fn test_pitch_nose_down() {
    // Pitch: rotate around body X axis. Positive angle by right-hand rule: Y→Z
    // For a drone: Y=up, -Z=forward. Rotating Y toward Z means up goes backward = nose DOWN
    let angle = 30.0f32 * PI / 180.0;
    let q = Quaternion::from_axis_angle(Vector3::unit_x(), Rad(angle));
    let up = q.rotate_vector(Vector3::unit_y());
    let fwd = q.rotate_vector(-Vector3::unit_z());
    
    println!("Pitch +30deg: up={:?} fwd={:?}", up, fwd);
    println!("  up.z={:.3} (should be >0 if nose down, up tilts back)");
    println!("  fwd.y={:.3} (should be <0 if nose down, forward points downward)");
    
    // Verify: positive rotation around X → Y goes toward Z → up tilts into Z (backward)
    // This means the nose goes DOWN (forward points downward)
    assert!(fwd.y < 0.0, "Positive X rotation should pitch nose down: fwd.y={}", fwd.y);
}

#[test]
fn test_body_rotation_after_yaw() {
    // First yaw 90 right, then pitch in body frame
    let yaw_q = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(PI / 2.0));
    let pitch_delta = Quaternion::from_axis_angle(Vector3::unit_x(), Rad(0.3));
    
    // Body frame rotation: orientation * delta
    let q = yaw_q * pitch_delta;
    
    let up = q.rotate_vector(Vector3::unit_y());
    let fwd = q.rotate_vector(-Vector3::unit_z());
    
    println!("Yaw90+BodyPitch: up={:?} fwd={:?}", up, fwd);
    // After yaw 90 right, body X axis is now world Z (or -Z).
    // Body pitch should tilt around the new body X axis (world Z).
    // This means the nose (now pointing along +X) should go down.
    println!("  fwd.y={:.3} (should be <0 = nose down)");
    
    // The key question: does fwd.y go negative?
    // If body rotation works correctly, pitch should still nose-down
    // regardless of yaw angle.
    assert!(fwd.y < 0.0, "Body pitch after yaw should still be nose-down");
}

#[test]
fn test_body_pitch_from_up_vector() {
    // Test that (-up.z).atan2(up.y) gives correct pitch
    let angle = 30.0f32 * PI / 180.0;
    let q = Quaternion::from_axis_angle(Vector3::unit_x(), Rad(angle));
    let up = q.rotate_vector(Vector3::unit_y());
    
    let measured_pitch = (-up.z).atan2(up.y) * 180.0 / PI;
    println!("Pitched 30deg: measured_pitch={:.1} (expected -30 if nose-down-positive convention)",
             measured_pitch);
    
    // Positive X rotation = nose down. up.z > 0 (up tilts backward).
    // (-up.z) < 0, atan2(negative, positive) < 0.
    // So measured_pitch < 0 for nose-down. This means our convention is:
    // positive measured_pitch = nose UP.
    println!("  Convention: measured_pitch<0 means nose down");
}
