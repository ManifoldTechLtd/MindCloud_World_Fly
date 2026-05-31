//! Test to verify camera rotation convention in world2view.
//! Run: cargo test --test camera_convention --no-default-features -- --nocapture

use cgmath::{
    InnerSpace, Matrix, Matrix3, Matrix4, Quaternion, Rad, Rotation, Rotation3, 
    Transform, Vector3, Vector4,
};
use std::f32::consts::PI;

fn deg(d: f32) -> Rad<f32> { Rad(d * PI / 180.0) }

/// Reproduce web-splat's world2view function
fn world2view(r: Matrix3<f32>, t: Vector3<f32>) -> Matrix4<f32> {
    let mut rt = Matrix4::from(r);
    rt[0].w = t.x;
    rt[1].w = t.y;
    rt[2].w = t.z;
    rt[3].w = 1.0;
    rt.inverse_transform().unwrap().transpose()
}

#[test]
fn test_body_frame_simulation() {
    println!("\n=== Simulating FPV: Yaw right 90° then pitch forward ===\n");

    // Simulate: start at identity, yaw -90 degrees (right = negative Y rotation in RH),
    // then push pitch forward (negative pitch = nose down)
    
    // Step 1: identity orientation
    let mut orientation: Quaternion<f32> = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    println!("Initial orientation: identity");
    let rot0 = Matrix3::from(orientation);
    println!("  forward: ({:.3}, {:.3}, {:.3})", -rot0.z.x, -rot0.z.y, -rot0.z.z);
    println!("  up:      ({:.3}, {:.3}, {:.3})", rot0.y.x, rot0.y.y, rot0.y.z);
    println!("  right:   ({:.3}, {:.3}, {:.3})", rot0.x.x, rot0.x.y, rot0.x.z);

    // Step 2: Apply yaw +90 around body Y (in the Rust code, pressing D gives yaw > 0)
    // After many frames this accumulates to 90 degrees
    let yaw_q = Quaternion::from_axis_angle(Vector3::unit_y(), deg(90.0));
    orientation = (orientation * yaw_q).normalize();
    
    let rot1 = Matrix3::from(orientation);
    println!("\nAfter yaw +90° (body Y):");
    println!("  forward: ({:.3}, {:.3}, {:.3})", -rot1.z.x, -rot1.z.y, -rot1.z.z);
    println!("  up:      ({:.3}, {:.3}, {:.3})", rot1.y.x, rot1.y.y, rot1.y.z);
    println!("  right:   ({:.3}, {:.3}, {:.3})", rot1.x.x, rot1.x.y, rot1.x.z);
    
    // Verify: we should be facing -X direction now
    // In cgmath: yaw +90 around Y means X -> -Z, Z -> X
    // forward = -local_Z = -(1,0,0) = (-1,0,0) - facing -X direction

    // Step 3: Apply pitch (nose down = which sign?)
    // In control_fpv: input.pitch > 0 → pitchRate > 0 → apply_body_rotation(unit_x, positive)
    // From test: +pitch around X means nose goes UP (forward.y becomes positive)
    // So for nose DOWN, need negative pitch
    // "Push pitch stick forward" = nose down = negative pitch in this convention
    
    let pitch_q = Quaternion::from_axis_angle(Vector3::unit_x(), deg(-30.0)); // nose down
    orientation = (orientation * pitch_q).normalize();
    
    let rot2 = Matrix3::from(orientation);
    println!("\nAfter body pitch -30° (nose down):");
    println!("  forward: ({:.3}, {:.3}, {:.3})", -rot2.z.x, -rot2.z.y, -rot2.z.z);
    println!("  up:      ({:.3}, {:.3}, {:.3})", rot2.y.x, rot2.y.y, rot2.y.z);
    println!("  right:   ({:.3}, {:.3}, {:.3})", rot2.x.x, rot2.x.y, rot2.x.z);
    
    // The forward vector should tilt downward (forward.y < 0) while staying mostly in -X
    // The up vector should tilt toward -X (the forward direction)
    println!("  forward.y = {:.3} (should be < 0 for nose-down)", -rot2.z.y);
    println!("  up stays in XY plane? up.z = {:.3} (should be ~0 for pure pitch)", rot2.y.z);
    
    // Step 4: Check thrust direction
    let up = rot2.y;
    println!("\nThrust direction (up vector): ({:.3}, {:.3}, {:.3})", up.x, up.y, up.z);
    println!("  Horizontal component: ({:.3}, 0, {:.3})", up.x, up.z);
    println!("  If drone faces -X, thrust horizontal should be in -X direction for nose-down");
    println!("  up.x = {:.3} (should be < 0 for forward acceleration when facing -X)", up.x);
    
    // Step 5: What would WORLD-FRAME pitch look like?
    println!("\n--- Compare with WORLD-FRAME pitch (BUG scenario) ---");
    let mut orientation_bug = Quaternion::new(1.0f32, 0.0, 0.0, 0.0);
    let yaw_q2 = Quaternion::from_axis_angle(Vector3::unit_y(), deg(90.0));
    orientation_bug = (orientation_bug * yaw_q2).normalize();
    
    // World-frame pitch = delta * orientation (instead of orientation * delta)
    let pitch_q_world = Quaternion::from_axis_angle(Vector3::unit_x(), deg(-30.0));
    orientation_bug = (pitch_q_world * orientation_bug).normalize(); // WORLD frame!
    
    let rot_bug = Matrix3::from(orientation_bug);
    println!("After yaw +90 then WORLD-FRAME pitch -30°:");
    println!("  forward: ({:.3}, {:.3}, {:.3})", -rot_bug.z.x, -rot_bug.z.y, -rot_bug.z.z);
    println!("  up:      ({:.3}, {:.3}, {:.3})", rot_bug.y.x, rot_bug.y.y, rot_bug.y.z);
    println!("  right:   ({:.3}, {:.3}, {:.3})", rot_bug.x.x, rot_bug.x.y, rot_bug.x.z);
    println!("  Thrust horizontal: ({:.3}, 0, {:.3})", rot_bug.y.x, rot_bug.y.z);
    println!("  This would look like a ROLL from the camera's perspective!");
    
    // Step 6: Check camera view - does a horizon line appear tilted (roll) or shifted (pitch)?
    println!("\n--- Camera view analysis ---");
    // In the FPV camera, the horizon should:
    // - Shift up/down for pitch changes
    // - Tilt for roll changes
    // The camera up direction in world tells us if the horizon appears tilted:
    // Camera up = orientation's Y column = rot2.y for correct, rot_bug.y for bug
    
    println!("Correct body-frame pitch: camera up = ({:.3}, {:.3}, {:.3})", rot2.y.x, rot2.y.y, rot2.y.z);
    println!("  Camera up.z = {:.3} (non-zero means horizon is TILTED = appears as ROLL)", rot2.y.z);
    println!("  Camera up projected to XZ plane magnitude = {:.3}", (rot2.y.x*rot2.y.x + rot2.y.z*rot2.y.z).sqrt());
    
    println!("World-frame pitch (BUG): camera up = ({:.3}, {:.3}, {:.3})", rot_bug.y.x, rot_bug.y.y, rot_bug.y.z);
    println!("  Camera up.z = {:.3} (non-zero means horizon is TILTED = appears as ROLL)", rot_bug.y.z);
    
    println!("\n=== DONE ===");
}

#[test]
fn test_decompose_orientation() {
    println!("\n=== Decompose Orientation Test ===\n");
    
    // Reproduce decompose_orientation logic
    fn decompose(orientation: Quaternion<f32>) -> (f32, f32, f32) {
        let rot = Matrix3::from(orientation);
        let local_z = rot.z;
        let yaw_rad = local_z.x.atan2(local_z.z);
        let yaw_deg = yaw_rad * 180.0 / PI;
        let yaw_q = Quaternion::from_axis_angle(Vector3::unit_y(), Rad(yaw_rad));
        let tilt_q = yaw_q.invert() * orientation;
        let tilt_rot = Matrix3::from(tilt_q);
        let body_pitch = (-tilt_rot.z.y).asin() * 180.0 / PI;
        let body_roll = tilt_rot.x.y.atan2(tilt_rot.y.y) * 180.0 / PI;
        (yaw_deg, body_pitch, body_roll)
    }
    
    // Test 1: Pure pitch 30 at identity
    let q_pitch = Quaternion::from_axis_angle(Vector3::unit_x(), deg(30.0));
    let (y, bp, br) = decompose(q_pitch);
    println!("Pure pitch +30: yaw={:.1}, body_pitch={:.1}, body_roll={:.1}", y, bp, br);
    
    // Test 2: Pure roll 30 at identity
    let q_roll = Quaternion::from_axis_angle(Vector3::unit_z(), deg(30.0));
    let (y, bp, br) = decompose(q_roll);
    println!("Pure roll +30: yaw={:.1}, body_pitch={:.1}, body_roll={:.1}", y, bp, br);
    
    // Test 3: Yaw 90 then body pitch 30
    let q_yaw90 = Quaternion::from_axis_angle(Vector3::unit_y(), deg(90.0));
    let q_bp30 = Quaternion::from_axis_angle(Vector3::unit_x(), deg(30.0));
    let q_combined = (q_yaw90 * q_bp30).normalize();
    let (y, bp, br) = decompose(q_combined);
    println!("Yaw90 + body pitch +30: yaw={:.1}, body_pitch={:.1}, body_roll={:.1}", y, bp, br);
    println!("  Expected: yaw~90, body_pitch=30, body_roll~0");
    
    // Test 4: Yaw 90 then body roll 30
    let q_br30 = Quaternion::from_axis_angle(Vector3::unit_z(), deg(30.0));
    let q_combined2 = (q_yaw90 * q_br30).normalize();
    let (y, bp, br) = decompose(q_combined2);
    println!("Yaw90 + body roll +30: yaw={:.1}, body_pitch={:.1}, body_roll={:.1}", y, bp, br);
    println!("  Expected: yaw~90, body_pitch~0, body_roll=30");
    
    // Test 5: Yaw 45 then body pitch 30
    let q_yaw45 = Quaternion::from_axis_angle(Vector3::unit_y(), deg(45.0));
    let q_combined3 = (q_yaw45 * q_bp30).normalize();
    let (y, bp, br) = decompose(q_combined3);
    println!("Yaw45 + body pitch +30: yaw={:.1}, body_pitch={:.1}, body_roll={:.1}", y, bp, br);
    println!("  Expected: yaw~45, body_pitch~30, body_roll~0");
    
    println!("\n=== DONE decompose test ===");
}

#[test]
fn test_fpv_full_simulation() {
    println!("\n=== Full FPV Simulation: verify view matrix after yaw+pitch ===\n");

    fn world2view_fn(r: Matrix3<f32>, t: Vector3<f32>) -> Matrix4<f32> {
        let mut rt = Matrix4::from(r);
        rt[0].w = t.x;
        rt[1].w = t.y;
        rt[2].w = t.z;
        rt[3].w = 1.0;
        rt.inverse_transform().unwrap().transpose()
    }

    // Simulate: identity → yaw +90° → pitch -30° (nose down)
    let yaw_q = Quaternion::from_axis_angle(Vector3::unit_y(), deg(90.0));
    let pitch_q = Quaternion::from_axis_angle(Vector3::unit_x(), deg(-30.0));
    let orientation = (yaw_q * pitch_q).normalize();

    let rot = Matrix3::from(orientation);
    let forward = Vector3::new(-rot.z.x, -rot.z.y, -rot.z.z);
    let up = Vector3::new(rot.y.x, rot.y.y, rot.y.z);
    let right = Vector3::new(rot.x.x, rot.x.y, rot.x.z);

    println!("Drone orientation (body-to-world):");
    println!("  forward: ({:.3}, {:.3}, {:.3})", forward.x, forward.y, forward.z);
    println!("  up:      ({:.3}, {:.3}, {:.3})", up.x, up.y, up.z);
    println!("  right:   ({:.3}, {:.3}, {:.3})", right.x, right.y, right.z);

    // Camera position = drone position + forward * half_size
    let drone_pos = Vector3::new(0.0, 5.0, 0.0);
    let cam_pos = drone_pos + forward * 0.15;

    // View matrix using body-to-world orientation directly
    let r_cam = Matrix3::from(orientation);
    let view = world2view_fn(r_cam, cam_pos);

    // Place test points in world and see where they appear in camera space
    // Point directly ahead of drone (in drone's forward direction)
    let p_ahead = Vector4::new(
        drone_pos.x + forward.x * 10.0,
        drone_pos.y + forward.y * 10.0,
        drone_pos.z + forward.z * 10.0,
        1.0
    );
    let p_ahead_cam = view * p_ahead;
    println!("\nPoint 10m ahead in camera space: ({:.3}, {:.3}, {:.3})", p_ahead_cam.x, p_ahead_cam.y, p_ahead_cam.z);
    println!("  Should be near center (x~0, y~0) with positive z (in front)");

    // Point on the ground directly below the ahead point
    let p_ground = Vector4::new(
        drone_pos.x + forward.x * 10.0,
        0.0,  // ground level
        drone_pos.z + forward.z * 10.0,
        1.0
    );
    let p_ground_cam = view * p_ground;
    println!("Ground point below ahead: ({:.3}, {:.3}, {:.3})", p_ground_cam.x, p_ground_cam.y, p_ground_cam.z);
    println!("  Should be below center (y < 0 or y > 0 depending on convention)");

    // Point to the RIGHT of the drone (in body right direction)
    let p_right = Vector4::new(
        drone_pos.x + right.x * 5.0,
        drone_pos.y + right.y * 5.0,
        drone_pos.z + right.z * 5.0,
        1.0
    );
    let p_right_cam = view * p_right;
    println!("Point 5m to right in camera space: ({:.3}, {:.3}, {:.3})", p_right_cam.x, p_right_cam.y, p_right_cam.z);
    println!("  x should be positive (to the right in camera view)");

    // World horizon test: a point on the horizon (same Y as drone, far away)
    // After yaw+pitch, the horizon should appear tilted in camera if there's a roll
    // For pure yaw+pitch (no roll), horizon should stay level
    let p_horizon_left = Vector4::new(-100.0, 5.0, 0.0, 1.0);
    let p_horizon_right = Vector4::new(100.0, 5.0, 0.0, 1.0);
    let h_left = view * p_horizon_left;
    let h_right = view * p_horizon_right;
    println!("\nHorizon test (same Y as drone):");
    println!("  Left (-100,5,0):  cam=({:.3}, {:.3}, {:.3})", h_left.x, h_left.y, h_left.z);
    println!("  Right (100,5,0):  cam=({:.3}, {:.3}, {:.3})", h_right.x, h_right.y, h_right.z);
    
    // For a level horizon (no roll), both points should have the same camera Y
    let horizon_tilt = (h_right.y / h_right.z) - (h_left.y / h_left.z);
    println!("  Horizon tilt (normalized y difference): {:.6}", horizon_tilt);
    println!("  If ~0, horizon is level (no apparent roll). If non-zero, horizon is tilted.");

    // Now test with INVERTED orientation (world-to-body)
    println!("\n--- Same test with INVERTED quaternion ---");
    let orientation_inv = orientation.invert();
    let r_cam_inv = Matrix3::from(orientation_inv);
    let view_inv = world2view_fn(r_cam_inv, cam_pos);

    let p_ahead_cam2 = view_inv * p_ahead;
    println!("Point 10m ahead (inverted): ({:.3}, {:.3}, {:.3})", p_ahead_cam2.x, p_ahead_cam2.y, p_ahead_cam2.z);

    let p_right_cam2 = view_inv * p_right;
    println!("Point 5m right (inverted):  ({:.3}, {:.3}, {:.3})", p_right_cam2.x, p_right_cam2.y, p_right_cam2.z);

    let h_left2 = view_inv * p_horizon_left;
    let h_right2 = view_inv * p_horizon_right;
    println!("Horizon left (inverted):  ({:.3}, {:.3}, {:.3})", h_left2.x, h_left2.y, h_left2.z);
    println!("Horizon right (inverted): ({:.3}, {:.3}, {:.3})", h_right2.x, h_right2.y, h_right2.z);
    let tilt2 = (h_right2.y / h_right2.z) - (h_left2.y / h_left2.z);
    println!("Horizon tilt (inverted): {:.6}", tilt2);

    // Pure yaw test for comparison
    println!("\n--- Pure yaw test ---");
    let yaw_only = yaw_q;
    let rot_y = Matrix3::from(yaw_only);
    let fwd_y = Vector3::new(-rot_y.z.x, -rot_y.z.y, -rot_y.z.z);
    let r_yaw = Matrix3::from(yaw_only);
    let view_yaw = world2view_fn(r_yaw, Vector3::new(0.0, 5.0, 0.0));
    
    // Point ahead (10m in forward direction = -X)
    let p_fwd_yaw = Vector4::new(-10.0, 5.0, 0.0, 1.0);
    let p_fwd_cam = view_yaw * p_fwd_yaw;
    println!("Pure yaw90: point ahead (-10,5,0) -> cam ({:.3}, {:.3}, {:.3})", p_fwd_cam.x, p_fwd_cam.y, p_fwd_cam.z);
    println!("  Should be (0, 0, ~+10) if camera looks correct direction");
    
    // Point behind (10m behind = +X)
    let p_back = Vector4::new(10.0, 5.0, 0.0, 1.0);
    let p_back_cam = view_yaw * p_back;
    println!("Pure yaw90: point behind (10,5,0) -> cam ({:.3}, {:.3}, {:.3})", p_back_cam.x, p_back_cam.y, p_back_cam.z);

    // Test using look_at to create camera rotation
    println!("\n--- look_at based camera rotation (yaw+pitch) ---");
    let cam_rot_lookat = Quaternion::look_at(forward, up);
    let r_lookat = Matrix3::from(cam_rot_lookat);
    let view_la = world2view_fn(r_lookat, cam_pos);
    let p_ahead_la = view_la * p_ahead;
    println!("Ahead (look_at): ({:.3}, {:.3}, {:.3})", p_ahead_la.x, p_ahead_la.y, p_ahead_la.z);
    let p_right_la = view_la * p_right;
    println!("Right (look_at): ({:.3}, {:.3}, {:.3})", p_right_la.x, p_right_la.y, p_right_la.z);
    let hl_la = view_la * p_horizon_left;
    let hr_la = view_la * p_horizon_right;
    println!("Horizon tilt (look_at): {:.6}", (hr_la.y / hr_la.z) - (hl_la.y / hl_la.z));

    // Test with TRANSPOSED rotation matrix
    println!("\n--- Transposed rotation test (yaw+pitch) ---");
    let r_transposed = Matrix3::from(orientation).transpose();
    let view_t = world2view_fn(r_transposed, cam_pos);
    let p_ahead_t = view_t * p_ahead;
    println!("Ahead (transposed): ({:.3}, {:.3}, {:.3})", p_ahead_t.x, p_ahead_t.y, p_ahead_t.z);
    let p_right_t = view_t * p_right;
    println!("Right (transposed): ({:.3}, {:.3}, {:.3})", p_right_t.x, p_right_t.y, p_right_t.z);
    let hl_t = view_t * p_horizon_left;
    let hr_t = view_t * p_horizon_right;
    println!("Horizon tilt (transposed): {:.6}", (hr_t.y / hr_t.z) - (hl_t.y / hl_t.z));

    // Pure pitch test
    println!("\n--- Pure pitch test ---");
    let pitch_only = Quaternion::from_axis_angle(Vector3::unit_x(), deg(-30.0));
    let rot_p = Matrix3::from(pitch_only);
    let fwd_p = Vector3::new(-rot_p.z.x, -rot_p.z.y, -rot_p.z.z);
    println!("Pure pitch-30: forward=({:.3},{:.3},{:.3})", fwd_p.x, fwd_p.y, fwd_p.z);
    let view_pitch = world2view_fn(Matrix3::from(pitch_only), Vector3::new(0.0, 5.0, 0.0));
    // Point ahead (10m forward in -Z direction with downward tilt)
    let p_fwd_p = Vector4::new(fwd_p.x * 10.0, 5.0 + fwd_p.y * 10.0, fwd_p.z * 10.0, 1.0);
    let p_fwd_p_cam = view_pitch * p_fwd_p;
    println!("Pure pitch-30: point 10m ahead ({:.1},{:.1},{:.1}) -> cam ({:.3}, {:.3}, {:.3})", 
        fwd_p.x*10.0, 5.0+fwd_p.y*10.0, fwd_p.z*10.0, p_fwd_p_cam.x, p_fwd_p_cam.y, p_fwd_p_cam.z);
    println!("  Should be (~0, ~0, ~+10)");

    println!("\n=== DONE full sim ===");
}
