use cgmath::{InnerSpace, Matrix3, Quaternion, Vector3};

/// CAM_TO_NED: body+X→cam+Z, body+Y→cam+X, body+Z→cam+Y
const CAM_TO_NED: Quaternion<f32> = Quaternion::new(0.5, -0.5, -0.5, -0.5);

/// Extract body axes from orientation quaternion (rows of Matrix3)
fn body_axes(q: Quaternion<f32>) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    let rot = Matrix3::from(q);
    let fwd = Vector3::new(rot.x.x, rot.y.x, rot.z.x);   // row 0 = body+X
    let right = Vector3::new(rot.x.y, rot.y.y, rot.z.y);  // row 1 = body+Y
    let down = Vector3::new(rot.x.z, rot.y.z, rot.z.z);   // row 2 = body+Z
    (fwd, right, down)
}

/// Get optical axis from camera orientation: -col2 of Matrix3::from(cam_orient)
fn optical_axis(cam_q: Quaternion<f32>) -> Vector3<f32> {
    let m = Matrix3::from(cam_q);
    // col2 = (m[0][2], m[1][2], m[2][2]) but in cgmath column-major: m.z = col2
    Vector3::new(-m.z.x, -m.z.y, -m.z.z)
}

#[test]
fn verify_zup_identity() {
    let sqrt2_2 = std::f32::consts::FRAC_1_SQRT_2;
    let q = Quaternion::new(0.0, sqrt2_2, sqrt2_2, 0.0);
    let (fwd, right, down) = body_axes(q);
    let thrust = Vector3::new(-down.x, -down.y, -down.z);
    let cam_orient = CAM_TO_NED * q;
    let optical = optical_axis(cam_orient);

    println!("=== Z-up ned_identity ===");
    println!("body+X (fwd):   ({:.3}, {:.3}, {:.3})", fwd.x, fwd.y, fwd.z);
    println!("body+Y (right): ({:.3}, {:.3}, {:.3})", right.x, right.y, right.z);
    println!("body+Z (down):  ({:.3}, {:.3}, {:.3})", down.x, down.y, down.z);
    println!("thrust (-Z):    ({:.3}, {:.3}, {:.3})", thrust.x, thrust.y, thrust.z);
    println!("optical axis:   ({:.3}, {:.3}, {:.3})", optical.x, optical.y, optical.z);
    println!();

    // Z-up: thrust should be (0,0,+1), optical should match forward = (0,1,0)
    assert!((thrust.z - 1.0).abs() < 0.01, "thrust should be +Z");
    assert!((optical.y - 1.0).abs() < 0.01, "optical should be +Y (forward)");
}

#[test]
fn verify_colmap_identity_current() {
    // Current (possibly broken) COLMAP quaternion
    let q = Quaternion::new(0.5, -0.5, 0.5, 0.5);
    let (fwd, right, down) = body_axes(q);
    let thrust = Vector3::new(-down.x, -down.y, -down.z);
    let cam_orient = CAM_TO_NED * q;
    let optical = optical_axis(cam_orient);

    println!("=== COLMAP ned_identity (CURRENT: 0.5, -0.5, 0.5, 0.5) ===");
    println!("body+X (fwd):   ({:.3}, {:.3}, {:.3})", fwd.x, fwd.y, fwd.z);
    println!("body+Y (right): ({:.3}, {:.3}, {:.3})", right.x, right.y, right.z);
    println!("body+Z (down):  ({:.3}, {:.3}, {:.3})", down.x, down.y, down.z);
    println!("thrust (-Z):    ({:.3}, {:.3}, {:.3})", thrust.x, thrust.y, thrust.z);
    println!("optical axis:   ({:.3}, {:.3}, {:.3})", optical.x, optical.y, optical.z);
    println!();

    // COLMAP: thrust should be (0,-1,0), optical should point in same dir as fwd
    println!("EXPECTED: thrust=(0,-1,0), optical=fwd direction");
    println!("MATCH thrust: {}", (thrust.y + 1.0).abs() < 0.01);
    println!("MATCH optical==fwd: {}", (optical - fwd).magnitude() < 0.01);
}

#[test]
fn find_correct_colmap_identity() {
    // Requirements for COLMAP ned_identity:
    // 1. body+Z (down) = (0, 1, 0) → thrust = (0, -1, 0) [world up = -Y]
    // 2. optical axis = body+X (forward) — must be in XZ plane
    // 3. The forward direction at heading=0 should be -Z (matching orbit camera at yaw=0)
    //
    // From Z-up analysis: optical_axis(CAM_TO_NED * q) = body+X of q.
    // Let's verify this property and find a q that satisfies all constraints.

    println!("=== Finding correct COLMAP ned_identity ===");
    println!();

    // Try various quaternions with body+Z = (0,1,0) and body+X in XZ plane
    // body+Z = (0,1,0), body+X = (0,0,-1) → body+Y = body+Z × body+X = (0,1,0)×(0,0,-1) = (-1,0,0)
    let candidates: Vec<(&str, Quaternion<f32>)> = vec![
        ("(0.5, -0.5, 0.5, 0.5) [current]", Quaternion::new(0.5, -0.5, 0.5, 0.5)),
        ("(0.5, 0.5, -0.5, -0.5)", Quaternion::new(0.5, 0.5, -0.5, -0.5)),
        ("(0.5, -0.5, -0.5, 0.5)", Quaternion::new(0.5, -0.5, -0.5, 0.5)),
        ("(0.5, 0.5, 0.5, -0.5)", Quaternion::new(0.5, 0.5, 0.5, -0.5)),
        ("(0.5, 0.5, -0.5, 0.5)", Quaternion::new(0.5, 0.5, -0.5, 0.5)),
        ("(0.5, -0.5, 0.5, -0.5)", Quaternion::new(0.5, -0.5, 0.5, -0.5)),
        ("(0.5, 0.5, 0.5, 0.5)", Quaternion::new(0.5, 0.5, 0.5, 0.5)),
        ("(0.5, -0.5, -0.5, -0.5)", Quaternion::new(0.5, -0.5, -0.5, -0.5)),
    ];

    for (name, q) in &candidates {
        let (fwd, _right, down) = body_axes(*q);
        let thrust = Vector3::new(-down.x, -down.y, -down.z);
        let cam_orient = CAM_TO_NED * *q;
        let optical = optical_axis(cam_orient);

        let thrust_ok = (thrust.y + 1.0).abs() < 0.01; // thrust = (0,-1,0)
        let optical_matches_fwd = (optical - fwd).magnitude() < 0.01;
        let fwd_is_neg_z = (fwd.z + 1.0).abs() < 0.1; // forward ≈ (0,0,-1)

        if thrust_ok || optical_matches_fwd {
            println!("Candidate {}", name);
            println!("  fwd=({:.3},{:.3},{:.3}) thrust=({:.3},{:.3},{:.3}) optical=({:.3},{:.3},{:.3})",
                fwd.x, fwd.y, fwd.z, thrust.x, thrust.y, thrust.z, optical.x, optical.y, optical.z);
            println!("  thrust_ok={} optical==fwd={} fwd_is_neg_z={}",
                thrust_ok, optical_matches_fwd, fwd_is_neg_z);
            if thrust_ok && optical_matches_fwd && fwd_is_neg_z {
                println!("  *** WINNER! ***");
            }
            println!();
        }
    }

    // Also try constructing from desired axes directly
    // We want: row0=(0,0,-1), row1=(-1,0,0), row2=(0,1,0)
    // BUT the optical axis must equal fwd. Let's check what optical axis we get.
    println!("--- Direct construction test ---");
    // From matrix: col0=(row0[0],row1[0],row2[0])=(0,-1,0), col1=(row0[1],row1[1],row2[1])=(0,0,1), col2=(row0[2],row1[2],row2[2])=(-1,0,0)
    // This matrix corresponds to q = (0.5, 0.5, -0.5, -0.5) [calculated earlier]
    let q_test = Quaternion::new(0.5, 0.5, -0.5, -0.5);
    let (fwd, right, down) = body_axes(q_test);
    let cam_orient = CAM_TO_NED * q_test;
    let optical = optical_axis(cam_orient);
    println!("q=(0.5, 0.5, -0.5, -0.5):");
    println!("  fwd=({:.3},{:.3},{:.3}) right=({:.3},{:.3},{:.3}) down=({:.3},{:.3},{:.3})",
        fwd.x, fwd.y, fwd.z, right.x, right.y, right.z, down.x, down.y, down.z);
    println!("  optical=({:.3},{:.3},{:.3})", optical.x, optical.y, optical.z);
    println!("  optical==fwd? {}", (optical - fwd).magnitude() < 0.01);
    println!();

    // Exhaustive search: try all unit quaternions with components from {-1, -0.5, 0, 0.5, 1}
    // (only those that normalize to unit quaternions)
    println!("\n--- Exhaustive search ---");
    let vals: Vec<f32> = vec![-1.0, -0.5, 0.0, 0.5, 1.0];
    let sqrt2_2 = std::f32::consts::FRAC_1_SQRT_2;
    let vals2: Vec<f32> = vec![-1.0, -sqrt2_2, -0.5, 0.0, 0.5, sqrt2_2, 1.0];
    let mut found = false;
    for &w in &vals2 {
        for &x in &vals2 {
            for &y in &vals2 {
                for &z in &vals2 {
                    let q = Quaternion::new(w, x, y, z);
                    let mag = (w*w + x*x + y*y + z*z).sqrt();
                    if (mag - 1.0).abs() > 0.01 { continue; }
                    let q = Quaternion::new(w/mag, x/mag, y/mag, z/mag);

                    let (fwd, _right, down) = body_axes(q);
                    let thrust = Vector3::new(-down.x, -down.y, -down.z);
                    let cam_orient = CAM_TO_NED * q;
                    let optical = optical_axis(cam_orient);

                    // Constraints:
                    // 1. thrust = (0, -1, 0)
                    let thrust_ok = (thrust.y + 1.0).abs() < 0.01 && thrust.x.abs() < 0.01 && thrust.z.abs() < 0.01;
                    // 2. optical = fwd (camera looks where drone goes)
                    let optical_ok = (optical - fwd).magnitude() < 0.01;
                    // 3. fwd in XZ plane (fwd.y ≈ 0)
                    let fwd_horiz = fwd.y.abs() < 0.01;

                    if thrust_ok && optical_ok && fwd_horiz {
                        println!("*** FOUND: q=({:.4}, {:.4}, {:.4}, {:.4})", q.s, q.v.x, q.v.y, q.v.z);
                        println!("    fwd=({:.3},{:.3},{:.3}) optical=({:.3},{:.3},{:.3}) thrust=({:.3},{:.3},{:.3})",
                            fwd.x, fwd.y, fwd.z, optical.x, optical.y, optical.z, thrust.x, thrust.y, thrust.z);
                        found = true;
                    }
                }
            }
        }
    }
    if !found {
        println!("No quaternion found with all constraints satisfied!");
        println!("This means CAM_TO_NED cannot produce optical=fwd for COLMAP orientations.");
        println!("A different approach is needed (e.g., modified camera_transform for COLMAP).");
    }
}
