//! Numerically find the rotation conversion that makes the web-splat camera render the SAME view
//! as Bevy's camera (so splat background and Bevy meshes stay locked under ALL camera rotations).
//!
//! The mesh pass uses Bevy's camera (looks -Z, +Y up). The splat pass uses web-splat's camera
//! (`world2view` + `VIEWPORT_Y_FLIP * build_proj`, looks +Z, +Y down) with rotation derived from
//! the Bevy rotation. We project sample world points through BOTH pipelines and compare NDC.xy.
//! The candidate conversion with ~0 error across all rotations is the correct one.
//!
//! Run: cargo test --test splat_camera_align -- --nocapture

use bevy::math::{Mat4, Quat, Vec3};
use cgmath::{
    Matrix, Matrix3, Matrix4 as CMat4, Quaternion as CQuat, Rad, Transform as CTransform,
    Vector3 as CVec3, Vector4 as CVec4, Zero,
};

// ---- web-splat camera math (copied verbatim from web-splat/src/camera.rs) ----

fn world2view(r: Matrix3<f32>, t: CVec3<f32>) -> CMat4<f32> {
    let mut rt = CMat4::from(r);
    rt[0].w = t.x;
    rt[1].w = t.y;
    rt[2].w = t.z;
    rt[3].w = 1.0;
    rt.inverse_transform().unwrap().transpose()
}

#[rustfmt::skip]
const VIEWPORT_Y_FLIP: CMat4<f32> = CMat4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
);

fn build_proj(znear: f32, zfar: f32, fov_x: Rad<f32>, fov_y: Rad<f32>) -> CMat4<f32> {
    let tan_half_fov_y = (fov_y / 2.0).0.tan();
    let tan_half_fov_x = (fov_x / 2.0).0.tan();
    let top = tan_half_fov_y * znear;
    let bottom = -top;
    let right = tan_half_fov_x * znear;
    let left = -right;
    let mut p = CMat4::zero();
    p[0][0] = 2.0 * znear / (right - left);
    p[1][1] = 2.0 * znear / (top - bottom);
    p[0][2] = (right + left) / (right - left);
    p[1][2] = (top + bottom) / (top - bottom);
    p[3][2] = 1.0;
    p[2][2] = zfar / (zfar - znear);
    p[2][3] = -(zfar * znear) / (zfar - znear);
    p.transpose()
}

// glam Quat -> cgmath Quaternion (w, x, y, z)
fn to_cq(q: Quat) -> CQuat<f32> {
    CQuat::new(q.w, q.x, q.y, q.z)
}

const FLIP_X: CQuat<f32> = CQuat {
    s: 0.0,
    v: CVec3 { x: 1.0, y: 0.0, z: 0.0 },
};

/// Bevy mesh-pass NDC.xy of world point `p` for camera (rot, pos). Square aspect, 60° fov.
fn bevy_ndc(rot: Quat, pos: Vec3, fov_y: f32, near: f32, p: Vec3) -> (f32, f32) {
    let world_from_view = Mat4::from_rotation_translation(rot, pos);
    let view_from_world = world_from_view.inverse();
    let proj = Mat4::perspective_infinite_reverse_rh(fov_y, 1.0, near);
    let clip = proj * (view_from_world * p.extend(1.0));
    (clip.x / clip.w, clip.y / clip.w)
}

/// web-splat splat-pass NDC.xy of world point `p` for splat camera (splat_rot, pos).
fn splat_ndc(splat_rot: CQuat<f32>, pos: Vec3, fov: f32, near: f32, far: f32, p: Vec3) -> (f32, f32) {
    let view = world2view(Matrix3::from(splat_rot), CVec3::new(pos.x, pos.y, pos.z));
    let proj = VIEWPORT_Y_FLIP * build_proj(near, far, Rad(fov), Rad(fov));
    let clip = proj * (view * CVec4::new(p.x, p.y, p.z, 1.0));
    (clip.x / clip.w, clip.y / clip.w)
}

#[test]
fn find_splat_rotation_conversion() {
    let fov = 60.0_f32.to_radians();
    let near = 0.1_f32;
    let far = 1000.0_f32;
    let pos = Vec3::new(1.0, 2.0, 3.0);

    // A spread of camera orientations (the symptom is rotation-dependent, so test many).
    let rots: Vec<(&str, Quat)> = vec![
        ("identity", Quat::IDENTITY),
        ("yaw30", Quat::from_axis_angle(Vec3::Y, 30f32.to_radians())),
        ("pitch30", Quat::from_axis_angle(Vec3::X, 30f32.to_radians())),
        ("roll30", Quat::from_axis_angle(Vec3::Z, 30f32.to_radians())),
        (
            "yaw40_pitch20",
            Quat::from_axis_angle(Vec3::Y, 40f32.to_radians())
                * Quat::from_axis_angle(Vec3::X, 20f32.to_radians()),
        ),
        (
            "yaw-70_pitch-25_roll15",
            Quat::from_axis_angle(Vec3::Y, -70f32.to_radians())
                * Quat::from_axis_angle(Vec3::X, -25f32.to_radians())
                * Quat::from_axis_angle(Vec3::Z, 15f32.to_radians()),
        ),
    ];

    // Candidate conversions: bevy rotation (cgmath) -> splat rotation.
    let candidates: Vec<(&str, fn(CQuat<f32>) -> CQuat<f32>)> = vec![
        ("bevy_q * flip_x (current)", |q| q * FLIP_X),
        ("flip_x * bevy_q", |q| FLIP_X * q),
        ("bevy_q", |q| q),
        ("conj(bevy_q)", |q| q.conjugate()),
        ("flip_x * conj(bevy_q)", |q| FLIP_X * q.conjugate()),
        ("conj(bevy_q) * flip_x", |q| q.conjugate() * FLIP_X),
        ("conj(bevy_q * flip_x)", |q| (q * FLIP_X).conjugate()),
    ];

    // Sample world points placed in front of the camera for each rotation (so both pipelines see
    // them with the right sign of depth), plus lateral/vertical offsets to catch rotation errors.
    let offsets = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(3.0, 0.0, 0.0),
        Vec3::new(0.0, 3.0, 0.0),
        Vec3::new(2.0, 2.0, 0.0),
        Vec3::new(-2.0, 1.0, 1.5),
    ];

    println!("\n=== Splat camera rotation conversion search (NDC.xy error vs Bevy) ===");
    for (cname, conv) in &candidates {
        let mut max_err = 0.0_f32;
        for (_rname, rot) in &rots {
            // Forward in Bevy convention = rot * -Z; place points ~10m ahead + offsets (also
            // offset laterally in the camera's right/up so they project on-screen).
            let fwd = *rot * Vec3::NEG_Z;
            let right = *rot * Vec3::X;
            let up = *rot * Vec3::Y;
            for off in &offsets {
                let p = pos + fwd * 10.0 + right * off.x + up * off.y + fwd * off.z;
                let (bx, by) = bevy_ndc(*rot, pos, fov, near, p);
                let (sx, sy) = splat_ndc(conv(to_cq(*rot)), pos, fov, near, far, p);
                let err = ((bx - sx).powi(2) + (by - sy).powi(2)).sqrt();
                if err.is_finite() {
                    max_err = max_err.max(err);
                }
            }
        }
        println!("  {:<28} max NDC.xy error = {:.6}", cname, max_err);
    }
    println!("=== (the conversion with ~0 error is the correct splat rotation) ===\n");
}

/// The real app is widescreen (fovx != fovy) and the splat node derives fovx from Bevy's
/// projection. Assert the conjugate conversion keeps splat + meshes aligned there too.
#[test]
fn conjugate_conversion_holds_for_widescreen() {
    let fovy = 60.0_f32.to_radians();
    let aspect = 2560.0 / 1600.0;
    let near = 0.1_f32;
    let far = 1000.0_f32;
    let pos = Vec3::new(-2.0, 1.5, 4.0);
    // Splat node derivation: m00 = (1/tan(fovy/2))/aspect => fovx = 2*atan(aspect*tan(fovy/2)).
    let fovx = 2.0 * (aspect * (fovy * 0.5).tan()).atan();

    let rots = [
        Quat::IDENTITY,
        Quat::from_axis_angle(Vec3::Y, 55f32.to_radians()),
        Quat::from_axis_angle(Vec3::X, -35f32.to_radians()),
        Quat::from_axis_angle(Vec3::Y, 120f32.to_radians())
            * Quat::from_axis_angle(Vec3::X, 20f32.to_radians())
            * Quat::from_axis_angle(Vec3::Z, -10f32.to_radians()),
    ];
    let offsets = [
        Vec3::ZERO,
        Vec3::new(4.0, 0.0, 0.0),
        Vec3::new(0.0, 3.0, 0.0),
        Vec3::new(-3.0, -2.0, 2.0),
    ];

    let mut max_err = 0.0_f32;
    for rot in &rots {
        let fwd = *rot * Vec3::NEG_Z;
        let right = *rot * Vec3::X;
        let up = *rot * Vec3::Y;
        let view_from_world = Mat4::from_rotation_translation(*rot, pos).inverse();
        let bproj = Mat4::perspective_infinite_reverse_rh(fovy, aspect, near);
        let splat_rot = (to_cq(*rot) * FLIP_X).conjugate();
        let view = world2view(Matrix3::from(splat_rot), CVec3::new(pos.x, pos.y, pos.z));
        let sproj = VIEWPORT_Y_FLIP * build_proj(near, far, Rad(fovx), Rad(fovy));
        for off in &offsets {
            let p = pos + fwd * 10.0 + right * off.x + up * off.y + fwd * off.z;
            let bc = bproj * (view_from_world * p.extend(1.0));
            let sc = sproj * (view * CVec4::new(p.x, p.y, p.z, 1.0));
            let err = ((bc.x / bc.w - sc.x / sc.w).powi(2)
                + (bc.y / bc.w - sc.y / sc.w).powi(2))
            .sqrt();
            if err.is_finite() {
                max_err = max_err.max(err);
            }
        }
    }
    println!("widescreen conjugate max NDC.xy error = {:.6}", max_err);
    assert!(max_err < 1e-4, "splat/mesh misaligned at widescreen: {}", max_err);
}

/// Full app pipeline. `compute_orbit_camera` / `drone.camera_transform()` produce a web-splat
/// convention rotation `q_wb`; the INTENDED view is what web-splat renders with `q_wb` (this is the
/// behaviour the user confirmed was correct). The Bevy camera must be built so the MESHES render
/// that same view and the splat (via the node's `(bevy_q*flip_x).conjugate()`) matches it.
///
/// Asserts: with `bevy_q = q_wb.conjugate()*flip_x`, both meshes and splat == intended web-splat(q_wb).
/// Also shows the OLD `bevy_q = q_wb*flip_x` makes the meshes diverge from the intended view.
#[test]
fn app_pipeline_meshes_and_splat_match_intended_view() {
    let fov = 60.0_f32.to_radians();
    let near = 0.1_f32;
    let far = 1000.0_f32;
    let pos = Vec3::new(2.0, -1.0, 5.0);

    // Arbitrary web-splat-convention orientations q_wb (the orbit/drone output).
    let q_wbs = [
        Quat::IDENTITY,
        Quat::from_axis_angle(Vec3::Y, 35f32.to_radians()),
        Quat::from_axis_angle(Vec3::X, -25f32.to_radians()),
        Quat::from_axis_angle(Vec3::Z, 18f32.to_radians()),
        Quat::from_axis_angle(Vec3::Y, 100f32.to_radians())
            * Quat::from_axis_angle(Vec3::X, 30f32.to_radians())
            * Quat::from_axis_angle(Vec3::Z, -12f32.to_radians()),
    ];
    let offsets = [
        Vec3::ZERO,
        Vec3::new(3.0, 0.0, 0.0),
        Vec3::new(0.0, 2.5, 0.0),
        Vec3::new(-2.0, -1.5, 1.0),
    ];

    let mut max_err_new = 0.0_f32;
    let mut max_err_old = 0.0_f32;
    for q_wb_g in &q_wbs {
        let q_wb = to_cq(*q_wb_g);
        // NEW (correct) Bevy camera, and OLD (buggy) Bevy camera.
        let bevy_new_c = q_wb.conjugate() * FLIP_X;
        let bevy_old_c = q_wb * FLIP_X;
        let bevy_new = Quat::from_xyzw(bevy_new_c.v.x, bevy_new_c.v.y, bevy_new_c.v.z, bevy_new_c.s);
        let bevy_old = Quat::from_xyzw(bevy_old_c.v.x, bevy_old_c.v.y, bevy_old_c.v.z, bevy_old_c.s);
        // Splat node output for the NEW camera.
        let splat_new = (to_cq(bevy_new) * FLIP_X).conjugate();

        // Points in front of the intended (web-splat q_wb) view: forward +Z, up -Y, right +X.
        let fwd = *q_wb_g * Vec3::Z;
        let right = *q_wb_g * Vec3::X;
        let up = *q_wb_g * Vec3::NEG_Y;
        for off in &offsets {
            let p = pos + fwd * 10.0 + right * off.x + up * off.y;
            let reference = splat_ndc(q_wb, pos, fov, near, far, p); // intended/correct view
            let mesh_new = bevy_ndc(bevy_new, pos, fov, near, p);
            let splat_new_ndc = splat_ndc(splat_new, pos, fov, near, far, p);
            let mesh_old = bevy_ndc(bevy_old, pos, fov, near, p);
            let d = |a: (f32, f32), b: (f32, f32)| ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();
            if d(mesh_new, reference).is_finite() {
                max_err_new = max_err_new.max(d(mesh_new, reference));
                max_err_new = max_err_new.max(d(splat_new_ndc, reference));
            }
            if d(mesh_old, reference).is_finite() {
                max_err_old = max_err_old.max(d(mesh_old, reference));
            }
        }
    }
    println!("NEW (q_wb.conj()*flip_x): mesh+splat vs intended = {:.6}", max_err_new);
    println!("OLD (q_wb*flip_x):        mesh      vs intended = {:.6}", max_err_old);
    assert!(max_err_new < 1e-4, "NEW pipeline should match intended view: {}", max_err_new);
    assert!(max_err_old > 0.1, "OLD pipeline should visibly diverge (sanity): {}", max_err_old);
}
