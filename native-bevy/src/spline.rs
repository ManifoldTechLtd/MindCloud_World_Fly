//! Centripetal Catmull-Rom spline, closed-loop only. Ported **verbatim** from
//! `native/src/spline.rs` (itself a port of `src/catmull-rom.js`). Pure cgmath, no Bevy.

use cgmath::Vector3;

const ALPHA: f32 = 0.5;
const EPS: f32 = 1e-6;
const DELTA: f32 = 1e-4;

type Pt = Vector3<f32>;

fn dist(a: Pt, b: Pt) -> f32 {
    let d = b - a;
    (d.x * d.x + d.y * d.y + d.z * d.z).sqrt()
}

fn lerp_pt(a: Pt, b: Pt, s: f32) -> Pt {
    a + (b - a) * s
}

fn pos_at(p0: Pt, p1: Pt, p2: Pt, p3: Pt, t0: f32, t1: f32, t2: f32, t3: f32, u: f32) -> Pt {
    let d01 = (t1 - t0).max(EPS);
    let d12 = (t2 - t1).max(EPS);
    let d23 = (t3 - t2).max(EPS);
    let d02 = (t2 - t0).max(EPS);
    let d13 = (t3 - t1).max(EPS);

    let a1 = lerp_pt(p0, p1, (u - t0) / d01);
    let a2 = lerp_pt(p1, p2, (u - t1) / d12);
    let a3 = lerp_pt(p2, p3, (u - t2) / d23);
    let b1 = lerp_pt(a1, a2, (u - t0) / d02);
    let b2 = lerp_pt(a2, a3, (u - t1) / d13);
    lerp_pt(b1, b2, (u - t1) / d12)
}

pub struct SplineResult {
    pub pos: Pt,
    pub tangent: Pt,
}

/// Evaluate the closed-loop Catmull-Rom curve on segment `i` at parameter t ∈ [0, 1].
pub fn evaluate_closed(points: &[Pt], i: usize, t: f32) -> SplineResult {
    let n = points.len();
    assert!(n >= 3, "closed Catmull-Rom requires >= 3 points");

    let p0 = points[(i + n - 1) % n];
    let p1 = points[i % n];
    let p2 = points[(i + 1) % n];
    let p3 = points[(i + 2) % n];

    let t0 = 0.0f32;
    let t1 = t0 + dist(p0, p1).powf(ALPHA).max(EPS);
    let t2 = t1 + dist(p1, p2).powf(ALPHA).max(EPS);
    let t3 = t2 + dist(p2, p3).powf(ALPHA).max(EPS);

    let u = t1 + t.clamp(0.0, 1.0) * (t2 - t1);
    let pos = pos_at(p0, p1, p2, p3, t0, t1, t2, t3, u);

    // Tangent via central difference
    let du = DELTA * (t2 - t1);
    let u_lo = (u - du).max(t1);
    let u_hi = (u + du).min(t2);
    let p_lo = pos_at(p0, p1, p2, p3, t0, t1, t2, t3, u_lo);
    let p_hi = pos_at(p0, p1, p2, p3, t0, t1, t2, t3, u_hi);
    let mut tv = p_hi - p_lo;
    let len = (tv.x * tv.x + tv.y * tv.y + tv.z * tv.z).sqrt();
    if len > 1e-9 {
        tv /= len;
    } else {
        tv = p2 - p1;
        let clen = (tv.x * tv.x + tv.y * tv.y + tv.z * tv.z).sqrt().max(1.0);
        tv /= clen;
    }

    SplineResult { pos, tangent: tv }
}

/// Unit tangent at a control point (average of incoming and outgoing segment tangents).
pub fn tangent_at_point(points: &[Pt], i: usize) -> Pt {
    let n = points.len();
    if n < 3 {
        return Vector3::new(1.0, 0.0, 0.0);
    }
    let inc = evaluate_closed(points, (i + n - 1) % n, 1.0).tangent;
    let out = evaluate_closed(points, i % n, 0.0).tangent;
    let mut t = inc + out;
    let len = (t.x * t.x + t.y * t.y + t.z * t.z).sqrt();
    if len > 1e-9 {
        t /= len;
    } else {
        t = Vector3::new(1.0, 0.0, 0.0);
    }
    t
}

/// Sample the whole closed curve at `samples_per_segment` per segment → flat `Vec<SplineResult>`.
/// Used by the editor preview + gate-mesh debug rendering. `[]` when < 3 points.
pub fn sample_closed(points: &[Pt], samples_per_segment: usize) -> Vec<SplineResult> {
    let n = points.len();
    if n < 3 {
        return Vec::new();
    }
    let sps = samples_per_segment.max(1);
    let mut out = Vec::with_capacity(n * sps);
    for i in 0..n {
        for s in 0..sps {
            let t = s as f32 / sps as f32;
            out.push(evaluate_closed(points, i, t));
        }
    }
    out
}
