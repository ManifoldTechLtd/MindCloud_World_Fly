// we cutoff at 1/255 alpha value 
const CUTOFF:f32 = 2.3539888583335364; // = sqrt(log(255))

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) screen_pos: vec2<f32>,
    @location(1) color: vec4<f32>,
};

struct VertexInput {
    @location(0) v: vec4<f32>,
    @location(1) pos: vec4<f32>,
    @location(2) color: vec4<f32>,
};

struct Splat {
     // 4x f16 packed as u32
    v_0: u32, v_1: u32,
    // 2x f16 packed as u32
    pos: u32,
    // rgba packed as f16
    color_0: u32,color_1: u32,
    // depth in host clip space (reverse-Z NDC) for depth testing
    depth: f32,
};

@group(0) @binding(2)
var<storage, read> points_2d : array<Splat>;
@group(1) @binding(4)
var<storage, read> indices : array<u32>;

@vertex
fn vs_main(
    @builtin(vertex_index) in_vertex_index: u32,
    @builtin(instance_index) in_instance_index: u32
) -> VertexOutput {
    var out: VertexOutput;

    let vertex = points_2d[indices[in_instance_index] + 0u];

    // scaled eigenvectors in screen space 
    let v1 = unpack2x16float(vertex.v_0);
    let v2 = unpack2x16float(vertex.v_1);

    let v_center = unpack2x16float(vertex.pos);

    // splat rectangle with left lower corner at (-1,-1)
    // and upper right corner at (1,1)
    let x = f32(in_vertex_index % 2u == 0u) * 2. - (1.);
    let y = f32(in_vertex_index < 2u) * 2. - (1.);

    let position = vec2<f32>(x, y) * CUTOFF;

    let offset = 2. * mat2x2<f32>(v1, v2) * position;
    out.position = vec4<f32>(v_center + offset, vertex.depth, 1.);
    out.screen_pos = position;
    out.color = vec4<f32>(unpack2x16float(vertex.color_0), unpack2x16float(vertex.color_1));
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let a = dot(in.screen_pos, in.screen_pos);
    if a > 2. * CUTOFF {
        discard;
    }
    let b = min(0.99, exp(-a) * in.color.a);
    return vec4<f32>(in.color.rgb, 1.) * b;
}

// Depth-pass base-alpha cutoff: splats whose CENTRE alpha is below this are collapsed in the
// vertex shader (never rasterized). Lower = more surfaces occlude, higher = faster.
const DEPTH_BASE_ALPHA: f32 = 0.4;

// Depth-pass vertex: identical geometry to vs_main, but splats too faint to count as solid
// are collapsed into a zero-area quad (all 4 verts at the origin => culled by the
// rasterizer). Doing the cutoff HERE instead of discarding in the fragment shader keeps
// hardware early depth testing active — with the front-to-back sort, splats behind the first
// solid cover are rejected per pixel without running any fragment work (the big speedup).
@vertex
fn vs_depth(
    @builtin(vertex_index) in_vertex_index: u32,
    @builtin(instance_index) in_instance_index: u32
) -> VertexOutput {
    var out: VertexOutput;

    let vertex = points_2d[indices[in_instance_index] + 0u];
    let color = vec4<f32>(unpack2x16float(vertex.color_0), unpack2x16float(vertex.color_1));

    // Too faint to occlude: emit a degenerate quad (rasterizer culls it, zero fragments).
    if color.a < DEPTH_BASE_ALPHA {
        out.position = vec4<f32>(0., 0., 0., 1.);
        out.screen_pos = vec2<f32>(0., 0.);
        out.color = vec4<f32>(0.);
        return out;
    }

    let v1 = unpack2x16float(vertex.v_0);
    let v2 = unpack2x16float(vertex.v_1);
    let v_center = unpack2x16float(vertex.pos);

    let x = f32(in_vertex_index % 2u == 0u) * 2. - (1.);
    let y = f32(in_vertex_index < 2u) * 2. - (1.);
    // Shrink the quad to the core of the gaussian (where per-pixel alpha would clear the
    // solidity cutoff anyway) instead of the full ±CUTOFF soft footprint — tighter
    // silhouettes AND fewer fragments than the color pass.
    let position = vec2<f32>(x, y) * 1.0;

    let offset = 2. * mat2x2<f32>(v1, v2) * position;
    out.position = vec4<f32>(v_center + offset, vertex.depth, 1.);
    out.screen_pos = position;
    out.color = color;
    return out;
}

// Depth-only fragment (no color targets, depth_write ON): intentionally EMPTY — no discard,
// so early-z stays enabled and each surviving splat just stamps its quad depth. Faint splats
// were already culled in vs_depth; the reverse-Z Greater test keeps the nearest surface.
@fragment
fn fs_depth(in: VertexOutput) {
}