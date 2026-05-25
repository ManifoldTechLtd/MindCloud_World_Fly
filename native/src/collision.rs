/// Octree-based collision detection for point cloud data.
/// Ported from src/collision.js.

use cgmath::Vector3;

const MAX_POINTS_PER_NODE: usize = 64;
const MAX_DEPTH: u32 = 12;

struct OctreeNode {
    min: [f32; 3],
    max: [f32; 3],
    depth: u32,
    children: Option<Box<[OctreeNode; 8]>>,
    point_indices: Vec<u32>,
}

impl OctreeNode {
    fn new(min: [f32; 3], max: [f32; 3], depth: u32) -> Self {
        Self { min, max, depth, children: None, point_indices: Vec::new() }
    }

    fn mid(&self) -> [f32; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    fn intersects_sphere(&self, cx: f32, cy: f32, cz: f32, r: f32) -> bool {
        let dx = (self.min[0] - cx).max(0.0f32).max(cx - self.max[0]);
        let dy = (self.min[1] - cy).max(0.0f32).max(cy - self.max[1]);
        let dz = (self.min[2] - cz).max(0.0f32).max(cz - self.max[2]);
        dx * dx + dy * dy + dz * dz <= r * r
    }
}

pub struct SphereHit {
    pub index: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub dist_sq: f32,
}

pub struct CollisionResult {
    pub normal: Vector3<f32>,
    pub penetration: f32,
}

pub struct Octree {
    root: Option<OctreeNode>,
    positions: Vec<f32>, // [x0,y0,z0, x1,y1,z1, ...]
    point_count: usize,
}

impl Octree {
    pub fn new() -> Self {
        Self { root: None, positions: Vec::new(), point_count: 0 }
    }

    /// Build from a flat f32 slice of positions [x,y,z,...] and AABB bounds.
    pub fn build(&mut self, positions: Vec<f32>, bounds_min: [f32; 3], bounds_max: [f32; 3]) {
        self.positions = positions;
        self.point_count = self.positions.len() / 3;

        let pad = 0.1;
        let mut root = OctreeNode::new(
            [bounds_min[0] - pad, bounds_min[1] - pad, bounds_min[2] - pad],
            [bounds_max[0] + pad, bounds_max[1] + pad, bounds_max[2] + pad],
            0,
        );

        let indices: Vec<u32> = (0..self.point_count as u32).collect();
        Self::subdivide(&mut root, &indices, &self.positions);
        self.root = Some(root);
    }

    fn subdivide(node: &mut OctreeNode, indices: &[u32], positions: &[f32]) {
        if indices.len() <= MAX_POINTS_PER_NODE || node.depth >= MAX_DEPTH {
            node.point_indices = indices.to_vec();
            return;
        }

        let [mx, my, mz] = node.mid();
        let mn = node.min;
        let mx_ = node.max;
        let child_bounds: [[f32; 6]; 8] = [
            [mn[0], mn[1], mn[2], mx, my, mz],
            [mx, mn[1], mn[2], mx_[0], my, mz],
            [mn[0], my, mn[2], mx, mx_[1], mz],
            [mx, my, mn[2], mx_[0], mx_[1], mz],
            [mn[0], mn[1], mz, mx, my, mx_[2]],
            [mx, mn[1], mz, mx_[0], my, mx_[2]],
            [mn[0], my, mz, mx, mx_[1], mx_[2]],
            [mx, my, mz, mx_[0], mx_[1], mx_[2]],
        ];

        let mut child_indices: [Vec<u32>; 8] = Default::default();

        for &idx in indices {
            let i = idx as usize;
            let x = positions[i * 3];
            let y = positions[i * 3 + 1];
            let z = positions[i * 3 + 2];
            let ci = (if x >= mx { 1 } else { 0 })
                | (if y >= my { 2 } else { 0 })
                | (if z >= mz { 4 } else { 0 });
            child_indices[ci].push(idx);
        }

        let mut children: [OctreeNode; 8] = std::array::from_fn(|i| {
            let b = &child_bounds[i];
            OctreeNode::new([b[0], b[1], b[2]], [b[3], b[4], b[5]], node.depth + 1)
        });

        for i in 0..8 {
            if !child_indices[i].is_empty() {
                Self::subdivide(&mut children[i], &child_indices[i], positions);
            }
        }

        node.children = Some(Box::new(children));
    }

    /// Query all points within a sphere.
    pub fn query_sphere(&self, cx: f32, cy: f32, cz: f32, radius: f32) -> Vec<SphereHit> {
        let mut results = Vec::new();
        let r_sq = radius * radius;
        if let Some(root) = &self.root {
            Self::query_sphere_node(root, cx, cy, cz, radius, r_sq, &self.positions, &mut results);
        }
        results
    }

    fn query_sphere_node(
        node: &OctreeNode, cx: f32, cy: f32, cz: f32,
        r: f32, r_sq: f32, positions: &[f32], results: &mut Vec<SphereHit>,
    ) {
        if !node.intersects_sphere(cx, cy, cz, r) { return; }

        if let Some(children) = &node.children {
            for child in children.iter() {
                Self::query_sphere_node(child, cx, cy, cz, r, r_sq, positions, results);
            }
        } else {
            for &idx in &node.point_indices {
                let i = idx as usize;
                let px = positions[i * 3];
                let py = positions[i * 3 + 1];
                let pz = positions[i * 3 + 2];
                let dx = px - cx;
                let dy = py - cy;
                let dz = pz - cz;
                let dist_sq = dx * dx + dy * dy + dz * dz;
                if dist_sq <= r_sq {
                    results.push(SphereHit { index: idx, x: px, y: py, z: pz, dist_sq });
                }
            }
        }
    }
}

/// Compute collision normal and penetration from sphere query results.
pub fn compute_collision_response(
    drone_x: f32, drone_y: f32, drone_z: f32,
    radius: f32,
    hits: &[SphereHit],
) -> Option<CollisionResult> {
    if hits.is_empty() { return None; }

    let mut nx = 0.0f32;
    let mut ny = 0.0f32;
    let mut nz = 0.0f32;
    let mut min_dist = f32::INFINITY;

    for pt in hits {
        let dist = pt.dist_sq.sqrt();
        if dist < 0.0001 { continue; }

        let dx = drone_x - pt.x;
        let dy = drone_y - pt.y;
        let dz = drone_z - pt.z;

        let w = 1.0 / (dist + 0.001);
        nx += dx * w;
        ny += dy * w;
        nz += dz * w;

        if dist < min_dist { min_dist = dist; }
    }

    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    if len < 0.0001 {
        return Some(CollisionResult {
            normal: Vector3::new(0.0, 1.0, 0.0),
            penetration: (radius - min_dist).max(0.0),
        });
    }

    nx /= len; ny /= len; nz /= len;

    Some(CollisionResult {
        normal: Vector3::new(nx, ny, nz),
        penetration: (radius - min_dist).max(0.0),
    })
}
