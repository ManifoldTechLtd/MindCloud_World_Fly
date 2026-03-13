/*
 * Copyright 2026 Manifold Tech Ltd.
 * Author: MENG Guotao <mengguotao@manifoldtech.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Octree-based collision detection for point cloud data.
 * Builds a spatial index from Gaussian center positions and provides
 * sphere-vs-pointcloud queries for drone collision.
 */

const MAX_POINTS_PER_NODE = 64;
const MAX_DEPTH = 12;

class OctreeNode {
    constructor(minX, minY, minZ, maxX, maxY, maxZ, depth) {
        this.minX = minX; this.minY = minY; this.minZ = minZ;
        this.maxX = maxX; this.maxY = maxY; this.maxZ = maxZ;
        this.depth = depth;
        this.children = null; // null = leaf
        this.pointIndices = [];
    }

    get midX() { return (this.minX + this.maxX) * 0.5; }
    get midY() { return (this.minY + this.maxY) * 0.5; }
    get midZ() { return (this.minZ + this.maxZ) * 0.5; }

    intersectsSphere(cx, cy, cz, r) {
        // Closest point on AABB to sphere center
        const dx = Math.max(this.minX - cx, 0, cx - this.maxX);
        const dy = Math.max(this.minY - cy, 0, cy - this.maxY);
        const dz = Math.max(this.minZ - cz, 0, cz - this.maxZ);
        return (dx * dx + dy * dy + dz * dz) <= (r * r);
    }
}

export class Octree {
    constructor() {
        this.root = null;
        this.positions = null;
        this.pointCount = 0;
    }

    /**
     * Build the octree from a Float32Array of positions [x0,y0,z0, x1,y1,z1, ...]
     */
    build(positions, bounds) {
        this.positions = positions;
        this.pointCount = positions.length / 3;

        // Add small padding to bounds
        const pad = 0.1;
        this.root = new OctreeNode(
            bounds.min[0] - pad, bounds.min[1] - pad, bounds.min[2] - pad,
            bounds.max[0] + pad, bounds.max[1] + pad, bounds.max[2] + pad,
            0
        );

        // Insert all points
        const indices = [];
        for (let i = 0; i < this.pointCount; i++) {
            indices.push(i);
        }
        this._subdivide(this.root, indices);

        return this;
    }

    _subdivide(node, indices) {
        if (indices.length <= MAX_POINTS_PER_NODE || node.depth >= MAX_DEPTH) {
            node.pointIndices = indices;
            return;
        }

        node.children = new Array(8);
        const mx = node.midX, my = node.midY, mz = node.midZ;
        const childBounds = [
            [node.minX, node.minY, node.minZ, mx, my, mz],
            [mx, node.minY, node.minZ, node.maxX, my, mz],
            [node.minX, my, node.minZ, mx, node.maxY, mz],
            [mx, my, node.minZ, node.maxX, node.maxY, mz],
            [node.minX, node.minY, mz, mx, my, node.maxZ],
            [mx, node.minY, mz, node.maxX, my, node.maxZ],
            [node.minX, my, mz, mx, node.maxY, node.maxZ],
            [mx, my, mz, node.maxX, node.maxY, node.maxZ],
        ];

        const childIndices = [[], [], [], [], [], [], [], []];

        for (const idx of indices) {
            const x = this.positions[idx * 3];
            const y = this.positions[idx * 3 + 1];
            const z = this.positions[idx * 3 + 2];
            const ci = (x >= mx ? 1 : 0) | (y >= my ? 2 : 0) | (z >= mz ? 4 : 0);
            childIndices[ci].push(idx);
        }

        for (let i = 0; i < 8; i++) {
            const b = childBounds[i];
            node.children[i] = new OctreeNode(b[0], b[1], b[2], b[3], b[4], b[5], node.depth + 1);
            if (childIndices[i].length > 0) {
                this._subdivide(node.children[i], childIndices[i]);
            }
        }
    }

    /**
     * Collect AABBs of all non-empty leaf nodes.
     * Returns a Float32Array: [minX,minY,minZ, maxX,maxY,maxZ, ...] (6 floats per leaf).
     * Result is cached after first call.
     */
    getLeafBounds() {
        if (this._leafBoundsCache) return this._leafBoundsCache;
        const list = [];
        if (this.root) this._collectLeaves(this.root, list);
        this._leafBoundsCache = new Float32Array(list);
        return this._leafBoundsCache;
    }

    _collectLeaves(node, list) {
        if (node.children) {
            for (const child of node.children) {
                if (child) this._collectLeaves(child, list);
            }
        } else if (node.pointIndices.length > 0) {
            list.push(node.minX, node.minY, node.minZ, node.maxX, node.maxY, node.maxZ);
        }
    }

    /**
     * Query all points within a sphere. Returns array of { index, x, y, z, distSq }.
     */
    querySphere(cx, cy, cz, radius) {
        const results = [];
        const rSq = radius * radius;
        if (this.root) {
            this._querySphereNode(this.root, cx, cy, cz, radius, rSq, results);
        }
        return results;
    }

    _querySphereNode(node, cx, cy, cz, r, rSq, results) {
        if (!node.intersectsSphere(cx, cy, cz, r)) return;

        if (node.children) {
            for (const child of node.children) {
                if (child) this._querySphereNode(child, cx, cy, cz, r, rSq, results);
            }
        } else {
            for (const idx of node.pointIndices) {
                const px = this.positions[idx * 3];
                const py = this.positions[idx * 3 + 1];
                const pz = this.positions[idx * 3 + 2];
                const dx = px - cx, dy = py - cy, dz = pz - cz;
                const distSq = dx * dx + dy * dy + dz * dz;
                if (distSq <= rSq) {
                    results.push({ index: idx, x: px, y: py, z: pz, distSq });
                }
            }
        }
    }
}

/**
 * Collision detection result processor.
 * Given a sphere query result, computes collision normal and penetration.
 */
export function computeCollisionResponse(dronePos, radius, queryResults) {
    if (queryResults.length === 0) {
        return null;
    }

    // Compute average direction from nearby points to drone (surface normal estimate)
    let nx = 0, ny = 0, nz = 0;
    let minDist = Infinity;

    for (const pt of queryResults) {
        const dx = dronePos.x - pt.x;
        const dy = dronePos.y - pt.y;
        const dz = dronePos.z - pt.z;
        const dist = Math.sqrt(pt.distSq);
        if (dist < 0.0001) continue;

        // Weight by inverse distance (closer points contribute more to normal)
        const w = 1.0 / (dist + 0.001);
        nx += dx * w;
        ny += dy * w;
        nz += dz * w;

        if (dist < minDist) minDist = dist;
    }

    // Normalize
    const len = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 0.0001) {
        // Degenerate case: push straight up
        return { normal: { x: 0, y: 1, z: 0 }, penetration: radius - minDist, pointCount: queryResults.length };
    }

    nx /= len; ny /= len; nz /= len;
    const penetration = radius - minDist;

    return {
        normal: { x: nx, y: ny, z: nz },
        penetration: Math.max(0, penetration),
        pointCount: queryResults.length
    };
}
