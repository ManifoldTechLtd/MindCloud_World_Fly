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
 * PLY Parser - extracts Gaussian center positions (x, y, z) from a .ply file
 * for use in the collision system. Supports both ASCII and binary (little-endian) PLY formats.
 * Applies Z-up to Y-up coordinate transform: x' = x, y' = z, z' = -y
 */

export function parsePlyForPositions(arrayBuffer, options = {}) {
    const zUp = options.zUp !== undefined ? options.zUp : true;
    const decoder = new TextDecoder('utf-8');
    const bytes = new Uint8Array(arrayBuffer);

    // Find end of header
    let headerEnd = -1;
    for (let i = 0; i < Math.min(bytes.length, 65536); i++) {
        if (bytes[i] === 0x65 && bytes[i+1] === 0x6e && bytes[i+2] === 0x64 &&
            bytes[i+3] === 0x5f && bytes[i+4] === 0x68 && bytes[i+5] === 0x65 &&
            bytes[i+6] === 0x61 && bytes[i+7] === 0x64 && bytes[i+8] === 0x65 &&
            bytes[i+9] === 0x72) {
            // Find newline after "end_header"
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            headerEnd = j + 1;
            break;
        }
    }

    if (headerEnd === -1) {
        throw new Error('Invalid PLY file: could not find end_header');
    }

    const headerText = decoder.decode(bytes.slice(0, headerEnd));
    const headerLines = headerText.split('\n').map(l => l.trim());

    // Parse header
    let vertexCount = 0;
    let format = 'ascii';
    const properties = [];
    let inVertexElement = false;

    for (const line of headerLines) {
        if (line.startsWith('format')) {
            if (line.includes('binary_little_endian')) format = 'binary_little_endian';
            else if (line.includes('binary_big_endian')) format = 'binary_big_endian';
            else format = 'ascii';
        } else if (line.startsWith('element vertex')) {
            vertexCount = parseInt(line.split(/\s+/)[2], 10);
            inVertexElement = true;
        } else if (line.startsWith('element') && inVertexElement) {
            inVertexElement = false;
        } else if (line.startsWith('property') && inVertexElement) {
            const parts = line.split(/\s+/);
            properties.push({ type: parts[1], name: parts[2] });
        }
    }

    if (vertexCount === 0) {
        throw new Error('PLY file has no vertices');
    }

    // Find x, y, z property indices
    const xIdx = properties.findIndex(p => p.name === 'x');
    const yIdx = properties.findIndex(p => p.name === 'y');
    const zIdx = properties.findIndex(p => p.name === 'z');

    if (xIdx === -1 || yIdx === -1 || zIdx === -1) {
        throw new Error('PLY file missing x, y, or z properties');
    }

    const positions = new Float32Array(vertexCount * 3);

    if (format === 'ascii') {
        const bodyText = decoder.decode(bytes.slice(headerEnd));
        const lines = bodyText.trim().split('\n');
        for (let i = 0; i < vertexCount && i < lines.length; i++) {
            const vals = lines[i].trim().split(/\s+/);
            const rawX = parseFloat(vals[xIdx]);
            const rawY = parseFloat(vals[yIdx]);
            const rawZ = parseFloat(vals[zIdx]);
            if (zUp) {
                // Z-up to Y-up: x' = x, y' = z, z' = -y
                positions[i * 3]     = rawX;
                positions[i * 3 + 1] = rawZ;
                positions[i * 3 + 2] = -rawY;
            } else {
                // Y-up (COLMAP/OpenCV convention: Y-down) → flip Y
                positions[i * 3]     = rawX;
                positions[i * 3 + 1] = -rawY;
                positions[i * 3 + 2] = -rawZ;
            }
        }
    } else {
        // Binary format
        const propSizes = properties.map(p => getPropertySize(p.type));
        const vertexStride = propSizes.reduce((a, b) => a + b, 0);
        const xOffset = propSizes.slice(0, xIdx).reduce((a, b) => a + b, 0);
        const yOffset = propSizes.slice(0, yIdx).reduce((a, b) => a + b, 0);
        const zOffset = propSizes.slice(0, zIdx).reduce((a, b) => a + b, 0);

        const dataView = new DataView(arrayBuffer, headerEnd);
        const isLittle = format === 'binary_little_endian';

        for (let i = 0; i < vertexCount; i++) {
            const base = i * vertexStride;
            const rawX = readFloat(dataView, base + xOffset, properties[xIdx].type, isLittle);
            const rawY = readFloat(dataView, base + yOffset, properties[yIdx].type, isLittle);
            const rawZ = readFloat(dataView, base + zOffset, properties[zIdx].type, isLittle);
            if (zUp) {
                positions[i * 3]     = rawX;
                positions[i * 3 + 1] = rawZ;
                positions[i * 3 + 2] = -rawY;
            } else {
                // Y-up (COLMAP/OpenCV convention: Y-down) → flip Y
                positions[i * 3]     = rawX;
                positions[i * 3 + 1] = -rawY;
                positions[i * 3 + 2] = -rawZ;
            }
        }
    }

    // Sanitize NaN/Inf positions to avoid poisoning octree and distance calculations
    for (let i = 0; i < vertexCount; i++) {
        const off = i * 3;
        if (!isFinite(positions[off]))     positions[off]     = 0;
        if (!isFinite(positions[off + 1])) positions[off + 1] = 0;
        if (!isFinite(positions[off + 2])) positions[off + 2] = 0;
    }

    // Compute bounding box (skip NaN/Inf vertices)
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (let i = 0; i < vertexCount; i++) {
        const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }

    return {
        positions,
        vertexCount,
        bounds: { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] }
    };
}

/**
 * Parse opacity values from a PLY binary.
 * 3DGS stores opacity as logit; we apply sigmoid to get [0,1].
 * Returns Float32Array of length vertexCount, or null if no opacity property.
 */
export function parsePlyOpacities(arrayBuffer) {
    const bytes = new Uint8Array(arrayBuffer);
    const decoder = new TextDecoder('utf-8');

    // Find header end
    let headerEnd = -1;
    for (let i = 0; i < Math.min(bytes.length, 65536); i++) {
        if (bytes[i] === 0x65 && bytes[i+1] === 0x6e && bytes[i+2] === 0x64 &&
            bytes[i+3] === 0x5f && bytes[i+4] === 0x68 && bytes[i+5] === 0x65 &&
            bytes[i+6] === 0x61 && bytes[i+7] === 0x64 && bytes[i+8] === 0x65 &&
            bytes[i+9] === 0x72) {
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            headerEnd = j + 1;
            break;
        }
    }
    if (headerEnd === -1) return null;

    const headerText = decoder.decode(bytes.slice(0, headerEnd));
    const headerLines = headerText.split('\n').map(l => l.trim());

    let vertexCount = 0;
    let format = 'ascii';
    const properties = [];
    let inVertex = false;

    for (const line of headerLines) {
        if (line.startsWith('format')) {
            if (line.includes('binary_little_endian')) format = 'binary_little_endian';
            else if (line.includes('binary_big_endian')) format = 'binary_big_endian';
            else format = 'ascii';
        } else if (line.startsWith('element vertex')) {
            vertexCount = parseInt(line.split(/\s+/)[2], 10);
            inVertex = true;
        } else if (line.startsWith('element') && inVertex) {
            inVertex = false;
        } else if (line.startsWith('property') && inVertex) {
            const parts = line.split(/\s+/);
            properties.push({ type: parts[1], name: parts[2] });
        }
    }

    const opIdx = properties.findIndex(p => p.name === 'opacity');
    if (opIdx === -1 || vertexCount === 0) return null;

    const sigmoid = x => 1.0 / (1.0 + Math.exp(-x));
    const opacities = new Float32Array(vertexCount);

    if (format === 'ascii') {
        const bodyText = decoder.decode(bytes.slice(headerEnd));
        const lines = bodyText.trim().split('\n');
        for (let i = 0; i < vertexCount && i < lines.length; i++) {
            opacities[i] = sigmoid(parseFloat(lines[i].trim().split(/\s+/)[opIdx]));
        }
    } else {
        const propSizes = properties.map(p => getPropertySize(p.type));
        const vertexStride = propSizes.reduce((a, b) => a + b, 0);
        const opOffset = propSizes.slice(0, opIdx).reduce((a, b) => a + b, 0);
        const dataView = new DataView(arrayBuffer, headerEnd);
        const isLittle = format === 'binary_little_endian';
        for (let i = 0; i < vertexCount; i++) {
            const raw = readFloat(dataView, i * vertexStride + opOffset, properties[opIdx].type, isLittle);
            opacities[i] = sigmoid(raw);
        }
    }
    return opacities;
}

/**
 * Compute centroid from raw (untransformed) PLY positions.
 * Returns { x, y, z } in the PLY file's native coordinate space.
 */
export function parsePlyRawCentroid(arrayBuffer) {
    const bytes = new Uint8Array(arrayBuffer);
    const decoder = new TextDecoder('utf-8');

    let headerEnd = -1;
    for (let i = 0; i < Math.min(bytes.length, 65536); i++) {
        if (bytes[i] === 0x65 && bytes[i+1] === 0x6e && bytes[i+2] === 0x64 &&
            bytes[i+3] === 0x5f && bytes[i+4] === 0x68 && bytes[i+5] === 0x65 &&
            bytes[i+6] === 0x61 && bytes[i+7] === 0x64 && bytes[i+8] === 0x65 &&
            bytes[i+9] === 0x72) {
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            headerEnd = j + 1;
            break;
        }
    }
    if (headerEnd === -1) return { x: 0, y: 0, z: 0 };

    const headerText = decoder.decode(bytes.slice(0, headerEnd));
    const headerLines = headerText.split('\n').map(l => l.trim());

    let vertexCount = 0;
    let format = 'ascii';
    const properties = [];
    let inVertex = false;

    for (const line of headerLines) {
        if (line.startsWith('format')) {
            if (line.includes('binary_little_endian')) format = 'binary_little_endian';
            else if (line.includes('binary_big_endian')) format = 'binary_big_endian';
            else format = 'ascii';
        } else if (line.startsWith('element vertex')) {
            vertexCount = parseInt(line.split(/\s+/)[2], 10);
            inVertex = true;
        } else if (line.startsWith('element') && inVertex) {
            inVertex = false;
        } else if (line.startsWith('property') && inVertex) {
            const parts = line.split(/\s+/);
            properties.push({ type: parts[1], name: parts[2] });
        }
    }

    const xIdx = properties.findIndex(p => p.name === 'x');
    const yIdx = properties.findIndex(p => p.name === 'y');
    const zIdx = properties.findIndex(p => p.name === 'z');
    if (xIdx === -1 || yIdx === -1 || zIdx === -1 || vertexCount === 0) return { x: 0, y: 0, z: 0 };

    let cx = 0, cy = 0, cz = 0;
    let validCount = 0;

    if (format === 'ascii') {
        const bodyText = decoder.decode(bytes.slice(headerEnd));
        const lines = bodyText.trim().split('\n');
        for (let i = 0; i < vertexCount && i < lines.length; i++) {
            const vals = lines[i].trim().split(/\s+/);
            const rx = parseFloat(vals[xIdx]), ry = parseFloat(vals[yIdx]), rz = parseFloat(vals[zIdx]);
            if (!isFinite(rx) || !isFinite(ry) || !isFinite(rz)) continue;
            cx += rx; cy += ry; cz += rz;
            validCount++;
        }
    } else {
        const propSizes = properties.map(p => getPropertySize(p.type));
        const vertexStride = propSizes.reduce((a, b) => a + b, 0);
        const xOff = propSizes.slice(0, xIdx).reduce((a, b) => a + b, 0);
        const yOff = propSizes.slice(0, yIdx).reduce((a, b) => a + b, 0);
        const zOff = propSizes.slice(0, zIdx).reduce((a, b) => a + b, 0);
        const dataView = new DataView(arrayBuffer, headerEnd);
        const isLittle = format === 'binary_little_endian';
        for (let i = 0; i < vertexCount; i++) {
            const base = i * vertexStride;
            const rx = readFloat(dataView, base + xOff, properties[xIdx].type, isLittle);
            const ry = readFloat(dataView, base + yOff, properties[yIdx].type, isLittle);
            const rz = readFloat(dataView, base + zOff, properties[zIdx].type, isLittle);
            if (!isFinite(rx) || !isFinite(ry) || !isFinite(rz)) continue;
            cx += rx; cy += ry; cz += rz;
            validCount++;
        }
    }

    if (validCount === 0) return { x: 0, y: 0, z: 0 };
    return { x: cx / validCount, y: cy / validCount, z: cz / validCount };
}

/**
 * Count points passing both distance AND opacity filters.
 * Uses transformed positions (same as collision system) and parsed opacities.
 */
export function countFilteredPoints(positions, opacities, vertexCount, cx, cy, cz, maxDist, minOpacity) {
    const maxDistSq = maxDist * maxDist;
    let count = 0;
    for (let i = 0; i < vertexCount; i++) {
        if (opacities && opacities[i] < minOpacity) continue;
        const dx = positions[i * 3] - cx;
        const dy = positions[i * 3 + 1] - cy;
        const dz = positions[i * 3 + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) count++;
    }
    return count;
}

/**
 * Filter PLY binary by both distance AND opacity.
 * Returns { filteredBuffer, keptCount }.
 */
export function filterPlyByCriteria(arrayBuffer, positions, opacities, vertexCount, cx, cy, cz, maxDist, minOpacity) {
    const maxDistSq = maxDist * maxDist;
    const keepIndices = [];
    for (let i = 0; i < vertexCount; i++) {
        if (opacities && opacities[i] < minOpacity) continue;
        const dx = positions[i * 3] - cx;
        const dy = positions[i * 3 + 1] - cy;
        const dz = positions[i * 3 + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) {
            keepIndices.push(i);
        }
    }

    if (keepIndices.length === vertexCount) {
        return { filteredBuffer: arrayBuffer, keptCount: vertexCount };
    }

    const bytes = new Uint8Array(arrayBuffer);
    const decoder = new TextDecoder('utf-8');

    let headerEnd = -1;
    for (let i = 0; i < Math.min(bytes.length, 65536); i++) {
        if (bytes[i] === 0x65 && bytes[i + 1] === 0x6e && bytes[i + 2] === 0x64 &&
            bytes[i + 3] === 0x5f && bytes[i + 4] === 0x68 && bytes[i + 5] === 0x65 &&
            bytes[i + 6] === 0x61 && bytes[i + 7] === 0x64 && bytes[i + 8] === 0x65 &&
            bytes[i + 9] === 0x72) {
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            headerEnd = j + 1;
            break;
        }
    }

    const headerText = decoder.decode(bytes.slice(0, headerEnd));
    const newHeader = headerText.replace(/element vertex \d+/, `element vertex ${keepIndices.length}`);

    const headerLines = headerText.split('\n').map(l => l.trim());
    const properties = [];
    let inV = false;
    for (const line of headerLines) {
        if (line.startsWith('element vertex')) inV = true;
        else if (line.startsWith('element') && inV) inV = false;
        else if (line.startsWith('property') && inV) {
            properties.push({ type: line.split(/\s+/)[1] });
        }
    }
    const vertexStride = properties.map(p => getPropertySize(p.type)).reduce((a, b) => a + b, 0);

    const headerBytes = new TextEncoder().encode(newHeader);
    const dataAfterVertices = bytes.slice(headerEnd + vertexCount * vertexStride);
    const newSize = headerBytes.length + keepIndices.length * vertexStride + dataAfterVertices.length;
    const newBuffer = new ArrayBuffer(newSize);
    const newBytes = new Uint8Array(newBuffer);

    newBytes.set(headerBytes, 0);
    let offset = headerBytes.length;
    for (const idx of keepIndices) {
        const srcStart = headerEnd + idx * vertexStride;
        newBytes.set(new Uint8Array(arrayBuffer, srcStart, vertexStride), offset);
        offset += vertexStride;
    }
    if (dataAfterVertices.length > 0) {
        newBytes.set(dataAfterVertices, offset);
    }

    return { filteredBuffer: newBuffer, keptCount: keepIndices.length };
}

/**
 * Analyze point cloud distances from centroid.
 * Returns { centroid: {x,y,z}, maxDistance: number }
 */
export function analyzePlyDistances(positions, vertexCount) {
    let cx = 0, cy = 0, cz = 0;
    let validCount = 0;
    for (let i = 0; i < vertexCount; i++) {
        const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        cx += x; cy += y; cz += z;
        validCount++;
    }
    if (validCount === 0) return { centroid: { x: 0, y: 0, z: 0 }, maxDistance: 1 };
    cx /= validCount; cy /= validCount; cz /= validCount;

    let maxDist = 0;
    for (let i = 0; i < vertexCount; i++) {
        const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        const dx = x - cx, dy = y - cy, dz = z - cz;
        const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (d > maxDist) maxDist = d;
    }

    return { centroid: { x: cx, y: cy, z: cz }, maxDistance: maxDist };
}

/**
 * Count how many points are within maxDistance from center.
 */
export function countPointsWithinDistance(positions, vertexCount, cx, cy, cz, maxDistance) {
    const maxDistSq = maxDistance * maxDistance;
    let count = 0;
    for (let i = 0; i < vertexCount; i++) {
        const dx = positions[i * 3] - cx;
        const dy = positions[i * 3 + 1] - cy;
        const dz = positions[i * 3 + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) count++;
    }
    return count;
}

/**
 * Filter a PLY binary ArrayBuffer, keeping only vertices within maxDistance
 * from the given center (computed on the transformed positions).
 * Returns { filteredBuffer: ArrayBuffer, keptCount: number }
 */
export function filterPlyByDistance(arrayBuffer, positions, vertexCount, cx, cy, cz, maxDistance) {
    const maxDistSq = maxDistance * maxDistance;
    const keepIndices = [];
    for (let i = 0; i < vertexCount; i++) {
        const dx = positions[i * 3] - cx;
        const dy = positions[i * 3 + 1] - cy;
        const dz = positions[i * 3 + 2] - cz;
        if (dx * dx + dy * dy + dz * dz <= maxDistSq) {
            keepIndices.push(i);
        }
    }

    if (keepIndices.length === vertexCount) {
        return { filteredBuffer: arrayBuffer, keptCount: vertexCount };
    }

    const bytes = new Uint8Array(arrayBuffer);
    const decoder = new TextDecoder('utf-8');

    // Find header end
    let headerEnd = -1;
    for (let i = 0; i < Math.min(bytes.length, 65536); i++) {
        if (bytes[i] === 0x65 && bytes[i + 1] === 0x6e && bytes[i + 2] === 0x64 &&
            bytes[i + 3] === 0x5f && bytes[i + 4] === 0x68 && bytes[i + 5] === 0x65 &&
            bytes[i + 6] === 0x61 && bytes[i + 7] === 0x64 && bytes[i + 8] === 0x65 &&
            bytes[i + 9] === 0x72) {
            let j = i + 10;
            while (j < bytes.length && bytes[j] !== 0x0a) j++;
            headerEnd = j + 1;
            break;
        }
    }

    const headerText = decoder.decode(bytes.slice(0, headerEnd));

    // Replace vertex count in header
    const newHeader = headerText.replace(
        /element vertex \d+/,
        `element vertex ${keepIndices.length}`
    );

    // Compute vertex stride from properties
    const headerLines = headerText.split('\n').map(l => l.trim());
    const properties = [];
    let inVertex = false;
    for (const line of headerLines) {
        if (line.startsWith('element vertex')) inVertex = true;
        else if (line.startsWith('element') && inVertex) inVertex = false;
        else if (line.startsWith('property') && inVertex) {
            properties.push({ type: line.split(/\s+/)[1] });
        }
    }
    const vertexStride = properties.map(p => getPropertySize(p.type)).reduce((a, b) => a + b, 0);

    // Build new buffer
    const headerBytes = new TextEncoder().encode(newHeader);
    const dataAfterVertices = bytes.slice(headerEnd + vertexCount * vertexStride);
    const newSize = headerBytes.length + keepIndices.length * vertexStride + dataAfterVertices.length;
    const newBuffer = new ArrayBuffer(newSize);
    const newBytes = new Uint8Array(newBuffer);

    // Copy header
    newBytes.set(headerBytes, 0);

    // Copy kept vertex rows
    let offset = headerBytes.length;
    for (const idx of keepIndices) {
        const srcStart = headerEnd + idx * vertexStride;
        newBytes.set(new Uint8Array(arrayBuffer, srcStart, vertexStride), offset);
        offset += vertexStride;
    }

    // Copy any data after vertices (other elements)
    if (dataAfterVertices.length > 0) {
        newBytes.set(dataAfterVertices, offset);
    }

    return { filteredBuffer: newBuffer, keptCount: keepIndices.length };
}

function getPropertySize(type) {
    switch (type) {
        case 'char': case 'int8': case 'uchar': case 'uint8': return 1;
        case 'short': case 'int16': case 'ushort': case 'uint16': return 2;
        case 'int': case 'int32': case 'uint': case 'uint32': case 'float': case 'float32': return 4;
        case 'double': case 'float64': return 8;
        default: return 4;
    }
}

function readFloat(dataView, offset, type, isLittle) {
    switch (type) {
        case 'float': case 'float32': return dataView.getFloat32(offset, isLittle);
        case 'double': case 'float64': return dataView.getFloat64(offset, isLittle);
        case 'int': case 'int32': return dataView.getInt32(offset, isLittle);
        case 'uint': case 'uint32': return dataView.getUint32(offset, isLittle);
        case 'short': case 'int16': return dataView.getInt16(offset, isLittle);
        case 'ushort': case 'uint16': return dataView.getUint16(offset, isLittle);
        case 'char': case 'int8': return dataView.getInt8(offset);
        case 'uchar': case 'uint8': return dataView.getUint8(offset);
        default: return dataView.getFloat32(offset, isLittle);
    }
}
