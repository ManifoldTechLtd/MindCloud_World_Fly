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
 * Drone physics v3 — quaternion-based orientation.
 *
 * All rotations are applied in the drone's BODY frame via quaternion multiplication.
 * This eliminates Euler-angle cross-coupling: roll is always around the drone's
 * nose-to-tail axis regardless of heading.
 *
 * Geometry (top view = square):
 *   - droneSize: width = depth (configurable)
 *   - CG at center
 *   - Camera at front edge (CG + local forward * droneSize/2)
 *   - Thrust along local +Y through CG
 *   - Forward = local -Z at identity orientation
 *
 * FPV:   sticks → body-frame angular rates,  throttle → thrust,  no self-leveling
 * Drone: sticks → velocity command → position setpoint,  cascaded PI position/velocity/tilt hold
 */

import { computeCollisionResponse } from './collision.js';

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const G = 9.81;              // gravitational acceleration (m/s²)
const AIR_DENSITY = 1.225;   // kg/m³ at sea level

// Reusable PlayCanvas math objects (avoid per-frame allocation)
const _quat  = new pc.Quat();
const _quat2 = new pc.Quat();
const _mat4  = new pc.Mat4();
const _v3    = new pc.Vec3();

export class Drone {
    constructor() {
        // ---- Geometry ----
        this.droneSize = 0.3;

        // ---- State ----
        this.x = 0; this.y = 2; this.z = 0;
        this.vx = 0; this.vy = 0; this.vz = 0;

        // Quaternion orientation (single source of truth)
        this.orientation = new pc.Quat();

        // Angular velocity in body frame (deg/s)
        this.pitchRate = 0;
        this.rollRate  = 0;
        this.yawRate   = 0;

        // Euler angles (derived from orientation each frame, for HUD/readout)
        this.pitch = 0;
        this.roll  = 0;
        this.yaw   = 0;

        // ---- Tunable parameters ----
        this.flightMode  = 'drone';
        this.mass        = 500;    // grams
        this.maxThrust   = 1000;   // grams-force
        this.dragCd      = 1.0;    // drag coefficient (dimensionless)
        this.dragArea     = 0.01;  // frontal area (m²)

        this.maxPitchRate = 220;
        this.maxRollRate  = 220;
        this.maxYawRate   = 120;

        this.droneMaxAngle   = 30;
        this.droneAngleRate  = 150;
        this.droneMaxVSpeed  = 3.0;
        this.droneMaxSpeed   = 5.0;

        // Cascaded PI gains
        this.dronePosKp  = 2.0;
        this.dronePosKi  = 0.3;
        this.droneVelKp  = 3.0;
        this.droneVelKi  = 1.0;
        this.droneAltKp  = 4.0;
        this.droneAltKi  = 2.0;

        // Position-hold setpoints
        this._targetX = 0; this._targetY = 2; this._targetZ = 0;
        this._targetYaw = 0;

        // Integral accumulators (position loop)
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        // Integral accumulators (velocity loop)
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        // Anti-windup limits
        this._posIntMax = 5.0;
        this._velIntMax = 15.0;

        this.angularDrag = 8.0;

        this.collisionRadius = 0.3;
        this.bounceDamping   = 0.3;

        // ---- Output state ----
        this.isColliding      = false;
        this.collisionIntensity = 0;
        this.speed            = 0;
        this.verticalSpeed    = 0;
        this.thrustOutput     = 0;

        // Camera mount angle (degrees, positive = tilted up)
        // FPV mode: fixed during flight, set via settings (0..60)
        // Drone mode: live tilt via input (-90..0)
        this.cameraMountAngle = 30; // FPV default
        this.cameraTiltAngle  = 0;  // Drone mode live tilt

        // Spawn
        this._spawnX = 0; this._spawnY = 2; this._spawnZ = 0;
    }

    // ---- Public API ----

    setSpawnPoint(x, y, z) {
        this._spawnX = x; this._spawnY = y; this._spawnZ = z;
        this.reset();
    }

    reset() {
        this.x = this._spawnX; this.y = this._spawnY; this.z = this._spawnZ;
        this.vx = 0; this.vy = 0; this.vz = 0;
        this.orientation.set(0, 0, 0, 1); // identity
        this.pitchRate = 0; this.rollRate = 0; this.yawRate = 0;
        this.pitch = 0; this.roll = 0; this.yaw = 0;
        this.isColliding = false;
        this.collisionIntensity = 0;
        this.thrustOutput = 0;
        this._targetX = this._spawnX; this._targetY = this._spawnY; this._targetZ = this._spawnZ;
        this._targetYaw = 0;
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
    }

    readSettings() {
        const el = (id) => document.getElementById(id);
        const v  = (id) => { const e = el(id); return e ? parseFloat(e.value) : null; };
        const massVal   = v('phys-mass');
        const thrustVal = v('phys-thrust');
        const cdVal     = v('phys-drag-cd');
        const areaVal   = v('phys-drag-area');
        const radiusVal = v('phys-collision-radius');
        const sizeVal   = v('phys-drone-size');
        const modeEl    = el('flight-mode-select');
        const posKp = v('ctrl-pos-kp');
        const posKi = v('ctrl-pos-ki');
        const velKp = v('ctrl-vel-kp');
        const velKi = v('ctrl-vel-ki');
        const altKp = v('ctrl-alt-kp');
        const altKi = v('ctrl-alt-ki');
        if (massVal !== null)   this.mass = massVal;
        if (thrustVal !== null) this.maxThrust = thrustVal;
        if (cdVal !== null)     this.dragCd = cdVal;
        if (areaVal !== null)   this.dragArea = areaVal;
        if (radiusVal !== null) this.collisionRadius = radiusVal;
        if (sizeVal !== null)   this.droneSize = sizeVal;
        if (modeEl) this.flightMode = modeEl.value;
        const mountAngle = v('cam-mount-angle');
        if (mountAngle !== null) this.cameraMountAngle = mountAngle;
        if (posKp !== null) this.dronePosKp = posKp;
        if (posKi !== null) this.dronePosKi = posKi;
        if (velKp !== null) this.droneVelKp = velKp;
        if (velKi !== null) this.droneVelKi = velKi;
        if (altKp !== null) this.droneAltKp = altKp;
        if (altKi !== null) this.droneAltKi = altKi;
    }

    update(dt, input, octree) {
        dt = Math.min(dt, 0.05);

        // 1. Control law → updates orientation quaternion and thrustOutput
        if (!input.armed) {
            this._updateDisarmed(dt);
        } else if (this.flightMode === 'drone') {
            this._controlDrone(dt, input);
        } else {
            this._controlFPV(dt, input);
        }

        // 2. Extract rotation matrix from orientation
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);

        // Local up = Y column of rotation matrix
        _mat4.getY(_v3);
        const upX = _v3.x, upY = _v3.y, upZ = _v3.z;

        // 3. Forces: thrust along local up + gravity + quadratic drag
        const massG = Math.max(this.mass, 1); // guard against zero mass
        const massKg = massG / 1000;
        // thrustOutput is in grams-force; convert to acceleration: (gf / g_mass) * G
        const thrustAccel = (this.thrustOutput / massG) * G;
        let ax = upX * thrustAccel;
        let ay = upY * thrustAccel - G;
        let az = upZ * thrustAccel;

        // Quadratic drag: F = 0.5 * Cd * A * rho * v^2, a = F / m
        const spd = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
        if (spd > 0.001) {
            const dragForce = 0.5 * this.dragCd * this.dragArea * AIR_DENSITY * spd * spd;
            const dragAccel = dragForce / massKg;
            ax -= (this.vx / spd) * dragAccel;
            ay -= (this.vy / spd) * dragAccel;
            az -= (this.vz / spd) * dragAccel;
        }

        // 4. Integrate velocity & position
        this.vx += ax * dt;
        this.vy += ay * dt;
        this.vz += az * dt;
        this.x += this.vx * dt;
        this.y += this.vy * dt;
        this.z += this.vz * dt;

        // NaN guard — reset if physics blew up
        if (isNaN(this.x) || isNaN(this.y) || isNaN(this.z)) {
            console.warn('NaN detected in drone state, resetting.',
                { mass: this.mass, thrust: this.thrustOutput, dragCd: this.dragCd, dragArea: this.dragArea });
            this.reset();
            return;
        }

        // 5. Collisions
        this._handleCollisions(octree);

        // 6. Derive euler angles for HUD
        this._updateEulerFromQuat();
        this.speed = Math.sqrt(this.vx * this.vx + this.vz * this.vz);
        this.verticalSpeed = this.vy;
    }

    getCameraTransform() {
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);

        // Local forward = -Z column
        _mat4.getZ(_v3);
        _v3.mulScalar(-1);
        const halfSize = this.droneSize * 0.5;

        // Camera mount pitch offset (body-frame X rotation)
        const mountDeg = this.flightMode === 'fpv' ? this.cameraMountAngle : this.cameraTiltAngle;
        const mountRad = mountDeg * DEG2RAD * 0.5;
        _quat.set(Math.sin(mountRad), 0, 0, Math.cos(mountRad));
        _quat2.copy(this.orientation).mul(_quat);

        // Extract euler angles from camera orientation (with mount offset)
        const euler = this._quatToEuler(_quat2);

        return {
            position: {
                x: this.x + _v3.x * halfSize,
                y: this.y + _v3.y * halfSize,
                z: this.z + _v3.z * halfSize
            },
            rotation: { x: euler.x, y: euler.y, z: euler.z }
        };
    }

    adjustCameraTilt(delta) {
        this.cameraTiltAngle = Math.max(-90, Math.min(0, this.cameraTiltAngle + delta));
    }

    // ---- Orientation helpers ----

    /**
     * Apply an incremental body-frame rotation.
     * bodyAxis: 'x' (pitch), 'y' (yaw), or 'z' (roll)
     * angleDeg: rotation in degrees
     *
     * Body-frame: orientation = orientation * deltaQuat
     * World-frame (yaw): orientation = deltaQuat * orientation
     */
    _applyBodyRotation(axisX, axisY, axisZ, angleDeg) {
        if (Math.abs(angleDeg) < 1e-8) return;
        const halfRad = (angleDeg * DEG2RAD) * 0.5;
        const s = Math.sin(halfRad);
        _quat.set(axisX * s, axisY * s, axisZ * s, Math.cos(halfRad));
        // Body frame: q_new = q_current * q_delta
        _quat2.copy(this.orientation).mul(_quat);
        this.orientation.copy(_quat2).normalize();
    }


    /**
     * Decompose orientation into yaw (world Y rotation) and body tilt.
     * Returns { yawDeg, bodyPitchDeg, bodyRollDeg }
     */
    _decomposeOrientation() {
        // Extract yaw from the local +Z column projected onto the XZ plane.
        // R_Y(yaw) maps (0,0,1) → (sinYaw, 0, cosYaw), so:
        //   sinYaw = localZ.x,  cosYaw = localZ.z
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3); // local +Z direction in world
        const yawRad = Math.atan2(_v3.x, _v3.z);
        const yawDeg = yawRad * RAD2DEG;

        // Build yaw-only quaternion
        const halfYaw = yawRad * 0.5;
        _quat.set(0, Math.sin(halfYaw), 0, Math.cos(halfYaw));

        // Body tilt = inverse(yawQuat) * orientation
        _quat2.copy(_quat).invert().mul(this.orientation);

        // Extract pitch and roll from the tilt quaternion
        // tiltQuat represents R_X(pitch) * R_Z(roll) approximately
        const tiltEuler = new pc.Vec3();
        _quat2.getEulerAngles(tiltEuler);

        return {
            yawDeg: yawDeg,
            bodyPitchDeg: tiltEuler.x,
            bodyRollDeg: tiltEuler.z
        };
    }

    _updateEulerFromQuat() {
        const e = new pc.Vec3();
        this.orientation.getEulerAngles(e);
        this.pitch = e.x;
        this.yaw   = e.y;
        this.roll  = e.z;

        // Yaw-independent body tilt for OSD artificial horizon
        const dec = this._decomposeOrientation();
        this.bodyPitch = dec.bodyPitchDeg;
        this.bodyRoll  = dec.bodyRollDeg;
    }

    _quatToEuler(q) {
        const e = new pc.Vec3();
        q.getEulerAngles(e);
        return { x: e.x, y: e.y, z: e.z };
    }

    // ---- Control laws ----

    _updateDisarmed(dt) {
        this.thrustOutput = 0;
        // Damp angular rates
        const damp = Math.exp(-this.angularDrag * dt);
        this.pitchRate *= damp;
        this.rollRate  *= damp;
        this.yawRate   *= damp;

        // Auto-level toward identity tilt (keep current yaw)
        const dec = this._decomposeOrientation();
        const levelSpeed = 60; // deg/s
        const pitchStep = Math.min(levelSpeed * dt, Math.abs(dec.bodyPitchDeg));
        const rollStep  = Math.min(levelSpeed * dt, Math.abs(dec.bodyRollDeg));

        if (pitchStep > 0.01) {
            this._applyBodyRotation(1, 0, 0, -Math.sign(dec.bodyPitchDeg) * pitchStep);
        }
        if (rollStep > 0.01) {
            this._applyBodyRotation(0, 0, 1, -Math.sign(dec.bodyRollDeg) * rollStep);
        }
    }

    _controlFPV(dt, input) {
        const boost = input.boost ? 1.5 : 1.0;

        // Sticks → target angular rates (body frame)
        const tPR = input.pitch * this.maxPitchRate * boost;
        const tRR = -input.roll * this.maxRollRate * boost;
        const tYR = input.yaw  * this.maxYawRate  * boost;

        // Smooth rate tracking
        const s = 1 - Math.exp(-15 * dt);
        this.pitchRate += (tPR - this.pitchRate) * s;
        this.rollRate  += (tRR - this.rollRate)  * s;
        this.yawRate   += (tYR - this.yawRate)   * s;

        // Damp when centered
        const ad = Math.exp(-this.angularDrag * dt);
        if (Math.abs(input.pitch) < 0.05) this.pitchRate *= ad;
        if (Math.abs(input.roll)  < 0.05) this.rollRate  *= ad;
        if (Math.abs(input.yaw)   < 0.05) this.yawRate   *= ad;

        // Apply body-frame rotations
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt); // pitch around body X
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);  // roll around body Z
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);      // yaw around body Y

        // Throttle → thrust (in grams-force)
        this.thrustOutput = ((input.throttle + 1) * 0.5) * this.maxThrust * boost;
    }

    _controlDrone(dt, input) {
        const boost = input.boost ? 1.5 : 1.0;
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

        // ---- 1. Update target setpoints from stick input ----
        // Get body-frame forward (-Z) and right (+X) in world XZ plane
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        const fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        const rightX = _v3.x, rightZ = _v3.z;

        // Pitch stick → forward velocity command, Roll stick → right velocity command
        const cmdFwd   = -input.pitch * this.droneMaxSpeed * boost; // pitch=-1 (up) → forward
        const cmdRight = input.roll * this.droneMaxSpeed * boost;   // roll=+1 (right) → right

        // Convert body-frame velocity command to world-frame and integrate into target position
        const cmdWorldX = cmdFwd * fwdX + cmdRight * rightX;
        const cmdWorldZ = cmdFwd * fwdZ + cmdRight * rightZ;
        this._targetX += cmdWorldX * dt;
        this._targetZ += cmdWorldZ * dt;

        // Throttle → target altitude
        this._targetY += input.throttle * this.droneMaxVSpeed * boost * dt;

        // Yaw stick → target yaw
        this._targetYaw += input.yaw * this.maxYawRate * boost * dt;

        // When sticks active, clear position integrals to prevent windup
        if (Math.abs(input.pitch) > 0.05 || Math.abs(input.roll) > 0.05) {
            this._posIntX = 0; this._posIntZ = 0;
            this._velIntX = 0; this._velIntZ = 0;
        }
        if (Math.abs(input.throttle) > 0.05) {
            this._posIntY = 0; this._velIntY = 0;
        }

        // ---- 2. Outer loop: Position PI → desired velocity (world frame) ----
        const posErrX = this._targetX - this.x;
        const posErrY = this._targetY - this.y;
        const posErrZ = this._targetZ - this.z;

        // Accumulate position integral (with anti-windup)
        const piMax = this._posIntMax;
        this._posIntX = clamp(this._posIntX + posErrX * dt, -piMax, piMax);
        this._posIntY = clamp(this._posIntY + posErrY * dt, -piMax, piMax);
        this._posIntZ = clamp(this._posIntZ + posErrZ * dt, -piMax, piMax);

        // Desired velocity = Kp * posErr + Ki * posIntegral
        const maxSpd = this.droneMaxSpeed * boost;
        let vDesX = this.dronePosKp * posErrX + this.dronePosKi * this._posIntX;
        let vDesY = this.droneAltKp * posErrY + this.droneAltKi * this._posIntY;
        let vDesZ = this.dronePosKp * posErrZ + this.dronePosKi * this._posIntZ;

        // Clamp desired horizontal velocity
        const vDesH = Math.sqrt(vDesX * vDesX + vDesZ * vDesZ);
        if (vDesH > maxSpd) {
            const s = maxSpd / vDesH;
            vDesX *= s; vDesZ *= s;
        }
        vDesY = clamp(vDesY, -this.droneMaxVSpeed * boost, this.droneMaxVSpeed * boost);

        // ---- 3. Inner loop: Velocity PI → desired tilt angles ----
        const velErrX = vDesX - this.vx;
        const velErrY = vDesY - this.vy;
        const velErrZ = vDesZ - this.vz;

        // Accumulate velocity integral (with anti-windup)
        const viMax = this._velIntMax;
        this._velIntX = clamp(this._velIntX + velErrX * dt, -viMax, viMax);
        this._velIntY = clamp(this._velIntY + velErrY * dt, -viMax, viMax);
        this._velIntZ = clamp(this._velIntZ + velErrZ * dt, -viMax, viMax);

        // Desired world-frame horizontal acceleration
        const aDesX = this.droneVelKp * velErrX + this.droneVelKi * this._velIntX;
        const aDesZ = this.droneVelKp * velErrZ + this.droneVelKi * this._velIntZ;

        // Project desired acceleration onto body forward/right to get tilt angles
        // Tilt angle ≈ atan(a_horizontal / g), small angle: angle ≈ a/g in radians
        const aFwd   = aDesX * fwdX + aDesZ * fwdZ;
        const aRight = aDesX * rightX + aDesZ * rightZ;

        const maxAngle = this.droneMaxAngle;
        // Forward accel → negative pitch (nose down), right accel → positive roll
        const targetPitch = clamp(-aFwd / G * RAD2DEG, -maxAngle, maxAngle);
        const targetRoll  = clamp(-aRight / G * RAD2DEG, -maxAngle, maxAngle);

        // ---- 4. Attitude P-controller: tilt error → body rotation ----
        const dec = this._decomposeOrientation();
        const pitchErr = targetPitch - dec.bodyPitchDeg;
        const rollErr  = targetRoll  - dec.bodyRollDeg;

        const maxStep = this.droneAngleRate * dt;
        const dpitch = clamp(pitchErr, -maxStep, maxStep);
        const droll  = clamp(rollErr,  -maxStep, maxStep);

        this._applyBodyRotation(1, 0, 0, dpitch);
        this._applyBodyRotation(0, 0, 1, droll);

        this.pitchRate = pitchErr * 5;
        this.rollRate  = rollErr  * 5;

        // ---- 5. Yaw hold ----
        const yawErr = this._targetYaw - dec.yawDeg;
        // Normalize yaw error to [-180, 180]
        const yawErrNorm = ((yawErr + 180) % 360 + 360) % 360 - 180;
        const tYR = clamp(yawErrNorm * 2.0, -this.maxYawRate, this.maxYawRate);
        const ys = 1 - Math.exp(-15 * dt);
        this.yawRate += (tYR - this.yawRate) * ys;
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);

        // ---- 6. Altitude PI → thrust (in grams-force) ----
        const aDesY = this.droneVelKp * velErrY + this.droneVelKi * this._velIntY;
        // Desired upward acceleration → required thrust in gf
        // thrustAccel = (thrustOutput / mass) * G, so thrustOutput = mass * (G + aDesY) / G
        let cmdGf = this.mass * (G + aDesY) / G;

        // Tilt compensation
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getY(_v3);
        const cosT = Math.max(0.1, _v3.y);
        cmdGf /= cosT;

        this.thrustOutput = clamp(cmdGf, 0, this.maxThrust * boost);
    }

    // ---- Collision ----

    _handleCollisions(octree) {
        this.isColliding = false;
        this.collisionIntensity = 0;

        if (octree) {
            const results = octree.querySphere(this.x, this.y, this.z, this.collisionRadius);
            const collision = computeCollisionResponse(
                { x: this.x, y: this.y, z: this.z },
                this.collisionRadius,
                results
            );

            if (collision && collision.penetration > 0) {
                this.isColliding = true;
                this.collisionIntensity = Math.min(1, collision.penetration / this.collisionRadius);

                const pushDist = collision.penetration + 0.01;
                this.x += collision.normal.x * pushDist;
                this.y += collision.normal.y * pushDist;
                this.z += collision.normal.z * pushDist;

                const vDotN = this.vx * collision.normal.x +
                              this.vy * collision.normal.y +
                              this.vz * collision.normal.z;
                if (vDotN < 0) {
                    this.vx -= collision.normal.x * vDotN * (1 + this.bounceDamping);
                    this.vy -= collision.normal.y * vDotN * (1 + this.bounceDamping);
                    this.vz -= collision.normal.z * vDotN * (1 + this.bounceDamping);
                }

                this.vx *= 0.8;
                this.vy *= 0.8;
                this.vz *= 0.8;
            }
        }

    }
}
