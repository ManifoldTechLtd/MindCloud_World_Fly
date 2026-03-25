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
 * HUD overlay — updates HTML elements with drone telemetry, FPS, controller status.
 */

export class HUD {
    constructor() {
        this.altitudeEl = document.getElementById('hud-altitude');
        this.vspeedEl = document.getElementById('hud-vspeed');
        this.gspeedEl = document.getElementById('hud-gspeed');
        this.fpsEl = document.getElementById('hud-fps');
        this.controllerEl = document.getElementById('hud-controller');
        this.collisionWarnEl = document.getElementById('hud-collision-warn');
        this.armedEl = document.getElementById('armed-indicator');
        this.collisionFlashEl = document.getElementById('collision-flash');
        this.hudContainer = document.getElementById('hud');

        // FPS tracking
        this._frameTimes = [];
        this._lastTime = performance.now();
        this._collisionFlashTimer = 0;
    }

    show() {
        this.hudContainer.classList.remove('hidden');
    }

    hide() {
        this.hudContainer.classList.add('hidden');
    }

    /**
     * Update HUD each frame.
     * @param {Drone} drone
     * @param {Controller} controller
     */
    update(drone, controller) {
        const now = performance.now();
        const dt = now - this._lastTime;
        this._lastTime = now;

        // FPS calculation (rolling average)
        this._frameTimes.push(dt);
        if (this._frameTimes.length > 60) this._frameTimes.shift();
        const avgDt = this._frameTimes.reduce((a, b) => a + b, 0) / this._frameTimes.length;
        const fps = Math.round(1000 / avgDt);

        // Update values
        if (this.altitudeEl) this.altitudeEl.textContent = drone.y.toFixed(1);
        if (this.vspeedEl) {
            this.vspeedEl.textContent = drone.verticalSpeed.toFixed(1);
            this.vspeedEl.style.color = drone.verticalSpeed < -2 ? '#f80' : '#0f0';
        }
        if (this.gspeedEl) this.gspeedEl.textContent = drone.speed.toFixed(1);
        if (this.fpsEl) this.fpsEl.textContent = fps;

        // Controller status
        if (this.controllerEl) {
            if (controller.connected) {
                this.controllerEl.textContent = 'RC Connected';
                this.controllerEl.style.color = '#4272F5';
            } else {
                this.controllerEl.textContent = 'Keyboard';
                this.controllerEl.style.color = '#0f0';
            }
        }

        // Armed indicator
        if (this.armedEl) {
            if (controller.armed) {
                this.armedEl.textContent = 'ARMED';
                this.armedEl.className = 'armed';
            } else {
                this.armedEl.textContent = 'DISARMED';
                this.armedEl.className = 'disarmed';
            }
        }

        // Collision warning
        if (drone.isColliding) {
            this._collisionFlashTimer = 150; // ms
            if (this.collisionWarnEl) this.collisionWarnEl.style.display = 'block';
        } else {
            if (this.collisionWarnEl) this.collisionWarnEl.style.display = 'none';
        }

        // Collision flash effect
        if (this._collisionFlashTimer > 0) {
            this._collisionFlashTimer -= dt;
            if (this.collisionFlashEl) {
                this.collisionFlashEl.classList.add('active');
                this.collisionFlashEl.style.opacity = Math.min(1, drone.collisionIntensity * 0.5);
            }
        } else {
            if (this.collisionFlashEl) {
                this.collisionFlashEl.classList.remove('active');
            }
        }
    }
}
