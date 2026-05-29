/// Controller input system — HID RC transmitter + keyboard.
/// Ported from src/controller.js (core input logic only, no DOM/UI).

use crate::drone::DroneInput;

const MAX_CHANNELS: usize = 16;

fn clamp(v: f32, lo: f32, hi: f32) -> f32 { v.max(lo).min(hi) }

/// Apply expo curve: blends linear with cubic for smoother center feel.
/// expo=0 → pure linear, expo=1 → pure cubic.
fn apply_expo(val: f32, expo: f32) -> f32 {
    let linear = val;
    let cubic = val * val * val;
    linear * (1.0 - expo) + cubic * expo
}

// ---- Axis mapping ----

#[derive(Clone, Debug)]
pub struct AxisMapping {
    pub channel: i32,    // HID channel index, -1 = unassigned
    pub inverted: bool,
    pub deadzone: f32,
    pub rate: f32,
    pub expo: f32,
}

impl Default for AxisMapping {
    fn default() -> Self {
        Self { channel: -1, inverted: false, deadzone: 0.0, rate: 1.0, expo: 0.0 }
    }
}

// ---- Button mapping ----

#[derive(Clone, Debug)]
pub struct ButtonMapping {
    pub channel: i32,
    pub inverted: bool,
    pub threshold: f32,
    /// 'toggle' = rising edge flips, 'level' = follows switch position
    pub trigger_mode: TriggerMode,
}

#[derive(Clone, Debug, PartialEq)]
pub enum TriggerMode {
    Toggle,
    Level,
}

impl Default for ButtonMapping {
    fn default() -> Self {
        Self { channel: -1, inverted: false, threshold: 0.5, trigger_mode: TriggerMode::Toggle }
    }
}

// ---- Calibration ----

#[derive(Clone, Debug)]
pub struct ChannelCalibration {
    pub min: Option<u16>,
    pub center: Option<u16>,
    pub max: Option<u16>,
}

impl Default for ChannelCalibration {
    fn default() -> Self {
        Self { min: None, center: None, max: None }
    }
}

impl ChannelCalibration {
    /// Apply calibration: center = (min+max)/2, output -1..1
    pub fn apply(&self, raw: u16) -> f32 {
        match (self.min, self.max) {
            (Some(min), Some(max)) if max > min => {
                // Center is midpoint (same as JS version)
                let center = (min as f32 + max as f32) / 2.0;
                let span = if raw as f32 >= center {
                    (max as f32 - center).max(1.0)
                } else {
                    (center - min as f32).max(1.0)
                };
                clamp((raw as f32 - center) / span, -1.0, 1.0)
            }
            _ => {
                // No calibration: pass through raw as 0-2047 centered at 1024
                clamp((raw as f32 - 1024.0) / 820.0, -1.0, 1.0)
            }
        }
    }

    pub fn is_calibrated(&self) -> bool {
        self.min.is_some() && self.max.is_some()
    }
}

// ---- Controller ----

/// Axis names for display
pub const AXIS_NAMES: [&str; 4] = ["Roll", "Pitch", "Throttle", "Yaw"];
/// Switch names for display
pub const SWITCH_NAMES: [&str; 2] = ["Arm", "Mode"];

pub struct Controller {
    /// Axis mappings: roll, pitch, throttle, yaw (indices 0-3)
    pub axis_map: [AxisMapping; 4],
    /// Per-mode expo and rate (index: 0=FPV, 1=Drone)
    pub mode_expo: [[f32; 4]; 2],  // [fpv][roll,pitch,thr,yaw], [drone][...]
    pub mode_rate: [[f32; 4]; 2],  // same
    /// Switch channel mappings: arm (0), mode (1). Channel -1 = unassigned.
    pub switch_channels: [i32; 2],
    pub switch_inverted: [bool; 2],
    pub switch_threshold: f32,
    /// Trigger mode: false = toggle (momentary/self-reset switch), true = level (2-pos switch)
    pub switch_level_mode: [bool; 2],

    // HID state
    pub hid_axes: [f32; MAX_CHANNELS],
    pub hid_raw: [u16; MAX_CHANNELS],
    pub calibration: [ChannelCalibration; MAX_CHANNELS],
    pub hid_connected: bool,

    // Keyboard state
    keys_down: std::collections::HashSet<winit::keyboard::KeyCode>,

    // Output
    pub armed: bool,
    pub boost: bool,
    prev_arm_state: bool,
    prev_mode_state: bool,

    pub mode_switch_triggered: bool,
    pub reset_triggered: bool,

    /// Calibration in progress
    pub calibrating: bool,
    /// Current flight mode index (0=FPV, 1=Drone) — set externally by main.rs
    pub current_mode: usize,

    /// Channel listen mode: Some(axis_or_switch_index) means waiting for user to move a stick
    /// axis 0-3 = axis mapping, 10/11 = switch mapping
    pub listening: Option<usize>,
    listen_baseline: [f32; MAX_CHANNELS],
}

impl Controller {
    pub fn new() -> Self {
        let mut calibration: [ChannelCalibration; MAX_CHANNELS] = std::array::from_fn(|_| ChannelCalibration::default());
        crate::persistence::load_calibration(&mut calibration);

        let mut ctrl = Self {
            axis_map: [
                AxisMapping { channel: 0, ..Default::default() },  // roll
                AxisMapping { channel: 1, ..Default::default() },  // pitch
                AxisMapping { channel: 2, ..Default::default() },  // throttle
                AxisMapping { channel: 3, ..Default::default() },  // yaw
            ],
            mode_expo: [[0.0; 4]; 2],  // [FPV, Drone] × [roll, pitch, thr, yaw]
            mode_rate: [[1.0; 4]; 2],  // default rate = 1.0
            switch_channels: [-1, -1],
            switch_inverted: [false, false],
            switch_threshold: 0.5,
            switch_level_mode: [false, false],

            hid_axes: [0.0; MAX_CHANNELS],
            hid_raw: [0; MAX_CHANNELS],
            calibration,
            hid_connected: false,

            keys_down: std::collections::HashSet::new(),

            armed: false,
            boost: false,
            prev_arm_state: false,
            prev_mode_state: false,

            mode_switch_triggered: false,
            reset_triggered: false,
            calibrating: false,
            current_mode: 0,
            listening: None,
            listen_baseline: [0.0; MAX_CHANNELS],
        };
        crate::persistence::load_controller_mapping(&mut ctrl);
        ctrl
    }

    /// Start listening for channel assignment.
    pub fn start_listen(&mut self, target: usize) {
        self.listen_baseline = self.hid_axes;
        self.listening = Some(target);
    }

    /// Check if a channel moved significantly from baseline → assign it.
    pub fn poll_listen(&mut self) -> bool {
        if let Some(target) = self.listening {
            for i in 0..MAX_CHANNELS {
                let delta = (self.hid_axes[i] - self.listen_baseline[i]).abs();
                if delta > 0.5 {
                    // Found the channel that moved
                    if target < 4 {
                        self.axis_map[target].channel = i as i32;
                    } else if target == 10 {
                        self.switch_channels[0] = i as i32;
                    } else if target == 11 {
                        self.switch_channels[1] = i as i32;
                    }
                    self.listening = None;
                    let _ = crate::persistence::save_controller_mapping(self);
                    return true;
                }
            }
        }
        false
    }

    /// Feed raw HID report bytes (16-bit LE channels).
    pub fn feed_hid_report(&mut self, data: &[u8]) {
        let channel_count = (data.len() / 2).min(MAX_CHANNELS);
        for i in 0..channel_count {
            let raw = u16::from_le_bytes([data[i * 2], data[i * 2 + 1]]);
            self.hid_raw[i] = raw;
            // Auto-learn min/max only during calibration
            if self.calibrating {
                let cal = &mut self.calibration[i];
                match cal.min {
                    None => cal.min = Some(raw),
                    Some(m) if raw < m => cal.min = Some(raw),
                    _ => {}
                }
                match cal.max {
                    None => cal.max = Some(raw),
                    Some(m) if raw > m => cal.max = Some(raw),
                    _ => {}
                }
            }
            self.hid_axes[i] = self.calibration[i].apply(raw);
        }
        self.hid_connected = true;
    }

    /// Process keyboard event.
    pub fn key_event(&mut self, key: winit::keyboard::KeyCode, pressed: bool) {
        if pressed {
            self.keys_down.insert(key);
        } else {
            self.keys_down.remove(&key);
        }
    }

    /// Call once per frame. Returns a DroneInput.
    pub fn update(&mut self) -> DroneInput {
        use winit::keyboard::KeyCode;

        self.mode_switch_triggered = false;
        self.reset_triggered = false;

        // Poll channel listen mode
        self.poll_listen();

        let mut roll = 0.0f32;
        let mut pitch = 0.0f32;
        let mut throttle = -1.0f32;
        let mut yaw = 0.0f32;

        // ---- HID axes ----
        if self.hid_connected {
            let axes = [&mut roll, &mut pitch, &mut throttle, &mut yaw];
            for (i, val_out) in axes.into_iter().enumerate() {
                let map = &self.axis_map[i];
                if map.channel >= 0 && (map.channel as usize) < MAX_CHANNELS {
                    let mut val = self.hid_axes[map.channel as usize];
                    if map.inverted { val = -val; }
                    if val.abs() < map.deadzone { val = 0.0; }
                    let m = self.current_mode.min(1);
                    val = apply_expo(val, self.mode_expo[m][i]);
                    val *= self.mode_rate[m][i];
                    *val_out = val;
                }
            }

            // Switch: arm
            if self.switch_channels[0] >= 0 {
                let ch = self.switch_channels[0] as usize;
                if ch < MAX_CHANNELS {
                    let mut v = self.hid_axes[ch];
                    if self.switch_inverted[0] { v = -v; }
                    let state = v > self.switch_threshold;
                    if self.switch_level_mode[0] {
                        // Level mode (2-pos switch): armed follows switch position on transitions
                        if state != self.prev_arm_state { self.armed = state; }
                    } else {
                        // Toggle mode (momentary): rising edge flips
                        if state && !self.prev_arm_state { self.armed = !self.armed; }
                    }
                    self.prev_arm_state = state;
                }
            }

            // Switch: mode
            if self.switch_channels[1] >= 0 {
                let ch = self.switch_channels[1] as usize;
                if ch < MAX_CHANNELS {
                    let mut v = self.hid_axes[ch];
                    if self.switch_inverted[1] { v = -v; }
                    let state = v > self.switch_threshold;
                    if self.switch_level_mode[1] {
                        // Level mode: trigger on transitions only
                        if state != self.prev_mode_state { self.mode_switch_triggered = true; }
                    } else {
                        // Toggle mode: rising edge
                        if state && !self.prev_mode_state { self.mode_switch_triggered = true; }
                    }
                    self.prev_mode_state = state;
                }
            }
        }

        // ---- Keyboard overlay ----
        let kb = |k: KeyCode| self.keys_down.contains(&k);
        if kb(KeyCode::ArrowRight) { roll += 1.0; }
        if kb(KeyCode::ArrowLeft) { roll -= 1.0; }
        if kb(KeyCode::ArrowUp) { pitch += 1.0; }
        if kb(KeyCode::ArrowDown) { pitch -= 1.0; }
        if kb(KeyCode::KeyW) { throttle = 0.5; }
        if kb(KeyCode::KeyS) { throttle = -1.0; }
        if kb(KeyCode::KeyD) { yaw += 1.0; }
        if kb(KeyCode::KeyA) { yaw -= 1.0; }
        self.boost = kb(KeyCode::ShiftLeft) || kb(KeyCode::ShiftRight);
        if kb(KeyCode::KeyR) { self.reset_triggered = true; }

        DroneInput {
            roll: clamp(roll, -1.0, 1.0),
            pitch: clamp(pitch, -1.0, 1.0),
            throttle: clamp(throttle, -1.0, 1.0),
            yaw: clamp(yaw, -1.0, 1.0),
            armed: self.armed,
            boost: self.boost,
            rates: [self.axis_map[0].rate, self.axis_map[1].rate, self.axis_map[3].rate],
        }
    }
}

// ---- HID device management ----

/// Info about an available HID device.
#[derive(Debug, Clone)]
pub struct HidDeviceInfo {
    pub path: std::ffi::CString,
    pub product_name: String,
    pub vendor_id: u16,
    pub product_id: u16,
}

/// List all available HID devices.
pub fn list_hid_devices() -> Vec<HidDeviceInfo> {
    let api = match hidapi::HidApi::new() {
        Ok(a) => a,
        Err(e) => {
            log::warn!("Failed to init hidapi: {}", e);
            return Vec::new();
        }
    };
    api.device_list()
        .map(|d| HidDeviceInfo {
            path: d.path().to_owned(),
            product_name: d.product_string().unwrap_or("Unknown").to_string(),
            vendor_id: d.vendor_id(),
            product_id: d.product_id(),
        })
        .collect()
}

/// Open a HID device by path and start reading reports in a background thread.
/// Returns a receiver that yields raw report byte vectors.
pub fn open_hid_device(
    path: &std::ffi::CStr,
) -> Result<std::sync::mpsc::Receiver<Vec<u8>>, String> {
    let api = hidapi::HidApi::new().map_err(|e| format!("hidapi init: {}", e))?;
    let device = api.open_path(path).map_err(|e| format!("open_path: {}", e))?;
    device.set_blocking_mode(true).map_err(|e| format!("set_blocking: {}", e))?;

    let (tx, rx) = std::sync::mpsc::channel();

    std::thread::spawn(move || {
        let mut buf = [0u8; 64];
        loop {
            match device.read_timeout(&mut buf, 50) {
                Ok(0) => continue, // timeout, no data
                Ok(n) if n > 1 => {
                    // Skip first byte (report ID), send channel data only
                    if tx.send(buf[1..n].to_vec()).is_err() {
                        break; // receiver dropped
                    }
                }
                Ok(_) => continue,
                Err(e) => {
                    log::error!("HID read error: {}", e);
                    break;
                }
            }
        }
        log::info!("HID reader thread exited");
    });

    Ok(rx)
}
