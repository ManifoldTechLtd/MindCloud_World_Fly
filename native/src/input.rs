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
    pub fn apply(&self, raw: u16) -> f32 {
        match (self.min, self.center, self.max) {
            (Some(min), Some(center), Some(max)) => {
                let center = center as f32;
                let span = if raw as f32 >= center {
                    (max as f32 - center).max(1.0)
                } else {
                    (center - min as f32).max(1.0)
                };
                clamp((raw as f32 - center) / span, -1.0, 1.0)
            }
            _ => {
                // No calibration: assume 0-2047 range, center 1024
                clamp((raw as f32 - 1024.0) / 1024.0, -1.0, 1.0)
            }
        }
    }
}

// ---- Controller ----

pub struct Controller {
    // Axis mappings: roll, pitch, throttle, yaw, cameraTilt
    pub axis_map: [AxisMapping; 5],
    pub button_map_arm: ButtonMapping,
    pub button_map_mode: ButtonMapping,

    // HID state
    pub hid_axes: [f32; MAX_CHANNELS],
    hid_raw: [u16; MAX_CHANNELS],
    pub calibration: [ChannelCalibration; MAX_CHANNELS],
    pub hid_connected: bool,
    pub hid_device_name: String,

    // Keyboard state
    keys_down: std::collections::HashSet<winit::keyboard::KeyCode>,

    // Output
    pub armed: bool,
    pub boost: bool,
    prev_arm_button: bool,
    prev_mode_button: bool,

    // Mode switch output (edge-detected)
    pub mode_switch_triggered: bool,
    pub reset_triggered: bool,
}

impl Controller {
    pub fn new() -> Self {
        Self {
            axis_map: [
                AxisMapping { channel: 0, ..Default::default() },  // roll
                AxisMapping { channel: 1, ..Default::default() },  // pitch
                AxisMapping { channel: 2, ..Default::default() },  // throttle
                AxisMapping { channel: 3, ..Default::default() },  // yaw
                AxisMapping::default(),                             // cameraTilt
            ],
            button_map_arm: ButtonMapping::default(),
            button_map_mode: ButtonMapping::default(),

            hid_axes: [0.0; MAX_CHANNELS],
            hid_raw: [0; MAX_CHANNELS],
            calibration: std::array::from_fn(|_| ChannelCalibration::default()),
            hid_connected: false,
            hid_device_name: String::new(),

            keys_down: std::collections::HashSet::new(),

            armed: false,
            boost: false,
            prev_arm_button: false,
            prev_mode_button: false,

            mode_switch_triggered: false,
            reset_triggered: false,
        }
    }

    /// Feed raw HID report bytes (16-bit LE channels).
    pub fn feed_hid_report(&mut self, data: &[u8]) {
        let channel_count = (data.len() / 2).min(MAX_CHANNELS);
        for i in 0..channel_count {
            let raw = u16::from_le_bytes([data[i * 2], data[i * 2 + 1]]);
            self.hid_raw[i] = raw;
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

        // Start with zeros
        let mut roll = 0.0f32;
        let mut pitch = 0.0f32;
        let mut throttle = -1.0f32; // idle
        let mut yaw = 0.0f32;

        // ---- HID input ----
        if self.hid_connected {
            for (i, map) in self.axis_map.iter().enumerate() {
                if map.channel >= 0 && (map.channel as usize) < MAX_CHANNELS {
                    let mut val = self.hid_axes[map.channel as usize];
                    if map.inverted { val = -val; }
                    if val.abs() < map.deadzone { val = 0.0; }
                    val = apply_expo(val, map.expo);
                    val *= map.rate;
                    match i {
                        0 => roll = val,
                        1 => pitch = val,
                        2 => throttle = val,
                        3 => yaw = val,
                        _ => {}
                    }
                }
            }

            // Button: arm
            let arm_pressed = self.read_button(&self.button_map_arm);
            let arm_rising = arm_pressed && !self.prev_arm_button;
            self.prev_arm_button = arm_pressed;
            if arm_rising { self.armed = !self.armed; }

            // Button: mode switch
            let mode_pressed = self.read_button(&self.button_map_mode);
            let mode_rising = mode_pressed && !self.prev_mode_button;
            self.prev_mode_button = mode_pressed;
            if mode_rising { self.mode_switch_triggered = true; }
        }

        // ---- Keyboard overlay ----
        let kb = |k: KeyCode| self.keys_down.contains(&k);

        // Keyboard arm (Space) — edge toggle
        // (handled externally in main.rs for now)

        // Keyboard axes (additive with HID)
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
            rates: [
                self.axis_map[0].rate,
                self.axis_map[1].rate,
                self.axis_map[3].rate,
            ],
        }
    }

    fn read_button(&self, map: &ButtonMapping) -> bool {
        if map.channel < 0 || map.channel as usize >= MAX_CHANNELS { return false; }
        let mut v = self.hid_axes[map.channel as usize];
        if map.inverted { v = -v; }
        v > map.threshold
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
                Ok(n) => {
                    if tx.send(buf[..n].to_vec()).is_err() {
                        break; // receiver dropped
                    }
                }
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
