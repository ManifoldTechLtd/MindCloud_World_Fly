//! HID RC-transmitter controller — ported from `native/src/input.rs` (HID core only).
//!
//! Keyboard is handled by Bevy in `flight.rs`; this module owns HID channel reading, calibration,
//! axis/switch mapping, and turns the live stick values into a [`DroneInput`]. The Bevy wiring
//! (resources, reader thread, per-frame poll) lives in `input_plugin.rs`.

use crate::drone::DroneInput;
use bevy::log::{error, info, warn};

pub const MAX_CHANNELS: usize = 16;

fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    v.max(lo).min(hi)
}

/// Expo curve: `expo=0` → linear, `expo=1` → cubic; blends for a softer stick center.
fn apply_expo(val: f32, expo: f32) -> f32 {
    let cubic = val * val * val;
    val * (1.0 - expo) + cubic * expo
}

// ---- Axis mapping ----

#[derive(Clone, Debug)]
pub struct AxisMapping {
    /// HID channel index, `-1` = unassigned.
    pub channel: i32,
    pub inverted: bool,
    pub deadzone: f32,
    pub rate: f32,
}

impl Default for AxisMapping {
    fn default() -> Self {
        Self { channel: -1, inverted: false, deadzone: 0.0, rate: 1.0 }
    }
}

// ---- Calibration ----

#[derive(Clone, Debug, Default)]
pub struct ChannelCalibration {
    pub min: Option<u16>,
    pub center: Option<u16>,
    pub max: Option<u16>,
}

impl ChannelCalibration {
    /// Map a raw channel value to `-1..1`, centered at `(min+max)/2`. Uncalibrated channels assume
    /// the common 0..2047-centered-at-1024 range.
    pub fn apply(&self, raw: u16) -> f32 {
        match (self.min, self.max) {
            (Some(min), Some(max)) if max > min => {
                let center = (min as f32 + max as f32) / 2.0;
                let span = if raw as f32 >= center {
                    (max as f32 - center).max(1.0)
                } else {
                    (center - min as f32).max(1.0)
                };
                clamp((raw as f32 - center) / span, -1.0, 1.0)
            }
            _ => clamp((raw as f32 - 1024.0) / 820.0, -1.0, 1.0),
        }
    }

    pub fn is_calibrated(&self) -> bool {
        self.min.is_some() && self.max.is_some()
    }
}

// ---- Controller ----

/// Axis names for display (indices 0-3).
pub const AXIS_NAMES: [&str; 4] = ["Roll", "Pitch", "Throttle", "Yaw"];
/// Switch names for display (indices 0-1).
pub const SWITCH_NAMES: [&str; 2] = ["Arm", "Mode"];

/// Listen-target sentinels for [`Controller::start_listen`] (switches share the axis target space).
pub const LISTEN_ARM: usize = 10;
pub const LISTEN_MODE: usize = 11;

pub struct Controller {
    /// Axis mappings: roll, pitch, throttle, yaw (indices 0-3).
    pub axis_map: [AxisMapping; 4],
    /// Per-mode expo, index `0=FPV, 1=Drone`, then `[roll, pitch, throttle, yaw]`.
    pub mode_expo: [[f32; 4]; 2],
    /// Per-mode rate (same layout as `mode_expo`).
    pub mode_rate: [[f32; 4]; 2],
    /// Switch channels: arm (0), mode (1). `-1` = unassigned.
    pub switch_channels: [i32; 2],
    pub switch_inverted: [bool; 2],
    pub switch_threshold: f32,
    /// `false` = toggle (rising edge flips), `true` = level (follows 2-pos switch).
    pub switch_level_mode: [bool; 2],

    // HID state
    pub hid_axes: [f32; MAX_CHANNELS],
    pub hid_raw: [u16; MAX_CHANNELS],
    /// Number of channels in the most recent HID report (so the UI can list every live channel).
    pub hid_channel_count: usize,
    pub calibration: [ChannelCalibration; MAX_CHANNELS],
    pub hid_connected: bool,

    // Output state
    pub armed: bool,
    pub boost: bool,
    prev_arm_state: bool,
    prev_mode_state: bool,
    pub mode_switch_triggered: bool,
    pub reset_triggered: bool,

    /// Auto-learn min/max while true.
    pub calibrating: bool,
    /// Current flight mode index (`0=FPV, 1=Drone`) — set by the caller before `poll_input`.
    pub current_mode: usize,
    /// Which player this controller drives (0 = P1, 1 = P2). Selects the per-player persistence files.
    pub player: usize,

    /// Channel-listen mode: `Some(target)` where target `0-3` = axis, `LISTEN_ARM`/`LISTEN_MODE` = switch.
    pub listening: Option<usize>,
    listen_baseline: [f32; MAX_CHANNELS],
}

impl Controller {
    pub fn new(player: usize) -> Self {
        let mut calibration: [ChannelCalibration; MAX_CHANNELS] =
            std::array::from_fn(|_| ChannelCalibration::default());
        crate::persistence::load_calibration(player, &mut calibration);

        let mut ctrl = Self {
            axis_map: [
                AxisMapping { channel: 0, ..Default::default() }, // roll
                AxisMapping { channel: 1, ..Default::default() }, // pitch
                AxisMapping { channel: 2, ..Default::default() }, // throttle
                AxisMapping { channel: 3, ..Default::default() }, // yaw
            ],
            mode_expo: [[0.0; 4]; 2],
            mode_rate: [[1.0; 4]; 2],
            switch_channels: [-1, -1],
            switch_inverted: [false, false],
            switch_threshold: 0.5,
            switch_level_mode: [false, false],

            hid_axes: [0.0; MAX_CHANNELS],
            hid_raw: [0; MAX_CHANNELS],
            hid_channel_count: 0,
            calibration,
            hid_connected: false,

            armed: false,
            boost: false,
            prev_arm_state: false,
            prev_mode_state: false,
            mode_switch_triggered: false,
            reset_triggered: false,

            calibrating: false,
            current_mode: 0,
            player,
            listening: None,
            listen_baseline: [0.0; MAX_CHANNELS],
        };
        crate::persistence::load_controller_mapping(&mut ctrl);
        ctrl
    }

    /// Begin listening for a channel assignment: the next channel to move > 0.5 from its current
    /// value gets assigned to `target` (`0-3` axis, `LISTEN_ARM`/`LISTEN_MODE` switch).
    pub fn start_listen(&mut self, target: usize) {
        self.listen_baseline = self.hid_axes;
        self.listening = Some(target);
    }

    /// Poll listen mode: assign + persist the moved channel. Returns true when an assignment lands.
    pub fn poll_listen(&mut self) -> bool {
        if let Some(target) = self.listening {
            for i in 0..MAX_CHANNELS {
                if (self.hid_axes[i] - self.listen_baseline[i]).abs() > 0.5 {
                    if target < 4 {
                        self.axis_map[target].channel = i as i32;
                    } else if target == LISTEN_ARM {
                        self.switch_channels[0] = i as i32;
                    } else if target == LISTEN_MODE {
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

    /// Feed one raw HID report (16-bit LE channels; report-ID byte already stripped).
    pub fn feed_hid_report(&mut self, data: &[u8]) {
        let channel_count = (data.len() / 2).min(MAX_CHANNELS);
        self.hid_channel_count = channel_count;
        for i in 0..channel_count {
            let raw = u16::from_le_bytes([data[i * 2], data[i * 2 + 1]]);
            self.hid_raw[i] = raw;
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

    /// Wipe every channel's learned min/center/max so a fresh calibration starts from scratch
    /// (called when the user presses *Start Calibration*).
    pub fn reset_calibration(&mut self) {
        for c in self.calibration.iter_mut() {
            *c = ChannelCalibration::default();
        }
    }

    /// Per-frame HID input. Updates `armed` (via the arm switch) and sets the mode/reset edge flags.
    /// The caller sets `current_mode` and `armed` (to sync external arm state) before calling, and
    /// reads `mode_switch_triggered` / `reset_triggered` after. Keyboard is overlaid in `flight.rs`.
    pub fn poll_input(&mut self) -> DroneInput {
        self.mode_switch_triggered = false;
        self.reset_triggered = false;

        let mut roll = 0.0f32;
        let mut pitch = 0.0f32;
        let mut throttle = -1.0f32;
        let mut yaw = 0.0f32;

        if self.hid_connected {
            let outs = [&mut roll, &mut pitch, &mut throttle, &mut yaw];
            for (i, out) in outs.into_iter().enumerate() {
                let map = &self.axis_map[i];
                if map.channel >= 0 && (map.channel as usize) < MAX_CHANNELS {
                    let mut v = self.hid_axes[map.channel as usize];
                    if map.inverted {
                        v = -v;
                    }
                    if v.abs() < map.deadzone {
                        v = 0.0;
                    }
                    let m = self.current_mode.min(1);
                    v = apply_expo(v, self.mode_expo[m][i]);
                    v *= self.mode_rate[m][i];
                    *out = v;
                }
            }

            // Arm switch
            if self.switch_channels[0] >= 0 {
                let ch = self.switch_channels[0] as usize;
                if ch < MAX_CHANNELS {
                    let mut v = self.hid_axes[ch];
                    if self.switch_inverted[0] {
                        v = -v;
                    }
                    let state = v > self.switch_threshold;
                    if self.switch_level_mode[0] {
                        if state != self.prev_arm_state {
                            self.armed = state;
                        }
                    } else if state && !self.prev_arm_state {
                        self.armed = !self.armed;
                    }
                    self.prev_arm_state = state;
                }
            }

            // Mode switch
            if self.switch_channels[1] >= 0 {
                let ch = self.switch_channels[1] as usize;
                if ch < MAX_CHANNELS {
                    let mut v = self.hid_axes[ch];
                    if self.switch_inverted[1] {
                        v = -v;
                    }
                    let state = v > self.switch_threshold;
                    if self.switch_level_mode[1] {
                        if state != self.prev_mode_state {
                            self.mode_switch_triggered = true;
                        }
                    } else if state && !self.prev_mode_state {
                        self.mode_switch_triggered = true;
                    }
                    self.prev_mode_state = state;
                }
            }
        }

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

/// Info about an available HID device (for the settings UI device picker).
#[derive(Debug, Clone)]
pub struct HidDeviceInfo {
    pub path: std::ffi::CString,
    pub product_name: String,
    pub vendor_id: u16,
    pub product_id: u16,
    /// Top-level HID usage page / usage (USB HID usage tables) — used to reject pointing/typing
    /// devices so the RC controller never binds to a mouse or keyboard.
    pub usage_page: u16,
    pub usage: u16,
}

impl HidDeviceInfo {
    /// True for mice / keyboards / keypads (Generic Desktop page 0x01, usages Pointer 0x01,
    /// Mouse 0x02, Keyboard 0x06, Keypad 0x07). RC transmitters enumerate as Joystick (0x04) or
    /// Gamepad (0x05), or on a vendor-defined page, so they are NOT rejected by this test.
    pub fn is_pointer_like(&self) -> bool {
        self.usage_page == 0x01 && matches!(self.usage, 0x01 | 0x02 | 0x06 | 0x07)
    }
}

/// List all available HID devices.
pub fn list_hid_devices() -> Vec<HidDeviceInfo> {
    let api = match hidapi::HidApi::new() {
        Ok(a) => a,
        Err(e) => {
            warn!("[HID] failed to init hidapi: {}", e);
            return Vec::new();
        }
    };
    api.device_list()
        .map(|d| HidDeviceInfo {
            path: d.path().to_owned(),
            product_name: d.product_string().unwrap_or("Unknown").to_string(),
            vendor_id: d.vendor_id(),
            product_id: d.product_id(),
            usage_page: d.usage_page(),
            usage: d.usage(),
        })
        .collect()
}

/// HID devices suitable as an RC transmitter (mice / keyboards filtered out). Used by the settings
/// device picker and auto-connect so a pointing device is never selected.
pub fn list_rc_devices() -> Vec<HidDeviceInfo> {
    list_hid_devices()
        .into_iter()
        .filter(|d| !d.is_pointer_like())
        .collect()
}

/// Open a HID device by path and start a background reader thread.
/// Returns an `mpsc::Receiver` yielding raw report byte vectors (report-ID byte stripped).
pub fn open_hid_device(
    path: &std::ffi::CStr,
) -> Result<std::sync::mpsc::Receiver<Vec<u8>>, String> {
    let api = hidapi::HidApi::new().map_err(|e| format!("hidapi init: {}", e))?;
    let device = api.open_path(path).map_err(|e| format!("open_path: {}", e))?;
    device
        .set_blocking_mode(true)
        .map_err(|e| format!("set_blocking: {}", e))?;

    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let mut buf = [0u8; 64];
        loop {
            match device.read_timeout(&mut buf, 50) {
                Ok(0) => continue, // timeout, no data
                Ok(n) if n > 1 => {
                    // Skip the first byte (report ID); send channel bytes only.
                    if tx.send(buf[1..n].to_vec()).is_err() {
                        break; // receiver dropped
                    }
                }
                Ok(_) => continue,
                Err(e) => {
                    error!("[HID] read error: {}", e);
                    break;
                }
            }
        }
        info!("[HID] reader thread exited");
    });

    Ok(rx)
}
