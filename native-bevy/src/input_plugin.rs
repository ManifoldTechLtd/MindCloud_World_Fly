//! Bevy wiring for the HID RC-transmitter controller (`input::Controller`).
//!
//! - [`ControllerRes`] holds the `Controller` (mapping/calibration/live channels).
//! - [`HidConnection`] holds the device path + the background reader's `Receiver` (in a `Mutex` so
//!   the resource is `Sync`). It exists only while a device is connected.
//! - [`hid_poll_system`] drains the reader each frame into the controller and detects disconnects.
//! - [`autoconnect_hid`] reopens the last-used device on startup (path shared with `native/`).
//!
//! `flight::drone_input_system` reads `ControllerRes` to drive player 1 when a device is connected.

use bevy::prelude::*;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::Mutex;

use crate::input::{list_hid_devices, open_hid_device, Controller};
use crate::persistence;

/// The HID controller state (axis/switch mapping, calibration, live channel values).
#[derive(Resource)]
pub struct ControllerRes(pub Controller);

/// An open HID connection. Present only while a device is connected; removed on disconnect.
#[derive(Resource)]
pub struct HidConnection {
    pub path: std::ffi::CString,
    pub product_name: String,
    /// `Mutex` makes the `!Sync` `Receiver` usable as a Bevy resource.
    rx: Mutex<Receiver<Vec<u8>>>,
}

/// Last HID device scan (populated by the settings-panel "Scan" button; for the device picker).
#[derive(Resource, Default)]
pub struct HidDevices(pub Vec<crate::input::HidDeviceInfo>);

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ControllerRes(Controller::new()));
        app.init_resource::<HidDevices>();
        app.add_systems(Startup, autoconnect_hid);
        app.add_systems(Update, hid_poll_system);
    }
}

/// Open `path`, start its reader thread, persist it as the last-used device, and insert the
/// [`HidConnection`] resource (replacing any existing one). Used by auto-connect + the settings UI.
pub fn connect_hid(
    commands: &mut Commands,
    path: std::ffi::CString,
    product_name: String,
) -> Result<(), String> {
    let rx = open_hid_device(&path)?;
    let _ = persistence::save_hid_device_path(&path);
    info!("[HID] connected: {} ({:?})", product_name, path);
    commands.insert_resource(HidConnection { path, product_name, rx: Mutex::new(rx) });
    Ok(())
}

/// Explicit user disconnect: forget the saved device + drop the connection (stops the reader).
pub fn disconnect_hid(commands: &mut Commands, ctrl: &mut Controller) {
    persistence::clear_hid_device_path();
    ctrl.hid_connected = false;
    commands.remove_resource::<HidConnection>();
    info!("[HID] disconnected by user");
}

/// `Startup`: reopen the last-used HID device (path written by `native/` or a prior session), but
/// only if it is still present AND is not a pointing device. A mouse/keyboard saved by mistake (or
/// by a stale path) would otherwise feed its reports as RC channels and hijack the controls — so we
/// skip it and clear the bad save. An absent device (transmitter currently unplugged) keeps its
/// saved path so it reconnects next launch.
fn autoconnect_hid(mut commands: Commands) {
    let Some(path) = persistence::load_hid_device_path() else {
        return;
    };
    let Some(dev) = list_hid_devices().into_iter().find(|d| d.path == path) else {
        return; // device not present right now — keep the saved path for next time.
    };
    if dev.is_pointer_like() {
        warn!(
            "[HID] saved device {:?} is a pointing device ({}); ignoring + clearing it",
            path, dev.product_name
        );
        persistence::clear_hid_device_path();
        return;
    }
    if let Err(e) = connect_hid(&mut commands, path.clone(), dev.product_name) {
        warn!("[HID] auto-connect failed for {:?}: {}", path, e);
    }
}

/// `Update` (always): drain queued HID reports into the controller; drop the connection if the
/// reader thread has exited. Also services channel-assignment "listen" mode for the settings UI.
fn hid_poll_system(
    conn: Option<Res<HidConnection>>,
    mut ctrl: ResMut<ControllerRes>,
    mut commands: Commands,
) {
    let Some(conn) = conn else {
        if ctrl.0.hid_connected {
            ctrl.0.hid_connected = false;
        }
        return;
    };

    let mut disconnected = false;
    if let Ok(rx) = conn.rx.lock() {
        loop {
            match rx.try_recv() {
                Ok(report) => ctrl.0.feed_hid_report(&report),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    disconnected = true;
                    break;
                }
            }
        }
    }

    if disconnected {
        warn!("[HID] device {:?} disconnected (reader thread exited)", conn.path);
        ctrl.0.hid_connected = false;
        // Keep the saved path so the device auto-reconnects next launch (only an explicit
        // user disconnect clears it).
        commands.remove_resource::<HidConnection>();
    } else {
        // Settings UI "listen for channel" assignment.
        ctrl.0.poll_listen();
    }
}
