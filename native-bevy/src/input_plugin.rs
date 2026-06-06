//! Bevy wiring for the HID RC-transmitter controller (`input::Controller`).
//!
//! - [`ControllerRes`] holds the per-player `Controller`s (P1 = index 0, P2 = index 1).
//! - [`HidConnections`] holds each player's open connection (device path + the background reader's
//!   `Receiver`, in a `Mutex` so the resource is `Sync`); a slot is `Some` only while connected.
//! - [`hid_poll_system`] drains each reader every frame into its controller and detects disconnects.
//! - [`autoconnect_hid`] reopens each player's last-used device on startup.
//!
//! `flight::drone_input_system` reads `ControllerRes` to drive each player when its device is on.

use bevy::prelude::*;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::Mutex;

use crate::input::{list_hid_devices, open_hid_device, Controller};
use crate::persistence;

/// Number of players that can each bind an independent HID transmitter.
pub const NUM_PLAYERS: usize = 2;

/// Per-player HID controller state (index 0 = P1, 1 = P2): axis/switch mapping, calibration, live
/// channels.
#[derive(Resource)]
pub struct ControllerRes(pub [Controller; NUM_PLAYERS]);

/// One open HID connection: device path + the background reader's `Receiver`.
pub struct HidConn {
    pub path: std::ffi::CString,
    pub product_name: String,
    /// `Mutex` makes the `!Sync` `Receiver` usable inside a Bevy resource.
    rx: Mutex<Receiver<Vec<u8>>>,
}

/// Per-player HID connections; slot `p` is `Some` only while player `p` has a device connected.
#[derive(Resource)]
pub struct HidConnections(pub [Option<HidConn>; NUM_PLAYERS]);

impl Default for HidConnections {
    fn default() -> Self {
        Self([None, None])
    }
}

/// Last HID device scan (populated by the settings-panel "Scan" button; shared device picker).
#[derive(Resource, Default)]
pub struct HidDevices(pub Vec<crate::input::HidDeviceInfo>);

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ControllerRes(std::array::from_fn(Controller::new)));
        app.init_resource::<HidConnections>();
        app.init_resource::<HidDevices>();
        app.add_systems(Startup, autoconnect_hid);
        app.add_systems(Update, hid_poll_system);
    }
}

/// Open `path`, start its reader thread, persist it as the last-used device, and insert the
/// [`HidConnection`] resource (replacing any existing one). Used by auto-connect + the settings UI.
pub fn connect_hid(
    conns: &mut HidConnections,
    player: usize,
    path: std::ffi::CString,
    product_name: String,
) -> Result<(), String> {
    // Refuse to bind one physical device to two players: both reader threads would receive the same
    // reports, making P1 and P2 inputs identical. Each player must use a separate transmitter.
    if let Some(other) = conns.0.iter().position(|c| c.as_ref().is_some_and(|c| c.path == path)) {
        if other != player {
            return Err(format!(
                "device already bound to P{} — use a separate transmitter for P{}",
                other + 1,
                player + 1
            ));
        }
    }
    let rx = open_hid_device(&path)?;
    let _ = persistence::save_hid_device_path(player, &path);
    info!("[HID] P{} connected: {} ({:?})", player + 1, product_name, path);
    conns.0[player] = Some(HidConn { path, product_name, rx: Mutex::new(rx) });
    Ok(())
}

/// Explicit user disconnect: forget the saved device + drop the connection (stops the reader).
pub fn disconnect_hid(conns: &mut HidConnections, ctrl: &mut Controller, player: usize) {
    persistence::clear_hid_device_path(player);
    ctrl.hid_connected = false;
    conns.0[player] = None;
    info!("[HID] P{} disconnected by user", player + 1);
}

/// `Startup`: reopen the last-used HID device (path written by `native/` or a prior session), but
/// only if it is still present AND is not a pointing device. A mouse/keyboard saved by mistake (or
/// by a stale path) would otherwise feed its reports as RC channels and hijack the controls — so we
/// skip it and clear the bad save. An absent device (transmitter currently unplugged) keeps its
/// saved path so it reconnects next launch.
fn autoconnect_hid(mut conns: ResMut<HidConnections>) {
    for player in 0..NUM_PLAYERS {
        let Some(path) = persistence::load_hid_device_path(player) else {
            continue;
        };
        let Some(dev) = list_hid_devices().into_iter().find(|d| d.path == path) else {
            continue; // device not present right now — keep the saved path for next time.
        };
        if dev.is_pointer_like() {
            warn!(
                "[HID] saved P{} device {:?} is a pointing device ({}); ignoring + clearing it",
                player + 1, path, dev.product_name
            );
            persistence::clear_hid_device_path(player);
            continue;
        }
        if let Err(e) = connect_hid(&mut conns, player, path.clone(), dev.product_name) {
            warn!("[HID] P{} auto-connect failed for {:?}: {}", player + 1, path, e);
        }
    }
}

/// `Update` (always): drain each player's queued HID reports into its controller; drop a connection
/// whose reader thread has exited (keeping the saved path for auto-reconnect). Also services the
/// settings-UI channel-assignment "listen" mode per controller.
fn hid_poll_system(mut conns: ResMut<HidConnections>, mut ctrl: ResMut<ControllerRes>) {
    for player in 0..NUM_PLAYERS {
        let mut has_conn = false;
        let mut disconnected = false;
        if let Some(conn) = &conns.0[player] {
            has_conn = true;
            if let Ok(rx) = conn.rx.lock() {
                loop {
                    match rx.try_recv() {
                        Ok(report) => ctrl.0[player].feed_hid_report(&report),
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => {
                            disconnected = true;
                            break;
                        }
                    }
                }
            }
        }
        if !has_conn {
            if ctrl.0[player].hid_connected {
                ctrl.0[player].hid_connected = false;
            }
            continue;
        }
        if disconnected {
            let path = conns.0[player].as_ref().map(|c| c.path.clone());
            warn!("[HID] P{} device {:?} disconnected (reader thread exited)", player + 1, path);
            ctrl.0[player].hid_connected = false;
            conns.0[player] = None; // keep saved path for auto-reconnect next launch
        } else {
            ctrl.0[player].poll_listen();
        }
    }
}
