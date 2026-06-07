//! Bevy wiring for the HID RC-transmitter controller (`input::Controller`).
//!
//! - [`ControllerRes`] holds the per-player `Controller`s (P1 = index 0, P2 = index 1).
//! - [`HidConnections`] holds each player's open connection (device path + the background reader's
//!   `Receiver`, in a `Mutex` so the resource is `Sync`); a slot is `Some` only while connected.
//! - [`hid_poll_system`] drains each reader every frame into its controller and detects disconnects.
//! - [`reconcile_hid`] reconnects each active slot's last-used device once the game mode is chosen.
//!
//! `flight::drone_input_system` reads `ControllerRes` to drive each player when its device is on.

use bevy::prelude::*;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::Mutex;

use crate::app_state::{AppState, GameMode};
use crate::input::{list_rc_devices, open_hid_device, Controller};
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
        // Reconnect last-used transmitters once the game mode is known (not at Startup).
        app.add_systems(OnEnter(AppState::Playing), reconcile_hid);
        app.add_systems(Update, hid_poll_system);
    }
}

/// Open `path`, start its reader thread, persist it as the last-used device, and insert the
/// [`HidConnection`] resource (replacing any existing one). Used by auto-connect + the settings UI.
///
/// `active_players` = how many players are live in the current mode (1 = single, 2 = split). The
/// same transmitter may not drive two LIVE players (their inputs would be identical), but a stale
/// connection on an INACTIVE slot is released so the device can be rebound to the requested player.
pub fn connect_hid(
    conns: &mut HidConnections,
    ctrl: &mut Controller,
    player: usize,
    active_players: usize,
    path: std::ffi::CString,
    product_name: String,
) -> Result<(), String> {
    // Refuse to bind one physical device to two ACTIVE players: both reader threads would receive
    // the same reports, making their inputs identical. But a leftover connection on an INACTIVE slot
    // (e.g. P2's auto-connected transmitter while in single-player) must NOT block rebinding — free
    // it so the device is available to the requested player.
    if let Some(other) = conns.0.iter().position(|c| c.as_ref().is_some_and(|c| c.path == path)) {
        if other != player {
            if other < active_players {
                return Err(format!(
                    "device already bound to P{} — use a separate transmitter for P{}",
                    other + 1,
                    player + 1
                ));
            }
            conns.0[other] = None; // release the inactive slot's hold so we can rebind here
        }
    }
    let rx = open_hid_device(&path)?;
    // Bind this slot's controller to the device + load ITS saved config (mapping/calibration) by
    // name, so the transmitter behaves the same in any player slot / game mode.
    ctrl.set_device(&product_name);
    info!("[HID] P{} connected: {} ({:?})", player + 1, product_name, path);
    conns.0[player] = Some(HidConn { path, product_name, rx: Mutex::new(rx) });
    Ok(())
}

/// Explicit user disconnect: drop the connection (stops the reader) + unbind the controller. The
/// caller clears the per-slot last-used name so it does not auto-reconnect.
pub fn disconnect_hid(conns: &mut HidConnections, ctrl: &mut Controller, player: usize) {
    ctrl.clear_device();
    conns.0[player] = None;
    info!("[HID] P{} disconnected by user", player + 1);
}

/// Logical slot key for the last-used-transmitter store, by game mode + player index.
pub fn slot_key(mode: GameMode, player: usize) -> &'static str {
    match (mode, player) {
        (GameMode::SinglePlayer, _) => "single",
        (GameMode::SplitScreen, 0) => "dual_p1",
        (GameMode::SplitScreen, _) => "dual_p2",
    }
}

/// `OnEnter(Playing)`: reconnect each ACTIVE slot's last-used transmitter (matched by product name,
/// so it survives replugging), loading that device's saved config. Slots unused by this mode (P2 in
/// single-player) are released so their transmitter is free to be rebound. Runs after the game mode
/// is chosen, so single-player and split-screen each restore their own remembered controllers.
fn reconcile_hid(
    mode: Res<GameMode>,
    mut conns: ResMut<HidConnections>,
    mut ctrl: ResMut<ControllerRes>,
) {
    let active = match *mode {
        GameMode::SinglePlayer => 1,
        GameMode::SplitScreen => 2,
    };
    let devices = list_rc_devices();
    for slot in 0..active {
        let key = slot_key(*mode, slot);
        let Some(name) = persistence::load_last_device(key) else {
            continue;
        };
        // Already on the right transmitter? leave the live connection alone.
        if conns.0[slot].as_ref().is_some_and(|c| c.product_name == name) {
            continue;
        }
        let Some(dev) = devices.iter().find(|d| d.product_name == name) else {
            info!("[HID] last-used '{}' for {} not present", name, key);
            continue;
        };
        let (path, pname) = (dev.path.clone(), dev.product_name.clone());
        if let Err(e) = connect_hid(&mut conns, &mut ctrl.0[slot], slot, active, path, pname) {
            warn!("[HID] reconnect '{}' for {} failed: {}", name, key, e);
        }
    }
    // Free any slot this mode does not use, so its transmitter can be rebound elsewhere.
    for slot in active..NUM_PLAYERS {
        if conns.0[slot].is_some() {
            conns.0[slot] = None;
            ctrl.0[slot].clear_device();
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
