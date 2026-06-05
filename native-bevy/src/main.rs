/// MindCloud Fly — Bevy + web-splat hybrid renderer
///
/// Architecture:
/// - Bevy renders PBR 3D objects (gates, buildings, glTF models)
/// - web-splat renders Gaussian Splat scene as background (via SplatPlugin)
/// - Both share the same wgpu 27 Device

mod app_state;
mod drone;
mod flight;
mod gate_editor;
mod gate_plugin;
mod gates;
mod hud;
mod input;
mod input_plugin;
mod menu;
mod menu_ui;
mod persistence;
mod placement;
mod placement_ui;
mod scene;
mod settings_ui;
mod spline;
mod splat_plugin;

use bevy::prelude::*;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy_egui::EguiPlugin;
use clap::Parser;

use app_state::{AppState, GameMode, WorldUp};
use flight::FlightPlugin;
use gate_editor::GateEditorPlugin;
use gate_plugin::GatePlugin;
use input_plugin::InputPlugin;
use menu::MenuPlugin;
use placement::PlacementPlugin;
use scene::ScenePlugin;
use splat_plugin::SplatPlugin;

#[derive(Parser, Debug)]
#[command(name = "mindcloud-fly-bevy", about = "MindCloud Fly — Bevy + web-splat hybrid")]
struct Args {
    /// Path to a .ply gaussian splat file
    #[arg(short, long)]
    input: Option<String>,

    /// Disable vsync
    #[arg(long)]
    no_vsync: bool,

    /// Split-screen (two players). Default is single-player (one full-window view).
    #[arg(long)]
    split: bool,

    /// World up convention: `zup` (default) or `colmap` (Y-down). Overrides the per-scene config.
    #[arg(long, value_name = "zup|colmap")]
    world_up: Option<String>,
}

fn main() {
    let args = Args::parse();

    let mut app = App::new();

    let window_plugin = WindowPlugin {
        primary_window: Some(Window {
            title: "MindCloud Fly — Bevy + web-splat".into(),
            mode: bevy::window::WindowMode::BorderlessFullscreen(
                bevy::window::MonitorSelection::Current,
            ),
            present_mode: if args.no_vsync {
                bevy::window::PresentMode::AutoNoVsync
            } else {
                bevy::window::PresentMode::AutoVsync
            },
            resolution: bevy::window::WindowResolution::default()
                .with_scale_factor_override(1.0),
            ..default()
        }),
        ..default()
    };

    app.add_plugins(DefaultPlugins.set(window_plugin));
    app.add_plugins(EguiPlugin::default());
    app.add_plugins(SplatPlugin);
    app.add_plugins(FrameTimeDiagnosticsPlugin::default());

    // Black void color. The splat camera(s) (order 0/1) clear the shared window target; the UI
    // Camera2d (order 10, clear=None) only overlays egui on top.
    app.insert_resource(ClearColor(Color::BLACK));

    // Game mode + initial state. With `--input` we skip the menus and jump straight to Loading
    // (using the CLI `--split` mode); without it we start at the ModeSelect menu.
    let mode = if args.split { GameMode::SplitScreen } else { GameMode::SinglePlayer };
    let initial = if args.input.is_some() { AppState::Loading } else { AppState::ModeSelect };
    let world_up_override = args.world_up.as_deref().map(WorldUp::parse);
    app.insert_resource(mode);
    app.insert_resource(SceneInput { path: args.input, world_up_override });
    app.insert_state(initial);

    // Each phase is a plugin that registers its own resources + systems; main() just wires them:
    //  - MenuPlugin:      UI camera, mode/scene/loading screens, exit dialog, scene scan/cleanup.
    //  - ScenePlugin:     shared split-screen viewport layout + window title/FPS.
    //  - PlacementPlugin: scene build, world-up-aware orbit camera, spawn/heading editing overlay.
    //  - FlightPlugin:    the player drone (keyboard/HID → physics → camera).
    //  - InputPlugin:     HID RC-transmitter reader + Controller resource (drives P1 when connected).
    app.add_plugins((MenuPlugin, ScenePlugin, PlacementPlugin, FlightPlugin, GatePlugin, GateEditorPlugin, InputPlugin));

    app.run();
}

/// CLI-derived scene input: the `.ply` path to load and an optional `--world-up` override. Read by
/// the menu (load kickoff) and placement (config init) plugins.
#[derive(Resource)]
pub(crate) struct SceneInput {
    pub(crate) path: Option<String>,
    /// CLI `--world-up` override applied to the loaded per-scene config (None = use persisted value).
    pub(crate) world_up_override: Option<WorldUp>,
}
