/// MindCloud Fly — Bevy + web-splat hybrid renderer
///
/// Architecture:
/// - Bevy renders PBR 3D objects (gates, buildings, glTF models)
/// - web-splat renders Gaussian Splat scene as background (via SplatPlugin)
/// - Both share the same wgpu 27 Device

mod app_state;
mod menu_ui;
mod persistence;
mod placement_ui;
mod splat_plugin;

use bevy::prelude::*;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::camera::{ClearColorConfig, Viewport};
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use clap::Parser;
use std::f32::consts::FRAC_PI_2;
use std::path::{Path, PathBuf};

use app_state::{AppState, CurrentSceneConfig, GameMode, SceneConfig, WorldUp};
use menu_ui::{ExitAction, ModeAction, SceneAction};
use placement_ui::{PlacementAction, PlacementUiState};
use splat_plugin::{SplatPlugin, SplatCamera};

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
    app.add_plugins(PanOrbitCameraPlugin);
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
    app.insert_resource(MenuState::default());
    app.insert_state(initial);
    app.init_resource::<PlacementWantsText>();

    app.add_systems(Startup, setup_ui_camera);
    // Menu flow: egui draws in the EguiPrimaryContextPass schedule, gated by state.
    app.add_systems(
        EguiPrimaryContextPass,
        (
            mode_select_system.run_if(in_state(AppState::ModeSelect)),
            scene_select_system.run_if(in_state(AppState::SceneSelect)),
            loading_screen_system.run_if(in_state(AppState::Loading)),
            placement_overlay_system.run_if(in_state(AppState::Placement)),
            exit_dialog_system,
        ),
    );
    app.add_systems(OnEnter(AppState::ModeSelect), cleanup_scene);
    app.add_systems(OnEnter(AppState::SceneSelect), scan_scenes);
    app.add_systems(OnEnter(AppState::Loading), start_loading);
    app.add_systems(Update, advance_when_loaded.run_if(in_state(AppState::Loading)));
    // init_scene_config loads/persists the per-scene config before the scene is built.
    app.add_systems(OnEnter(AppState::Placement), (init_scene_config, setup_scene).chain());
    app.add_systems(Update, (update_window_title, set_split_viewports, handle_esc));
    app.add_systems(Update, placement_update.run_if(in_state(AppState::Placement)));
    app.run();
}

#[derive(Resource)]
struct SceneInput {
    path: Option<String>,
    /// CLI `--world-up` override applied to the loaded per-scene config (None = use persisted value).
    world_up_override: Option<WorldUp>,
}

/// Marks the UI/menu camera (Camera2d) that hosts egui — persists across all states.
#[derive(Component)]
struct UiCamera;

/// Marks entities spawned for the active scene (cameras, meshes, lights) so they can be despawned
/// when returning to the menu.
#[derive(Component)]
struct SceneEntity;

/// Marks the placement spawn marker (bright sphere + heading arrow) so `placement_update` can keep
/// it on the configured spawn point + heading.
#[derive(Component)]
struct SpawnMarker;

/// Set by the placement overlay each frame: true while egui has keyboard focus (a text field is
/// being edited), so `placement_update` suppresses WASD spawn movement.
#[derive(Resource, Default)]
struct PlacementWantsText(bool);

/// Shared state for the menu systems.
#[derive(Resource, Default)]
struct MenuState {
    scene_files: Vec<PathBuf>,
    selected: Option<usize>,
    show_exit: bool,
}

/// Startup: spawn the 2D UI camera that hosts egui. bevy_egui attaches the primary context to the
/// FIRST camera, and menus render before any 3D camera exists, so this must exist up front. Order
/// 10 + clear=None overlays egui on top of the splat cameras (order 0/1) without erasing them.
fn setup_ui_camera(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Camera { order: 10, clear_color: ClearColorConfig::None, ..default() },
        // Msaa::Off is REQUIRED: the splat Camera3d is single-sample and writes the window target
        // directly. A multisampled UI camera would render to a 4x buffer and RESOLVE it (black,
        // since egui draws nothing in-game) over the splat every frame. Single-sample + clear=None
        // makes this camera LOAD the existing target and only overlay egui on top.
        Msaa::Off,
        UiCamera,
    ));
}

/// `OnEnter(SceneSelect)`: scan for scene files.
fn scan_scenes(mut menu: ResMut<MenuState>) {
    menu.scene_files = menu_ui::scan_scene_files();
    menu.selected = None;
    info!("[SceneSelect] found {} scene file(s)", menu.scene_files.len());
}

/// `OnEnter(ModeSelect)`: despawn any scene entities (when returning to the menu from a game).
fn cleanup_scene(mut commands: Commands, q: Query<Entity, With<SceneEntity>>) {
    let n = q.iter().count();
    for e in &q {
        commands.entity(e).despawn();
    }
    if n > 0 {
        info!("[ModeSelect] cleaned up {} scene entit(ies)", n);
    }
}

/// ModeSelect menu.
fn mode_select_system(
    mut contexts: EguiContexts,
    mut mode: ResMut<GameMode>,
    mut next: ResMut<NextState<AppState>>,
) -> Result {
    let Ok(ctx) = contexts.ctx_mut() else { return Ok(()); };
    if let ModeAction::Select(m) = menu_ui::draw_mode_select(ctx) {
        *mode = m;
        info!("[ModeSelect] mode = {:?} -> SceneSelect", m);
        next.set(AppState::SceneSelect);
    }
    Ok(())
}

/// SceneSelect menu.
fn scene_select_system(
    mut contexts: EguiContexts,
    mut menu: ResMut<MenuState>,
    mode: Res<GameMode>,
    mut scene_input: ResMut<SceneInput>,
    mut next: ResMut<NextState<AppState>>,
) -> Result {
    let Ok(ctx) = contexts.ctx_mut() else { return Ok(()); };
    let mut sel = menu.selected;
    let action = menu_ui::draw_scene_select(ctx, &menu.scene_files, *mode, &mut sel);
    menu.selected = sel;
    match action {
        SceneAction::Load(path) => {
            info!("[SceneSelect] load {} -> Loading", path.display());
            scene_input.path = Some(path.to_string_lossy().into_owned());
            next.set(AppState::Loading);
        }
        SceneAction::Back => next.set(AppState::ModeSelect),
        SceneAction::None => {}
    }
    Ok(())
}

/// Loading screen.
fn loading_screen_system(mut contexts: EguiContexts, scene_input: Res<SceneInput>) -> Result {
    let Ok(ctx) = contexts.ctx_mut() else { return Ok(()); };
    let name = scene_input
        .path
        .as_deref()
        .and_then(|p| Path::new(p).file_name())
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "scene".to_string());
    menu_ui::draw_loading_screen(ctx, &name);
    Ok(())
}

/// Placement overlay: edit spawn / heading / world-up, then Start Flight (Enter) or Back (Esc).
fn placement_overlay_system(
    mut contexts: EguiContexts,
    mut config: ResMut<CurrentSceneConfig>,
    mut ui_state: ResMut<PlacementUiState>,
    mut wants_text: ResMut<PlacementWantsText>,
    scene_input: Res<SceneInput>,
    menu: Res<MenuState>,
    keys: Res<ButtonInput<KeyCode>>,
    mut next: ResMut<NextState<AppState>>,
) -> Result {
    let Ok(ctx) = contexts.ctx_mut() else { return Ok(()); };
    let wants_kb = ctx.wants_keyboard_input();
    wants_text.0 = wants_kb;
    // Generous bounds for now; scene-AABB clamping ports with the spawn-marker/drone slice.
    let b = 1.0e4_f32;
    let mut action =
        placement_ui::draw_overlay(ctx, &mut config.0, &mut ui_state, [-b, -b, -b], [b, b, b]);
    // Enter = Start Flight (unless a text field is focused). Esc is handled by `handle_esc`.
    if matches!(action, PlacementAction::None) && !wants_kb && keys.just_pressed(KeyCode::Enter) {
        action = PlacementAction::StartFlight;
    }
    // Ignore overlay interaction while the exit dialog is up (it draws on top).
    if menu.show_exit {
        return Ok(());
    }
    match action {
        PlacementAction::StartFlight => {
            if let Some(p) = &scene_input.path {
                if let Err(e) =
                    persistence::save_scene_config(&persistence::scene_key(Path::new(p)), &config.0)
                {
                    warn!("[Placement] failed to save scene config: {e}");
                }
            }
            info!("[Placement] Start Flight -> Playing");
            next.set(AppState::Playing);
        }
        PlacementAction::GoBack => {
            info!("[Placement] Back -> ModeSelect");
            next.set(AppState::ModeSelect);
        }
        PlacementAction::None => {}
    }
    Ok(())
}

/// `Update` while in `Placement`: WASD/QE move the spawn point (camera-relative, on the world-up
/// plane; Shift = faster), and the spawn marker + orbit-camera focus are kept on the spawn so any
/// edit (text fields or WASD) is immediately visible.
fn placement_update(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    wants_text: Res<PlacementWantsText>,
    menu: Res<MenuState>,
    mut config: ResMut<CurrentSceneConfig>,
    mut cams: Query<(&mut PanOrbitCamera, &Transform), (With<SplatCamera>, Without<SpawnMarker>)>,
    mut markers: Query<&mut Transform, (With<SpawnMarker>, Without<SplatCamera>)>,
) {
    // "Up" axis on which the spawn moves horizontally (and lifts with E/Q).
    let world_up = match config.0.world_up {
        WorldUp::Zup => Vec3::Z,
        WorldUp::Colmap => Vec3::NEG_Y, // gravity is +Y, so up is -Y
    };
    // Movement basis from the first camera's facing, projected onto the world-up plane.
    let basis = cams.iter().next().map(|(_, t)| {
        let proj = |v: Vec3| (v - world_up * v.dot(world_up)).normalize_or_zero();
        (proj(t.rotation * Vec3::NEG_Z), proj(t.rotation * Vec3::X))
    });
    if !wants_text.0 && !menu.show_exit {
        if let Some((fwd, right)) = basis {
            let mut speed = 8.0 * time.delta_secs();
            if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
                speed *= 5.0;
            }
            let mut delta = Vec3::ZERO;
            if keys.pressed(KeyCode::KeyW) { delta += fwd; }
            if keys.pressed(KeyCode::KeyS) { delta -= fwd; }
            if keys.pressed(KeyCode::KeyD) { delta += right; }
            if keys.pressed(KeyCode::KeyA) { delta -= right; }
            if keys.pressed(KeyCode::KeyE) { delta += world_up; }
            if keys.pressed(KeyCode::KeyQ) { delta -= world_up; }
            if delta != Vec3::ZERO {
                let s = Vec3::from(config.0.spawn) + delta * speed;
                config.0.spawn = [s.x, s.y, s.z];
            }
        }
    }
    // Keep the marker (position + heading) and the orbit focus on the spawn point.
    let spawn = Vec3::from(config.0.spawn);
    let h = config.0.heading_deg.to_radians();
    let fwd_dir = match config.0.world_up {
        WorldUp::Zup => Vec3::new(-h.sin(), h.cos(), 0.0),
        WorldUp::Colmap => Vec3::new(h.cos(), 0.0, h.sin()),
    };
    let rot = Quat::from_rotation_arc(Vec3::Y, fwd_dir);
    for mut t in &mut markers {
        t.translation = spawn;
        t.rotation = rot;
    }
    for (mut pc, _) in &mut cams {
        pc.target_focus = spawn;
    }
}

/// Exit/back confirmation dialog (drawn on top whenever `show_exit` is set).
fn exit_dialog_system(
    mut contexts: EguiContexts,
    mut menu: ResMut<MenuState>,
    state: Res<State<AppState>>,
    mut next: ResMut<NextState<AppState>>,
    mut exit: MessageWriter<AppExit>,
) -> Result {
    if !menu.show_exit {
        return Ok(());
    }
    let Ok(ctx) = contexts.ctx_mut() else { return Ok(()); };
    let in_game = matches!(state.get(), AppState::Placement | AppState::Playing);
    let mut show = true;
    match menu_ui::draw_exit_confirm(ctx, &mut show, in_game) {
        ExitAction::Quit => {
            exit.write(AppExit::Success);
        }
        ExitAction::BackToMenu => {
            next.set(AppState::ModeSelect);
        }
        ExitAction::None => {}
    }
    menu.show_exit = show;
    Ok(())
}

/// ESC: menus open the quit-dialog / go back; in-game toggles the exit dialog.
fn handle_esc(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<AppState>>,
    mut menu: ResMut<MenuState>,
    mut next: ResMut<NextState<AppState>>,
) {
    if !keys.just_pressed(KeyCode::Escape) {
        return;
    }
    match state.get() {
        AppState::ModeSelect => menu.show_exit = !menu.show_exit,
        AppState::SceneSelect => next.set(AppState::ModeSelect),
        // Placement: Esc backs straight out to the menu (matches native). Playing: confirm dialog.
        AppState::Placement => next.set(AppState::ModeSelect),
        AppState::Playing => menu.show_exit = !menu.show_exit,
        AppState::Loading => {}
    }
}

/// `OnEnter(Loading)`: kick off the background PLY load (the splat plugin loads on a worker thread).
fn start_loading(scene_input: Res<SceneInput>, mut splat_scene: ResMut<splat_plugin::SplatScene>) {
    if let Some(ref path) = scene_input.path {
        info!("[Loading] starting PLY background load: {}", path);
        splat_scene.start_loading(path.clone());
    }
}

/// Update while in `Loading`: advance to `Placement` once the splat is uploaded to the GPU.
fn advance_when_loaded(
    splat_scene: Res<splat_plugin::SplatScene>,
    mut next: ResMut<NextState<AppState>>,
) {
    if splat_scene.loaded {
        info!("[Loading] splat ready -> Placement");
        next.set(AppState::Placement);
    }
}

/// `OnEnter(Placement)`: load this scene's persisted config (`world_up`/`spawn`/`heading`), apply
/// any `--world-up` CLI override, store it as `CurrentSceneConfig`, and write it back (a round-trip
/// that also creates `scenes.json` on first run). Placement + flight read this resource.
fn init_scene_config(
    mut commands: Commands,
    scene_input: Res<SceneInput>,
    splat_scene: Res<splat_plugin::SplatScene>,
) {
    let mut config = match &scene_input.path {
        Some(p) => persistence::load_scene_config(&persistence::scene_key(Path::new(p))),
        None => SceneConfig::default(),
    };
    if let Some(wu) = scene_input.world_up_override {
        config.world_up = wu;
    }
    if let Some(p) = &scene_input.path {
        if let Err(e) = persistence::save_scene_config(&persistence::scene_key(Path::new(p)), &config) {
            warn!("[Placement] failed to save scene config: {e}");
        }
    }
    // Anchor spawn to the scene center on first use (no saved spawn). Bevy world coords == PLY
    // coords for positions (the splat node passes camera position through unchanged), so the
    // point-cloud center is directly usable as a Bevy-space spawn / orbit focus.
    if config.spawn == SceneConfig::default().spawn {
        if let Some(c) = splat_scene.scene_center {
            config.spawn = c;
        }
    }
    info!(
        "[Placement] scene config: world_up={:?} spawn={:?} heading={:.1} deg",
        config.world_up, config.spawn, config.heading_deg
    );
    commands.insert_resource(PlacementUiState::from_config(&config));
    commands.insert_resource(CurrentSceneConfig(config));
}

/// `OnEnter(Placement)`: build the scene (demo PBR objects + lighting) and spawn the camera(s) for
/// the current `GameMode` (SinglePlayer = one full-window camera; SplitScreen = two stacked halves).
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mode: Res<GameMode>,
    config: Res<CurrentSceneConfig>,
) {
    // Orbit focus + demo-object anchor = the configured spawn point (scene center on first use).
    let focus = Vec3::from(config.0.spawn);

    // --- PBR 3D objects placed inside the gaussian splat scene ---
    let cube_mesh = meshes.add(Cuboid::new(8.0, 8.0, 8.0));
    let sphere_mesh = meshes.add(Sphere::new(5.0));

    // Red cube at the focus
    commands.spawn((
        Mesh3d(cube_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.15, 0.15),
            perceptual_roughness: 0.5,
            ..default()
        })),
        Transform::from_translation(focus),
        SceneEntity,
    ));
    // Blue sphere offset along +X
    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.45, 0.95),
            metallic: 0.3,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(focus + Vec3::new(16.0, 0.0, 0.0)),
        SceneEntity,
    ));
    // Green cube offset along -X
    commands.spawn((
        Mesh3d(cube_mesh),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.85, 0.3),
            perceptual_roughness: 0.6,
            ..default()
        })),
        Transform::from_translation(focus + Vec3::new(-16.0, 0.0, 0.0)),
        SceneEntity,
    ));

    // --- Lighting for the PBR meshes ---
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_translation(focus + Vec3::new(30.0, 60.0, 40.0))
            .looking_at(focus, Vec3::Z),
        SceneEntity,
    ));

    // --- Spawn marker: bright unlit sphere + forward arrow at the configured spawn point. ---
    // `placement_update` moves/rotates this each frame to follow `spawn` + `heading`, so editing
    // the spawn (text fields or WASD) is visible; the orbit camera also recenters on `spawn`.
    let marker_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.85, 0.1),
        emissive: LinearRgba::rgb(1.5, 1.2, 0.0),
        unlit: true,
        ..default()
    });
    commands
        .spawn((
            Transform::from_translation(focus),
            Visibility::default(),
            SpawnMarker,
            SceneEntity,
        ))
        .with_children(|p| {
            p.spawn((
                Mesh3d(meshes.add(Sphere::new(1.5))),
                MeshMaterial3d(marker_mat.clone()),
                Transform::default(),
            ));
            // Arrow modeled along +Y; the parent rotation aims it along the heading direction.
            p.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.4, 6.0, 0.4))),
                MeshMaterial3d(marker_mat),
                Transform::from_xyz(0.0, 3.5, 0.0),
            ));
        });

    // --- Split-screen: two stacked cameras (top = player 0, bottom = player 1) ---
    // Each renders the full scene (splat + PBR meshes) into its own viewport half of the shared
    // window target. `set_split_viewports` (re)assigns the viewport rects every frame; the
    // SplatNode reads each camera's viewport and restricts the splat raster to it.
    // Both cameras have PanOrbitCamera with distinct initial yaw, so bevy_panorbit_camera routes
    // mouse input per-viewport (hovering a half controls that half's camera).
    //
    // No MSAA: the splat rasterize uses a single-sample intermediate + the single-sample mesh
    // depth buffer; MSAA would multisample depth and mismatch the splat pass.
    let cams: &[(u8, isize, f32)] = match *mode {
        GameMode::SinglePlayer => &[(0, 0, 0.0)],
        GameMode::SplitScreen => &[(0, 0, 0.0), (1, 1, FRAC_PI_2)],
    };
    for &(index, order, yaw) in cams {
        let mut cam = commands.spawn((
            Camera3d::default(),
            Camera { order, ..default() },
            Tonemapping::None,
            Msaa::Off,
            AmbientLight {
                color: Color::WHITE,
                brightness: 400.0,
                ..default()
            },
            Transform::from_translation(focus + Vec3::new(5.0, 0.0, 3.0)).looking_at(focus, Vec3::Z),
            PanOrbitCamera {
                focus,
                radius: Some(45.0),
                yaw: Some(yaw),
                pitch: Some(-0.3),
                ..default()
            },
            SplatCamera,
            SceneEntity,
        ));
        // Only split-screen cameras get a viewport rect (assigned by `set_split_viewports`);
        // the single-player camera stays full-window (viewport = None).
        if matches!(*mode, GameMode::SplitScreen) {
            cam.insert(SplitCamera { index });
        }
    }

    info!("[Placement] scene ready ({:?}). Orbit: hover a viewport, Left-drag=orbit, Right-drag=pan, Scroll=zoom", *mode);
}

/// Marks a camera as one half of the split screen. `index` 0 = top, 1 = bottom.
#[derive(Component)]
struct SplitCamera {
    index: u8,
}

/// Assigns each split-screen camera its viewport rect (top/bottom half) based on the current
/// window size. Runs every frame but only writes when the rect actually changes (so window
/// resizes are handled without spamming change detection).
fn set_split_viewports(
    windows: Query<&Window>,
    mut cameras: Query<(&SplitCamera, &mut Camera)>,
) {
    let Ok(window) = windows.single() else { return; };
    let w = window.physical_width();
    let h = window.physical_height();
    if w == 0 || h == 0 {
        return;
    }
    let half = h / 2;
    for (split, mut cam) in &mut cameras {
        let (pos, size) = match split.index {
            0 => (UVec2::new(0, 0), UVec2::new(w, half)),
            _ => (UVec2::new(0, half), UVec2::new(w, h.saturating_sub(half))),
        };
        let needs_update = match &cam.viewport {
            Some(v) => v.physical_position != pos || v.physical_size != size,
            None => true,
        };
        if needs_update {
            cam.viewport = Some(Viewport {
                physical_position: pos,
                physical_size: size,
                depth: 0.0..1.0,
            });
        }
    }
}

fn update_window_title(
    diagnostics: Res<DiagnosticsStore>,
    mut windows: Query<&mut Window>,
    splat_scene: Res<splat_plugin::SplatScene>,
) {
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    let status = if splat_scene.loaded {
        "splat loaded"
    } else if splat_scene.ply_path.is_some() {
        "loading PLY..."
    } else {
        "no scene"
    };

    if let Ok(mut window) = windows.single_mut() {
        window.title = format!("{:.0} FPS | {}", fps, status);
    }
}
