//! Menu shell: the egui mode/scene-select + loading screens, the exit/back dialog, the persistent
//! UI camera, and the scene scan/cleanup + PLY load kickoff. Registered by [`MenuPlugin`].

use bevy::camera::ClearColorConfig;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass};
use std::path::{Path, PathBuf};

use crate::app_state::{AppState, GameMode};
use crate::menu_ui::{self, ExitAction, ModeAction, SceneAction};
use crate::scene::SceneEntity;
use crate::splat_plugin;
use crate::SceneInput;

/// Shared state for the menu systems. `show_exit` is also read by the placement systems (so the
/// orbit/spawn editing pauses while the dialog is up).
#[derive(Resource, Default)]
pub(crate) struct MenuState {
    scene_files: Vec<PathBuf>,
    selected: Option<usize>,
    pub(crate) show_exit: bool,
}

/// Marks the UI/menu camera (Camera2d) that hosts egui — persists across all states.
#[derive(Component)]
struct UiCamera;

/// Wires the menu shell: UI camera at startup, the egui screens (gated by state), scene scan/cleanup
/// on state entry, the PLY load kickoff, and the loading→placement advance + global Esc handling.
pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MenuState::default());
        app.add_systems(Startup, setup_ui_camera);
        app.add_systems(
            EguiPrimaryContextPass,
            (
                mode_select_system.run_if(in_state(AppState::ModeSelect)),
                scene_select_system.run_if(in_state(AppState::SceneSelect)),
                loading_screen_system.run_if(in_state(AppState::Loading)),
                exit_dialog_system,
            ),
        );
        app.add_systems(OnEnter(AppState::ModeSelect), cleanup_scene);
        app.add_systems(OnEnter(AppState::SceneSelect), scan_scenes);
        app.add_systems(OnEnter(AppState::Loading), start_loading);
        app.add_systems(
            Update,
            (advance_when_loaded.run_if(in_state(AppState::Loading)), handle_esc),
        );
    }
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
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
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
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
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
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
    let name = scene_input
        .path
        .as_deref()
        .and_then(|p| Path::new(p).file_name())
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "scene".to_string());
    menu_ui::draw_loading_screen(ctx, &name);
    Ok(())
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
    let Ok(ctx) = contexts.ctx_mut() else {
        return Ok(());
    };
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
