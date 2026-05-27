/// Application state machine:
///   Splash → SceneSelect → Loading → Playing
///
/// Each state renders its own UI via egui on a simple wgpu surface.
/// The Playing state owns the WindowContext (GSplat renderer).

use std::path::PathBuf;

/// Top-level app state.
pub enum AppState {
    /// Initial splash screen with logo/title.
    Splash { timer: f32 },
    /// Scene selection menu.
    SceneSelect,
    /// Loading a scene file (shows progress).
    Loading { path: PathBuf, status: String },
    /// In-game (single or split-screen).
    Playing,
}

/// Game mode selected before entering Playing state.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum GameMode {
    SinglePlayer,
    SplitScreen,
}

/// Result of a state's frame update — tells the main loop what to do next.
pub enum StateTransition {
    /// Stay in current state.
    None,
    /// Move to scene selection.
    ToSceneSelect,
    /// Start loading a scene.
    ToLoading(PathBuf, GameMode),
    /// Scene loaded, enter game.
    ToPlaying,
    /// Quit the app.
    Quit,
}
