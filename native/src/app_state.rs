/// Application state machine:
///   ModeSelect -> SceneSelect -> Loading -> Playing
///                    ^                        |
///                    +---- (Esc in game) -----+

use std::path::PathBuf;

/// Game mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GameMode {
    SinglePlayer,
    SplitScreen,
}

/// Result of a state's frame update.
pub enum StateTransition {
    None,
    ToModeSelect,
    ToSceneSelect(GameMode),
    ToLoading(PathBuf, GameMode),
    ToPlaying,
    Quit,
}
