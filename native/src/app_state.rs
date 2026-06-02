/// Application state machine:
///   ModeSelect -> SceneSelect -> Loading -> Placement -> Playing
///                    ^                                      |
///                    +---------- (Esc in game) -------------+

use std::path::PathBuf;

/// Game mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GameMode {
    SinglePlayer,
    SplitScreen,
}

/// World coordinate system (scene-dependent).
#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum WorldUp {
    /// Z-up: +Z = sky, gravity = (0, 0, -G). Horizontal plane = XY.
    Zup,
    /// COLMAP: +Y = gravity direction (down), gravity = (0, +G, 0). Horizontal plane = XZ.
    Colmap,
}

/// Per-scene persistent config.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SceneConfig {
    pub world_up: WorldUp,
    pub spawn: [f32; 3],
    pub heading_deg: f32,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            world_up: WorldUp::Zup,
            spawn: [0.0, 0.0, 2.0],
            heading_deg: 0.0,
        }
    }
}

/// Result of a state's frame update.
pub enum StateTransition {
    None,
    ToModeSelect,
    ToSceneSelect(GameMode),
    ToLoading(PathBuf, GameMode),
    ToPlacement,
    ToPlaying,
    Quit,
}
