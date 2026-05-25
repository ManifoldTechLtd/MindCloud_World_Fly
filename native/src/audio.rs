/// Audio system — engine sound + BGM.
/// Placeholder for kira/rodio integration (Batch 6).
/// Ported from src/audio.js + src/bgm.js.

/// Engine sound state — throttle-modulated pitch/gain.
pub struct EngineAudio {
    enabled: bool,
    // TODO: kira/rodio sound handle
}

impl EngineAudio {
    pub fn new() -> Self {
        Self { enabled: false }
    }

    /// Update engine sound based on throttle (0..1) and armed state.
    pub fn update(&mut self, _throttle_01: f32, _armed: bool) {
        // TODO: implement with kira/rodio
        // - pitch = base_pitch * (1.0 + throttle_01 * pitch_range)
        // - gain  = idle_gain + throttle_01 * (max_gain - idle_gain)
        // - if !armed: fade to silence
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
}

/// Background music player — playlist shuffle.
pub struct BgmPlayer {
    enabled: bool,
    // TODO: kira/rodio playlist state
}

impl BgmPlayer {
    pub fn new() -> Self {
        Self { enabled: false }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Switch playlist for game mode (init / flight).
    pub fn play_playlist(&mut self, _name: &str) {
        // TODO: implement playlist loading + shuffle + crossfade
    }
}
