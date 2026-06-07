//! Phase 9 audio — engine sound + gate-pass SFX (Bevy port of `src/audio.js` + the procedural
//! gate ding in `src/gates.js`). `native/src/audio.rs` was only a stub, so the JS is the spec.
//!
//! - **Engine** (`src/audio.js` `EngineAudio`): the looping motor sample (`assets/audio/fpv_loop.wav`,
//!   embedded) played through a custom [`EngineAudio`] rodio source whose playback-rate + gain track
//!   normalized throttle. Unlike `AudioSink::set_speed`/`set_volume` (one abrupt swap per call → an
//!   audible "zipper"/stepping during throttle sweeps), the decoder smooths both **per sample**, so
//!   the note glides continuously — matching Web Audio's `setTargetAtTime`. Spawned on entering
//!   flight, despawned on leaving it; one shared voice driven by the most-throttled armed drone.
//! - **Gate SFX** (`src/gates.js` `playGatePassSfx`): a short rising "ding" generated procedurally
//!   into an in-memory WAV at startup, fired one-shot whenever a player crosses a gate.
//! - **BGM** (`src/bgm.js` `BgmAudio`): one playlist active at a time (`init` in menus/placement,
//!   `flight` while racing), tracks played in a reshuffled sequence and looped; switching playlists
//!   crossfades. Tracks are **discovered from disk** at startup — drop any decodable file into
//!   `assets/audio/bgm/<init|flight>/` and it joins the loop next launch (the bundled set was
//!   transcoded FLAC→OGG, ~19 MB).

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;

use bevy::audio::{
    AddAudioSource, AudioPlayer, AudioSink, AudioSinkPlayback, AudioSource, Decodable,
    PlaybackSettings, Source, Volume,
};
use bevy::prelude::*;

use crate::app_state::AppState;
use crate::flight::Players;

// ---- Engine tuning (verbatim from src/audio.js) ----
// Playback-rate range (1.0 = native pitch/speed of the sample).
const RATE_IDLE: f32 = 0.7;
const RATE_FULL: f32 = 1.5;
// Master gain per state (samples are ~0 dBFS, so keep well below 1.0).
const GAIN_IDLE: f32 = 0.18;
const GAIN_FULL: f32 = 0.70;
// Smoothing time constant (seconds) for the gain/rate ramps. Larger = more sluggish.
const SMOOTH_TAU: f32 = 0.08;

// ---- BGM tuning (from src/bgm.js) ----
// Crossfade / fade-in / fade-out time (seconds) on track + playlist transitions.
const FADE_SEC: f32 = 0.6;

/// User-facing audio levels (0..1) + master mute. Defaults to full volume, unmuted. Driven by the
/// settings-panel sliders and persisted to `audio.json`; the engine/SFX/BGM systems all honour it.
/// `#[serde(default)]` keeps old/partial files loadable as fields are added.
#[derive(Resource, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct AudioSettings {
    pub engine_volume: f32,
    pub sfx_volume: f32,
    /// Background music is quieter by default so it sits under the engine + SFX (matches bgm.js).
    pub bgm_volume: f32,
    pub muted: bool,
}

impl Default for AudioSettings {
    fn default() -> Self {
        Self { engine_volume: 1.0, sfx_volume: 1.0, bgm_volume: 0.5, muted: false }
    }
}

/// Fired once per gate crossing (any player) so the audio layer can play the ding without coupling
/// to the gate race logic. Emitted by `gate_plugin::gate_race_system`.
#[derive(Message)]
pub struct GatePassed;

/// Handle to the procedural looping engine source (built once at startup).
#[derive(Resource)]
struct EngineHandle(Handle<EngineAudio>);

/// Shared, lock-free engine parameters: written each frame by `drive_engine_sound`, read
/// **per sample** by the audio thread (`EngineDecoder`). `f32` bit-cast into `AtomicU32`.
#[derive(Resource, Clone)]
struct EngineParams {
    /// Target playback rate (pitch/speed multiplier).
    rate: Arc<AtomicU32>,
    /// Target output gain — already folds in the idle→full envelope, user volume, and mute.
    gain: Arc<AtomicU32>,
}

/// Embedded engine loop (PCM16 WAV). Embedded so the engine voice never depends on cwd / async
/// asset loading; decoded to mono `f32` once at startup.
const ENGINE_WAV: &[u8] = include_bytes!("../assets/audio/fpv_loop.wav");

/// Custom looping engine audio source. Holds the decoded loop + the shared target rate/gain. Its
/// decoder runs on the audio thread and smooths rate + gain **per sample**, so throttle sweeps glide
/// continuously with no "zipper"/stepping (rodio's `set_speed`/`set_volume` only swap per call).
#[derive(Asset, TypePath, Clone)]
struct EngineAudio {
    samples: Arc<[f32]>,
    sample_rate: u32,
    rate: Arc<AtomicU32>,
    gain: Arc<AtomicU32>,
}

impl Decodable for EngineAudio {
    type DecoderItem = f32;
    type Decoder = EngineDecoder;

    fn decoder(&self) -> EngineDecoder {
        let sr = self.sample_rate.max(1);
        EngineDecoder {
            samples: self.samples.clone(),
            sample_rate: sr,
            rate: self.rate.clone(),
            gain: self.gain.clone(),
            pos: 0.0,
            cur_rate: RATE_IDLE,
            cur_gain: 0.0,
            // One-pole coefficient for an exp approach over `SMOOTH_TAU` seconds, applied per sample.
            k: 1.0 - (-1.0 / (SMOOTH_TAU * sr as f32)).exp(),
        }
    }
}

/// Per-playback decoder: fractional-rate, linearly-interpolated loop readout with per-sample
/// smoothing of rate + gain. Infinite (never ends) — stopped by despawning the engine entity.
struct EngineDecoder {
    samples: Arc<[f32]>,
    sample_rate: u32,
    rate: Arc<AtomicU32>,
    gain: Arc<AtomicU32>,
    pos: f64,
    cur_rate: f32,
    cur_gain: f32,
    k: f32,
}

impl Iterator for EngineDecoder {
    type Item = f32;

    fn next(&mut self) -> Option<f32> {
        let n = self.samples.len();
        if n == 0 {
            return Some(0.0);
        }
        // Glide toward the shared targets (one-pole, per-sample → click-free).
        self.cur_rate += (load_f32(&self.rate) - self.cur_rate) * self.k;
        self.cur_gain += (load_f32(&self.gain) - self.cur_gain) * self.k;

        // Linear interpolation between adjacent loop samples at the fractional read position.
        let i0 = self.pos as usize % n;
        let i1 = (i0 + 1) % n;
        let frac = (self.pos - self.pos.floor()) as f32;
        let s = self.samples[i0] * (1.0 - frac) + self.samples[i1] * frac;

        self.pos += self.cur_rate.max(0.0) as f64;
        while self.pos >= n as f64 {
            self.pos -= n as f64;
        }
        Some(s * self.cur_gain)
    }
}

impl Source for EngineDecoder {
    fn current_frame_len(&self) -> Option<usize> {
        None
    }
    fn channels(&self) -> u16 {
        1
    }
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }
    fn total_duration(&self) -> Option<Duration> {
        None
    }
}

#[inline]
fn store_f32(a: &AtomicU32, v: f32) {
    a.store(v.to_bits(), Ordering::Relaxed);
}

#[inline]
fn load_f32(a: &AtomicU32) -> f32 {
    f32::from_bits(a.load(Ordering::Relaxed))
}

/// Minimal PCM16 WAV → mono `f32` decoder (channels downmixed by averaging). Returns
/// `(samples, sample_rate)`, or `None` if `bytes` is not a PCM16 WAV.
fn decode_wav_mono_f32(bytes: &[u8]) -> Option<(Vec<f32>, u32)> {
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut pos = 12;
    let (mut channels, mut sample_rate, mut bits) = (0u16, 0u32, 0u16);
    let mut data: Option<&[u8]> = None;
    while pos + 8 <= bytes.len() {
        let id = &bytes[pos..pos + 4];
        let sz =
            u32::from_le_bytes([bytes[pos + 4], bytes[pos + 5], bytes[pos + 6], bytes[pos + 7]])
                as usize;
        let body = pos + 8;
        let end = body.saturating_add(sz).min(bytes.len());
        match id {
            b"fmt " if end - body >= 16 => {
                let b = &bytes[body..];
                channels = u16::from_le_bytes([b[2], b[3]]);
                sample_rate = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                bits = u16::from_le_bytes([b[14], b[15]]);
            }
            b"data" => data = Some(&bytes[body..end]),
            _ => {}
        }
        pos = end + (sz & 1); // chunks are word-aligned (skip the pad byte for odd sizes)
    }
    let data = data?;
    if channels == 0 || sample_rate == 0 || bits != 16 {
        return None;
    }
    let ch = channels as usize;
    let frames = data.len() / (2 * ch);
    let mut out = Vec::with_capacity(frames);
    for f in 0..frames {
        let base = f * 2 * ch;
        let mut acc = 0i32;
        for c in 0..ch {
            let o = base + c * 2;
            acc += i16::from_le_bytes([data[o], data[o + 1]]) as i32;
        }
        out.push(acc as f32 / ch as f32 / 32768.0);
    }
    Some((out, sample_rate))
}

/// Handle to the procedurally-generated gate "ding" (built once at startup).
#[derive(Resource)]
struct GateDingHandle(Handle<AudioSource>);

/// Marks the single looping engine-sound entity (spawned per flight session).
#[derive(Component)]
struct EngineSound;

/// Marks the single BGM track entity currently playing (or fading).
#[derive(Component)]
struct BgmTrack;

#[derive(Clone, Copy, PartialEq)]
enum BgmMode {
    /// Nothing playing; the driver will start the desired playlist's next track.
    Idle,
    FadingIn,
    Playing,
    FadingOut,
}

/// Playlist-based background-music state (port of `src/bgm.js` `BgmAudio`). Holds the loaded track
/// handles per playlist + the current shuffle order, and drives one-track-at-a-time playback with
/// fades. `init` plays in menus/placement, `flight` while racing.
#[derive(Resource)]
struct BgmController {
    init: Vec<Handle<AudioSource>>,
    flight: Vec<Handle<AudioSource>>,
    /// Name of the playlist currently being played (`"init"`/`"flight"`), or `None` when stopped.
    current: Option<&'static str>,
    /// Shuffle order (indices into the active playlist) + cursor into it.
    shuffled: Vec<usize>,
    idx: usize,
    /// Last track index played (to avoid an immediate repeat after a reshuffle).
    last: Option<usize>,
    /// Fade envelope in 0..1 (multiplied by the user BGM volume).
    fade: f32,
    mode: BgmMode,
    /// True once the current sink has actually started (guards the load delay + initial `empty()`).
    track_started: bool,
    /// xorshift64 state for the Fisher–Yates shuffle (seeded from wall-clock at startup).
    rng: u64,
}

impl BgmController {
    fn tracks(&self, name: &str) -> &[Handle<AudioSource>] {
        if name == "flight" {
            &self.flight
        } else {
            &self.init
        }
    }

    fn next_rand(&mut self) -> u64 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng = x;
        x
    }

    /// Fisher–Yates shuffle of `0..n`, biased to avoid putting the just-played track first.
    fn reshuffle(&mut self, n: usize) {
        self.shuffled = (0..n).collect();
        for i in (1..n).rev() {
            let j = (self.next_rand() % (i as u64 + 1)) as usize;
            self.shuffled.swap(i, j);
        }
        if n > 1 {
            if let Some(last) = self.last {
                if self.shuffled[0] == last {
                    self.shuffled.swap(0, 1);
                }
            }
        }
        self.idx = 0;
    }

    /// Next track handle for `name`, reshuffling + looping when the order is exhausted.
    fn pick_next(&mut self, name: &'static str) -> Option<Handle<AudioSource>> {
        let n = self.tracks(name).len();
        if n == 0 {
            return None;
        }
        if self.idx >= self.shuffled.len() {
            self.reshuffle(n);
        }
        let track_i = self.shuffled[self.idx];
        self.idx += 1;
        self.last = Some(track_i);
        Some(self.tracks(name)[track_i].clone())
    }
}

pub struct AudioPlugin;

impl Plugin for AudioPlugin {
    fn build(&self, app: &mut App) {
        // Restore persisted volumes/mute (falls back to defaults if the file is absent).
        app.insert_resource(crate::persistence::load_audio_settings().unwrap_or_default());
        app.add_audio_source::<EngineAudio>();
        app.add_message::<GatePassed>();
        app.add_systems(Startup, setup_audio);
        app.add_systems(OnEnter(AppState::Playing), spawn_engine_sound);
        app.add_systems(OnExit(AppState::Playing), despawn_engine_sound);
        // BGM plays in every state (init in menus, flight while racing); engine + SFX are flight-only.
        app.add_systems(Update, drive_bgm);
        app.add_systems(
            Update,
            (drive_engine_sound, gate_sfx_system).run_if(in_state(AppState::Playing)),
        );
    }
}

/// Decodable audio extensions for BGM discovery (limited to what the enabled Bevy features can play:
/// `wav` + `vorbis`). Files with other extensions are ignored so a playlist never stalls on a track
/// rodio can't decode.
const BGM_EXTENSIONS: &[&str] = &["ogg", "oga", "wav"];

/// Scan `assets/audio/bgm/<subdir>/` for decodable audio files and return their loaded handles
/// (sorted for determinism; the playlist shuffles playback anyway). Mirrors bgm.js `_discoverPlaylist`
/// so dropping a track into the folder adds it to the loop on the next launch. Empty if the folder is
/// missing/unreadable (that playlist just won't play).
fn discover_playlist(asset_server: &AssetServer, subdir: &str) -> Vec<Handle<AudioSource>> {
    let dir = std::path::Path::new("assets/audio/bgm").join(subdir);
    let mut names: Vec<String> = match std::fs::read_dir(&dir) {
        Ok(rd) => rd
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.is_file())
            .filter_map(|p| {
                let ext = p.extension()?.to_str()?.to_ascii_lowercase();
                if BGM_EXTENSIONS.contains(&ext.as_str()) {
                    p.file_name()?.to_str().map(str::to_string)
                } else {
                    None
                }
            })
            .collect(),
        Err(e) => {
            warn!("[BGM] cannot read {}: {} — no '{}' music", dir.display(), e, subdir);
            Vec::new()
        }
    };
    names.sort();
    let handles: Vec<Handle<AudioSource>> = names
        .iter()
        .map(|n| asset_server.load(format!("audio/bgm/{subdir}/{n}")))
        .collect();
    info!("[BGM] playlist '{}': {} track(s)", subdir, handles.len());
    handles
}

/// `Startup`: build the custom engine source from the embedded loop, bake the procedural gate ding,
/// and discover the BGM playlists.
fn setup_audio(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut sources: ResMut<Assets<AudioSource>>,
    mut engines: ResMut<Assets<EngineAudio>>,
) {
    // Engine: decode the embedded loop to mono f32 and wrap it in a custom per-sample-smoothed
    // source so throttle sweeps glide continuously (no rodio set_speed zipper).
    let (samples, sr) = decode_wav_mono_f32(ENGINE_WAV).unwrap_or_else(|| {
        warn!("[audio] engine loop decode failed; engine will be silent");
        (vec![0.0_f32], 44_100)
    });
    let rate = Arc::new(AtomicU32::new(RATE_IDLE.to_bits()));
    let gain = Arc::new(AtomicU32::new(0.0_f32.to_bits()));
    let engine = engines.add(EngineAudio {
        samples: samples.into(),
        sample_rate: sr,
        rate: rate.clone(),
        gain: gain.clone(),
    });
    commands.insert_resource(EngineHandle(engine));
    commands.insert_resource(EngineParams { rate, gain });

    let ding = sources.add(AudioSource { bytes: make_gate_ding_wav().into() });
    commands.insert_resource(GateDingHandle(ding));

    // BGM playlists discovered from disk (like bgm.js `_discoverPlaylist`): drop any decodable audio
    // file into `assets/audio/bgm/<init|flight>/` and it joins the loop on next launch. init =
    // menus/placement, flight = racing.
    let init = discover_playlist(&asset_server, "init");
    let flight = discover_playlist(&asset_server, "flight");
    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x9E37_79B9_7F4A_7C15)
        | 1; // xorshift64 requires a non-zero seed
    commands.insert_resource(BgmController {
        init,
        flight,
        current: None,
        shuffled: Vec::new(),
        idx: 0,
        last: None,
        fade: 0.0,
        mode: BgmMode::Idle,
        track_started: false,
        rng: seed,
    });
}

/// `OnEnter(Playing)`: start the engine voice silent at idle pitch; the decoder fades the gain in
/// per-sample as `drive_engine_sound` raises the target.
fn spawn_engine_sound(mut commands: Commands, engine: Res<EngineHandle>, params: Res<EngineParams>) {
    store_f32(&params.gain, 0.0);
    store_f32(&params.rate, RATE_IDLE);
    commands.spawn((
        AudioPlayer::<EngineAudio>(engine.0.clone()),
        PlaybackSettings::ONCE,
        EngineSound,
    ));
}

/// `OnExit(Playing)`: stop the engine when leaving flight (back to menu / mode select).
fn despawn_engine_sound(mut commands: Commands, q: Query<Entity, With<EngineSound>>) {
    for e in &q {
        commands.entity(e).despawn();
    }
}

/// `Update` (Playing): publish the engine's target gain (idle→full by throttle, 0 when
/// disarmed/muted) and playback rate for the audio thread. The decoder glides to these targets
/// **per sample** (one-pole, `SMOOTH_TAU`), so there's no per-frame stepping. One shared voice
/// follows the most-throttled armed drone so split-screen still gets a responsive engine note.
fn drive_engine_sound(
    players: Res<Players>,
    settings: Res<AudioSettings>,
    params: Res<EngineParams>,
) {
    let mut armed = false;
    let mut max_t = 0.0_f32;
    for p in &players.0 {
        if p.armed {
            armed = true;
        }
        let mt = p.drone.max_thrust.max(1.0);
        let t = (p.drone.thrust_output / mt).clamp(0.0, 1.0);
        if t > max_t {
            max_t = t;
        }
    }

    let user = settings.engine_volume.clamp(0.0, 1.0);
    let target_gain = if settings.muted || !armed {
        0.0
    } else {
        (GAIN_IDLE + (GAIN_FULL - GAIN_IDLE) * max_t) * user
    };
    let target_rate = RATE_IDLE + (RATE_FULL - RATE_IDLE) * max_t;

    store_f32(&params.gain, target_gain);
    store_f32(&params.rate, target_rate);
}

/// `Update` (Playing): play one gate "ding" when any player crossed a gate this frame.
fn gate_sfx_system(
    mut commands: Commands,
    mut reader: MessageReader<GatePassed>,
    ding: Res<GateDingHandle>,
    settings: Res<AudioSettings>,
) {
    // Drain the queue regardless so it never backs up; collapse multiple same-frame passes to one
    // ding to avoid an overlapping cacophony.
    let passed = reader.read().count() > 0;
    if !passed || settings.muted {
        return;
    }
    let vol = settings.sfx_volume.clamp(0.0, 1.0);
    commands.spawn((
        AudioPlayer::new(ding.0.clone()),
        PlaybackSettings::DESPAWN.with_volume(Volume::Linear(vol)),
    ));
}

/// `Update` (all states): one-track-at-a-time BGM with fades. Picks the playlist from the app state
/// (`Playing` → `flight`, else `init`), fades between playlists, advances when a track ends, and
/// reshuffles+loops at the end of the list. Bevy gives no "ended" callback, so we poll `AudioSink`:
/// a finished `Once` sink reports `empty()`, which (once the track has actually started) advances us.
fn drive_bgm(
    mut commands: Commands,
    time: Res<Time>,
    state: Res<State<AppState>>,
    settings: Res<AudioSettings>,
    mut ctrl: ResMut<BgmController>,
    mut q: Query<(Entity, Option<&mut AudioSink>), With<BgmTrack>>,
) {
    let desired: &'static str = if *state.get() == AppState::Playing { "flight" } else { "init" };
    let step = if FADE_SEC > 0.0 { time.delta_secs() / FADE_SEC } else { 1.0 };
    let user = settings.bgm_volume.clamp(0.0, 1.0);
    let muted = settings.muted;
    let applied = |fade: f32| if muted { 0.0 } else { user * fade };

    if let Some((entity, sink_opt)) = q.iter_mut().next() {
        // A pending playlist change interrupts whatever we're doing with a fade-out.
        if ctrl.current != Some(desired) && ctrl.mode != BgmMode::FadingOut {
            ctrl.mode = BgmMode::FadingOut;
        }
        match ctrl.mode {
            BgmMode::FadingOut => match sink_opt {
                Some(mut sink) => {
                    ctrl.fade = (ctrl.fade - step).max(0.0);
                    sink.set_volume(Volume::Linear(applied(ctrl.fade)));
                    if ctrl.fade <= 0.0 {
                        commands.entity(entity).despawn();
                        ctrl.current = None;
                        ctrl.track_started = false;
                        ctrl.mode = BgmMode::Idle;
                    }
                }
                // Never started (still loading) — cancel immediately.
                None => {
                    commands.entity(entity).despawn();
                    ctrl.current = None;
                    ctrl.track_started = false;
                    ctrl.mode = BgmMode::Idle;
                }
            },
            BgmMode::FadingIn => {
                if let Some(mut sink) = sink_opt {
                    if !sink.empty() {
                        ctrl.track_started = true;
                    }
                    ctrl.fade = (ctrl.fade + step).min(1.0);
                    sink.set_volume(Volume::Linear(applied(ctrl.fade)));
                    if ctrl.fade >= 1.0 {
                        ctrl.mode = BgmMode::Playing;
                    }
                }
            }
            BgmMode::Playing => {
                if let Some(mut sink) = sink_opt {
                    let empty = sink.empty();
                    if !empty {
                        ctrl.track_started = true;
                    }
                    sink.set_volume(Volume::Linear(applied(1.0)));
                    if ctrl.track_started && empty {
                        // Track finished — drop it; the next track starts on the following frame.
                        commands.entity(entity).despawn();
                        ctrl.track_started = false;
                        ctrl.mode = BgmMode::Idle;
                    }
                }
            }
            BgmMode::Idle => {}
        }
    } else {
        // Nothing playing: (re)start the desired playlist.
        if ctrl.current != Some(desired) {
            let n = ctrl.tracks(desired).len();
            if n == 0 {
                return;
            }
            ctrl.current = Some(desired);
            ctrl.reshuffle(n);
        }
        if let Some(handle) = ctrl.pick_next(desired) {
            commands.spawn((
                AudioPlayer::new(handle),
                PlaybackSettings::ONCE.with_volume(Volume::Linear(applied(0.0))),
                BgmTrack,
            ));
            ctrl.fade = 0.0;
            ctrl.track_started = false;
            ctrl.mode = BgmMode::FadingIn;
        }
    }
}

// ---- Procedural gate ding (port of `src/gates.js` playGatePassSfx) ----

/// Build a short two-oscillator "checkpoint ding" as a mono 16-bit PCM WAV (decoded by rodio).
/// osc1: 1320 Hz sine, gain 0.18→0.001 over 0.25 s; osc2: 1760 Hz triangle, 0.08→0.001 over 0.15 s.
fn make_gate_ding_wav() -> Vec<u8> {
    const SR: u32 = 44_100;
    let dur = 0.25_f32;
    let n = (SR as f32 * dur) as usize;
    let mut samples: Vec<i16> = Vec::with_capacity(n);
    let tau = std::f32::consts::TAU;
    for i in 0..n {
        let tt = i as f32 / SR as f32;
        // osc1 — sine 1320 Hz with exponential decay.
        let env1 = 0.18 * (0.001f32 / 0.18).powf((tt / 0.25).min(1.0));
        let s1 = env1 * (tau * 1320.0 * tt).sin();
        // osc2 — triangle 1760 Hz, faster decay, silent after 0.15 s.
        let s2 = if tt <= 0.15 {
            let env2 = 0.08 * (0.001f32 / 0.08).powf(tt / 0.15);
            let phase = 1760.0 * tt;
            let tri = 2.0 * (2.0 * (phase - (phase + 0.5).floor())).abs() - 1.0;
            env2 * tri
        } else {
            0.0
        };
        let s = (s1 + s2).clamp(-1.0, 1.0);
        samples.push((s * 32_767.0) as i16);
    }
    encode_wav_mono_i16(&samples, SR)
}

/// Minimal canonical 44-byte-header mono 16-bit PCM WAV encoder.
fn encode_wav_mono_i16(samples: &[i16], sr: u32) -> Vec<u8> {
    let data_len = (samples.len() * 2) as u32;
    let byte_rate = sr * 2; // mono * 2 bytes/sample
    let mut v = Vec::with_capacity(44 + data_len as usize);
    v.extend_from_slice(b"RIFF");
    v.extend_from_slice(&(36 + data_len).to_le_bytes());
    v.extend_from_slice(b"WAVE");
    v.extend_from_slice(b"fmt ");
    v.extend_from_slice(&16u32.to_le_bytes()); // PCM fmt chunk size
    v.extend_from_slice(&1u16.to_le_bytes()); // audio format = PCM
    v.extend_from_slice(&1u16.to_le_bytes()); // channels = mono
    v.extend_from_slice(&sr.to_le_bytes());
    v.extend_from_slice(&byte_rate.to_le_bytes());
    v.extend_from_slice(&2u16.to_le_bytes()); // block align
    v.extend_from_slice(&16u16.to_le_bytes()); // bits per sample
    v.extend_from_slice(b"data");
    v.extend_from_slice(&data_len.to_le_bytes());
    for s in samples {
        v.extend_from_slice(&s.to_le_bytes());
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wav_header_is_well_formed() {
        let wav = encode_wav_mono_i16(&[0, 1, -1, 32767, -32768], 44_100);
        assert_eq!(&wav[0..4], b"RIFF");
        assert_eq!(&wav[8..12], b"WAVE");
        assert_eq!(&wav[12..16], b"fmt ");
        assert_eq!(&wav[36..40], b"data");
        // 5 mono i16 samples = 10 data bytes; file = 44-byte header + data.
        let data_len = u32::from_le_bytes(wav[40..44].try_into().unwrap());
        assert_eq!(data_len, 10);
        assert_eq!(wav.len(), 44 + 10);
        let riff_len = u32::from_le_bytes(wav[4..8].try_into().unwrap());
        assert_eq!(riff_len as usize, wav.len() - 8);
    }

    #[test]
    fn gate_ding_is_a_nonempty_quarter_second_clip() {
        let wav = make_gate_ding_wav();
        assert_eq!(&wav[0..4], b"RIFF");
        // 0.25 s mono @ 44.1 kHz = 11025 samples = 22050 data bytes.
        let data_len = u32::from_le_bytes(wav[40..44].try_into().unwrap());
        assert_eq!(data_len, 11_025 * 2);
        // First sample is silence-ish (both envelopes start small), last 16-bit sample exists.
        assert_eq!(wav.len(), 44 + 11_025 * 2);
    }
}
