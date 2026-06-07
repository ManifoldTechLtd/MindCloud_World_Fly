# MindCloud World Fly

<p align="center">
  <img src="asset/display/mt_mcwf_logo.jpg" alt="MindCloud World Fly Logo" height="80">
</p>

<p align="center">
  <img src="asset/display/demo_flight.gif" alt="MindCloud World Fly — Flight demo" width="100%">
</p>

<p align="center">
  <img src="asset/display/demo_teaser2.jpg" alt="MindCloud World Fly — Drone mode flight in a 3DGS scene" width="100%">
</p>

A native drone flight simulator for **3D Gaussian Splatting** scenes. Fly through any 3DGS scene with
realistic quadcopter physics, FPV or stabilized flight, single- or split-screen play, a built-in
gate-racing mode, and RC-transmitter support.

It is a **Rust / [Bevy](https://bevyengine.org) 0.18** application: Bevy renders the PBR 3D objects
(gates, spawn marker, UI) while the proven **[web-splat](https://github.com/KeKsBoTer/web-splat)**
Gaussian-splat renderer draws the scene as a background, both sharing one `wgpu` device with full
depth occlusion between splats and meshes.

## About Manifold Tech

[Manifold Tech Ltd.](https://manifoldtech.cn) builds tools and infrastructure for spatial
intelligence — 3D reconstruction, scene understanding, and embodied AI, bridging real-world capture
and interactive simulation. Manifold Tech hardware (**Q9000**, **Pocket 2 / 2 Pro**, **Odin 1**) can
capture high-quality 3DGS models of real environments, which load directly into MindCloud World Fly
as flyable scenes.

## Requirements

| Requirement | Notes |
|-------------|-------|
| **Rust** | Stable toolchain (edition 2021). Install via [rustup](https://rustup.rs). |
| **GPU + Vulkan** | A Vulkan-capable GPU and drivers (validated on NVIDIA RTX, Linux). `wgpu` 27 backend. |
| **Linux / X11** | Built with the `x11` Bevy feature. Other platforms may work but are untested. |
| **A 3DGS scene** | A `.ply` Gaussian-splat file (see [Scenes](#scenes)). |
| **(Optional) RC transmitter** | USB HID transmitter for hardware-in-the-loop control. |

## Build & Run

The application lives in [`native-bevy/`](native-bevy). All commands are run from there:

```bash
cd native-bevy

# Build (the first build compiles Bevy and may take several minutes):
cargo build --release

# Run — opens the in-app menu (Single Player / Split Screen → Scene Select):
cargo run --release

# …or jump straight into a scene, skipping the menus:
cargo run --release -- --input ../scene/your_scene.ply
```

### Command-line options

| Flag | Description |
|------|-------------|
| `-i`, `--input <PATH>` | Load a `.ply` scene immediately (skips the menu). |
| `--split` | Split-screen, two players (default is single-player, one full-window view). |
| `--world-up <zup\|colmap>` | Override the up-axis convention (`zup` default, `colmap` = Y-down). |
| `--no-vsync` | Disable vsync. |

> **Dev builds:** plain `cargo run` works too. The build profile optimizes dependencies
> (`opt-level=3`) while keeping fast incremental builds of the app itself (`opt-level=1`).

<a name="scenes"></a>
## Scenes

Scene files are **not** committed (too large). Drop your Gaussian-splat files into the repo's
`scene/` directory (or `native-bevy/`); the **Scene Select** menu scans `.`, `scene/`, and
`../scene/` for `*.ply`, `*.splat`, and `*.sog` files.

```
scene/
└── your_scene.ply
```

Two ready-to-fly demo scenes captured with Manifold Tech hardware:

- [**field_z-up**](https://drive.google.com/file/d/11yztizITalnHwnichd4iXHVaQbMYplTD/view?usp=sharing) — outdoor field, **Z-up** (`--world-up zup`).
- [**nanjing**](https://drive.google.com/file/d/1ft5q-ALGwFB3hp44vt648kdYUoNi_S9f/view?usp=sharing) — urban scene captured in Nanjing, China.

<p align="center">
  <img src="asset/display/demo_field.gif" alt="Field scene flight demo" width="49%">
  <img src="asset/display/demo_nanjing.gif" alt="Nanjing scene flight demo" width="49%">
</p>

## How it plays

1. **Mode** — pick *Single Player* or *Split Screen (2 Players)* (or pass `--split`).
2. **Scene** — select a scene file; it loads on a background thread (point cloud + collision octree).
3. **Placement** — position the spawn point with an orbit camera, optionally draw a gate course.
4. **Fly** — press **Enter** to start; arm the drone and fly.

### Placement controls

| Input | Action |
|-------|--------|
| **W / A / S / D** | Move spawn horizontally (camera-relative) |
| **Q / E** | Move spawn down / up |
| **Shift** | Move faster |
| **Left-drag / Scroll** | Orbit camera / zoom |
| **Edit Gate Path** (button) | Open the top-down gate-course editor |
| **Enter** | Start flight |
| **Esc** | Back to menu |

### Flight controls (keyboard)

| Player 1 | Player 2 (split) | Action |
|----------|------------------|--------|
| **W / S** | **T / G** | Throttle up / down |
| **A / D** | **F / H** | Yaw left / right |
| **↑ / ↓** | **I / K** | Pitch forward / back |
| **← / →** | **J / L** | Roll left / right |
| **Space** | **Enter** | Arm / disarm |
| **M** | **N** | Toggle flight mode (FPV ↔ Drone) |
| **R** | — | Reset drone(s) to spawn |

| Key | Action |
|-----|--------|
| **F1** | Open / close the settings panel (pauses flight) |
| **G** | Toggle gate-course visibility |
| **P** | Start the 3-2-1 race countdown (split-screen race) |
| **Esc** | Exit-scene dialog (or close settings) |

You must **arm** (Space) before the drone responds.

### Flight modes

| Mode | Behavior |
|------|----------|
| **Drone (stabilized)** | Position/altitude hold; sticks command velocity, release to hover. Cascaded PID keeps it level. Best for exploration. |
| **FPV (manual rate)** | Direct body-rate control, no self-leveling, throttle = thrust. Realistic FPV feel. |

Switch any time with **M** (P2: **N**), or via the settings panel. Each mode keeps its own PID gains
and rate/expo parameters. Auto-levels on disarm.

### HUD / OSD

Fighter-jet style overlay: artificial horizon + pitch ladder, speed/altitude tapes, heading compass,
vertical-speed indicator, armed state, FPS, and (during a race) lap timer + gate count. In
split-screen each half draws its own HUD.

## Gate racing

Draw a **closed-loop gate course** in the top-down editor (placement → **Edit Gate Path**), then race
it for best lap time. Gates are sensor rings on a smooth Catmull-Rom spline; pass-through is detected
per frame (the drone never physically collides with them).

**Editor:** left-click to add a gate, drag to move, **Z / X** to lower / raise altitude,
**Del / Backspace** to remove, wheel to zoom, right-drag to pan, **Enter** to accept (≥ 3 gates),
**Esc** to cancel. Each gate previews **green** when clear of the point cloud, **red** when it
intersects geometry.

- **Single-player** = lap racing (the course loops; best lap is saved per scene).
- **Split-screen** = a head-to-head linear race: press **P** for a 3-2-1 countdown, first to cross the
  last gate wins (per-half **FINISH No.1 / No.2** banners). **R** restarts the race.

Gate frames render with a black/white checker start gate and per-face shading; only the next few gates
ahead are shown while flying.

## Physics & collision

Quaternion-based rigid-body physics: thrust along the body up-axis, quadratic aerodynamic drag, and
gravity. Tunable live in the settings panel (mass, max thrust, drag, frontal area, drone size,
collision radius).

Collision uses an **octree** built from the scene's Gaussian centers (filtered by opacity). On impact
the drone is pushed out along the surface normal and its velocity is reflected/damped, with a red
on-screen collision flash.

## RC transmitter (HID)

Connect a USB RC transmitter (RadioMaster, FrSky, Jumper, ELRS, generic HID). On Linux, grant non-root
access to the device once:

```bash
sudo bash setup_udev.sh   # writes udev rules + adds you to the plugdev group (re-login after)
```

Then open the settings panel (**F1**) → scan & connect the device, assign axes (Roll / Pitch /
Throttle / Yaw) and the Arm / Mode switches with listen-mode auto-detect, set inversion / dead-zones,
and run calibration. Each transmitter's configuration is saved by device name; the last-used device is
remembered per mode/slot.

## Settings panel (F1)

- **Flight Mode** — FPV / Drone (parameters swap automatically).
- **Physics** — mass, max thrust, drag Cd, frontal area, drone size, collision radius.
- **Rates & Expo** — per-axis rate + expo (stored per mode).
- **Controller Gains** — cascaded PID (position / velocity / altitude), per mode.
- **RC / Buttons** — channel mapping, calibration, Arm / Mode switch assignment (Toggle / Level).
- **Audio** — engine / SFX / music volume sliders + mute.

All settings persist as JSON under `~/.config/mindcloud-fly/` (`scenes.json`, `drone.json`,
`audio.json`, `controllers/<name>.json`, `gate-paths/<scene>.json`).

## Audio

- **Engine sound** — throttle-driven pitch/volume (smoothed per-sample).
- **Gate-pass SFX** + **3-2-1-GO countdown** beeps (procedurally generated).
- **Background music** — playlists discovered from `native-bevy/assets/audio/bgm/<init|flight>/`; drop
  any `.ogg` / `.wav` in and it joins the rotation on the next launch.

## Project structure

```
native-bevy/
├── Cargo.toml              # mindcloud-fly-bevy crate
├── src/
│   ├── main.rs             # CLI, plugin wiring, window/app setup
│   ├── app_state.rs        # AppState / GameMode / WorldUp + SceneConfig
│   ├── splat_plugin.rs     # Bevy ↔ web-splat bridge (PLY load, GPU upload, render node)
│   ├── menu.rs / menu_ui.rs        # mode / scene / loading / exit screens
│   ├── placement.rs / placement_ui.rs  # orbit camera, spawn/heading editor
│   ├── flight.rs           # players, keyboard/HID → physics → camera, race state machine
│   ├── drone.rs            # quaternion physics + FPV/stabilized control laws + PID
│   ├── collision.rs        # octree spatial index + collision response
│   ├── gates.rs / spline.rs        # gate course logic + closed Catmull-Rom spline
│   ├── gate_plugin.rs / gate_editor.rs  # 3D gate frames + top-down path editor
│   ├── input.rs / input_plugin.rs  # HID RC-transmitter reader + Controller
│   ├── hud.rs              # flight HUD / OSD (egui)
│   ├── settings_ui.rs      # in-flight settings panel (egui)
│   └── persistence.rs      # JSON config under ~/.config/mindcloud-fly/
├── assets/
│   └── audio/              # engine loop + BGM (OGG)
└── web-splat/              # vendored Gaussian-splat renderer (KeKsBoTer/web-splat)
setup_udev.sh               # one-time HID device permissions (Linux)
asset/display/              # logo + demo media used by this README
scene/                      # your scene files (gitignored)
```

## Dependencies

| Crate | Purpose |
|-------|---------|
| [`bevy`](https://bevyengine.org) 0.18 | ECS, PBR rendering, windowing, audio |
| [`web_splats`](native-bevy/web-splat) (vendored) | Gaussian-splat rendering (GPU radix sort + rasterize) |
| [`bevy_egui`](https://github.com/vladbat00/bevy_egui) 0.39 | Menus / HUD / settings UI |
| `wgpu` 27 | GPU backend (shared by Bevy + web-splat) |
| `hidapi` | RC-transmitter input |
| `clap`, `serde`, `serde_json`, `bytemuck`, `cgmath`, `pollster` | CLI / serialization / math utilities |

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).

Copyright 2026 Manifold Tech Ltd.
