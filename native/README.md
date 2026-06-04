# MindCloud World Fly — Native

Rust + wgpu native split-screen FPV drone racing simulator with Gaussian Splatting rendering.

## Prerequisites

- **Rust** 1.95.0+ (`rustup default 1.95.0`)
- **Vulkan** driver (NVIDIA / AMD / Intel)
- **Linux** x86_64 (Ubuntu 20.04+)

## Build

```bash
ulimit -s 65536 && RUST_MIN_STACK=67108864 cargo build --release
```

> **⚠️ Build Note**: The default thread stack size (8MB) is too small for rustc/LLVM
> optimization passes on large crates (`ash`, `syn`, `x11rb-protocol`, etc.).
> Without `ulimit -s 65536` and `RUST_MIN_STACK=67108864`, release builds will
> crash with `SIGSEGV: invalid memory reference` (signal 11).
> Debug builds (`cargo build`) are unaffected.

## Run

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json WGPU_BACKEND=vulkan ./target/release/mindcloud-fly
```

For AMD/Intel GPUs, omit `VK_ICD_FILENAMES` or point to the appropriate ICD file.

## Controls

| Key | Action |
|-----|--------|
| WASD | FPV: pitch/roll |
| QE | Yaw |
| Space/Shift | Throttle up/down |
| M | Toggle FPV / Drone mode |
| R | Reset to spawn |
| G | Toggle gate course |
| Esc | Pause / Exit |
| Tab | Settings panel |

## Architecture

```
src/
├── main.rs          # App loop, event handling, dual viewport
├── drone.rs         # Quaternion-based drone physics (NED body frame)
├── collision.rs     # Octree collision detection on point cloud
├── gates.rs         # Gate course + lap timing
├── input.rs         # HID controller + keyboard input
├── hud.rs           # egui HUD overlay
├── osd.rs           # Fighter-jet style OSD (horizon/speed/alt)
├── scene_mesh.rs    # wgpu line/mesh renderer for debug/gates
├── placement.rs     # Spawn placement phase
├── spline.rs        # Catmull-Rom spline for gate paths
├── persistence.rs   # JSON config persistence
├── audio.rs         # Audio stubs
├── app_state.rs     # State machine (menu/loading/playing)
└── settings.rs      # Settings panel
```
