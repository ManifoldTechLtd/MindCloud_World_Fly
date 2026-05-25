mod audio;
mod collision;
mod drone;
mod gates;
mod hud;
mod input;
mod persistence;
mod spline;

use std::{fs::File, path::PathBuf, time::{Duration, Instant}};

use cgmath::Point3;
use clap::Parser;
use drone::{Drone, DroneInput};
use web_splats::{RenderConfig, SplattingArgs, WindowContext};
use winit::{
    dpi::LogicalSize,
    event::{DeviceEvent, ElementState, Event, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::Window,
};

#[derive(Debug, Parser)]
#[command(author, version, about = "MindCloud World Fly — native FPV drone racing simulator")]
struct Opt {
    /// Input PLY/splat scene file
    input: PathBuf,

    /// Disable V-sync for max framerate
    #[arg(long, default_value_t = false)]
    no_vsync: bool,

    /// Enable split-screen two-player mode
    #[arg(long, default_value_t = false)]
    split: bool,
}

/// Per-player keyboard state
#[derive(Default)]
struct KeyState {
    w: bool, s: bool, a: bool, d: bool,
    up: bool, down: bool, left: bool, right: bool,
}

impl KeyState {
    fn to_drone_input(&self, armed: bool) -> DroneInput {
        DroneInput {
            roll: if self.right { 1.0 } else if self.left { -1.0 } else { 0.0 },
            pitch: if self.up { 1.0 } else if self.down { -1.0 } else { 0.0 },
            throttle: if self.w { 0.5 } else if self.s { -1.0 } else { -0.2 },
            yaw: if self.d { 1.0 } else if self.a { -1.0 } else { 0.0 },
            armed,
            boost: false,
            rates: [1.0, 1.0, 1.0],
        }
    }
}

/// P2 uses IJKL + TFGH layout (so both players can fly on one keyboard)
#[derive(Default)]
struct KeyStateP2 {
    i: bool, k: bool, j: bool, l: bool,
    t: bool, g: bool, f: bool, h: bool,
}

impl KeyStateP2 {
    fn to_drone_input(&self, armed: bool) -> DroneInput {
        DroneInput {
            roll: if self.l { 1.0 } else if self.j { -1.0 } else { 0.0 },
            pitch: if self.i { 1.0 } else if self.k { -1.0 } else { 0.0 },
            throttle: if self.t { 0.5 } else if self.g { -1.0 } else { -0.2 },
            yaw: if self.h { 1.0 } else if self.f { -1.0 } else { 0.0 },
            armed,
            boost: false,
            rates: [1.0, 1.0, 1.0],
        }
    }
}

/// Update camera from drone state.
/// Mirrors what web-splat's update() does but with drone position/orientation.
fn apply_drone_camera(
    state: &mut WindowContext,
    fly_drone: &Drone,
    dt: std::time::Duration,
) {
    // Increment walltime (web-splat uses this for fade-in effects)
    if state.splatting_args.walltime < std::time::Duration::from_secs(5) {
        state.splatting_args.walltime += dt;
    }

    let (cam_pos, cam_orient) = fly_drone.camera_transform();
    state.splatting_args.camera.position = Point3::new(cam_pos.x, cam_pos.y, cam_pos.z);
    state.splatting_args.camera.rotation = cam_orient;

    // Resize projection to match current window (critical — without this, viewport mismatch)
    state.splatting_args.camera.projection.resize(state.config.width, state.config.height);

    // fit_near_far then override znear for inside-scene flying
    let aabb = state.pc.bbox();
    state.splatting_args.camera.fit_near_far(aabb);
    state.splatting_args.camera.projection.znear = 0.1;
}

#[pollster::main]
async fn main() {
    env_logger::init();
    let opt = Opt::parse();

    let data_file = File::open(&opt.input).unwrap_or_else(|e| {
        eprintln!("Failed to open {:?}: {}", opt.input, e);
        std::process::exit(1);
    });

    let config = RenderConfig { no_vsync: opt.no_vsync, hdr: false };
    let split = opt.split;

    let event_loop = EventLoop::new().unwrap();
    let window = event_loop
        .create_window(
            Window::default_attributes()
                .with_inner_size(LogicalSize::new(1280, if split { 960 } else { 720 }))
                .with_title(if split { "MindCloud Fly [SPLIT]" } else { "MindCloud World Fly" }),
        )
        .unwrap();

    let mut state = WindowContext::new(window, data_file, &config).await.unwrap();
    state.pointcloud_file_path = Some(opt.input);
    state.ui_visible = !split; // hide web-splat UI in split mode

    // ---- Player 1 ----
    let mut drone1 = Drone::new();
    drone1.reset(0.0, 2.0, 0.0);
    let mut keys1 = KeyState::default();
    let mut armed1 = false;
    let mut drone_mode = !split; // in split mode, always drone; in single, toggle with M

    // ---- Player 2 (split only) ----
    let mut drone2 = Drone::new();
    drone2.reset(2.0, 2.0, 0.0); // offset slightly from P1
    let mut keys2 = KeyStateP2::default();
    let mut armed2 = false;

    if split {
        drone_mode = true;
        armed1 = true;
        armed2 = true;
        log::info!("Split-screen mode: P1=WASD+Arrows, P2=TFGH+IJKL");
    }

    let mut last_time = Instant::now();
    let min_wait = state.window
        .current_monitor()
        .map(|m| Duration::from_millis(1000000 / m.refresh_rate_millihertz().unwrap_or(60_000) as u64))
        .unwrap_or(Duration::from_millis(17));

    #[allow(deprecated)]
    event_loop.run(move |event, target| {
        match event {
            Event::NewEvents(winit::event::StartCause::ResumeTimeReached { .. }) => {
                state.window.request_redraw();
            }
            Event::WindowEvent { ref event, window_id }
                if window_id == state.window.id()
                    && !state.ui_renderer.on_event(&state.window, event) =>
            {
                match event {
                    WindowEvent::Resized(s) => state.resize(*s, None),
                    WindowEvent::ScaleFactorChanged { scale_factor, .. } => {
                        state.scale_factor = *scale_factor as f32;
                    }
                    WindowEvent::CloseRequested => target.exit(),
                    WindowEvent::ModifiersChanged(m) => {
                        state.controller.alt_pressed = m.state().alt_key();
                    }
                    WindowEvent::KeyboardInput { event: key_event, .. } => {
                        if let PhysicalKey::Code(key) = key_event.physical_key {
                            let pressed = key_event.state == ElementState::Pressed;

                            // ---- Global keys ----
                            if key == KeyCode::Escape { target.exit(); }

                            if !split {
                                // Single-player: M toggles drone mode
                                if key == KeyCode::KeyM && key_event.state == ElementState::Released {
                                    drone_mode = !drone_mode;
                                    if drone_mode {
                                        drone1.reset(0.0, 0.0, 0.0);
                                        armed1 = true;
                                    } else {
                                        armed1 = false;
                                    }
                                }
                            }

                            // ---- P1 keys (WASD + arrows + Space + R) ----
                            if drone_mode {
                                match key {
                                    KeyCode::KeyW => keys1.w = pressed,
                                    KeyCode::KeyS => keys1.s = pressed,
                                    KeyCode::KeyA => keys1.a = pressed,
                                    KeyCode::KeyD => keys1.d = pressed,
                                    KeyCode::ArrowUp => keys1.up = pressed,
                                    KeyCode::ArrowDown => keys1.down = pressed,
                                    KeyCode::ArrowLeft => keys1.left = pressed,
                                    KeyCode::ArrowRight => keys1.right = pressed,
                                    KeyCode::Space => {
                                        if pressed { armed1 = !armed1; }
                                    }
                                    KeyCode::KeyR => {
                                        if pressed { drone1.reset(0.0, 2.0, 0.0); }
                                    }
                                    _ => {}
                                }
                            } else {
                                state.controller.process_keyboard(key, pressed);
                            }

                            // ---- P2 keys (TFGH + IJKL + Enter + Backspace) ----
                            if split {
                                match key {
                                    KeyCode::KeyT => keys2.t = pressed,
                                    KeyCode::KeyG => keys2.g = pressed,
                                    KeyCode::KeyF => keys2.f = pressed,
                                    KeyCode::KeyH => keys2.h = pressed,
                                    KeyCode::KeyI => keys2.i = pressed,
                                    KeyCode::KeyK => keys2.k = pressed,
                                    KeyCode::KeyJ => keys2.j = pressed,
                                    KeyCode::KeyL => keys2.l = pressed,
                                    KeyCode::Enter => {
                                        if pressed { armed2 = !armed2; }
                                    }
                                    KeyCode::Backspace => {
                                        if pressed { drone2.reset(2.0, 2.0, 0.0); }
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                    WindowEvent::MouseWheel { delta, .. } => {
                        if !drone_mode {
                            match delta {
                                winit::event::MouseScrollDelta::LineDelta(_, dy) => state.controller.process_scroll(*dy),
                                winit::event::MouseScrollDelta::PixelDelta(p) => state.controller.process_scroll(p.y as f32 / 100.),
                            }
                        }
                    }
                    WindowEvent::MouseInput { state: bs, button, .. } => {
                        if !drone_mode {
                            match button {
                                winit::event::MouseButton::Left => state.controller.left_mouse_pressed = *bs == ElementState::Pressed,
                                winit::event::MouseButton::Right => state.controller.right_mouse_pressed = *bs == ElementState::Pressed,
                                _ => {}
                            }
                        }
                    }
                    WindowEvent::RedrawRequested => {
                        if !config.no_vsync {
                            target.set_control_flow(ControlFlow::wait_duration(min_wait));
                        }
                        let now = Instant::now();
                        let dt = now - last_time;
                        last_time = now;
                        let dt_s = dt.as_secs_f32();

                        state.fps = (1.0 / dt_s) * 0.05 + state.fps * 0.95;

                        if split {
                            // ---- Split-screen: update both drones ----
                            let input1 = keys1.to_drone_input(armed1);
                            drone1.update(dt_s, &input1);

                            let input2 = keys2.to_drone_input(armed2);
                            drone2.update(dt_s, &input2);

                            // Update walltime for both
                            if state.splatting_args.walltime < std::time::Duration::from_secs(5) {
                                state.splatting_args.walltime += dt;
                            }

                            // Build camera args for each player
                            let mut args_top = state.splatting_args;
                            {
                                let (p, r) = drone1.camera_transform();
                                args_top.camera.position = Point3::new(p.x, p.y, p.z);
                                args_top.camera.rotation = r;
                                args_top.camera.projection.resize(state.config.width, state.config.height / 2);
                                let aabb = state.pc.bbox();
                                args_top.camera.fit_near_far(aabb);
                                args_top.camera.projection.znear = 0.1;
                            }

                            let mut args_bottom = state.splatting_args;
                            {
                                let (p, r) = drone2.camera_transform();
                                args_bottom.camera.position = Point3::new(p.x, p.y, p.z);
                                args_bottom.camera.rotation = r;
                                args_bottom.camera.projection.resize(state.config.width, state.config.height / 2);
                                let aabb = state.pc.bbox();
                                args_bottom.camera.fit_near_far(aabb);
                                args_bottom.camera.projection.znear = 0.1;
                            }

                            match state.render_split(args_top, args_bottom) {
                                Ok(_) => {}
                                Err(wgpu::CurrentSurfaceTexture::Suboptimal(_) | wgpu::CurrentSurfaceTexture::Lost) => {
                                    state.resize(state.window.inner_size(), None);
                                }
                                Err(e) => eprintln!("render error: {:?}", e),
                            }

                            state.window.set_title(&format!(
                                "MindCloud Fly [SPLIT] — {:.0} FPS | P1:{} P2:{}",
                                state.fps,
                                if armed1 { "ARMED" } else { "OFF" },
                                if armed2 { "ARMED" } else { "OFF" },
                            ));
                        } else if drone_mode {
                            // ---- Single player drone ----
                            let input1 = keys1.to_drone_input(armed1);
                            drone1.update(dt_s, &input1);
                            apply_drone_camera(&mut state, &drone1, dt);

                            let (_redraw_ui, shapes) = state.ui();
                            match state.render(true, state.ui_visible.then_some(shapes)) {
                                Ok(_) => {}
                                Err(wgpu::CurrentSurfaceTexture::Suboptimal(_) | wgpu::CurrentSurfaceTexture::Lost) => {
                                    state.resize(state.window.inner_size(), None);
                                }
                                Err(e) => eprintln!("render error: {:?}", e),
                            }

                            state.window.set_title(&format!(
                                "MindCloud Fly [DRONE] — {:.0} FPS | Armed: {}",
                                state.fps, armed1
                            ));
                        } else {
                            // ---- Orbit camera ----
                            state.update(dt);
                            let (_redraw_ui, shapes) = state.ui();
                            match state.render(true, state.ui_visible.then_some(shapes)) {
                                Ok(_) => {}
                                Err(wgpu::CurrentSurfaceTexture::Suboptimal(_) | wgpu::CurrentSurfaceTexture::Lost) => {
                                    state.resize(state.window.inner_size(), None);
                                }
                                Err(e) => eprintln!("render error: {:?}", e),
                            }
                            state.window.set_title(&format!(
                                "MindCloud Fly [ORBIT] — {:.0} FPS | Press M for drone",
                                state.fps
                            ));
                        }

                        if config.no_vsync { state.window.request_redraw(); }
                    }
                    _ => {}
                }
            }
            Event::DeviceEvent { event: DeviceEvent::MouseMotion { delta }, .. } => {
                if !drone_mode { state.controller.process_mouse(delta.0 as f32, delta.1 as f32); }
            }
            _ => {}
        }
    }).unwrap();
}
