mod collision;
mod drone;
mod input;

use std::{fs::File, io::{Read, Seek}, path::PathBuf, sync::Arc, time::{Duration, Instant}};

use cgmath::{Deg, Point3, Quaternion, Rad, Vector2};
use clap::Parser;
use drone::{Drone, DroneInput};
use web_splats::{
    PerspectiveCamera, PerspectiveProjection, RenderConfig, WindowContext,
};
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
}

/// Keyboard state for drone control
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

#[pollster::main]
async fn main() {
    env_logger::init();
    let opt = Opt::parse();

    let data_file = File::open(&opt.input).unwrap_or_else(|e| {
        eprintln!("Failed to open {:?}: {}", opt.input, e);
        std::process::exit(1);
    });

    let config = RenderConfig {
        no_vsync: opt.no_vsync,
        hdr: false,
    };

    // Create event loop and window
    let event_loop = EventLoop::new().unwrap();
    let window = event_loop
        .create_window(
            Window::default_attributes()
                .with_inner_size(LogicalSize::new(1280, 720))
                .with_title("MindCloud World Fly"),
        )
        .unwrap();

    // Create WindowContext (handles all wgpu + renderer setup internally)
    let mut state = WindowContext::new(window, data_file, &config)
        .await
        .unwrap();
    state.pointcloud_file_path = Some(opt.input);
    state.ui_visible = true;

    // Init drone at PLY origin
    let mut fly_drone = Drone::new();
    fly_drone.reset(0.0, 2.0, 0.0);

    let mut keys = KeyState::default();
    let mut last_time = Instant::now();
    let mut drone_mode = false; // false = web-splat orbit camera, true = drone FPV
    let mut armed = false;

    let min_wait = state.window
        .current_monitor()
        .map(|m| {
            let hz = m.refresh_rate_millihertz().unwrap_or(60_000);
            Duration::from_millis(1000000 / hz as u64)
        })
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
                    WindowEvent::Resized(physical_size) => {
                        state.resize(*physical_size, None);
                    }
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

                            // Toggle drone mode with M key
                            if key == KeyCode::KeyM && key_event.state == ElementState::Released {
                                drone_mode = !drone_mode;
                                if drone_mode {
                                    // Enter drone mode: start at PLY origin
                                    fly_drone.reset(0.0, 0.0, 0.0);
                                    armed = true;
                                    log::info!("Drone mode ON — armed at origin");
                                } else {
                                    armed = false;
                                    log::info!("Drone mode OFF — orbit camera");
                                }
                            }

                            if drone_mode {
                                // Drone controls
                                match key {
                                    KeyCode::KeyW => keys.w = pressed,
                                    KeyCode::KeyS => keys.s = pressed,
                                    KeyCode::KeyA => keys.a = pressed,
                                    KeyCode::KeyD => keys.d = pressed,
                                    KeyCode::ArrowUp => keys.up = pressed,
                                    KeyCode::ArrowDown => keys.down = pressed,
                                    KeyCode::ArrowLeft => keys.left = pressed,
                                    KeyCode::ArrowRight => keys.right = pressed,
                                    KeyCode::Space => {
                                        if pressed {
                                            armed = !armed;
                                            log::info!("Armed: {}", armed);
                                        }
                                    }
                                    KeyCode::KeyR => {
                                        if pressed {
                                            fly_drone.reset(0.0, 2.0, 0.0);
                                        }
                                    }
                                    _ => {}
                                }
                            } else {
                                // Pass to web-splat's orbit controller
                                state.controller.process_keyboard(key, pressed);
                            }

                            if key == KeyCode::Escape {
                                target.exit();
                            }
                        }
                    }
                    WindowEvent::MouseWheel { delta, .. } => {
                        if !drone_mode {
                            match delta {
                                winit::event::MouseScrollDelta::LineDelta(_, dy) => {
                                    state.controller.process_scroll(*dy);
                                }
                                winit::event::MouseScrollDelta::PixelDelta(p) => {
                                    state.controller.process_scroll(p.y as f32 / 100.);
                                }
                            }
                        }
                    }
                    WindowEvent::MouseInput { state: button_state, button, .. } => {
                        if !drone_mode {
                            match button {
                                winit::event::MouseButton::Left => {
                                    state.controller.left_mouse_pressed =
                                        *button_state == ElementState::Pressed;
                                }
                                winit::event::MouseButton::Right => {
                                    state.controller.right_mouse_pressed =
                                        *button_state == ElementState::Pressed;
                                }
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

                        if drone_mode {
                            // Drone physics drives camera
                            let input = keys.to_drone_input(armed);
                            fly_drone.update(dt.as_secs_f32(), &input);

                            let (cam_pos, cam_orient) = fly_drone.camera_transform();
                            state.splatting_args.camera.position =
                                Point3::new(cam_pos.x, cam_pos.y, cam_pos.z);
                            state.splatting_args.camera.rotation = cam_orient;

                            // Fit near/far for scene visibility
                            let aabb = state.pc.bbox();
                            state.splatting_args.camera.fit_near_far(aabb);
                            // Override znear to be small (we're inside the scene)
                            state.splatting_args.camera.projection.znear = 0.1;

                            // Update title
                            state.fps = (1. / dt.as_secs_f32()) * 0.05 + state.fps * 0.95;
                            state.window.set_title(&format!(
                                "MindCloud Fly [DRONE] — {:.0} FPS | Armed: {}",
                                state.fps, armed
                            ));
                        } else {
                            // web-splat's orbit camera controller
                            state.update(dt);
                            state.fps = (1. / dt.as_secs_f32()) * 0.05 + state.fps * 0.95;
                            state.window.set_title(&format!(
                                "MindCloud Fly [ORBIT] — {:.0} FPS | Press M for drone mode",
                                state.fps
                            ));
                        }

                        let (redraw_ui, shapes) = state.ui();
                        match state.render(true, state.ui_visible.then_some(shapes)) {
                            Ok(_) => {}
                            Err(wgpu::CurrentSurfaceTexture::Suboptimal(_)) => {
                                state.resize(state.window.inner_size(), None);
                            }
                            Err(wgpu::CurrentSurfaceTexture::Lost) => {
                                state.resize(state.window.inner_size(), None);
                            }
                            Err(e) => eprintln!("render error: {:?}", e),
                        }

                        if config.no_vsync {
                            state.window.request_redraw();
                        }
                    }
                    _ => {}
                }
            }
            Event::DeviceEvent {
                event: DeviceEvent::MouseMotion { delta },
                ..
            } => {
                if !drone_mode {
                    state.controller.process_mouse(delta.0 as f32, delta.1 as f32);
                }
            }
            _ => {}
        }
    }).unwrap();
}
