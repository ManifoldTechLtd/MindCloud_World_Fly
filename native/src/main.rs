mod app_state;
mod audio;
mod collision;
mod drone;
mod gates;
mod hud;
mod input;
mod menu_ui;
mod persistence;
mod settings;
mod spline;

use std::{fs::File, path::PathBuf, sync::Arc, time::{Duration, Instant}};

use app_state::GameMode;
use cgmath::Point3;
use clap::Parser;
use drone::{Drone, DroneInput};
use web_splats::{RenderConfig, SplattingArgs, WGPUContext, WindowContext};
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
    /// Input PLY/splat scene file (shows menu if not provided)
    input: Option<PathBuf>,
    #[arg(long, default_value_t = false)]
    no_vsync: bool,
    #[arg(long, default_value_t = false)]
    split: bool,
}

#[derive(Default)]
struct KeyState { w: bool, s: bool, a: bool, d: bool, up: bool, down: bool, left: bool, right: bool }
impl KeyState {
    fn to_drone_input(&self, armed: bool) -> DroneInput {
        DroneInput { roll: if self.right {1.0} else if self.left {-1.0} else {0.0},
            pitch: if self.up {1.0} else if self.down {-1.0} else {0.0},
            throttle: if self.w {0.5} else if self.s {-1.0} else {-0.2},
            yaw: if self.d {1.0} else if self.a {-1.0} else {0.0},
            armed, boost: false, rates: [1.0,1.0,1.0] }
    }
}
#[derive(Default)]
struct KeyStateP2 { i: bool, k: bool, j: bool, l: bool, t: bool, g: bool, f: bool, h: bool }
impl KeyStateP2 {
    fn to_drone_input(&self, armed: bool) -> DroneInput {
        DroneInput { roll: if self.l {1.0} else if self.j {-1.0} else {0.0},
            pitch: if self.i {1.0} else if self.k {-1.0} else {0.0},
            throttle: if self.t {0.5} else if self.g {-1.0} else {-0.2},
            yaw: if self.h {1.0} else if self.f {-1.0} else {0.0},
            armed, boost: false, rates: [1.0,1.0,1.0] }
    }
}

fn apply_drone_camera(state: &mut WindowContext, d: &Drone, dt: Duration) {
    if state.splatting_args.walltime < Duration::from_secs(5) { state.splatting_args.walltime += dt; }
    let (p, r) = d.camera_transform();
    state.splatting_args.camera.position = Point3::new(p.x, p.y, p.z);
    state.splatting_args.camera.rotation = r;
    state.splatting_args.camera.projection.resize(state.config.width, state.config.height);
    let aabb = state.pc.bbox(); state.splatting_args.camera.fit_near_far(aabb);
    state.splatting_args.camera.projection.znear = 0.1;
}

enum Phase {
    Menu { scene_files: Vec<PathBuf> },
    Loading { path: PathBuf, split: bool },
    Playing { drone1: Drone, drone2: Drone, keys1: KeyState, keys2: KeyStateP2,
              armed1: bool, armed2: bool, drone_mode: bool, split: bool,
              settings_open: bool, controller1: input::Controller },
}

#[pollster::main]
async fn main() {
    env_logger::init();
    let opt = Opt::parse();

    let event_loop = EventLoop::new().unwrap();
    let window = Arc::new(event_loop.create_window(
        Window::default_attributes()
            .with_inner_size(LogicalSize::new(1280, 720))
            .with_title("MindCloud World Fly"),
    ).unwrap());

    // Single shared wgpu context for the entire app lifetime
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let surface = instance.create_surface(window.clone()).unwrap();
    let wgpu_ctx = WGPUContext::new(&instance, Some(&surface)).await;

    let surface_caps = surface.get_capabilities(&wgpu_ctx.adapter);
    let surface_format = *surface_caps.formats.iter().find(|f| f.is_srgb()).unwrap_or(&surface_caps.formats[0]);
    let size = window.inner_size();
    let mut surf_config = wgpu::SurfaceConfiguration {
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT, format: surface_format,
        width: size.width.max(1), height: size.height.max(1),
        desired_maximum_frame_latency: 2,
        present_mode: if opt.no_vsync { wgpu::PresentMode::AutoNoVsync } else { wgpu::PresentMode::AutoVsync },
        alpha_mode: wgpu::CompositeAlphaMode::Auto, view_formats: vec![],
    };
    surface.configure(&wgpu_ctx.device, &surf_config);

    // egui for menu
    let egui_ctx = egui::Context::default();
    let mut egui_state = egui_winit::State::new(egui_ctx.clone(), egui::ViewportId::ROOT, &*window, None, None, None);
    let mut egui_rend = egui_wgpu::Renderer::new(&wgpu_ctx.device, surface_format, egui_wgpu::RendererOptions::default());

    // If CLI arg, skip menu
    let mut phase = if let Some(ref p) = opt.input {
        Phase::Loading { path: p.clone(), split: opt.split }
    } else {
        Phase::Menu { scene_files: menu_ui::scan_scene_files() }
    };

    let mut game_state: Option<WindowContext> = None;
    let mut last_time = Instant::now();
    let no_vsync = opt.no_vsync;

    #[allow(deprecated)]
    event_loop.run(move |event, target| {
        // Forward window events to the appropriate egui
        if let Event::WindowEvent { ref event, .. } = event {
            if let Some(ref mut gs) = game_state {
                if gs.ui_renderer.on_event(&gs.window, event) { return; }
            } else {
                let _ = egui_state.on_window_event(&*window, event);
            }
        }

        match event {
            Event::NewEvents(winit::event::StartCause::ResumeTimeReached { .. }) => { window.request_redraw(); }
            Event::WindowEvent { ref event, window_id } if window_id == window.id() => {
                match event {
                    WindowEvent::CloseRequested => target.exit(),
                    WindowEvent::Resized(new_size) => {
                        if new_size.width > 0 && new_size.height > 0 {
                            if let Some(ref mut gs) = game_state {
                                gs.resize(*new_size, None);
                            } else {
                                surf_config.width = new_size.width;
                                surf_config.height = new_size.height;
                                surface.configure(&wgpu_ctx.device, &surf_config);
                            }
                        }
                    }
                    WindowEvent::KeyboardInput { event: key_event, .. } => {
                        if let PhysicalKey::Code(key) = key_event.physical_key {
                            let pressed = key_event.state == ElementState::Pressed;
                            if key == KeyCode::Escape { target.exit(); }
                            if let Phase::Playing { ref mut keys1, ref mut keys2, ref mut armed1, ref mut armed2,
                                ref mut drone_mode, ref mut drone1, ref mut settings_open, split, .. } = phase {
                                if key == KeyCode::F1 && key_event.state == ElementState::Released { *settings_open = !*settings_open; }
                                if !split && key == KeyCode::KeyM && key_event.state == ElementState::Released {
                                    *drone_mode = !*drone_mode;
                                    if *drone_mode { drone1.reset(0.0,0.0,0.0); *armed1 = true; } else { *armed1 = false; }
                                }
                                if *drone_mode {
                                    match key {
                                        KeyCode::KeyW => keys1.w=pressed, KeyCode::KeyS => keys1.s=pressed,
                                        KeyCode::KeyA => keys1.a=pressed, KeyCode::KeyD => keys1.d=pressed,
                                        KeyCode::ArrowUp => keys1.up=pressed, KeyCode::ArrowDown => keys1.down=pressed,
                                        KeyCode::ArrowLeft => keys1.left=pressed, KeyCode::ArrowRight => keys1.right=pressed,
                                        KeyCode::Space => { if pressed { *armed1=!*armed1; } }
                                        KeyCode::KeyR => { if pressed { drone1.reset(0.0,2.0,0.0); } }
                                        _ => {}
                                    }
                                }
                                if split {
                                    match key {
                                        KeyCode::KeyT => keys2.t=pressed, KeyCode::KeyG => keys2.g=pressed,
                                        KeyCode::KeyF => keys2.f=pressed, KeyCode::KeyH => keys2.h=pressed,
                                        KeyCode::KeyI => keys2.i=pressed, KeyCode::KeyK => keys2.k=pressed,
                                        KeyCode::KeyJ => keys2.j=pressed, KeyCode::KeyL => keys2.l=pressed,
                                        KeyCode::Enter => { if pressed { *armed2=!*armed2; } }
                                        KeyCode::Backspace => { if pressed { drone1.reset(2.0,2.0,0.0); } }
                                        _ => {}
                                    }
                                }
                            }
                        }
                    }
                    WindowEvent::MouseWheel { delta, .. } => {
                        if let Some(ref mut gs) = game_state {
                            if let Phase::Playing { drone_mode, .. } = &phase {
                                if !drone_mode { match delta {
                                    winit::event::MouseScrollDelta::LineDelta(_,dy) => gs.controller.process_scroll(*dy),
                                    winit::event::MouseScrollDelta::PixelDelta(p) => gs.controller.process_scroll(p.y as f32/100.),
                                }}
                            }
                        }
                    }
                    WindowEvent::MouseInput { state: bs, button, .. } => {
                        if let Some(ref mut gs) = game_state {
                            if let Phase::Playing { drone_mode, .. } = &phase {
                                if !drone_mode { match button {
                                    winit::event::MouseButton::Left => gs.controller.left_mouse_pressed = *bs==ElementState::Pressed,
                                    winit::event::MouseButton::Right => gs.controller.right_mouse_pressed = *bs==ElementState::Pressed,
                                    _ => {}
                                }}
                            }
                        }
                    }
                    WindowEvent::RedrawRequested => {
                        let now = Instant::now();
                        let dt = now - last_time; last_time = now;
                        let dt_s = dt.as_secs_f32();

                        match &mut phase {
                            Phase::Menu { ref scene_files } => {
                                target.set_control_flow(ControlFlow::wait_duration(Duration::from_millis(16)));
                                let raw = egui_state.take_egui_input(&*window);
                                egui_ctx.begin_pass(raw);
                                let transition = menu_ui::draw_scene_select(&egui_ctx, scene_files);
                                let output = egui_ctx.end_pass();
                                egui_state.handle_platform_output(&*window, output.platform_output.clone());

                                if let wgpu::CurrentSurfaceTexture::Success(frame) | wgpu::CurrentSurfaceTexture::Suboptimal(frame) = surface.get_current_texture() {
                                    let view = frame.texture.create_view(&Default::default());
                                    let mut enc = wgpu_ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
                                    let screen = egui_wgpu::ScreenDescriptor { size_in_pixels: [surf_config.width, surf_config.height], pixels_per_point: window.scale_factor() as f32 };
                                    let tris = egui_ctx.tessellate(output.shapes, output.pixels_per_point);
                                    for (id, d) in &output.textures_delta.set { egui_rend.update_texture(&wgpu_ctx.device, &wgpu_ctx.queue, *id, d); }
                                    egui_rend.update_buffers(&wgpu_ctx.device, &wgpu_ctx.queue, &mut enc, &tris, &screen);
                                    let mut rp = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                                        label: None, color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                            view: &view, resolve_target: None,
                                            ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color{r:0.05,g:0.07,b:0.11,a:1.0}), store: wgpu::StoreOp::Store },
                                            depth_slice: None })], ..Default::default()
                                    }).forget_lifetime();
                                    egui_rend.render(&mut rp, &tris, &screen);
                                    drop(rp);
                                    for id in &output.textures_delta.free { egui_rend.free_texture(id); }
                                    wgpu_ctx.queue.submit([enc.finish()]);
                                    frame.present();
                                }

                                match transition {
                                    app_state::StateTransition::ToLoading(path, mode) => {
                                        let split = mode == GameMode::SplitScreen;
                                        phase = Phase::Loading { path, split };
                                    }
                                    _ => {}
                                }
                                window.request_redraw();
                            }
                            Phase::Loading { ref path, split } => {
                                let split_v = *split;
                                let p = path.clone();
                                window.set_title("MindCloud Fly — Loading...");

                                let data_file = File::open(&p).unwrap_or_else(|e| {
                                    eprintln!("Failed: {:?}: {}", p, e); std::process::exit(1);
                                });
                                let rcfg = RenderConfig { no_vsync, hdr: false };

                                // Use same window but create fresh WindowContext (new surface internally)
                                let gs = pollster::block_on(
                                    WindowContext::new_from_arc(window.clone(), data_file, &rcfg)
                                ).unwrap();
                                let mut gs = gs;
                                gs.pointcloud_file_path = Some(p);
                                gs.ui_visible = !split_v;

                                let mut d1 = Drone::new(); d1.reset(0.0,2.0,0.0);
                                let mut d2 = Drone::new(); d2.reset(2.0,2.0,0.0);
                                let (mut a1, mut a2, mut dm) = (false, false, !split_v);
                                if split_v { dm=true; a1=true; a2=true; }

                                game_state = Some(gs);
                                phase = Phase::Playing {
                                    drone1:d1, drone2:d2, keys1:KeyState::default(), keys2:KeyStateP2::default(),
                                    armed1:a1, armed2:a2, drone_mode:dm, split:split_v,
                                    settings_open:false, controller1:input::Controller::new(),
                                };
                                window.set_title(if split_v {"MindCloud Fly [SPLIT]"} else {"MindCloud World Fly"});
                                window.request_redraw();
                            }
                            Phase::Playing { ref mut drone1, ref mut drone2, ref mut keys1, ref mut keys2,
                                ref mut armed1, ref mut armed2, ref mut drone_mode, split,
                                ref mut settings_open, ref mut controller1 } =>
                            {
                                let gs = game_state.as_mut().unwrap();
                                gs.fps = (1.0/dt_s)*0.05 + gs.fps*0.95;
                                let split = *split;
                                if !no_vsync { target.set_control_flow(ControlFlow::wait_duration(Duration::from_millis(1))); }

                                if split {
                                    let i1=keys1.to_drone_input(*armed1); drone1.update(dt_s,&i1);
                                    let i2=keys2.to_drone_input(*armed2); drone2.update(dt_s,&i2);
                                    if gs.splatting_args.walltime<Duration::from_secs(5){gs.splatting_args.walltime+=dt;}
                                    let mut at=gs.splatting_args;
                                    {let(p,r)=drone1.camera_transform();at.camera.position=Point3::new(p.x,p.y,p.z);at.camera.rotation=r;
                                     at.camera.projection.resize(gs.config.width,gs.config.height/2);let a=gs.pc.bbox();at.camera.fit_near_far(a);at.camera.projection.znear=0.1;}
                                    let mut ab=gs.splatting_args;
                                    {let(p,r)=drone2.camera_transform();ab.camera.position=Point3::new(p.x,p.y,p.z);ab.camera.rotation=r;
                                     ab.camera.projection.resize(gs.config.width,gs.config.height/2);let a=gs.pc.bbox();ab.camera.fit_near_far(a);ab.camera.projection.znear=0.1;}
                                    gs.ui_renderer.begin_frame(&gs.window);
                                    {let ctx=gs.ui_renderer.winit.egui_ctx().clone();let s=ctx.screen_rect();let hh=s.height()/2.0;let w=s.width();
                                     hud::draw_hud(&ctx,drone1,None,*armed1,gs.fps,Some("P1"),Some(egui::Rect::from_min_size(s.min,egui::Vec2::new(w,hh))));
                                     hud::draw_hud(&ctx,drone2,None,*armed2,gs.fps,Some("P2"),Some(egui::Rect::from_min_size(egui::Pos2::new(s.min.x,s.min.y+hh),egui::Vec2::new(w,hh))));
                                     settings::draw_settings(&ctx,settings_open,drone1,controller1,armed1);}
                                    let eo=gs.ui_renderer.end_frame(&gs.window);
                                    match gs.render_split(at,ab,Some(eo)){Ok(_)=>{}Err(_)=>gs.resize(gs.window.inner_size(),None)}
                                    gs.window.set_title(&format!("MindCloud Fly [SPLIT] — {:.0} FPS | P1:{} P2:{}",gs.fps,if *armed1{"ARMED"}else{"OFF"},if *armed2{"ARMED"}else{"OFF"}));
                                } else if *drone_mode {
                                    let i1=keys1.to_drone_input(*armed1); drone1.update(dt_s,&i1);
                                    apply_drone_camera(gs,drone1,dt);
                                    let(_,sh)=gs.ui(); match gs.render(true,gs.ui_visible.then_some(sh)){Ok(_)=>{}Err(_)=>gs.resize(gs.window.inner_size(),None)}
                                    gs.window.set_title(&format!("MindCloud Fly [DRONE] — {:.0} FPS | Armed: {}",gs.fps,armed1));
                                } else {
                                    gs.update(dt); let(_,sh)=gs.ui();
                                    match gs.render(true,gs.ui_visible.then_some(sh)){Ok(_)=>{}Err(_)=>gs.resize(gs.window.inner_size(),None)}
                                    gs.window.set_title(&format!("MindCloud Fly [ORBIT] — {:.0} FPS",gs.fps));
                                }
                                if no_vsync { gs.window.request_redraw(); } else { window.request_redraw(); }
                            }
                        }
                    }
                    _ => {}
                }
            }
            Event::DeviceEvent { event: DeviceEvent::MouseMotion { delta }, .. } => {
                if let Some(ref mut gs) = game_state {
                    if let Phase::Playing { drone_mode, .. } = &phase {
                        if !drone_mode { gs.controller.process_mouse(delta.0 as f32, delta.1 as f32); }
                    }
                }
            }
            _ => {}
        }
    }).unwrap();
}
