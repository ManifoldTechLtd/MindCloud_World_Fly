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

use app_state::{GameMode, StateTransition};
use egui::Color32;
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
#[command(author, version, about = "MindCloud World Fly")]
struct Opt {
    input: Option<PathBuf>,
    #[arg(long)] no_vsync: bool,
    #[arg(long)] split: bool,
}

#[derive(Default)]
struct KeyState { w:bool,s:bool,a:bool,d:bool,up:bool,down:bool,left:bool,right:bool }
impl KeyState {
    fn to_input(&self, armed: bool) -> DroneInput {
        DroneInput { roll:if self.right{1.}else if self.left{-1.}else{0.}, pitch:if self.up{1.}else if self.down{-1.}else{0.},
            throttle:if self.w{0.5}else if self.s{-1.}else{-0.2}, yaw:if self.d{1.}else if self.a{-1.}else{0.},
            armed, boost:false, rates:[1.,1.,1.] }
    }
}
#[derive(Default)]
struct KeyStateP2 { i:bool,k:bool,j:bool,l:bool,t:bool,g:bool,f:bool,h:bool }
impl KeyStateP2 {
    fn to_input(&self, armed: bool) -> DroneInput {
        DroneInput { roll:if self.l{1.}else if self.j{-1.}else{0.}, pitch:if self.i{1.}else if self.k{-1.}else{0.},
            throttle:if self.t{0.5}else if self.g{-1.}else{-0.2}, yaw:if self.h{1.}else if self.f{-1.}else{0.},
            armed, boost:false, rates:[1.,1.,1.] }
    }
}

fn apply_drone_cam(s: &mut WindowContext, d: &Drone, dt: Duration) {
    if s.splatting_args.walltime < Duration::from_secs(5) { s.splatting_args.walltime += dt; }
    let (p,r) = d.camera_transform();
    s.splatting_args.camera.position = Point3::new(p.x,p.y,p.z);
    s.splatting_args.camera.rotation = r;
    s.splatting_args.camera.projection.resize(s.config.width, s.config.height);
    let a = s.pc.bbox(); s.splatting_args.camera.fit_near_far(a); s.splatting_args.camera.projection.znear = 0.1;
}

enum Phase {
    ModeSelect,
    SceneSelect { mode: GameMode, scene_files: Vec<PathBuf> },
    Loading { path: PathBuf, mode: GameMode },
    Playing { drone1:Drone, drone2:Drone, keys1:KeyState, keys2:KeyStateP2,
              armed1:bool, armed2:bool, drone_mode:bool, split:bool,
              settings_open:bool, controller1:input::Controller },
}

#[pollster::main]
async fn main() {
    env_logger::init();
    let opt = Opt::parse();

    let event_loop = EventLoop::new().unwrap();
    let window = Arc::new(event_loop.create_window(
        Window::default_attributes().with_inner_size(LogicalSize::new(1280,720)).with_title("MindCloud World Fly"),
    ).unwrap());

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let surface = instance.create_surface(window.clone()).unwrap();
    let wgpu_ctx = WGPUContext::new(&instance, Some(&surface)).await;
    let surface_caps = surface.get_capabilities(&wgpu_ctx.adapter);
    let surface_format = *surface_caps.formats.iter().find(|f| f.is_srgb()).unwrap_or(&surface_caps.formats[0]);
    let size = window.inner_size();
    let mut surf_cfg = wgpu::SurfaceConfiguration {
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT, format: surface_format,
        width: size.width.max(1), height: size.height.max(1), desired_maximum_frame_latency: 2,
        present_mode: if opt.no_vsync {wgpu::PresentMode::AutoNoVsync} else {wgpu::PresentMode::AutoVsync},
        alpha_mode: wgpu::CompositeAlphaMode::Auto, view_formats: vec![] };
    surface.configure(&wgpu_ctx.device, &surf_cfg);

    let egui_ctx = egui::Context::default();
    let mut egui_st = egui_winit::State::new(egui_ctx.clone(), egui::ViewportId::ROOT, &*window, None, None, None);
    let mut egui_rn = egui_wgpu::Renderer::new(&wgpu_ctx.device, surface_format, egui_wgpu::RendererOptions::default());

    let mut phase = if let Some(ref p) = opt.input {
        let m = if opt.split {GameMode::SplitScreen} else {GameMode::SinglePlayer};
        Phase::Loading { path: p.clone(), mode: m }
    } else {
        Phase::ModeSelect
    };

    let mut game_state: Option<WindowContext> = None;
    let mut last_time = Instant::now();
    let no_vsync = opt.no_vsync;
    let mut show_exit_dialog = false;
    let mut want_restart = false;

    #[allow(deprecated)]
    event_loop.run(move |event, target| {
        if let Event::WindowEvent { ref event, .. } = event {
            if let Some(ref mut gs) = game_state {
                if gs.ui_renderer.on_event(&gs.window, event) { return; }
            } else {
                let _ = egui_st.on_window_event(&*window, event);
            }
        }

        match event {
            Event::NewEvents(winit::event::StartCause::ResumeTimeReached{..}) => { window.request_redraw(); }
            Event::WindowEvent { ref event, window_id } if window_id == window.id() => {
                match event {
                    WindowEvent::CloseRequested => {
                        // Show confirmation instead of immediate exit
                        show_exit_dialog = true;
                        window.request_redraw();
                    }
                    WindowEvent::Resized(ns) => {
                        if ns.width > 0 && ns.height > 0 {
                            if let Some(ref mut gs) = game_state { gs.resize(*ns, None); }
                            else { surf_cfg.width=ns.width; surf_cfg.height=ns.height; surface.configure(&wgpu_ctx.device, &surf_cfg); }
                        }
                    }
                    WindowEvent::KeyboardInput { event: ke, .. } => {
                        if let PhysicalKey::Code(key) = ke.physical_key {
                            let pressed = ke.state == ElementState::Pressed;
                            let released = ke.state == ElementState::Released;

                            // Esc: if dialog open → close it; otherwise open exit dialog
                            if key == KeyCode::Escape && released {
                                if show_exit_dialog {
                                    show_exit_dialog = false;
                                } else if let Phase::Playing { ref mut settings_open, .. } = phase {
                                    if *settings_open {
                                        *settings_open = false;
                                    } else {
                                        show_exit_dialog = true;
                                    }
                                } else {
                                    show_exit_dialog = true;
                                }
                            }

                            if let Phase::Playing { ref mut keys1, ref mut keys2, ref mut armed1, ref mut armed2,
                                ref mut drone_mode, ref mut drone1, ref mut drone2, ref mut settings_open, split, .. } = phase {
                                if key == KeyCode::F1 && released { *settings_open = !*settings_open; }
                                // M: toggle flight mode (per-player in split, shared in single)
                                if key == KeyCode::KeyM && released {
                                    if !split { *drone_mode = !*drone_mode;
                                        if *drone_mode { drone1.reset(0.,0.,0.); *armed1=true; } else { *armed1=false; } }
                                }
                                // N: toggle P2 flight mode in split
                                if key == KeyCode::KeyN && released && split {
                                    // Switch P2 between FPV and Drone mode
                                    use crate::drone::FlightMode;
                                    drone2.flight_mode = match drone2.flight_mode {
                                        FlightMode::Fpv => FlightMode::Drone,
                                        FlightMode::Drone => FlightMode::Fpv,
                                    };
                                }
                                if *drone_mode {
                                    match key {
                                        KeyCode::KeyW=>keys1.w=pressed, KeyCode::KeyS=>keys1.s=pressed,
                                        KeyCode::KeyA=>keys1.a=pressed, KeyCode::KeyD=>keys1.d=pressed,
                                        KeyCode::ArrowUp=>keys1.up=pressed, KeyCode::ArrowDown=>keys1.down=pressed,
                                        KeyCode::ArrowLeft=>keys1.left=pressed, KeyCode::ArrowRight=>keys1.right=pressed,
                                        KeyCode::Space=>{if pressed{*armed1=!*armed1;}}
                                        KeyCode::KeyR=>{if pressed{drone1.reset(0.,2.,0.);}}
                                        _ => {}
                                    }
                                }
                                if split {
                                    match key {
                                        KeyCode::KeyT=>keys2.t=pressed, KeyCode::KeyG=>keys2.g=pressed,
                                        KeyCode::KeyF=>keys2.f=pressed, KeyCode::KeyH=>keys2.h=pressed,
                                        KeyCode::KeyI=>keys2.i=pressed, KeyCode::KeyK=>keys2.k=pressed,
                                        KeyCode::KeyJ=>keys2.j=pressed, KeyCode::KeyL=>keys2.l=pressed,
                                        KeyCode::Enter=>{if pressed{*armed2=!*armed2;}}
                                        KeyCode::Backspace=>{if pressed{drone2.reset(2.,2.,0.);}}
                                        _ => {}
                                    }
                                }
                            }
                        }
                    }
                    WindowEvent::MouseWheel { delta, .. } => {
                        if let Some(ref mut gs) = game_state {
                            if let Phase::Playing{drone_mode,..} = &phase { if !drone_mode { match delta {
                                winit::event::MouseScrollDelta::LineDelta(_,dy)=>gs.controller.process_scroll(*dy),
                                winit::event::MouseScrollDelta::PixelDelta(p)=>gs.controller.process_scroll(p.y as f32/100.),
                            }}}
                        }
                    }
                    WindowEvent::MouseInput { state:bs, button, .. } => {
                        if let Some(ref mut gs) = game_state {
                            if let Phase::Playing{drone_mode,..} = &phase { if !drone_mode { match button {
                                winit::event::MouseButton::Left=>gs.controller.left_mouse_pressed=*bs==ElementState::Pressed,
                                winit::event::MouseButton::Right=>gs.controller.right_mouse_pressed=*bs==ElementState::Pressed,
                                _=>{}
                            }}}
                        }
                    }
                    WindowEvent::RedrawRequested => {
                        let now = Instant::now(); let dt = now-last_time; last_time=now; let dt_s=dt.as_secs_f32();

                        match &mut phase {
                            Phase::ModeSelect | Phase::SceneSelect{..} => {
                                target.set_control_flow(ControlFlow::wait_duration(Duration::from_millis(16)));
                                let raw = egui_st.take_egui_input(&*window);
                                egui_ctx.begin_pass(raw);

                                let transition = match &phase {
                                    Phase::ModeSelect => menu_ui::draw_mode_select(&egui_ctx),
                                    Phase::SceneSelect{mode, scene_files} => menu_ui::draw_scene_select(&egui_ctx, scene_files, *mode),
                                    _ => StateTransition::None,
                                };

                                // Exit confirmation dialog (drawn on top)
                                match menu_ui::draw_exit_confirm(&egui_ctx, &mut show_exit_dialog, false) {
                                    menu_ui::ExitAction::Quit => { target.exit(); return; }
                                    _ => {}
                                }

                                let output = egui_ctx.end_pass();
                                egui_st.handle_platform_output(&*window, output.platform_output.clone());

                                if let wgpu::CurrentSurfaceTexture::Success(frame)|wgpu::CurrentSurfaceTexture::Suboptimal(frame) = surface.get_current_texture() {
                                    let view = frame.texture.create_view(&Default::default());
                                    let mut enc = wgpu_ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor{label:None});
                                    let screen = egui_wgpu::ScreenDescriptor{size_in_pixels:[surf_cfg.width,surf_cfg.height],pixels_per_point:window.scale_factor() as f32};
                                    let tris = egui_ctx.tessellate(output.shapes, output.pixels_per_point);
                                    for(id,d) in &output.textures_delta.set { egui_rn.update_texture(&wgpu_ctx.device,&wgpu_ctx.queue,*id,d); }
                                    egui_rn.update_buffers(&wgpu_ctx.device,&wgpu_ctx.queue,&mut enc,&tris,&screen);
                                    let mut rp = enc.begin_render_pass(&wgpu::RenderPassDescriptor{label:None,
                                        color_attachments:&[Some(wgpu::RenderPassColorAttachment{view:&view,resolve_target:None,
                                            ops:wgpu::Operations{load:wgpu::LoadOp::Clear(wgpu::Color{r:0.05,g:0.07,b:0.11,a:1.0}),store:wgpu::StoreOp::Store},depth_slice:None})],
                                        ..Default::default()}).forget_lifetime();
                                    egui_rn.render(&mut rp, &tris, &screen); drop(rp);
                                    for id in &output.textures_delta.free { egui_rn.free_texture(id); }
                                    wgpu_ctx.queue.submit([enc.finish()]); frame.present();
                                }

                                match transition {
                                    StateTransition::ToModeSelect => { phase = Phase::ModeSelect; }
                                    StateTransition::ToSceneSelect(mode) => {
                                        phase = Phase::SceneSelect { mode, scene_files: menu_ui::scan_scene_files() };
                                    }
                                    StateTransition::ToLoading(path, mode) => { phase = Phase::Loading{path, mode}; }
                                    _ => {}
                                }
                                window.request_redraw();
                            }
                            Phase::Loading { ref path, mode } => {
                                let mode = *mode; let split = mode == GameMode::SplitScreen;
                                let p = path.clone();
                                window.set_title("MindCloud Fly - Loading...");
                                if split { let _ = window.request_inner_size(LogicalSize::new(1280u32,960)); }
                                let f = File::open(&p).unwrap_or_else(|e|{eprintln!("Failed: {:?}: {}",p,e);std::process::exit(1);});
                                let rcfg = RenderConfig{no_vsync, hdr:false};
                                let gs = pollster::block_on(WindowContext::new_from_arc(window.clone(),f,&rcfg)).unwrap();
                                let mut gs = gs; gs.pointcloud_file_path=Some(p); gs.ui_visible=!split;
                                let mut d1=Drone::new(); d1.reset(0.,2.,0.);
                                let mut d2=Drone::new(); d2.reset(2.,2.,0.);
                                let(mut a1,mut a2,mut dm)=(false,false,!split);
                                if split{dm=true;a1=true;a2=true;}
                                game_state=Some(gs);
                                phase=Phase::Playing{drone1:d1,drone2:d2,keys1:KeyState::default(),keys2:KeyStateP2::default(),
                                    armed1:a1,armed2:a2,drone_mode:dm,split,settings_open:false,controller1:input::Controller::new()};
                                window.set_title(if split{"MindCloud Fly [SPLIT]"}else{"MindCloud World Fly"});
                                window.request_redraw();
                            }
                            Phase::Playing{ref mut drone1,ref mut drone2,ref mut keys1,ref mut keys2,
                                ref mut armed1,ref mut armed2,ref mut drone_mode,split,
                                ref mut settings_open,ref mut controller1} =>
                            {
                                let gs = game_state.as_mut().unwrap();
                                gs.fps = (1./dt_s)*0.05 + gs.fps*0.95;
                                let split = *split;
                                if !no_vsync { target.set_control_flow(ControlFlow::wait_duration(Duration::from_millis(1))); }

                                // Pause game when any dialog is open
                                let paused = *settings_open || show_exit_dialog;

                                if split {
                                    // Only update physics when not paused
                                    if !paused {
                                        let i1=keys1.to_input(*armed1); drone1.update(dt_s,&i1);
                                        let i2=keys2.to_input(*armed2); drone2.update(dt_s,&i2);
                                    }
                                    if gs.splatting_args.walltime<Duration::from_secs(5){gs.splatting_args.walltime+=dt;}
                                    let mut at=gs.splatting_args;
                                    {let(p,r)=drone1.camera_transform();at.camera.position=Point3::new(p.x,p.y,p.z);at.camera.rotation=r;
                                     at.camera.projection.resize(gs.config.width,gs.config.height/2);let a=gs.pc.bbox();at.camera.fit_near_far(a);at.camera.projection.znear=0.1;}
                                    let mut ab=gs.splatting_args;
                                    {let(p,r)=drone2.camera_transform();ab.camera.position=Point3::new(p.x,p.y,p.z);ab.camera.rotation=r;
                                     ab.camera.projection.resize(gs.config.width,gs.config.height/2);let a=gs.pc.bbox();ab.camera.fit_near_far(a);ab.camera.projection.znear=0.1;}
                                    gs.ui_renderer.begin_frame(&gs.window);
                                    {let ctx=gs.ui_renderer.winit.egui_ctx().clone();
                                     // Always draw HUD
                                     let s=ctx.screen_rect();let hh=s.height()/2.;let w=s.width();
                                     hud::draw_hud(&ctx,drone1,None,*armed1,gs.fps,Some("P1"),Some(egui::Rect::from_min_size(s.min,egui::Vec2::new(w,hh))));
                                     hud::draw_hud(&ctx,drone2,None,*armed2,gs.fps,Some("P2"),Some(egui::Rect::from_min_size(egui::Pos2::new(s.min.x,s.min.y+hh),egui::Vec2::new(w,hh))));
                                     // Dark overlay on top of HUD when paused
                                     if paused {
                                         let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("pause_dim")));
                                         painter.rect_filled(s, 0.0, Color32::from_black_alpha(230));
                                     }
                                     // Dialogs on top of overlay
                                     settings::draw_settings(&ctx,settings_open,drone1,controller1,armed1);
                                     match menu_ui::draw_exit_confirm(&ctx, &mut show_exit_dialog, true) {
                                         menu_ui::ExitAction::Quit => { target.exit(); return; }
                                         menu_ui::ExitAction::BackToMenu => {
                                             show_exit_dialog = false;
                                             want_restart = true;
                                         }
                                         _ => {}
                                     }}
                                    let eo=gs.ui_renderer.end_frame(&gs.window);
                                    match gs.render_split(at,ab,Some(eo)){Ok(_)=>{}Err(_)=>gs.resize(gs.window.inner_size(),None)}
                                    gs.window.set_title(&format!("MindCloud Fly [SPLIT] - {:.0} FPS | P1:{} P2:{}{}",gs.fps,
                                        if *armed1{"ARM"}else{"OFF"},if *armed2{"ARM"}else{"OFF"},if paused{" [PAUSED]"}else{""}));
                                } else if *drone_mode {
                                    if !paused { let i1=keys1.to_input(*armed1); drone1.update(dt_s,&i1); }
                                    apply_drone_cam(gs,drone1,dt);
                                    gs.ui_renderer.begin_frame(&gs.window);
                                    {let ctx=gs.ui_renderer.winit.egui_ctx().clone();
                                     hud::draw_hud(&ctx,drone1,None,*armed1,gs.fps,None,None);
                                     if paused {
                                         let s = ctx.screen_rect();
                                         let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("pause_dim")));
                                         painter.rect_filled(s, 0.0, Color32::from_black_alpha(230));
                                     }
                                     settings::draw_settings(&ctx,settings_open,drone1,controller1,armed1);
                                     match menu_ui::draw_exit_confirm(&ctx, &mut show_exit_dialog, true) {
                                         menu_ui::ExitAction::Quit => { target.exit(); return; }
                                         menu_ui::ExitAction::BackToMenu => {
                                             show_exit_dialog = false;
                                             want_restart = true;
                                         }
                                         _ => {}
                                     }}
                                    let eo=gs.ui_renderer.end_frame(&gs.window);
                                    match gs.render(true,Some(eo)){Ok(_)=>{}Err(_)=>gs.resize(gs.window.inner_size(),None)}
                                    gs.window.set_title(&format!("MindCloud Fly [DRONE] - {:.0} FPS | Armed: {}{}",gs.fps,armed1,if paused{" [PAUSED]"}else{""}));
                                } else {
                                    if !paused { gs.update(dt); }
                                    gs.ui_renderer.begin_frame(&gs.window);
                                    {let ctx=gs.ui_renderer.winit.egui_ctx().clone();
                                     if paused {
                                         let s = ctx.screen_rect();
                                         let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("pause_dim")));
                                         painter.rect_filled(s, 0.0, Color32::from_black_alpha(230));
                                     }
                                     settings::draw_settings(&ctx,settings_open,drone1,controller1,armed1);
                                     match menu_ui::draw_exit_confirm(&ctx, &mut show_exit_dialog, true) {
                                         menu_ui::ExitAction::Quit => { target.exit(); return; }
                                         menu_ui::ExitAction::BackToMenu => {
                                             show_exit_dialog = false;
                                             want_restart = true;
                                         }
                                         _ => {}
                                     }}
                                    let eo=gs.ui_renderer.end_frame(&gs.window);
                                    match gs.render(true,Some(eo)){Ok(_)=>{}Err(_)=>gs.resize(gs.window.inner_size(),None)}
                                    gs.window.set_title(&format!("MindCloud Fly [ORBIT] - {:.0} FPS{}",gs.fps,if paused{" [PAUSED]"}else{""}));
                                }
                                if no_vsync{gs.window.request_redraw();}else{window.request_redraw();}

                                // Handle deferred restart (after frame is fully rendered)
                                if want_restart {
                                    want_restart = false;
                                    game_state = None;
                                    let exe = std::env::current_exe().unwrap();
                                    let _ = std::process::Command::new(&exe)
                                        .envs(std::env::vars())
                                        .spawn();
                                    target.exit();
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            Event::DeviceEvent{event:DeviceEvent::MouseMotion{delta},..} => {
                if let Some(ref mut gs) = game_state {
                    if let Phase::Playing{drone_mode,..} = &phase {
                        if !drone_mode { gs.controller.process_mouse(delta.0 as f32, delta.1 as f32); }
                    }
                }
            }
            _ => {}
        }
    }).unwrap();
}
