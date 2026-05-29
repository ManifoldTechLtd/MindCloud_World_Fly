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
use web_splats::{AppGpu, SceneState, SplattingArgs};
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
        // Convention: right key = roll right = negative (body Z+ = roll left)
        DroneInput { roll:if self.right{-1.}else if self.left{1.}else{0.},
            pitch:if self.up{-1.}else if self.down{1.}else{0.},
            throttle:if self.w{0.5}else if self.s{-1.}else{-0.2},
            yaw:if self.d{1.}else if self.a{-1.}else{0.},
            armed, boost:false, rates:[1.,1.,1.] }
    }
}
#[derive(Default)]
struct KeyStateP2 { i:bool,k:bool,j:bool,l:bool,t:bool,g:bool,f:bool,h:bool }
impl KeyStateP2 {
    fn to_input(&self, armed: bool) -> DroneInput {
        DroneInput { roll:if self.l{-1.}else if self.j{1.}else{0.},
            pitch:if self.i{-1.}else if self.k{1.}else{0.},
            throttle:if self.t{0.5}else if self.g{-1.}else{-0.2},
            yaw:if self.h{1.}else if self.f{-1.}else{0.},
            armed, boost:false, rates:[1.,1.,1.] }
    }
}

fn apply_drone_cam(scene: &mut SceneState, d: &Drone, dt: Duration) {
    if scene.splatting_args.walltime < Duration::from_secs(5) { scene.splatting_args.walltime += dt; }
    let (p, r) = d.camera_transform();
    scene.splatting_args.camera.position = Point3::new(p.x, p.y, p.z);
    scene.splatting_args.camera.rotation = r;
    let aabb = scene.pc.bbox();
    scene.splatting_args.camera.fit_near_far(aabb);
    scene.splatting_args.camera.projection.znear = 0.1;
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

    // Single AppGpu for entire app lifetime (menu + game share same GPU context)
    let mut gpu = AppGpu::new(window.clone(), opt.no_vsync).await;

    // Standalone egui for menu screens (uses same device)
    let egui_ctx = egui::Context::default();
    let mut egui_st = egui_winit::State::new(egui_ctx.clone(), egui::ViewportId::ROOT, &*window, None, None, None);
    let mut egui_rn = egui_wgpu::Renderer::new(gpu.device(), gpu.config.format, egui_wgpu::RendererOptions::default());

    let mut phase = if let Some(ref p) = opt.input {
        Phase::Loading { path: p.clone(), mode: if opt.split {GameMode::SplitScreen} else {GameMode::SinglePlayer} }
    } else {
        Phase::ModeSelect
    };

    let mut scene: Option<SceneState> = None;
    let mut hid_rx: Option<std::sync::mpsc::Receiver<Vec<u8>>> = None;
    let mut last_time = Instant::now();
    let no_vsync = opt.no_vsync;
    let mut show_exit_dialog = false;

    #[allow(deprecated)]
    event_loop.run(move |event, target| {
        // Forward events to appropriate egui
        if let Event::WindowEvent { ref event, .. } = event {
            if scene.is_some() {
                if gpu.egui.on_event(&gpu.window, event) { return; }
            } else {
                let _ = egui_st.on_window_event(&*window, event);
            }
        }

        match event {
            Event::NewEvents(winit::event::StartCause::ResumeTimeReached{..}) => { window.request_redraw(); }
            Event::WindowEvent { ref event, window_id } if window_id == window.id() => {
                match event {
                    WindowEvent::CloseRequested => { show_exit_dialog = true; window.request_redraw(); }
                    WindowEvent::Resized(ns) => {
                        if ns.width > 0 && ns.height > 0 {
                            gpu.resize(ns.width, ns.height);
                            if let Some(ref mut sc) = scene { sc.resize(gpu.device(), ns.width, ns.height); }
                        }
                    }
                    WindowEvent::KeyboardInput { event: ke, .. } => {
                        if let PhysicalKey::Code(key) = ke.physical_key {
                            let pressed = ke.state == ElementState::Pressed;
                            let released = ke.state == ElementState::Released;

                            // Esc: close dialog or open exit dialog
                            if key == KeyCode::Escape && released {
                                if show_exit_dialog { show_exit_dialog = false; }
                                else if let Phase::Playing { ref mut settings_open, ref mut controller1, ref mut drone1, .. } = phase {
                                    if *settings_open {
                                        *settings_open = false;
                                        let _ = persistence::save_controller_mapping(controller1);
                                        let _ = persistence::save_drone_settings(drone1);
                                    }
                                    else { show_exit_dialog = true; }
                                } else { show_exit_dialog = true; }
                            }

                            if let Phase::Playing { ref mut keys1, ref mut keys2, ref mut armed1, ref mut armed2,
                                ref mut drone_mode, ref mut drone1, ref mut drone2, ref mut settings_open, ref mut controller1, split, .. } = phase {
                                if key == KeyCode::F1 && released {
                                    if *settings_open {
                                        let _ = persistence::save_controller_mapping(controller1);
                                        let _ = persistence::save_drone_settings(drone1);
                                    }
                                    *settings_open = !*settings_open;
                                }
                                if key == KeyCode::KeyM && released && !split {
                                    *drone_mode = !*drone_mode;
                                    if *drone_mode { drone1.reset(0.,0.,0.); *armed1=true; } else { *armed1=false; }
                                }
                                if key == KeyCode::KeyN && released && split {
                                    drone2.flight_mode = match drone2.flight_mode {
                                        drone::FlightMode::Fpv => drone::FlightMode::Drone,
                                        drone::FlightMode::Drone => drone::FlightMode::Fpv,
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
                        // Only orbit mode uses scroll
                    }
                    WindowEvent::RedrawRequested => {
                        let now = Instant::now(); let dt = now-last_time; last_time=now; let dt_s=dt.as_secs_f32();
                        gpu.fps = (1./dt_s)*0.05 + gpu.fps*0.95;

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
                                match menu_ui::draw_exit_confirm(&egui_ctx, &mut show_exit_dialog, false) {
                                    menu_ui::ExitAction::Quit => { target.exit(); return; }
                                    _ => {}
                                }
                                let output = egui_ctx.end_pass();
                                egui_st.handle_platform_output(&*window, output.platform_output.clone());

                                // Render menu egui
                                if let wgpu::CurrentSurfaceTexture::Success(frame)|wgpu::CurrentSurfaceTexture::Suboptimal(frame) = gpu.surface.get_current_texture() {
                                    let view = frame.texture.create_view(&Default::default());
                                    let mut enc = gpu.device().create_command_encoder(&wgpu::CommandEncoderDescriptor{label:None});
                                    let screen = egui_wgpu::ScreenDescriptor{size_in_pixels:[gpu.config.width,gpu.config.height],pixels_per_point:window.scale_factor() as f32};
                                    let tris = egui_ctx.tessellate(output.shapes, output.pixels_per_point);
                                    for(id,d) in &output.textures_delta.set { egui_rn.update_texture(gpu.device(),gpu.queue(),*id,d); }
                                    egui_rn.update_buffers(gpu.device(),gpu.queue(),&mut enc,&tris,&screen);
                                    let mut rp = enc.begin_render_pass(&wgpu::RenderPassDescriptor{label:None,
                                        color_attachments:&[Some(wgpu::RenderPassColorAttachment{view:&view,resolve_target:None,
                                            ops:wgpu::Operations{load:wgpu::LoadOp::Clear(wgpu::Color{r:0.05,g:0.07,b:0.11,a:1.0}),store:wgpu::StoreOp::Store},depth_slice:None})],
                                        ..Default::default()}).forget_lifetime();
                                    egui_rn.render(&mut rp, &tris, &screen); drop(rp);
                                    for id in &output.textures_delta.free { egui_rn.free_texture(id); }
                                    gpu.queue().submit([enc.finish()]); frame.present();
                                }
                                match transition {
                                    StateTransition::ToModeSelect => { phase = Phase::ModeSelect; }
                                    StateTransition::ToSceneSelect(mode) => { phase = Phase::SceneSelect{mode, scene_files: menu_ui::scan_scene_files()}; }
                                    StateTransition::ToLoading(path, mode) => { phase = Phase::Loading{path, mode}; }
                                    _ => {}
                                }
                                window.request_redraw();
                            }
                            Phase::Loading { ref path, mode } => {
                                let mode = *mode; let split = mode == GameMode::SplitScreen;
                                let p = path.clone();
                                window.set_title("MindCloud Fly - Loading...");
                                if split { let _ = window.request_inner_size(LogicalSize::new(1280u32, 960)); }

                                let f = File::open(&p).unwrap_or_else(|e|{eprintln!("Failed: {:?}: {}",p,e);std::process::exit(1);});
                                let mut sc = pollster::block_on(SceneState::load(&gpu, f, false)).unwrap();
                                sc.pointcloud_file_path = Some(p);
                                // Resize scene to match current window
                                let s = window.inner_size();
                                sc.resize(gpu.device(), s.width.max(1), s.height.max(1));
                                scene = Some(sc);

                                let mut d1=Drone::new(); persistence::load_drone_settings(&mut d1); d1.reset(0.,2.,0.);
                                let mut d2=Drone::new(); persistence::load_drone_settings(&mut d2); d2.reset(2.,2.,0.);
                                let(mut a1,mut a2,mut dm)=(false,false,!split);
                                if split{dm=true;a1=true;a2=true;}

                                // HID device is now managed via Settings panel (Detect → Select)

                                phase=Phase::Playing{drone1:d1,drone2:d2,keys1:KeyState::default(),keys2:KeyStateP2::default(),
                                    armed1:a1,armed2:a2,drone_mode:dm,split,settings_open:false,controller1:input::Controller::new()};
                                window.set_title(if split{"MindCloud Fly [SPLIT]"}else{"MindCloud World Fly"});
                                window.request_redraw();
                            }
                            Phase::Playing{ref mut drone1,ref mut drone2,ref mut keys1,ref mut keys2,
                                ref mut armed1,ref mut armed2,ref mut drone_mode,split,
                                ref mut settings_open,ref mut controller1} =>
                            {
                                let sc = scene.as_mut().unwrap();
                                let split = *split;
                                let paused = *settings_open || show_exit_dialog;
                                if !no_vsync { target.set_control_flow(ControlFlow::wait_duration(Duration::from_millis(1))); }

                                // Check if user selected a new HID device from settings
                                if let Some(path) = settings::take_selected_hid_path() {
                                    hid_rx = None; // drop old receiver
                                    controller1.hid_connected = false;
                                    match input::open_hid_device(&path) {
                                        Ok(rx) => { hid_rx = Some(rx); log::info!("HID device opened"); }
                                        Err(e) => { log::warn!("Failed to open HID: {}", e); }
                                    }
                                }

                                // Poll HID controller data (always, even when paused — needed for listen mode)
                                if let Some(ref rx) = hid_rx {
                                    // Check for disconnect
                                    let mut got_data = false;
                                    while let Ok(data) = rx.try_recv() {
                                        controller1.feed_hid_report(&data);
                                        got_data = true;
                                    }
                                    let _ = got_data;
                                }
                                // Always poll listen mode (channel mapping works while settings open)
                                controller1.poll_listen();
                                // Sync current mode for per-mode rate/expo
                                controller1.current_mode = if drone1.flight_mode == drone::FlightMode::Fpv { 0 } else { 1 };

                                if split {
                                    if !paused {
                                        // P1: use HID controller if connected, else keyboard
                                        let i1 = if controller1.hid_connected {
                                            controller1.armed = *armed1;
                                            controller1.update()
                                        } else { keys1.to_input(*armed1) };
                                        *armed1 = i1.armed;
                                        // Handle mode switch from controller
                                        if controller1.mode_switch_triggered {
                                            drone1.flight_mode = match drone1.flight_mode {
                                                drone::FlightMode::Fpv => drone::FlightMode::Drone,
                                                drone::FlightMode::Drone => drone::FlightMode::Fpv,
                                            };
                                        }
                                        drone1.update(dt_s, &i1);
                                        let i2=keys2.to_input(*armed2); drone2.update(dt_s,&i2);
                                    }
                                    if sc.splatting_args.walltime<Duration::from_secs(5){sc.splatting_args.walltime+=dt;}
                                    // Camera args
                                    let mut at=sc.splatting_args;
                                    {let(p,r)=drone1.camera_transform();at.camera.position=Point3::new(p.x,p.y,p.z);at.camera.rotation=r;
                                     at.camera.projection.resize(gpu.config.width,gpu.config.height/2);
                                     let a=sc.pc.bbox();at.camera.fit_near_far(a);at.camera.projection.znear=0.1;}
                                    let mut ab=sc.splatting_args;
                                    {let(p,r)=drone2.camera_transform();ab.camera.position=Point3::new(p.x,p.y,p.z);ab.camera.rotation=r;
                                     ab.camera.projection.resize(gpu.config.width,gpu.config.height/2);
                                     let a=sc.pc.bbox();ab.camera.fit_near_far(a);ab.camera.projection.znear=0.1;}

                                    // Render splats (returns surface texture without presenting)
                                    let surface_tex = match sc.render_split(&gpu, at, ab) {
                                        Ok(tex) => tex,
                                        Err(_) => { gpu.resize(window.inner_size().width, window.inner_size().height); window.request_redraw(); return; }
                                    };

                                    // egui overlay (HUD + dialogs) on top of splat render
                                    gpu.egui.begin_frame(&gpu.window);
                                    {let ctx=gpu.egui.winit.egui_ctx().clone();
                                     let s=ctx.screen_rect();let hh=s.height()/2.;let w=s.width();
                                     hud::draw_hud(&ctx,drone1,None,*armed1,gpu.fps,Some("P1"),Some(egui::Rect::from_min_size(s.min,egui::Vec2::new(w,hh))));
                                     hud::draw_hud(&ctx,drone2,None,*armed2,gpu.fps,Some("P2"),Some(egui::Rect::from_min_size(egui::Pos2::new(s.min.x,s.min.y+hh),egui::Vec2::new(w,hh))));
                                     if paused {
                                         let painter=ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground,egui::Id::new("dim")));
                                         painter.rect_filled(s,0.0,Color32::from_black_alpha(230));
                                     }
                                     settings::draw_settings(&ctx,settings_open,drone1,controller1,armed1);
                                     match menu_ui::draw_exit_confirm(&ctx,&mut show_exit_dialog,true) {
                                         menu_ui::ExitAction::Quit => {target.exit();return;}
                                         menu_ui::ExitAction::BackToMenu => {
                                             show_exit_dialog = false;
                                             scene = None; // drop scene, deferred phase change below
                                         }
                                         _ => {}
                                     }}
                                    let egui_output = gpu.egui.end_frame(&gpu.window);

                                    // Render egui onto the surface texture
                                    {
                                        let view_srgb = surface_tex.texture.create_view(&Default::default());
                                        let mut enc = gpu.device().create_command_encoder(&wgpu::CommandEncoderDescriptor{label:Some("egui split")});
                                        let screen = egui_wgpu::ScreenDescriptor{
                                            size_in_pixels:[gpu.config.width, gpu.config.height],
                                            pixels_per_point: gpu.window.scale_factor() as f32,
                                        };
                                        let ctx = gpu.egui.winit.egui_ctx().clone();
                                        let tris = ctx.tessellate(egui_output.shapes, egui_output.pixels_per_point);
                                        let device = &gpu.wgpu_ctx.device;
                                        let queue = &gpu.wgpu_ctx.queue;
                                        for (id,d) in &egui_output.textures_delta.set { gpu.egui.renderer.update_texture(device,queue,*id,d); }
                                        gpu.egui.renderer.update_buffers(device,queue,&mut enc,&tris,&screen);
                                        let mut rp = enc.begin_render_pass(&wgpu::RenderPassDescriptor{label:Some("egui"),
                                            color_attachments:&[Some(wgpu::RenderPassColorAttachment{view:&view_srgb,resolve_target:None,
                                                ops:wgpu::Operations{load:wgpu::LoadOp::Load,store:wgpu::StoreOp::Store},depth_slice:None})],
                                            ..Default::default()}).forget_lifetime();
                                        gpu.egui.renderer.render(&mut rp, &tris, &screen);
                                        drop(rp);
                                        for id in &egui_output.textures_delta.free { gpu.egui.renderer.free_texture(id); }
                                        gpu.queue().submit([enc.finish()]);
                                    }
                                    surface_tex.present();

                                    gpu.window.set_title(&format!("MindCloud Fly [SPLIT] - {:.0} FPS | P1:{} P2:{}{}",gpu.fps,
                                        if *armed1{"ARM"}else{"OFF"},if *armed2{"ARM"}else{"OFF"},if paused{" [PAUSED]"}else{""}));
                                } else if *drone_mode {
                                    if !paused {
                                        let i1 = if controller1.hid_connected {
                                            controller1.armed = *armed1; controller1.update()
                                        } else { keys1.to_input(*armed1) };
                                        *armed1 = i1.armed;
                                        if controller1.mode_switch_triggered {
                                            drone1.flight_mode = match drone1.flight_mode {
                                                drone::FlightMode::Fpv => drone::FlightMode::Drone,
                                                drone::FlightMode::Drone => drone::FlightMode::Fpv,
                                            };
                                        }
                                        drone1.update(dt_s, &i1);
                                    }
                                    apply_drone_cam(sc, drone1, dt);
                                    sc.splatting_args.camera.projection.resize(gpu.config.width, gpu.config.height);
                                    let surface_tex = match sc.render(&gpu, None) {
                                        Ok(t) => t, Err(_) => { gpu.resize(window.inner_size().width, window.inner_size().height); window.request_redraw(); return; }
                                    };
                                    // egui HUD + dialogs
                                    gpu.egui.begin_frame(&gpu.window);
                                    {let ctx=gpu.egui.winit.egui_ctx().clone();
                                     hud::draw_hud(&ctx,drone1,None,*armed1,gpu.fps,None,None);
                                     if paused {
                                         let s=ctx.screen_rect();
                                         let painter=ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground,egui::Id::new("dim")));
                                         painter.rect_filled(s,0.0,Color32::from_black_alpha(230));
                                     }
                                     settings::draw_settings(&ctx,settings_open,drone1,controller1,armed1);
                                     match menu_ui::draw_exit_confirm(&ctx,&mut show_exit_dialog,true) {
                                         menu_ui::ExitAction::Quit => {target.exit();return;}
                                         menu_ui::ExitAction::BackToMenu => { show_exit_dialog=false; scene=None; }
                                         _ => {}
                                     }}
                                    let eo=gpu.egui.end_frame(&gpu.window);
                                    {let view_srgb=surface_tex.texture.create_view(&Default::default());
                                     let mut enc=gpu.device().create_command_encoder(&wgpu::CommandEncoderDescriptor{label:Some("egui")});
                                     let screen=egui_wgpu::ScreenDescriptor{size_in_pixels:[gpu.config.width,gpu.config.height],pixels_per_point:gpu.window.scale_factor() as f32};
                                     let ctx=gpu.egui.winit.egui_ctx().clone();
                                     let tris=ctx.tessellate(eo.shapes,eo.pixels_per_point);
                                     let device=&gpu.wgpu_ctx.device;let queue=&gpu.wgpu_ctx.queue;
                                     for(id,d) in &eo.textures_delta.set{gpu.egui.renderer.update_texture(device,queue,*id,d);}
                                     gpu.egui.renderer.update_buffers(device,queue,&mut enc,&tris,&screen);
                                     let mut rp=enc.begin_render_pass(&wgpu::RenderPassDescriptor{label:None,
                                         color_attachments:&[Some(wgpu::RenderPassColorAttachment{view:&view_srgb,resolve_target:None,
                                             ops:wgpu::Operations{load:wgpu::LoadOp::Load,store:wgpu::StoreOp::Store},depth_slice:None})],
                                         ..Default::default()}).forget_lifetime();
                                     gpu.egui.renderer.render(&mut rp,&tris,&screen);drop(rp);
                                     for id in &eo.textures_delta.free{gpu.egui.renderer.free_texture(id);}
                                     queue.submit([enc.finish()]);}
                                    surface_tex.present();
                                    gpu.window.set_title(&format!("MindCloud Fly [DRONE] - {:.0} FPS | Armed: {}{}",gpu.fps,armed1,if paused{" [PAUSED]"}else{""}));
                                } else {
                                    // Orbit mode
                                    if !paused { sc.splatting_args.walltime += dt; }
                                    let surface_tex = match sc.render(&gpu, None) {
                                        Ok(t) => t, Err(_) => { gpu.resize(window.inner_size().width, window.inner_size().height); window.request_redraw(); return; }
                                    };
                                    surface_tex.present();
                                    gpu.window.set_title(&format!("MindCloud Fly [ORBIT] - {:.0} FPS{}",gpu.fps,if paused{" [PAUSED]"}else{""}));
                                }
                                if no_vsync{window.request_redraw();}else{window.request_redraw();}
                            }
                        }
                        // Deferred: if scene was dropped during Playing (BackToMenu), reset phase
                        if matches!(phase, Phase::Playing{..}) && scene.is_none() {
                            phase = Phase::ModeSelect;
                            window.set_title("MindCloud World Fly");
                            // Keep current window size — menu adapts to any size
                        }
                    }
                    _ => {}
                }
            }
            Event::DeviceEvent{event:DeviceEvent::MouseMotion{delta},..} => {
                // Orbit camera mouse — not using SceneState controller for now
            }
            _ => {}
        }
    }).unwrap();
}
