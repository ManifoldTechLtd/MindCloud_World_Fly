/// Bevy plugin that integrates web-splat gaussian splatting as a background render pass.
///
/// Architecture:
/// - Main world: loads PLY file in background thread
/// - Once loaded: uploads to GPU via PointCloud::new (uses RenderDevice)
/// - Render world: SplatNode runs prepare (GPU sort) + display (rasterize) each frame
/// - Splat output goes to ViewTarget before Bevy's main opaque pass

use bevy::prelude::*;
use bevy::render::{
    extract_resource::{ExtractResource, ExtractResourcePlugin},
    render_graph::{RenderGraphContext, RenderLabel, ViewNode, ViewNodeRunner, RenderGraphExt},
    renderer::{RenderContext, RenderDevice, RenderQueue},
    view::ViewTarget,
    RenderApp,
};
use bevy::core_pipeline::core_3d::graph::{Core3d, Node3d};
use bevy::ecs::query::QueryItem;

use web_splats::renderer::{GaussianRenderer, Display, SplattingArgs};
use web_splats::PointCloud;
use web_splats::io::GenericGaussianPointCloud;

use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// Marker component for cameras that should render gaussian splats
#[derive(Component, Default, Clone, bevy::render::extract_component::ExtractComponent)]
pub struct SplatCamera;

/// Main-world resource: manages PLY loading
#[derive(Resource)]
pub struct SplatScene {
    pub ply_path: Option<String>,
    loader: Option<JoinHandle<GenericGaussianPointCloud>>,
    pc_raw: Option<GenericGaussianPointCloud>,
    pub loaded: bool,
}

impl Default for SplatScene {
    fn default() -> Self {
        Self { ply_path: None, loader: None, pc_raw: None, loaded: false }
    }
}

impl SplatScene {
    /// Start loading PLY in background
    pub fn start_loading(&mut self, path: String) {
        self.ply_path = Some(path.clone());
        let handle = std::thread::spawn(move || {
            let file = std::fs::File::open(&path).expect("Failed to open PLY");
            let reader = std::io::BufReader::new(file);
            GenericGaussianPointCloud::load(reader).expect("Failed to parse PLY")
        });
        self.loader = Some(handle);
    }

    /// Check if loading finished, move result to pc_raw
    pub fn poll_loading(&mut self) -> bool {
        if self.loaded { return true; }
        if let Some(ref handle) = self.loader {
            if handle.is_finished() {
                let h = self.loader.take().unwrap();
                self.pc_raw = Some(h.join().unwrap());
                self.loaded = true;
                info!("PLY loaded: {} points", self.pc_raw.as_ref().unwrap().num_points);
                return true;
            }
        }
        false
    }

    /// Take the raw point cloud data (consumed once for GPU upload)
    pub fn take_raw(&mut self) -> Option<GenericGaussianPointCloud> {
        self.pc_raw.take()
    }
}

/// Render-world resource: holds the initialized GPU state for splat rendering.
/// Wrapped in Mutex because ViewNode::run takes &self but renderer.prepare needs &mut.
#[derive(Resource)]
pub struct SplatGpuState {
    inner: Mutex<SplatGpuInner>,
}

struct SplatGpuInner {
    renderer: GaussianRenderer,
    display: Display,
    pc: PointCloud,
    args: SplattingArgs,
    display_width: u32,
    display_height: u32,
}

/// Shared state between main world and render world for initialization
#[derive(Resource, Clone, ExtractResource)]
pub struct SplatInitData {
    /// Raw point cloud data ready for GPU upload (wrapped in Arc<Mutex> for Extract)
    pub raw: Arc<Mutex<Option<GenericGaussianPointCloud>>>,
    pub width: u32,
    pub height: u32,
}

/// Plugin
pub struct SplatPlugin;

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct SplatPassLabel;

impl Plugin for SplatPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SplatScene>();
        app.insert_resource(SplatInitData {
            raw: Arc::new(Mutex::new(None)),
            width: 1280,
            height: 720,
        });
        app.add_plugins(ExtractResourcePlugin::<SplatInitData>::default());
        app.add_plugins(bevy::render::extract_component::ExtractComponentPlugin::<SplatCamera>::default());
        app.add_systems(Update, poll_splat_loading);

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };

        render_app
            .add_systems(
                bevy::render::Render,
                init_splat_gpu_state
                    .in_set(bevy::render::RenderSystems::PrepareResources)
                    .run_if(not(resource_exists::<SplatGpuState>)),
            )
            .add_render_graph_node::<ViewNodeRunner<SplatNode>>(Core3d, SplatPassLabel)
            .add_render_graph_edges(
                Core3d,
                // Run splat BEFORE the main opaque pass so it fills the ViewTarget as a
                // background. The camera uses ClearColorConfig::None (LoadOp::Load), so the
                // opaque pass keeps the splat background and draws PBR meshes on top of it
                // (meshes depth-test among themselves; splat has no depth = always behind).
                (Node3d::StartMainPass, SplatPassLabel, Node3d::MainOpaquePass),
            );
    }
}

/// Render-world system: initializes SplatGpuState once raw data arrives
fn init_splat_gpu_state(
    mut commands: Commands,
    init_data: Res<SplatInitData>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
) {
    let mut raw_lock = init_data.raw.lock().unwrap();
    let Some(raw) = raw_lock.take() else { return; };

    info!("Initializing splat GPU state ({} points, {}x{})", raw.num_points, init_data.width, init_data.height);

    let device = render_device.wgpu_device();
    let queue: &wgpu::Queue = &**render_queue;

    // Create PointCloud (uploads gaussian data to GPU)
    let pc = PointCloud::new(device, raw).expect("Failed to create PointCloud");
    info!("  PointCloud: {} points, sh_deg={}, compressed={}", pc.num_points(), pc.sh_deg(), pc.compressed());

    // Create renderer — intermediate rasterization format (linear)
    let splat_format = wgpu::TextureFormat::Rgba8Unorm;
    let renderer = pollster::block_on(
        GaussianRenderer::new(device, queue, splat_format, pc.sh_deg(), pc.compressed())
    );

    // Display compositor: source=Rgba8Unorm (splat rasterize), target=Rgba8UnormSrgb (ViewTarget)
    // Bevy's non-HDR main_texture_format is Rgba8UnormSrgb (bevy_default).
    let target_format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let display = Display::new(
        device,
        splat_format,
        target_format,
        init_data.width,
        init_data.height,
    );

    // Initial camera from bounding box
    let aabb = pc.bbox();
    let center = cgmath::Point3::new(
        (aabb.min.x + aabb.max.x) * 0.5,
        (aabb.min.y + aabb.max.y) * 0.5,
        (aabb.min.z + aabb.max.z) * 0.5,
    );
    let radius = aabb.radius();
    let position = center - cgmath::Vector3::new(1.0, 1.0, 1.0) * radius * 0.5;

    let aspect = init_data.width as f32 / init_data.height.max(1) as f32;
    let camera = web_splats::PerspectiveCamera::new(
        position,
        cgmath::Quaternion::new(1.0, 0.0, 0.0, 0.0),
        web_splats::PerspectiveProjection::new(
            cgmath::Vector2::new(init_data.width, init_data.height),
            cgmath::Vector2::new(cgmath::Deg(45.0f32), cgmath::Deg(45.0f32 / aspect)),
            0.01,
            1000.0,
        ),
    );

    let args = SplattingArgs {
        camera,
        viewport: cgmath::Vector2::new(init_data.width, init_data.height),
        gaussian_scaling: 1.0,
        max_sh_deg: pc.sh_deg(),
        mip_splatting: None,
        kernel_size: None,
        clipping_box: None,
        // Large walltime so the gaussian "grow-in" animation is fully complete
        // (scale_mod saturates to 1.0). walltime=0 keeps all splats at scale 0 → invisible.
        walltime: std::time::Duration::from_secs(100),
        scene_center: None,
        scene_extend: None,
        background_color: wgpu::Color { r: 0.0, g: 0.0, b: 0.0, a: 1.0 },
    };

    commands.insert_resource(SplatGpuState {
        inner: Mutex::new(SplatGpuInner {
            renderer,
            display,
            pc,
            args,
            display_width: init_data.width,
            display_height: init_data.height,
        }),
    });

    info!("Splat GPU state initialized!");
}

/// Main-world system: polls PLY loading and pushes data to SplatInitData
fn poll_splat_loading(
    mut scene: ResMut<SplatScene>,
    mut init_data: ResMut<SplatInitData>,
    windows: Query<&Window>,
) {
    scene.poll_loading();

    // Update resolution
    if let Ok(win) = windows.single() {
        init_data.width = win.physical_width();
        init_data.height = win.physical_height();
    }

    // If loading just completed, push raw data for render world to pick up
    if scene.loaded {
        if let Some(raw) = scene.take_raw() {
            *init_data.raw.lock().unwrap() = Some(raw);
        }
    }
}

/// The render graph node
#[derive(Default)]
struct SplatNode;

impl ViewNode for SplatNode {
    type ViewQuery = (
        &'static ViewTarget,
        &'static SplatCamera,
        &'static bevy::render::camera::ExtractedCamera,
        &'static bevy::render::view::ExtractedView,
    );

    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        (view_target, _splat_camera, _extracted_camera, extracted_view): QueryItem<Self::ViewQuery>,
        world: &World,
    ) -> Result<(), bevy::render::render_graph::NodeRunError> {
        let Some(gpu_state) = world.get_resource::<SplatGpuState>() else {
            return Ok(());
        };

        let mut state = gpu_state.inner.lock().unwrap();

        let device = render_context.render_device().wgpu_device();
        let queue: &wgpu::Queue = &**world.resource::<RenderQueue>();

        // Update viewport size and resize display texture if needed
        let target_size = view_target.main_texture().size();
        let vp_w = target_size.width;
        let vp_h = target_size.height;
        state.args.viewport = cgmath::Vector2::new(vp_w, vp_h);

        if state.display_width != vp_w || state.display_height != vp_h {
            state.display.resize(device, vp_w, vp_h);
            state.display_width = vp_w;
            state.display_height = vp_h;
            state.args.camera.projection.resize(vp_w, vp_h);
        }

        // Sync camera from Bevy's ExtractedView.
        // Bevy uses OpenGL convention (camera looks -Z, +Y up).
        // web-splat uses COLMAP convention (camera looks +Z, +Y down).
        // Conversion: rotate camera basis 180° about X (flips Y and Z).
        {
            let (_, bevy_rot, bevy_pos) = extracted_view.world_from_view.to_scale_rotation_translation();
            // 180° about X: cgmath Quaternion::new(w=0, x=1, y=0, z=0)
            let flip_x = cgmath::Quaternion::new(0.0, 1.0, 0.0, 0.0);
            let bevy_q = cgmath::Quaternion::new(bevy_rot.w, bevy_rot.x, bevy_rot.y, bevy_rot.z);
            state.args.camera.position = cgmath::Point3::new(bevy_pos.x, bevy_pos.y, bevy_pos.z);
            state.args.camera.rotation = bevy_q * flip_x;
        }
        // Use fixed near/far — bbox has outliers that push near plane too far
        state.args.camera.projection.znear = 0.1;
        state.args.camera.projection.zfar = 5000.0;

        // --- Prepare (GPU sort) + Rasterize in our own encoder ---
        let args = state.args;
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("splat_prepare_raster"),
        });

        // Split borrows: raw pointers to avoid simultaneous &mut renderer + &pc
        let renderer_ptr = &mut state.renderer as *mut GaussianRenderer;
        let pc_ptr = &state.pc as *const PointCloud;
        let display_tex = state.display.texture() as *const wgpu::TextureView;
        unsafe {
            (*renderer_ptr).prepare(&mut encoder, device, queue, &*pc_ptr, args, &mut None);

            // Rasterize splats to Display's intermediate texture
            {
                let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("splat_rasterize"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &*display_tex,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                            store: wgpu::StoreOp::Store,
                        },
                        depth_slice: None,
                    })],
                    ..Default::default()
                });
                (*renderer_ptr).render(&mut render_pass, &*pc_ptr);
            }
        }

        // Submit prepare+rasterize as a command buffer in Bevy's queue (ordered before composite)
        render_context.add_command_buffer(encoder.finish());

        // --- Composite to ViewTarget using Bevy's encoder ---
        let target_view: &wgpu::TextureView = view_target.main_texture_view();
        let camera_uniform = state.renderer.camera();
        let settings = state.renderer.render_settings();
        let encoder = render_context.command_encoder();
        state.display.render(
            encoder,
            target_view,
            state.args.background_color,
            camera_uniform,
            settings,
        );

        Ok(())
    }
}
