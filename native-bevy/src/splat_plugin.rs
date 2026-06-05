/// Bevy plugin that integrates web-splat gaussian splatting as a background render pass.
///
/// Architecture:
/// - Main world: loads PLY file in background thread
/// - Once loaded: uploads to GPU via PointCloud::new (uses RenderDevice)
/// - Render world: SplatNode runs prepare (GPU sort) + display (rasterize) each frame
/// - Splat runs AFTER the mesh passes and depth-tests against the mesh depth buffer
///   (read-only reverse-Z), then composites over the meshes for splat<->mesh occlusion.

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
use std::collections::HashMap;

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
    /// Robust scene center (plane-fit centroid) of the loaded cloud, in PLY coords. Because the
    /// splat node passes the Bevy camera POSITION through unchanged (only rotation is converted),
    /// PLY coords == Bevy world coords for positions, so this is directly usable as a Bevy-space
    /// orbit focus / default spawn point. `None` until loading completes.
    pub scene_center: Option<[f32; 3]>,
}

impl Default for SplatScene {
    fn default() -> Self {
        Self { ply_path: None, loader: None, pc_raw: None, loaded: false, scene_center: None }
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
                if let Some(pc) = &self.pc_raw {
                    self.scene_center = Some([pc.center.x, pc.center.y, pc.center.z]);
                    info!(
                        "PLY loaded: {} points, center {:?}, bbox radius {:.1}",
                        pc.num_points,
                        self.scene_center.unwrap(),
                        pc.aabb.radius()
                    );
                }
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
    /// Shared, read-only during rendering — all views rasterize the same gaussians.
    pc: PointCloud,
    /// Template args; camera / projection / viewport are overwritten per view each frame.
    args: SplattingArgs,
    splat_format: wgpu::TextureFormat,
    target_format: wgpu::TextureFormat,
    depth_format: wgpu::TextureFormat,
    /// Per-view GPU state keyed by the frame-stable RetainedViewEntity. Each split-screen view
    /// owns its own GaussianRenderer (sort + indirect-draw scratch) and Display (intermediate +
    /// compositor); sharing one renderer across two views in a frame caused a GPU device loss.
    views: HashMap<bevy::render::view::RetainedViewEntity, ViewSplat>,
}

/// Per-view splat GPU resources (the PointCloud is shared, held in SplatGpuInner).
struct ViewSplat {
    renderer: GaussianRenderer,
    display: Display,
    width: u32,
    height: u32,
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
                (
                    init_splat_gpu_state.run_if(not(resource_exists::<SplatGpuState>)),
                    // Create per-view renderers AFTER init, once SplatGpuState exists. Runs as a
                    // system (not in the render graph) because GPURSSorter creation submits +
                    // polls the device, which would deadlock inside a render-graph node.
                    prepare_splat_views
                        .after(init_splat_gpu_state)
                        .run_if(resource_exists::<SplatGpuState>),
                )
                    .in_set(bevy::render::RenderSystems::PrepareResources),
            )
            .add_render_graph_node::<ViewNodeRunner<SplatNode>>(Core3d, SplatPassLabel)
            .add_render_graph_edges(
                Core3d,
                // Run splat AFTER the opaque/transparent mesh passes so the mesh depth buffer is
                // fully populated. The splat rasterize then depth-tests each splat against the
                // mesh depth (read-only, reverse-Z Greater): splats behind meshes are discarded,
                // splats in front are composited over. The camera clears to black; splats fill
                // the background wherever no mesh is present.
                (Node3d::MainTransparentPass, SplatPassLabel, Node3d::EndMainPass),
            );
    }
}

/// Render-world system: initializes SplatGpuState once raw data arrives
fn init_splat_gpu_state(
    mut commands: Commands,
    init_data: Res<SplatInitData>,
    render_device: Res<RenderDevice>,
) {
    let mut raw_lock = init_data.raw.lock().unwrap();
    let Some(raw) = raw_lock.take() else { return; };

    info!("Initializing splat GPU state ({} points, {}x{})", raw.num_points, init_data.width, init_data.height);

    let device = render_device.wgpu_device();

    // Create PointCloud (uploads gaussian data to GPU)
    let pc = PointCloud::new(device, raw).expect("Failed to create PointCloud");
    info!("  PointCloud: {} points, sh_deg={}, compressed={}", pc.num_points(), pc.sh_deg(), pc.compressed());

    // Per-view GaussianRenderer + Display are created lazily in `prepare_splat_views` (one per
    // camera/viewport), all sharing this PointCloud. Record the formats they need:
    //  - splat_format: intermediate rasterization format (linear Rgba8Unorm).
    //  - depth_format: Bevy CORE_3D_DEPTH_FORMAT (Depth32Float) so the rasterize pass can
    //    depth-test splats against the mesh depth buffer (read-only, reverse-Z) for occlusion.
    //  - target_format: Bevy non-HDR ViewTarget format (Rgba8UnormSrgb).
    let splat_format = wgpu::TextureFormat::Rgba8Unorm;
    let target_format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let depth_format = wgpu::TextureFormat::Depth32Float;

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
            pc,
            args,
            splat_format,
            target_format,
            depth_format,
            views: HashMap::new(),
        }),
    });

    info!("Splat GPU state initialized!");
}

/// Render-world system: ensure every `SplatCamera` view has its own `GaussianRenderer` + `Display`,
/// keyed by the frame-stable `RetainedViewEntity`. Runs as a system (safe to submit+poll the device
/// for sorter setup) rather than inside the render-graph node (which must not block).
fn prepare_splat_views(
    gpu_state: Option<Res<SplatGpuState>>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    views: Query<
        (
            &bevy::render::view::ExtractedView,
            &bevy::render::camera::ExtractedCamera,
        ),
        With<SplatCamera>,
    >,
) {
    let Some(gpu_state) = gpu_state else { return; };
    let mut state = gpu_state.inner.lock().unwrap();
    let device = render_device.wgpu_device();
    let queue: &wgpu::Queue = &**render_queue;

    for (extracted_view, extracted_camera) in &views {
        let key = extracted_view.retained_view_entity;
        if state.views.contains_key(&key) {
            continue;
        }
        // Size the per-view intermediate to the FULL render target (matches Bevy's depth texture).
        let size = extracted_camera
            .physical_target_size
            .unwrap_or(UVec2::new(1280, 720));
        let (w, h) = (size.x.max(1), size.y.max(1));
        let sh_deg = state.pc.sh_deg();
        let compressed = state.pc.compressed();
        let (sf, tf, df) = (state.splat_format, state.target_format, state.depth_format);
        info!("Creating per-view splat renderer for {:?} ({}x{})", key, w, h);
        let renderer = pollster::block_on(GaussianRenderer::new_with_depth(
            device, queue, sf, sh_deg, compressed, Some(df),
        ));
        let display = Display::new(device, sf, tf, w, h);
        state.views.insert(
            key,
            ViewSplat {
                renderer,
                display,
                width: w,
                height: h,
            },
        );
    }
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
        &'static bevy::render::view::ViewDepthTexture,
    );

    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        (view_target, _splat_camera, extracted_camera, extracted_view, view_depth): QueryItem<Self::ViewQuery>,
        world: &World,
    ) -> Result<(), bevy::render::render_graph::NodeRunError> {
        let Some(gpu_state) = world.get_resource::<SplatGpuState>() else {
            return Ok(());
        };

        let mut state = gpu_state.inner.lock().unwrap();

        let device = render_context.render_device().wgpu_device();
        let queue: &wgpu::Queue = &**world.resource::<RenderQueue>();

        // --- Resolve sizes (split-screen aware) ---
        // Bevy sizes the ViewTarget AND the mesh depth texture to the FULL render target (window),
        // even for viewport cameras. The splat raster pass binds the mesh depth as its depth
        // attachment, so its color target (our intermediate) must match the full target size.
        let target_size = view_target.main_texture().size();
        let full_w = target_size.width.max(1);
        let full_h = target_size.height.max(1);

        // This camera's viewport sub-rect within the target. None => single full-window camera.
        let (vp_x, vp_y, vp_w, vp_h) = match extracted_camera.viewport.as_ref() {
            Some(vp) => (
                vp.physical_position.x as f32,
                vp.physical_position.y as f32,
                vp.physical_size.x.max(1) as f32,
                vp.physical_size.y.max(1) as f32,
            ),
            None => (0.0, 0.0, full_w as f32, full_h as f32),
        };

        // Per-view GPU state (created by prepare_splat_views). Skip until it's ready.
        let key = extracted_view.retained_view_entity;
        if !state.views.contains_key(&key) {
            return Ok(());
        }

        // 2D splat sizing uses the viewport resolution; set_viewport maps NDC into the sub-rect.
        state.args.viewport = cgmath::Vector2::new(vp_w as u32, vp_h as u32);

        // Sync camera position + rotation from Bevy's ExtractedView.
        // Bevy uses OpenGL convention (camera looks -Z, +Y up); its `world_from_view` rotation is
        // camera→world. web-splat uses COLMAP convention (camera looks +Z, +Y down) and its
        // `world2view` treats `camera.rotation` as a WORLD→VIEW rotation. So the conversion is:
        //   1. flip the camera basis 180° about X (Bevy↔COLMAP: flips Y and Z), then
        //   2. CONJUGATE (camera→world ⇒ world→view).
        // `bevy_q * flip_x` alone only matches at identity and drifts under rotation (meshes slide
        // off the splat). Verified to 0.0 NDC error by tests/splat_camera_align.rs.
        {
            let (_, bevy_rot, bevy_pos) = extracted_view.world_from_view.to_scale_rotation_translation();
            // 180° about X: cgmath Quaternion::new(w=0, x=1, y=0, z=0)
            let flip_x = cgmath::Quaternion::new(0.0, 1.0, 0.0, 0.0);
            let bevy_q = cgmath::Quaternion::new(bevy_rot.w, bevy_rot.x, bevy_rot.y, bevy_rot.z);
            state.args.camera.position = cgmath::Point3::new(bevy_pos.x, bevy_pos.y, bevy_pos.z);
            state.args.camera.rotation = (bevy_q * flip_x).conjugate();
        }
        // Derive fov + near DIRECTLY from Bevy's projection matrix so the splat frustum matches the
        // meshes for ANY viewport aspect (split-screen halves are very wide). web-splat's own
        // resize() heuristic anchors fovx when w>h, which diverges hard from Bevy's fixed fovy.
        // Perspective infinite-reverse-z (column-major): m00 = f/aspect, m11 = f = 1/tan(fovy/2),
        // near = clip_from_view[3][2]. => fovx = 2*atan(1/m00), fovy = 2*atan(1/m11).
        {
            let p = extracted_view.clip_from_view;
            let m00 = p.x_axis.x;
            let m11 = p.y_axis.y;
            if m00.abs() > 1e-6 && m11.abs() > 1e-6 {
                state.args.camera.projection.fovx = cgmath::Rad(2.0 * (1.0 / m00).abs().atan());
                state.args.camera.projection.fovy = cgmath::Rad(2.0 * (1.0 / m11).abs().atan());
            }
            // Bevy's actual near plane drives the reverse-Z depth used for splat<->mesh occlusion.
            state.args.camera.projection.znear = p.w_axis.z.abs().max(1e-4);
            state.args.camera.projection.zfar = 5000.0;
        }
        let args = state.args;

        // Resize this view's intermediate to the full target (matches the mesh depth attachment).
        {
            let view = state.views.get_mut(&key).unwrap();
            if view.width != full_w || view.height != full_h {
                view.display.resize(device, full_w, full_h);
                view.width = full_w;
                view.height = full_h;
            }
        }

        // --- Prepare (GPU sort) + rasterize + composite, in ONE encoder for THIS view ---
        // Each view has independent GPU state (own renderer + intermediate), so there is no shared
        // mutable resource and no cross-view ordering hazard.
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("splat_prepare_raster_composite"),
        });

        // Split borrows through the MutexGuard via raw pointers (renderer is &mut, pc/display are &).
        let renderer_ptr = &mut state.views.get_mut(&key).unwrap().renderer as *mut GaussianRenderer;
        let display_tex = state.views.get(&key).unwrap().display.texture() as *const wgpu::TextureView;
        let pc_ptr = &state.pc as *const PointCloud;
        // Mesh depth buffer (written by the mesh passes). The splat pipeline tests against it
        // read-only (never writes), so splats behind meshes are discarded while splat<->splat
        // ordering stays handled by the GPU sort + front-to-back blend.
        let depth_view = view_depth.view();
        unsafe {
            (*renderer_ptr).prepare(&mut encoder, device, queue, &*pc_ptr, args, &mut None);

            // Rasterize splats into this view's full-window intermediate, depth-tested vs mesh
            // depth, restricted to the camera's viewport sub-rect (rest stays transparent).
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
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: depth_view,
                        // Load existing mesh depth; store is a no-op (pipeline depth_write=false)
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
                render_pass.set_viewport(vp_x, vp_y, vp_w, vp_h, 0.0, 1.0);
                (*renderer_ptr).render(&mut render_pass, &*pc_ptr);
            }
        }

        // Composite this view's intermediate OVER the meshes (premultiplied-alpha blend,
        // LoadOp::Load keeps mesh colors where splats are absent or depth-discarded). The
        // intermediate maps 1:1 to the full-window ViewTarget, so a FULL-SCREEN composite
        // (region=None) lands this view's sub-rect content in place; the transparent remainder is a
        // no-op and preserves the other viewport. Same encoder, after the raster pass.
        {
            let view = state.views.get(&key).unwrap();
            let target_view: &wgpu::TextureView = view_target.main_texture_view();
            view.display.render_to_region(
                &mut encoder,
                target_view,
                args.background_color,
                view.renderer.camera(),
                view.renderer.render_settings(),
                None,
                false,
            );
        }

        render_context.add_command_buffer(encoder.finish());

        Ok(())
    }
}
