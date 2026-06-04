/// MindCloud Fly — Bevy + web-splat hybrid renderer
///
/// Architecture:
/// - Bevy renders PBR 3D objects (gates, buildings, glTF models)
/// - web-splat renders Gaussian Splat scene as background (via SplatPlugin)
/// - Both share the same wgpu 27 Device

mod splat_plugin;

use bevy::prelude::*;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::camera::Viewport;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use clap::Parser;

use splat_plugin::{SplatPlugin, SplatCamera};

#[derive(Parser, Debug)]
#[command(name = "mindcloud-fly-bevy", about = "MindCloud Fly — Bevy + web-splat hybrid")]
struct Args {
    /// Path to a .ply gaussian splat file
    #[arg(short, long)]
    input: Option<String>,

    /// Disable vsync
    #[arg(long)]
    no_vsync: bool,
}

fn main() {
    let args = Args::parse();

    let mut app = App::new();

    let window_plugin = WindowPlugin {
        primary_window: Some(Window {
            title: "MindCloud Fly — Bevy + web-splat".into(),
            present_mode: if args.no_vsync {
                bevy::window::PresentMode::AutoNoVsync
            } else {
                bevy::window::PresentMode::AutoVsync
            },
            resolution: bevy::window::WindowResolution::default()
                .with_scale_factor_override(1.0),
            ..default()
        }),
        ..default()
    };

    app.add_plugins(DefaultPlugins.set(window_plugin));
    app.add_plugins(PanOrbitCameraPlugin);
    app.add_plugins(SplatPlugin);
    app.add_plugins(FrameTimeDiagnosticsPlugin::default());

    // Black void color. With two cameras on one window target, the order-0 camera clears the
    // shared target to this color and the order-1 camera loads it (ClearColorConfig::Default),
    // so neither viewport erases the other.
    app.insert_resource(ClearColor(Color::BLACK));
    app.insert_resource(SceneInput { path: args.input });

    app.add_systems(Startup, setup);
    app.add_systems(Update, (update_window_title, set_split_viewports));
    app.run();
}

#[derive(Resource)]
struct SceneInput {
    path: Option<String>,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene_input: Res<SceneInput>,
    mut splat_scene: ResMut<splat_plugin::SplatScene>,
) {
    if let Some(ref path) = scene_input.path {
        info!("Starting PLY background load: {}", path);
        splat_scene.start_loading(path.clone());
    }

    // Point that the camera orbits around and where we anchor the demo objects.
    let focus = Vec3::new(50.0, 35.0, 5.0);

    // --- PBR 3D objects placed inside the gaussian splat scene ---
    let cube_mesh = meshes.add(Cuboid::new(8.0, 8.0, 8.0));
    let sphere_mesh = meshes.add(Sphere::new(5.0));

    // Red cube at the focus
    commands.spawn((
        Mesh3d(cube_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.15, 0.15),
            perceptual_roughness: 0.5,
            ..default()
        })),
        Transform::from_translation(focus),
    ));
    // Blue sphere offset along +X
    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.45, 0.95),
            metallic: 0.3,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(focus + Vec3::new(16.0, 0.0, 0.0)),
    ));
    // Green cube offset along -X
    commands.spawn((
        Mesh3d(cube_mesh),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.85, 0.3),
            perceptual_roughness: 0.6,
            ..default()
        })),
        Transform::from_translation(focus + Vec3::new(-16.0, 0.0, 0.0)),
    ));

    // --- Lighting for the PBR meshes ---
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_translation(focus + Vec3::new(30.0, 60.0, 40.0))
            .looking_at(focus, Vec3::Z),
    ));

    // --- Split-screen: two stacked cameras (top = player 0, bottom = player 1) ---
    // Each renders the full scene (splat + PBR meshes) into its own viewport half of the shared
    // window target. `set_split_viewports` (re)assigns the viewport rects every frame; the
    // SplatNode reads each camera's viewport and restricts the splat raster to it.
    // Both cameras have PanOrbitCamera with distinct initial yaw, so bevy_panorbit_camera routes
    // mouse input per-viewport (hovering a half controls that half's camera).
    //
    // No MSAA: the splat rasterize uses a single-sample intermediate + the single-sample mesh
    // depth buffer; MSAA would multisample depth and mismatch the splat pass.
    for (index, order, yaw) in [(0u8, 0isize, 0.0f32), (1u8, 1isize, std::f32::consts::FRAC_PI_2)] {
        commands.spawn((
            Camera3d::default(),
            Camera { order, ..default() },
            Tonemapping::None,
            Msaa::Off,
            AmbientLight {
                color: Color::WHITE,
                brightness: 400.0,
                ..default()
            },
            Transform::from_translation(focus + Vec3::new(5.0, 0.0, 3.0)).looking_at(focus, Vec3::Z),
            PanOrbitCamera {
                focus,
                radius: Some(45.0),
                yaw: Some(yaw),
                pitch: Some(-0.3),
                ..default()
            },
            SplatCamera,
            SplitCamera { index },
        ));
    }

    info!("=== Bevy + web-splat hybrid: SPLIT-SCREEN (2 viewports) + PBR meshes ===");
    info!("Controls: hover a viewport, then Left-drag=orbit, Right-drag=pan, Scroll=zoom");
}

/// Marks a camera as one half of the split screen. `index` 0 = top, 1 = bottom.
#[derive(Component)]
struct SplitCamera {
    index: u8,
}

/// Assigns each split-screen camera its viewport rect (top/bottom half) based on the current
/// window size. Runs every frame but only writes when the rect actually changes (so window
/// resizes are handled without spamming change detection).
fn set_split_viewports(
    windows: Query<&Window>,
    mut cameras: Query<(&SplitCamera, &mut Camera)>,
) {
    let Ok(window) = windows.single() else { return; };
    let w = window.physical_width();
    let h = window.physical_height();
    if w == 0 || h == 0 {
        return;
    }
    let half = h / 2;
    for (split, mut cam) in &mut cameras {
        let (pos, size) = match split.index {
            0 => (UVec2::new(0, 0), UVec2::new(w, half)),
            _ => (UVec2::new(0, half), UVec2::new(w, h.saturating_sub(half))),
        };
        let needs_update = match &cam.viewport {
            Some(v) => v.physical_position != pos || v.physical_size != size,
            None => true,
        };
        if needs_update {
            cam.viewport = Some(Viewport {
                physical_position: pos,
                physical_size: size,
                depth: 0.0..1.0,
            });
        }
    }
}

fn update_window_title(
    diagnostics: Res<DiagnosticsStore>,
    mut windows: Query<&mut Window>,
    splat_scene: Res<splat_plugin::SplatScene>,
) {
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    let status = if splat_scene.loaded {
        "splat loaded"
    } else if splat_scene.ply_path.is_some() {
        "loading PLY..."
    } else {
        "no scene"
    };

    if let Ok(mut window) = windows.single_mut() {
        window.title = format!("{:.0} FPS | {}", fps, status);
    }
}
