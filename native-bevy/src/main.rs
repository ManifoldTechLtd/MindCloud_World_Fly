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

    app.insert_resource(SceneInput { path: args.input });

    app.add_systems(Startup, setup);
    app.add_systems(Update, update_window_title);
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

    // Camera — SplatCamera marks it for gaussian splat rendering.
    // clear_color=black: meshes are drawn first into the cleared ViewTarget (writing the depth
    // buffer), then the SplatNode (running after the mesh passes) rasterizes splats depth-tested
    // against that mesh depth and composites them over, giving splat<->mesh occlusion.
    commands.spawn((
        Camera3d::default(),
        Tonemapping::None,
        // No MSAA: the splat rasterize uses a single-sample intermediate + the (single-sample)
        // mesh depth buffer. With MSAA on, depth would be multisampled and mismatch the pass.
        Msaa::Off,
        Camera {
            clear_color: ClearColorConfig::Custom(Color::BLACK),
            ..default()
        },
        AmbientLight {
            color: Color::WHITE,
            brightness: 400.0,
            ..default()
        },
        Transform::from_translation(focus + Vec3::new(5.0, 0.0, 3.0)).looking_at(focus, Vec3::Z),
        PanOrbitCamera {
            focus,
            radius: Some(45.0),
            ..default()
        },
        SplatCamera,
    ));

    info!("=== Bevy + web-splat hybrid: splat background + PBR meshes ===");
    info!("Controls: Left-drag=orbit, Right-drag=pan, Scroll=zoom");
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
