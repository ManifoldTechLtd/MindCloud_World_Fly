/// Phase 0.B: web-splat + Bevy hybrid — validation
///
/// Step 1: Just get Bevy running with PBR objects + FPS display.
/// Step 2: Integrate web-splat as background renderer (next iteration).
///
/// For now this validates that Bevy 0.18 + wgpu 27 works on this machine
/// and that PBR mesh rendering is performant.

use bevy::prelude::*;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use clap::Parser;

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
) {
    if let Some(ref path) = scene_input.path {
        info!("PLY path provided: {} (web-splat integration pending)", path);
    }

    // Test PBR objects — these will eventually coexist with gaussian splats
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.2, 0.2),
            emissive: LinearRgba::new(2.0, 0.4, 0.4, 1.0),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.5, 0.0),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.5, 2.0, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.2, 0.8, 1.0, 0.7),
            alpha_mode: AlphaMode::Blend,
            emissive: LinearRgba::new(0.4, 1.6, 2.0, 1.0),
            ..default()
        })),
        Transform::from_xyz(2.0, 1.0, 0.0),
    ));

    // Gate-like frame (square ring)
    let gate_color = Color::srgb(1.0, 0.84, 0.0);
    let gate_mat = materials.add(StandardMaterial {
        base_color: gate_color,
        emissive: LinearRgba::new(3.0, 2.5, 0.0, 1.0),
        unlit: true,
        ..default()
    });
    // Top bar
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.0, 0.1, 0.1))),
        MeshMaterial3d(gate_mat.clone()),
        Transform::from_xyz(0.0, 1.0, -5.0),
    ));
    // Bottom bar
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.0, 0.1, 0.1))),
        MeshMaterial3d(gate_mat.clone()),
        Transform::from_xyz(0.0, -1.0, -5.0),
    ));
    // Left bar
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.1, 2.0, 0.1))),
        MeshMaterial3d(gate_mat.clone()),
        Transform::from_xyz(-1.0, 0.0, -5.0),
    ));
    // Right bar
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.1, 2.0, 0.1))),
        MeshMaterial3d(gate_mat.clone()),
        Transform::from_xyz(1.0, 0.0, -5.0),
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, 0.5, 0.0)),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Tonemapping::None,
        Transform::from_xyz(5.0, 3.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        PanOrbitCamera::default(),
    ));

    info!("=== Bevy + web-splat hybrid: mesh-only mode ===");
    info!("Controls: Left-drag=orbit, Right-drag=pan, Scroll=zoom");
}

fn update_window_title(
    diagnostics: Res<DiagnosticsStore>,
    mut windows: Query<&mut Window>,
) {
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    if let Ok(mut window) = windows.single_mut() {
        window.title = format!("{:.0} FPS | Bevy mesh-only (web-splat pending)", fps);
    }
}
