use std::{fs::File, path::PathBuf};

use clap::Parser;
use web_splats::{open_window, RenderConfig};

#[derive(Debug, Parser)]
#[command(author, version, about = "MindCloud World Fly — native FPV drone racing simulator")]
struct Opt {
    /// Input PLY/splat scene file
    input: PathBuf,

    /// Optional scene camera JSON
    scene: Option<PathBuf>,

    /// Disable V-sync for max framerate
    #[arg(long, default_value_t = false)]
    no_vsync: bool,
}

#[pollster::main]
async fn main() {
    // env_logger is initialized inside open_window; don't call it here.

    let opt = Opt::parse();

    let data_file = File::open(&opt.input).unwrap_or_else(|e| {
        eprintln!("Failed to open {:?}: {}", opt.input, e);
        std::process::exit(1);
    });

    let scene_file = opt.scene.as_ref().map(|p| {
        File::open(p).unwrap_or_else(|e| {
            eprintln!("Failed to open scene file {:?}: {}", p, e);
            std::process::exit(1);
        })
    });

    // For now, directly use web-splat's window+renderer.
    // Phase 2+ will replace this with our own game loop.
    open_window(
        data_file,
        scene_file,
        RenderConfig {
            no_vsync: opt.no_vsync,
            hdr: false,
        },
        Some(opt.input),
        opt.scene,
    )
    .await;
}
