/// Menu UI — splash screen + scene selection.
/// Rendered via egui on a plain wgpu surface (no GSplat needed).

use egui::{Align2, Color32, FontId, Pos2, RichText, Vec2};
use std::path::PathBuf;

use crate::app_state::{GameMode, StateTransition};

/// Draw splash screen. Returns transition after timer expires or key press.
pub fn draw_splash(ctx: &egui::Context, timer: f32) -> StateTransition {
    egui::Area::new(egui::Id::new("splash"))
        .anchor(Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(40.0);
                ui.label(RichText::new("MindCloud")
                    .size(48.0)
                    .color(Color32::from_rgb(66, 114, 245))
                    .strong());
                ui.label(RichText::new("World Fly")
                    .size(32.0)
                    .color(Color32::from_rgb(150, 170, 200)));
                ui.add_space(20.0);
                ui.label(RichText::new("FPV Drone Racing Simulator")
                    .size(16.0)
                    .color(Color32::from_rgb(120, 140, 160)));
                ui.add_space(40.0);
                if timer > 1.5 {
                    ui.label(RichText::new("Press any key or click to continue...")
                        .size(14.0)
                        .color(Color32::from_rgb(100, 120, 140))
                        .italics());
                }
            });
        });

    // Auto-advance after 3 seconds, or on any input
    if timer > 3.0 || ctx.input(|i| i.pointer.any_click() || i.keys_down.len() > 0) {
        StateTransition::ToSceneSelect
    } else {
        StateTransition::None
    }
}

/// Draw scene selection screen.
pub fn draw_scene_select(
    ctx: &egui::Context,
    scene_files: &[PathBuf],
) -> StateTransition {
    let mut transition = StateTransition::None;

    egui::CentralPanel::default()
        .frame(egui::Frame::NONE.fill(Color32::from_rgb(18, 22, 32)))
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(30.0);
                ui.label(RichText::new("Select Scene")
                    .size(28.0)
                    .color(Color32::from_rgb(66, 114, 245))
                    .strong());
                ui.add_space(10.0);
                ui.label(RichText::new("Choose a Gaussian Splat scene to fly in")
                    .size(14.0)
                    .color(Color32::GRAY));
                ui.add_space(20.0);

                // Scene file list
                egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
                    for path in scene_files {
                        let name = path.file_name()
                            .map(|n| n.to_string_lossy().to_string())
                            .unwrap_or_else(|| path.to_string_lossy().to_string());
                        let size = std::fs::metadata(path)
                            .map(|m| format!("{:.0} MB", m.len() as f64 / 1_000_000.0))
                            .unwrap_or_default();

                        ui.horizontal(|ui| {
                            if ui.button(
                                RichText::new(&name)
                                    .size(16.0)
                                    .color(Color32::from_rgb(180, 200, 220))
                            ).clicked() {
                                transition = StateTransition::ToLoading(
                                    path.clone(),
                                    GameMode::SinglePlayer,
                                );
                            }
                            ui.label(RichText::new(&size).size(12.0).color(Color32::GRAY));
                        });
                        ui.add_space(4.0);
                    }
                });

                ui.add_space(20.0);

                // Browse button
                if ui.button(
                    RichText::new("📁 Browse for file...")
                        .size(14.0)
                        .color(Color32::from_rgb(100, 180, 255))
                ).clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .set_title("Select a Gaussian Splat scene")
                        .add_filter("Gaussian Splat", &["ply", "splat", "sog"])
                        .pick_file()
                    {
                        transition = StateTransition::ToLoading(path, GameMode::SinglePlayer);
                    }
                }

                ui.add_space(15.0);
                ui.separator();
                ui.add_space(10.0);

                // Game mode buttons
                ui.label(RichText::new("Game Mode").size(12.0).color(Color32::GRAY));
                ui.add_space(5.0);
                ui.horizontal(|ui| {
                    // These will be used when a scene is selected via the list
                    ui.label(RichText::new("Select a scene above to start").size(12.0).color(Color32::from_rgb(80, 100, 120)));
                });
            });
        });

    transition
}

/// Draw loading screen.
pub fn draw_loading(ctx: &egui::Context, path: &PathBuf, status: &str) {
    egui::CentralPanel::default()
        .frame(egui::Frame::NONE.fill(Color32::from_rgb(10, 12, 20)))
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(100.0);
                ui.label(RichText::new("Loading...")
                    .size(28.0)
                    .color(Color32::from_rgb(66, 114, 245)));
                ui.add_space(15.0);
                let name = path.file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_default();
                ui.label(RichText::new(&name).size(16.0).color(Color32::WHITE));
                ui.add_space(10.0);
                ui.label(RichText::new(status).size(13.0).color(Color32::GRAY));
                ui.add_space(20.0);
                ui.spinner();
            });
        });
}

/// Scan directories for scene files.
pub fn scan_scene_files() -> Vec<PathBuf> {
    let dirs = [".", "scene", "../scene"];
    let mut files: Vec<PathBuf> = Vec::new();

    for dir in &dirs {
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if let Some(ext) = path.extension() {
                    let ext = ext.to_string_lossy().to_lowercase();
                    if ext == "ply" || ext == "splat" || ext == "sog" {
                        files.push(path);
                    }
                }
            }
        }
    }

    files.sort();
    files.dedup();
    files
}
