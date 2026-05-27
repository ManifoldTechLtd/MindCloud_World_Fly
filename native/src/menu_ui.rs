/// Menu UI: Mode Select + Scene Select screens.

use egui::{Color32, RichText, Vec2};
use std::path::PathBuf;
use crate::app_state::{GameMode, StateTransition};

/// Draw mode selection screen.
pub fn draw_mode_select(ctx: &egui::Context) -> StateTransition {
    let mut transition = StateTransition::None;

    egui::CentralPanel::default()
        .frame(egui::Frame::NONE.fill(Color32::from_rgb(14, 18, 28)))
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(60.0);
                ui.label(RichText::new("MindCloud World Fly")
                    .size(36.0).color(Color32::from_rgb(66, 114, 245)).strong());
                ui.add_space(8.0);
                ui.label(RichText::new("FPV Drone Racing Simulator")
                    .size(14.0).color(Color32::from_rgb(120, 140, 160)));
                ui.add_space(50.0);
                ui.label(RichText::new("Select Mode")
                    .size(20.0).color(Color32::WHITE));
                ui.add_space(20.0);

                let btn_size = Vec2::new(280.0, 50.0);

                if ui.add_sized(btn_size, egui::Button::new(
                    RichText::new("Single Player").size(18.0).color(Color32::WHITE)
                ).fill(Color32::from_rgb(40, 60, 100))).clicked() {
                    transition = StateTransition::ToSceneSelect(GameMode::SinglePlayer);
                }
                ui.add_space(12.0);
                if ui.add_sized(btn_size, egui::Button::new(
                    RichText::new("Split Screen (2 Players)").size(18.0).color(Color32::WHITE)
                ).fill(Color32::from_rgb(40, 80, 60))).clicked() {
                    transition = StateTransition::ToSceneSelect(GameMode::SplitScreen);
                }

                ui.add_space(40.0);
                ui.label(RichText::new("Esc to quit").size(11.0).color(Color32::from_rgb(80, 90, 100)));
            });
        });

    transition
}

/// Draw scene selection screen.
pub fn draw_scene_select(ctx: &egui::Context, scene_files: &[PathBuf], mode: GameMode) -> StateTransition {
    let mut transition = StateTransition::None;
    let mode_label = match mode { GameMode::SinglePlayer => "Single Player", GameMode::SplitScreen => "Split Screen" };

    egui::CentralPanel::default()
        .frame(egui::Frame::NONE.fill(Color32::from_rgb(14, 18, 28)))
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(30.0);
                ui.label(RichText::new("Select Scene").size(28.0).color(Color32::from_rgb(66, 114, 245)).strong());
                ui.add_space(5.0);
                ui.label(RichText::new(format!("Mode: {}", mode_label)).size(13.0).color(Color32::from_rgb(100, 200, 150)));
                ui.add_space(20.0);

                egui::ScrollArea::vertical().max_height(320.0).show(ui, |ui| {
                    for path in scene_files {
                        let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                        let size = std::fs::metadata(path).map(|m| format!("{:.0} MB", m.len() as f64 / 1_000_000.0)).unwrap_or_default();
                        ui.horizontal(|ui| {
                            if ui.button(RichText::new(&name).size(15.0).color(Color32::from_rgb(180, 200, 220))).clicked() {
                                transition = StateTransition::ToLoading(path.clone(), mode);
                            }
                            ui.label(RichText::new(&size).size(11.0).color(Color32::GRAY));
                        });
                        ui.add_space(4.0);
                    }
                });

                ui.add_space(15.0);
                if ui.button(RichText::new("Browse for file...").size(13.0).color(Color32::from_rgb(100, 180, 255))).clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .set_title("Select a Gaussian Splat scene")
                        .add_filter("Gaussian Splat", &["ply", "splat", "sog"])
                        .pick_file() {
                        transition = StateTransition::ToLoading(path, mode);
                    }
                }

                ui.add_space(20.0);
                if ui.button(RichText::new("< Back to Mode Select").size(12.0).color(Color32::from_rgb(150, 150, 180))).clicked() {
                    transition = StateTransition::ToModeSelect;
                }
            });
        });

    transition
}

/// Exit/back dialog result.
pub enum ExitAction {
    None,
    BackToMenu,
    Quit,
}

/// Draw exit confirmation dialog.
/// `in_game`: if true, shows "Back to Menu" option; otherwise just Quit/Cancel.
pub fn draw_exit_confirm(ctx: &egui::Context, show: &mut bool, in_game: bool) -> ExitAction {
    let mut action = ExitAction::None;
    if *show {
        egui::Window::new("Exit")
            .collapsible(false)
            .resizable(false)
            .order(egui::Order::Tooltip)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                if in_game {
                    ui.label(RichText::new("Exit current scene?").size(14.0));
                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        if ui.button(RichText::new(" Back to Menu ").size(13.0)).clicked() {
                            action = ExitAction::BackToMenu;
                            *show = false;
                        }
                        ui.add_space(10.0);
                        if ui.button(RichText::new(" Quit Game ").size(13.0).color(Color32::from_rgb(255, 80, 80))).clicked() {
                            action = ExitAction::Quit;
                            *show = false;
                        }
                        ui.add_space(10.0);
                        if ui.button(RichText::new(" Cancel ").size(13.0)).clicked() {
                            *show = false;
                        }
                    });
                } else {
                    ui.label(RichText::new("Quit MindCloud World Fly?").size(14.0));
                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        if ui.button(RichText::new("  Yes  ").size(13.0)).clicked() {
                            action = ExitAction::Quit;
                            *show = false;
                        }
                        ui.add_space(20.0);
                        if ui.button(RichText::new("  No  ").size(13.0)).clicked() {
                            *show = false;
                        }
                    });
                }
            });
    }
    action
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
                    if ext == "ply" || ext == "splat" || ext == "sog" { files.push(path); }
                }
            }
        }
    }
    files.sort(); files.dedup(); files
}
