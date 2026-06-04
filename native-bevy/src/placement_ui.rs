//! Placement overlay UI — ported from `native/src/placement.rs` `draw_overlay` (egui 0.33, reused
//! verbatim). Adaptations:
//!  - edits native-bevy's `SceneConfig` (held in `CurrentSceneConfig`) instead of native's,
//!  - native's custom `OrbitState` is dropped (native-bevy orbits via `bevy_panorbit_camera`); the
//!    "Set" button's `orbit.yaw = heading` coupling is therefore omitted,
//!  - `PlacementUiState` (the text-field scratch) is a Bevy `Resource`.

use crate::app_state::{SceneConfig, WorldUp};
use bevy::prelude::Resource;
use bevy_egui::egui;

/// Result of a placement overlay frame.
pub enum PlacementAction {
    None,
    StartFlight,
    GoBack,
}

/// Temporary input state for spawn/heading text fields (not yet applied). Bevy resource.
#[derive(Resource, Default)]
pub struct PlacementUiState {
    pub spawn_text: [String; 3],
    pub heading_text: String,
}

impl PlacementUiState {
    pub fn from_config(config: &SceneConfig) -> Self {
        Self {
            spawn_text: [
                format!("{:.2}", config.spawn[0]),
                format!("{:.2}", config.spawn[1]),
                format!("{:.2}", config.spawn[2]),
            ],
            heading_text: format!("{:.1}", config.heading_deg),
        }
    }
}

/// Draw placement overlay UI. Returns the action.
/// `scene_min`/`scene_max` are the scene AABB bounds for clamping typed spawn values.
pub fn draw_overlay(
    ctx: &egui::Context,
    config: &mut SceneConfig,
    input_state: &mut PlacementUiState,
    scene_min: [f32; 3],
    scene_max: [f32; 3],
) -> PlacementAction {
    let mut action = PlacementAction::None;

    // Sync text fields to live config values unless a text field currently has focus
    let any_focused = ctx.memory(|m| m.focused()).is_some_and(|id| {
        let spawn_ids: [egui::Id; 3] = std::array::from_fn(|i| egui::Id::new(format!("spawn_{}", i)));
        let heading_id = egui::Id::new("heading_input");
        spawn_ids.contains(&id) || id == heading_id
    });
    if !any_focused {
        input_state.spawn_text = [
            format!("{:.2}", config.spawn[0]),
            format!("{:.2}", config.spawn[1]),
            format!("{:.2}", config.spawn[2]),
        ];
        input_state.heading_text = format!("{:.1}", config.heading_deg);
    }

    let panel_frame = egui::Frame::NONE
        .fill(egui::Color32::from_black_alpha(230))
        .corner_radius(8.0)
        .inner_margin(egui::Margin::same(16));

    egui::Area::new(egui::Id::new("placement_ui"))
        .anchor(egui::Align2::CENTER_TOP, [0.0, 10.0])
        .show(ctx, |ui| {
            panel_frame.show(ui, |ui| {
                ui.set_min_width(420.0);
                ui.vertical_centered(|ui| {
                    ui.label(egui::RichText::new("Spawn Placement").size(22.0).color(egui::Color32::WHITE).strong());
                    ui.add_space(4.0);
                    ui.label(egui::RichText::new("WASD = move | QE = up/down | Mouse drag = orbit | Scroll = zoom")
                        .size(11.0).color(egui::Color32::from_rgb(180, 190, 200)));
                    ui.add_space(8.0);

                    // Spawn input fields
                    let field_width = 65.0;
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Spawn:").size(13.0).color(egui::Color32::WHITE));
                        ui.label(egui::RichText::new("X").size(11.0).color(egui::Color32::from_rgb(255, 120, 120)));
                        ui.add(egui::TextEdit::singleline(&mut input_state.spawn_text[0]).id(egui::Id::new("spawn_0")).desired_width(field_width).font(egui::TextStyle::Monospace));
                        ui.label(egui::RichText::new("Y").size(11.0).color(egui::Color32::from_rgb(120, 255, 120)));
                        ui.add(egui::TextEdit::singleline(&mut input_state.spawn_text[1]).id(egui::Id::new("spawn_1")).desired_width(field_width).font(egui::TextStyle::Monospace));
                        ui.label(egui::RichText::new("Z").size(11.0).color(egui::Color32::from_rgb(120, 160, 255)));
                        ui.add(egui::TextEdit::singleline(&mut input_state.spawn_text[2]).id(egui::Id::new("spawn_2")).desired_width(field_width).font(egui::TextStyle::Monospace));
                    });
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Heading:").size(13.0).color(egui::Color32::WHITE));
                        ui.add(egui::TextEdit::singleline(&mut input_state.heading_text).id(egui::Id::new("heading_input")).desired_width(field_width).font(egui::TextStyle::Monospace));
                        ui.label(egui::RichText::new("°").size(13.0).color(egui::Color32::WHITE));
                        ui.add_space(10.0);
                        if ui.button(egui::RichText::new(" Set ").size(13.0).color(egui::Color32::WHITE)).clicked() {
                            // Parse and clamp spawn
                            for i in 0..3 {
                                if let Ok(v) = input_state.spawn_text[i].trim().parse::<f32>() {
                                    config.spawn[i] = v.clamp(scene_min[i], scene_max[i]);
                                }
                            }
                            // Parse and clamp heading
                            if let Ok(v) = input_state.heading_text.trim().parse::<f32>() {
                                config.heading_deg = ((v % 360.0) + 360.0) % 360.0;
                            }
                            // Update text fields to reflect clamped values
                            for i in 0..3 {
                                input_state.spawn_text[i] = format!("{:.2}", config.spawn[i]);
                            }
                            input_state.heading_text = format!("{:.1}", config.heading_deg);
                        }
                    });

                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("World Up:").size(13.0).color(egui::Color32::WHITE));
                        if ui.selectable_label(config.world_up == WorldUp::Zup, egui::RichText::new("Z-up").size(13.0)).clicked() {
                            config.world_up = WorldUp::Zup;
                        }
                        if ui.selectable_label(config.world_up == WorldUp::Colmap, egui::RichText::new("COLMAP (Y-down)").size(13.0)).clicked() {
                            config.world_up = WorldUp::Colmap;
                        }
                    });
                    ui.add_space(12.0);
                    ui.horizontal(|ui| {
                        if ui.button(egui::RichText::new("  Start Flight (Enter)  ").size(15.0).color(egui::Color32::WHITE)).clicked() {
                            action = PlacementAction::StartFlight;
                        }
                        ui.add_space(15.0);
                        if ui.button(egui::RichText::new("  Back (Esc)  ").size(13.0).color(egui::Color32::from_rgb(150, 150, 180))).clicked() {
                            action = PlacementAction::GoBack;
                        }
                    });
                });
            });
        });

    action
}
