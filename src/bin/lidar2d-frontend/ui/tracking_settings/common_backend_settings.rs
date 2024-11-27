use egui::Ui;
use tether_lidar2d_consolidation::automasking::AutoMaskMessage;

use crate::model::{EditingCorner, Model};

pub fn render_common_backend_settings(
    model: &mut Model,
    ui: &mut Ui,
    should_publish_update: &mut bool,
) {
    ui.heading("Automasking");
    ui.horizontal(|ui| {
        if ui.button("New auto-calibration").clicked() {
            model
                .tether_agent
                .encode_and_publish(
                    &model.outputs.request_automask,
                    AutoMaskMessage {
                        r#type: "new".into(),
                    },
                )
                .expect("failed to publish automask command");
        }
        if ui.button("Clear calibration").clicked() {
            model
                .tether_agent
                .encode_and_publish(
                    &model.outputs.request_automask,
                    AutoMaskMessage {
                        r#type: "clear".into(),
                    },
                )
                .expect("failed to publish automask command");
        }
    });
    ui.separator();

    ui.heading("Tracking region (ROI)");
    ui.horizontal(|ui| {
        if ui
            .selectable_label(matches!(model.editing_corners, EditingCorner::None), "None")
            .clicked()
        {
            model.editing_corners = EditingCorner::None;
            model.is_editing = true;
        };
        if ui
            .selectable_label(matches!(model.editing_corners, EditingCorner::A), "A")
            .clicked()
        {
            model.is_editing = true;
            model.editing_corners = EditingCorner::A
        };
        if ui
            .selectable_label(matches!(model.editing_corners, EditingCorner::B), "B")
            .clicked()
        {
            model.is_editing = true;
            model.editing_corners = EditingCorner::B
        };
        if ui
            .selectable_label(matches!(model.editing_corners, EditingCorner::C), "C")
            .clicked()
        {
            model.is_editing = true;
            model.editing_corners = EditingCorner::C
        };
        if ui
            .selectable_label(matches!(model.editing_corners, EditingCorner::D), "D")
            .clicked()
        {
            model.is_editing = true;
            model.editing_corners = EditingCorner::D
        };
    });

    if let Some(backend_config) = &mut model.backend_config {
        ui.separator();
        ui.heading("Clustering");

        if ui
            .checkbox(
                &mut backend_config.smoothing_use_real_units,
                "Use real units",
            )
            .clicked()
        {
            *should_publish_update = true;
        };
    }
}
