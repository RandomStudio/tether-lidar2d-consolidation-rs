use egui::{Slider, Ui};
use log::debug;
use tether_lidar2d_consolidation::{automasking::AutoMaskMessage, smoothing::EmptyListSendMode};

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
        ui.heading("Smoothing");

        if ui
            .checkbox(&mut backend_config.smoothing_disable, "Disable smoothing")
            .clicked()
        {
            model.is_editing = true;
        }

        ui.add_enabled_ui(!backend_config.smoothing_disable, |ui| {
            ui.horizontal(|ui| {
                let (label_text, slider_range) = {
                    if backend_config.smoothing_use_real_units {
                        (String::from("Merge radius (mm)"), 0. ..=5000.)
                    } else {
                        (String::from("Merge radius (units)"), 0. ..=1.0)
                    }
                };
                ui.label(label_text);
                if ui
                    .add(Slider::new(
                        &mut backend_config.smoothing_merge_radius,
                        slider_range,
                    ))
                    .changed()
                {
                    debug!(
                        "Set smoothing merge radius to {}",
                        backend_config.smoothing_merge_radius
                    );
                    model.is_editing = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Wait before active");
                let mut value = backend_config.smoothing_wait_before_active_ms as u64;
                if ui
                    .add(Slider::new(&mut value, 0..=5000).suffix("ms"))
                    .changed()
                {
                    backend_config.smoothing_wait_before_active_ms = value as u128;
                    model.is_editing = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Wait before expire");
                let mut value = backend_config.smoothing_expire_ms as u64;
                if ui
                    .add(Slider::new(&mut value, 0..=5000).suffix("ms"))
                    .changed()
                {
                    backend_config.smoothing_expire_ms = value as u128;
                    model.is_editing = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Smoothing update interval");
                let mut value = backend_config.smoothing_update_interval as u64;
                if ui
                    .add(Slider::new(&mut value, 0..=5000).suffix("ms"))
                    .changed()
                {
                    backend_config.smoothing_update_interval = value as u128;
                    model.is_editing = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Lerp factor");
                if ui
                    .add(Slider::new(
                        &mut backend_config.smoothing_lerp_factor,
                        0. ..=1.,
                    ))
                    .changed()
                {
                    model.is_editing = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Empty list send mode:");
                ui.horizontal(|ui| {
                    if ui
                        .selectable_label(
                            matches!(
                                backend_config.smoothing_empty_send_mode,
                                EmptyListSendMode::Never
                            ),
                            "Never",
                        )
                        .clicked()
                    {
                        backend_config.smoothing_empty_send_mode = EmptyListSendMode::Never;
                        model.is_editing = true;
                    };
                    if ui
                        .selectable_label(
                            matches!(
                                backend_config.smoothing_empty_send_mode,
                                EmptyListSendMode::Once
                            ),
                            "Once",
                        )
                        .clicked()
                    {
                        backend_config.smoothing_empty_send_mode = EmptyListSendMode::Once;
                        model.is_editing = true;
                    };
                    if ui
                        .selectable_label(
                            matches!(
                                backend_config.smoothing_empty_send_mode,
                                EmptyListSendMode::Always
                            ),
                            "Always",
                        )
                        .clicked()
                    {
                        backend_config.smoothing_empty_send_mode = EmptyListSendMode::Always;
                        model.is_editing = true;
                    };
                });
            });

            if ui
                .checkbox(
                    &mut backend_config.smoothing_use_real_units,
                    "Use real units",
                )
                .clicked()
            {
                // *should_publish_update = true;
                model.is_editing = true;
            };
        });
    }
}
