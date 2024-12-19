use egui::{Slider, Ui};
use log::debug;
use tether_lidar2d_consolidation::{
    automasking::AutoMaskMessage,
    smoothing::{EmptyListSendMode, OriginLocation},
};

use crate::model::{EditingCorner, Model};

pub fn render_common_backend_settings(model: &mut Model, ui: &mut Ui) {
    if let Some(backend_config) = &mut model.backend_config {
        // ------------------------ AUTOMASKING SETTINGS

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
        ui.horizontal(|ui| {
            ui.label("Number of scans required");
            if ui
                .add(Slider::new(
                    &mut backend_config.automask_scans_required,
                    1..=100,
                ))
                .changed()
            {
                model.is_editing = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Threshold tolerance");
            if ui
                .add(Slider::new(
                    &mut backend_config.automask_threshold_margin,
                    0. ..=1000.,
                ))
                .changed()
            {
                model.is_editing = true;
            }
        });

        // ------------------------ CLUSTERING SETTINGS
        ui.separator();
        ui.heading("Clustering");

        ui.horizontal(|ui| {
            ui.label("Neighbourhood radius (mm)");
            if ui
                .add(
                    Slider::new(
                        &mut backend_config.clustering_neighbourhood_radius,
                        0. ..=2000.,
                    )
                    .suffix("mm"),
                )
                .changed()
            {
                model.is_editing = true;
            }
        });

        ui.horizontal(|ui| {
            ui.label("Min neighbours (count)");
            if ui
                .add(Slider::new(
                    &mut backend_config.clustering_min_neighbours,
                    0..=100,
                ))
                .changed()
            {
                model.is_editing = true;
            }
        });

        ui.horizontal(|ui| {
            ui.label("Max cluster size (mm)");
            if ui
                .add(
                    Slider::new(
                        &mut backend_config.clustering_max_cluster_size,
                        0. ..=10000.,
                    )
                    .suffix("mm"),
                )
                .changed()
            {
                model.is_editing = true;
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

        // ------------------------ SMOOTHING SETTINGS
        ui.separator();
        ui.heading("Smoothing");

        if ui
            .checkbox(&mut backend_config.smoothing_disable, "Disable smoothing")
            .clicked()
        {
            model.is_editing = true;
        }

        ui.add_enabled_ui(!backend_config.smoothing_disable, |ui| {
            if ui
                .checkbox(
                    &mut backend_config.enable_velocity,
                    "Enable velocity per point",
                )
                .clicked()
            {
                model.is_editing = true;
            }

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

            ui.horizontal(|ui| {
                ui.label("Origin mode:");
                ui.horizontal(|ui| {
                    if ui
                        .selectable_label(
                            matches!(backend_config.origin_location, OriginLocation::TopLeft),
                            "Top Left",
                        )
                        .clicked()
                    {
                        backend_config.origin_location = OriginLocation::TopLeft;
                        model.is_editing = true;
                    };
                    if ui
                        .selectable_label(
                            matches!(backend_config.origin_location, OriginLocation::TopCentre),
                            "TopCentre",
                        )
                        .clicked()
                    {
                        backend_config.origin_location = OriginLocation::TopCentre;
                        model.is_editing = true;
                    };
                    if ui
                        .selectable_label(
                            matches!(backend_config.origin_location, OriginLocation::BottomCentre),
                            "BottomCentre",
                        )
                        .clicked()
                    {
                        backend_config.origin_location = OriginLocation::BottomCentre;
                        model.is_editing = true;
                    };
                    if ui
                        .selectable_label(
                            matches!(backend_config.origin_location, OriginLocation::Centre),
                            "Centre",
                        )
                        .clicked()
                    {
                        backend_config.origin_location = OriginLocation::Centre;
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

        // ---------------- PERSPECTIVE TRANSFORM SETTINGS
        ui.separator();
        ui.heading("Perspective/Quad Transformer");

        if ui
            .checkbox(
                &mut backend_config.transform_include_outside,
                "Include points outside ROI",
            )
            .clicked()
        {
            model.is_editing = true;
        }

        ui.add_enabled_ui(!backend_config.transform_include_outside, |ui| {
            ui.horizontal(|ui| {
                let (label_text, slider_range) = {
                    if backend_config.smoothing_use_real_units {
                        (
                            String::from("Margin to ignore outside ROI (mm)"),
                            0. ..=1000.,
                        )
                    } else {
                        (
                            String::from("Margin to ignore outside ROI (units)"),
                            0. ..=1.0,
                        )
                    }
                };
                ui.label(label_text);
                if ui
                    .add(Slider::new(
                        &mut backend_config.transform_ignore_outside_margin,
                        slider_range,
                    ))
                    .changed()
                {
                    model.is_editing = true;
                }
            });
        });

        // ---------------- MOVEMENT ANALYSIS SETTINGS
        ui.separator();
        ui.heading("Movement Analysis");

        if ui
            .checkbox(
                &mut backend_config.movement_disable,
                "Disable calculation + output",
            )
            .clicked()
        {
            model.is_editing = true;
        }

        ui.add_enabled_ui(!backend_config.movement_disable, |ui| {
            let mut value = backend_config.movement_interval as u64;
            if ui.add(Slider::new(&mut value, 8..=3000)).changed() {
                model.is_editing = true;
                backend_config.movement_interval = value as u128;
            }
        });
    }
}
