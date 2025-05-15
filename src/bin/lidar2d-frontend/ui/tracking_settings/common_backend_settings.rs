use egui::{Color32, Grid, RichText, Slider, Ui};
use log::debug;
use tether_lidar2d_consolidation::systems::{
    automasking::AutoMaskMessage, position_remapping::OriginLocation, smoothing::EmptyListSendMode,
};

use crate::model::{EditingCorner, Model};

const BIG_TEXT_SIZE: f32 = 20.0;

pub fn render_common_backend_settings(model: &mut Model, ui: &mut Ui) {
    if let Some(backend_config) = &mut model.backend_config {
        // ------------------------ QUIET MODE
        ui.separator();
        ui.heading("Quiet Mode");

        if ui
            .checkbox(
                &mut backend_config.skip_some_outputs,
                "Skip nonessential Output messages",
            )
            .clicked()
        {
            model.is_editing = true;
        };
        if backend_config.skip_some_outputs {
            ui.label(
                RichText::new("WARNING: frontend will be missing clusters and raw tracked points.")
                    .color(Color32::LIGHT_RED),
            );
        } else {
            ui.label(RichText::new("All Outputs currently enabled.").color(Color32::LIGHT_GREEN));
        }

        // ------------------------ AUTOMASKING SETTINGS
        ui.separator();
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
            ui.label("Neighbourhood radius");
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
            ui.label("Max cluster size");
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
                .selectable_label(
                    matches!(model.editing_corners, EditingCorner::None),
                    RichText::new("None").size(BIG_TEXT_SIZE),
                )
                .clicked()
            {
                model.editing_corners = EditingCorner::None;
                model.is_editing = true;
            };
            if ui
                .selectable_label(
                    matches!(model.editing_corners, EditingCorner::A),
                    RichText::new("A").size(BIG_TEXT_SIZE),
                )
                .clicked()
            {
                model.is_editing = true;
                model.editing_corners = EditingCorner::A
            };
            if ui
                .selectable_label(
                    matches!(model.editing_corners, EditingCorner::B),
                    RichText::new("B").size(BIG_TEXT_SIZE),
                )
                .clicked()
            {
                model.is_editing = true;
                model.editing_corners = EditingCorner::B
            };
            if ui
                .selectable_label(
                    matches!(model.editing_corners, EditingCorner::C),
                    RichText::new("C").size(BIG_TEXT_SIZE),
                )
                .clicked()
            {
                model.is_editing = true;
                model.editing_corners = EditingCorner::C
            };
            if ui
                .selectable_label(
                    matches!(model.editing_corners, EditingCorner::D),
                    RichText::new("D").size(BIG_TEXT_SIZE),
                )
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
                    "Calculate velocity (mm/s) per point",
                )
                .clicked()
            {
                model.is_editing = true;
            }

            if ui
                .checkbox(
                    &mut backend_config.enable_bearing,
                    "Calculate bearing (from origin) per point",
                )
                .clicked()
            {
                model.is_editing = true;
            }

            if ui
                .checkbox(
                    &mut backend_config.enable_range,
                    "Calculate range from origin per point",
                )
                .clicked()
            {
                model.is_editing = true;
            }

            Grid::new("smooth_sliders").show(ui, |ui| {
                let slider_range = {
                    if backend_config.smoothing_use_real_units {
                        0. ..=5000.
                    } else {
                        0. ..=1.0
                    }
                };
                ui.label("Merge radius");
                if ui
                    .add(
                        Slider::new(&mut backend_config.smoothing_merge_radius, slider_range)
                            .suffix({
                                if backend_config.smoothing_use_real_units {
                                    "mm"
                                } else {
                                    ""
                                }
                            }),
                    )
                    .changed()
                {
                    debug!(
                        "Set smoothing merge radius to {}",
                        backend_config.smoothing_merge_radius
                    );
                    model.is_editing = true;
                }
                ui.end_row();

                ui.label("Wait before active");
                let mut value = backend_config.smoothing_wait_before_active_ms as u64;
                if ui
                    .add(Slider::new(&mut value, 0..=5000).suffix("ms"))
                    .changed()
                {
                    backend_config.smoothing_wait_before_active_ms = value as u128;
                    model.is_editing = true;
                }
                ui.end_row();

                ui.label("Wait before expire");
                let mut value = backend_config.smoothing_expire_ms as u64;
                if ui
                    .add(Slider::new(&mut value, 0..=5000).suffix("ms"))
                    .changed()
                {
                    backend_config.smoothing_expire_ms = value as u128;
                    model.is_editing = true;
                }
                ui.end_row();

                ui.label("Update interval");
                if ui
                    .add(
                        Slider::new(&mut backend_config.smoothing_update_interval, 0..=5000)
                            .step_by(5.0)
                            .suffix("ms"),
                    )
                    .changed()
                {
                    model.is_editing = true;
                }
                ui.label(format!(
                    "{:.0}Hz",
                    1.0 / (backend_config.smoothing_update_interval as f64 / 1000.)
                ));
                ui.end_row();

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
                ui.end_row();
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
                ui.add_enabled_ui(backend_config.smoothing_use_real_units, |ui| {
                    ui.label("Origin mode:");
                    ui.horizontal(|ui| {
                        if ui
                            .selectable_label(
                                matches!(backend_config.origin_location, OriginLocation::Corner),
                                "Corner",
                            )
                            .on_hover_text("0,0 at A")
                            .clicked()
                        {
                            backend_config.origin_location = OriginLocation::Corner;
                            model.is_editing = true;
                        };
                        if ui
                            .selectable_label(
                                matches!(
                                    backend_config.origin_location,
                                    OriginLocation::CloseCentre
                                ),
                                "CloseCentre",
                            )
                            .on_hover_text("0,0 on centre of AB line, positive Y towards DC")
                            .clicked()
                        {
                            backend_config.origin_location = OriginLocation::CloseCentre;
                            model.is_editing = true;
                        };

                        if ui
                            .selectable_label(
                                matches!(backend_config.origin_location, OriginLocation::Centre),
                                "Centre",
                            )
                            .on_hover_text("0,0 at geometric centre of ROI")
                            .clicked()
                        {
                            backend_config.origin_location = OriginLocation::Centre;
                            model.is_editing = true;
                        };
                    });
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
                let slider_range = {
                    if backend_config.smoothing_use_real_units {
                        0. ..=1000.
                    } else {
                        0. ..=1.0
                    }
                };
                ui.label("Margin to ignore outside ROI");
                if ui
                    .add(
                        Slider::new(
                            &mut backend_config.transform_ignore_outside_margin,
                            slider_range,
                        )
                        .suffix({
                            if backend_config.smoothing_use_real_units {
                                "mm"
                            } else {
                                ""
                            }
                        }),
                    )
                    .changed()
                {
                    model.is_editing = true;
                }
            });
        });

        // ---------------- MOVEMENT ANALYSIS SETTINGS
        ui.separator();
        ui.heading("Average Movement Analysis");

        if ui
            .checkbox(
                &mut backend_config.enable_average_movement,
                "Enable calculation + output",
            )
            .clicked()
        {
            model.is_editing = true;
            if backend_config.enable_average_movement {
                // Average movement calculations NEED per-point velocity calculations to be enabled, too
                backend_config.enable_velocity = true;
            }
        }

        if backend_config.enable_average_movement && !backend_config.enable_velocity {
            ui.label(
                RichText::new(
                    "Per-point velocity must also be enabled for avarege movement analysis to work",
                )
                .color(Color32::LIGHT_RED),
            );
        }

        ui.add_enabled_ui(backend_config.enable_average_movement, |ui| {
            ui.horizontal(|ui| {
                ui.label("Update interval");
                if ui
                    .add(
                        Slider::new(&mut backend_config.average_movement_interval, 8..=3000)
                            .suffix("ms"),
                    )
                    .changed()
                {
                    model.is_editing = true;
                }
            });
        });
    }
}
