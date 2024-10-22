mod scan_graph;
mod tracking_graph;

use std::f64::consts::TAU;

use egui::{
    plot::{Line, MarkerShape, PlotPoints, Points},
    remap, Checkbox, Color32, RichText, Slider,
};

use scan_graph::render_scan_graph;
use tether_lidar2d_consolidation::{
    automasking::AutoMaskMessage, tracking::TrackedPoint2D, Point2D,
};
use tracking_graph::render_tracking_graph;

use crate::model::{EditingCorner, Model};

pub fn render_ui(ctx: &egui::Context, model: &mut Model) {
    egui::SidePanel::left("config").show(ctx, |ui| {
        ui.heading("Visualisation Settings");
        ui.horizontal(|ui| {
            ui.label("Point radius");
            ui.add(Slider::new(&mut model.point_size, 1.0..=20.0));
        });
        ui.separator();

        ui.heading("Tracking Configuration");

        match &mut model.tracking_config {
            None => {
                ui.label("No config received (yet)");
            }
            Some(tracking_config) => {
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
                        .selectable_label(
                            matches!(model.editing_corners, EditingCorner::None),
                            "None",
                        )
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

                ui.separator();
                ui.heading("Devices");
                for device in tracking_config.devices_mut().iter_mut() {
                    ui.group(|ui| {
                        if model.is_editing {
                            ui.text_edit_singleline(&mut device.name);
                        } else {
                            ui.heading(&device.name);
                        }
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Serial #");
                            ui.label(&device.serial);
                        });
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Rotation");
                            if ui
                                .add(Slider::new(&mut device.rotation, 0. ..=360.))
                                .changed()
                            {
                                model.is_editing = true;
                            };
                        });
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Offset X");
                            if ui
                                .add(Slider::new(&mut device.x, -10000. ..=10000.))
                                .changed()
                            {
                                model.is_editing = true;
                            };
                        });
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Offset Y");
                            if ui
                                .add(Slider::new(&mut device.y, -10000. ..=10000.))
                                .changed()
                            {
                                model.is_editing = true;
                            };
                        });
                        ui.end_row();
                        let (current_flip_x, current_flip_y) = device.flip_coords.unwrap_or((1, 1));
                        ui.horizontal(|ui| {
                            let mut flip_x_checked = current_flip_x != 1;
                            if ui
                                .add(Checkbox::new(&mut flip_x_checked, "Flip X"))
                                .clicked()
                            {
                                model.is_editing = true;
                                let new_flip_x: i8 = if flip_x_checked { -1 } else { 1 };
                                device.flip_coords = Some((new_flip_x, current_flip_y));
                            };

                            let mut flip_y_checked = current_flip_y != 1;
                            if ui
                                .add(Checkbox::new(&mut flip_y_checked, "Flip Y"))
                                .clicked()
                            {
                                model.is_editing = true;
                                let new_flip_y: i8 = if flip_y_checked { -1 } else { 1 };
                                device.flip_coords = Some((current_flip_x, new_flip_y));
                            };
                        });
                    });
                }
                if model.is_editing {
                    if ui
                        .button(
                            RichText::new("Save 🖴")
                                .color(Color32::LIGHT_GREEN)
                                .size(16.0),
                        )
                        .clicked()
                    {
                        model
                            .tether_agent
                            .encode_and_publish(&model.outputs.config, &model.tracking_config)
                            .expect("failed to publish config");
                        model.is_editing = false;
                    }
                } else {
                    if ui.button("Edit ✏").clicked() {
                        model.is_editing = true;
                    }
                }
            }
        }
    });

    egui::SidePanel::right("stats").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.label("Clusters count: ");
            ui.label(format!("{}", model.clusters.len()));
        });
        ui.horizontal(|ui| {
            ui.label("(Raw) tracked points count: ");
            ui.label(format!("{}", model.raw_tracked_points.len()));
        });
        ui.horizontal(|ui| {
            ui.label("Smoothed tracked points count: ");
            ui.label(format!("{}", model.smoothed_tracked_points.len()));
        });
    });

    egui::CentralPanel::default().show(ctx, |ui| {
        ui.heading("Scan Area");
        render_scan_graph(model, ui);

        ui.heading("Tracking");
        render_tracking_graph(model, ui);
    });
}

pub fn scans_to_plot_points(
    measurements: &[Point2D],
    size: f32,
    color: Color32,
    rotate: f32,
    offset: (f32, f32),
    flip_coords: (i8, i8),
) -> Points {
    let (offset_x, offset_y) = offset;
    let (flip_x, flip_y) = flip_coords;

    let plot_points = PlotPoints::new(
        measurements
            .iter()
            .map(|(angle, distance)| {
                let x = (angle + rotate).to_radians().sin() * distance * flip_x as f32 + offset_x;
                let y = (angle + rotate).to_radians().cos() * distance * flip_y as f32 + offset_y;
                [x as f64, y as f64]
            })
            .collect(),
    );
    Points::new(plot_points)
        .filled(true)
        .radius(size)
        .shape(MarkerShape::Circle)
        .color(color)
}

pub fn smoothed_tracked_points_to_plot_points(
    tracked_points: &[TrackedPoint2D],
    size: f32,
    color: Color32,
) -> Points {
    let plot_points = PlotPoints::new(
        tracked_points
            .iter()
            .map(|tp| {
                let x = tp.x;
                let y = tp.y;
                [x as f64, y as f64]
            })
            .collect(),
    );
    Points::new(plot_points)
        .filled(true)
        .radius(size)
        .shape(MarkerShape::Circle)
        .color(color)
}

pub fn raw_tracked_points_to_plot_points(
    tracked_points: &[Point2D],
    size: f32,
    color: Color32,
) -> Points {
    let plot_points = PlotPoints::new(
        tracked_points
            .iter()
            .map(|tp| {
                let (x, y) = *tp;
                [x as f64, y as f64]
            })
            .collect(),
    );
    Points::new(plot_points)
        .filled(true)
        .radius(size)
        .shape(MarkerShape::Circle)
        .color(color)
}

fn draw_circle(x: f32, y: f32, radius: f32, colour: Color32) -> Line {
    let n = 512;
    let circle_points: PlotPoints = (0..=n)
        .map(|i| {
            let t = remap(i as f64, 0.0..=(n as f64), 0.0..=TAU);
            let r = radius as f64;
            [r * t.sin() + x as f64, r * t.cos() + y as f64]
        })
        .collect();
    Line::new(circle_points).color(colour).name("circle")
}

pub fn draw_line(x1: f32, y1: f32, x2: f32, y2: f32) -> Line {
    let slope = (y2 - y1) / (x2 - x1);
    let intercept = y1 - slope * x1;

    let range_x = if x1 < x2 {
        (x1 as f64)..(x2 as f64)
    } else {
        (x2 as f64)..(x1 as f64)
    };

    Line::new(PlotPoints::from_explicit_callback(
        move |x| (slope as f64) * x + (intercept as f64),
        range_x,
        256,
    ))
}
