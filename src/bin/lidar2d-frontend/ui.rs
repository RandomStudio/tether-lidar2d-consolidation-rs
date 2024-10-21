use std::f64::consts::TAU;

use colors_transform::{Color, Rgb};
use egui::{
    plot::{Line, MarkerShape, Plot, PlotPoints, Points, Polygon},
    remap, Checkbox, Color32, InnerResponse, RichText, Slider,
};

use tether_lidar2d_consolidation::{
    automasking::AutoMaskMessage, tracking::TrackedPoint2D, Point2D,
};

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
        })
    });

    egui::CentralPanel::default().show(ctx, |ui| {
        ui.heading("Scan Area");
        let markers_plot = Plot::new("scans")
            .data_aspect(1.0)
            .height(500.)
            .include_y(10000.)
            .include_y(-10000.)
            .include_x(10000.)
            .include_x(-10000.);

        // let PlotResponse {
        //     inner: pointer_coordinates,
        //     ...
        // } =
        let InnerResponse {
            response,
            inner: (pointer_coordinate, bounds),
            ..
        } = markers_plot.show(ui, |plot_ui| {
            if let Some(tracking_config) = &model.tracking_config {
                let mut all_points = Vec::new();

                for device in tracking_config.devices() {
                    let rgb: Rgb = Rgb::from_hex_str(&device.color).unwrap();
                    let (r, g, b) = (
                        rgb.get_red() as u8,
                        rgb.get_green() as u8,
                        rgb.get_blue() as u8,
                    );
                    if let Some(scans_this_device) = model.scans.get(&device.serial) {
                        let points = scans_to_plot_points(
                            scans_this_device,
                            model.point_size,
                            Color32::from_rgb(r, g, b),
                            device.rotation,
                            (device.x, device.y),
                            device.flip_coords.unwrap_or((1, 1)),
                        );
                        all_points.push(points);
                    }
                }
                for points_group in all_points {
                    plot_ui.points(points_group);
                }

                for cluster in model.clusters.iter() {
                    plot_ui.line(circle(
                        cluster.x,
                        cluster.y,
                        cluster.size / 2.0,
                        Color32::LIGHT_GRAY,
                    ))
                    // all_points.push(cluster_to_plot_points(cluster, radius_px.max(4.0)));
                }

                if let Some((a, b, c, d)) = tracking_config.region_of_interest() {
                    let corner_points: Vec<(f32, f32, &str)> = [d, c, b, a]
                        .iter()
                        .enumerate()
                        .map(|(index, cp)| {
                            (cp.x, cp.y, {
                                match index {
                                    0 => "A",
                                    1 => "B",
                                    2 => "C",
                                    3 => "D",
                                    _ => "unknown",
                                }
                            })
                        })
                        .collect();

                    for (x, y, name) in corner_points {
                        let plot_points = PlotPoints::new(vec![[x as f64, y as f64]]);
                        plot_ui.points(
                            Points::new(plot_points)
                                .filled(true)
                                .radius(10.)
                                .shape(MarkerShape::Circle)
                                .name(name)
                                .color(Color32::from_rgba_unmultiplied(255, 0, 0, 128)),
                        );
                    }

                    let line1 = line(a.x, a.y, d.x, d.y);
                    plot_ui.line(line1.color(Color32::RED));
                    let line2 = line(d.x, d.y, c.x, c.y);
                    plot_ui.line(line2.color(Color32::RED));
                    let line3 = line(c.x, c.y, b.x, b.y);
                    plot_ui.line(line3.color(Color32::RED));
                    let line4 = line(b.x, b.y, a.x, a.y);
                    plot_ui.line(line4.color(Color32::RED));
                }
            }
            (plot_ui.pointer_coordinate(), plot_ui.plot_bounds())
        });

        if response.clicked() {
            model.is_editing = true;
            match &mut model.editing_corners {
                EditingCorner::None => {
                    model.editing_corners = EditingCorner::A;
                }
                EditingCorner::A => {
                    model.editing_corners = EditingCorner::B;
                }
                EditingCorner::B => {
                    model.editing_corners = EditingCorner::C;
                }
                EditingCorner::C => {
                    model.editing_corners = EditingCorner::D;
                }
                EditingCorner::D => model.editing_corners = EditingCorner::None,
            }
        }

        if let Some(egui::plot::PlotPoint { x, y }) = pointer_coordinate {
            let x = x as f32;
            let y = y as f32;
            if let Some(config) = &mut model.tracking_config {
                if let Some((d, c, b, a)) = &mut config.region_of_interest_mut() {
                    // println!("{}, {}", x, y);
                    match model.editing_corners {
                        EditingCorner::None => {}
                        EditingCorner::A => {
                            a.x = x;
                            a.y = y;
                        }
                        EditingCorner::B => {
                            b.x = x;
                            b.y = y;
                        }
                        EditingCorner::C => {
                            c.x = x;
                            c.y = y;
                        }
                        EditingCorner::D => {
                            d.x = x;
                            d.y = y;
                        }
                    }
                }
            }
        }

        ui.heading("Tracking");

        let tracker_plot = Plot::new("tracker_plot")
            .data_aspect(1.0)
            .include_x(-1.)
            .include_x(3.0)
            .include_y(-1.)
            .include_y(2.0)
            .auto_bounds_x()
            .auto_bounds_y();

        tracker_plot.show(ui, |plot_ui| {
            let mut all_points = Vec::new();

            let raw_points = tracked_points_to_plot_points(
                &model.raw_tracked_points,
                12.0,
                Color32::from_rgba_unmultiplied(200, 200, 200, 8),
            );
            all_points.push(raw_points);

            let smoothed_points = tracked_points_to_plot_points(
                &model.smoothed_tracked_points,
                5.0,
                Color32::LIGHT_GREEN,
            );
            all_points.push(smoothed_points);

            for points_group in all_points {
                plot_ui.points(points_group);
            }
        })
    });
}

fn scans_to_plot_points(
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

fn tracked_points_to_plot_points(
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

fn circle(x: f32, y: f32, radius: f32, colour: Color32) -> Line {
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

fn line(x1: f32, y1: f32, x2: f32, y2: f32) -> Line {
    let slope = (y2 - y1) / (x2 - x1);
    let intercept = y1 - slope * x1;

    let range_x = if (x1 < x2) {
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
