use std::f64::consts::TAU;

use colors_transform::{Color, Rgb};
use egui::{
    plot::{Line, MarkerShape, Plot, PlotPoint, PlotPoints, Points},
    remap, Checkbox, Color32, Slider,
};

use tether_lidar2d_consolidation::{clustering::Cluster2D, Point2D};

use crate::model::Model;

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
                            ui.add(Slider::new(&mut device.rotation, 0. ..=360.));
                        });
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Offset X");
                            ui.add(Slider::new(&mut device.x, 0. ..=10000.));
                        });
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Offset Y");
                            ui.add(Slider::new(&mut device.y, 0. ..=10000.));
                        });
                        ui.end_row();
                        let (current_flip_x, current_flip_y) = device.flip_coords.unwrap_or((1, 1));
                        ui.horizontal(|ui| {
                            let mut flip_x_checked = current_flip_x != 1;
                            if ui
                                .add(Checkbox::new(&mut flip_x_checked, "Flip X"))
                                .clicked()
                            {
                                let new_flip_x: i8 = if flip_x_checked { -1 } else { 1 };
                                device.flip_coords = Some((new_flip_x, current_flip_y));
                            };

                            let mut flip_y_checked = current_flip_y != 1;
                            if ui
                                .add(Checkbox::new(&mut flip_y_checked, "Flip Y"))
                                .clicked()
                            {
                                let new_flip_y: i8 = if flip_y_checked { -1 } else { 1 };
                                device.flip_coords = Some((current_flip_x, new_flip_y));
                            };
                        });
                    });
                }
                if model.is_editing {
                    if ui.button("Save 🖴").clicked() {
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
        ui.heading("Graph Area");
        let markers_plot = Plot::new("scans")
            .data_aspect(1.0)
            // .center_x_axis(true)
            // .center_y_axis(true)
            .include_y(10000.)
            .include_y(-10000.)
            .include_x(10000.)
            .include_x(-10000.);

        markers_plot.show(ui, |plot_ui| {
            let mut all_points = Vec::new();

            if let Some(tracking_config) = &model.tracking_config {
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
                    plot_ui.points(Points::from(points_group));
                }

                for cluster in model.clusters.iter() {
                    plot_ui.line(circle(
                        cluster.x,
                        cluster.y,
                        cluster.size,
                        Color32::LIGHT_GRAY,
                    ))
                    // all_points.push(cluster_to_plot_points(cluster, radius_px.max(4.0)));
                }
            }
        });
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
                let x = (angle + rotate).to_radians().cos() * distance * flip_x as f32 + offset_x;
                let y = (angle + rotate).to_radians().sin() * distance * flip_y as f32 + offset_y;
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

fn cluster_to_plot_points(cluster: &Cluster2D, radius: f32) -> Points {
    // let plot_points = PlotPoints::new([cluster.x as f64, cluster.y as f64]);
    Points::new([cluster.x as f64, cluster.y as f64])
        .filled(false)
        .radius(radius)
        .shape(MarkerShape::Circle)
        .color(Color32::WHITE)
}

fn circle(x: f32, y: f32, radius: f32, colour: Color32) -> Line {
    let n = 512;
    let circle_points: PlotPoints = (0..=n)
        .map(|i| {
            let t = remap(i as f64, 0.0..=(n as f64), 0.0..=TAU);
            let r = radius as f64;
            [r * t.cos() + x as f64, r * t.sin() + y as f64]
        })
        .collect();
    Line::new(circle_points).color(colour).name("circle")
}
