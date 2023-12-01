use colors_transform::{Color, Rgb};
use egui::{
    plot::{MarkerShape, Plot, PlotPoints, Points},
    Checkbox, Color32, Slider,
};

use tether_lidar2d_consolidation::Point2D;

use crate::model::Model;

pub fn render_ui(ctx: &egui::Context, model: &mut Model) {
    egui::SidePanel::left("config").show(ctx, |ui| match &mut model.tracking_config {
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
    });

    egui::CentralPanel::default().show(ctx, |ui| {
        ui.heading("Graph Area");
        let markers_plot = Plot::new("scans").data_aspect(1.0);

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
                        5.0,
                        Color32::from_rgb(r, g, b),
                        device.rotation,
                        (device.x, device.y),
                        device.flip_coords.unwrap_or((1, 1)),
                    );
                    all_points.push(points);
                }
            }
        }

        markers_plot.show(ui, |plot_ui| {
            for points_group in all_points {
                plot_ui.points(Points::from(points_group));
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
