use colorsys::Rgb;
use egui::{
    plot::{MarkerShape, Plot, PlotPoint, PlotPoints, Points, Text},
    Color32, InnerResponse, Ui,
};
use log::{debug, warn};
use tether_lidar2d_consolidation::backend_config::ConfigRectCornerPoint;

use crate::model::{EditingCorner, Model};

use super::{angle_samples_to_plot_points, draw_circle, draw_line};

pub fn render_scan_graph(model: &mut Model, ui: &mut Ui) {
    let markers_plot = Plot::new("scans")
        .data_aspect(1.0)
        .height(500.)
        .include_y(10000.)
        .include_y(-10000.)
        .include_x(10000.)
        .include_x(-10000.);

    let InnerResponse {
        response,
        inner: (pointer_coordinate, _bounds),
        ..
    } = markers_plot.show(ui, |plot_ui| {
        if let Some(tracking_config) = &model.backend_config {
            let mut all_points = Vec::new();

            for device in tracking_config.devices() {
                if let Some(scans_this_device) = model.scans.get(&device.serial) {
                    let rgb: [u8; 3] = Rgb::from_hex_str(&device.colour).unwrap().into();
                    let [r, g, b] = rgb;
                    let points = angle_samples_to_plot_points(
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
                plot_ui.line(draw_circle(
                    cluster.x,
                    cluster.y,
                    cluster.size / 2.0,
                    Color32::LIGHT_GRAY,
                ))
                // all_points.push(cluster_to_plot_points(cluster, radius_px.max(4.0)));
            }

            if let Some((a, b, c, d)) = tracking_config.region_of_interest() {
                let corner_points: Vec<(f32, f32, &str)> = [a, b, c, d]
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
                            .color(Color32::from_rgba_unmultiplied(255, 0, 0, 32)),
                    );
                    plot_ui.text(
                        Text::new(PlotPoint::new(x, y), name)
                            .color(Color32::from_rgba_unmultiplied(255, 0, 0, 255)),
                    )
                }

                let line1 = draw_line(a.x, a.y, d.x, d.y);
                plot_ui.line(line1.color(Color32::RED));
                let line2 = draw_line(d.x, d.y, c.x, c.y);
                plot_ui.line(line2.color(Color32::RED));
                let line3 = draw_line(c.x, c.y, b.x, b.y);
                plot_ui.line(line3.color(Color32::RED));
                let line4 = draw_line(b.x, b.y, a.x, a.y);
                plot_ui.line(line4.color(Color32::RED));
            }
        }
        (plot_ui.pointer_coordinate(), plot_ui.plot_bounds())
    });

    if response.clicked() {
        debug!("Clicked scan graph");
        match &mut model.editing_corners {
            EditingCorner::None => {
                // Do nothing
                debug!("No corners currently edited; do nothing")
            }
            _ => {
                debug!("Was editing {:?}", model.editing_corners);
                model.is_editing = true;
                model.editing_corners = EditingCorner::None;
            }
        }
    }

    if let Some(egui::plot::PlotPoint { x, y }) = pointer_coordinate {
        // debug!("Should edit using pointer at {},{}", x, y);
        let x = x as f32;
        let y = y as f32;
        if let Some(config) = &mut model.backend_config {
            if let Some((a, b, c, d)) = &mut config.region_of_interest_mut() {
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
            } else {
                match model.editing_corners {
                    EditingCorner::None => {}
                    _ => {
                        warn!("No ROI, create a new one with some default points",);
                        let (x, y) = (0.0, 0.);
                        let distance = 1000.;
                        config.region_of_interest = Some((
                            ConfigRectCornerPoint::new(0, x, y),
                            ConfigRectCornerPoint::new(1, x + distance, y),
                            ConfigRectCornerPoint::new(2, x + distance, y + distance),
                            ConfigRectCornerPoint::new(3, x, y + distance),
                        ));
                    }
                }
            }
        }
    }
}
