mod scan_graph;
mod tracking_graph;
mod tracking_settings;

use std::f64::consts::TAU;

use egui::{
    plot::{Line, MarkerShape, PlotPoints, Points},
    remap, Color32, Slider,
};

use scan_graph::render_scan_graph;
use tether_lidar2d_consolidation::{geometry_utils::distance, tracking::TrackedPoint2D, Point2D};
use tracking_graph::render_tracking_graph;
use tracking_settings::render_tracking_settings;

use crate::model::Model;

pub const SPACING_AMOUNT: f32 = 16.0;

pub fn render_ui(ctx: &egui::Context, model: &mut Model) {
    egui::SidePanel::left("config").show(ctx, |ui| {
        ui.heading("Visualisation Settings");
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.label("Point radius");
                ui.add(Slider::new(&mut model.point_size, 1.0..=20.0));
            });
        });

        ui.add_space(SPACING_AMOUNT);

        render_tracking_settings(model, ui);
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
        if let Some(tracking_config) = &model.backend_config {
            ui.heading("Tracking Config");
            if tracking_config.smoothing_use_real_units {
                if let Some(roi) = tracking_config.region_of_interest() {
                    ui.horizontal(|ui| {
                        ui.label("Output width:");
                        let (a, b, _c, _d) = roi;
                        ui.label(format!("{:.1}mm", distance(a.x, a.y, b.x, b.y)))
                    });
                    ui.horizontal(|ui| {
                        ui.label("Output height:");
                        let (a, _b, _c, d) = roi;
                        ui.label(format!("{:.1}mm", distance(a.x, a.y, d.x, d.y)))
                    });
                }
            } else {
                ui.label("Normalised 1x1 output size");
            }
        }
    });

    egui::CentralPanel::default().show(ctx, |ui| {
        ui.heading("Scan Area");
        render_scan_graph(model, ui);

        ui.heading("Tracking");
        render_tracking_graph(model, ui);
    });
}

/// NB: measurements Point2D are (angle,distance) not (x,y)
pub fn angle_samples_to_plot_points(
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
