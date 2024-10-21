use crate::model::Model;
use egui::{plot::Plot, Color32, Ui};

use super::tracked_points_to_plot_points;

pub fn render_tracking_graph(model: &mut Model, ui: &mut Ui) {
    ui.heading("boo");
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
    });
}
