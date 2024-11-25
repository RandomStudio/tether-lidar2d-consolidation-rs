use crate::model::Model;
use egui::{
    plot::{MarkerShape, Plot, PlotPoint, PlotPoints, Points, Text},
    Color32, Ui,
};
use tether_lidar2d_consolidation::consolidator_system::calculate_dst_quad;

use super::{draw_line, raw_tracked_points_to_plot_points, smoothed_tracked_points_to_plot_points};

pub fn render_tracking_graph(model: &mut Model, ui: &mut Ui) {
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

        let raw_points = raw_tracked_points_to_plot_points(
            &model.raw_tracked_points,
            12.0,
            Color32::from_rgba_unmultiplied(200, 200, 200, 128),
        );
        all_points.push(raw_points);

        let smoothed_points = smoothed_tracked_points_to_plot_points(
            &model.smoothed_tracked_points,
            5.0,
            Color32::LIGHT_GREEN,
        );
        all_points.push(smoothed_points);

        for p in &model.smoothed_tracked_points {
            plot_ui.text(
                Text::new(PlotPoint::new(p.x + 0.1, p.y), format!("#{}", p.id()))
                    .color(Color32::WHITE),
            );
        }

        for points_group in all_points {
            plot_ui.points(points_group);
        }

        if let Some(tracking_config) = &model.tracking_config {
            if tracking_config.use_real_units() {
                if let Some(roi) = tracking_config.region_of_interest() {
                    let dst_quad = calculate_dst_quad(roi);

                    let [a, b, c, d] = dst_quad;

                    let line1 = draw_line(a.0, a.1, b.0, b.1);
                    plot_ui.line(line1.color(Color32::RED));
                    // TODO: vertical lines seem to disappear; hence the strange offset here
                    let line2 = draw_line(b.0, b.1, c.0 + 0.01, c.1);
                    plot_ui.line(line2.color(Color32::RED));
                    let line3 = draw_line(c.0, c.1, d.0, d.1);
                    plot_ui.line(line3.color(Color32::RED));
                    let line4 = draw_line(d.0, d.1, a.0 + 0.01, a.1);
                    plot_ui.line(line4.color(Color32::RED));
                }
            } else {
                let line1 = draw_line(0., 0., 1.0, 0.);
                plot_ui.line(line1.color(Color32::RED));
                // TODO: vertical lines seem to disappear; hence the strange offset here
                let line2 = draw_line(1.0, 0., 1.0 + 0.01, 1.0);
                plot_ui.line(line2.color(Color32::RED));
                let line3 = draw_line(1.0, 1.0, 0., 1.0);
                plot_ui.line(line3.color(Color32::RED));
                let line4 = draw_line(0., 1.0, 0. + 0.01, 0.);
                plot_ui.line(line4.color(Color32::RED));
            }
        }
    });
}
