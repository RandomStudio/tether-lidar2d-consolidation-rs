use crate::model::Model;
use egui::{
    plot::{Plot, PlotPoint, Text},
    Color32, Ui,
};
use quad_to_quad_transformer::DEFAULT_DST_QUAD;

use super::{draw_circle, draw_line, smoothed_tracked_points_to_plot_points};

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

        let config = &model.backend_config.as_ref();
        let radius = {
            if let Some(c) = config {
                c.smoothing_merge_radius
            } else {
                12.0
            }
        };

        // let raw_points = raw_tracked_points_to_plot_points(
        //     &model.raw_tracked_points,
        //     radius,
        //     Color32::from_rgba_unmultiplied(200, 200, 200, 128),
        // );
        // all_points.push(raw_points);

        for (x, y) in &model.raw_tracked_points {
            plot_ui.line(draw_circle(
                *x,
                *y,
                radius,
                Color32::from_rgba_unmultiplied(200, 200, 200, 128),
            ));
        }

        let smoothed_points = smoothed_tracked_points_to_plot_points(
            &model.smoothed_tracked_points,
            5.0,
            Color32::LIGHT_GREEN,
        );
        all_points.push(smoothed_points);

        for p in &model.smoothed_tracked_points {
            plot_ui.text(
                Text::new(PlotPoint::new(p.x, p.y), format!("#{}", p.id())).color(Color32::WHITE),
            );
        }

        for points_group in all_points {
            plot_ui.points(points_group);
        }

        if let Some(tracking_config) = &model.backend_config {
            // if let Some(dst_quad) = model.calculated_dst_quad {
            //     let remapped_origin_location: Point2D =
            //         point_remap_from_origin((0., 0.), tracking_config.origin_location, dst_quad);

            //     plot_ui.points(
            //         Points::new(vec![[
            //             remapped_origin_location.0 as f64,
            //             remapped_origin_location.1 as f64,
            //         ]])
            //         .filled(true)
            //         .radius(10.)
            //         .shape(MarkerShape::Circle)
            //         .color(Color32::DARK_RED),
            //     )
            // }

            if tracking_config.smoothing_use_real_units {
                let dst_quad = model.calculated_dst_quad.unwrap_or(DEFAULT_DST_QUAD);

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
