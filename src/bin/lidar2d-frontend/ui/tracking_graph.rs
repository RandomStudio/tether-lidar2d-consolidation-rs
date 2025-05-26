use crate::model::Model;
use egui::{
    plot::{Plot, PlotPoint, Text},
    Color32, Ui,
};
use quad_to_quad_transformer::DEFAULT_DST_QUAD;

use super::{draw_circle, draw_line};

pub fn render_tracking_graph(model: &mut Model, ui: &mut Ui) {
    let tracker_plot = Plot::new("tracker_plot")
        .data_aspect(1.0)
        .include_x(-1.)
        .include_x(3.0)
        .include_y(-1.)
        .include_y(2.0)
        .auto_bounds_x()
        .auto_bounds_y();

    let light_green = Color32::from_rgba_unmultiplied(200, 255, 200, 64);

    tracker_plot.show(ui, |plot_ui| {
        // let mut all_points = Vec::new();

        let config = &model.backend_config.as_ref();
        let radius = {
            if let Some(c) = config {
                c.smoothing_merge_radius
            } else {
                12.0
            }
        };

        for (x, y) in &model.raw_tracked_points {
            plot_ui.line(draw_circle(*x, *y, radius, Color32::DARK_GRAY));
        }

        let text_offset = {
            if let Some(c) = config {
                if c.smoothing_use_real_units {
                    Some(c.smoothing_merge_radius)
                } else {
                    Some(0.05)
                }
            } else {
                None
            }
        };

        if model.show_graph_labels {
            for p in &model.smoothed_tracked_points {
                if let Some(heading) = p.bearing {
                    plot_ui.text(Text::new(
                        PlotPoint::new(p.x, p.y - text_offset.unwrap_or_default()),
                        format!("{:.0}°", heading),
                    ));
                }
                plot_ui.text(
                    Text::new(PlotPoint::new(p.x, p.y), format!("#{}", p.id()))
                        .color(Color32::WHITE),
                );
                if let Some(range) = p.range {
                    plot_ui.text(Text::new(
                        PlotPoint::new(p.x, p.y + text_offset.unwrap_or_default()),
                        if let Some(c) = config {
                            if c.smoothing_use_real_units {
                                format!("{:.0}mm", range)
                            } else {
                                format!("{:.2}", range)
                            }
                        } else {
                            format!("{:.0}mm", range)
                        },
                    ));
                }
                // "Bearing" line from origin
                plot_ui.line(draw_line(0., 0., p.x, p.y).color(light_green));

                // Current position of smoothed point
                plot_ui.line(draw_circle(
                    p.x,
                    p.y,
                    p.size.unwrap_or(500.) / 2.0,
                    light_green,
                ));
            }
        }

        if let Some(tracking_config) = &model.backend_config {
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
