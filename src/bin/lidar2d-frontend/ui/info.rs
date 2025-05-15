use egui::{plot::Plot, Color32, Grid, RichText, Stroke, Ui};
use nalgebra::Vector2;
use tether_lidar2d_consolidation::{geometry_utils::distance, systems::movement::calculate};

use crate::model::Model;

use super::draw_line;

const INCLUDE_RANGE: f32 = 1000.;

pub fn render_info(model: &mut Model, ui: &mut Ui) {
    if let Some(tracking_config) = &model.backend_config {
        ui.heading("Tracking ROI");
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

        if tracking_config.enable_average_movement {
            ui.separator();
            ui.heading("Average Movement");

            let (mx, my) = calculate(&model.smoothed_tracked_points);

            let plot = Plot::new("averageMovement")
                .height(ui.available_width())
                .include_x(INCLUDE_RANGE)
                .include_x(-INCLUDE_RANGE)
                .include_y(INCLUDE_RANGE)
                .include_y(-INCLUDE_RANGE);
            plot.show(ui, |plot_ui| {
                plot_ui
                    .line(draw_line(0., 0., mx, my).stroke(Stroke::new(2.5, Color32::LIGHT_GREEN)));
            });

            let magnitude = Vector2::new(mx, my).magnitude();
            ui.horizontal(|ui| {
                ui.label("Magnitude (speed)");
                ui.label(RichText::new(format!("{:.0}mm/s", magnitude)));
            });
        }
    }

    ui.separator();

    Grid::new("tracking_grid").show(ui, |ui| {
        ui.label("Clusters count: ");
        ui.label(format!("{}", model.clusters.len()));
        ui.end_row();

        ui.label("(Raw) tracked points count: ");
        ui.label(format!("{}", model.raw_tracked_points.len()));
        ui.end_row();

        ui.label("Smoothed tracked points count: ");
        ui.label(format!("{}", model.smoothed_tracked_points.len()));
        ui.end_row();

        if let Some(tracking_config) = &model.backend_config {
            for (i, p) in model.smoothed_tracked_points.iter().enumerate() {
                ui.label(format!("{}:", i));
                ui.label(format!("#{}", p.id()));
                if tracking_config.enable_velocity {
                    if let Some([x, y]) = p.velocity {
                        let speed = (x.abs() + y.abs()) / 2.0 / 1000.;
                        ui.label(format!("@ {:.2} m/s", speed));
                    }
                }
                ui.end_row();
            }
        }
    });
}
