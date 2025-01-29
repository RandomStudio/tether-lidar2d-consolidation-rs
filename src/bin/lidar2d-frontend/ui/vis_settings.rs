use egui::{Slider, Ui};

use crate::model::Model;

pub fn render_vis_settings(model: &mut Model, ui: &mut Ui) {
    ui.heading("Visualisation Settings");
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.label("Point radius");
            ui.add(Slider::new(&mut model.point_size, 1.0..=20.0));
        });
        ui.checkbox(&mut model.show_graph_labels, "Show all graph text");
    });
}
