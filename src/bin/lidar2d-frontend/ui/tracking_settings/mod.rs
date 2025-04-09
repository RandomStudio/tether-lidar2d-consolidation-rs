use common_backend_settings::render_common_backend_settings;
use device_settings::render_device_settings;
use egui::{Color32, RichText, Ui};
use log::debug;

use crate::model::Model;

use super::SPACING_AMOUNT;

mod common_backend_settings;
mod device_settings;

pub fn render_tracking_settings(model: &mut Model, ui: &mut Ui) {
    ui.heading("Tracking Configuration");
    let mut should_publish_update = false;

    egui::ScrollArea::vertical()
        .max_height(600.)
        .show(ui, |ui| match &mut model.backend_config {
            None => {
                ui.label("No config received (yet)");
            }
            Some(_) => {
                render_common_backend_settings(model, ui);

                ui.add_space(SPACING_AMOUNT);
                ui.separator();

                render_device_settings(model, ui, &mut should_publish_update);
            }
        });

    ui.separator();
    ui.heading("Edit/Save");

    if model.is_editing {
        if ui
            .button(
                RichText::new("Save 🖴")
                    .color(Color32::LIGHT_GREEN)
                    .size(16.0),
            )
            .clicked()
        {
            should_publish_update = true;
            model.is_editing = false;
        }
    } else if ui.button("Edit ✏").clicked() {
        model.is_editing = true;
    }

    // We publish the updated config via Tether (on the plug "saveLidarConfig"). This is picked up by the backend which in turn re-saves the
    // config file (JSON) and republishes the updated Config (on the plug "provideLidarConfig").
    if should_publish_update {
        debug!("Publish new backend config: {:?}", &model.backend_config);
        model
            .tether_agent
            .encode_and_send(&model.outputs.config, &model.backend_config)
            .expect("failed to publish config");
    }
}
