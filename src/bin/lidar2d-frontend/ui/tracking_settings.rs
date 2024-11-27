use colorsys::Rgb;
use egui::{Checkbox, Color32, RichText, Slider, Ui};
use log::{debug, warn};
use tether_lidar2d_consolidation::automasking::AutoMaskMessage;

use crate::model::{EditingCorner, Model};

pub fn render_tracking_settings(model: &mut Model, ui: &mut Ui) {
    ui.heading("Tracking Configuration");
    ui.group(|ui| match &mut model.backend_config {
        None => {
            ui.label("No config received (yet)");
        }
        Some(tracking_config) => {
            ui.heading("Automasking");
            ui.horizontal(|ui| {
                if ui.button("New auto-calibration").clicked() {
                    model
                        .tether_agent
                        .encode_and_publish(
                            &model.outputs.request_automask,
                            AutoMaskMessage {
                                r#type: "new".into(),
                            },
                        )
                        .expect("failed to publish automask command");
                }
                if ui.button("Clear calibration").clicked() {
                    model
                        .tether_agent
                        .encode_and_publish(
                            &model.outputs.request_automask,
                            AutoMaskMessage {
                                r#type: "clear".into(),
                            },
                        )
                        .expect("failed to publish automask command");
                }
            });
            ui.separator();

            ui.heading("Tracking region (ROI)");
            ui.horizontal(|ui| {
                if ui
                    .selectable_label(matches!(model.editing_corners, EditingCorner::None), "None")
                    .clicked()
                {
                    model.editing_corners = EditingCorner::None;
                    model.is_editing = true;
                };
                if ui
                    .selectable_label(matches!(model.editing_corners, EditingCorner::A), "A")
                    .clicked()
                {
                    model.is_editing = true;
                    model.editing_corners = EditingCorner::A
                };
                if ui
                    .selectable_label(matches!(model.editing_corners, EditingCorner::B), "B")
                    .clicked()
                {
                    model.is_editing = true;
                    model.editing_corners = EditingCorner::B
                };
                if ui
                    .selectable_label(matches!(model.editing_corners, EditingCorner::C), "C")
                    .clicked()
                {
                    model.is_editing = true;
                    model.editing_corners = EditingCorner::C
                };
                if ui
                    .selectable_label(matches!(model.editing_corners, EditingCorner::D), "D")
                    .clicked()
                {
                    model.is_editing = true;
                    model.editing_corners = EditingCorner::D
                };
            });

            ui.separator();
            ui.heading("LIDAR Devices");
            let mut delete_index: Option<usize> = None;
            for (index, device) in tracking_config.devices_mut().iter_mut().enumerate() {
                ui.group(|ui| {
                    if model.is_editing {
                        ui.text_edit_singleline(&mut device.name);
                    } else {
                        ui.heading(&device.name);
                    }
                    ui.horizontal(|ui| {
                        ui.label("Delete");
                        if ui.button("🗑").clicked() {
                            warn!("Deleting {}", &device.name);
                            delete_index = Some(index);
                        }
                    });

                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Serial #");
                        ui.label(&device.serial);
                    });
                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Rotation");
                        if ui
                            .add(Slider::new(&mut device.rotation, 0. ..=360.))
                            .changed()
                        {
                            model.is_editing = true;
                        };
                    });
                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Offset X");
                        if ui
                            .add(Slider::new(&mut device.x, -10000. ..=10000.))
                            .changed()
                        {
                            model.is_editing = true;
                        };
                    });
                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Offset Y");
                        if ui
                            .add(Slider::new(&mut device.y, -10000. ..=10000.))
                            .changed()
                        {
                            model.is_editing = true;
                        };
                    });
                    ui.end_row();

                    ui.horizontal(|ui| {
                        let mut rgb: [u8; 3] = Rgb::from_hex_str(&device.colour).unwrap().into();

                        if ui.color_edit_button_srgb(&mut rgb).changed() {
                            debug!("Change device colour to {:?}", rgb);
                            model.is_editing = true;
                            let new_rgb = Rgb::from(rgb);
                            device.colour = new_rgb.to_hex_string();
                        }
                    });

                    ui.end_row();
                    let (current_flip_x, current_flip_y) = device.flip_coords.unwrap_or((1, 1));
                    ui.horizontal(|ui| {
                        let mut flip_x_checked = current_flip_x != 1;
                        if ui
                            .add(Checkbox::new(&mut flip_x_checked, "Flip X"))
                            .clicked()
                        {
                            model.is_editing = true;
                            let new_flip_x: i8 = if flip_x_checked { -1 } else { 1 };
                            device.flip_coords = Some((new_flip_x, current_flip_y));
                        };

                        let mut flip_y_checked = current_flip_y != 1;
                        if ui
                            .add(Checkbox::new(&mut flip_y_checked, "Flip Y"))
                            .clicked()
                        {
                            model.is_editing = true;
                            let new_flip_y: i8 = if flip_y_checked { -1 } else { 1 };
                            device.flip_coords = Some((current_flip_x, new_flip_y));
                        };
                    });
                });
            }

            let mut should_publish_update = false;

            if let Some(index) = delete_index {
                warn!("Deleting device in list with index {}...", index);
                tracking_config.devices_mut().remove(index);
                should_publish_update = true;
            }

            ui.separator();
            ui.heading("External Trackers");
            for t in tracking_config.external_trackers_mut().iter_mut() {
                ui.group(|ui| {
                    if model.is_editing {
                        ui.text_edit_singleline(&mut t.name);
                    } else {
                        ui.heading(&t.name);
                    }
                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Serial #");
                        ui.label(&t.serial);
                    });
                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Rotation");
                        if ui.add(Slider::new(&mut t.rotation, 0. ..=360.)).changed() {
                            model.is_editing = true;
                            should_publish_update = true;
                        };
                    });
                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Offset X");
                        if ui.add(Slider::new(&mut t.x, -10000. ..=10000.)).changed() {
                            model.is_editing = true;
                            should_publish_update = true;
                        };
                    });
                    ui.end_row();
                    ui.horizontal(|ui| {
                        ui.label("Offset Y");
                        if ui.add(Slider::new(&mut t.y, -10000. ..=10000.)).changed() {
                            model.is_editing = true;
                            should_publish_update = true;
                        };
                    });
                    ui.end_row();
                    let (current_flip_x, current_flip_y) = t.flip_coords.unwrap_or((1, 1));
                    ui.horizontal(|ui| {
                        let mut flip_x_checked = current_flip_x != 1;
                        if ui
                            .add(Checkbox::new(&mut flip_x_checked, "Flip X"))
                            .clicked()
                        {
                            model.is_editing = true;
                            let new_flip_x: i8 = if flip_x_checked { -1 } else { 1 };
                            t.flip_coords = Some((new_flip_x, current_flip_y));
                        };

                        let mut flip_y_checked = current_flip_y != 1;
                        if ui
                            .add(Checkbox::new(&mut flip_y_checked, "Flip Y"))
                            .clicked()
                        {
                            model.is_editing = true;
                            let new_flip_y: i8 = if flip_y_checked { -1 } else { 1 };
                            t.flip_coords = Some((current_flip_x, new_flip_y));
                        };
                    });
                });
            }

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
                model
                    .tether_agent
                    .encode_and_publish(&model.outputs.config, &model.backend_config)
                    .expect("failed to publish config");
            }
        }
    });
}
