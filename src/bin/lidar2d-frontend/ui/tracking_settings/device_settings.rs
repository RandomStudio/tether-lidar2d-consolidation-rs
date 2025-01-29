use colorsys::Rgb;
use egui::{Checkbox, Slider, Ui};
use log::{debug, warn};

use crate::model::Model;

pub fn render_device_settings(model: &mut Model, ui: &mut Ui, should_publish_update: &mut bool) {
    ui.heading("LIDAR Devices");

    match &mut model.backend_config {
        None => {}
        Some(backend_config) => {
            ui.horizontal(|ui| {
                ui.label("Clear all:");
                if ui.button("Clear ⏏").clicked() {
                    backend_config.devices_mut().clear();
                    *should_publish_update = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Default min distance threshold");
                if ui
                    .add(
                        Slider::new(
                            &mut backend_config.default_min_distance_threshold,
                            0. ..=2000.,
                        )
                        .suffix("mm"),
                    )
                    .changed()
                {
                    model.is_editing = true;
                }
            });

            ui.separator();
            let mut delete_index: Option<usize> = None;
            for (index, device) in backend_config.devices_mut().iter_mut().enumerate() {
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

            if let Some(index) = delete_index {
                warn!("Deleting device in list with index {}...", index);
                backend_config.devices_mut().remove(index);
                *should_publish_update = true;
            }

            // ui.separator();
            // ui.heading("External Trackers");
            // for t in backend_config.external_trackers_mut().iter_mut() {
            //     ui.group(|ui| {
            //         if model.is_editing {
            //             ui.text_edit_singleline(&mut t.name);
            //         } else {
            //             ui.heading(&t.name);
            //         }
            //         ui.end_row();
            //         ui.horizontal(|ui| {
            //             ui.label("Serial #");
            //             ui.label(&t.serial);
            //         });
            //         ui.end_row();
            //         ui.horizontal(|ui| {
            //             ui.label("Rotation");
            //             if ui.add(Slider::new(&mut t.rotation, 0. ..=360.)).changed() {
            //                 model.is_editing = true;
            //                 *should_publish_update = true;
            //             };
            //         });
            //         ui.end_row();
            //         ui.horizontal(|ui| {
            //             ui.label("Offset X");
            //             if ui.add(Slider::new(&mut t.x, -10000. ..=10000.)).changed() {
            //                 model.is_editing = true;
            //                 *should_publish_update = true;
            //             };
            //         });
            //         ui.end_row();
            //         ui.horizontal(|ui| {
            //             ui.label("Offset Y");
            //             if ui.add(Slider::new(&mut t.y, -10000. ..=10000.)).changed() {
            //                 model.is_editing = true;
            //                 *should_publish_update = true;
            //             };
            //         });
            //         ui.end_row();
            //         let (current_flip_x, current_flip_y) = t.flip_coords.unwrap_or((1, 1));
            //         ui.horizontal(|ui| {
            //             let mut flip_x_checked = current_flip_x != 1;
            //             if ui
            //                 .add(Checkbox::new(&mut flip_x_checked, "Flip X"))
            //                 .clicked()
            //             {
            //                 model.is_editing = true;
            //                 let new_flip_x: i8 = if flip_x_checked { -1 } else { 1 };
            //                 t.flip_coords = Some((new_flip_x, current_flip_y));
            //             };

            //             let mut flip_y_checked = current_flip_y != 1;
            //             if ui
            //                 .add(Checkbox::new(&mut flip_y_checked, "Flip Y"))
            //                 .clicked()
            //             {
            //                 model.is_editing = true;
            //                 let new_flip_y: i8 = if flip_y_checked { -1 } else { 1 };
            //                 t.flip_coords = Some((current_flip_x, new_flip_y));
            //             };
            //         });
            //     });
            // }
        }
    }
}
