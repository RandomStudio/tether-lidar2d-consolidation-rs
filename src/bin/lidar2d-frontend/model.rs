use std::{thread, time::Duration};

use colors_transform::{Color, Rgb};
use egui::{
    epaint::ahash::HashMap,
    plot::{MarkerShape, Plot, PlotPoints, Points},
    Color32, Slider,
};
use log::{error, info};
use tether_agent::{
    three_part_topic::parse_agent_id, PlugDefinition, PlugOptionsBuilder, TetherAgent,
    TetherAgentOptionsBuilder,
};
use tether_lidar2d_consolidation::{tracking_config::TrackingConfig, Point2D};

// use clap::Parser;

struct Inputs {
    config: PlugDefinition,
    scans: PlugDefinition,
}

struct Outputs {
    config: PlugDefinition,
}

pub struct Model {
    pub tether_agent: TetherAgent,
    inputs: Inputs,
    outputs: Outputs,
    tracking_config: Option<TrackingConfig>,
    scans: HashMap<String, Vec<(f32, f32)>>,
    is_editing: bool,
}

impl Default for Model {
    fn default() -> Self {
        // let cli = Cli::parse();

        let tether_agent = TetherAgentOptionsBuilder::new("lidar2dFrontend")
            .build()
            .expect("failed to init+connect Tether Agent");

        info!("Lidar2D Frontend started OK");

        let config_input = PlugOptionsBuilder::create_input("provideLidarConfig")
            .build(&tether_agent)
            .expect("failed to create Input Plug");

        let scans = PlugOptionsBuilder::create_input("scans")
            .build(&tether_agent)
            .expect("failed to create Input Plug");

        let config_output = PlugOptionsBuilder::create_output("saveLidarConfig")
            .build(&tether_agent)
            .expect("failed to create Output Plug");

        Model {
            tether_agent,
            inputs: Inputs {
                config: config_input,
                scans,
            },
            outputs: Outputs {
                config: config_output,
            },
            tracking_config: None,
            is_editing: false,
            scans: HashMap::default(),
        }
    }
}

impl eframe::App for Model {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();

        let mut work_done = false;
        while let Some((topic, msg)) = &self.tether_agent.check_messages() {
            work_done = true;

            if self.inputs.config.matches(topic) {
                if let Ok(tracking_config) = rmp_serde::from_slice(msg.payload()) {
                    info!("Got new Tracking Config: {:?}", tracking_config);
                    self.tracking_config = Some(tracking_config);
                } else {
                    error!("Error reading new config");
                }
            }

            if self.inputs.scans.matches(topic) {
                if let Ok(scans) = rmp_serde::from_slice::<Vec<(f32, f32)>>(msg.payload()) {
                    // self.scans = scans;
                    let serial_number = match topic {
                        tether_agent::TetherOrCustomTopic::Tether(t) => t.id(),
                        tether_agent::TetherOrCustomTopic::Custom(t) => {
                            error!("Could not retrieve serial number from topic {}", t);
                            "unknown"
                        }
                    };
                    self.scans.insert(serial_number.into(), scans);
                }
            }
        }

        egui::SidePanel::left("config").show(ctx, |ui| match &mut self.tracking_config {
            None => {
                ui.label("No config received (yet)");
            }
            Some(tracking_config) => {
                for device in tracking_config.devices_mut().iter_mut() {
                    ui.group(|ui| {
                        if self.is_editing {
                            ui.text_edit_singleline(&mut device.name);
                        } else {
                            ui.heading(&device.name);
                        }
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Serial #");
                            ui.label(&device.serial);
                        });
                        ui.end_row();
                        ui.horizontal(|ui| {
                            ui.label("Rotation");
                            ui.add(Slider::new(&mut device.rotation, 0. ..=360.));
                        });

                        // ui.columns(2, |columns| {
                        //     columns[0].label("Serial #");
                        //     columns[1].label(&device.serial);
                        // });
                        // ui.label(format!("Serial# {}", &device.serial));
                    });
                }
                if self.is_editing {
                    if ui.button("Save 🖴").clicked() {
                        self.tether_agent
                            .encode_and_publish(&self.outputs.config, &self.tracking_config)
                            .expect("failed to publish config");
                        self.is_editing = false;
                    }
                } else {
                    if ui.button("Edit ✏").clicked() {
                        self.is_editing = true;
                    }
                }
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Graph Area");
            let markers_plot = Plot::new("scans").data_aspect(1.0);

            let mut all_points = Vec::new();

            if let Some(tracking_config) = &self.tracking_config {
                for device in tracking_config.devices() {
                    let rgb: Rgb = Rgb::from_hex_str(&device.color).unwrap();
                    let (r, g, b) = (
                        rgb.get_red() as u8,
                        rgb.get_blue() as u8,
                        rgb.get_green() as u8,
                    );
                    if let Some(scans_this_device) = self.scans.get(&device.serial) {
                        let points = scans_to_plot_points(
                            scans_this_device,
                            5.0,
                            Color32::from_rgb(r, g, b),
                            device.rotation,
                        );
                        all_points.push(points);
                    }
                }
            }

            markers_plot.show(ui, |plot_ui| {
                for points_group in all_points {
                    plot_ui.points(Points::from(points_group));
                }
            });
        });

        if !work_done {
            thread::sleep(Duration::from_millis(1));
        }
    }
}

fn scans_to_plot_points(
    measurements: &[Point2D],
    size: f32,
    color: Color32,
    rotate: f32,
) -> Points {
    let plot_points = PlotPoints::new(
        measurements
            .iter()
            .map(|(angle, distance)| {
                let x = (angle + rotate).to_radians().cos() * distance;
                let y = (angle + rotate).to_radians().sin() * distance;
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
