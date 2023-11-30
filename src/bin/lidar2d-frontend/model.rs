use std::{thread, time::Duration};

use egui::{
    plot::{self, MarkerShape, Plot, PlotPoints, Points},
    Color32,
};
use log::{error, info};
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent, TetherAgentOptionsBuilder};
use tether_lidar2d_consolidation::{settings::Cli, tracking_config::TrackingConfig, Point2D};

use clap::Parser;

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
    scans: Vec<(f32, f32)>,
    is_editing: bool,
}

impl Default for Model {
    fn default() -> Self {
        let cli = Cli::parse();

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
            scans: Vec::new(),
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
                if let Ok(scans) = rmp_serde::from_slice(msg.payload()) {
                    self.scans = scans;
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
                        ui.label(format!("Serial# {}", &device.serial));
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

            let points = scans_to_plot_points(&self.scans, 5.0, Color32::RED);

            markers_plot.show(ui, |plot_ui| plot_ui.points(points));
            // egui::Window::new("Graph Area").show(ctx, |ui| {
            //     ui.heading("Graph");
            // });
        });

        if !work_done {
            thread::sleep(Duration::from_millis(1));
        }
    }
}

fn scans_to_plot_points(measurements: &[Point2D], size: f32, color: Color32) -> Points {
    let plot_points = PlotPoints::new(
        measurements
            .iter()
            .map(|(angle, distance)| {
                let x = angle.to_radians().cos() * distance;
                let y = angle.to_radians().sin() * distance;
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
