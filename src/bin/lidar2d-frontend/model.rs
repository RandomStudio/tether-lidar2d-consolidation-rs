use std::{thread, time::Duration};

use log::{error, info};
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent, TetherAgentOptionsBuilder};
use tether_lidar2d_consolidation::{settings::Cli, tracking_config::TrackingConfig};

use clap::Parser;

pub struct Model {
    pub tether_agent: TetherAgent,
    config_input: PlugDefinition,
    tracking_config: Option<TrackingConfig>,
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

        Model {
            tether_agent,
            config_input,
            tracking_config: None,
        }
    }
}

impl eframe::App for Model {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let mut work_done = false;
        while let Some((topic, msg)) = &self.tether_agent.check_messages() {
            work_done = true;

            if self.config_input.matches(topic) {
                if let Ok(tracking_config) = rmp_serde::from_slice(msg.payload()) {
                    info!("Got new Tracking Config: {:?}", tracking_config);
                    self.tracking_config = Some(tracking_config);
                } else {
                    error!("Error reading new config");
                }
            }
        }

        egui::CentralPanel::default().show(ctx, |ui| match &self.tracking_config {
            None => {
                ui.label("No config received (yet)");
            }
            Some(tracking_config) => {
                for device in tracking_config.devices().iter() {
                    ui.group(|ui| {
                        ui.heading(&device.name);
                        ui.label(format!("Serial# {}", &device.serial));
                    });
                }
            }
        });

        if !work_done {
            thread::sleep(Duration::from_millis(1));
        }
    }
}
