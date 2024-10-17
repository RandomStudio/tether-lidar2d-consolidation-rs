use std::{collections::HashMap, thread, time::Duration};

use log::{debug, error, info};
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent, TetherAgentOptionsBuilder};
use tether_lidar2d_consolidation::{
    clustering::Cluster2D, tracking::TrackedPoint2D, tracking_config::TrackingConfig,
};

use crate::ui::render_ui;

// use clap::Parser;

pub struct Inputs {
    pub config: PlugDefinition,
    pub scans: PlugDefinition,
    pub clusters: PlugDefinition,
    pub tracked_points: PlugDefinition,
}

pub struct Outputs {
    pub config: PlugDefinition,
}

pub struct Model {
    pub tether_agent: TetherAgent,
    pub inputs: Inputs,
    pub outputs: Outputs,
    pub tracking_config: Option<TrackingConfig>,
    pub scans: HashMap<String, Vec<(f32, f32)>>,
    pub clusters: Vec<Cluster2D>,
    pub tracked_points: Vec<TrackedPoint2D>,
    pub point_size: f32,
    pub graph_y_flip: bool,
    pub is_editing: bool,
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

        let clusters = PlugOptionsBuilder::create_input("clusters")
            .build(&tether_agent)
            .expect("failed to create Input Plug");

        let tracked_points = PlugOptionsBuilder::create_input("trackedPoints")
            .build(&tether_agent)
            .expect("failed to create Input Plug");

        Model {
            tether_agent,
            inputs: Inputs {
                config: config_input,
                scans,
                clusters,
                tracked_points,
            },
            outputs: Outputs {
                config: config_output,
            },
            tracking_config: None,
            is_editing: false,
            scans: HashMap::new(),
            clusters: Vec::new(),
            tracked_points: Vec::new(),
            point_size: 2.5,
            graph_y_flip: true,
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
                    debug!("Got new Tracking Config: {:?}", tracking_config);
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

            if self.inputs.clusters.matches(topic) {
                if let Ok(clusters) = rmp_serde::from_slice::<Vec<Cluster2D>>(msg.payload()) {
                    self.clusters = clusters;
                }
            }

            if self.inputs.tracked_points.matches(topic) {
                if let Ok(tracked_points) =
                    rmp_serde::from_slice::<Vec<TrackedPoint2D>>(msg.payload())
                {
                    self.tracked_points = tracked_points;
                }
            }
        }

        render_ui(ctx, self);

        if !work_done {
            thread::sleep(Duration::from_millis(1));
        }
    }
}
