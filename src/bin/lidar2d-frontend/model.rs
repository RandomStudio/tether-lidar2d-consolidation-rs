use std::{collections::HashMap, thread, time::Duration};

use log::{debug, error, info};
use quad_to_quad_transformer::RectCorners;
use tether_agent::{
    tether_compliant_topic::TetherOrCustomTopic, ChannelDefinition, ChannelOptionsBuilder,
    TetherAgent, TetherAgentOptionsBuilder,
};
use tether_lidar2d_consolidation::{
    backend_config::BackendConfig,
    systems::{clustering::Cluster2D, position_remapping::calculate_dst_quad},
    tracking::TrackedPoint2D,
    Point2D,
};

use crate::ui::render_ui;

// use clap::Parser;

pub struct Inputs {
    pub config: ChannelDefinition,
    pub scans: ChannelDefinition,
    pub clusters: ChannelDefinition,
    pub raw_tracked_points: ChannelDefinition,
    pub smoothed_tracked_points: ChannelDefinition,
}

pub struct Outputs {
    pub config: ChannelDefinition,
    pub request_automask: ChannelDefinition,
}

#[derive(Debug)]
pub enum EditingCorner {
    None,
    A,
    B,
    C,
    D,
}

pub struct Model {
    pub tether_agent: TetherAgent,
    pub inputs: Inputs,
    pub outputs: Outputs,
    pub backend_config: Option<BackendConfig>,
    pub calculated_dst_quad: Option<RectCorners>,
    /// Warning: these scan values are (angle,distance) for LIDAR devices, and (x,y) for External Trackers!
    pub scans: HashMap<String, Vec<(f32, f32)>>,
    pub clusters: Vec<Cluster2D>,
    pub raw_tracked_points: Vec<Point2D>,
    pub smoothed_tracked_points: Vec<TrackedPoint2D>,
    pub editing_corners: EditingCorner,
    pub point_size: f32,
    pub show_graph_labels: bool,
    pub is_editing: bool,
}

impl Default for Model {
    fn default() -> Self {
        // let cli = Cli::parse();

        let mut tether_agent = TetherAgentOptionsBuilder::new("lidar2dFrontend")
            .build()
            .expect("failed to init+connect Tether Agent");

        info!("Lidar2D Frontend started OK");

        let config_input = ChannelOptionsBuilder::create_receiver("provideLidarConfig")
            .build(&mut tether_agent)
            .expect("failed to create Input Plug");

        let scans = ChannelOptionsBuilder::create_receiver("scans")
            .build(&mut tether_agent)
            .expect("failed to create Input Plug");

        let clusters = ChannelOptionsBuilder::create_receiver("clusters")
            .build(&mut tether_agent)
            .expect("failed to create Input Plug");

        let raw_tracked_points = ChannelOptionsBuilder::create_receiver("trackedPoints")
            .build(&mut tether_agent)
            .expect("failed to create Input Plug");

        let smoothed_tracked_points =
            ChannelOptionsBuilder::create_receiver("smoothedTrackedPoints")
                .build(&mut tether_agent)
                .expect("failed to create Input Plug");

        let config_output = ChannelOptionsBuilder::create_sender("saveLidarConfig")
            .build(&mut tether_agent)
            .expect("failed to create Output Plug");

        let request_automask = ChannelOptionsBuilder::create_sender("requestAutoMask")
            .build(&mut tether_agent)
            .expect("failed to create Output Plug");

        Model {
            tether_agent,
            inputs: Inputs {
                config: config_input,
                scans,
                clusters,
                raw_tracked_points,
                smoothed_tracked_points,
            },
            outputs: Outputs {
                config: config_output,
                request_automask,
            },
            backend_config: None,
            is_editing: false,
            scans: HashMap::new(),
            clusters: Vec::new(),
            raw_tracked_points: Vec::new(),
            smoothed_tracked_points: Vec::new(),
            editing_corners: EditingCorner::None,
            point_size: 2.0,
            show_graph_labels: true,
            calculated_dst_quad: None,
        }
    }
}

impl eframe::App for Model {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();

        let mut work_done = false;
        while let Some((topic, payload)) = &self.tether_agent.check_messages() {
            work_done = true;

            if self.inputs.config.matches(topic) {
                if let Ok(tracking_config) = rmp_serde::from_slice::<BackendConfig>(payload) {
                    debug!("Got new Tracking Config: {:?}", tracking_config);
                    if let Some(roi) = tracking_config.region_of_interest() {
                        self.calculated_dst_quad =
                            Some(calculate_dst_quad(roi, tracking_config.origin_location));
                    } else {
                        self.calculated_dst_quad = None;
                    }
                    self.backend_config = Some(tracking_config);
                } else {
                    error!("Error parsing new config");
                }
            }

            if self.inputs.scans.matches(topic) {
                if let Ok(scans) = rmp_serde::from_slice::<Vec<(f32, f32)>>(payload) {
                    // self.scans = scans;
                    let serial_number = match topic {
                        TetherOrCustomTopic::Tether(t) => t.id().unwrap_or("unknown"),
                        TetherOrCustomTopic::Custom(t) => {
                            error!("Could not retrieve serial number from topic {}", t);
                            "unknown"
                        }
                    };
                    self.scans.insert(serial_number.into(), scans);
                }
            }

            if self.inputs.clusters.matches(topic) {
                if let Ok(clusters) = rmp_serde::from_slice::<Vec<Cluster2D>>(payload) {
                    self.clusters = clusters;
                }
            }

            if self.inputs.raw_tracked_points.matches(topic) {
                if let Ok(tracked_points) = rmp_serde::from_slice::<Vec<Point2D>>(payload) {
                    self.raw_tracked_points = tracked_points;
                }
            }

            if self.inputs.smoothed_tracked_points.matches(topic) {
                if let Ok(tracked_points) = rmp_serde::from_slice::<Vec<TrackedPoint2D>>(payload) {
                    self.smoothed_tracked_points = tracked_points;
                }
            }
        }

        render_ui(ctx, self);

        if !work_done {
            thread::sleep(Duration::from_millis(1));
        }
    }
}
