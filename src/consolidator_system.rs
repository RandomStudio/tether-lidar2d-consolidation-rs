use std::collections::HashMap;

use quad_to_quad_transformer::QuadTransformer;
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent};

use crate::{
    automasking::AutoMaskSamplerMap,
    clustering::ClusteringSystem,
    movement::MovementAnalysis,
    presence::PresenceDetectionZones,
    settings::Cli,
    smoothing::{self, get_mode, SmoothSettings, TrackingSmoother},
    tracking_config::TrackingConfig,
};

pub struct Outputs {
    pub config_output: PlugDefinition,
    pub clusters_output: PlugDefinition,
    pub tracking_output: PlugDefinition,
    pub smoothed_tracking_output: PlugDefinition,
    pub movement_output: PlugDefinition,
}

impl Outputs {
    pub fn new(tether_agent: &TetherAgent) -> Outputs {
        let config_output = PlugOptionsBuilder::create_output("provideLidarConfig")
            .qos(Some(2))
            .retain(Some(true))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        // Clusters, tracking outputs
        let tracking_output = PlugOptionsBuilder::create_output("trackedPoints")
            .qos(Some(0))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        let clusters_output = PlugOptionsBuilder::create_output("clusters")
            .qos(Some(0))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        // Smoothed tracked points output
        let smoothed_tracking_output = PlugOptionsBuilder::create_output("smoothedTrackedPoints")
            .qos(Some(1))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        // Movement vector output
        let movement_output = PlugOptionsBuilder::create_output("movement")
            .build(tether_agent)
            .expect("failed to create Output Plug");

        Outputs {
            config_output,
            tracking_output,
            clusters_output,
            smoothed_tracking_output,
            movement_output,
        }
    }
}

pub struct Inputs {
    pub scans_input: PlugDefinition,
    pub save_config_input: PlugDefinition,
    pub request_automask_input: PlugDefinition,
    pub external_tracking_input: PlugDefinition,
}

impl Inputs {
    pub fn new(tether_agent: &TetherAgent) -> Inputs {
        // Some subscriptions
        let scans_input = PlugOptionsBuilder::create_input("scans")
            .qos(Some(0))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        let save_config_input = PlugOptionsBuilder::create_input("saveLidarConfig")
            .qos(Some(2))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        let request_automask_input = PlugOptionsBuilder::create_input("requestAutoMask")
            .qos(Some(2))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        // TODO: the name of this input plug should be customisable
        let external_tracking_input = PlugOptionsBuilder::create_input("bodyFrames")
            .qos(Some(2))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        Inputs {
            scans_input,
            save_config_input,
            request_automask_input,
            external_tracking_input,
        }
    }
}

pub struct Systems {
    pub clustering_system: ClusteringSystem,
    pub perspective_transformer: QuadTransformer,
    pub smoothing_system: TrackingSmoother,
    pub automask_samplers: AutoMaskSamplerMap,
    pub presence_detector: PresenceDetectionZones,
    pub movement_analysis: MovementAnalysis,
}

impl Systems {
    pub fn new(cli: &Cli, tracking_config: &TrackingConfig) -> Systems {
        let clustering_system = ClusteringSystem::new(
            cli.clustering_neighbourhood_radius,
            cli.clustering_min_neighbours,
            cli.clustering_max_cluster_size,
        );

        let perspective_transformer = QuadTransformer::new(
            match tracking_config.region_of_interest() {
                Some(region_of_interest) => {
                    let (c1, c2, c3, c4) = region_of_interest;
                    let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                    Some(corners)
                }
                None => None,
            },
            {
                if cli.transform_include_outside {
                    None
                } else {
                    Some(cli.transform_ignore_outside_margin)
                }
            },
        );

        let smoothing_system = TrackingSmoother::new(SmoothSettings {
            merge_radius: cli.smoothing_merge_radius,
            wait_before_active_ms: cli.smoothing_wait_before_active_ms,
            expire_ms: cli.smoothing_expire_ms,
            lerp_factor: cli.smoothing_lerp_factor,
            // TODO: set this from CLI
            empty_list_send_mode: get_mode(&cli.smoothing_empty_send_mode)
                .unwrap_or(smoothing::EmptyListSendMode::Once),
        });

        let presence_detector =
            PresenceDetectionZones::new(tracking_config.zones().unwrap_or_default());

        Systems {
            clustering_system,
            smoothing_system,
            automask_samplers: HashMap::new(),
            perspective_transformer,
            presence_detector,
            movement_analysis: MovementAnalysis::new(),
        }
    }
}
