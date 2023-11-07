use std::collections::HashMap;

use log::info;
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent, TetherAgentOptionsBuilder};

use crate::{
    automasking::AutoMaskSamplerMap,
    clustering::ClusteringSystem,
    perspective::PerspectiveTransformer,
    presence::PresenceDetectionZones,
    settings::Cli,
    smoothing::{self, get_mode, SmoothSettings, TrackingSmoother},
    tracking_config::TrackingConfig,
};

// struct Outputs

pub struct ConsolidatorSystem {
    pub tether_agent: TetherAgent,

    // Some outputs (separate struct?)
    pub config_output: PlugDefinition,
    pub clusters_output: PlugDefinition,
    pub tracking_output: PlugDefinition,
    pub smoothed_tracking_output: PlugDefinition,

    // Some intputs (separate struct?)
    pub scans_input: PlugDefinition,
    pub save_config_input: PlugDefinition,
    pub request_config_input: PlugDefinition,
    pub request_automask_input: PlugDefinition,

    // Config, systems
    pub tracking_config: TrackingConfig,
    pub clustering_system: ClusteringSystem,
    pub perspective_transformer: PerspectiveTransformer,
    pub smoothing_system: TrackingSmoother,
    pub automask_samplers: AutoMaskSamplerMap,
    pub presence_detector: PresenceDetectionZones,
}

impl ConsolidatorSystem {
    pub fn new(cli: &Cli) -> Self {
        let tether_agent = TetherAgentOptionsBuilder::new(&cli.agent_role)
            .id(Some(&cli.agent_group))
            .host(Some(&cli.tether_host.to_string()))
            .build()
            .expect("failed to init and/or connect Tether Agent");

        let config_output = PlugOptionsBuilder::create_output("provideLidarConfig")
            .qos(Some(2))
            .build(&tether_agent)
            .expect("failed to create Output Plug");

        let mut tracking_config = TrackingConfig::new(&cli.config_path);
        match tracking_config.load_config_from_file() {
            Ok(count) => {
                info!("Loaded {} devices OK into Config", count);
                tether_agent
                    .encode_and_publish(&config_output, &tracking_config)
                    .expect("failed to publish config");
            }
            Err(()) => {
                panic!("Error loading devices into config manager!")
            }
        }

        // Clusters, tracking outputs
        let tracking_output = PlugOptionsBuilder::create_output("trackedPoints")
            .qos(Some(1))
            .build(&tether_agent)
            .expect("failed to create Output Plug");
        let clusters_output = PlugOptionsBuilder::create_output("clusters")
            .qos(Some(0))
            .build(&tether_agent)
            .expect("failed to create Output Plug");

        // Smoothed traacked points output
        let smoothed_tracking_output = PlugOptionsBuilder::create_output("trackedPoints")
            .qos(Some(1))
            .role(Some("trackingSmooth"))
            .build(&tether_agent)
            .expect("failed to create Output Plug");

        // Some subscriptions
        let scans_input = PlugOptionsBuilder::create_input("scans")
            .qos(Some(0))
            .build(&tether_agent)
            .expect("failed to create Output Plug");
        let save_config_input = PlugOptionsBuilder::create_input("saveLidarConfig")
            .qos(Some(2))
            .build(&tether_agent)
            .expect("failed to create Output Plug");
        let request_config_input = PlugOptionsBuilder::create_input("requestLidarConfig")
            .qos(Some(2))
            .build(&tether_agent)
            .expect("failed to create Output Plug");
        let request_automask_input = PlugOptionsBuilder::create_input("requestAutoMask")
            .qos(Some(2))
            .build(&tether_agent)
            .expect("failed to create Output Plug");

        let mut clustering_system = ClusteringSystem::new(
            cli.clustering_neighbourhood_radius,
            cli.clustering_min_neighbours,
            cli.clustering_max_cluster_size,
        );

        let mut perspective_transformer = PerspectiveTransformer::new(
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

        let mut smoothing_system = TrackingSmoother::new(SmoothSettings {
            merge_radius: cli.smoothing_merge_radius,
            wait_before_active_ms: cli.smoothing_wait_before_active_ms,
            expire_ms: cli.smoothing_expire_ms,
            lerp_factor: cli.smoothing_lerp_factor,
            // TODO: set this from CLI
            empty_list_send_mode: get_mode(&cli.smoothing_empty_send_mode)
                .unwrap_or(smoothing::EmptyListSendMode::Once),
        });

        let mut presence_detector =
            PresenceDetectionZones::new(tracking_config.zones().unwrap_or_default());

        ConsolidatorSystem {
            tether_agent,
            config_output,
            clusters_output,
            tracking_output,
            smoothed_tracking_output,
            scans_input,
            save_config_input,
            request_config_input,
            request_automask_input,
            tracking_config,
            clustering_system,
            perspective_transformer,
            smoothing_system,
            automask_samplers: HashMap::new(),
            presence_detector,
        }
    }
}
