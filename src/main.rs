use automasking::AutoMaskMessage;
use clap::Parser;
use tether_agent::three_part_topic::{build_topic, parse_agent_id};
use tracking_config::TrackingConfig;

use env_logger::Env;
use log::{debug, error, info};
use std::collections::HashMap;
use std::fmt::Error;
use std::thread;
use std::time::Duration;
use tether_agent::mqtt::Message;
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent, TetherAgentOptionsBuilder};

mod automasking;
mod clustering;
mod perspective;
mod presence;
mod settings;
mod smoothing;
mod tracking;
mod tracking_config;

use crate::automasking::AutoMaskSampler;
use crate::clustering::ClusteringSystem;
use crate::perspective::PerspectiveTransformer;
use crate::presence::PresenceDetectionZones;
use crate::settings::Cli;
use crate::smoothing::{get_mode, SmoothSettings, TrackingSmoother};

pub type Point2D = (f64, f64);

struct ConsolidatorOutputs<'a> {
    config_output: &'a PlugDefinition,
    clusters_output: &'a PlugDefinition,
    tracking_output: &'a PlugDefinition,
}

fn main() {
    let cli = Cli::parse();

    // Initialize the logger from the environment

    env_logger::Builder::from_env(Env::default().default_filter_or(&cli.log_level))
        .filter_module("paho_mqtt", log::LevelFilter::Warn)
        .init();

    debug!("Started; args: {:?}", cli);

    let tether_agent = TetherAgentOptionsBuilder::new(&cli.agent_role)
        .id(Some(&cli.agent_group))
        .host(Some(&cli.tether_host.to_string()))
        .build()
        .expect("failed to init and/or connect Tether Agent");

    // Initialise config, now that we have the MQTT client ready
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

    // Clusters, tracking, automask outputs
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

    // Collect all outputs for easier passing
    let all_outputs = ConsolidatorOutputs {
        tracking_output: &tracking_output,
        clusters_output: &clusters_output,
        config_output: &config_output,
    };

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

    debug!("Clustering system init OK");

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

    let mut smoothing = TrackingSmoother::new(SmoothSettings {
        merge_radius: cli.smoothing_merge_radius,
        wait_before_active_ms: cli.smoothing_wait_before_active_ms,
        expire_ms: cli.smoothing_expire_ms,
        lerp_factor: cli.smoothing_lerp_factor,
        // TODO: set this from CLI
        empty_list_send_mode: get_mode(&cli.smoothing_empty_send_mode)
            .unwrap_or(smoothing::EmptyListSendMode::Once),
    });

    debug!("Perspective transformer system init OK");

    let mut automask_samplers: HashMap<String, AutoMaskSampler> = HashMap::new();

    let mut presence_detector =
        PresenceDetectionZones::new(tracking_config.zones().unwrap_or_default());

    loop {
        let mut work_done = false;

        if let Some((topic, message)) = tether_agent.check_messages() {
            work_done = true;
            // debug!("Received {:?}", message);
            if scans_input.matches(&topic) {
                // debug!("Received scans message");
                handle_scans_message(
                    &message,
                    &tether_agent,
                    &all_outputs,
                    &mut tracking_config,
                    &mut clustering_system,
                    &perspective_transformer,
                    &mut automask_samplers,
                    &mut smoothing,
                    cli.default_min_distance_threshold,
                );
            }

            if save_config_input.matches(&topic) {
                handle_save_message(
                    &tether_agent,
                    &config_output,
                    &message,
                    &mut tracking_config,
                    &mut perspective_transformer,
                )
                .expect("config should save");
            }

            if request_config_input.matches(&topic) {
                info!("requestLidarConfig; respond with provideLidarConfig message");
                tether_agent
                    .encode_and_publish(&config_output, &tracking_config)
                    .expect("failed to publish config");
            }

            if request_automask_input.matches(&topic) {
                info!("requestAutoMask message");
                handle_automask_message(
                    &message,
                    &mut automask_samplers,
                    &mut tracking_config,
                    cli.automask_scans_required,
                    cli.automask_threshold_margin,
                )
                .expect("failed to publish automask config");
            }
        }

        if !cli.smoothing_disable {
            if let Ok(elapsed) = smoothing.last_updated().elapsed() {
                if elapsed.as_millis() > cli.smoothing_update_interval {
                    work_done = true;
                    smoothing.update_smoothing();
                    if let Some(smoothed_points) = smoothing.get_smoothed_points() {
                        tether_agent
                            .encode_and_publish(&smoothed_tracking_output, &smoothed_points)
                            .expect("failed to publish smoothed tracking points");
                        for changed_zone in presence_detector.update_zones(&smoothed_points).iter()
                        {
                            debug!("ZONE CHANGED: {:?}", changed_zone);
                            let topic = build_topic(
                                "presenceDetection",
                                &changed_zone.id.to_string(),
                                "presence",
                            );
                            let payload = if changed_zone.active { &[1] } else { &[0] };
                            tether_agent
                                .publish_raw(&topic, payload, Some(2), Some(false))
                                .expect("failed to send presence update");
                        }
                    } else {
                        for changed_zone in presence_detector.update_zones(&[]).iter() {
                            debug!("ZONE CHANGED: {:?}", changed_zone);
                            let topic = build_topic(
                                "presenceDetection",
                                &changed_zone.id.to_string(),
                                "presence",
                            );
                            let payload = if changed_zone.active { &[1] } else { &[0] };
                            tether_agent
                                .publish_raw(&topic, payload, Some(2), Some(false))
                                .expect("failed to send presence update");
                        }
                    }
                }
            }
        }

        if !work_done {
            thread::sleep(Duration::from_millis(1));
        }
    }
}

fn handle_scans_message(
    incoming_message: &Message,
    tether_agent: &TetherAgent,
    outputs: &ConsolidatorOutputs,
    config: &mut TrackingConfig,
    clustering_system: &mut ClusteringSystem,
    perspective_transformer: &PerspectiveTransformer,
    automask_samplers: &mut HashMap<String, AutoMaskSampler>,
    smoothing: &mut TrackingSmoother,
    default_min_distance: f64,
) {
    let ConsolidatorOutputs {
        config_output,
        clusters_output,
        tracking_output,
    } = outputs;

    // TODO: this could be parsed from the ThreePartTopic earlier
    let serial = parse_agent_id(incoming_message.topic()).unwrap_or("unknown");

    // If an unknown device was found (and added), re-publish the Device config
    if let Some(()) = config.check_or_create_device(serial, default_min_distance) {
        tether_agent
            .encode_and_publish(config_output, &config)
            .expect("failed to publish config");
    }

    let scans: Vec<(f64, f64)> =
        rmp_serde::from_slice(incoming_message.payload()).expect("failed to decode scans");

    if let Some(device) = config.get_device(serial) {
        if let Ok(clusters) = clustering_system.handle_scan_message(&scans, device) {
            tether_agent
                .encode_and_publish(clusters_output, &clusters)
                .expect("failed to publish clusters");

            if perspective_transformer.is_ready() {
                let points: Vec<Point2D> = clusters
                    .into_iter()
                    .map(|c| perspective_transformer.transform(&(c.x, c.y)).unwrap())
                    .collect();

                if let Ok(tracked_points) = perspective_transformer.get_tracked_points(&points) {
                    // Normal (unsmoothed) tracked points...
                    tether_agent
                        .encode_and_publish(tracking_output, &tracked_points)
                        .expect("failed to publish tracked points");
                    smoothing.update_tracked_points(&tracked_points);
                }
            }

            if let Some(sampler) = automask_samplers.get_mut(serial) {
                if !sampler.is_complete() {
                    if let Some(new_mask) = sampler.add_samples(&scans) {
                        debug!("Sufficient samples for masking device {}", serial);
                        match config.update_device_masking(new_mask, serial) {
                            Ok(()) => {
                                info!("Updated masking for device {}", serial);
                                // Automasking was updated, so re-publish Device Config
                                tether_agent
                                    .encode_and_publish(&config_output, &config)
                                    .expect("failed to publish config");
                                config
                                    .write_config_to_file()
                                    .expect("failed to save config");
                                sampler.angles_with_thresholds.clear();
                            }
                            Err(()) => {
                                error!("Error updating masking for device {}", serial);
                            }
                        }
                    }
                }
            }
        }
    } else {
        error!("Failed to find device; it should have been added if it was unknown");
    }
}

pub fn handle_save_message(
    tether_agent: &TetherAgent,
    config_output: &PlugDefinition,
    incoming_message: &Message,
    config: &mut TrackingConfig,
    perspective_transformer: &mut PerspectiveTransformer,
) -> Result<(), Error> {
    match config.parse_remote_config(incoming_message) {
        Ok(()) => {
            info!("Remote-provided config parsed OK; now save to disk and (re) publish");
            config
                .write_config_to_file()
                .expect("failed to save to disk");

            tether_agent
                .encode_and_publish(config_output, &config)
                .expect("failed to publish config");

            if let Some(region_of_interest) = config.region_of_interest() {
                info!("New Region of Interest was provided remotely; update the Perspective Transformer");
                let (c1, c2, c3, c4) = region_of_interest;
                let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                perspective_transformer.set_new_quad(&corners);
                Ok(())
            } else {
                Ok(())
            }
        }
        Err(()) => Err(Error),
    }
}

fn handle_automask_message(
    incoming_message: &Message,
    automask_samplers: &mut HashMap<String, AutoMaskSampler>,
    config: &mut TrackingConfig,
    scans_required: usize,
    threshold_margin: f64,
) -> Result<(), ()> {
    let payload = incoming_message.payload().to_vec();

    if let Ok(automask_command) = rmp_serde::from_slice::<AutoMaskMessage>(&payload) {
        let command_type: &str = &automask_command.r#type;
        match command_type {
            "new" => {
                info!("request NEW auto mask samplers");
                automask_samplers.clear();
                config.clear_device_masking();
                for device in config.devices().iter() {
                    automask_samplers.insert(
                        String::from(&device.serial),
                        AutoMaskSampler::new(scans_required, threshold_margin),
                    );
                }
                Ok(())
            }
            "clear" => {
                info!("request CLEAR all device masking thresholds");
                automask_samplers.clear();
                config.clear_device_masking();
                Ok(())
            }
            _ => {
                error!("Unrecognised command type for RequestAutoMask message");
                Err(())
            }
        }
    } else {
        error!("Failed to parse auto mask command");
        Err(())
    }
}
