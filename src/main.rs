use automasking::AutoMaskMessage;
use clap::Parser;
use consolidator_system::{Outputs, Systems};
use tracking_config::TrackingConfig;

use env_logger::Env;
use log::{debug, error, info};
use std::collections::HashMap;
use std::fmt::Error;
use std::thread;
use std::time::Duration;
use tether_agent::mqtt::Message;
use tether_agent::{PlugDefinition, TetherAgent, TetherAgentOptionsBuilder};

mod automasking;
mod clustering;
mod consolidator_system;
mod perspective;
mod presence;
mod settings;
mod smoothing;
mod tracking;
mod tracking_config;

use crate::automasking::AutoMaskSampler;
use crate::consolidator_system::Inputs;
use crate::perspective::PerspectiveTransformer;
use crate::presence::publish_presence_change;
use crate::settings::Cli;

pub type Point2D = (f32, f32);

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

    let inputs = Inputs::new(&tether_agent);
    let outputs = Outputs::new(&tether_agent);

    let mut tracking_config = TrackingConfig::new(&cli.config_path);

    match tracking_config.load_config_from_file() {
        Ok(count) => {
            info!("Loaded {} devices OK into Config", count);
            tether_agent
                .encode_and_publish(&outputs.config_output, &tracking_config)
                .expect("failed to publish config");
        }
        Err(()) => {
            panic!("Error loading devices into config manager!")
        }
    };

    let mut systems = Systems::new(&cli, &tracking_config);

    loop {
        let mut work_done = false;

        if let Some((topic, message)) = tether_agent.check_messages() {
            work_done = true;
            // debug!("Received {:?}", message);
            if inputs.scans_input.matches(&topic) {
                let serial_number = match &topic {
                    tether_agent::TetherOrCustomTopic::Tether(t) => t.id(),
                    tether_agent::TetherOrCustomTopic::Custom(s) => {
                        panic!(
                            "The topic \"{}\" is not expected for Lidar scan messages",
                            &s
                        );
                    }
                };

                let scans: Vec<(f32, f32)> =
                    rmp_serde::from_slice(message.payload()).expect("failed to decode scans");

                handle_scans_message(
                    serial_number,
                    &scans,
                    &mut tracking_config,
                    &tether_agent,
                    &mut systems,
                    &outputs,
                    cli.default_min_distance_threshold,
                )
            }

            if inputs.save_config_input.matches(&topic) {
                handle_save_message(
                    &tether_agent,
                    &outputs.config_output,
                    &message,
                    &mut tracking_config,
                    &mut systems.perspective_transformer,
                )
                .expect("config should save");
            }

            if inputs.request_config_input.matches(&topic) {
                info!("requestLidarConfig; respond with provideLidarConfig message");
                tether_agent
                    .encode_and_publish(&outputs.config_output, &tracking_config)
                    .expect("failed to publish config");
            }

            if inputs.request_automask_input.matches(&topic) {
                info!("requestAutoMask message");
                handle_automask_message(
                    &message,
                    &mut systems.automask_samplers,
                    &mut tracking_config,
                    cli.automask_scans_required,
                    cli.automask_threshold_margin,
                )
                .expect("failed to publish automask config");
            }
        }

        if !cli.smoothing_disable {
            if let Ok(elapsed) = systems.smoothing_system.last_updated().elapsed() {
                if elapsed.as_millis() > cli.smoothing_update_interval {
                    work_done = true;
                    systems.smoothing_system.update_smoothing();

                    let smoothed_points = systems.smoothing_system.get_smoothed_points();

                    if let Some(active_smoothed_points) = smoothed_points {
                        tether_agent
                            .encode_and_publish(
                                &outputs.smoothed_tracking_output,
                                &active_smoothed_points,
                            )
                            .expect("failed to publish smoothed tracking points");
                        for changed_zone in systems
                            .presence_detector
                            .update_zones(&active_smoothed_points)
                            .iter()
                        {
                            publish_presence_change(changed_zone, &tether_agent);
                        }
                    } else {
                        for changed_zone in systems.presence_detector.update_zones(&[]).iter() {
                            publish_presence_change(changed_zone, &tether_agent);
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
    serial: &str,
    scans: &[Point2D],
    tracking_config: &mut TrackingConfig,
    tether_agent: &TetherAgent,
    systems: &mut Systems,
    outputs: &Outputs,
    default_min_distance: f32,
) {
    let Systems {
        clustering_system,
        perspective_transformer,
        automask_samplers,
        smoothing_system,
        ..
    } = systems;

    let Outputs {
        config_output,
        clusters_output,
        tracking_output,
        ..
    } = outputs;

    // If an unknown device was found (and added), re-publish the Device config
    if let Some(()) = tracking_config.check_or_create_device(serial, default_min_distance) {
        tether_agent
            .encode_and_publish(config_output, &tracking_config)
            .expect("failed to publish config");
    }

    if let Some(device) = tracking_config.get_device(serial) {
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
                    smoothing_system.update_tracked_points(&tracked_points);
                }
            }

            if let Some(sampler) = automask_samplers.get_mut(serial) {
                if !sampler.is_complete() {
                    if let Some(new_mask) = sampler.add_samples(&scans) {
                        debug!("Sufficient samples for masking device {}", serial);
                        match tracking_config.update_device_masking(new_mask, serial) {
                            Ok(()) => {
                                info!("Updated masking for device {}", serial);
                                // Automasking was updated, so re-publish Device Config
                                tether_agent
                                    .encode_and_publish(&config_output, &tracking_config)
                                    .expect("failed to publish config");
                                tracking_config
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
    threshold_margin: f32,
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
