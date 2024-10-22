use clap::Parser;
use tether_lidar2d_consolidation::consolidator_system::{Outputs, Systems};
use tether_lidar2d_consolidation::tracking::{Body3D, BodyFrame3D};
use tether_lidar2d_consolidation::tracking_config::TrackingConfig;

use env_logger::Env;
use log::{debug, info};
use std::thread;
use std::time::Duration;
use tether_agent::TetherAgentOptionsBuilder;

use tether_lidar2d_consolidation::automasking::handle_automask_message;
use tether_lidar2d_consolidation::clustering::{
    handle_external_tracking_message, handle_scans_message,
};
use tether_lidar2d_consolidation::consolidator_system::Inputs;
use tether_lidar2d_consolidation::movement::get_total_movement;
use tether_lidar2d_consolidation::presence::publish_presence_change;
use tether_lidar2d_consolidation::settings::Cli;

fn main() {
    let cli = Cli::parse();

    // Initialize the logger from the environment

    env_logger::Builder::from_env(Env::default().default_filter_or(&cli.log_level))
        .filter_module("paho_mqtt", log::LevelFilter::Warn)
        .filter_module("tether_agent", log::LevelFilter::Warn)
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
            info!(
                "Loaded {} devices OK into Config; publish with retain=true",
                count
            );
            // Always publish on first start/load...
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

            if inputs.external_tracking_input.matches(&topic) {
                let serial_number = match &topic {
                    tether_agent::TetherOrCustomTopic::Tether(t) => t.id(),
                    tether_agent::TetherOrCustomTopic::Custom(s) => {
                        panic!(
                            "The topic \"{}\" is not expected for Lidar scan messages",
                            &s
                        );
                    }
                };

                let position_data: BodyFrame3D =
                    rmp_serde::from_slice(message.payload()).expect("failed to decode bodyFrames");

                for body in position_data.iter() {
                    let Body3D { body_xyz, .. } = body;
                    let (x, y, z) = *body_xyz;
                    debug!(
                        "External body tracking position received: {},{},{}",
                        x, y, z
                    );
                    let points = vec![(x, z)];
                    handle_external_tracking_message(
                        serial_number,
                        &points,
                        &mut tracking_config,
                        &tether_agent,
                        &mut systems,
                        &outputs,
                    );
                }
            }

            if inputs.save_config_input.matches(&topic) {
                tracking_config
                    .handle_save_message(
                        &tether_agent,
                        &outputs.config_output,
                        &message,
                        &mut systems.perspective_transformer,
                    )
                    .expect("config failed to update and save");
            }

            if inputs.request_automask_input.matches(&topic) {
                info!("requestAutoMask message");
                if let Ok(should_update_config) = handle_automask_message(
                    &message,
                    &mut systems.automask_samplers,
                    &mut tracking_config,
                    cli.automask_scans_required,
                    cli.automask_threshold_margin,
                ) {
                    if should_update_config {
                        tracking_config
                            .save_and_republish(&tether_agent, &outputs.config_output)
                            .expect("failed to save and republish config");
                    }
                }
            }
        }

        if !cli.smoothing_disable
            && systems.smoothing_system.get_elapsed().as_millis() > cli.smoothing_update_interval
        {
            work_done = true;
            systems.smoothing_system.update_smoothing();

            let smoothed_points = systems.smoothing_system.get_smoothed_points();

            if let Some(active_smoothed_points) = smoothed_points {
                tether_agent
                    .encode_and_publish(&outputs.smoothed_tracking_output, &active_smoothed_points)
                    .expect("failed to publish smoothed tracking points");

                if !cli.movement_disable
                    && systems.movement_analysis.get_elapsed()
                        >= Duration::from_millis(cli.movement_interval as u64)
                {
                    // Use smoothed points for movement analysis...
                    let movement_vector = get_total_movement(&active_smoothed_points);

                    tether_agent
                        .encode_and_publish(&outputs.movement_output, movement_vector)
                        .expect("failed to publish movement vector");

                    systems.movement_analysis.reset_timer();
                }

                // Use smoothed points for presence detection, if any zones are defined...
                for changed_zone in systems
                    .presence_detector
                    .update_zones(&active_smoothed_points)
                    .iter()
                {
                    publish_presence_change(changed_zone, &tether_agent);
                }
            } else {
                // No smoothed points, but update presence detection with zero-points...
                for changed_zone in systems.presence_detector.update_zones(&[]).iter() {
                    publish_presence_change(changed_zone, &tether_agent);
                }
                // No smoothed points, but update movement analysis with zero-points...
                if !cli.movement_disable
                    && systems.movement_analysis.get_elapsed()
                        >= Duration::from_millis(cli.movement_interval as u64)
                {
                    let movement_vector = get_total_movement(&[]);

                    tether_agent
                        .encode_and_publish(&outputs.movement_output, movement_vector)
                        .expect("failed to publish movement vector");

                    systems.movement_analysis.reset_timer();
                }
            }
        }

        if !work_done {
            thread::sleep(Duration::from_millis(1));
        }
    }
}
