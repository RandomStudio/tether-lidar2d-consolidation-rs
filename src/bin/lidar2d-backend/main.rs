use clap::Parser;
use tether_lidar2d_consolidation::backend_config::load_config_from_file;
use tether_lidar2d_consolidation::systems::automasking::handle_automask_message;
use tether_lidar2d_consolidation::systems::movement::calculate;
use tether_lidar2d_consolidation::systems::presence::publish_presence_change;
use tether_lidar2d_consolidation::systems::Systems;
use tether_lidar2d_consolidation::tether_interface::Outputs;

use env_logger::Env;
use log::{debug, info};
use std::thread;
use std::time::Duration;
use tether_agent::{tether_compliant_topic::TetherOrCustomTopic, TetherAgentOptionsBuilder};

use tether_lidar2d_consolidation::tether_interface::{handle_scans_message, Inputs};

mod cli;
use cli::Cli;

fn main() {
    let cli = Cli::parse();

    // Initialize the logger from the environment

    env_logger::Builder::from_env(Env::default().default_filter_or(&cli.log_level))
        .filter_module("paho_mqtt", log::LevelFilter::Warn)
        .filter_module("tether_agent", log::LevelFilter::Warn)
        .filter_module("quad_to_quad_transformer", log::LevelFilter::Warn)
        .init();

    debug!("Started; args: {:?}", cli);

    let mut tether_agent = TetherAgentOptionsBuilder::new(&cli.agent_role)
        .id(Some(&cli.agent_group))
        .host(Some(&cli.tether_host.to_string()))
        .build()
        .expect("failed to init and/or connect Tether Agent");

    let inputs = Inputs::new(&mut tether_agent);
    let outputs = Outputs::new(&mut tether_agent);

    let mut backend_config = match load_config_from_file(&cli.config_path) {
        Ok(config) => {
            info!("Loaded tracking config OK into Config; publish with retain=true",);
            // Always save and publish on first start/load...
            config
                .save_and_republish(&tether_agent, &outputs.config_output, &cli.config_path)
                .expect("failed to save and publish config");
            config
        }
        Err(e) => {
            panic!("Error loading devices into config manager: {}", e)
        }
    };

    let mut systems = Systems::new(&backend_config);

    loop {
        let mut work_done = false;

        if let Some((topic, message)) = tether_agent.check_messages() {
            work_done = true;
            // debug!("Received {:?}", message);
            if inputs.scans_input.matches(&topic) {
                let serial_number = match &topic {
                    TetherOrCustomTopic::Tether(t) => t.id(),
                    TetherOrCustomTopic::Custom(s) => {
                        panic!(
                            "The topic \"{}\" is not expected for Lidar scan messages",
                            &s
                        );
                    }
                };

                let scans: Vec<(f32, f32)> =
                    rmp_serde::from_slice(&message).expect("failed to decode scans");

                handle_scans_message(
                    serial_number.unwrap(),
                    &scans,
                    &mut backend_config,
                    &tether_agent,
                    &mut systems,
                    &outputs,
                    &cli.config_path,
                )
            }

            if inputs.save_config_input.matches(&topic) {
                backend_config
                    .handle_save_message(
                        &tether_agent,
                        &outputs.config_output,
                        &message,
                        &mut systems.position_remapping,
                        &cli.config_path,
                    )
                    .expect("config failed to update and save");

                info!("New config was received and saved; must update systems now...");

                systems = Systems::new(&backend_config);
            }

            if inputs.request_automask_input.matches(&topic) {
                info!("requestAutoMask message");
                if let Ok(should_update_config) = handle_automask_message(
                    &message,
                    &mut systems.automask_samplers,
                    &mut backend_config,
                ) {
                    if should_update_config {
                        backend_config
                            .save_and_republish(
                                &tether_agent,
                                &outputs.config_output,
                                &cli.config_path,
                            )
                            .expect("failed to save and republish config");
                    }
                }
            }
        }

        if !backend_config.smoothing_disable
            && systems.smoothing_system.get_elapsed().as_millis()
                > backend_config.smoothing_update_interval as u128
        {
            work_done = true;
            systems
                .smoothing_system
                .update_smoothing(backend_config.smoothing_update_interval);

            let smoothed_points = systems.smoothing_system.get_active_smoothed_points();

            if let Some(active_smoothed_points) = smoothed_points {
                let payload = rmp_serde::to_vec(&active_smoothed_points)
                    .expect("failed to serialize smoothed tracking points");
                tether_agent
                    .send(&outputs.smoothed_tracking_output, Some(&payload))
                    .expect("failed to publish smoothed tracking points");

                if backend_config.enable_average_movement
                    && systems.movement_analysis.get_elapsed()
                        >= Duration::from_millis(backend_config.average_movement_interval as u64)
                {
                    // Use smoothed points for movement analysis...
                    let movement_vector = calculate(&active_smoothed_points);
                    let payload = rmp_serde::to_vec(&movement_vector)
                        .expect("failed to serialize movement vector");
                    tether_agent
                        .send(&outputs.movement_output, Some(&payload))
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
                if backend_config.enable_average_movement
                    && systems.movement_analysis.get_elapsed()
                        >= Duration::from_millis(backend_config.average_movement_interval as u64)
                {
                    let movement_vector = calculate(&[]);
                    let payload = rmp_serde::to_vec(&movement_vector)
                        .expect("failed to serialize movement vector");

                    tether_agent
                        .send(&outputs.movement_output, Some(&payload))
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
