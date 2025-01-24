use clap::Parser;
use quad_to_quad_transformer::DEFAULT_DST_QUAD;
use tether_lidar2d_consolidation::backend_config::load_config_from_file;
use tether_lidar2d_consolidation::consolidator_system::{calculate_dst_quad, Outputs, Systems};
use tether_lidar2d_consolidation::smoothing::OriginLocation;
use tether_lidar2d_consolidation::tracking::{Body3D, BodyFrame3D, TrackedPoint2D};

use env_logger::Env;
use log::{debug, info};
use map_range::MapRange;
use std::thread;
use std::time::Duration;
use tether_agent::TetherAgentOptionsBuilder;

use tether_lidar2d_consolidation::automasking::handle_automask_message;
use tether_lidar2d_consolidation::consolidator_system::{
    handle_external_tracking_message, handle_scans_message, Inputs,
};
use tether_lidar2d_consolidation::movement::get_total_movement;
use tether_lidar2d_consolidation::presence::publish_presence_change;

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

    let tether_agent = TetherAgentOptionsBuilder::new(&cli.agent_role)
        .id(Some(&cli.agent_group))
        .host(Some(&cli.tether_host.to_string()))
        .build()
        .expect("failed to init and/or connect Tether Agent");

    let inputs = Inputs::new(&tether_agent);
    let outputs = Outputs::new(&tether_agent);

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
                    &mut backend_config,
                    &tether_agent,
                    &mut systems,
                    &outputs,
                    &cli.config_path,
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
                        &mut backend_config,
                        &tether_agent,
                        &mut systems,
                        &outputs,
                        &cli.config_path,
                    );
                }

                if position_data.is_empty() {
                    handle_external_tracking_message(
                        serial_number,
                        &[],
                        &mut backend_config,
                        &tether_agent,
                        &mut systems,
                        &outputs,
                        &cli.config_path,
                    );
                }
            }

            if inputs.save_config_input.matches(&topic) {
                backend_config
                    .handle_save_message(
                        &tether_agent,
                        &outputs.config_output,
                        &message,
                        &mut systems.perspective_transformer,
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
                > backend_config.smoothing_update_interval
        {
            work_done = true;
            systems.smoothing_system.update_smoothing();

            let smoothed_points = systems.smoothing_system.get_smoothed_points();

            if let Some(active_smoothed_points) = smoothed_points {
                tether_agent
                    .encode_and_publish(&outputs.smoothed_tracking_output, &active_smoothed_points)
                    .expect("failed to publish smoothed tracking points");

                if let Some(roi) = &backend_config.region_of_interest {
                    let dst_quad = if backend_config.smoothing_use_real_units {
                        calculate_dst_quad(roi)
                    } else {
                        DEFAULT_DST_QUAD
                    };
                    let [_a, b, c, _d] = dst_quad;
                    let mid_x = b.0 / 2.0;

                    let remapped_points: Vec<TrackedPoint2D> = match &backend_config.origin_location
                    {
                        OriginLocation::TopLeft => active_smoothed_points.clone(),
                        OriginLocation::TopCentre => active_smoothed_points
                            .iter()
                            .map(|p| TrackedPoint2D {
                                x: p.x.map_range(0. ..b.0, -mid_x..mid_x),
                                ..*p
                            })
                            .collect(),
                        OriginLocation::BottomCentre => active_smoothed_points
                            .iter()
                            .map(|p| TrackedPoint2D {
                                x: p.x.map_range(0. ..b.0, -mid_x..mid_x),
                                y: c.1 - p.y, // inverted
                                ..*p
                            })
                            .collect(),
                        OriginLocation::Centre => {
                            let mid_y = c.1 / 2.0;

                            active_smoothed_points
                                .iter()
                                .map(|p| TrackedPoint2D {
                                    x: p.x.map_range(0. ..b.0, -mid_x..mid_x),
                                    y: p.y.map_range(0. ..c.1, -mid_y..mid_y),
                                    ..*p
                                })
                                .collect()
                        }
                    };
                    tether_agent
                        .encode_and_publish(&outputs.smoothed_remapped_output, &remapped_points)
                        .expect("failed to publish smoothed+remapped points");
                }

                if !backend_config.movement_disable
                    && systems.movement_analysis.get_elapsed()
                        >= Duration::from_millis(backend_config.movement_interval as u64)
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
                if !backend_config.movement_disable
                    && systems.movement_analysis.get_elapsed()
                        >= Duration::from_millis(backend_config.movement_interval as u64)
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
