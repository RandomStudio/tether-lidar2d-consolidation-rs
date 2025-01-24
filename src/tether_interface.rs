use log::{debug, error, info};
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent};

use crate::{backend_config::BackendConfig, systems::Systems, Point2D};

pub struct Outputs {
    pub config_output: PlugDefinition,
    pub clusters_output: PlugDefinition,
    pub tracking_output: PlugDefinition,
    pub smoothed_tracking_output: PlugDefinition,
    pub smoothed_remapped_output: PlugDefinition,
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

        // Smoothed tracked points output (with TopLeft origin)
        let smoothed_tracking_output = PlugOptionsBuilder::create_output("smoothedTrackedPoints")
            .qos(Some(0))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        let smoothed_remapped_output = PlugOptionsBuilder::create_output("smoothedRemappedPoints")
            .qos(Some(0))
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
            smoothed_remapped_output,
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

pub fn handle_scans_message(
    serial: &str,
    scans: &[Point2D],
    config: &mut BackendConfig,
    tether_agent: &TetherAgent,
    systems: &mut Systems,
    outputs: &Outputs,
    config_file_path: &str,
) {
    let Systems {
        clustering_system,
        position_remapping,
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
    if let Some(()) = config.check_or_create_device(serial, config.default_min_distance_threshold) {
        config
            .save_and_republish(tether_agent, config_output, config_file_path)
            .expect("failed to save and republish config");
    }

    if let Some(device) = config.get_device(serial) {
        clustering_system.update_from_scan(scans, device);
        let clusters = clustering_system.clusters();
        tether_agent
            .encode_and_publish(clusters_output, clusters)
            .expect("failed to publish clusters");

        if position_remapping.is_ready() {
            let points: Vec<Point2D> = position_remapping.transform_clusters(clusters);

            let tracked_points = position_remapping.filter_points_inside(&points);
            // Normal (unsmoothed) tracked points...
            tether_agent
                .encode_and_publish(tracking_output, &tracked_points)
                .expect("failed to publish tracked points");
            smoothing_system.update_tracked_points(&tracked_points);
        }

        if let Some(sampler) = automask_samplers.get_mut(serial) {
            if !sampler.is_complete() {
                if let Some(new_mask) = sampler.add_samples(scans) {
                    debug!("Sufficient samples for masking device {}", serial);
                    match config.update_device_masking(new_mask, serial) {
                        Ok(()) => {
                            info!("Updated masking for device {}", serial);
                            config
                                .save_and_republish(tether_agent, config_output, config_file_path)
                                .expect("failed save and republish config");
                            sampler.angles_with_thresholds.clear();
                        }
                        Err(e) => {
                            error!("Error updating masking for device {}: {}", serial, e);
                        }
                    }
                }
            }
        }
    }
}

// pub fn handle_external_tracking_message(
//     serial: &str,
//     points: &[Point2D],
//     config: &mut BackendConfig,
//     tether_agent: &TetherAgent,
//     systems: &mut Systems,
//     outputs: &Outputs,
//     config_file_path: &str,
// ) {
//     let Systems {
//         clustering_system,
//         perspective_transformer,
//         smoothing_system,
//         ..
//     } = systems;

//     let Outputs {
//         config_output,
//         clusters_output,
//         tracking_output,
//         ..
//     } = outputs;

//     // If an unknown device was found (and added), re-publish the Device config
//     if let Some(()) = config.check_or_create_external_tracker(serial) {
//         config
//             .save_and_republish(tether_agent, config_output, config_file_path)
//             .expect("failed to save and republish config");
//     }

//     if let Some(tracker) = config.get_external_tracker(serial) {
//         clustering_system.update_from_external_tracker(points, tracker);
//         let clusters = clustering_system.clusters();
//         tether_agent
//             .encode_and_publish(clusters_output, clusters)
//             .expect("failed to publish clusters");

//         if perspective_transformer.is_ready() {
//             let points: Vec<Point2D> = clusters
//                 .iter()
//                 .map(|c| perspective_transformer.transform(&(c.x, c.y)).unwrap())
//                 .collect();

//             if let Ok(tracked_points) = perspective_transformer.filter_points_inside(&points) {
//                 // Normal (unsmoothed) tracked points...
//                 tether_agent
//                     .encode_and_publish(tracking_output, &tracked_points)
//                     .expect("failed to publish tracked points");
//                 smoothing_system.update_tracked_points(&tracked_points);
//             }
//         }
//     }
// }
