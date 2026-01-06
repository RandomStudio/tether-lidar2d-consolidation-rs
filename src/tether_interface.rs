use log::{debug, error, info};
use tether_agent::{ChannelDefinition, ChannelOptionsBuilder, TetherAgent};

use crate::{
    backend_config::BackendConfig,
    systems::{clustering::Cluster2D, Systems},
    Point2D,
};

pub struct Outputs {
    pub config_output: ChannelDefinition,
    pub clusters_output: ChannelDefinition,
    pub tracking_output: ChannelDefinition,
    pub smoothed_tracking_output: ChannelDefinition,
    pub smoothed_remapped_output: ChannelDefinition,
    pub movement_output: ChannelDefinition,
}

impl Outputs {
    pub fn new(tether_agent: &mut TetherAgent) -> Outputs {
        let config_output = ChannelOptionsBuilder::create_sender("provideLidarConfig")
            .qos(Some(2))
            .retain(Some(true))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        // Clusters, tracking outputs
        let tracking_output = ChannelOptionsBuilder::create_sender("trackedPoints")
            .qos(Some(0))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        let clusters_output = ChannelOptionsBuilder::create_sender("clusters")
            .qos(Some(0))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        // Smoothed tracked points output (with TopLeft origin)
        let smoothed_tracking_output =
            ChannelOptionsBuilder::create_sender("smoothedTrackedPoints")
                .qos(Some(0))
                .build(tether_agent)
                .expect("failed to create Output Plug");

        let smoothed_remapped_output =
            ChannelOptionsBuilder::create_sender("smoothedRemappedPoints")
                .qos(Some(0))
                .build(tether_agent)
                .expect("failed to create Output Plug");

        // Movement vector output
        let movement_output = ChannelOptionsBuilder::create_sender("movement")
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
    pub scans_input: ChannelDefinition,
    pub save_config_input: ChannelDefinition,
    pub request_automask_input: ChannelDefinition,
    pub external_tracking_input: ChannelDefinition,
}

impl Inputs {
    pub fn new(tether_agent: &mut TetherAgent) -> Inputs {
        // Some subscriptions
        let scans_input = ChannelOptionsBuilder::create_receiver("scans")
            .qos(Some(0))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        let save_config_input = ChannelOptionsBuilder::create_receiver("saveLidarConfig")
            .qos(Some(2))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        let request_automask_input = ChannelOptionsBuilder::create_receiver("requestAutoMask")
            .qos(Some(2))
            .build(tether_agent)
            .expect("failed to create Output Plug");
        // TODO: the name of this input plug should be customisable
        let external_tracking_input = ChannelOptionsBuilder::create_receiver("bodyFrames")
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

        if !config.skip_some_outputs {
            let payload = rmp_serde::to_vec(&clusters).expect("failed to serialize clusters");
            tether_agent
                .send(clusters_output, Some(&payload))
                .expect("failed to publish clusters");
        }

        if position_remapping.is_ready() {
            let transformed_clusters: Vec<Cluster2D> =
                position_remapping.transform_clusters(clusters);

            // Normal (unsmoothed) tracked points...
            let filtered_clusters =
                position_remapping.filter_clusters_inside(&transformed_clusters);

            smoothing_system.update_tracked_points(&filtered_clusters);

            if !config.skip_some_outputs {
                let raw_points: Vec<Point2D> =
                    filtered_clusters.iter().map(|c| (c.x, c.y)).collect();

                let payload =
                    rmp_serde::to_vec(&raw_points).expect("failed to serialize tracked points");
                tether_agent
                    .send(tracking_output, Some(&payload))
                    .expect("failed to publish tracked points");
            }
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
