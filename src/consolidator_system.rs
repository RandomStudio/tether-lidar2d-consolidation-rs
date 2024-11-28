use std::collections::HashMap;

use log::{debug, error, info, warn};
use quad_to_quad_transformer::{QuadTransformer, RectCorners};
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent};

use crate::{
    automasking::AutoMaskSamplerMap,
    backend_config::{BackendConfig, CornerPoints},
    clustering::ClusteringSystem,
    movement::MovementAnalysis,
    presence::PresenceDetectionZones,
    smoothing::{SmoothSettings, TrackingSmoother},
    Point2D,
};

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
            .qos(Some(1))
            .build(tether_agent)
            .expect("failed to create Output Plug");

        let smoothed_remapped_output = PlugOptionsBuilder::create_output("smoothedRemappedPoints")
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

pub struct Systems {
    pub clustering_system: ClusteringSystem,
    pub perspective_transformer: QuadTransformer,
    pub smoothing_system: TrackingSmoother,
    pub automask_samplers: AutoMaskSamplerMap,
    pub presence_detector: PresenceDetectionZones,
    pub movement_analysis: MovementAnalysis,
}

impl Systems {
    pub fn new(config: &BackendConfig) -> Systems {
        let clustering_system = ClusteringSystem::new(
            config.clustering_neighbourhood_radius,
            config.clustering_min_neighbours,
            config.clustering_max_cluster_size,
        );

        let perspective_transformer = QuadTransformer::new(
            match config.region_of_interest() {
                Some(region_of_interest) => {
                    let (c1, c2, c3, c4) = region_of_interest;
                    let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                    Some(corners)
                }
                None => None,
            },
            if config.smoothing_use_real_units {
                info!("Using real units");
                config.region_of_interest().map(calculate_dst_quad)
            } else {
                warn!("Using normalised units");
                None
            },
            {
                if config.transform_include_outside {
                    None
                } else {
                    Some(config.transform_ignore_outside_margin)
                }
            },
        );

        let smoothing_system = TrackingSmoother::new(SmoothSettings {
            merge_radius: config.smoothing_merge_radius,
            wait_before_active_ms: config.smoothing_wait_before_active_ms,
            expire_ms: config.smoothing_expire_ms,
            lerp_factor: config.smoothing_lerp_factor,
            empty_list_send_mode: config.smoothing_empty_send_mode,
            origin_mode: config.origin_location,
        });

        let presence_detector = PresenceDetectionZones::new(config.zones().unwrap_or_default());

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

pub fn calculate_dst_quad(roi: &CornerPoints) -> RectCorners {
    let (a, b, _c, d) = roi;
    let w = distance(a.x, a.y, b.x, b.y);
    let h = distance(a.x, a.y, d.x, d.y);
    [(0., 0.), (w, 0.), (w, h), (0., h)]
}

pub fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    ((x2 - x1).powf(2.0) + (y2 - y1).powf(2.0)).sqrt()
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

        if perspective_transformer.is_ready() {
            let points: Vec<Point2D> = clusters
                .iter()
                .map(|c| perspective_transformer.transform(&(c.x, c.y)).unwrap())
                .collect();

            if let Ok(tracked_points) = perspective_transformer.filter_points_inside(&points) {
                // Normal (unsmoothed) tracked points...
                tether_agent
                    .encode_and_publish(tracking_output, &tracked_points)
                    .expect("failed to publish tracked points");
                smoothing_system.update_tracked_points(&tracked_points);
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

pub fn handle_external_tracking_message(
    serial: &str,
    points: &[Point2D],
    config: &mut BackendConfig,
    tether_agent: &TetherAgent,
    systems: &mut Systems,
    outputs: &Outputs,
    config_file_path: &str,
) {
    let Systems {
        clustering_system,
        perspective_transformer,
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
    if let Some(()) = config.check_or_create_external_tracker(serial) {
        config
            .save_and_republish(tether_agent, config_output, config_file_path)
            .expect("failed to save and republish config");
    }

    if let Some(tracker) = config.get_external_tracker(serial) {
        clustering_system.update_from_external_tracker(points, tracker);
        let clusters = clustering_system.clusters();
        tether_agent
            .encode_and_publish(clusters_output, clusters)
            .expect("failed to publish clusters");

        if perspective_transformer.is_ready() {
            let points: Vec<Point2D> = clusters
                .iter()
                .map(|c| perspective_transformer.transform(&(c.x, c.y)).unwrap())
                .collect();

            if let Ok(tracked_points) = perspective_transformer.filter_points_inside(&points) {
                // Normal (unsmoothed) tracked points...
                tether_agent
                    .encode_and_publish(tracking_output, &tracked_points)
                    .expect("failed to publish tracked points");
                smoothing_system.update_tracked_points(&tracked_points);
            }
        }
    }
}
