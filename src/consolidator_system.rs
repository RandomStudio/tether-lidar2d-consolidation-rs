use std::collections::HashMap;

use log::{debug, error, info, warn};
use quad_to_quad_transformer::{QuadTransformer, RectCorners, DEFAULT_DST_QUAD};
use tether_agent::{PlugDefinition, PlugOptionsBuilder, TetherAgent};

use crate::{
    automasking::AutoMaskSamplerMap,
    clustering::ClusteringSystem,
    movement::MovementAnalysis,
    presence::PresenceDetectionZones,
    settings::Cli,
    smoothing::{self, get_mode, SmoothSettings, TrackingSmoother},
    tracking_config::{self, CornerPoints, TrackingConfig},
    Point2D,
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
            if tracking_config.use_real_units() {
                info!("Using real units");
                tracking_config.region_of_interest().map(calculate_dst_quad)
            } else {
                warn!("Using normalised units");
                None
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

pub fn calculate_dst_quad(roi: &CornerPoints) -> RectCorners {
    // DEFAULT_DST_QUAD
    let (a, b, c, d) = roi;
    let w = distance(a.x, a.y, b.x, b.y);
    let h = distance(a.x, a.y, d.x, d.y);
    [(0., 0.), (w, 0.), (w, h), (0., h)]
}

fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    ((x2 - x1).powf(2.0) + (y2 - y1).powf(2.0)).sqrt()
}

pub fn handle_scans_message(
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
        tracking_config
            .save_and_republish(tether_agent, config_output)
            .expect("failed to save and republish config");
    }

    if let Some(device) = tracking_config.get_device(serial) {
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
                    match tracking_config.update_device_masking(new_mask, serial) {
                        Ok(()) => {
                            info!("Updated masking for device {}", serial);
                            tracking_config
                                .save_and_republish(tether_agent, config_output)
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
    tracking_config: &mut TrackingConfig,
    tether_agent: &TetherAgent,
    systems: &mut Systems,
    outputs: &Outputs,
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
    if let Some(()) = tracking_config.check_or_create_external_tracker(serial) {
        tracking_config
            .save_and_republish(tether_agent, config_output)
            .expect("failed to save and republish config");
    }

    if let Some(tracker) = tracking_config.get_external_tracker(serial) {
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
