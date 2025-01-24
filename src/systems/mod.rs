pub mod automasking;
pub mod clustering;
pub mod movement;
pub mod presence;
pub mod smoothing;

use automasking::AutoMaskSamplerMap;
use clustering::ClusteringSystem;
use indexmap::IndexMap;
use log::{info, warn};
use movement::MovementAnalysis;
use presence::PresenceDetectionZones;
use quad_to_quad_transformer::QuadTransformer;
use smoothing::{SmoothSettings, TrackingSmoother};

use crate::{backend_config::BackendConfig, consolidator_system::calculate_dst_quad};

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
            should_calculate_velocity: config.enable_velocity,
            should_calculate_angles: config.enable_angles,
        });

        let presence_detector = PresenceDetectionZones::new(config.zones().unwrap_or_default());

        Systems {
            clustering_system,
            smoothing_system,
            automask_samplers: IndexMap::new(),
            perspective_transformer,
            presence_detector,
            movement_analysis: MovementAnalysis::new(),
        }
    }
}
