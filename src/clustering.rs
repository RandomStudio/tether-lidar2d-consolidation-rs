use crate::{
    consolidator_system::{Outputs, Systems},
    tracking_config::{ExternalTracker, LidarDevice, TrackingConfig},
    Point2D,
};

use log::{debug, error, info};
use serde::{Deserialize, Serialize};

use ndarray::{Array, ArrayView};
use petal_clustering::{Dbscan, Fit};
use petal_neighbors::distance::Euclidean;
use std::{collections::HashMap, f32::consts::TAU};
use tether_agent::TetherAgent;

#[derive(Serialize, Deserialize, Debug)]
pub struct Cluster2D {
    pub id: usize,
    pub x: f32,
    pub y: f32,
    pub size: f32,
}

struct Bounds2D {
    x_min: Option<f32>,
    y_min: Option<f32>,
    x_max: Option<f32>,
    y_max: Option<f32>,
}

pub struct ClusteringSystem {
    scan_points: HashMap<String, Vec<Point2D>>,
    clustering_engine: Dbscan<f32, Euclidean>,
    cached_clusters: Vec<Cluster2D>,
    max_cluster_size: f32,
}

impl ClusteringSystem {
    pub fn new(
        neighbourhood_radius: f32,
        min_neighbourss: usize,
        max_cluster_size: f32,
    ) -> ClusteringSystem {
        ClusteringSystem {
            scan_points: HashMap::new(),
            clustering_engine: Dbscan {
                eps: neighbourhood_radius,
                min_samples: min_neighbourss,
                metric: Euclidean::default(),
            },
            cached_clusters: Vec::new(),
            max_cluster_size,
        }
    }

    /** A snapshot of the most recently-calculated clusters list */
    pub fn clusters(&self) -> &[Cluster2D] {
        &self.cached_clusters
    }

    pub fn update_from_scan(&mut self, scans: &[Point2D], device: &LidarDevice) {
        debug!("Decoded {} scans", scans.len());
        let mut points_this_scan: Vec<Point2D> = Vec::with_capacity(scans.len());

        for sample in scans {
            let (angle, distance) = sample;

            if *distance > 0.0 {
                if let Some(point) = scan_sample_to_point(angle, distance, device) {
                    points_this_scan.push(point);
                }
            }
        }

        self.scan_points
            .insert(String::from(&device.serial), points_this_scan);

        let combined_points = self.combine_all_points();

        let (clusters, outliers) = self.clustering_engine.fit(&combined_points);

        debug!(
            "Found {} clusters, {} outliers",
            clusters.len(),
            outliers.len()
        );

        self.cached_clusters = clusters
            .iter()
            .map(|c| {
                let (cluster_index, point_indexes) = c;
                let matched_points = point_indexes
                    .iter()
                    .map(|i| {
                        let point = combined_points.row(*i);
                        (point[0], point[1])
                    })
                    .collect();

                circle_of_cluster_points(matched_points, *cluster_index)
            })
            .filter(|cluster| cluster.size <= self.max_cluster_size)
            .collect()
    }

    pub fn update_from_external_tracker(&mut self, points: &[Point2D], tracker: &ExternalTracker) {
        debug!("Tracker is {:?}", tracker);
        let transformed_points: Vec<Point2D> = points
            .iter()
            .map(|p| external_point_transformed(p, tracker))
            .collect();

        let mut fake_points = Vec::new();
        for (x, y) in transformed_points {
            for i in 0..32 {
                let t = (i as f32) / 32. * TAU;
                let r = 500.;
                fake_points.push((r * t.sin() + x, r * t.cos() + y));
            }
        }

        self.scan_points
            .insert(String::from(&tracker.serial), fake_points);

        let combined_points = self.combine_all_points();

        let (clusters, _outliers) = self.clustering_engine.fit(&combined_points);

        self.cached_clusters = clusters
            .iter()
            .map(|c| {
                let (cluster_index, point_indexes) = c;
                let matched_points = point_indexes
                    .iter()
                    .map(|i| {
                        let point = combined_points.row(*i);
                        (point[0], point[1])
                    })
                    .collect();

                circle_of_cluster_points(matched_points, *cluster_index)
            })
            .filter(|cluster| cluster.size <= self.max_cluster_size)
            .collect()

        // for (x, y) in points {
        //     self.cached_clusters.push(Cluster2D {
        //         id: self.cached_clusters.len(),
        //         x: *x,
        //         y: *y,
        //         size: 500.0,
        //     })
        // }

        // self.scan_points
        //     .insert(String::from(&tracker.serial), transformed_points.to_vec());
    }

    pub fn combine_all_points(&self) -> ndarray::Array2<f32> {
        let mut all_points = Array::zeros((0, 2));
        for points in self.scan_points.values() {
            for (x, y) in points {
                all_points.push_row(ArrayView::from(&[*x, *y])).unwrap()
            }
        }
        all_points
    }
}

/**
Represent points in a cluster as a single "Cluster2D" (same as Point2D, but including size)
*/
pub fn circle_of_cluster_points(points: Vec<Point2D>, id: usize) -> Cluster2D {
    let bounds = points.iter().fold(
        Bounds2D {
            x_min: None,
            y_min: None,
            x_max: None,
            y_max: None,
        },
        |acc, point| {
            let (x, y) = point;
            Bounds2D {
                x_min: match acc.x_min {
                    None => Some(*x),
                    Some(v) => Some(v.min(*x)),
                },
                y_min: match acc.y_min {
                    None => Some(*y),
                    Some(v) => Some(v.min(*y)),
                },
                x_max: match acc.x_max {
                    None => Some(*x),
                    Some(v) => Some(v.max(*x)),
                },
                y_max: match acc.y_max {
                    None => Some(*y),
                    Some(v) => Some(v.max(*y)),
                },
            }
        },
    );
    let width = bounds.x_max.unwrap() - bounds.x_min.unwrap();
    let height = bounds.y_max.unwrap() - bounds.y_min.unwrap();
    Cluster2D {
        id,
        x: bounds.x_min.unwrap() + 0.5 * width,
        y: bounds.y_min.unwrap() + 0.5 * height,
        size: { width.max(height) },
    }
}

/**
Take in angle, distance return as Point2D as (x,y) coordinates
*/
fn scan_sample_to_point(angle: &f32, distance: &f32, device: &LidarDevice) -> Option<Point2D> {
    let LidarDevice {
        x,
        y,
        rotation,
        flip_coords,
        min_distance_threshold,
        scan_mask_thresholds,
        ..
    } = device;
    if *distance > 0.
        && *distance > *min_distance_threshold
        && passes_mask_threshold(angle, distance, scan_mask_thresholds)
    {
        match flip_coords {
            None => Some((
                *x + (angle + *rotation).to_radians().sin() * distance,
                *y + (angle + *rotation).to_radians().cos() * distance,
            )),
            Some((flip_x, flip_y)) => {
                let altered_angle = {
                    if flip_x == flip_y {
                        *angle + *rotation
                    } else {
                        *angle - *rotation
                    }
                };
                Some((
                    *x + altered_angle.to_radians().sin() * *distance * (*flip_x as f32),
                    *y + altered_angle.to_radians().cos() * *distance * (*flip_y as f32),
                ))
            }
        }
    } else {
        None
    }
}

fn external_point_transformed(p: &Point2D, tracker: &ExternalTracker) -> Point2D {
    let ExternalTracker {
        x,
        y,
        rotation,
        flip_coords,
        ..
    } = tracker;
    // let rotation = -rotation;
    let (px, py) = p;
    // Translate so origin is at (x,y), then tRotate about origin...
    let px = px * rotation.to_radians().cos() - py * rotation.to_radians().sin() + *x;
    let py = py * rotation.to_radians().cos() + px * rotation.to_radians().sin() + *y;
    debug!("{},{} => {},{}", p.0, p.1, px, py);
    match flip_coords {
        None => (px, py),
        Some((flip_x, flip_y)) => (px * (*flip_x as f32), py * (*flip_y as f32)),
    }
}

fn passes_mask_threshold(
    angle: &f32,
    distance: &f32,
    mask_thresholds: &Option<HashMap<String, f32>>,
) -> bool {
    match mask_thresholds {
        None => true,
        Some(masking_map) => {
            let angle_key = angle.round().to_string();
            if let Some(threshold) = masking_map.get(&angle_key) {
                *distance < *threshold
            } else {
                true
            }
        }
    }
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
