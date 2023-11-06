use crate::{tracking_config::LidarDevice, Point2D};

use log::debug;
use serde::{Deserialize, Serialize};

use ndarray::{Array, ArrayView};
use petal_clustering::{Dbscan, Fit};
use petal_neighbors::distance::Euclidean;
use std::collections::HashMap;

#[derive(Serialize, Deserialize, Debug)]
pub struct Cluster2D {
    pub id: usize,
    pub x: f64,
    pub y: f64,
    pub size: f64,
}

struct Bounds2D {
    x_min: Option<f64>,
    y_min: Option<f64>,
    x_max: Option<f64>,
    y_max: Option<f64>,
}

pub struct ClusteringSystem {
    scan_points: HashMap<String, Vec<Point2D>>,
    clustering_engine: Dbscan<f64, Euclidean>,
    max_cluster_size: f64,
}

impl ClusteringSystem {
    pub fn new(
        neighbourhood_radius: f64,
        min_neighbourss: usize,
        max_cluster_size: f64,
    ) -> ClusteringSystem {
        ClusteringSystem {
            scan_points: HashMap::new(),
            clustering_engine: Dbscan {
                eps: neighbourhood_radius,
                min_samples: min_neighbourss,
                metric: Euclidean::default(),
            },
            max_cluster_size,
        }
    }

    pub fn handle_scan_message(
        &mut self,
        scans: &[(f64, f64)],
        device: &LidarDevice,
    ) -> Result<Vec<Cluster2D>, ()> {
        debug!("Decoded {} scans", scans.len());

        let mut points_this_scan: Vec<Point2D> = Vec::new();

        for sample in scans {
            let (angle, distance) = sample;

            if *distance > 0.0 {
                if let Some(point) = measurement_to_point(&angle, &distance, device) {
                    points_this_scan.push(point);
                }
            }
        }

        self.scan_points
            .insert(String::from(&device.serial), points_this_scan);

        let combined_points = self.combine_all_points();

        debug!(
            "Combined {} points from all devices",
            (combined_points.len() / 2)
        );

        let (clusters, outliers) = self.clustering_engine.fit(&combined_points);

        debug!("Clustering done");
        debug!(
            "Found {} clusters, {} outliers",
            clusters.len(),
            outliers.len()
        );

        // Shadowed "clusters" - now as Cluster2D ("points")
        let clusters: Vec<Cluster2D> = clusters
            .iter()
            .map(|c| {
                let (cluster_index, point_indexes) = c;
                let matched_points = point_indexes
                    .iter()
                    .map(|i| {
                        let point = combined_points.row(*i);
                        (point[0], point[1])
                        // Point2D {
                        //     x: point[0],
                        //     y: point[1],
                        // }
                    })
                    .collect();

                consolidate_cluster_points(matched_points, *cluster_index)
            })
            .filter(|cluster| cluster.size <= self.max_cluster_size)
            .collect();

        // let payload: Vec<u8> = to_vec_named(&clusters).unwrap();
        // let message = mqtt::Message::new(&self.output_topic, payload, mqtt::QOS_0);

        Ok(clusters)
    }

    pub fn combine_all_points(&self) -> ndarray::Array2<f64> {
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
Consolidate points in a cluster to a single "Cluster2D" (same as Point2D, but including size)
*/
pub fn consolidate_cluster_points(points: Vec<Point2D>, id: usize) -> Cluster2D {
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

fn measurement_to_point(angle: &f64, distance: &f64, device: &LidarDevice) -> Option<Point2D> {
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
                *x + (angle + *rotation).to_radians().cos() * distance,
                *y + (angle + *rotation).to_radians().sin() * distance,
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
                    *x + altered_angle.to_radians().cos() * *distance * (*flip_x as f64),
                    *y + altered_angle.to_radians().sin() * *distance * (*flip_y as f64),
                ))
            }
        }
    } else {
        None
    }
}

fn passes_mask_threshold(
    angle: &f64,
    distance: &f64,
    mask_thresholds: &Option<HashMap<String, f64>>,
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
