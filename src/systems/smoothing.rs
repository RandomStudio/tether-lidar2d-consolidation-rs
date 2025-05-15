use std::time::{Duration, SystemTime, UNIX_EPOCH};

use log::debug;
use serde::{Deserialize, Serialize};

use crate::{
    geometry_utils::{bearing, centroid, distance, distance_points, lerp},
    tracking::TrackedPoint2D,
    Point2D,
};

use super::{clustering::Cluster2D, position_remapping::OriginLocation};

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum EmptyListSendMode {
    Never,
    Once,
    Always,
}

pub struct SmoothSettings {
    pub merge_radius: f32,
    pub wait_before_active_ms: u128,
    pub expire_ms: u128,
    pub lerp_factor: f32,
    pub empty_list_send_mode: EmptyListSendMode,
    pub origin_mode: OriginLocation,
    pub should_calculate_velocity: bool,
    pub should_calculate_bearing: bool,
    pub should_calculate_range: bool,
}

#[derive(Debug)]
struct SmoothedPoint {
    id: usize,
    /// This is the **diameter**
    size: f32,
    current_position: Point2D,
    target_position: Point2D,
    velocity: Option<[f32; 2]>,
    distance: Option<f32>,
    ready: bool,
    first_updated: SystemTime,
    last_updated: SystemTime,
    /// A list of raw tracking point **indexes** currently in range of this point
    points_in_range: Vec<usize>,
}

pub struct TrackingSmoother {
    settings: SmoothSettings,
    known_points: Vec<SmoothedPoint>,
    empty_lists_sent: u128,
    last_updated: SystemTime,
}

impl TrackingSmoother {
    pub fn new(settings: SmoothSettings) -> Self {
        if settings.lerp_factor <= 0. {
            panic!("Smoothing lerp factor must be above 0");
        }
        TrackingSmoother {
            settings,
            known_points: Vec::new(),
            empty_lists_sent: 0,
            last_updated: SystemTime::now(),
        }
    }

    /// Add some raw points (clusters, position data, etc.) to the tracking-smoothing system
    pub fn update_tracked_points(&mut self, incoming_clusters: &[Cluster2D]) {
        let mut marked_points_in_range_indexes: Vec<usize> = Vec::new();

        for known_point in self.known_points.iter_mut() {
            known_point.points_in_range.clear();
            let clusters_in_my_range: Vec<(usize, Cluster2D)> = incoming_clusters
                .iter()
                .enumerate()
                .filter(|(_i, p)| {
                    distance_points(&(p.x, p.y), &known_point.target_position) <= known_point.size
                    // <= self.settings.merge_radius
                })
                .map(|(i, c)| (i, c.clone()))
                .collect();
            for (i, c) in clusters_in_my_range.iter() {
                marked_points_in_range_indexes.push(*i);
                known_point.points_in_range.push(*i);
                known_point.size = known_point.size.max(c.size); // only allowed to grow, not shrink
            }
            if !clusters_in_my_range.is_empty() {
                // There were points in range; so update time
                known_point.last_updated = SystemTime::now();
                // If the SmoothedPoint was not ready till now, check if it's time to mark it "ready"
                if !known_point.ready
                    && known_point.first_updated.elapsed().unwrap().as_millis()
                        > self.settings.wait_before_active_ms
                {
                    known_point.ready = true;
                }
                // Finally, set the target position as the centroid between all the points in range
                if let Some(centroid) = centroid(
                    &clusters_in_my_range
                        .iter()
                        .map(|(_i, c)| (c.x, c.y))
                        .collect::<Vec<Point2D>>(),
                ) {
                    known_point.target_position = centroid;
                }
            }
        }

        for (i, p) in incoming_clusters.iter().enumerate() {
            if !marked_points_in_range_indexes.contains(&i) {
                // Does not appear in known points; append to list
                let (x, y) = (p.x, p.y);

                // let timestamp = SystemTime::now()
                //     .duration_since(UNIX_EPOCH)
                //     .expect("time error");
                //
                let id = {
                    if self.known_points.is_empty() {
                        0
                    } else {
                        let mut i = 0;
                        for check in 0..=self.known_points.len() {
                            if !self.known_points.iter().any(|p| p.id == check) {
                                i = check;
                                break;
                            } else {
                                i += 1;
                            }
                        }
                        i
                    }
                };

                let new_point = SmoothedPoint {
                    id,
                    size: p.size,
                    current_position: (x, y),
                    target_position: (x, y),
                    first_updated: SystemTime::now(),
                    last_updated: SystemTime::now(),
                    velocity: None,
                    distance: if self.settings.should_calculate_range {
                        Some(distance(x, y, 0., 0.))
                    } else {
                        None
                    },
                    ready: self.settings.wait_before_active_ms == 0,
                    points_in_range: Vec::new(), // will be cleared next frame, anyway
                };
                debug!("Added new, unknown point {:?}", &new_point);

                self.known_points.push(new_point);
            }
        }
    }

    /// Do time-based smoothing of all known points, and also automatically expire any points
    /// that are "stale". This function should be called as often as possible, not necessarily
    /// only when a new TrackedPoint message comes in.
    pub fn update_smoothing(&mut self, interval: u64) {
        self.last_updated = SystemTime::now();
        // First, remove all points which were waiting too long to become "active"...
        if let Some(i) = self
            .known_points
            .iter()
            .position(|p| match p.last_updated.elapsed() {
                Ok(elapsed) => {
                    if elapsed.as_millis() > self.settings.wait_before_active_ms && !p.ready {
                        debug!(
                            "Remove point {:?} waiting too long to become active; {}ms > {} ms",
                            p,
                            elapsed.as_millis(),
                            self.settings.wait_before_active_ms
                        );
                        true
                    } else {
                        false
                    }
                }
                Err(_) => false,
            })
        {
            // swap_remove is a bit faster than remove,
            // and we don't care about the order
            self.known_points.swap_remove(i);
        }

        // Next, remove any duplicate points (within merge radius of each other)...
        let mut duplicate_index = None;
        self.known_points
            .iter()
            .enumerate()
            .for_each(|(this_index, this_point)| {
                if let Some((other_index, other_point)) =
                    self.known_points
                        .iter()
                        .enumerate()
                        .find(|(other_index, other_point)| {
                            *other_index != this_index
                                && other_point.ready
                                && distance_points(
                                    &other_point.current_position,
                                    &this_point.current_position,
                                ) < self.settings.merge_radius
                        })
                {
                    if other_point.first_updated.gt(&this_point.first_updated) {
                        duplicate_index = Some(other_index);
                    } else {
                        duplicate_index = Some(this_index);
                    }
                }
            });
        if let Some(i) = duplicate_index {
            self.known_points.swap_remove(i);
        };

        // Next, remove all points which were active but have now expired...
        if let Some(i) = self
            .known_points
            .iter()
            .position(|p| match p.last_updated.elapsed() {
                Ok(elapsed) => elapsed.as_millis() > self.settings.expire_ms,
                Err(_) => false,
            })
        {
            debug!("Remove point expired");
            self.known_points.swap_remove(i);
        }

        // Next, smooth (lerp) points towards target positions...
        self.known_points.iter_mut().for_each(|p| {
            let t = self.settings.lerp_factor;
            let (x1, y1) = p.current_position;
            let (x2, y2) = p.target_position;
            let [new_x, new_y] = [lerp(x1, x2, t), lerp(y1, y2, t)];
            if self.settings.should_calculate_velocity {
                p.velocity = Some([
                    (new_x - x1) / interval as f32 * 1000.,
                    (new_y - y1) / interval as f32 * 1000.,
                ]);
            }
            if self.settings.should_calculate_range {
                p.distance = Some(distance(x1, y1, 0., 0.));
            }
            p.current_position = (new_x, new_y);
        })
    }

    pub fn get_active_smoothed_points(&mut self) -> Option<Vec<TrackedPoint2D>> {
        let known_points: Vec<TrackedPoint2D> = self
            .known_points
            .iter()
            .filter(|p| p.ready)
            .map(|p| {
                let mut tp = TrackedPoint2D::new(p.id, p.current_position, Some(p.size));
                tp.velocity = p.velocity;
                tp.range = p.distance;
                if self.settings.should_calculate_bearing {
                    let (x, y) = p.current_position;
                    tp.bearing = Some(bearing(x, y));
                }
                tp
            })
            .collect();

        let points_count = known_points.len();

        let points = match self.settings.empty_list_send_mode {
            EmptyListSendMode::Always => Some(known_points),
            EmptyListSendMode::Once => {
                if !known_points.is_empty() || known_points.is_empty() && self.empty_lists_sent < 1
                {
                    Some(known_points)
                } else {
                    None
                }
            }
            EmptyListSendMode::Never => {
                if known_points.is_empty() {
                    None
                } else {
                    Some(known_points)
                }
            }
        };

        if points_count == 0 {
            self.empty_lists_sent += 1; // count
        } else {
            self.empty_lists_sent = 0; // reset
        }

        points
    }

    pub fn get_elapsed(&self) -> Duration {
        self.last_updated.elapsed().unwrap_or_default()
    }
}
