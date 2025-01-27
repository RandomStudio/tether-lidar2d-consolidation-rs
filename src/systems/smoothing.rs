use std::time::{Duration, SystemTime, UNIX_EPOCH};

use log::debug;
use serde::{Deserialize, Serialize};

use crate::{tracking::TrackedPoint2D, Point2D};

use super::position_remapping::OriginLocation;

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
    pub should_calculate_angles: bool,
}

#[derive(Debug)]
struct SmoothedPoint {
    id: usize,
    current_position: Point2D,
    target_position: Point2D,
    velocity: Option<[f32; 2]>,
    ready: bool,
    first_updated: SystemTime,
    last_updated: SystemTime,
    /// A list of raw tracking points currently in range of this point
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
    pub fn update_tracked_points(&mut self, points: &[Point2D]) {
        let mut marked_points_in_range: Vec<usize> = Vec::new();

        for sp in self.known_points.iter_mut() {
            sp.points_in_range.clear();
            let points_in_my_range: Vec<(usize, Point2D)> = points
                .iter()
                .enumerate()
                .filter(|(_i, p)| distance(p, &sp.current_position) <= self.settings.merge_radius)
                .map(|(i, p)| (i, *p))
                .collect();
            for (i, _p) in points_in_my_range.iter() {
                marked_points_in_range.push(*i);
                sp.points_in_range.push(*i);
            }
            if !points_in_my_range.is_empty() {
                // There were points in range; so update time
                sp.last_updated = SystemTime::now();
                // If the SmoothedPoint was not ready till now, check if it's time to mark it "ready"
                if !sp.ready
                    && sp.first_updated.elapsed().unwrap().as_millis()
                        > self.settings.wait_before_active_ms
                {
                    sp.ready = true;
                }
                // Finally, set the target position as the centroid between all the points in range
                if let Some(centroid) = centroid(
                    &points_in_my_range
                        .iter()
                        .map(|(_i, p)| *p)
                        .collect::<Vec<Point2D>>(),
                ) {
                    sp.target_position = centroid;
                }
            }
        }

        for (i, p) in points.iter().enumerate() {
            if !marked_points_in_range.contains(&i) {
                // Append to list
                let (x, y) = p;

                let timestamp = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .expect("time error");

                let new_point = SmoothedPoint {
                    id: timestamp.as_millis() as usize,
                    current_position: (*x, *y),
                    target_position: (*x, *y),
                    first_updated: SystemTime::now(),
                    last_updated: SystemTime::now(),
                    velocity: None,
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
    pub fn update_smoothing(&mut self) {
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
                                && distance(
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
                p.velocity = Some([x2 - x1, y2 - y1]);
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
                let mut tp = TrackedPoint2D::new(p.id, p.current_position);
                tp.set_velocity(p.velocity);
                if self.settings.should_calculate_angles {
                    let (x, y) = p.current_position;
                    tp.heading = Some(heading(x, y));
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

fn centroid(points: &[Point2D]) -> Option<Point2D> {
    let count = points.len();
    points
        .iter()
        .cloned()
        .reduce(|acc, el| (acc.0 + el.0, acc.1 + el.1))
        .map(|(x, y)| (x / count as f32, y / count as f32))
}

fn distance(a: &Point2D, b: &Point2D) -> f32 {
    let (x1, y1) = a;
    let (x2, y2) = b;

    f32::sqrt(f32::powi(x1 - x2, 2) + f32::powi(y1 - y2, 2))
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a * (1. - t) + (b * t)
}

/// Return the clockwise angle (in degrees) between two lines:
/// - origin to a point on the y-axis (0,a) where a is positive
/// - origin to the point (x,y)
///
/// This corresponds to the common-sense "heading" of a point
/// from the point-of-view of the origin
fn heading(x: f32, y: f32) -> f32 {
    let angle_rad = y.atan2(x); // Get the angle from the positive x-axis in radians
    let angle_deg = angle_rad.to_degrees(); // Convert to degrees

    // Convert from positive x-axis reference to positive y-axis reference
    let heading = (90.0 - angle_deg) % 360.0;
    if heading < 0.0 {
        heading + 360.0
    } else {
        heading
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heading_easy_cardinals() {
        assert_eq!(heading(0., 1.0), 0.); // N
        assert_eq!(heading(1.0, 1.0), 45.0); // NE
        assert_eq!(heading(3.5, 3.5), 45.0); // Also NE
        assert_eq!(heading(1.0, 0.), 90.0); // E
        assert_eq!(heading(1.0, -1.0), 135.); // SE
        assert_eq!(heading(0.0, -101.0), 180.); // S
        assert_eq!(heading(-1.0, -1.0), 225.); // SW
        assert_eq!(heading(-1.0, -0.), 270.); // W
        assert_eq!(heading(-3.1, 3.1), 315.); // NW
    }
}
