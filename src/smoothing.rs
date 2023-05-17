use std::time::SystemTime;

use log::debug;

use crate::{tracking::TrackedPoint2D, Point2D};

pub enum EmptyListSendMode {
    Never,
    Once,
    Always,
}

pub struct SmoothSettings {
    pub merge_radius: f64,
    pub wait_before_active_ms: u128,
    pub expire_ms: u128,
    pub lerp_factor: f64,
    pub empty_list_send_mode: EmptyListSendMode,
}

#[derive(Debug)]
struct SmoothedPoint {
    current_position: Point2D,
    target_position: Point2D,
    ready: bool,
    first_updated: SystemTime,
    last_updated: SystemTime,
}

pub struct TrackingSmoother {
    settings: SmoothSettings,
    known_points: Vec<SmoothedPoint>,
    empty_lists_sent: u128,
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
        }
    }

    pub fn update_tracked_points(&mut self, points: &[TrackedPoint2D]) {
        points.iter().for_each(|new_point| {
            // Fist, check if this "is" actually an existing point that wasn't (yet)
            // marked active
            if let Some(existing) = self.known_points.iter_mut().find(|known_point| {
                let TrackedPoint2D { x, y, .. } = new_point;
                distance(&(*x, *y), &known_point.current_position) <= self.settings.merge_radius
            }) {
                // ---- CASE A: This "is" a point we already know

                if !existing.ready {
                    if let Ok(elapsed) = existing.first_updated.elapsed() {
                        if elapsed.as_millis() > self.settings.wait_before_active_ms {
                            debug!("Existing point {:?} ready to become active", &existing);
                            existing.ready = true;
                        }
                    } else {
                        panic!("Failed to get elapsed time");
                    }
                }

                // If this "is" actually the same point, update the time
                // it was last updated
                existing.last_updated = SystemTime::now();

                // If this "is" actually the same point, and only if it's "ready",
                // update its target position
                if existing.ready {
                    let TrackedPoint2D { x, y, .. } = new_point;
                    existing.target_position = (*x, *y);
                }
            } else {
                // ---- CASE B: This is not (close to) a point we already know

                // Append to list

                let TrackedPoint2D { x, y, .. } = new_point;

                debug!("Added new, unknown point {:?}", &new_point);

                let new_point = SmoothedPoint {
                    current_position: (*x, *y),
                    target_position: (*x, *y),
                    first_updated: SystemTime::now(),
                    last_updated: SystemTime::now(),
                    ready: {
                        if self.settings.wait_before_active_ms > 0 {
                            false
                        } else {
                            true
                        }
                    },
                };
                self.known_points.push(new_point);
            }
        });
    }

    /// Do time-based smoothing of all known points, and also automatically expire any points
    /// that are "stale". This function should be called as often as possible, not necessarily
    /// only when a new TrackedPoint message comes in.
    pub fn update_smoothing(&mut self) {
        // First, remove all points which were waiting too long to become "active"...
        if let Some(i) = self
            .known_points
            .iter()
            .position(|p| match p.last_updated.elapsed() {
                Ok(elapsed) => {
                    if elapsed.as_millis() > self.settings.wait_before_active_ms && !p.ready {
                        debug!(
                            "Remove point waiting too long to become active; {}ms > {} ms",
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

        // Next, smooth (lerp) points towards target positions
        self.known_points.iter_mut().for_each(|p| {
            let t = self.settings.lerp_factor;
            let (x1, y1) = p.current_position;
            let (x2, y2) = p.target_position;
            let [new_x, new_y] = [lerp(x1, x2, t), lerp(y1, y2, t)];
            p.current_position = (new_x, new_y);
        })
    }

    pub fn get_smoothed_points(&mut self) -> Option<Vec<TrackedPoint2D>> {
        let known_points: Vec<TrackedPoint2D> = self
            .known_points
            .iter()
            .filter(|p| p.ready)
            .enumerate()
            .map(|(i, p)| TrackedPoint2D::new(i, p.current_position))
            .collect();

        let points_count = known_points.len();

        let points = match self.settings.empty_list_send_mode {
            EmptyListSendMode::Always => Some(known_points),
            EmptyListSendMode::Once => {
                if known_points.len() > 0 || known_points.len() == 0 && self.empty_lists_sent < 1 {
                    Some(known_points)
                } else {
                    None
                }
            }
            EmptyListSendMode::Never => {
                if known_points.len() == 0 {
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
}

fn distance(a: &Point2D, b: &Point2D) -> f64 {
    let (x1, y1) = a;
    let (x2, y2) = b;

    f64::sqrt(f64::powi(x1 - x2, 2) + f64::powi(y1 - y2, 2))
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a * (1. - t) + (b * t)
}
