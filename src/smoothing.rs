use std::time::SystemTime;

use crate::Point2D;

pub struct SmoothSettings {
    merge_radius: f64,
    expire_ms: u128,
    lerp_factor: f64,
    wait_before_active_ms: u128,
}

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
}

impl TrackingSmoother {
    pub fn new(settings: SmoothSettings) -> Self {
        TrackingSmoother {
            settings,
            known_points: Vec::new(),
        }
    }

    pub fn update_tracked_points(&mut self, points: &[Point2D]) {
        points.iter().for_each(|p| {
            // Fist, check if this "is" actually an existing point that wasn't (yet)
            // marked active
            if let Some(existing) = self.known_points.iter_mut().find(|known_point| {
                let (x, y) = *p;
                distance(&(x, y), &known_point.current_position) > self.settings.merge_radius
            }) {
                // ---- CASE A: This "is" a point we already know

                // TODO: Do we need to check if wait_before_active_ms > 0 ?
                if !existing.ready {
                    if let Ok(elapsed) = existing.first_updated.elapsed() {
                        if elapsed.as_millis() > self.settings.wait_before_active_ms {
                            existing.ready = true;
                        }
                    }
                }
                // If this "is" actually the same point, update the time
                // it was last updated
                existing.last_updated = SystemTime::now();

                // If this "is" actually the same point, update its target position
                let (x, y) = *p;
                existing.target_position = (x, y);
            } else {
                // ---- CASE B: This is not (close to) a point we already know

                // Append to list

                let (x, y) = *p;

                let new_point = SmoothedPoint {
                    current_position: (x, y),
                    target_position: (x, y),
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
}

/// Do time-based smoothing of all known points, and also automatically expire any points
/// that are "stale". This function should be called as often as possible, not necessarily
/// only when a new TrackedPoint message comes in.
pub fn update_smoothing() {}

fn distance(a: &Point2D, b: &Point2D) -> f64 {
    let (x1, y1) = a;
    let (x2, y2) = b;

    f64::sqrt(f64::powi(x1 - x2, 2) + f64::powi(y1 - y2, 2))
}
