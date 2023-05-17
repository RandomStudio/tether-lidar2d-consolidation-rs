use std::time::SystemTime;

use crate::{tracking::TrackedPoint2D, Point2D};

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
    topic: String,
}

impl TrackingSmoother {
    pub fn new(settings: SmoothSettings, topic: &str) -> Self {
        TrackingSmoother {
            settings,
            known_points: Vec::new(),
            topic: String::from(topic),
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

                // If this "is" actually the same point, and only if it's "ready",
                // update its target position
                if existing.ready {
                    let (x, y) = *p;
                    existing.target_position = (x, y);
                }
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

    /// Do time-based smoothing of all known points, and also automatically expire any points
    /// that are "stale". This function should be called as often as possible, not necessarily
    /// only when a new TrackedPoint message comes in.
    pub fn update_smoothing(&mut self) {
        // First, remove all points which were waiting too long to become "active"...
        if let Some(i) = self
            .known_points
            .iter()
            .position(|p| match p.last_updated.elapsed() {
                Ok(elapsed) => elapsed.as_millis() > self.settings.wait_before_active_ms,
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

    pub fn get_smoothed_points(&self) -> Vec<TrackedPoint2D> {
        self.known_points
            .iter()
            .filter(|p| p.ready)
            .enumerate()
            .map(|(i, p)| TrackedPoint2D::new(i, p.current_position))
            .collect()
    }
}

fn distance(a: &Point2D, b: &Point2D) -> f64 {
    let (x1, y1) = a;
    let (x2, y2) = b;

    f64::sqrt(f64::powi(x1 - x2, 2) + f64::powi(y1 - y2, 2))
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + t * (a - b)
}
