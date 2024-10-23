use std::time::{Duration, SystemTime};

use crate::{tracking::TrackedPoint2D, Point2D};

pub struct MovementAnalysis {
    last_updated: SystemTime,
}

impl MovementAnalysis {
    pub fn new() -> Self {
        MovementAnalysis {
            last_updated: SystemTime::now(),
        }
    }

    pub fn get_elapsed(&self) -> Duration {
        self.last_updated.elapsed().unwrap_or_default()
    }

    pub fn reset_timer(&mut self) {
        self.last_updated = SystemTime::now();
    }
}

pub fn get_total_movement(points: &[TrackedPoint2D]) -> Point2D {
    points.iter().fold((0., 0.), |acc, p| {
        if let Some(v) = p.velocity {
            let [vx, vy] = v;
            (acc.0 + vx, acc.1 + vy)
        } else {
            acc
        }
    })
}

impl Default for MovementAnalysis {
    fn default() -> Self {
        MovementAnalysis::new()
    }
}
