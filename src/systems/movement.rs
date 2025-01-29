use std::time::{Duration, SystemTime};

use log::{error, info};

use crate::{tracking::TrackedPoint2D, Point2D};

pub struct AverageMovementAnalysis {
    last_updated: SystemTime,
}

impl AverageMovementAnalysis {
    pub fn new() -> Self {
        AverageMovementAnalysis {
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

pub fn calculate(points: &[TrackedPoint2D]) -> Point2D {
    points.iter().fold((0., 0.), |acc, p| {
        if let Some(v) = p.velocity {
            let [vx, vy] = v;
            (acc.0 + vx, acc.1 + vy)
        } else {
            error!("No velocity for points; is velocity calculation enabled?");
            acc
        }
    })
}

impl Default for AverageMovementAnalysis {
    fn default() -> Self {
        AverageMovementAnalysis::new()
    }
}
