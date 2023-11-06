use std::time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

use crate::tracking::TrackedPoint2D;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Zone {
    pub id: usize,
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
    #[serde(default)]
    pub active: bool,
    #[serde(skip)]
    last_active: Option<SystemTime>,
}

pub struct PresenceDetectionZones {
    zones: Vec<Zone>,
    timeout: Duration,
}

impl PresenceDetectionZones {
    pub fn new(zones: &[Zone]) -> Self {
        PresenceDetectionZones {
            zones: Vec::from(zones),
            timeout: Duration::from_millis(500),
        }
    }

    pub fn update_zones(&mut self, points: &[TrackedPoint2D]) -> Vec<Zone> {
        let mut zones_changed = Vec::new();

        for p in points {
            let TrackedPoint2D { x, y, .. } = p;
            for zone in self
                .zones
                .iter_mut()
                .filter(|z| *x > z.x && *y > z.y && *x < z.x + z.width && *y < z.y + z.height)
            {
                zone.last_active = Some(SystemTime::now());
                if !zone.active {
                    zone.active = true;
                    zones_changed.push(zone.clone());
                }
            }
        }

        for zone in self.zones.iter_mut() {
            if let Some(timestamp) = zone.last_active {
                if zone.active
                    && timestamp.elapsed().expect("failed to get elapsed time") > self.timeout
                {
                    zone.active = false;
                    zones_changed.push(zone.clone());
                }
            }
        }

        zones_changed
    }

    // pub fn get_zones(&self) -> &[Zone] {
    //     &self.zones
    // }
}
