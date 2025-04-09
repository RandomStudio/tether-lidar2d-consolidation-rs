use std::time::{Duration, SystemTime};

use log::debug;
use serde::{Deserialize, Serialize};
use tether_agent::{tether_compliant_topic::build_publish_topic, TetherAgent};

use crate::tracking::TrackedPoint2D;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Zone {
    pub id: usize,
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
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

pub fn publish_presence_change(changed_zone: &Zone, tether_agent: &TetherAgent) {
    debug!("ZONE CHANGED: {:?}", changed_zone);
    let topic = build_publish_topic(
        "presenceDetection",
        "presence",
        Some(&changed_zone.id.to_string()),
    );
    let payload = if changed_zone.active { &[1] } else { &[0] };
    tether_agent
        .publish_raw(&topic, payload, Some(2), Some(false))
        .expect("failed to send presence update");
}
