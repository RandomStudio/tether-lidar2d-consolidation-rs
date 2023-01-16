use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub type MaskThresholdMap = HashMap<String, f64>;

pub struct AutoMaskSampler {
    threshold_margin: f64,
    pub angles_with_thresholds: MaskThresholdMap,
    scans_remaining: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AutoMaskMessage {
    pub r#type: String,
}

impl AutoMaskSampler {
    pub fn new(required_scans_count: usize, threshold_margin: f64) -> AutoMaskSampler {
        AutoMaskSampler {
            threshold_margin,
            angles_with_thresholds: HashMap::new(),
            scans_remaining: required_scans_count,
        }
    }

    /** Add samples (vector of angles with distances) until sufficient scans have been recorded;
     * return the mapping once we're done, otherwise return None
     */
    pub fn add_samples(&mut self, samples: &Vec<(f64, f64)>) -> Option<&MaskThresholdMap> {
        self.scans_remaining = self.scans_remaining - 1;

        if self.scans_remaining > 0 {
            for (angle, distance) in samples {
                let distance_minus_threshold = *distance - self.threshold_margin;
                if *distance > 0. && distance_minus_threshold > 0. {
                    self.angles_with_thresholds
                        .insert(angle.to_string(), distance_minus_threshold);
                }
            }
            None
        } else {
            Some(&self.angles_with_thresholds)
        }
    }
}
