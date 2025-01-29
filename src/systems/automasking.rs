use anyhow::{anyhow, Result};
use indexmap::IndexMap;
use log::info;
use serde::{Deserialize, Serialize};
use tether_agent::mqtt::Message;

use crate::{backend_config::BackendConfig, Point2D};

pub type MaskThresholdMap = IndexMap<String, f32>;

pub struct AutoMaskSampler {
    threshold_margin: f32,
    pub angles_with_thresholds: MaskThresholdMap,
    scans_remaining: usize,
}

pub type AutoMaskSamplerMap = IndexMap<String, AutoMaskSampler>;

#[derive(Serialize, Deserialize, Debug)]
pub struct AutoMaskMessage {
    pub r#type: String,
}

impl AutoMaskSampler {
    pub fn new(required_scans_count: usize, threshold_margin: f32) -> AutoMaskSampler {
        AutoMaskSampler {
            threshold_margin,
            angles_with_thresholds: IndexMap::new(),
            scans_remaining: required_scans_count,
        }
    }

    /** Add samples (vector of angles with distances) until sufficient scans have been recorded;
     * return the mapping once we're done, otherwise return None
     */
    pub fn add_samples(&mut self, samples: &[Point2D]) -> Option<&MaskThresholdMap> {
        self.scans_remaining -= 1;

        if self.scans_remaining > 0 {
            for (angle, distance) in samples {
                let distance_minus_threshold = *distance - self.threshold_margin;
                if *distance > 0. && distance_minus_threshold > 0. {
                    self.angles_with_thresholds
                        .insert(angle.round().to_string(), distance_minus_threshold);
                }
            }
            None
        } else {
            info!(
                "Set new automask using {} angles",
                self.angles_with_thresholds.len()
            );
            Some(&self.angles_with_thresholds)
        }
    }

    pub fn is_complete(&self) -> bool {
        self.scans_remaining == 0
    }
}

/// Process a command relating to "automasking", returns true in the
/// Result if the action requires re-saving and re-publishing the
/// updated Tracking Config.
pub fn handle_automask_message(
    incoming_message: &Message,
    automask_samplers: &mut IndexMap<String, AutoMaskSampler>,
    config: &mut BackendConfig,
) -> Result<bool> {
    let payload = incoming_message.payload().to_vec();

    if let Ok(automask_command) = rmp_serde::from_slice::<AutoMaskMessage>(&payload) {
        let command_type: &str = &automask_command.r#type;
        match command_type {
            "new" => {
                info!("request NEW auto mask samplers");
                automask_samplers.clear();
                config.clear_device_masking();
                for device in config.devices().iter() {
                    automask_samplers.insert(
                        String::from(&device.serial),
                        AutoMaskSampler::new(
                            config.automask_scans_required,
                            config.automask_threshold_margin,
                        ),
                    );
                }
                Ok(false)
            }
            "clear" => {
                info!("request CLEAR all device masking thresholds");
                automask_samplers.clear();
                config.clear_device_masking();
                Ok(true)
            }
            _ => Err(anyhow!(
                "Unrecognised command type for RequestAutoMask message"
            )),
        }
    } else {
        Err(anyhow!("Failed to parse auto mask command"))
    }
}
