use indexmap::IndexMap;
use log::{debug, error, info, warn};
use std::{fmt::Error, fs};
use tether_agent::{PlugDefinition, TetherAgent};

use anyhow::{anyhow, Result};
use serde::{Deserialize, Serialize};

use crate::systems::{
    automasking::MaskThresholdMap,
    position_remapping::{OriginLocation, PositionRemapping},
    presence::Zone,
    smoothing::EmptyListSendMode,
};

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct LidarDevice {
    pub serial: String,
    pub name: String,
    pub rotation: f32,
    pub x: f32,
    pub y: f32,
    pub colour: String,
    pub min_distance_threshold: f32,
    pub scan_mask_thresholds: Option<MaskThresholdMap>,
    pub flip_coords: Option<(i8, i8)>,
}

// #[derive(Serialize, Deserialize, Debug)]
// #[serde(rename_all = "camelCase")]
// pub struct ExternalTracker {
//     pub serial: String,
//     pub name: String,
//     pub rotation: f32,
//     pub x: f32,
//     pub y: f32,
//     pub color: String,
//     pub flip_coords: Option<(i8, i8)>,
// }

#[derive(Serialize, Deserialize, Debug)]
pub struct ConfigRectCornerPoint {
    corner: u8,
    pub x: f32,
    pub y: f32,
}

impl ConfigRectCornerPoint {
    pub fn new(corner_id: u8, x: f32, y: f32) -> Self {
        ConfigRectCornerPoint {
            corner: corner_id,
            x,
            y,
        }
    }
}

pub type CornerPoints = (
    ConfigRectCornerPoint,
    ConfigRectCornerPoint,
    ConfigRectCornerPoint,
    ConfigRectCornerPoint,
);

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct BackendConfig {
    pub devices: Vec<LidarDevice>,
    // pub external_trackers: Vec<ExternalTracker>,
    pub region_of_interest: Option<CornerPoints>,
    pub zones: Option<Vec<Zone>>,

    /// Default min distance threshold (in mm) to use for unconfigured new devices
    pub default_min_distance_threshold: f32,

    // -------- CLUSTERING SETTINGS
    /// Max distance in mm to a point which can be included in a cluster
    pub clustering_neighbourhood_radius: f32,

    /// Min points count that constitutes a valid cluster
    pub clustering_min_neighbours: usize,

    /// Exclude clusters above this size (where size is bigger of height/width bounds, in mm)
    pub clustering_max_cluster_size: f32,

    // -------- SMOOTHING SETTINGS
    /// Flag to disable integrated time-based "smoothed tracking" output. Note that this will
    /// also disable presence detection + movement analysis.
    pub smoothing_disable: bool,

    /// How close to count two points as the same Tracked Point
    pub smoothing_merge_radius: f32,

    /// How long (ms) before deciding a new point is valid/active
    pub smoothing_wait_before_active_ms: u128,

    /// How long (ms) before removing a non-updated known tracking point
    pub smoothing_expire_ms: u128,

    /// How much to interpolate (smooth) current position towards target position
    /// (1.0 is immediate, i.e. no smoothing, 0 is invalid)
    pub smoothing_lerp_factor: f32,

    /// How to treat empty smoothed tracking points lists - either send an empty
    /// list "once", "never" or "always"
    pub smoothing_empty_send_mode: EmptyListSendMode,

    /// How often (ms) to update smoothed tracking points - regardless of scan
    /// message rate
    pub smoothing_update_interval: u64,

    /// If enabled, smoothing will use "real units" (i.e. mm); otherwise the
    /// destination quad will be a normalised rect in the range [0;1] on both axes
    pub smoothing_use_real_units: bool,

    pub origin_location: OriginLocation,

    pub enable_velocity: bool,
    pub enable_heading: bool,
    pub enable_distance: bool,

    // -------- PERSPECTIVE TRANSFORM SETTINGS
    /// By default, we drop tracking points (resolved clusters) that lie outside of the defined quad
    /// **(with a little margin for error; see perspectiveTransform.ignoreOutsideMargin)**;
    /// enable (use) this flag to include them all (no filtering)
    pub transform_include_outside: bool,

    /// **Unless perspectiveTransform.includeOutside is enabled**, drop tracking points beyond this
    /// distance from the edges of the destination quad, i.e. tge range [0-margin,1+margin]
    pub transform_ignore_outside_margin: f32,

    // -------- AUTOMASKING SETTINGS
    pub automask_scans_required: usize,
    pub automask_threshold_margin: f32,

    // -------- MOVEMENT ANALYSIS SETTINGS
    /// Disable movement analysis calculation and output, even if available
    pub enable_average_movement: bool,

    /// How often (ms) to send movement messages
    pub average_movement_interval: u64,

    /// If enabled, skip publishing messages that are typically only used by the lidar2d-frontend
    /// Can reduce I/O load and improve broker performance
    pub skip_some_outputs: bool,
}

impl Default for BackendConfig {
    fn default() -> Self {
        BackendConfig {
            devices: Vec::new(),
            // external_trackers: Vec::new(),
            region_of_interest: None,
            zones: None,
            smoothing_use_real_units: true,
            default_min_distance_threshold: 20.,
            clustering_neighbourhood_radius: 200.,
            clustering_min_neighbours: 4,
            clustering_max_cluster_size: 2500.,
            smoothing_disable: false,
            smoothing_merge_radius: 100.,
            smoothing_wait_before_active_ms: 100,
            smoothing_expire_ms: 3000,
            smoothing_lerp_factor: 0.1,
            smoothing_empty_send_mode: EmptyListSendMode::Once,
            smoothing_update_interval: 16,
            origin_location: OriginLocation::Centre,
            transform_include_outside: false,
            transform_ignore_outside_margin: 0.,
            automask_scans_required: 60,
            automask_threshold_margin: 50.,
            enable_average_movement: false,
            average_movement_interval: 250,
            enable_velocity: false,
            enable_heading: false,
            enable_distance: false,
            skip_some_outputs: false,
        }
    }
}

impl BackendConfig {
    pub fn parse_remote_config(&mut self, payload: &[u8]) -> Result<()> {
        match rmp_serde::from_slice::<BackendConfig>(payload) {
            Ok(config) => {
                *self = config;
                Ok(())
            }
            Err(e) => Err(anyhow!("Failed to parse Config from message: {}", e)),
        }
    }

    pub fn write_config_to_file(&self, config_file_path: &str) -> Result<(), Error> {
        info!("Current state of config: {:?}", self);
        let text = serde_json::to_string_pretty(self).unwrap();
        match fs::write(config_file_path, text) {
            Ok(()) => {
                info!("Wrote config to file: {:?}", config_file_path);
                Ok(())
            }
            Err(e) => {
                error!("Error writing config to file: {:?}", e);
                Err(Error)
            }
        }
    }

    /**  If the device is known, return None; if unknown, create it and return
    Some(())
    */
    pub fn check_or_create_device(
        &mut self,
        serial: &str,
        default_min_distance: f32,
    ) -> Option<()> {
        let existing = self.devices.iter().find(|&d| d.serial.eq(serial));
        match existing {
            Some(_device) => None,
            None => {
                warn!("Unrecognised device for serial {}", serial);
                let new_device = LidarDevice {
                    serial: String::from(serial),
                    name: String::from(serial),
                    rotation: 0.,
                    x: 0.,
                    y: 0.,
                    colour: pick_from_palette(self.devices.len()), // TODO: use random colour
                    min_distance_threshold: default_min_distance,
                    scan_mask_thresholds: None,
                    flip_coords: None,
                };
                self.devices.push(new_device);
                info!("Creating a device with defaults for serial {}", serial);
                Some(())
            }
        }
    }

    // /**  If the external tracker is known, return None; if unknown, create it and return
    // Some(())
    // */
    // pub fn check_or_create_external_tracker(&mut self, serial: &str) -> Option<()> {
    //     let existing = self
    //         .external_trackers()
    //         .iter()
    //         .find(|&d| d.serial.eq(serial));
    //     match existing {
    //         Some(_tracker) => None,
    //         None => {
    //             warn!("Unrecognised tracker for serial {}", serial);
    //             let new_tracker = ExternalTracker {
    //                 serial: String::from(serial),
    //                 name: String::from(serial),
    //                 rotation: 0.,
    //                 x: 0.,
    //                 y: 0.,
    //                 color: pick_from_palette(self.devices.len()), // TODO: use random colour
    //                 flip_coords: None,
    //             };
    //             // self.external_trackers.push(new_tracker);
    //             info!(
    //                 "Creating an external tracker with defaults for serial {}",
    //                 serial
    //             );
    //             Some(())
    //         }
    //     }
    // }

    pub fn clear_device_masking(&mut self) {
        for d in self.devices.iter_mut() {
            d.scan_mask_thresholds = None;
        }
    }

    pub fn update_device_masking(
        &mut self,
        masking: &MaskThresholdMap,
        serial: &str,
    ) -> Result<()> {
        let device = self.get_device_mut(serial);
        match device {
            Some(d) => {
                let mut m: MaskThresholdMap = IndexMap::new();
                for (key, value) in masking {
                    m.insert(String::from(key), *value);
                }
                d.scan_mask_thresholds = Some(m);
                Ok(())
            }
            None => Err(anyhow!("could not find device with serial {}", serial)),
        }
    }

    pub fn get_device(&self, serial: &str) -> Option<&LidarDevice> {
        self.devices.iter().find(|&d| d.serial.eq(serial))
    }

    pub fn get_device_mut(&mut self, serial: &str) -> Option<&mut LidarDevice> {
        self.devices.iter_mut().find(|d| d.serial.eq(serial))
    }

    // pub fn get_external_tracker(&self, serial: &str) -> Option<&ExternalTracker> {
    //     self.external_trackers.iter().find(|&d| d.serial.eq(serial))
    // }

    // pub fn get_external_tracker_mut(&mut self, serial: &str) -> Option<&mut ExternalTracker> {
    //     self.external_trackers
    //         .iter_mut()
    //         .find(|d| d.serial.eq(serial))
    // }

    pub fn devices(&self) -> &Vec<LidarDevice> {
        &self.devices
    }

    pub fn devices_mut(&mut self) -> &mut Vec<LidarDevice> {
        &mut self.devices
    }

    // pub fn external_trackers(&self) -> &Vec<ExternalTracker> {
    //     &self.external_trackers
    // }

    // pub fn external_trackers_mut(&mut self) -> &mut Vec<ExternalTracker> {
    //     &mut self.external_trackers
    // }

    pub fn region_of_interest(&self) -> Option<&CornerPoints> {
        self.region_of_interest.as_ref()
    }

    pub fn region_of_interest_mut(&mut self) -> Option<&mut CornerPoints> {
        self.region_of_interest.as_mut()
    }

    pub fn zones(&self) -> Option<&[Zone]> {
        self.zones.as_deref()
    }

    pub fn handle_save_message(
        &mut self,
        tether_agent: &TetherAgent,
        config_output: &PlugDefinition,
        payload: &[u8],
        position_remapping: &mut PositionRemapping,
        config_file_path: &str,
    ) -> anyhow::Result<()> {
        match self.parse_remote_config(payload) {
            Ok(()) => {
                if let Some(region_of_interest) = self.region_of_interest() {
                    info!("New Region of Interest was provided remotely; update the Perspective Transformer");
                    position_remapping.update_with_roi(
                        region_of_interest,
                        self.origin_location,
                        self.smoothing_use_real_units,
                    );
                }

                info!("Remote-provided config parsed OK; now save to disk and (re) publish");
                self.save_and_republish(tether_agent, config_output, config_file_path)
                // Ok(())
            }
            Err(e) => Err(anyhow!("Handle save-message failure: {e}")),
        }
    }

    pub fn save_and_republish(
        &self,
        tether_agent: &TetherAgent,
        config_output: &PlugDefinition,
        config_file_path: &str,
    ) -> Result<()> {
        info!("Saving config to disk and re-publishing via Tether...");
        self.write_config_to_file(config_file_path)
            .expect("failed to save to disk");

        tether_agent
            .encode_and_publish(config_output, self)
            .expect("failed to publish config");
        Ok(())
    }
}

// TODO: some more imaginative colours, please?
const PALETTE: &[&str] = &["#ffff00", "#00ffff", "#ff00ff"];

fn pick_from_palette(index: usize) -> String {
    let c = PALETTE[index % PALETTE.len()];
    String::from(c)
}

pub fn load_config_from_file(config_file_path: &str) -> Result<BackendConfig> {
    let config = BackendConfig::default();
    debug!("Created init config object {:?}", config);

    match std::fs::read_to_string(config_file_path) {
        Err(e) => {
            if e.kind().to_string() == "entity not found" {
                warn!(
                    "Tracking Config file not found, will create a blank one at {}",
                    &config_file_path
                );
                Ok(config)
            } else {
                println!("kind: {}", e.kind());
                panic!("Failed to load Tracking Config from disk; error: {:?}", e);
            }
        }
        Ok(s) => {
            info!("Loaded Tracking config OK from \"{}\"", config_file_path);
            match serde_json::from_str::<BackendConfig>(&s) {
                Ok(loaded_config) => {
                    debug!("Config parsed data from file: {:?}", &loaded_config);
                    Ok(loaded_config)
                }
                Err(e) => Err(anyhow!("Failed to parse config data: {}", e)),
            }
        }
    }
}
