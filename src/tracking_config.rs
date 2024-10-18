use log::{debug, error, info, warn};
use std::{collections::HashMap, fmt::Error, fs};
use tether_agent::{mqtt::Message, PlugDefinition, TetherAgent};

use serde::{Deserialize, Serialize};

use crate::{automasking::MaskThresholdMap, perspective::PerspectiveTransformer, presence::Zone};

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct LidarDevice {
    pub serial: String,
    pub name: String,
    pub rotation: f32,
    pub x: f32,
    pub y: f32,
    pub color: String,
    pub min_distance_threshold: f32,
    pub scan_mask_thresholds: Option<MaskThresholdMap>,
    pub flip_coords: Option<(i8, i8)>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConfigRectCornerPoint {
    corner: u8,
    pub x: f32,
    pub y: f32,
}

type CornerPoints = (
    ConfigRectCornerPoint,
    ConfigRectCornerPoint,
    ConfigRectCornerPoint,
    ConfigRectCornerPoint,
);

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct TrackingConfig {
    devices: Vec<LidarDevice>,
    region_of_interest: Option<CornerPoints>,
    zones: Option<Vec<Zone>>,
    #[serde(skip)]
    config_file_path: String,
}

impl TrackingConfig {
    pub fn new(config_file_path: &str) -> TrackingConfig {
        TrackingConfig {
            devices: vec![],
            region_of_interest: None,
            zones: None,
            config_file_path: String::from(config_file_path),
        }
    }

    // pub async fn publish_config(&self, also_save: bool) -> Result<mqtt::Message, Error> {
    //     let payload: Vec<u8> = to_vec_named(&self).unwrap();
    //     let message = mqtt::Message::new(&self.output_topic, payload, 2);
    //     if also_save {
    //         self.write_config_to_file().await?;
    //         Ok(message)
    //     } else {
    //         Ok(message)
    //     }
    // }

    pub fn parse_remote_config(&mut self, incoming_message: &Message) -> Result<(), ()> {
        let payload = incoming_message.payload().to_vec();

        match rmp_serde::from_slice::<TrackingConfig>(&payload) {
            Ok(config) => {
                let TrackingConfig {
                    devices,
                    region_of_interest,
                    ..
                } = config;
                self.devices = devices;
                self.region_of_interest = region_of_interest;
                Ok(())
            }
            Err(e) => {
                error!("Failed to parse Config from message: {}", e);
                Err(())
            }
        }
    }

    pub fn load_config_from_file(&mut self) -> Result<usize, ()> {
        let text = match std::fs::read_to_string(&self.config_file_path) {
            Err(e) => {
                if e.kind().to_string() == "entity not found" {
                    warn!("Tracking Config file not found, will create a blank one");
                    String::from("{\"devices\": [] }")
                } else {
                    println!("kind: {}", e.kind());
                    panic!("Failed to load Tracking Config from disk; error: {:?}", e);
                }
            }
            Ok(s) => {
                info!(
                    "Loaded Tracking config OK from \"{}\"",
                    &self.config_file_path
                );
                s
            }
        };

        match serde_json::from_str::<TrackingConfig>(&text) {
            Ok(data) => {
                debug!("Config parsed data from file: {:?}", data);

                self.devices = data.devices;
                self.region_of_interest = data.region_of_interest;
                self.zones = data.zones;

                Ok(self.devices.len())
            }
            Err(e) => {
                error!("Failed to parse config data: {}", e);
                Err(())
            }
        }
    }

    pub fn write_config_to_file(&self) -> Result<(), Error> {
        info!("Current state of config: {:?}", self);
        let text = serde_json::to_string_pretty(self).unwrap();
        match fs::write(&self.config_file_path, text) {
            Ok(()) => {
                info!("Wrote config to file: {:?}", self.config_file_path);
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
                    color: pick_from_palette(self.devices.len()), // TODO: use random colour
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

    pub fn clear_device_masking(&mut self) {
        for d in self.devices.iter_mut() {
            d.scan_mask_thresholds = None;
        }
    }

    pub fn update_device_masking(
        &mut self,
        masking: &MaskThresholdMap,
        serial: &str,
    ) -> Result<(), ()> {
        let device = self.get_device_mut(serial);
        match device {
            Some(d) => {
                let mut m: MaskThresholdMap = HashMap::new();
                for (key, value) in masking {
                    m.insert(String::from(key), *value);
                }
                d.scan_mask_thresholds = Some(m);
                Ok(())
            }
            None => Err(()),
        }
    }

    pub fn get_device(&self, serial: &str) -> Option<&LidarDevice> {
        self.devices.iter().find(|&d| d.serial.eq(serial))
    }

    pub fn get_device_mut(&mut self, serial: &str) -> Option<&mut LidarDevice> {
        self.devices.iter_mut().find(|d| d.serial.eq(serial))
    }

    pub fn devices(&self) -> &Vec<LidarDevice> {
        &self.devices
    }

    pub fn devices_mut(&mut self) -> &mut Vec<LidarDevice> {
        &mut self.devices
    }

    pub fn region_of_interest(&self) -> Option<&CornerPoints> {
        self.region_of_interest.as_ref()
    }

    pub fn zones(&self) -> Option<&[Zone]> {
        self.zones.as_deref()
    }

    pub fn handle_save_message(
        &mut self,
        tether_agent: &TetherAgent,
        config_output: &PlugDefinition,
        incoming_message: &Message,
        perspective_transformer: &mut PerspectiveTransformer,
    ) -> Result<(), Error> {
        match self.parse_remote_config(incoming_message) {
            Ok(()) => {
                if let Some(region_of_interest) = self.region_of_interest() {
                    info!("New Region of Interest was provided remotely; update the Perspective Transformer");
                    let (c1, c2, c3, c4) = region_of_interest;
                    let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                    perspective_transformer.set_new_quad(&corners);
                }

                info!("Remote-provided config parsed OK; now save to disk and (re) publish");
                self.save_and_republish(tether_agent, config_output)
                // Ok(())
            }
            Err(()) => Err(Error),
        }
    }

    pub fn save_and_republish(
        &self,
        tether_agent: &TetherAgent,
        config_output: &PlugDefinition,
    ) -> Result<(), Error> {
        info!("Saving config to disk and re-publishing via Tether...");
        self.write_config_to_file().expect("failed to save to disk");

        tether_agent
            .encode_and_publish(config_output, self)
            .expect("failed to publish config");
        Ok(())
    }
}

const PALETTE: &[&str] = &["#ffff00", "#00ffff", "#ff00ff"];

fn pick_from_palette(index: usize) -> String {
    let c = PALETTE[index % PALETTE.len()];
    String::from(c)
}
