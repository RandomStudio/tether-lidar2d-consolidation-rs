pub mod config_state {

    use std::{collections::HashMap, fmt::Error};

    use paho_mqtt as mqtt;

    use rmp_serde as rmps;
    use rmps::to_vec_named;
    use serde::{Deserialize, Serialize};

    use crate::automasking::{AutoMaskSampler, MaskThresholdMap};

    #[derive(Serialize, Deserialize, Debug)]
    #[serde(rename_all = "camelCase")]
    pub struct LidarDevice {
        pub serial: String,
        name: String,
        pub rotation: f64,
        pub x: f64,
        pub y: f64,
        color: String,
        pub min_distance_threshold: f64,
        pub scan_mask_thresholds: Option<MaskThresholdMap>,
        pub flip_coords: Option<(i8, i8)>,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct ConfigRectCornerPoint {
        corner: u8,
        pub x: f64,
        pub y: f64,
    }

    type CornerPoints = (
        ConfigRectCornerPoint,
        ConfigRectCornerPoint,
        ConfigRectCornerPoint,
        ConfigRectCornerPoint,
    );

    #[derive(Serialize, Deserialize, Debug)]
    #[serde(rename_all = "camelCase")]
    pub struct Config {
        devices: Vec<LidarDevice>,
        region_of_interest: Option<CornerPoints>,
        #[serde(skip)]
        output_topic: String,
        #[serde(skip)]
        config_file_path: String,
    }

    impl Config {
        pub fn new(output_topic: &str, config_file_path: &str) -> Config {
            Config {
                devices: vec![],
                region_of_interest: None,
                output_topic: String::from(output_topic),
                config_file_path: String::from(config_file_path),
            }
        }

        pub fn publish_config(&self, also_save: bool) -> Result<mqtt::Message, ()> {
            let payload: Vec<u8> = to_vec_named(&self).unwrap();
            let message = mqtt::Message::new(&self.output_topic, payload, 2);
            if also_save {
                self.write_config_to_file().unwrap();
            }
            Ok(message)
        }

        pub fn parse_remote_config(&mut self, incoming_message: &mqtt::Message) -> Result<(), ()> {
            let payload = incoming_message.payload().to_vec();

            match rmp_serde::from_slice::<Config>(&payload) {
                Ok(config) => {
                    let Config {
                        devices,
                        region_of_interest,
                        ..
                    } = config;
                    self.devices = devices;
                    self.region_of_interest = region_of_interest;
                    Ok(())
                }
                Err(e) => {
                    println!("Failed to parse Config from message: {}", e);
                    Err(())
                }
            }
        }

        pub fn load_config_from_file(&mut self) -> Result<usize, ()> {
            let text =
                std::fs::read_to_string(&self.config_file_path).expect("Error opening config file");

            match serde_json::from_str::<Config>(&text) {
                Ok(data) => {
                    println!("Config parsed data from file: {:?}", data);

                    self.devices = data.devices;
                    self.region_of_interest = data.region_of_interest;

                    Ok(self.devices.len())
                }
                Err(e) => {
                    println!("Failed to parse config data: {}", e);
                    Err(())
                }
            }
        }

        pub fn write_config_to_file(&self) -> Result<(), Error> {
            let text = serde_json::to_string_pretty(self).unwrap();
            std::fs::write(&self.config_file_path, text).unwrap();

            println!("Wrote config to file: {:?}", self.config_file_path);

            Ok(())
        }

        /**  If the device is known, return None; if unknown, create it and return
        Some(())
        */
        pub fn check_or_create_device(&mut self, serial: &str) -> Option<()> {
            let existing = self.devices.iter().find(|&d| d.serial.eq(serial));
            match existing {
                Some(_device) => None,
                None => {
                    let new_device = LidarDevice {
                        serial: String::from(serial),
                        name: String::from(serial),
                        rotation: 0.,
                        x: 0.,
                        y: 0.,
                        color: String::from("#ffffff"), // TODO: use random colour
                        min_distance_threshold: 0.,
                        scan_mask_thresholds: None,
                        flip_coords: None,
                    };
                    self.devices.push(new_device);
                    Some(())
                }
            }
        }

        pub fn clear_device_masking(&mut self) {
            for d in self.devices.iter_mut() {
                d.scan_mask_thresholds = None;
            }
        }

        pub fn get_device(&self, serial: &str) -> Option<&LidarDevice> {
            self.devices.iter().find(|&d| d.serial.eq(serial))
        }

        pub fn devices(&self) -> &Vec<LidarDevice> {
            &self.devices
        }

        pub fn region_of_interest(&self) -> Option<&CornerPoints> {
            self.region_of_interest.as_ref()
        }
    }
}
