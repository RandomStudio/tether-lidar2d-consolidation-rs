pub mod config_state {

    use std::{collections::HashMap, fmt::Error};

    use paho_mqtt as mqtt;

    use rmp_serde as rmps;
    use rmps::to_vec_named;
    use serde::{Deserialize, Serialize};

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
        pub scan_mask_thresholds: Option<HashMap<String, f64>>,
        pub flip_coords: Option<(i8, i8)>,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Config {
        devices: Vec<LidarDevice>,
        #[serde(skip)]
        output_topic: String,
        #[serde(skip)]
        config_file_path: String,
    }

    impl Config {
        pub fn new(output_topic: &str, config_file_path: &str) -> Config {
            Config {
                devices: vec![],
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

        pub fn load_config_from_file(&mut self) -> Result<usize, ()> {
            let text = std::fs::read_to_string(&self.config_file_path).unwrap();
            let data: Config = serde_json::from_str(&text).unwrap();

            println!("Config parsed data from file: {:?}", data);

            self.devices = data.devices;

            Ok(self.devices.len())
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

        pub fn get_device(&self, serial: &str) -> Option<&LidarDevice> {
            self.devices.iter().find(|&d| d.serial.eq(serial))
        }
    }
}
