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
        rotation: f64,
        x: f64,
        y: f64,
        color: String,
        min_distance_threshold: f64,
        scan_mask_thresholds: Option<HashMap<String, f64>>,
        flip_coords: Option<(i8, i8)>,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Config {
        devices: Vec<LidarDevice>,
        #[serde(skip)]
        output_topic: String,
    }

    impl Config {
        pub fn new(output_topic: &str) -> Config {
            Config {
                devices: vec![],
                output_topic: String::from(output_topic),
            }
        }

        pub fn publish_config(&self) -> Result<mqtt::Message, ()> {
            let payload: Vec<u8> = to_vec_named(&self).unwrap();
            let message = mqtt::Message::new(&self.output_topic, payload, 2);
            Ok(message)
        }

        pub fn load_config_from_file(&mut self, path: &str) -> Result<usize, ()> {
            let text = std::fs::read_to_string(&path).unwrap();
            let data: Config = serde_json::from_str(&text).unwrap();

            println!("Config parsed data from file: {:?}", data);

            self.devices = data.devices;

            Ok(self.devices.len())
        }

        /**  If the device is known, return 0; if unknown, create it and report that 1
        device was created. */
        pub fn check_or_create_device(&mut self, serial: &str) -> Result<usize, Error> {
            let existing = self.devices.iter().find(|&d| d.serial.eq(serial));
            match existing {
                Some(_device) => Ok(0),
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
                    Ok(1)
                }
            }
        }

        pub fn get_device(&self, serial: &str) -> Option<&LidarDevice> {
            self.devices.iter().find(|&d| d.serial.eq(serial))
        }
    }
}
