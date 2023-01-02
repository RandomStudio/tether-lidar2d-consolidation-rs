pub mod config_state {

    use paho_mqtt as mqtt;

    use rmp_serde as rmps;
    use rmps::to_vec_named;
    use serde::{Deserialize, Serialize};

    type ScanMaskThreshold = (f64, f64); // angle, distance
    #[derive(Serialize, Deserialize, Debug)]
    #[serde(rename_all = "camelCase")]
    pub struct LidarDevice {
        serial: String,
        name: String,
        rotation: f64,
        x: f64,
        y: f64,
        color: String,
        min_distance_threshold: f64,
        scan_mask_thresholds: Option<Vec<ScanMaskThreshold>>,
        flip_coords: Option<(i8, i8)>,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Config {
        devices: Vec<LidarDevice>,
    }

    impl Config {
        pub fn new() -> Config {
            Config { devices: vec![] }
        }

        pub fn publish_config(&self, provide_config_topic: &str) -> Result<mqtt::Message, ()> {
            let payload: Vec<u8> = to_vec_named(&self).unwrap();
            let message = mqtt::Message::new(provide_config_topic, payload, 2);
            Ok(message)
        }

        pub fn load_config_from_file(&mut self, path: &str) -> Result<usize, ()> {
            let text = std::fs::read_to_string(&path).unwrap();
            let data: Config = serde_json::from_str(&text).unwrap();

            println!("Config parsed data from file: {:?}", data);

            self.devices = data.devices;

            Ok(self.devices.len())
        }
    }

    // pub fn provide_lidar_config(provide_config_topic: &str) -> Result<mqtt::Message, ()> {}
}
