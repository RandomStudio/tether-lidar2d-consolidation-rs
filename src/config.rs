pub mod config_state {

    use msgpack_simple::{MapElement, MsgPack};
    use paho_mqtt as mqtt;

    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug)]
    pub struct LidarDevice {
        pub serial: String,
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
            let devices: Vec<MsgPack> = self
                .devices
                .iter()
                .map(|d| {
                    MsgPack::Map(vec![MapElement {
                        key: MsgPack::String("serial".to_string()),
                        value: MsgPack::String(d.serial.clone()),
                    }])
                })
                .collect();

            let payload = MsgPack::Map(vec![MapElement {
                key: MsgPack::String("devices".to_string()),
                value: MsgPack::Array(devices),
            }]);
            let message = mqtt::Message::new(provide_config_topic, payload.encode(), 2);
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
