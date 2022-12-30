pub mod config_state {

    use msgpack_simple::{MapElement, MsgPack};
    use paho_mqtt as mqtt;

    use serde::{Deserialize, Serialize};
    use serde_json::Value;

    #[derive(Serialize, Deserialize, Debug)]
    pub struct LidarDevice {
        pub serial: String,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct ConfigManager {
        devices: Vec<LidarDevice>,
    }

    impl ConfigManager {
        pub fn new() -> ConfigManager {
            ConfigManager { devices: vec![] }
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

        pub fn load_lidar_config(&mut self, devices: Vec<LidarDevice>) -> Result<usize, ()> {
            // self.devices.clear();
            self.devices = devices;
            Ok(self.devices.len())
        }

        pub fn load_config_from_file(&mut self, path: &str) -> Result<(), ()> {
            let text = std::fs::read_to_string(&path).unwrap();
            let data = serde_json::from_str::<Value>(&text).unwrap();

            println!("We loaded data: {:?}", data);

            Ok(())
        }
    }

    // pub fn provide_lidar_config(provide_config_topic: &str) -> Result<mqtt::Message, ()> {}
}
