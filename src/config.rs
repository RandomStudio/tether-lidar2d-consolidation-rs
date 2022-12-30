pub mod config_state {

    use msgpack_simple::{MapElement, MsgPack};
    use paho_mqtt as mqtt;
    pub struct LidarDevice {
        pub serial: String,
    }

    pub struct ConfigManager {
        devices: Vec<LidarDevice>,
    }

    impl ConfigManager {
        pub fn new() -> ConfigManager {
            ConfigManager { devices: vec![] }
        }

        pub fn load_config(&self, provide_config_topic: &str) -> Result<mqtt::Message, ()> {
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
    }

    // pub fn provide_lidar_config(provide_config_topic: &str) -> Result<mqtt::Message, ()> {}
}
