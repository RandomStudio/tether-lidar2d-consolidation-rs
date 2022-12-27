# Tether Lidar2D Consolidator, in Rust

## Notes on Libraries

### MQTT Client
Initially we tried using [mqtt-rs](https://github.com/zonyitoo/mqtt-rs), as it seems relatively simple to use, but in the future [mqttrs](https://github.com/00imvj00/mqttrs) might be "better".

For now have settled on paho-mqtt since it seems well-supported and provides examples in realistic scenarios (especially async).

### MessagePack encoding/decoding
The "proper" high-performance solution should be something like [msgpack-rust](https://github.com/3Hren/msgpack-rust) but it is not clear how to construct maps (key/values) without further examples.

For now, we are using [msgpack-simple](https://crates.io/crates/msgpack_simple) which warns that it is "not as performant as static solutions" but seems much easier to use as a starting point.