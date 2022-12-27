# Tether Lidar2D Consolidator, in Rust

## Notes on Libraries

### MQTT Client
Initially we tried using [mqtt-rs](https://github.com/zonyitoo/mqtt-rs), as it seems relatively simple to use, but in the future [mqttrs](https://github.com/00imvj00/mqttrs) might be "better".

For now have settled on paho-mqtt since it seems well-supported and provides examples in realistic scenarios (especially async).

### MessagePack encoding/decoding
The "proper" high-performance solution should be something like [msgpack-rust](https://github.com/3Hren/msgpack-rust) but it is not clear how to construct maps (key/values) without further examples.

For now, we are using [msgpack-simple](https://crates.io/crates/msgpack_simple) which warns that it is "not as performant as static solutions" but seems much easier to use as a starting point.

## Notes on implementation
- As with OG Agent, the serial string for the LIDAR device is extracted from the topic, specifically the `agentIdOrGroup` part in `lidar2d/{agentIdOrGroup}/scans`
- The samples are copied from the array of arrays; each "sample" is an array with the elements `[angle, distance]` and sometimes `[angle, distance, quality]` (although the latter is not handled yet). The samples are copied into a vector and then the final vector is "inserted" (i.e. replaced, if key already exists) into the hashmap which represents all LIDAR devices (with serial strings as keys). 
  - There is a lot of new allocation and copying here; possibly this needs to be reduced as far as is practical
  - There are plenty of instances of `.unwrap()` in this process; errors should be handled more carefully
- It remains to be seen whether the OG Agent logic around the timing of publishing "cluster" and "tracking" data should be retained. The original version seems to imply that every single incoming "scans" message triggers a corresponding set of "cluster" and "tracking" messages, which in turn require all the clustering and transformation calculations to be run each time; this seems excessive. 
  - It might be possible to determine a complete "frame" by checking the number of known devices (by serial) against counts of messages incoming from those devices. But this is likely to get quite complicated, and prone to failure (what if a device delivers a message "late", for example, or drops out altogether for a while.)
  - It might be simpler to emit the tracking/cluster data on a regular interval instead. This means that the calculations are done, at most, once per "frame" before publishing any messages. We could even be smart about keeping track of which scan data has been updated each time. For example, we keep a separate list of calculated/transformed points per device separately from the raw scan data - and only update the latter when new scan data for that device is incoming. We could also skip/cache calculations for clustering if none of the devices have updated scan data since the previous frame.