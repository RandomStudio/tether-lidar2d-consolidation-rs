# Tether Lidar2D Consolidator, in Rust

## Notes on Libraries

### MQTT Client
Initially we tried using [mqtt-rs](https://github.com/zonyitoo/mqtt-rs), as it seems relatively simple to use, but in the future [mqttrs](https://github.com/00imvj00/mqttrs) might be "better".

For now have settled on paho-mqtt since it seems well-supported and provides examples in realistic scenarios (especially async).

### MessagePack encoding/decoding
The "proper" high-performance solution should be something like [msgpack-rust](https://github.com/3Hren/msgpack-rust) but it is not clear how to construct maps (key/values) without further examples.

For now, we are using [msgpack-simple](https://crates.io/crates/msgpack_simple) which warns that it is "not as performant as static solutions" but seems much easier to use as a starting point.

### Clustering
We tried the [kddbscan](https://crates.io/crates/kddbscan), but although this may well be more "accurate" it seems to run far too slowly. In any case, this is a very different algorithm from the DBSCAN used in the OG Agent.

We then settled for the more humble (but apparently much more performant) [petal-clustering](https://crates.io/crates/petal-clustering). This in turn requires something called [ndarray](https://docs.rs/crate/ndarray/0.15.6) which seems very similar (and likely based on) [numpy](https://numpy.org/) for Python. 

For now, we use the DBSCAN method as per the OG Agent, but in future it might be worth tested the other supported mode in this library, [HDbscan](https://docs.rs/petal-clustering/0.5.1/petal_clustering/struct.HDbscan.html) which may be faster still (see [the paper](https://dl.acm.org/doi/abs/10.1145/3448016.3457296)).

Another possibility might be the library [linfa-clustering](https://crates.io/crates/linfa-clustering).

## Notes on implementation
- As with OG Agent, the serial string for the LIDAR device is extracted from the topic, specifically the `agentIdOrGroup` part in `lidar2d/{agentIdOrGroup}/scans`
- The samples are copied from the array of arrays; each "sample" is an array with the elements `[angle, distance]` and sometimes `[angle, distance, quality]` (although the latter is not handled yet). The samples are copied into a vector and then the final vector is "inserted" (i.e. replaced, if key already exists) into the hashmap which represents all LIDAR devices (with serial strings as keys). 
  - There is a lot of new allocation and copying here; possibly this needs to be reduced as far as is practical
  - There are plenty of instances of `.unwrap()` in this process; errors should be handled more carefully
- It remains to be seen whether the OG Agent logic around the timing of publishing "cluster" and "tracking" data should be retained. The original version seems to imply that every single incoming "scans" message triggers a corresponding set of "cluster" and "tracking" messages, which in turn require all the clustering and transformation calculations to be run each time; this seems excessive and likely to increase calculations/message volumes exponentially as the number of devices increases!
  - Calculations should probably be updated on each message, but only for the incoming device
  - Clustering should be recalculated on every incoming message because point data has now been updated. Possibly this message should also be re-emitted on an interval (timeout/debounce).

  ## Useful resources
  General recipes, including some trigonometry: https://rust-lang-nursery.github.io/rust-cookbook/about.html
  MessagePack spec (includes details about supported types): https://github.com/msgpack/msgpack/blob/master/spec.md
  Rust by Example, including custom types (structs): https://doc.rust-lang.org/rust-by-example/custom_types/structs.html
  Rust Programming Language "Book", including useful info about hash maps: https://doc.rust-lang.org/book/ch08-03-hash-maps.html
  