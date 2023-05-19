# Tether Lidar2D Consolidator, in Rust

This is more or a less a direct port of the [original NodeJS Agent](https://github.com/RandomStudio/tether-lidar2d-consolidation) (here referred to as "the OG Agent" ✌️) into Rust 🦀.

All the essential features of the OG Agent have been implemented, but the structure of the application does not attempt to replicate the original. 

The two main goals were
- Learn to use Rust in a real-world application, following an existing codebase which was well known and where the fundamental engineering challenges had already been "solved"
- Produce a usable version of the Lidar Consolidator Agent that was faster and lighter (e.g. on RAM) which could be deployed to lower-resource platforms (e.g. Raspberry Pi) without putting as much strain on the system
## Dev dependencies
Paho Eclipse MQTT has some (non-Rust) dependencies of its own. On Mac, you might need to the following:

```
brew install openssh cmake
```

And on Linux:
```
sudo apt install libssl-dev build-essential cmake
```

## Basic usage
If using cargo, you can simply use
```
cargo run
```

Or, for example running in release (recommended) and with some command-line arguments:
```
cargo run --release -- --loglevel debug
```

Alternatively, run the compiled binary (assuming you've already run `cargo build --release`), typically located in `./target/release/tether-lidar2d-consolidation-rs`

## Command-line configuration
You can see a full list of available command-line arguments by appending `--help` onto your executing command, e.g. `cargo run -- --help`

Options are deliberately made to be almost identical to those used in the OG Agent, with a few notable exceptions:

- Here we use a flag `--perspectiveTransform.includeOutside` instead of setting a boolean `ignoreOutside`. This reflects the default usage better and makes more sense when using a flag (which defaults to `false` if not explicitly appended)
- We currently only provide `tether.host` as an override - no other MQTT Client Options (for now)
- There are no options relating to `autoBroadcastConfig` as there is no need for this behaviour; we save and (re)publish configuration essentially every time it changes, on startup and when requested

## Notes on Libraries

### MQTT Client
Initially we tried using [mqtt-rs](https://github.com/zonyitoo/mqtt-rs), as it seems relatively simple to use, but in the future [mqttrs](https://github.com/00imvj00/mqttrs) might be "better".

For now have settled on [paho-mqtt](https://crates.io/crates/paho-mqtt) since it seems well-supported and provides examples in realistic scenarios (especially async).

### MessagePack encoding/decoding
[rmp_serde](https://docs.rs/rmp-serde/latest/rmp_serde/) is useful for both JSON and MsgPack serialisation/deserialisation. We might not be taking full advantage of zero-copy operations everywhere, but this will take a little more time to figure out.

In the beginning we tried [msgpack-simple](https://crates.io/crates/msgpack_simple) which warns that it is "not as performant as static solutions" but was much easier to use as a starting point.


### Clustering
We tried the library [kddbscan](https://crates.io/crates/kddbscan), but although this may well be more "accurate" it seems to run far too slowly. In any case, this is a very different algorithm from the DBSCAN used in the OG Agent.

We then settled for the more humble (but apparently much more performant) [petal-clustering](https://crates.io/crates/petal-clustering). This in turn requires something called [ndarray](https://docs.rs/crate/ndarray/0.15.6) which seems very similar (and likely based on) [numpy](https://numpy.org/) for Python. 

For now, we use the DBSCAN method as per the OG Agent, but in future it might be worth tested the other supported mode in this library, [HDbscan](https://docs.rs/petal-clustering/0.5.1/petal_clustering/struct.HDbscan.html) which may be faster still (see [the paper](https://dl.acm.org/doi/abs/10.1145/3448016.3457296)).

Another possibility might be the library [linfa-clustering](https://crates.io/crates/linfa-clustering).

### JSON serialisation / deserialisation
We are using a combination of the libraries [serde](https://serde.rs/) and [serde_json](https://docs.rs/serde_json/latest/serde_json/#) which makes it easy to handle JSON in various ways - including strongly typed corresponding to Rust types/structs, which is what we need here in the case of our Config loading/saving.

### Perspective transformation
We are attempting to do a "quad to quad projection" from the ROI to a normalised "square" output quad, similar to [perspective-transform](https://www.npmjs.com/package/perspective-transform) as per the OG Agent.

So far
- https://www.physicsforums.com/threads/transform-that-maps-points-from-any-quad-to-an-reactangle.833996/
- https://docs.rs/projective/0.3.0/projective/trait.Projective.html provides the necessary API - I think what is needed is the 3x3 (or is it 4x4?) matrix to apply to any given point. Could be worked out by replicating https://github.com/jlouthan/perspective-transform/blob/master/dist/perspective-transform.js ?
- https://math.stackexchange.com/questions/296794/finding-the-transform-matrix-from-4-projected-points-with-javascript/339033#339033
- https://stackoverflow.com/questions/14244032/redraw-image-from-3d-perspective-to-2d/14244616#14244616
- https://blog.mbedded.ninja/mathematics/geometry/projective-transformations/
- https://en.wikipedia.org/wiki/Homography#Mathematical_definition
- https://docs.rs/cgmath/0.18.0/cgmath/struct.Perspective.html
- https://franklinta.com/2014/09/08/computing-css-matrix3d-transforms/
- https://yasenh.github.io/post/homogeneous-coordinates/

Finally, used a combination of `ndarray` (which was already installed, to support the clustering calculations) and `nalgebra`.

### Logging
We are using [log](https://crates.io/crates/log) and [env-logger](https://crates.io/crates/env_logger). Log level has been set to INFO by default, but can be overridden, for example by prefixing with an environment variable, e.g. 
```
RUST_LOG=debug cargo run
```

### Command-line configuration
We are using [clap](https://crates.io/crates/clap) which does command-line argument parsing only (no use of files, environment variables, etc.)

Something like [more-config](https://crates.io/crates/more-config) could be useful, since it includes functionality similar to the [rc](https://www.npmjs.com/package/rc) package for NodeJS. 
## Notes on implementation
- As with OG Agent, the serial string for the LIDAR device is extracted from the topic, specifically the `agentIdOrGroup` part in `lidar2d/{agentIdOrGroup}/scans`
- The samples are copied from the array of arrays; each "sample" is an array with the elements `[angle, distance]` and sometimes `[angle, distance, quality]` (although the latter is not handled yet). The samples are converted into points and the list(vector) of all converted points are "inserted" (i.e. replaced, if key already exists) into the hashmap which represents all LIDAR devices (with serial strings as keys). 
  - There is possibly some allocation / copying of data going on; this needs to be reduced as far as is practical
  - There are some of instances of `.unwrap()` in this process; errors should be handled more carefully in some cases

## TODO
- [x] Add (optional) tether-tracking-smooth functionality, built-in
- [ ] Smoothing should also remove duplicate existing points that should be merged (do not leave "stale" points behind)
- [ ] Ensure that this compiles on Raspberry Pi, then cross-compiles (e.g. from Mac)
- [ ] Retain messages, for config publish - remove the need for "request config" topic
- [x] Debug loglevel should suppress MQTT log messages (too verbose)
- [ ] Allow all critical settings (clustering, smoothing) to be updated live on command
- [ ] Print warning/error (and optionally panic?) if no data received from known devices after timeout
- [x] Maintain "state" (config data) and publish this on startup this should allow the visualiser to start showing scan data
- [x] Use rmp-serde for MessagePack instead of msgpack-simple
- [x] Read/write config data from/to disk
- [x] Transform incoming scan points: rotation and position/offset
- [x] Apply Scan Mask Thresholds on incoming samples
- [x] Apply maxClusterSize filtering
- [x] Handle ROI, transformation (warping)
- [x] Allow incoming config (devices, ROI) to be saved via "saveLidarConfig"
- [x] Allow AutoMaskSampler to be created on request
- [x] Should test with multiple sensors
- [x] Tracking output should apply ignoreOutside, ignoreOutsideMargin logic
- [x] If receiving an unknown / unconfigured device, add it with some defaults to Config
- [x] Load/override some settings (e.g. Tether, clustering) from command-line args / defaults
- [x] Generate agent UUID automatically
- [ ] Use "real" Tether Agent crate - although currently this is not set up for async
- [x] Move potentially blocking processes (e.g. Config write to disk) into a separate thread or non-blocking functions
- [ ] Close the client properly on quit, so that the queue is also properly destroyed
- [x] Currently, if "scan samples" are tuples of the form (f64,f64) i.e. (angle,distance), then the system will panic if quality is included. This implies we either need an array without fixed length, or simply drop the quality "field" altogether
- [ ] Possible memory leak when clearing and/or setting new AutoMask Samplers

## Useful resources
- General recipes, including some trigonometry: https://rust-lang-nursery.github.io/rust-cookbook/about.html
- MessagePack spec (includes details about supported types): https://github.com/msgpack/msgpack/blob/master/spec.md
- Rust by Example, including custom types (structs): https://doc.rust-lang.org/rust-by-example/custom_types/structs.html
- Rust Programming Language "Book", including useful info about hash maps: https://doc.rust-lang.org/book/ch08-03-hash-maps.html
- Useful tips for debugging via VSCode + LLDB