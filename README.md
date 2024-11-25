# Tether Lidar2D Consolidator, in Rust

This was originally a direct port of the [original NodeJS Agent](https://github.com/RandomStudio/tether-lidar2d-consolidation) (here referred to as "the OG Agent" ✌️) into Rust 🦀. Some new features have now been added after v0.3.

The two main goals were
- Learn to use Rust in a real-world application, following an existing codebase which was well known and where the fundamental engineering challenges had already been "solved"
- Produce a usable version of the Lidar Consolidator Agent that was faster and lighter (e.g. on RAM) which could be deployed to lower-resource platforms (e.g. Raspberry Pi) without putting as much strain on the system

## Easy install via Cargo
Since v0.3.1, you can install the binary via Cargo, ie.:
```
cargo install tether-lidar2d-consolidation
```

...and then run:
```
lidar2d-backend
lidar2d-frontend
```

## Command-line configuration
For the backend executable, you can see a full list of available command-line arguments by appending `--help` onto your executing command, e.g. `lidar2d-backend --help` (installed) or `cargo run --bin lidar2d-backend -- --help` (development)

Options are deliberately made to be almost identical to those used in the OG Agent, with a few notable exceptions:

- Here we use a flag `--perspectiveTransform.includeOutside` instead of setting a boolean `ignoreOutside`. This reflects the default usage better and makes more sense when using a flag (which defaults to `false` if not explicitly appended)
- For Tether, you can optionally provide `--tether.host`, `--tether.username`, `--tether.password` if these differ from the defaults
- There are no options relating to `autoBroadcastConfig` as there is no need for this behaviour; we save and (re)publish configuration essentially every time it changes, on startup and when requested

## Dev dependencies
If you are compiling on your own system, Paho Eclipse MQTT has some (non-Rust) dependencies of its own. On Mac, you might need to the following:

```
brew install openssh cmake
```

And on Linux:
```
sudo apt install libssl-dev build-essential cmake
```

## Expected Output
Plugs:
- "trackedPoints": an array of 2D vectors (arrays with [x,y]) for transformed but not-smoothed points within the tracking region (ROI)
- "smoothedTrackedPoints:" an array of objects with "id", "x", y" and "velocity" (2D vector) for each smoothed point

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

## Some differences from OG version
- There is no `requestLlidarConfig` Plug any more; the `retain` feature of MQTT is used to provide a persistent and up-to-date config for all clients
- Smoothing is incorporated into this Agent; there is no need to run a separate `tether-tracking-smooth` agent any more
- Zones for "presence detection" can be set up within this Agent

## Useful resources
- General recipes, including some trigonometry: https://rust-lang-nursery.github.io/rust-cookbook/about.html
- MessagePack spec (includes details about supported types): https://github.com/msgpack/msgpack/blob/master/spec.md
- Rust by Example, including custom types (structs): https://doc.rust-lang.org/rust-by-example/custom_types/structs.html
- Rust Programming Language "Book", including useful info about hash maps: https://doc.rust-lang.org/book/ch08-03-hash-maps.html
- Useful tips for debugging via VSCode + LLDB
