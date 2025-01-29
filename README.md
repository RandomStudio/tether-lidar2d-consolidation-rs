# Tether Lidar2D Consolidator, in Rust

This is a [Tether](https://github.com/RandomStudio/tether) agent which combines scan data from one or more 2D LIDAR sensors and produces smooth tracking output.

Typically, you will use this agent in combination with one or more [tether-rplidar](https://github.com/RandomStudio/tether-rplidar-rs) agents.

## Install and run
Use the instructions in [releases](https://github.com/RandomStudio/tether-rplidar-rs/releases) or `cargo install tether-lidar2d-consolidation`

Then run the following processes (e.g. in separate terminal panes/tabs/windows):

```
tether-rplidar
```
(assumimg you have [tether-rplidar](https://github.com/RandomStudio/tether-rplidar-rs) installed)

...Then:
```
lidar2d-backend
```

...Then, optionally:
```
lidar2d-frontend
```


## Command-line configuration
For both executables, you can see a full list of available command-line arguments by appending `--help` onto your executing command, e.g. `lidar2d-backend --help` (installed) or `cargo run --bin lidar2d-backend -- --help` (development)

## Expected Output
Most important plug from `lidar2d-backend`:
- `smoothedTrackedPoints`: an array of objects with "id", "x", y" for each smoothed point. Only produces output once a region of interest (ROI) has been defined

Other plugs from `lidar2d-backend`:
- `trackedPoints`: an array of 2D vectors arrays with [x,y]) for _transformed_ but not _smoothed_ points within the tracking region (ROI)
- `provideLidarConfig`: a retained-message with the complete backend configuration, typically used by `lidar2d-frontend`
- `clusters`: an array of clusters with size and position, typically used by `lidar2d-frontend` to display clustering on the tracking graph
- `movement`: if "enableAverageMovement" is `true`, then this will output a single 2D vector representing movement averaged from all smoothed tracked points

From `lidar2D-frontend` only:
- `saveLidarConfig`: used whenever a new configuration is saved from the frontend UI

---
## Notes on Libraries

### Clustering
We tried the library [kddbscan](https://crates.io/crates/kddbscan), but although this may well be more "accurate" it seems to run far too slowly. In any case, this is a very different algorithm from the DBSCAN used in the OG Agent.

We then settled for the more humble (but apparently much more performant) [petal-clustering](https://crates.io/crates/petal-clustering). This in turn requires something called [ndarray](https://docs.rs/crate/ndarray/0.15.6) which seems very similar (and likely based on) [numpy](https://numpy.org/) for Python.

For now, we use the DBSCAN method as per the OG Agent, but in future it might be worth tested the other supported mode in this library, [HDbscan](https://docs.rs/petal-clustering/0.5.1/petal_clustering/struct.HDbscan.html) which may be faster still (see [the paper](https://dl.acm.org/doi/abs/10.1145/3448016.3457296)).

Another possibility might be the library [linfa-clustering](https://crates.io/crates/linfa-clustering).

### JSON serialisation / deserialisation
We are using a combination of the libraries [serde](https://serde.rs/) and [serde_json](https://docs.rs/serde_json/latest/serde_json/#) which makes it easy to handle JSON in various ways - including strongly typed corresponding to Rust types/structs, which is what we need here in the case of our Config loading/saving.

### Perspective transformation
We are attempting to do a "quad to quad projection" from the ROI to a normalised "square" output quad, similar to [perspective-transform](https://www.npmjs.com/package/perspective-transform) as per the OG Agent.

A library specifically for this job was spun out into a separate crate: https://github.com/RandomStudio/quad-to-quad-transformer

### Logging
We are using [log](https://crates.io/crates/log) and [env-logger](https://crates.io/crates/env_logger). Log level has been set to INFO by default, but can be overridden, for example by prefixing with an environment variable, e.g.
```
RUST_LOG=debug cargo run
```

### Command-line configuration
We are using [clap](https://crates.io/crates/clap) which does command-line argument parsing only (no use of files, environment variables, etc.)

Something like [more-config](https://crates.io/crates/more-config) could be useful, since it includes functionality similar to the [rc](https://www.npmjs.com/package/rc) package for NodeJS.
