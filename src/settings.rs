use std::net::{IpAddr, Ipv4Addr};

use clap::{command, Parser};

// Some defaults; some of which can be overriden via CLI args
const CONFIG_FILE_PATH: &str = "./dummyConfig.json";
const TETHER_HOST: std::net::IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));

const MIN_DISTANCE_THRESHOLD: f64 = 20.;
const NEIGHBOURHOOD_RADIUS: f64 = 200.;
const MIN_NEIGHBOURS: usize = 3;
const MAX_CLUSTER_SIZE: f64 = 2500.;

const IGNORE_OUTSIDE_MARGIN: f64 = 0.04;

const AUTOMASK_SCANS_REQUIRED: usize = 60;
const AUTOMASK_MIN_THRESHOLD_MARGIN: f64 = 50.;

const AGENT_TYPE: &str = "lidarConsolidation";

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Cli {
    /// Where to load LIDAR device config
    #[arg(long="lidarConfigPath",default_value_t=String::from(CONFIG_FILE_PATH))]
    pub config_path: String,

    #[arg(long="agentType",default_value_t=String::from(AGENT_TYPE))]
    pub agent_type: String,

    /// The IP address of the MQTT broker (server)
    #[arg(long = "tether.host", default_value_t=TETHER_HOST)]
    pub tether_host: std::net::IpAddr,

    #[arg(long = "loglevel",default_value_t=String::from("info"))]
    pub log_level: String,

    /// Default min distance threshold (in mm) to use for unconfigured new devices
    #[arg(long = "defaultMinDistanceThreshold", default_value_t = MIN_DISTANCE_THRESHOLD)]
    pub default_min_distance_threshold: f64,

    /// Max distance in mm to a point which can be included in a cluster
    #[arg(long = "clustering.neighbourhoodRadius", default_value_t = NEIGHBOURHOOD_RADIUS)]
    pub clustering_neighbourhood_radius: f64,

    /// Min points count that constitutes a valid cluster
    #[arg(long = "clustering.minNeighbours", default_value_t = MIN_NEIGHBOURS)]
    pub clustering_min_neighbours: usize,

    /// Exclude clusters above this size, in radius
    #[arg(long = "clustering.maxClusterSize", default_value_t = MAX_CLUSTER_SIZE)]
    pub clustering_max_cluster_size: f64,

    /// By default, we drop tracking points (resolved clusters) that lie outside of the defined quad;
    /// enable (use) this flag to include them
    #[arg(long = "perspectiveTransform.includeOutside")]
    pub transform_include_outside: bool,

    /// Unless perspectiveTransform.includeOutside is enabled, drop tracking points outside range [0-margin,1+margin]
    #[arg(long = "perspectiveTransform.ignoreOutsideMargin", default_value_t=IGNORE_OUTSIDE_MARGIN)]
    pub transform_ignore_outside_margin: f64,

    #[arg(long = "autoMask.numScansRequired", default_value_t = AUTOMASK_SCANS_REQUIRED)]
    pub automask_scans_required: usize,

    #[arg(long = "autoMask.minThresholdMargin", default_value_t = AUTOMASK_MIN_THRESHOLD_MARGIN)]
    pub automask_threshold_margin: f64,
}
