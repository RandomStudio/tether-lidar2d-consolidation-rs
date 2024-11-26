use std::net::{IpAddr, Ipv4Addr};

use clap::{command, Parser};

// Some defaults; some of which can be overriden via CLI args
const CONFIG_FILE_PATH: &str = "./lidar.json";
const TETHER_HOST: std::net::IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Cli {
    /// Where to load LIDAR device config
    #[arg(long="lidarConfigPath",default_value_t=String::from(CONFIG_FILE_PATH))]
    pub config_path: String,

    /// The IP address of the MQTT broker (server)
    #[arg(long = "tether.host", default_value_t=TETHER_HOST)]
    pub tether_host: std::net::IpAddr,

    /// The Agent Role (type)
    #[arg(long="tether.role",default_value_t=String::from("lidarConsolidation"))]
    pub agent_role: String,

    /// The Agent Group (ID)
    #[arg(long="tether.group",default_value_t=String::from("any"))]
    pub agent_group: String,

    /// Optional username for MQTT Broker
    #[arg(long = "tether.username")]
    pub tether_username: Option<String>,

    /// Optional password for MQTT Broker
    #[arg(long = "tether.password")]
    pub tether_password: Option<String>,

    #[arg(long = "loglevel",default_value_t=String::from("info"))]
    pub log_level: String,
}
