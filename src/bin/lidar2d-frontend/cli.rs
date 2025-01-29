use std::net::{IpAddr, Ipv4Addr};

use clap::{command, Parser};

const TETHER_HOST: std::net::IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Cli {
    /// The IP address of the MQTT broker (server)
    #[arg(long = "tether.host", default_value_t=TETHER_HOST)]
    pub tether_host: std::net::IpAddr,

    /// Optional username for MQTT Broker
    #[arg(long = "tether.username")]
    pub tether_username: Option<String>,

    /// Optional password for MQTT Broker
    #[arg(long = "tether.password")]
    pub tether_password: Option<String>,

    #[arg(long = "loglevel",default_value_t=String::from("info"))]
    pub log_level: String,
}
