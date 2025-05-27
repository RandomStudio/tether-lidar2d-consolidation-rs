use clap::{command, Parser};

#[derive(Parser, Debug, Clone)]
#[command(version, about, long_about = None)]
pub struct Cli {
    ///Hostname of MQTT broker (server)
    #[arg(long = "tether.host")]
    pub tether_host: Option<String>,

    /// Optional username for MQTT Broker
    #[arg(long = "tether.username")]
    pub tether_username: Option<String>,

    /// Optional password for MQTT Broker
    #[arg(long = "tether.password")]
    pub tether_password: Option<String>,

    #[arg(long = "loglevel",default_value_t=String::from("info"))]
    pub log_level: String,
}
