use automasking::AutoMaskMessage;
use config::Config;

use env_logger::Env;
use futures::{executor::block_on, stream::StreamExt};
use log::{debug, error, info, warn};
use paho_mqtt as mqtt;
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr};
use std::{process, time::Duration};

mod automasking;
mod clustering;
mod config;
mod tether_utils;
mod tracking;

// The topics to which we subscribe (Input Plugs)
const SCANS_TOPIC: &str = "+/+/scans";
const SAVE_CONFIG_TOPIC: &str = "+/+/saveLidarConfig";
const REQUEST_CONFIG_TOPIC: &str = "+/+/requestLidarConfig";
const REQUEST_AUTOMASK_TOPIC: &str = "+/+/requestAutoMask";
const TOPICS: &[&str] = &[
    SCANS_TOPIC,
    SAVE_CONFIG_TOPIC,
    REQUEST_CONFIG_TOPIC,
    REQUEST_AUTOMASK_TOPIC,
];

// Corresponding QOS level for each of the above
const QOS: &[i32; TOPICS.len()] = &[0, 2, 2, 2];

use crate::automasking::AutoMaskSampler;
use crate::clustering::ClusteringSystem;
use crate::tether_utils::{build_topic, parse_agent_id, parse_plug_name};
use crate::tracking::PerspectiveTransformer;
use clap::Parser;

pub type Point2D = (f64, f64);

// Some defaults; some of which can be overriden via CLI args
const CONFIG_FILE_PATH: &str = "./dummyConfig.json";
const TETHER_HOST: std::net::IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
const AGENT_TYPE: &str = "lidarConsolidation";
const AGENT_ID: &str = "rsTest";
const CLUSTERS_PLUG_NAME: &str = "clusters";
const TRACKING_PLUG_NAME: &str = "trackedPoints";

const MIN_DISTANCE_THRESHOLD: f64 = 20.;
const NEIGHBOURHOOD_RADIUS: f64 = 300.;
const MIN_NEIGHBOURS: usize = 2;
const MAX_CLUSTER_SIZE: f64 = 2500.;

const IGNORE_OUTSIDE_MARGIN: f64 = 0.04;

const AUTOMASK_SCANS_REQUIRED: usize = 45;
const AUTOMASK_MIN_THRESHOLD_MARGIN: f64 = 50.;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]

struct Cli {
    /// Where to load LIDAR device config
    #[arg(long="lidarConfigPath",default_value_t=String::from(CONFIG_FILE_PATH))]
    config_path: String,

    #[arg(long="agentType",default_value_t=String::from(AGENT_TYPE))]
    agent_type: String,

    /// The IP address of the MQTT broker (server)
    #[arg(long = "tether.host", default_value_t=TETHER_HOST)]
    tether_host: std::net::IpAddr,

    #[arg(long = "loglevel",default_value_t=String::from("info"))]
    log_level: String,

    /// Default min distance threshold (in mm) to use for unconfigured new devices
    #[arg(long = "defaultMinDistanceThreshold", default_value_t = MIN_DISTANCE_THRESHOLD)]
    default_min_distance_threshold: f64,

    /// Max distance in mm to a point which can be included in a cluster
    #[arg(long = "clustering.neighbourhoodRadius", default_value_t = NEIGHBOURHOOD_RADIUS)]
    clustering_neighbourhood_radius: f64,

    /// Min points count that constitutes a valid cluster
    #[arg(long = "clustering.minNeighbours", default_value_t = MIN_NEIGHBOURS)]
    clustering_min_neighbours: usize,

    /// Exclude clusters above this size, in radius
    #[arg(long = "clustering.maxClusterSize", default_value_t = MAX_CLUSTER_SIZE)]
    clustering_max_cluster_size: f64,

    /// By default, we drop tracking points (resolved clusters) that lie outside of the defined quad;
    /// enable (use) this flag to include them
    #[arg(long = "perspectiveTransform.includeOutside")]
    transform_include_outside: bool,

    /// Unless perspectiveTransform.includeOutside is enabled, drop tracking points outside range [0-margin,1+margin]
    #[arg(long = "perspectiveTransform.ignoreOutsideMargin", default_value_t=IGNORE_OUTSIDE_MARGIN)]
    transform_ignore_outside_margin: f64,

    #[arg(long = "autoMask.numScansRequired", default_value_t = AUTOMASK_SCANS_REQUIRED)]
    automask_scans_required: usize,

    #[arg(long = "autoMask.minThresholdMargin", default_value_t = AUTOMASK_MIN_THRESHOLD_MARGIN)]
    automask_threshold_margin: f64,
}

/////////////////////////////////////////////////////////////////////////////

fn main() {
    let cli = Cli::parse();

    // Initialize the logger from the environment
    env_logger::Builder::from_env(Env::default().default_filter_or(&cli.log_level)).init();
    debug!("Started; args: {:?}", cli);

    let broker_uri = format!("tcp://{}:1883", cli.tether_host);

    info!("Connecting to Tether @ {} ...", broker_uri);
    let create_opts = mqtt::CreateOptionsBuilder::new()
        .server_uri(broker_uri)
        .client_id("")
        .finalize();

    // Create the client connection
    let mut client = mqtt::AsyncClient::new(create_opts).unwrap_or_else(|e| {
        error!("Error creating the client: {:?}", e);
        process::exit(1);
    });

    if let Err(err) = block_on(async {
        // Get message stream before connecting.
        let mut strm = client.get_stream(25);

        let conn_opts = mqtt::ConnectOptionsBuilder::new()
            .user_name("tether")
            .password("sp_ceB0ss!")
            .keep_alive_interval(Duration::from_secs(30))
            .mqtt_version(mqtt::MQTT_VERSION_3_1_1)
            .clean_session(true)
            .finalize();

        // Make the connection to the broker
        debug!("Connecting to the MQTT server...");
        client.connect(conn_opts).await?;

        // Initialise config, now that we have the MQTT client ready
        let mut config = Config::new(
            &build_topic(&cli.agent_type, AGENT_ID, "provideLidarConfig"),
            &cli.config_path,
        );
        match config.load_config_from_file() {
            Ok(count) => {
                info!("Loaded {} devices OK into Config", count);
                let message = config.publish_config(false);
                client.publish(message.unwrap()).await.unwrap();
            }
            Err(()) => {
                panic!("Error loading devices into config manager!")
            }
        }

        info!("Subscribing to topics: {:?}", TOPICS);
        client.subscribe_many(TOPICS, QOS).await?;

        let mut clustering_system = ClusteringSystem::new(
            cli.clustering_neighbourhood_radius,
            cli.clustering_min_neighbours,
            &build_topic(&cli.agent_type, AGENT_ID, CLUSTERS_PLUG_NAME),
            cli.clustering_max_cluster_size,
        );

        debug!("Clustering system init OK");

        let mut perspective_transformer = PerspectiveTransformer::new(
            &build_topic(&AGENT_TYPE, AGENT_ID, TRACKING_PLUG_NAME),
            match config.region_of_interest() {
                Some(region_of_interest) => {
                    let (c1, c2, c3, c4) = region_of_interest;
                    let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                    Some(corners)
                }
                None => None,
            },
            {
                if cli.transform_include_outside {
                    None
                } else {
                    Some(cli.transform_ignore_outside_margin)
                }
            },
        );

        debug!("Perspective transformer system init OK");

        let mut automask_samplers: HashMap<String, AutoMaskSampler> = HashMap::new();

        // Just loop on incoming messages.
        debug!("Waiting for messages...");

        while let Some(msg_opt) = strm.next().await {
            match msg_opt {
                Some(incoming_message) => match parse_plug_name(incoming_message.topic()) {
                    "scans" => {
                        handle_scans_message(
                            &incoming_message,
                            &mut config,
                            &client,
                            &mut clustering_system,
                            &perspective_transformer,
                            &mut automask_samplers,
                            cli.default_min_distance_threshold,
                        )
                        .await;
                    }
                    "saveLidarConfig" => {
                        debug!("Save Config topic");
                        handle_save_message(
                            &incoming_message,
                            &mut config,
                            &client,
                            &mut perspective_transformer,
                        )
                        .await;
                    }
                    "requestLidarConfig" => {
                        info!("requestLidarConfig; respond with provideLidarConfig message");
                        let message = config.publish_config(true);
                        client.publish(message.unwrap()).await.unwrap();
                    }
                    "requestAutoMask" => {
                        info!("requestAutoMask message");
                        handle_automask_message(
                            &incoming_message,
                            &mut automask_samplers,
                            &mut config,
                            &client,
                            cli.automask_scans_required,
                            cli.automask_threshold_margin,
                        )
                        .await;
                    }
                    _ => {
                        warn!("Unknown topic: {}", incoming_message.topic());
                    }
                },
                None => {
                    // A "None" means we were disconnected. Try to reconnect...
                    warn!("Lost connection. Attempting reconnect.");
                    while let Err(err) = client.reconnect().await {
                        error!("Error reconnecting: {}", err);
                        async_std::task::sleep(Duration::from_millis(1000)).await;
                    }
                }
            }
        }
        // Explicit return type for the async block
        Ok::<(), mqtt::Error>(())
    }) {
        error!("{}", err);
    }
}

async fn handle_scans_message(
    incoming_message: &mqtt::Message,
    config: &mut Config,
    client: &mqtt::AsyncClient,
    clustering_system: &mut ClusteringSystem,
    perspective_transformer: &PerspectiveTransformer,
    automask_samplers: &mut HashMap<String, AutoMaskSampler>,
    default_min_distance: f64,
) {
    let serial = parse_agent_id(incoming_message.topic());
    if let Some(()) = config.check_or_create_device(serial, default_min_distance) {
        let message = config.publish_config(true);
        client.publish(message.unwrap()).await.unwrap();
    }
    let device = config.get_device(serial).unwrap();
    if let Ok((clusters, clusters_message)) = clustering_system
        .handle_scan_message(incoming_message, device)
        .await
    {
        client.publish(clusters_message).await.unwrap();

        if perspective_transformer.is_ready() {
            let points: Vec<Point2D> = clusters
                .into_iter()
                .map(|c| perspective_transformer.transform(&(c.x, c.y)).unwrap())
                .collect();

            if let Ok((_tracked_points, message)) =
                perspective_transformer.publish_tracked_points(&points)
            {
                client.publish(message).await.unwrap();
            }
        }

        if let Some(sampler) = automask_samplers.get_mut(serial) {
            if !sampler.is_complete() {
                let payload = incoming_message.payload().to_vec();
                let scans: Vec<(f64, f64)> = rmp_serde::from_slice(&payload).unwrap();
                if let Some(new_mask) = sampler.add_samples(&scans) {
                    debug!("Sufficient samples for masking device {}", serial);
                    match config.update_device_masking(new_mask, serial) {
                        Ok(()) => {
                            info!("Updated masking for device {}", serial);
                            let message = config.publish_config(true);
                            client.publish(message.unwrap()).await.unwrap();
                            sampler.angles_with_thresholds.clear();
                        }
                        Err(()) => {
                            error!("Error updating masking for device {}", serial);
                        }
                    }
                }
            }
        }
    }
}

async fn handle_save_message(
    incoming_message: &mqtt::Message,
    config: &mut Config,
    client: &mqtt::AsyncClient,
    perspective_transformer: &mut PerspectiveTransformer,
) {
    match config.parse_remote_config(incoming_message) {
        Ok(()) => {
            info!("Remote-provided config parsed OK; now save to disk and (re) publish");
            let message = config.publish_config(true);
            client.publish(message.unwrap()).await.unwrap();

            if let Some(region_of_interest) = config.region_of_interest() {
                let (c1, c2, c3, c4) = region_of_interest;
                let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                perspective_transformer.set_new_quad(&corners)
            }
        }
        Err(()) => error!("There was an error saving the remote-provided config"),
    }
}

async fn handle_automask_message(
    incoming_message: &mqtt::Message,
    automask_samplers: &mut HashMap<String, AutoMaskSampler>,
    config: &mut Config,
    client: &mqtt::AsyncClient,
    scans_required: usize,
    threshold_margin: f64,
) {
    let payload = incoming_message.payload().to_vec();

    // let scans: Vec<(f64, f64)> = rmp_serde::from_slice(&payload).unwrap();
    let automask_command: Result<AutoMaskMessage, rmp_serde::decode::Error> =
        rmp_serde::from_slice(&payload);
    match automask_command {
        Ok(parsed_message) => {
            let command_type: &str = &parsed_message.r#type;
            let result: Result<(), ()> = match command_type {
                "new" => {
                    info!("request NEW auto mask samplers");
                    automask_samplers.clear();
                    config.clear_device_masking();
                    for device in config.devices().iter() {
                        automask_samplers.insert(
                            String::from(&device.serial),
                            AutoMaskSampler::new(scans_required, threshold_margin),
                        );
                    }
                    Ok(())
                }
                "clear" => {
                    info!("request CLEAR all device masking thresholds");
                    automask_samplers.clear();
                    config.clear_device_masking();
                    Ok(())
                }
                _ => {
                    error!("Unrecognised command type for RequestAutoMask message");
                    Err(())
                }
            };

            match result {
                Ok(()) => {
                    let message = config.publish_config(true);
                    client.publish(message.unwrap()).await.unwrap();
                }
                Err(()) => {
                    error!("Error publishing updated config");
                }
            }
        }
        Err(e) => {
            error!("Failed to parse auto mask command: {}", e)
        }
    }
}
