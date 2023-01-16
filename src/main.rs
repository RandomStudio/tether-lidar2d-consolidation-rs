use automasking::AutoMaskMessage;
use config::config_state::Config;

use futures::{executor::block_on, stream::StreamExt};
use paho_mqtt as mqtt;
use std::collections::HashMap;
use std::{env, process, time::Duration};

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
use crate::tracking::tracking::PerspectiveTransformer;

pub type Point2D = (f64, f64);

// TODO: some/all of these constants should be
// overrideable via commandline args, etc.
const AGENT_TYPE: &str = "lidarConsolidation";
const AGENT_ID: &str = "rsTest";

const NEIGHBOURHOOD_RADIUS: f64 = 300.;
const MIN_NEIGHBOURS: usize = 2;
const MAX_CLUSTER_SIZE: f64 = 2500.;

/////////////////////////////////////////////////////////////////////////////

fn main() {
    // Initialize the logger from the environment
    env_logger::init();

    let host = env::args()
        .nth(1)
        .unwrap_or_else(|| "tcp://localhost:1883".to_string());

    // Create the client. Use an ID for a persistent session.
    // A real system should try harder to use a unique ID.
    let create_opts = mqtt::CreateOptionsBuilder::new()
        .server_uri(host)
        .client_id("")
        .finalize();

    // Create the client connection
    let mut client = mqtt::AsyncClient::new(create_opts).unwrap_or_else(|e| {
        println!("Error creating the client: {:?}", e);
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
        println!("Connecting to the MQTT server...");
        client.connect(conn_opts).await?;

        // Initialise config, now that we have the MQTT client ready
        let mut config = Config::new(
            &build_topic(AGENT_TYPE, AGENT_ID, "provideLidarConfig"),
            "./dummyConfig.json",
        );
        match config.load_config_from_file() {
            Ok(count) => {
                println!("Loaded {} devices OK into Config", count);
                let message = config.publish_config(false);
                client.publish(message.unwrap()).await.unwrap();
            }
            Err(()) => {
                panic!("Error loading devices into config manager!")
            }
        }

        println!("Subscribing to topics: {:?}", TOPICS);
        client.subscribe_many(TOPICS, QOS).await?;

        let mut clustering_system = ClusteringSystem::new(
            NEIGHBOURHOOD_RADIUS,
            MIN_NEIGHBOURS,
            &build_topic(AGENT_TYPE, AGENT_ID, "clusters"),
            MAX_CLUSTER_SIZE,
        );

        println!("Clustering system init OK");

        let mut perspective_transformer = PerspectiveTransformer::new(
            &build_topic(AGENT_TYPE, AGENT_ID, "trackedPoints"),
            match config.region_of_interest() {
                Some(region_of_interest) => {
                    let (c1, c2, c3, c4) = region_of_interest;
                    let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                    Some(corners)
                }
                None => None,
            },
        );

        println!("Perspective transformer system init OK");

        let mut automask_samplers: HashMap<String, AutoMaskSampler> = HashMap::new();

        // Just loop on incoming messages.
        println!("Waiting for messages...");

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
                        )
                        .await;
                    }
                    "saveLidarConfig" => {
                        println!("Save Config topic");
                        handle_save_message(
                            &incoming_message,
                            &mut config,
                            &client,
                            &mut perspective_transformer,
                        )
                        .await;
                    }
                    "requestLidarConfig" => {
                        println!("requestLidarConfig; respond with provideLidarConfig message");
                        let message = config.publish_config(true);
                        client.publish(message.unwrap()).await.unwrap();
                    }
                    "requestAutoMask" => {
                        println!("requestAutoMask message");
                        handle_automask_message(
                            &incoming_message,
                            &mut automask_samplers,
                            &mut config,
                            &client,
                        )
                        .await;
                    }
                    _ => {
                        println!("Unknown topic: {}", incoming_message.topic());
                    }
                },
                None => {
                    // A "None" means we were disconnected. Try to reconnect...
                    println!("Lost connection. Attempting reconnect.");
                    while let Err(err) = client.reconnect().await {
                        println!("Error reconnecting: {}", err);
                        async_std::task::sleep(Duration::from_millis(1000)).await;
                    }
                }
            }
        }
        // Explicit return type for the async block
        Ok::<(), mqtt::Error>(())
    }) {
        eprintln!("{}", err);
    }
}

async fn handle_scans_message(
    incoming_message: &mqtt::Message,
    config: &mut Config,
    client: &mqtt::AsyncClient,
    clustering_system: &mut ClusteringSystem,
    perspective_transformer: &PerspectiveTransformer,
) {
    let serial = parse_agent_id(incoming_message.topic());
    if let Some(()) = config.check_or_create_device(serial) {
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
            println!("Remote-provided config parsed OK; now save to disk and (re) publish");
            let message = config.publish_config(true);
            client.publish(message.unwrap()).await.unwrap();

            if let Some(region_of_interest) = config.region_of_interest() {
                let (c1, c2, c3, c4) = region_of_interest;
                let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                perspective_transformer.set_new_quad(&corners)
            }
        }
        Err(()) => println!("There was an error saving the remote-provided config"),
    }
}

async fn handle_automask_message(
    incoming_message: &mqtt::Message,
    automask_samplers: &mut HashMap<String, AutoMaskSampler>,
    config: &mut Config,
    client: &mqtt::AsyncClient,
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
                    println!("request NEW auto mask samplers");
                    automask_samplers.clear();
                    config.clear_device_masking();
                    for device in config.devices().iter() {
                        automask_samplers
                            .insert(String::from(&device.serial), AutoMaskSampler::new(45, 50.));
                    }
                    Ok(())
                }
                "clear" => {
                    println!("request CLEAR all device masking thresholds");
                    automask_samplers.clear();
                    config.clear_device_masking();
                    Ok(())
                }
                _ => {
                    println!("Unrecognised command type for RequestAutoMask message");
                    Err(())
                }
            };

            match result {
                Ok(()) => {
                    let message = config.publish_config(true);
                    client.publish(message.unwrap()).await.unwrap();
                }
                Err(()) => {
                    println!("Error publishing updated config");
                }
            }
        }
        Err(e) => {
            println!("Failed to parse auto mask command: {}", e)
        }
    }
}
