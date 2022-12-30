use config::config_state::{ConfigManager, LidarDevice};
use petal_clustering::Dbscan;
use petal_neighbors::distance::Euclidean;

use futures::{executor::block_on, stream::StreamExt};
use paho_mqtt as mqtt;
use std::{env, process, time::Duration};

mod clustering;
mod config;
mod tether_utils;

// The topics to which we subscribe (Input Plugs)
const SCANS_TOPIC: &str = "+/+/scans";
const TOPICS: &[&str] = &[SCANS_TOPIC];

// Corresponding QOS level for each of the above
const QOS: &[i32] = &[0];

use std::collections::HashMap;

use crate::tether_utils::build_topic;

pub type Point2D = (f64, f64);

#[derive(Debug)]
pub struct Cluster2D {
    id: u64,
    position: Point2D,
    size: f64,
}

const AGENT_TYPE: &str = "lidarConsolidation";
const AGENT_ID: &str = "rsTest";

/////////////////////////////////////////////////////////////////////////////

fn main() {
    // Initialize the logger from the environment
    env_logger::init();

    // Initialise config
    let mut config = ConfigManager::new();
    match config.load_lidar_config(vec![LidarDevice {
        serial: String::from("dummy"),
    }]) {
        Ok(count) => {
            println!("Loaded {} devices OK into Config", count);
        }
        Err(()) => {
            panic!("Error loading devices into config manager!")
        }
    }

    let host = env::args()
        .nth(1)
        .unwrap_or_else(|| "tcp://localhost:1883".to_string());

    // Create the client. Use an ID for a persistent session.
    // A real system should try harder to use a unique ID.
    let create_opts = mqtt::CreateOptionsBuilder::new()
        .server_uri(host)
        .client_id("rust_async_subscribe")
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
            .clean_session(false)
            .finalize();

        // Make the connection to the broker
        println!("Connecting to the MQTT server...");
        client.connect(conn_opts).await?;

        println!("Subscribing to topics: {:?}", TOPICS);
        client.subscribe_many(TOPICS, QOS).await?;

        // Just loop on incoming messages.
        println!("Waiting for messages...");

        // Note that we're not providing a way to cleanly shut down and
        // disconnect. Therefore, when you kill this app (with a ^C or
        // whatever) the server will get an unexpected drop and then
        // should emit the LWT message.

        let mut scan_points: HashMap<String, Vec<Point2D>> = HashMap::new();

        let mut clustering = Dbscan {
            eps: 300.,
            min_samples: 2,
            metric: Euclidean::default(),
        };

        let cluster_output_topic = build_topic(AGENT_TYPE, AGENT_ID, "clusters");

        while let Some(msg_opt) = strm.next().await {
            match msg_opt {
                Some(message) => {
                    // TODO: check which topic we received on, so that messages are passed to correct handlers
                    match clustering::handle_scan_message(
                        &message,
                        &mut scan_points,
                        &mut clustering,
                        &cluster_output_topic,
                    )
                    .await
                    {
                        Ok(message) => {
                            client.publish(message).await.unwrap();
                        }
                        Err(()) => {
                            println!("Something went wrong building the clusters message to send");
                        }
                    }
                }
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
