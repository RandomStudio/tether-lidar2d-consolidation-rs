use async_std::stream::Map;
use msgpack_simple::{MapElement, MsgPack};
use ndarray::{Array, ArrayView};
use petal_clustering::{Dbscan, Fit};
use petal_neighbors::distance::Euclidean;

use futures::{executor::block_on, stream::StreamExt};
use paho_mqtt as mqtt;
use std::{env, process, time::Duration};

// The topics to which we subscribe.
const TOPICS: &[&str] = &["+/+/scans"];
const QOS: &[i32] = &[1, 1];

use std::collections::HashMap;

#[derive(Debug)]
struct Point2D {
    x: f64,
    y: f64,
}

const AGENT_TYPE: &str = "lidarConsolidation";
const AGENT_ID: &str = "rsTest";

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

        // Define the set of options for the connection
        let lwt = mqtt::Message::new("test", "Async subscriber lost connection", mqtt::QOS_1);

        let conn_opts = mqtt::ConnectOptionsBuilder::new()
            .user_name("tether")
            .password("sp_ceB0ss!")
            .keep_alive_interval(Duration::from_secs(30))
            .mqtt_version(mqtt::MQTT_VERSION_3_1_1)
            .clean_session(false)
            .will_message(lwt)
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

        let mut scan_points = HashMap::new();

        let mut clustering = Dbscan {
            eps: 300.,
            min_samples: 2,
            metric: Euclidean::default(),
        };

        let cluster_output_topic = build_topic(AGENT_TYPE, AGENT_ID, "clusters");

        while let Some(msg_opt) = strm.next().await {
            if let Some(msg) = msg_opt {
                println!("Received message on topic \"{}\":", msg.topic());
                let payload = msg.payload().to_vec();
                let decoded = MsgPack::parse(&payload).unwrap();

                let serial = parse_agent_id(msg.topic());
                println!("Device serial is determined as: {}", serial);

                if decoded.is_array() {
                    let scans = decoded.as_array().unwrap();
                    println!("Decoded {} scans", scans.len());

                    let mut points_this_scan: Vec<Point2D> = Vec::new();

                    for sample in scans {
                        let el = sample.as_array().unwrap();
                        let angle = &el[0].clone().as_float().unwrap();
                        let distance = &el[1].clone().as_float().unwrap();

                        if *distance > 0.0 {
                            let point = measurement_to_point(angle, distance);
                            if point.x < 0.0 && point.y < 0.0 {
                                points_this_scan.push(point);
                            }
                        }
                    }

                    scan_points.insert(String::from(serial), points_this_scan);

                    // println!("Updated scan samples hashmap: {:?}", scan_points);

                    let combined_points = combine_all_points(&scan_points);

                    println!(
                        "Combined {} points from all devices",
                        (combined_points.len() / 2)
                    );

                    let (clusters, outliers) = clustering.fit(&combined_points);

                    println!("Clustering done");
                    println!(
                        "Found {} clusters, {} outliers",
                        clusters.len(),
                        outliers.len()
                    );

                    for c in clusters.iter() {
                        let (cluster_index, point_indexes) = c;
                        println!("cluster #{} = {:?}", cluster_index, point_indexes);

                        let matched_points = point_indexes.iter().map(|i| {
                            let point = combined_points.row(*i);
                            let index = u64::try_from(*i).unwrap();
                            MsgPack::Map(vec![MapElement {
                                key: MsgPack::String("id".to_string()),
                                value: MsgPack::Uint(index),
                            }])
                        });
                        let matched_points: Vec<MsgPack> = matched_points.collect();

                        assert_eq!(matched_points.len(), point_indexes.len());

                        // for el in matched_points {
                        //     //     let p: Point2D = Point2D { x: el[0], y: el[1] };
                        //     println!("this should be point: {:?}", el);
                        // }

                        // let message = MsgPack::Map(vec![MapElement {
                        //     key: MsgPack::Int(0),
                        //     value: MsgPack::Int(42),
                        // }]);
                        let message = MsgPack::Array(matched_points);
                        // let message = MsgPack::Map(matched_points.map(|point|
                        //     MapElement { key: point., value: () }
                        // ).collect());

                        let msg = mqtt::Message::new(
                            &cluster_output_topic,
                            message.encode(),
                            mqtt::QOS_1,
                        );
                        client.publish(msg).await?;
                    }
                }
            } else {
                // A "None" means we were disconnected. Try to reconnect...
                println!("Lost connection. Attempting reconnect.");
                while let Err(err) = client.reconnect().await {
                    println!("Error reconnecting: {}", err);
                    async_std::task::sleep(Duration::from_millis(1000)).await;
                }
            }
        }

        // Explicit return type for the async block
        Ok::<(), mqtt::Error>(())
    }) {
        eprintln!("{}", err);
    }
}

fn parse_agent_id(topic: &str) -> &str {
    let parts: Vec<&str> = topic.split('/').collect();
    parts[1]
}

fn build_topic(agent_type: &str, agent_id: &str, plug_name: &str) -> String {
    format!("{}/{}/{}", agent_type, agent_id, plug_name)
}

fn measurement_to_point(angle: &f64, distance: &f64) -> Point2D {
    Point2D {
        x: angle.to_radians().cos() * distance,
        y: angle.to_radians().sin() * distance,
    }
}

fn combine_all_points(device_points: &HashMap<String, Vec<Point2D>>) -> ndarray::Array2<f64> {
    let mut all_points = Array::zeros((0, 2));
    for (_device, points) in device_points {
        for p in points {
            all_points.push_row(ArrayView::from(&[p.x, p.y])).unwrap()
        }
    }
    all_points
}
