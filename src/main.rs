 use msgpack_simple::{MsgPack};
 use kddbscan::{cluster, IntoPoint, ClusterId};

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
    y: f64
  }

  impl IntoPoint for Point2D {
    fn get_distance(&self, neighbour: &Point2D) -> f64 {
        ((self.x - neighbour.x).powi(2) + (self.y - neighbour.y).powi(2)).powf(0.5)
    }
  }

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
     let mut cli = mqtt::AsyncClient::new(create_opts).unwrap_or_else(|e| {
         println!("Error creating the client: {:?}", e);
         process::exit(1);
     });
 
     if let Err(err) = block_on(async {
         // Get message stream before connecting.
         let mut strm = cli.get_stream(25);
 
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
         cli.connect(conn_opts).await?;
 
         println!("Subscribing to topics: {:?}", TOPICS);
         cli.subscribe_many(TOPICS, QOS).await?;
 
         // Just loop on incoming messages.
         println!("Waiting for messages...");
 
         // Note that we're not providing a way to cleanly shut down and
         // disconnect. Therefore, when you kill this app (with a ^C or
         // whatever) the server will get an unexpected drop and then
         // should emit the LWT message.

         let mut scan_points = HashMap::new();
         let mut clusters: Vec<Point2D> = vec![];
 
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
                        let el =sample.as_array().unwrap();
                        let angle = &el[0].clone().as_float().unwrap();
                        let distance = &el[1].clone().as_float().unwrap();

                        let point = measurement_to_point(angle, distance);
                        points_this_scan.push(point);
                    }

                    scan_points.insert(String::from(serial), points_this_scan);

                    // println!("Updated scan samples hashmap: {:?}", scan_points);

                    let combined_points = combine_all_points(&scan_points);

                    println!("Combined {} points from all devices", combined_points.len());

                    let clusters = cluster(combined_points, 2, None, None);
                    println!("Found {} clusters", clusters.len());

                    // for c in clusters {
                    //     println!("cluster #{:?}: {:?}", c.get_cluster_id(), c.into_inner());
                    // }
 
                }
             }
             else {
                 // A "None" means we were disconnected. Try to reconnect...
                 println!("Lost connection. Attempting reconnect.");
                 while let Err(err) = cli.reconnect().await {
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

 fn parse_agent_id(topic: &str) -> &str{
    let parts: Vec<&str> = topic.split('/').collect();
    parts[1]
 }

 fn measurement_to_point(angle: &f64, distance: &f64) -> Point2D {
    Point2D {
        x: angle.to_radians().cos() * distance,
        y: angle.to_radians().sin() * distance
    }
 }

 fn combine_all_points(device_points: &HashMap<String, Vec<Point2D>>) -> Vec<Point2D> {
    let mut all_points: Vec<Point2D> = vec![];
    for (device, points) in device_points {
        for p in points {
            all_points.push(Point2D{ x: p.x, y: p.y});
        }
    }
    all_points
 }