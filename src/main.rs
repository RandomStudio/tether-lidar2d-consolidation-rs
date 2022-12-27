 use msgpack_simple::{MsgPack};

 use futures::{executor::block_on, stream::StreamExt};
 use paho_mqtt as mqtt;
 use std::{env, process, time::Duration};
 
 // The topics to which we subscribe.
 const TOPICS: &[&str] = &["+/+/scans"];
 const QOS: &[i32] = &[1, 1];

 use std::collections::HashMap;

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

         let mut scan_samples = HashMap::new();
 
         while let Some(msg_opt) = strm.next().await {
             if let Some(msg) = msg_opt {
                println!("Received message on topic \"{}\":", msg.topic());
                //  println!("Received on topic \"{}\": {}", msg.topic(), msg.payload_str());
                 let payload = msg.payload().to_vec();
                 let decoded = MsgPack::parse(&payload).unwrap();
                //  println!("decoded: {}", decoded);

                let serial = parse_agent_id(msg.topic());
                println!("Device serial is determined as: {}", serial);

                if decoded.is_array() {
                    let scans = decoded.as_array().unwrap();
                    println!("Decoded {} scans", scans.len());

                    let mut samples_this_device: Vec<(f64, f64)> = Vec::new();

                    for sample in scans {
                        let el =sample.as_array().unwrap();
                        // println!("el: {:?}", el);
                        let angle = &el[0].clone().as_float().unwrap();
                        let distance = &el[1].clone().as_float().unwrap();
                        // let angle = sample[0];
                        // let distance = sample[1];
                        // println!("angle = {}, distance = {}", angle, distance);

                        let scan_sample: (f64, f64) = (*angle, *distance);
                        samples_this_device.push(scan_sample);
                    }
                    scan_samples.insert(String::from(serial), samples_this_device);

                    println!("Updated scan samples hashmap: {:?}", scan_samples);
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