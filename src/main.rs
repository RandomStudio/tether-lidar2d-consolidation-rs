
use paho_mqtt as mqtt;
use std::{env, process};

const QOS: i32 = 1;

/////////////////////////////////////////////////////////////////////////////

fn main() {
    // Initialize the logger from the environment
    env_logger::init();

    let host = env::args()
        .nth(1)
        .unwrap_or_else(|| "tcp://localhost:1883".to_string());

    // Create a client & define connect options
    let cli = mqtt::AsyncClient::new(host).unwrap_or_else(|err| {
        println!("Error creating the client: {}", err);
        process::exit(1);
    });



    // let conn_opts = mqtt::ConnectOptions::new();
    let connect_opts= mqtt::ConnectOptionsBuilder::new().user_name("tether").password("sp_ceB0ss!").finalize();

    // Connect and wait for it to complete or fail
    if let Err(e) = cli.connect(connect_opts).wait() {
        println!("Unable to connect: {:?}", e);
        process::exit(1);
    }

    // Create a topic and publish to it
    println!("Publishing messages on the 'test' topic");
    let topic = mqtt::Topic::new(&cli, "test", QOS);
    for _ in 0..5 {
        let tok = topic.publish("Hello there");

        if let Err(e) = tok.wait() {
            println!("Error sending message: {:?}", e);
            break;
        }
    }

    // Disconnect from the broker
    let tok = cli.disconnect(None);
    tok.wait().unwrap();
}