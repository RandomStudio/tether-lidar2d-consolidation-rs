use crate::{tether_utils::parse_agent_id, Cluster2D, Point2D};
use msgpack_simple::{MapElement, MsgPack};
use ndarray::{Array, ArrayView};
use paho_mqtt as mqtt;
use petal_clustering::{Dbscan, Fit};
use petal_neighbors::distance::Euclidean;
use std::collections::HashMap;

pub fn combine_all_points(device_points: &HashMap<String, Vec<Point2D>>) -> ndarray::Array2<f64> {
    let mut all_points = Array::zeros((0, 2));
    for (_device, points) in device_points {
        for (x, y) in points {
            all_points.push_row(ArrayView::from(&[*x, *y])).unwrap()
        }
    }
    all_points
}

struct Bounds2D {
    x_min: Option<f64>,
    y_min: Option<f64>,
    x_max: Option<f64>,
    y_max: Option<f64>,
}
/**
Consolidate points in a cluster to a single "Cluster2D" (same as Point2D, but including size)
*/
pub fn consolidate_cluster_points(points: Vec<Point2D>, id: u64) -> Cluster2D {
    let bounds = points.iter().fold(
        Bounds2D {
            x_min: None,
            y_min: None,
            x_max: None,
            y_max: None,
        },
        |acc, point| {
            let (x, y) = point;
            Bounds2D {
                x_min: match acc.x_min {
                    None => Some(*x),
                    Some(v) => Some(v.min(*x)),
                },
                y_min: match acc.y_min {
                    None => Some(*y),
                    Some(v) => Some(v.min(*y)),
                },
                x_max: match acc.x_max {
                    None => Some(*x),
                    Some(v) => Some(v.max(*x)),
                },
                y_max: match acc.y_max {
                    None => Some(*y),
                    Some(v) => Some(v.max(*y)),
                },
            }
        },
    );
    let width = bounds.x_max.unwrap() - bounds.x_min.unwrap();
    let height = bounds.y_max.unwrap() - bounds.y_min.unwrap();
    Cluster2D {
        id,
        position: (
            bounds.x_min.unwrap() + 0.5 * width,
            bounds.y_min.unwrap() + 0.5 * height,
        ),
        size: { width.max(height) },
    }
}

pub async fn handle_scan_message(
    msg: &mqtt::Message,
    scan_points: &mut HashMap<String, Vec<Point2D>>,
    clustering: &mut Dbscan<f64, Euclidean>,
    cluster_output_topic: &str,
) -> Result<mqtt::Message, ()> {
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
                let (x, y) = point;
                if x < 0.0 && y < 0.0 {
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

        // Shadowed "clusters" - now as Cluster2D ("points")
        let clusters: Vec<Cluster2D> = clusters
            .iter()
            .map(|c| {
                let (cluster_index, point_indexes) = c;
                let matched_points = point_indexes
                    .iter()
                    .map(|i| {
                        let point = combined_points.row(*i);
                        (point[0], point[1])
                        // Point2D {
                        //     x: point[0],
                        //     y: point[1],
                        // }
                    })
                    .collect();

                let id = u64::try_from(*cluster_index).unwrap();
                consolidate_cluster_points(matched_points, id)
            })
            .collect();

        // Shadowed "clusters" - now as MsgPack Elements
        let clusters: Vec<MsgPack> = clusters
            .iter()
            .map(|c| {
                MsgPack::Map(vec![
                    MapElement {
                        key: MsgPack::String("id".to_string()),
                        value: MsgPack::Uint(c.id),
                    },
                    MapElement {
                        key: MsgPack::String("x".to_string()),
                        value: MsgPack::Float(c.position.0),
                    },
                    MapElement {
                        key: MsgPack::String("y".to_string()),
                        value: MsgPack::Float(c.position.1),
                    },
                    MapElement {
                        key: MsgPack::String("size".to_string()),
                        value: MsgPack::Float(c.size),
                    },
                ])
            })
            .collect();

        let payload = MsgPack::Array(clusters);
        let message = mqtt::Message::new(cluster_output_topic, payload.encode(), mqtt::QOS_0);

        Ok(message)
    } else {
        Err(())
    }
}

fn measurement_to_point(angle: &f64, distance: &f64) -> Point2D {
    (
        angle.to_radians().cos() * distance,
        angle.to_radians().sin() * distance,
    )
}
