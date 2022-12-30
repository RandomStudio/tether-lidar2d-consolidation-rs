use crate::{Cluster2D, Point2D};
use ndarray::{Array, ArrayView};
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
