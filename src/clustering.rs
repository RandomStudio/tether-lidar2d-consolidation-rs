use crate::Point2D;
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
