use crate::Point2D;

pub fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    ((x2 - x1).powf(2.0) + (y2 - y1).powf(2.0)).sqrt()
}

pub fn centroid(points: &[Point2D]) -> Option<Point2D> {
    let count = points.len();
    points
        .iter()
        .cloned()
        .reduce(|acc, el| (acc.0 + el.0, acc.1 + el.1))
        .map(|(x, y)| (x / count as f32, y / count as f32))
}

pub fn distance_points(a: &Point2D, b: &Point2D) -> f32 {
    let (x1, y1) = *a;
    let (x2, y2) = *b;

    f32::sqrt(f32::powi(x1 - x2, 2) + f32::powi(y1 - y2, 2))
}

pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a * (1. - t) + (b * t)
}

/// Return the clockwise angle (in degrees) between two lines:
/// - origin to a point on the y-axis (0,a) where a is positive
/// - origin to the point (x,y)
///
/// This corresponds to the common-sense "heading" of a point
/// from the point-of-view of the origin
pub fn bearing(x: f32, y: f32) -> f32 {
    let angle_rad = y.atan2(x); // Get the angle from the positive x-axis in radians
    let angle_deg = angle_rad.to_degrees(); // Convert to degrees

    // Convert from positive x-axis reference to positive y-axis reference
    let heading = (90.0 - angle_deg) % 360.0;
    if heading < 0.0 {
        heading + 360.0
    } else {
        heading
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bearing_easy_cardinals() {
        assert_eq!(bearing(0., 1.0), 0.); // N
        assert_eq!(bearing(1.0, 1.0), 45.0); // NE
        assert_eq!(bearing(3.5, 3.5), 45.0); // Also NE
        assert_eq!(bearing(1.0, 0.), 90.0); // E
        assert_eq!(bearing(1.0, -1.0), 135.); // SE
        assert_eq!(bearing(0.0, -101.0), 180.); // S
        assert_eq!(bearing(-1.0, -1.0), 225.); // SW
        assert_eq!(bearing(-1.0, -0.), 270.); // W
        assert_eq!(bearing(-3.1, 3.1), 315.); // NW
    }
}
