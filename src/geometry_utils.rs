pub fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    ((x2 - x1).powf(2.0) + (y2 - y1).powf(2.0)).sqrt()
}
