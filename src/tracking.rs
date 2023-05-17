use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct TrackedPoint2D {
    id: usize,
    x: f64,
    y: f64,
}

impl TrackedPoint2D {
    pub fn new(id: usize, position: (f64, f64)) -> Self {
        TrackedPoint2D {
            id,
            x: position.0,
            y: position.1,
        }
    }
}
