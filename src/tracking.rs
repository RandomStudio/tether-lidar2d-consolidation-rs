use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct TrackedPoint2D {
    id: usize,
    pub x: f64,
    pub y: f64,
    pub velocity: Option<[f64; 2]>,
}

impl TrackedPoint2D {
    pub fn new(id: usize, position: (f64, f64)) -> Self {
        TrackedPoint2D {
            id,
            x: position.0,
            y: position.1,
            velocity: None,
        }
    }
    pub fn set_velocity(&mut self, velocity: Option<[f64; 2]>) {
        self.velocity = velocity;
    }
}
