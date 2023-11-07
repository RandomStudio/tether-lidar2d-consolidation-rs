use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct TrackedPoint2D {
    id: usize,
    pub x: f32,
    pub y: f32,
    pub velocity: Option<[f32; 2]>,
}

impl TrackedPoint2D {
    pub fn new(id: usize, position: (f32, f32)) -> Self {
        TrackedPoint2D {
            id,
            x: position.0,
            y: position.1,
            velocity: None,
        }
    }
    pub fn set_velocity(&mut self, velocity: Option<[f32; 2]>) {
        self.velocity = velocity;
    }
}
