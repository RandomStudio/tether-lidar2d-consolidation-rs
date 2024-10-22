use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct TrackedPoint2D {
    id: usize,
    pub x: f32,
    pub y: f32,
    pub velocity: Option<[f32; 2]>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Keypoint3D {
    pub i: usize,
    pub xyz: (f32, f32, f32),
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
/// As per tether-oakd-blazepose tracking
pub struct Body3D {
    pub body_xyz: (f32, f32, f32),
    pub kp: Vec<Keypoint3D>,
}

/// As per tether-oakd-blazepose tracking
pub type BodyFrame3D = Vec<Body3D>;

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

    pub fn id(&self) -> usize {
        self.id
    }
}
