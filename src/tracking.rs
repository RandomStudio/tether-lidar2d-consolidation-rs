use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TrackedPoint2D {
    pub id: usize,
    pub x: f32,
    pub y: f32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub velocity: Option<[f32; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading: Option<f32>,
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
            heading: None,
        }
    }
    pub fn set_velocity(&mut self, velocity: Option<[f32; 2]>) {
        self.velocity = velocity;
    }

    pub fn id(&self) -> usize {
        self.id
    }
}
