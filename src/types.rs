use strum_macros::EnumIter;

#[derive(Debug, EnumIter, Clone, Copy, Eq, PartialEq, Hash)]
pub enum Servo {
    ClawGrip = 1,
    ClawTwist = 2,
    WristTilt = 3,    // -125 to 125 up to down
    ElbowTilt = 4,    // -125 to 125 up to down
    ShoulderTilt = 5, // -125 to 125 up to down
    BaseSpin = 6,     // -125 to 125 clockwise
}

#[derive(Debug)]
pub struct JointAngles {
    pub shoulder: f32,
    pub elbow: f32,
    pub wrist: f32,
}

pub(crate) fn clamp_angle(angle: f32) -> f32 {
    use crate::constants::{MAX_ANGLE, MIN_ANGLE};
    angle.max(MIN_ANGLE).min(MAX_ANGLE)
}
