#![allow(clippy::manual_clamp)]

mod constants;
mod controller;
mod transport;
mod types;

pub use controller::Controller;
pub use types::{JointAngles, Servo};

// Re-export commonly used items
pub use constants::{PRODUCT_ID, VENDOR_ID};
