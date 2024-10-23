pub const VENDOR_ID: u16 = 0x0483;
pub const PRODUCT_ID: u16 = 0x5750;
pub const SIGNATURE: u8 = 0x55;

// Command constants
pub const CMD_SERVO_MOVE: u8 = 0x03;
pub const CMD_GET_BATTERY_VOLTAGE: u8 = 0x0f;
pub const CMD_SERVO_STOP: u8 = 0x14;
pub const CMD_GET_SERVO_POSITION: u8 = 0x15;

// Servo movement constants
pub const MIN_ANGLE: f32 = -125.0;
pub const MAX_ANGLE: f32 = 125.0;
pub const MIN_ELEVATION: f32 = -60.0;
pub const MAX_ELEVATION: f32 = 90.0;