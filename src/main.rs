use hidapi::HidApi;
use std::error::Error;

const VENDOR_ID: u16 = 0x0483;
const PRODUCT_ID: u16 = 0x5750;
const SIGNATURE: u8 = 0x55;
const CMD_SERVO_MOVE: u8 = 0x03;
const CMD_GET_BATTERY_VOLTAGE: u8 = 0x0f;
const CMD_SERVO_STOP: u8 = 0x14;
const CMD_GET_SERVO_POSITION: u8 = 0x15;

struct Controller {
    device: hidapi::HidDevice,
}

struct JointAngles {
    pub shoulder: f32,
    pub elbow: f32,
    pub wrist: f32,
}

impl Controller {
    fn new() -> Result<Self, Box<dyn Error>> {
        let api = HidApi::new()?;
        let device = api.open(VENDOR_ID, PRODUCT_ID)?;
        Ok(Controller { device })
    }

    fn _send(&mut self, cmd: u8, data: &[u8]) -> Result<(), Box<dyn Error>> {
        let mut report_data = vec![0, SIGNATURE, SIGNATURE, (data.len() + 2) as u8, cmd];
        report_data.extend_from_slice(data);
        self.device.write(&report_data)?;
        Ok(())
    }

    fn _recv(&mut self, cmd: u8) -> Result<Vec<u8>, Box<dyn Error>> {
        let mut buf = [0u8; 64];
        let res = self.device.read_timeout(&mut buf, 1000)?;
        if res >= 4 && buf[0] == SIGNATURE && buf[1] == SIGNATURE && buf[3] == cmd {
            let length = buf[2] as usize;
            Ok(buf[4..4 + length].to_vec())
        } else {
            Err("Invalid response".into())
        }
    }

    fn get_battery_voltage(&mut self) -> Result<f32, Box<dyn Error>> {
        self._send(CMD_GET_BATTERY_VOLTAGE, &[])?;
        let data = self._recv(CMD_GET_BATTERY_VOLTAGE)?;
        if data.len() >= 2 {
            Ok(((data[1] as u16 * 256 + data[0] as u16) as f32) / 1000.0)
        } else {
            Err("Invalid battery voltage data".into())
        }
    }

    // Helper function to convert angle to position
    fn _angle_to_position(angle: f32) -> u16 {
        // Assuming the same conversion as in Python's Util class
        ((angle + 125.0) * 1000.0 / 250.0) as u16
    }

    // Helper function to convert position to angle
    fn _position_to_angle(position: u16) -> f32 {
        (position as f32) * 250.0 / 1000.0 - 125.0
    }

    /// Set the position of a servo
    /// If degrees=true, position should be between -125.0 and 125.0 degrees
    /// If degrees=false, position should be between 0 and 1000
    fn set_position<T: Into<f32>>(&mut self, servo_id: Servo, position: T, degrees: bool, duration_ms: u16) -> Result<(), Box<dyn Error>> {
        let mut data = vec![
            1u8, // number of servos
            (duration_ms & 0xff) as u8,
            ((duration_ms & 0xff00) >> 8) as u8,
        ];

        let position = position.into();
        let pos = if degrees {
            if !(-125.0..=125.0).contains(&position) {
                return Err("Angle must be between -125.0 and 125.0 degrees".into());
            }
            Self::_angle_to_position(position)
        } else {
            if !(0.0..=1000.0).contains(&position) {
                return Err("Position must be between 0 and 1000".into());
            }
            position as u16
        };

        data.extend_from_slice(&[
            servo_id as u8,
            (pos & 0xff) as u8,
            ((pos & 0xff00) >> 8) as u8,
        ]);

        self._send(CMD_SERVO_MOVE, &data)?;

        // Wait till it completes
        std::thread::sleep(std::time::Duration::from_millis(duration_ms as u64));

        Ok(())
    }

    /// Get the position of a servo
    /// Returns either raw position (0-1000) or angle (-125.0 to 125.0) if degrees=true
    fn get_position(&mut self, servo_id: Servo, degrees: bool) -> Result<f32, Box<dyn Error>> {
        let data = [1u8, servo_id as u8];
        self._send(CMD_GET_SERVO_POSITION, &data)?;

        let response = self._recv(CMD_GET_SERVO_POSITION)?;
        if response.len() >= 4 {
            let position = (response[3] as u16) * 256 + response[2] as u16;
            if degrees {
                Ok(Self::_position_to_angle(position))
            } else {
                Ok(position as f32)
            }
        } else {
            Err("Invalid position data received".into())
        }
    }

    /// Turn off one or all servos
    /// If servo_id is None, turns off all servos (1-6)
    fn servo_off(&mut self, servo_id: Option<u8>) -> Result<(), Box<dyn Error>> {
        let data = match servo_id {
            Some(id) => vec![1u8, id],
            None => vec![6u8, 1, 2, 3, 4, 5, 6], // Turn off all servos
        };

        self._send(CMD_SERVO_STOP, &data)?;
        Ok(())
    }

    fn calculate_joint_angles(&self, target_elevation: f32) -> JointAngles {
        const MIN_ANGLE: f32 = -125.0;
        const MAX_ANGLE: f32 = 125.0;

        // Helper function to clamp angles within constraints
        fn clamp_angle(angle: f32) -> f32 {
            angle.max(MIN_ANGLE).min(MAX_ANGLE)
        }

        const MIN_ELEVATION: f32 = -60.0;
        const MAX_ELEVATION: f32 = 90.0;

        // Clamp the target_elevation
        let target_elevation = target_elevation.max(MIN_ELEVATION).min(MAX_ELEVATION);

        // For the claw to point at target elevation,
        // the sum of all angles should be (90 - elevation)
        let target_total_angle = 90.0 - target_elevation;

        // Start with shoulder at 40% of the target total (negative because CCW)
        let shoulder = clamp_angle(-target_total_angle * 0.4);

        // Use elbow to bring us 80% of the way there
        let elbow = clamp_angle(target_total_angle * 0.8);

        // Wrist makes the final adjustment to achieve the target angle
        // Total angle needs to be target_total_angle
        // So wrist = target_total_angle - shoulder - elbow
        let wrist = clamp_angle(target_total_angle - shoulder - elbow);

        JointAngles {
            shoulder: (shoulder * 10.0).round() / 10.0,  // Round to 1 decimal place
            elbow: - (elbow * 10.0).round() / 10.0,
            wrist: (wrist * 10.0).round() / 10.0,
        }
    }

    fn set_look(&mut self, target_elevation: f32, target_azimuth: f32) -> Result<(), Box<dyn Error>> {
        let angles = self.calculate_joint_angles(target_elevation);

        self.set_position(Servo::WristTilt, angles.wrist, true, 200)?;
        self.set_position(Servo::ElbowTilt, angles.elbow, true, 200)?;
        self.set_position(Servo::ShoulderTilt, angles.shoulder, true, 200)?;
        self.set_position(Servo::BaseSpin, target_azimuth, true, 200)?;

        Ok(())
    }

}

use strum::IntoEnumIterator;
use strum_macros::EnumIter;

#[derive(Debug, EnumIter, Clone, Copy)]
enum Servo {
    WristTilt = 3, // -125 to 125 up to down
    ElbowTilt = 4, // -125 to 125 up to down
    ShoulderTilt = 5, // -125 to 125 up to down
    BaseSpin = 6, // -125 to 125 clockwise
}

fn scan(controller: &mut Controller) -> Result<(), Box<dyn Error>> {
    // Move between 90 and -60 and back to 0
    for i in 0..=15 {
        controller.set_look(90.0 - i as f32 * 10.0, 0.0)?;
    }
    for i in 1..=6 {
        controller.set_look(-60.0 + i as f32 * 10.0, 0.0)?;
    }

    for i in 0..=25 {
        controller.set_look(0.0, -125.0 + i as f32 * 10.0)?;
    }

    controller.set_look(0.0, 0.0)?;
    Ok(())
}

// Example usage in main
fn main() -> Result<(), Box<dyn Error>> {
    let mut controller = Controller::new()?;

    // Get battery voltage
    if let Ok(voltage) = controller.get_battery_voltage() {
        println!("Battery voltage: {:.2}V", voltage);
    }

    scan(&mut controller)?;
    controller.set_look(0.0, 0.0)?;

    // Get positions of all servos in degrees
    for servo in Servo::iter() {
        if let Ok(position) = controller.get_position(servo, true) {
            println!("{:?} position: {:.1} degrees", servo, position);
        } else {
            println!("Failed to get position for {:?}", servo);
        }
    }

    Ok(())
}