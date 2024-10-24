// src/controller.rs
use crate::{
    constants::*,
    types::{clamp_angle, JointAngles, Servo},
};
use hidapi::HidApi;
use std::collections::HashMap;
use std::error::Error;
use std::fmt;

pub struct Controller {
    device: hidapi::HidDevice,
}
// Custom error type for better error messages
#[derive(Debug)]
pub enum ControllerError {
    InvalidResponse {
        expected_len: usize,
        actual_len: usize,
        raw_data: Vec<u8>,
    },
    DeviceError(String),
}

impl fmt::Display for ControllerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ControllerError::InvalidResponse { expected_len, actual_len, raw_data } => {
                write!(f, "Invalid response data: expected length {} but got {}. Raw data: {:02x?}",
                       expected_len, actual_len, raw_data)
            }
            ControllerError::DeviceError(msg) => write!(f, "Device error: {}", msg),
        }
    }
}

impl Error for ControllerError {}

impl Controller {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let api = HidApi::new()?;
        let device = api.open(VENDOR_ID, PRODUCT_ID)?;

        Ok(Controller {
            device,
        })
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

        if res < 4 {
            return Err(ControllerError::InvalidResponse {
                expected_len: 4,
                actual_len: res,
                raw_data: buf[..res].to_vec(),
            }.into());
        }

        if buf[0] != SIGNATURE || buf[1] != SIGNATURE {
            return Err(ControllerError::DeviceError(
                format!("Invalid signature: {:02x} {:02x}", buf[0], buf[1])
            ).into());
        }

        if buf[3] != cmd {
            return Err(ControllerError::DeviceError(
                format!("Command mismatch: expected {:02x}, got {:02x}", cmd, buf[3])
            ).into());
        }

        let length = buf[2] as usize;
        if res < 4 + length {
            return Err(ControllerError::InvalidResponse {
                expected_len: 4 + length,
                actual_len: res,
                raw_data: buf[..res].to_vec(),
            }.into());
        }

        Ok(buf[4..4 + length].to_vec())
    }
    pub fn get_battery_voltage(&mut self) -> Result<f32, Box<dyn Error>> {
        self._send(CMD_GET_BATTERY_VOLTAGE, &[])?;
        let data = self._recv(CMD_GET_BATTERY_VOLTAGE)?;
        if data.len() >= 2 {
            Ok(((data[1] as u16 * 256 + data[0] as u16) as f32) / 1000.0)
        } else {
            Err("Invalid battery voltage data".into())
        }
    }

    fn _angle_to_position(angle: f32) -> u16 {
        ((angle + 125.0) * 1000.0 / 250.0) as u16
    }

    fn _position_to_angle(position: u16) -> f32 {
        (position as f32) * 250.0 / 1000.0 - 125.0
    }

    pub fn get_positions(&mut self, servos: &[Servo]) -> Result<HashMap<Servo, f32>, Box<dyn Error>> {
        if servos.is_empty() {
            return Ok(HashMap::new());
        }

        let mut data = vec![servos.len() as u8];
        for &servo in servos {
            data.push(servo as u8);
        }

        self._send(CMD_GET_SERVO_POSITION, &data)?;

        let response = self._recv(CMD_GET_SERVO_POSITION)?;

        let mut positions = HashMap::with_capacity(servos.len());
        let mut response_idx = 1; // Skip the count byte

        while response_idx + 2 < response.len() {
            let servo_id = response[response_idx];
            let position_low = response[response_idx + 1];
            let position_high = response[response_idx + 2];

            // Convert servo_id back to Servo enum
            if let Some(servo) = servos.iter().find(|&&s| s as u8 == servo_id) {
                let position = (position_high as u16) * 256 + position_low as u16;
                let angle = Self::_position_to_angle(position);
                positions.insert(*servo, angle);
            }

            response_idx += 3;
        }

        if positions.len() != servos.len() {
            println!("Warning: Only got positions for {}/{} servos",
                     positions.len(), servos.len());
        }

        Ok(positions)
    }

    pub fn servo_off(&mut self, servo_id: Option<u8>) -> Result<(), Box<dyn Error>> {
        let data = match servo_id {
            Some(id) => vec![1u8, id],
            None => vec![6u8, 1, 2, 3, 4, 5, 6],
        };
        self._send(CMD_SERVO_STOP, &data)?;
        Ok(())
    }

    pub fn calculate_joint_angles(&self, target_elevation: f32) -> JointAngles {
        let target_elevation = target_elevation.max(MIN_ELEVATION).min(MAX_ELEVATION);
        let target_total_angle = 90.0 - target_elevation;
        let shoulder = clamp_angle(-target_total_angle * 0.4);
        let elbow = clamp_angle(target_total_angle * 0.8);
        let wrist = clamp_angle(target_total_angle - shoulder - elbow);

        JointAngles {
            shoulder: (shoulder * 10.0).round() / 10.0,
            elbow: -(elbow * 10.0).round() / 10.0,
            wrist: (wrist * 10.0).round() / 10.0,
        }
    }

    pub fn set_look(&mut self, target_elevation: f32, target_azimuth: f32) -> Result<u32, Box<dyn Error>> {
        let angles = self.calculate_joint_angles(target_elevation);

        let movements = vec![
            (Servo::WristTilt, angles.wrist),
            (Servo::ElbowTilt, angles.elbow),
            (Servo::ShoulderTilt, angles.shoulder),
            (Servo::BaseSpin, target_azimuth),
        ];

        self.set_multiple_positions(&movements)
    }

    pub fn set_multiple_positions(&mut self, movements: &[(Servo, f32)]) -> Result<u32, Box<dyn Error>> {
        let angular_speed = 5.0; // degrees per millisecond
        let mut max_duration_ms = 20u16; // Minimum duration

        // Get current positions for all servos at once
        let servos: Vec<Servo> = movements.iter().map(|(servo, _)| *servo).collect();
        let current_positions = self.get_positions(&servos)?;

        // Calculate max duration based on the largest movement
        for &(servo, target_angle) in movements {
            if let Some(&current_angle) = current_positions.get(&servo) {
                let movement_size = (target_angle - current_angle).abs();

                if movement_size >= 1.0 {
                    let duration = ((movement_size * angular_speed).round() as u16).max(20);
                    max_duration_ms = max_duration_ms.max(duration);
                }
            }
        }

        // Prepare and send movement command
        let mut data = vec![
            movements.len() as u8,
            (max_duration_ms & 0xff) as u8,
            ((max_duration_ms & 0xff00) >> 8) as u8,
        ];

        // Add each servo movement to the command
        for &(servo, target_angle) in movements {
            if !(-125.0..=125.0).contains(&target_angle) {
                return Err(format!("Angle {} must be between -125.0 and 125.0 degrees", target_angle).into());
            }

            let position = Self::_angle_to_position(target_angle);
            data.extend_from_slice(&[
                servo as u8,
                (position & 0xff) as u8,
                ((position & 0xff00) >> 8) as u8,
            ]);
        }

        // Send command for all servos
        self._send(CMD_SERVO_MOVE, &data)?;

        // Wait for movement to complete
        std::thread::sleep(std::time::Duration::from_millis(max_duration_ms as u64));

        // Check final positions for all servos at once
        let final_positions = self.get_positions(&servos)?;
        let mut retry_movements = Vec::new();

        for &(servo, target_angle) in movements {
            if let Some(&achieved_position) = final_positions.get(&servo) {
                let error = achieved_position - target_angle;

                println!("Servo {} is off by {}", servo as u8, error);

                if error.abs() > 1.0 {
                    if let Some(&current_position) = current_positions.get(&servo) {
                        if current_position != achieved_position {
                            println!("Adding servo {} to retry list", servo as u8);
                            retry_movements.push((servo, target_angle));
                        }
                    }
                }
            }
        }

        // Recursively retry failed movements
        if !retry_movements.is_empty() {
            println!("Retrying movement for {} servos", retry_movements.len());
            let retry_count = self.set_multiple_positions(&retry_movements)?;
            Ok(retry_count + 1)
        } else {
            Ok(0)
        }
    }
}