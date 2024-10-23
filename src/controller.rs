// src/controller.rs
use crate::{
    constants::*,
    types::{Servo, JointAngles, clamp_angle},
    calibration::{ServoCalibration, CalibrationData},
};
use hidapi::HidApi;
use std::error::Error;
use strum::IntoEnumIterator;

pub struct Controller {
    device: hidapi::HidDevice,
    calibration_data: CalibrationData,
}

impl Controller {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let api = HidApi::new()?;
        let device = api.open(VENDOR_ID, PRODUCT_ID)?;

        Ok(Controller {
            device,
            calibration_data: CalibrationData::new(),
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
        if res >= 4 && buf[0] == SIGNATURE && buf[1] == SIGNATURE && buf[3] == cmd {
            let length = buf[2] as usize;
            Ok(buf[4..4 + length].to_vec())
        } else {
            Err("Invalid response".into())
        }
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

    pub fn start_collecting_data(&mut self) {
        self.calibration_data.collecting_data = true;
        for servo in Servo::iter() {
            self.calibration_data.movement_data.insert(servo, (Vec::new(), Vec::new()));
        }
        println!("Started collecting calibration data");
    }

    pub fn calculate_calibration(&mut self) {
        self.calibration_data.collecting_data = false;
        for servo in Servo::iter() {
            if let Some((positive_moves, negative_moves)) = self.calibration_data.movement_data.get(&servo) {
                let calibration = ServoCalibration::calculate_from_movements(positive_moves, negative_moves);
                self.calibration_data.calibrations.insert(servo, calibration);
            }
        }
        println!("Calibration calculated from collected data:");
        self.print_calibration_status();
    }

    fn _set_position_internal<T: Into<f32>>(&mut self, servo_id: Servo, position: T, is_retry: bool) -> Result<f32, Box<dyn Error>> {
        let angular_speed = 5.0;
        let target_position = position.into();

        if !(-125.0..=125.0).contains(&target_position) {
            return Err("Angle must be between -125.0 and 125.0 degrees".into());
        }

        let current_angle = self.get_position(servo_id)?;
        let movement_size = target_position - current_angle;

        if movement_size.abs() < 1.0 {
            return Ok(0.0);
        }

        let adjusted_target = if !self.calibration_data.collecting_data {
            if let Some(calibration) = self.calibration_data.calibrations.get(&servo_id) {
                if movement_size > 0.0 {
                    target_position + (movement_size * calibration.positive_movement / 100.0)
                } else {
                    target_position + (movement_size * calibration.negative_movement / 100.0)
                }
            } else {
                target_position
            }
        } else {
            target_position
        };

        let duration_ms = ((movement_size.abs() * angular_speed).round() as u16).max(20);
        println!(
            "Moving servo {} from {} to {} ({}ms)",
            servo_id as u8, current_angle, target_position, duration_ms
        );

        let mut data = vec![
            1u8,
            (duration_ms & 0xff) as u8,
            ((duration_ms & 0xff00) >> 8) as u8,
        ];

        let pos = Self::_angle_to_position(adjusted_target);
        data.extend_from_slice(&[
            servo_id as u8,
            (pos & 0xff) as u8,
            ((pos & 0xff00) >> 8) as u8,
        ]);

        self._send(CMD_SERVO_MOVE, &data)?;
        std::thread::sleep(std::time::Duration::from_millis(duration_ms as u64));

        let achieved_position = self.get_position(servo_id)?;
        let error = achieved_position - target_position;
        println!("Servo {} is off by {}", servo_id as u8, error);

        // Only collect data if in calibration mode and this is not a retry
        if self.calibration_data.collecting_data && !is_retry {
            if let Some((positive_moves, negative_moves)) = self.calibration_data.movement_data.get_mut(&servo_id) {
                if movement_size > 0.0 {
                    positive_moves.push((movement_size, error));
                } else {
                    negative_moves.push((movement_size.abs(), error));
                }
            }
        }

        Ok(error)
    }

    pub fn set_position<T: Into<f32>>(&mut self, servo_id: Servo, position: T) -> Result<(), Box<dyn Error>> {
        let target = position.into();
        let error = self._set_position_internal(servo_id, target, false)?;

        if error.abs() > 1.0 {
            println!("Retrying servo {} move", servo_id as u8);
            self._set_position_internal(servo_id, target, true)?;
        }

        Ok(())
    }

    pub fn get_position(&mut self, servo_id: Servo) -> Result<f32, Box<dyn Error>> {
        let data = [1u8, servo_id as u8];
        self._send(CMD_GET_SERVO_POSITION, &data)?;

        let response = self._recv(CMD_GET_SERVO_POSITION)?;
        if response.len() >= 4 {
            let position = (response[3] as u16) * 256 + response[2] as u16;
            Ok(Self::_position_to_angle(position))
        } else {
            Err("Invalid position data received".into())
        }
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

    pub fn set_look(&mut self, target_elevation: f32, target_azimuth: f32) -> Result<(), Box<dyn Error>> {
        let angles = self.calculate_joint_angles(target_elevation);
        self.set_position(Servo::WristTilt, angles.wrist)?;
        self.set_position(Servo::ElbowTilt, angles.elbow)?;
        self.set_position(Servo::ShoulderTilt, angles.shoulder)?;
        self.set_position(Servo::BaseSpin, target_azimuth)?;
        Ok(())
    }

    pub fn print_calibration_status(&self) {
        println!("\nServo Calibration Status:");
        for servo in Servo::iter() {
            if let Some(cal) = self.calibration_data.calibrations.get(&servo) {
                println!("{:?}:", servo);
                println!("  Positive movements: {:.2}% adjustment", cal.positive_movement);
                println!("  Negative movements: {:.2}% adjustment", cal.negative_movement);
            }
        }
    }
}