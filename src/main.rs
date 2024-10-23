use hidapi::HidApi;
use std::error::Error;
use std::collections::HashMap;
use strum::IntoEnumIterator;
use strum_macros::EnumIter;

const VENDOR_ID: u16 = 0x0483;
const PRODUCT_ID: u16 = 0x5750;
const SIGNATURE: u8 = 0x55;
const CMD_SERVO_MOVE: u8 = 0x03;
const CMD_GET_BATTERY_VOLTAGE: u8 = 0x0f;
const CMD_SERVO_STOP: u8 = 0x14;
const CMD_GET_SERVO_POSITION: u8 = 0x15;

#[derive(Debug, EnumIter, Clone, Copy, Eq, PartialEq, Hash)]
enum Servo {
    WristTilt = 3,    // -125 to 125 up to down
    ElbowTilt = 4,    // -125 to 125 up to down
    ShoulderTilt = 5, // -125 to 125 up to down
    BaseSpin = 6,     // -125 to 125 clockwise
}

#[derive(Debug)]
struct JointAngles {
    pub shoulder: f32,
    pub elbow: f32,
    pub wrist: f32,
}

#[derive(Debug, Clone)]
struct ServoCalibration {
    positive_movement: f32,
    negative_movement: f32,
}

impl ServoCalibration {
    fn new() -> Self {
        ServoCalibration {
            positive_movement: 0.0,
            negative_movement: 0.0,
        }
    }

    fn calculate_from_movements(positive_errors: &[(f32, f32)], negative_errors: &[(f32, f32)]) -> Self {
        let pos_pct = if !positive_errors.is_empty() {
            positive_errors.iter()
                .map(|(size, error)| (error / size) * 100.0)
                .sum::<f32>() / positive_errors.len() as f32
        } else {
            0.0
        };

        let neg_pct = if !negative_errors.is_empty() {
            negative_errors.iter()
                .map(|(size, error)| (error / size) * 100.0)
                .sum::<f32>() / negative_errors.len() as f32
        } else {
            0.0
        };

        ServoCalibration {
            positive_movement: pos_pct,
            negative_movement: neg_pct,
        }
    }
}

struct Controller {
    device: hidapi::HidDevice,
    calibrations: HashMap<Servo, ServoCalibration>,
    collecting_data: bool,
    movement_data: HashMap<Servo, (Vec<(f32, f32)>, Vec<(f32, f32)>)>,
}

impl Controller {
    fn new() -> Result<Self, Box<dyn Error>> {
        let api = HidApi::new()?;
        let device = api.open(VENDOR_ID, PRODUCT_ID)?;
        let mut calibrations = HashMap::new();
        let mut movement_data = HashMap::new();

        for servo in Servo::iter() {
            calibrations.insert(servo, ServoCalibration::new());
            movement_data.insert(servo, (Vec::new(), Vec::new()));
        }

        Ok(Controller {
            device,
            calibrations,
            collecting_data: false,
            movement_data,
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

    fn get_battery_voltage(&mut self) -> Result<f32, Box<dyn Error>> {
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

    fn start_collecting_data(&mut self) {
        self.collecting_data = true;
        for servo in Servo::iter() {
            self.movement_data.insert(servo, (Vec::new(), Vec::new()));
        }
        println!("Started collecting calibration data");
    }

    fn calculate_calibration(&mut self) {
        self.collecting_data = false;
        for servo in Servo::iter() {
            if let Some((positive_moves, negative_moves)) = self.movement_data.get(&servo) {
                let calibration = ServoCalibration::calculate_from_movements(positive_moves, negative_moves);
                self.calibrations.insert(servo, calibration);
            }
        }
        println!("Calibration calculated from collected data:");
        self.print_calibration_status();
    }

    fn set_position<T: Into<f32>>(&mut self, servo_id: Servo, position: T) -> Result<(), Box<dyn Error>> {
        let angular_speed = 5.0;
        let target_position = position.into();

        if !(-125.0..=125.0).contains(&target_position) {
            return Err("Angle must be between -125.0 and 125.0 degrees".into());
        }

        let current_angle = self.get_position(servo_id)?;
        let movement_size = target_position - current_angle;

        if movement_size.abs() < 1.0 {
            return Ok(());
        }

        let adjusted_target = if !self.collecting_data {
            if let Some(calibration) = self.calibrations.get(&servo_id) {
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
        println!("Servo {} is of by {}", servo_id as u8, error);

        if self.collecting_data {
            if let Some((positive_moves, negative_moves)) = self.movement_data.get_mut(&servo_id) {
                if movement_size > 0.0 {
                    positive_moves.push((movement_size, error));
                } else {
                    negative_moves.push((movement_size.abs(), error));
                }
            }
        }

        if error.abs() > 1.0 {
            println!("Retrying servo {} move", servo_id as u8);
            self.set_position(servo_id, target_position)?;
        }

        Ok(())
    }

    fn get_position(&mut self, servo_id: Servo) -> Result<f32, Box<dyn Error>> {
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

    fn servo_off(&mut self, servo_id: Option<u8>) -> Result<(), Box<dyn Error>> {
        let data = match servo_id {
            Some(id) => vec![1u8, id],
            None => vec![6u8, 1, 2, 3, 4, 5, 6],
        };
        self._send(CMD_SERVO_STOP, &data)?;
        Ok(())
    }

    fn calculate_joint_angles(&self, target_elevation: f32) -> JointAngles {
        const MIN_ANGLE: f32 = -125.0;
        const MAX_ANGLE: f32 = 125.0;
        const MIN_ELEVATION: f32 = -60.0;
        const MAX_ELEVATION: f32 = 90.0;

        fn clamp_angle(angle: f32) -> f32 {
            angle.max(MIN_ANGLE).min(MAX_ANGLE)
        }

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

    fn set_look(&mut self, target_elevation: f32, target_azimuth: f32) -> Result<(), Box<dyn Error>> {
        let angles = self.calculate_joint_angles(target_elevation);
        self.set_position(Servo::WristTilt, angles.wrist)?;
        self.set_position(Servo::ElbowTilt, angles.elbow)?;
        self.set_position(Servo::ShoulderTilt, angles.shoulder)?;
        self.set_position(Servo::BaseSpin, target_azimuth)?;
        Ok(())
    }

    fn print_calibration_status(&self) {
        println!("\nServo Calibration Status:");
        for servo in Servo::iter() {
            if let Some(cal) = self.calibrations.get(&servo) {
                println!("{:?}:", servo);
                println!("  Positive movements: {:.2}% adjustment", cal.positive_movement);
                println!("  Negative movements: {:.2}% adjustment", cal.negative_movement);
            }
        }
    }
}

fn scan(controller: &mut Controller) -> Result<(), Box<dyn Error>> {
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

fn main() -> Result<(), Box<dyn Error>> {
    let mut controller = Controller::new()?;

    if let Ok(voltage) = controller.get_battery_voltage() {
        println!("Battery voltage: {:.2}V", voltage);
    }

    // Start collecting calibration data
    controller.start_collecting_data();

    // Run calibration movements here
    // For example:
    println!("Running calibration movements...");
    scan(&mut controller)?;

    controller.set_look(0.0, -125.0)?;
    controller.set_look(0.0, 125.0)?;
    controller.set_look(0.0, 0.0)?;

    // Calculate calibrations from collected data
    controller.calculate_calibration();

    // Now run normal operations with calibration applied

    for servo in Servo::iter() {
        if let Ok(position) = controller.get_position(servo) {
            println!("{:?} position: {:.1} degrees", servo, position);
        } else {
            println!("Failed to get position for {:?}", servo);
        }
    }

    Ok(())
}