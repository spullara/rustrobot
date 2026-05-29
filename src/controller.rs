use crate::{
    constants::*,
    types::{clamp_angle, JointAngles, Servo},
    transport::Transport,
};
use std::collections::HashMap;
use std::error::Error;
use tokio::time::Duration;
use std::pin::Pin;
use std::future::Future;

pub struct Controller {
    transport: Transport,
}

impl Controller {
    pub async fn new() -> Result<Self, Box<dyn Error + Send + Sync>> {
        Ok(Controller {
            transport: Transport::new().await?,
        })
    }

    pub async fn get_battery_voltage(&mut self) -> Result<f32, Box<dyn Error + Send + Sync>> {
        self.transport.send(CMD_GET_BATTERY_VOLTAGE, &[]).await?;
        let data = self.transport.recv(CMD_GET_BATTERY_VOLTAGE).await?;

        if data.len() >= 2 {
            let voltage = (data[0] as u16 | ((data[1] as u16) << 8)) as f32 / 1000.0;
            println!("Battery voltage: {}V", voltage);
            Ok(voltage)
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

    pub async fn get_positions(&mut self, servos: &[Servo]) -> Result<HashMap<Servo, f32>, Box<dyn Error + Send + Sync>> {
        if servos.is_empty() {
            return Ok(HashMap::new());
        }

        let mut data = vec![servos.len() as u8];
        for &servo in servos {
            data.push(servo as u8);
        }

        self.transport.send(CMD_GET_SERVO_POSITION, &data).await?;
        let response = self.transport.recv(CMD_GET_SERVO_POSITION).await?;

        let mut positions = HashMap::with_capacity(servos.len());
        let mut response_idx = 1;

        while response_idx + 2 < response.len() {
            let servo_id = response[response_idx];
            let position = response[response_idx + 1] as u16 | ((response[response_idx + 2] as u16) << 8);

            if let Some(servo) = servos.iter().find(|&&s| s as u8 == servo_id) {
                let angle = Self::_position_to_angle(position);
                println!("Servo {} position: {} (raw: {})", servo_id, angle, position);
                positions.insert(*servo, angle);
            }

            response_idx += 3;
        }

        Ok(positions)
    }

    pub async fn servo_off(&mut self, servo_id: Option<u8>) -> Result<(), Box<dyn Error + Send + Sync>> {
        let data = match servo_id {
            Some(id) => vec![1u8, id],
            None => vec![6u8, 1, 2, 3, 4, 5, 6],
        };
        self.transport.send(CMD_SERVO_STOP, &data).await?;
        Ok(())
    }

    /// Closed-form posture solver for the phone-holder use case. The input is a
    /// target elevation, meaning the direction the payload should aim, and the
    /// output is the shoulder, elbow, and wrist tilt angles. This is not a
    /// generic inverse-kinematics routine: base spin, or azimuth, is handled by
    /// the caller.
    ///
    /// The geometry assumed here is an equal-length upper arm and forearm,
    /// `L1 = L2 = 10 cm`. The third link, from wrist hinge to payload, is about
    /// `6 cm` to the camera face with the current mount. That length does not
    /// enter this joint split; it changes where the payload ends up in space,
    /// but not how the shoulder and elbow divide the aiming angle.
    ///
    /// Let `t = 90 - elevation`, after clamping the elevation to
    /// `MIN_ELEVATION..=MAX_ELEVATION`. The wrist formula,
    /// `wrist = t - shoulder - elbow`, is the orientation closure. Each joint
    /// angle is a tilt relative to the previous link, so the cumulative payload
    /// tilt is `shoulder + elbow + wrist`. Forcing that sum to be `t` makes the
    /// payload point at the requested elevation regardless of how the upper
    /// joints split the angle.
    ///
    /// The `0.8` elbow coefficient is `2 × 0.4` because that 2:1 ratio is the
    /// stability constraint. In side view, the horizontal wrist offset is
    /// `-L1·sin(k1·t) + L2·sin((k2 - k1)·t)`, which is identically zero exactly
    /// when `L1 == L2` and `k2 == 2·k1`. With the phone mounted at or near the
    /// wrist, keeping the wrist over the base is the center-of-mass-over-base
    /// condition.
    ///
    /// The specific `k1 = 0.4` value comes from the elbow servo limit. At
    /// `MIN_ELEVATION = -60`, the worst-case total aim is `t_max = 150°`; the
    /// elbow magnitude is `2·k1·t_max`. Keeping that within the `MIN_ANGLE` to
    /// `MAX_ANGLE` servo range gives `k1 ≤ 125 / 300 = 0.4167`, so `0.4` leaves
    /// about `5°` of elbow margin, with the elbow at `120°`. The shoulder and
    /// wrist limits are not binding.
    ///
    /// These constants must be re-derived if `L1` and `L2` stop being equal. In
    /// the small-angle case the ratio becomes `k2 = k1·(1 + L1/L2)`, and the
    /// exact solution should be used otherwise. Changing the elbow servo limit,
    /// `MIN_ELEVATION`, or `MAX_ELEVATION` shifts the allowed `k1` range, and
    /// mounting the payload significantly past the wrist hinge would make
    /// wrist-over-base a weaker approximation for center-of-mass-over-base.
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

    pub fn set_multiple_positions<'a>(&'a mut self, movements: &'a [(Servo, f32)])
                                      -> Pin<Box<dyn Future<Output = Result<u32, Box<dyn Error + Send + Sync>>> + Send + 'a>>
    {
        Box::pin(async move {
            let angular_speed = 5.0; // degrees per millisecond
            let mut max_duration_ms = 20u16; // Minimum duration

            // Get current positions for all servos at once
            let servos: Vec<Servo> = movements.iter().map(|(servo, _)| *servo).collect();
            let current_positions = self.get_positions(&servos).await?;

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

            // Prepare movement command
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
            self.transport.send(CMD_SERVO_MOVE, &data).await?;
            println!("Waiting for {}ms", max_duration_ms);
            tokio::time::sleep(Duration::from_millis(max_duration_ms as u64)).await;

            // Check final positions
            let final_positions = self.get_positions(&servos).await?;
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
                let retry_count = self.set_multiple_positions(&retry_movements).await?;
                Ok(retry_count + 1)
            } else {
                Ok(0)
            }
        })
    }

    // You'll need to update set_look to handle the changed signature
    pub async fn set_look(&mut self, target_elevation: f32, target_azimuth: f32) -> Result<u32, Box<dyn Error + Send + Sync>> {
        let angles = self.calculate_joint_angles(target_elevation);

        let movements = vec![
            (Servo::WristTilt, angles.wrist),
            (Servo::ElbowTilt, angles.elbow),
            (Servo::ShoulderTilt, angles.shoulder),
            (Servo::BaseSpin, target_azimuth),
        ];

        self.set_multiple_positions(&movements).await
    }
}