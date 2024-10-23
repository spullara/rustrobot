use crate::types::Servo;
use std::collections::HashMap;
use strum::IntoEnumIterator;

#[derive(Debug, Clone)]
pub struct ServoCalibration {
    pub(crate) positive_movement: f32,
    pub(crate) negative_movement: f32,
}

impl ServoCalibration {
    pub fn new() -> Self {
        ServoCalibration {
            positive_movement: 0.0,
            negative_movement: 0.0,
        }
    }

    pub fn calculate_from_movements(positive_errors: &[(f32, f32)], negative_errors: &[(f32, f32)]) -> Self {
        let pos_pct = if !positive_errors.is_empty() {
            positive_errors.iter()
                .map(|(size, error)| error / size)
                .sum::<f32>() / positive_errors.len() as f32
        } else {
            0.0
        };

        let neg_pct = if !negative_errors.is_empty() {
            negative_errors.iter()
                .map(|(size, error)| error / size)
                .sum::<f32>() / negative_errors.len() as f32
        } else {
            0.0
        };

        ServoCalibration {
            positive_movement: pos_pct * 10.0,
            negative_movement: neg_pct * 10.0,
        }
    }
}

#[derive(Default)]
pub(crate) struct CalibrationData {
    pub calibrations: HashMap<Servo, ServoCalibration>,
    pub collecting_data: bool,
    pub movement_data: HashMap<Servo, (Vec<(f32, f32)>, Vec<(f32, f32)>)>,
}

impl CalibrationData {
    pub fn new() -> Self {
        let mut calibrations = HashMap::new();
        let mut movement_data = HashMap::new();

        for servo in Servo::iter() {
            calibrations.insert(servo, ServoCalibration::new());
            movement_data.insert(servo, (Vec::new(), Vec::new()));
        }

        CalibrationData {
            calibrations,
            collecting_data: false,
            movement_data,
        }
    }
}