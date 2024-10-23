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
        // Filter out outliers: Remove measurements where error/size ratio is more than 2 standard deviations from mean
        let filter_outliers = |measurements: &[(f32, f32)]| -> Vec<f32> {
            if measurements.is_empty() {
                return vec![];
            }

            // Calculate ratios
            let ratios: Vec<f32> = measurements.iter()
                .map(|(size, error)| error / size)
                .collect();

            // Calculate mean and standard deviation
            let mean = ratios.iter().sum::<f32>() / ratios.len() as f32;
            let variance = ratios.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f32>() / ratios.len() as f32;
            let std_dev = variance.sqrt();

            // Filter outliers
            ratios.into_iter()
                .filter(|&ratio| (ratio - mean).abs() <= 2.0 * std_dev)
                .collect()
        };

        // Apply adaptive scaling factor based on error magnitude
        let calculate_scaling = |avg_error: f32| -> f32 {
            // Smaller corrections for larger errors to prevent overshooting
            if avg_error.abs() > 0.5 {
                5.0
            } else if avg_error.abs() > 0.2 {
                7.0
            } else {
                10.0
            }
        };

        let pos_ratios = filter_outliers(positive_errors);
        let neg_ratios = filter_outliers(negative_errors);

        let pos_avg = if !pos_ratios.is_empty() {
            let avg = pos_ratios.iter().sum::<f32>() / pos_ratios.len() as f32;
            avg * calculate_scaling(avg)
        } else {
            0.0
        };

        let neg_avg = if !neg_ratios.is_empty() {
            let avg = neg_ratios.iter().sum::<f32>() / neg_ratios.len() as f32;
            avg * calculate_scaling(avg)
        } else {
            0.0
        };

        ServoCalibration {
            positive_movement: pos_avg,
            negative_movement: neg_avg,
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