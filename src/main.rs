use robot_controller::{Controller, Servo};
use std::error::Error;
use strum::IntoEnumIterator;

fn scan(controller: &mut Controller) -> Result<u32, Box<dyn Error>> {
    let mut total_retries = 0;

    for i in 0..=15 {
        total_retries += controller.set_look(90.0 - i as f32 * 10.0, 0.0)?;
    }
    for i in 1..=6 {
        total_retries += controller.set_look(-60.0 + i as f32 * 10.0, 0.0)?;
    }

    for i in 0..=25 {
        total_retries += controller.set_look(0.0, -125.0 + i as f32 * 10.0)?;
    }

    total_retries += controller.set_look(0.0, 0.0)?;
    Ok(total_retries)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut controller = Controller::new()?;

    if let Ok(voltage) = controller.get_battery_voltage() {
        println!("Battery voltage: {:.2}V", voltage);
    }

    // Start collecting calibration data
    controller.start_collecting_data();

    println!("Running calibration movements...");
    let mut calibration_retries = scan(&mut controller)?;

    calibration_retries += controller.set_look(0.0, -125.0)?;
    calibration_retries += controller.set_look(0.0, 125.0)?;
    calibration_retries += controller.set_look(0.0, 0.0)?;

    println!("Calibration movements complete. Total retries during calibration: {}", calibration_retries);

    controller.calculate_calibration();

    println!("Calibration calculated. Running the same movements after calibration...");
    
    let mut post_calibration_retries = scan(&mut controller)?;

    post_calibration_retries += controller.set_look(0.0, -125.0)?;
    post_calibration_retries += controller.set_look(0.0, 125.0)?;
    post_calibration_retries += controller.set_look(0.0, 0.0)?;

    println!("Post-calibration movements complete. Total retries after calibration: {}", post_calibration_retries);

    if calibration_retries > post_calibration_retries {
        println!("Improvement: {} fewer retries", calibration_retries - post_calibration_retries);
    } else if calibration_retries < post_calibration_retries {
        println!("Regression: {} more retries", post_calibration_retries - calibration_retries);
    } else {
        println!("No change in the number of retries");
    }

    for servo in Servo::iter() {
        if let Ok(position) = controller.get_position(servo) {
            println!("{:?} position: {:.1} degrees", servo, position);
        } else {
            println!("Failed to get position for {:?}", servo);
        }
    }

    Ok(())
}