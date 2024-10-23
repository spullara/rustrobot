use robot_controller::{Controller, Servo};
use std::error::Error;
use strum::IntoEnumIterator;

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

    println!("Running calibration movements...");
    scan(&mut controller)?;

    controller.set_look(0.0, -125.0)?;
    controller.set_look(0.0, 125.0)?;
    controller.set_look(0.0, 0.0)?;

    controller.calculate_calibration();

    for servo in Servo::iter() {
        if let Ok(position) = controller.get_position(servo) {
            println!("{:?} position: {:.1} degrees", servo, position);
        } else {
            println!("Failed to get position for {:?}", servo);
        }
    }

    Ok(())
}