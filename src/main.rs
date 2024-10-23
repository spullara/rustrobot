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

    let mut retries = scan(&mut controller)?;
    retries += controller.set_look(0.0, -125.0)?;
    retries += controller.set_look(0.0, 125.0)?;
    retries += controller.set_look(0.0, 0.0)?;

    println!("Total retries: {}", retries);

    for servo in Servo::iter() {
        if let Ok(position) = controller.get_position(servo) {
            println!("{:?} position: {:.1} degrees", servo, position);
        } else {
            println!("Failed to get position for {:?}", servo);
        }
    }

    Ok(())
}