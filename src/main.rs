use robot_controller::{Controller, Servo};
use std::error::Error;
use strum::IntoEnumIterator;
use std::f32::consts::PI;

fn scan_circle(controller: &mut Controller) -> Result<u32, Box<dyn Error>> {
    let mut total_retries = 0;

    // Center point
    let center_azimuth = 0.0;
    let center_elevation = 30.0;

    // Radius of 30 degrees
    let radius = 60.0;

    // Number of points in the circle
    let num_points = 36; // This gives points every 10 degrees

    // Move to each point in the circle
    for i in 0..=num_points {
        // Calculate angle in radians
        let angle = (2.0 * PI * i as f32) / num_points as f32;

        // Calculate point on circle
        // Using spherical coordinates converted to azimuth and elevation
        let x = radius * angle.cos();
        let y = radius * angle.sin();

        // Convert to azimuth and elevation
        // Add to center point to offset the circle
        let azimuth = center_azimuth + x;
        let elevation = center_elevation + y;

        // Move to the point
        total_retries += controller.set_look(azimuth, elevation)?;
    }

    // Return to center position
    total_retries += controller.set_look(center_azimuth, center_elevation)?;

    Ok(total_retries)
}

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

    let mut retries = scan_circle(&mut controller)?;

    retries += controller.set_look(-60.0, -125.0)?;
    retries += controller.set_look(60.0, 125.0)?;
    retries += scan(&mut controller)?;
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