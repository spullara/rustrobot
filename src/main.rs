use robot_controller::{Controller, Servo};
use std::error::Error;
use std::f32::consts::PI;

const USAGE: &str = "Usage:\nrobot_controller [OPTIONS]\n\nOPTIONS:\n    -s, --speed <MULTIPLIER>    Speed multiplier (positive finite f32, default 1.0).\n                                Values > 1.0 are faster; values < 1.0 are slower.\n    -h, --help                  Print this help message and exit.\n";

#[derive(Debug, PartialEq)]
struct Config {
    speed: f32,
}

#[derive(Debug, PartialEq)]
enum Args {
    Run(Config),
    Help,
}

/// Parses process arguments. `Args::Help` is the help sentinel used to print
/// usage and exit before attempting to create a `Controller`.
fn parse_args<I: IntoIterator<Item = String>>(args: I) -> Result<Args, String> {
    let mut args = args.into_iter();
    let _program = args.next();

    let mut speed = 1.0;

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => return Ok(Args::Help),
            "--speed" | "-s" => {
                let value = args
                    .next()
                    .ok_or_else(|| format!("missing value for {arg}"))?;
                speed = parse_speed(&value)?;
            }
            _ if arg.starts_with("--speed=") => {
                let value = &arg["--speed=".len()..];
                speed = parse_speed(value)?;
            }
            _ => return Err(format!("unexpected argument '{arg}'")),
        }
    }

    Ok(Args::Run(Config { speed }))
}

fn parse_speed(value: &str) -> Result<f32, String> {
    let speed = value
        .parse::<f32>()
        .map_err(|_| format!("invalid value '{value}' for --speed: not a number"))?;

    if speed.is_finite() && speed > 0.0 {
        Ok(speed)
    } else {
        Err(format!(
            "--speed must be a positive finite number, got {value}"
        ))
    }
}

async fn scan_circle(controller: &mut Controller) -> Result<u32, Box<dyn Error + Send + Sync>> {
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
        total_retries += controller.set_look(azimuth, elevation).await?;
    }

    // Return to center position
    total_retries += controller.set_look(center_azimuth, center_elevation).await?;

    Ok(total_retries)
}

async fn scan(controller: &mut Controller) -> Result<u32, Box<dyn Error + Send + Sync>> {
    let mut total_retries = 0;

    for i in 0..=15 {
        total_retries += controller.set_look(90.0 - i as f32 * 10.0, 0.0).await?;
    }
    for i in 1..=6 {
        total_retries += controller.set_look(-60.0 + i as f32 * 10.0, 0.0).await?;
    }

    for i in 0..=25 {
        total_retries += controller.set_look(0.0, -125.0 + i as f32 * 10.0).await?;
    }

    total_retries += controller.set_look(0.0, 0.0).await?;
    Ok(total_retries)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let config = match parse_args(std::env::args()) {
        Ok(Args::Run(config)) => config,
        Ok(Args::Help) => {
            print!("{USAGE}");
            return Ok(());
        }
        Err(message) => {
            eprintln!("error: {message}");
            eprint!("{USAGE}");
            std::process::exit(2);
        }
    };

    let mut controller = Controller::new().await?;

    if config.speed != 1.0 {
        controller.set_speed_multiplier(config.speed)?;
    }
    println!("Speed multiplier: {}", controller.speed_multiplier());

    if let Ok(voltage) = controller.get_battery_voltage().await {
        println!("Battery voltage: {:.2}V", voltage);
    }

    let mut retries = 0;

    controller.set_look(90.0, 0.0).await?;
    controller.set_multiple_positions(&[(Servo::ClawGrip, -100.0)]).await?;
    controller.set_multiple_positions(&[(Servo::ClawTwist, 0.0)]).await?;
    controller.set_look(0.0, 0.0).await?;


    retries += scan_circle(&mut controller).await?;
    retries += controller.set_look(-60.0, -125.0).await?;
    retries += controller.set_look(60.0, 125.0).await?;
    retries += scan(&mut controller).await?;
    retries += controller.set_look(0.0, -125.0).await?;
    retries += controller.set_look(0.0, 125.0).await?;
    retries += controller.set_look(0.0, 0.0).await?;

    println!("Total retries: {}", retries);

    if let Ok(positions) = controller.get_positions(&[
        Servo::WristTilt,
        Servo::ElbowTilt,
        Servo::ShoulderTilt,
        Servo::BaseSpin,
        Servo::ClawGrip,
        Servo::ClawTwist,
    ]).await {
        for (servo, position) in positions {
            println!("{:?} position: {:.1} degrees", servo, position);
        }
    } else {
        println!("Failed to get positions for servos");
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(values: Vec<&str>) -> Vec<String> {
        values.into_iter().map(String::from).collect()
    }

    #[test]
    fn parse_args_defaults_speed_to_one() {
        assert_eq!(
            parse_args(args(vec!["prog"])),
            Ok(Args::Run(Config { speed: 1.0 }))
        );
    }

    #[test]
    fn parse_args_accepts_long_speed_with_separate_value() {
        assert_eq!(
            parse_args(args(vec!["prog", "--speed", "2.0"])),
            Ok(Args::Run(Config { speed: 2.0 }))
        );
    }

    #[test]
    fn parse_args_accepts_long_speed_with_equals_value() {
        assert_eq!(
            parse_args(args(vec!["prog", "--speed=0.5"])),
            Ok(Args::Run(Config { speed: 0.5 }))
        );
    }

    #[test]
    fn parse_args_accepts_short_speed_with_separate_value() {
        assert_eq!(
            parse_args(args(vec!["prog", "-s", "3.0"])),
            Ok(Args::Run(Config { speed: 3.0 }))
        );
    }

    #[test]
    fn parse_args_accepts_long_help() {
        assert_eq!(parse_args(args(vec!["prog", "--help"])), Ok(Args::Help));
    }

    #[test]
    fn parse_args_accepts_short_help() {
        assert_eq!(parse_args(args(vec!["prog", "-h"])), Ok(Args::Help));
    }

    #[test]
    fn parse_args_rejects_non_numeric_speed() {
        assert!(parse_args(args(vec!["prog", "--speed", "abc"])).is_err());
    }

    #[test]
    fn parse_args_rejects_zero_speed() {
        assert!(parse_args(args(vec!["prog", "--speed", "0"])).is_err());
    }

    #[test]
    fn parse_args_rejects_negative_speed() {
        assert!(parse_args(args(vec!["prog", "--speed", "-1"])).is_err());
    }

    #[test]
    fn parse_args_rejects_nan_speed() {
        assert!(parse_args(args(vec!["prog", "--speed", "nan"])).is_err());
    }

    #[test]
    fn parse_args_rejects_unknown_flag() {
        assert!(parse_args(args(vec!["prog", "--bogus"])).is_err());
    }

    #[test]
    fn parse_args_rejects_extra_positional_arg() {
        assert!(parse_args(args(vec!["prog", "extra"])).is_err());
    }

    #[test]
    fn parse_args_rejects_missing_speed_value() {
        assert!(parse_args(args(vec!["prog", "--speed"])).is_err());
    }
}
