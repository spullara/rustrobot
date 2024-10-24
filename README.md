# RustRobot: XArm Servo Controller

RustRobot is a Rust-based controller for the XArm robotic arm, providing precise control of servo motors and various arm movements. This project has evolved significantly, incorporating new features and improvements.

## Features

- Advanced servo motor control
- Precise joint angle calculations
- Customizable scanning movements
- Real-time battery voltage monitoring
- Bluetooth Low Energy (BLE) connectivity
- [Interactive visualization](https://claude.site/artifacts/4e26de69-b6ae-40b5-85c5-cbe835629ca2)

## Requirements

- Rust (edition 2021)
- HID-compatible XArm device
- Bluetooth Low Energy capable hardware (for BLE features)

## Installation

1. Clone this repository:

    ```bash
    git clone https://github.com/spullara/rustrobot.git
    cd rustrobot
    ```

2. Build the project:

    ```bash
    cargo build --release
    ```

## Usage

Run the main program:

```bash
cargo run --release
```

This will initialize the XArm controller and execute a series of predefined movements.

## Configuration

The `xarm_config.json` file allows you to customize various parameters of the XArm controller. Modify this file to adjust servo limits, movement speeds, and other settings.

## Project Structure

- `src/main.rs`: Entry point and main control logic
- `src/controller.rs`: XArm controller implementation
- `src/transport.rs`: Communication layer for HID and BLE
- `src/constants.rs`: Constant values and configurations
- `src/types.rs`: Custom type definitions
- `src/lib.rs`: Library interface for external use
- `Cargo.toml`: Project dependencies and metadata

## Key Dependencies

- `btleplug`: For Bluetooth Low Energy functionality
- `hidapi`: For communicating with the HID device
- `serde`: For JSON serialization/deserialization
- `tokio`: For asynchronous programming

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

MIT License

## Acknowledgements

This project builds upon the work of the open-source community and various libraries that make robotic control possible in Rust.
