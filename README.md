# XArm Servo Controller

This project provides a Rust-based controller for the XArm robotic arm, allowing precise control of servo motors and various arm movements.

## Features

- Control individual servo motors
- Get and set servo positions
- Calculate joint angles for target elevations
- Perform scanning movements
- Read battery voltage
- Interactive visualization (separate component)

## Requirements

- Rust (edition 2021)
- HID-compatible XArm device

## Installation

1. Clone this repository:

```bash
git clone https://github.com/spullara/rustrobot.git
cd xarm-servo-controller
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
This will perform a series of movements and display the current positions of all servos.

## Project Structure

- `src/main.rs`: Main controller logic and servo control functions
- `Cargo.toml`: Project dependencies and metadata
- `robot-arm-interactive.tsx`: React component for interactive arm visualization (separate from main Rust project)

## Dependencies

- `hidapi`: For communicating with the HID device
- `strum` and `strum_macros`: For enum iteration

## License

[Insert your chosen license here]

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgements

This project uses the HID protocol to communicate with the XArm device and is inspired by similar projects in other languages.

