use hidapi::HidApi;
use std::error::Error;

const VENDOR_ID: u16 = 0x0483;
const PRODUCT_ID: u16 = 0x5750;
const SIGNATURE: u8 = 0x55;
const CMD_GET_BATTERY_VOLTAGE: u8 = 0x0f;

struct Controller {
    device: hidapi::HidDevice,
}

impl Controller {
    fn new() -> Result<Self, Box<dyn Error>> {
        let api = HidApi::new()?;
        let device = api.open(VENDOR_ID, PRODUCT_ID)?;
        Ok(Controller { device })
    }

    fn _send(&mut self, cmd: u8, data: &[u8]) -> Result<(), Box<dyn Error>> {
        let mut report_data = vec![0, SIGNATURE, SIGNATURE, (data.len() + 2) as u8, cmd];
        report_data.extend_from_slice(data);
        self.device.write(&report_data)?;
        Ok(())
    }

    fn _recv(&mut self, cmd: u8) -> Result<Vec<u8>, Box<dyn Error>> {
        let mut buf = [0u8; 64];
        let res = self.device.read_timeout(&mut buf, 1000)?;
        if res >= 4 && buf[0] == SIGNATURE && buf[1] == SIGNATURE && buf[3] == cmd {
            let length = buf[2] as usize;
            Ok(buf[4..4 + length].to_vec())
        } else {
            Err("Invalid response".into())
        }
    }

    fn get_battery_voltage(&mut self) -> Result<f32, Box<dyn Error>> {
        self._send(CMD_GET_BATTERY_VOLTAGE, &[])?;
        let data = self._recv(CMD_GET_BATTERY_VOLTAGE)?;
        if data.len() >= 2 {
            Ok(((data[1] as u16 * 256 + data[0] as u16) as f32) / 1000.0)
        } else {
            Err("Invalid battery voltage data".into())
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut controller = Controller::new()?;
    match controller.get_battery_voltage() {
        Ok(voltage) => println!("Battery voltage: {:.2}V", voltage),
        Err(e) => eprintln!("Error getting battery voltage: {}", e),
    }
    Ok(())
}