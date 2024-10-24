use crate::constants::*;
use std::error::Error;
use std::fmt;
use std::sync::Arc;
use tokio::time::Duration;
use hidapi::HidApi;
use parking_lot::Mutex;  // Add this dependency to Cargo.toml
use btleplug::api::{Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType};
use btleplug::platform::{Manager, Peripheral};
use futures::stream::StreamExt;
use uuid::Uuid;

const SERVICE_UUID: Uuid = Uuid::from_u128(0x0000ffe000001000800000805f9b34fb);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x0000ffe100001000800000805f9b34fb);

#[derive(Debug)]
pub enum TransportError {
    InvalidResponse {
        expected_len: usize,
        actual_len: usize,
        raw_data: Vec<u8>,
    },
    DeviceError(String),
    NoDeviceFound,
}


impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransportError::InvalidResponse { expected_len, actual_len, raw_data } => {
                write!(f, "Invalid response data: expected length {} but got {}. Raw data: {:02x?}",
                       expected_len, actual_len, raw_data)
            }
            TransportError::DeviceError(msg) => write!(f, "Device error: {}", msg),
            TransportError::NoDeviceFound => write!(f, "No device found"),
        }
    }
}

impl Error for TransportError {}

pub enum Transport {
    Hid(Arc<Mutex<hidapi::HidDevice>>),  // Wrap HidDevice in a Mutex
    Bluetooth {
        device: Peripheral,
        characteristic: Characteristic,
    },
}

impl Transport {
    pub async fn new() -> Result<Self, Box<dyn Error + Send + Sync>> {
        match Self::try_hid().await {
            Ok(hid_device) => {
                println!("Connected via USB HID");
                Ok(Transport::Hid(Arc::new(Mutex::new(hid_device))))  // Wrap in Mutex
            }
            Err(e) => {
                println!("Failed to connect via USB HID: {}. Trying Bluetooth...", e);
                match Self::try_bluetooth().await {
                    Ok((device, characteristic)) => {
                        println!("Connected via Bluetooth");
                        Ok(Transport::Bluetooth {
                            device,
                            characteristic,
                        })
                    }
                    Err(e) => {
                        println!("Failed to connect via Bluetooth: {}", e);
                        Err(Box::new(TransportError::NoDeviceFound))
                    }
                }
            }
        }
    }

    async fn try_hid() -> Result<hidapi::HidDevice, Box<dyn Error + Send + Sync>> {
        tokio::task::spawn_blocking(move || -> Result<hidapi::HidDevice, Box<dyn Error + Send + Sync>> {
            let api = HidApi::new().map_err(|e| Box::new(e) as Box<dyn Error + Send + Sync>)?;
            api.open(VENDOR_ID, PRODUCT_ID)
                .map_err(|e| Box::new(e) as Box<dyn Error + Send + Sync>)
        }).await?
    }

    async fn try_bluetooth() -> Result<(Peripheral, Characteristic), Box<dyn Error + Send + Sync>> {
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let adapter = adapters.into_iter().next().ok_or("No Bluetooth adapter found")?;

        adapter.start_scan(ScanFilter::default()).await?;

        let mut events = adapter.events().await?;
        let scan_timeout = Duration::from_secs(5);

        println!("Scanning for xArm...");

        let mut found_device = None;
        while let Ok(Some(event)) = tokio::time::timeout(scan_timeout, events.next()).await {
            if let btleplug::api::CentralEvent::DeviceDiscovered(id) = event {
                let peripheral = adapter.peripheral(&id).await?;
                if let Ok(Some(properties)) = peripheral.properties().await {
                    if let Some(name) = &properties.local_name {
                        if name == "xArm" {
                            found_device = Some(peripheral);
                            break;
                        }
                    }
                }
            }
        }

        adapter.stop_scan().await?;

        let device = found_device.ok_or("xArm not found")?;
        device.connect().await?;
        device.discover_services().await?;

        let characteristic = device.characteristics()
            .into_iter()
            .find(|c| c.uuid == CHARACTERISTIC_UUID && c.service_uuid == SERVICE_UUID)
            .ok_or("Communication characteristic not found")?;

        if characteristic.properties.contains(CharPropFlags::NOTIFY) {
            device.subscribe(&characteristic).await?;
        }

        Ok((device, characteristic))
    }

    pub async fn send(&mut self, cmd: u8, data: &[u8]) -> Result<(), Box<dyn Error + Send + Sync>> {
        match self {
            Transport::Hid(device) => {
                let device = Arc::clone(device);
                let mut report_data = vec![0, SIGNATURE, SIGNATURE, (data.len() + 2) as u8, cmd];
                report_data.extend_from_slice(data);

                tokio::task::spawn_blocking(move || {
                    device.lock().write(&report_data)  // Use lock() to access the device
                }).await??;

                Ok(())
            }
            Transport::Bluetooth { device, characteristic } => {
                let mut report_data = vec![SIGNATURE, SIGNATURE, (data.len() + 2) as u8, cmd];
                report_data.extend_from_slice(data);
                device.write(characteristic, &report_data, WriteType::WithResponse).await?;
                Ok(())
            }
        }
    }

    pub async fn recv(&mut self, cmd: u8) -> Result<Vec<u8>, Box<dyn Error + Send + Sync>> {
        match self {
            Transport::Hid(device) => {
                let device = Arc::clone(device);
                let buf = tokio::task::spawn_blocking(move || -> Result<(Vec<u8>, usize), Box<dyn Error + Send + Sync>> {
                    let mut buf = [0u8; 64];
                    let res = device.lock().read_timeout(&mut buf, 1000)?;  // Use lock() to access the device
                    Ok((buf.to_vec(), res))
                }).await??;

                let (buf, res) = buf;
                if res < 4 {
                    return Err(Box::new(TransportError::InvalidResponse {
                        expected_len: 4,
                        actual_len: res,
                        raw_data: buf[..res].to_vec(),
                    }));
                }

                if buf[0] != SIGNATURE || buf[1] != SIGNATURE {
                    return Err(Box::new(TransportError::DeviceError(
                        format!("Invalid signature: {:02x} {:02x}", buf[0], buf[1])
                    )));
                }

                let length = buf[2] as usize;
                Ok(buf[4..4 + length].to_vec())
            }
            Transport::Bluetooth { device, characteristic } => {
                if characteristic.properties.contains(CharPropFlags::NOTIFY) {
                    let mut notifications = device.notifications().await?;
                    match tokio::time::timeout(Duration::from_secs(1), notifications.next()).await {
                        Ok(Some(data)) => {
                            let buf = data.value;
                            if buf.len() >= 4 && buf[0] == SIGNATURE && buf[1] == SIGNATURE {
                                Ok(buf[4..].to_vec())
                            } else {
                                Err(Box::new(TransportError::DeviceError("Invalid response format".into())))
                            }
                        }
                        _ => Err(Box::new(TransportError::DeviceError("No response received".into()))),
                    }
                } else {
                    let buf = device.read(characteristic).await?;
                    if buf.len() >= 4 && buf[0] == SIGNATURE && buf[1] == SIGNATURE {
                        Ok(buf[4..].to_vec())
                    } else {
                        Err(Box::new(TransportError::DeviceError("Invalid response format".into())))
                    }
                }
            }
        }
    }
}