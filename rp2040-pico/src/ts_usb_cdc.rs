use usb_device::{bus::UsbBus, prelude::*};

/// RP2040-local USB CDC Serial adapter using the HAL-compatible usb-device 0.2 line.
pub struct CdcSerial<'a, B: UsbBus> {
    pub serial: usbd_serial::SerialPort<'a, B>,
    pub dev: UsbDevice<'a, B>,
}

impl<'a, B: UsbBus> ecu_core::ts::serial::SerialPort for CdcSerial<'a, B> {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let _ = self.dev.poll(&mut [&mut self.serial]);
        self.serial.read(buf).unwrap_or_default()
    }

    fn write(&mut self, buf: &[u8]) -> usize {
        let _ = self.dev.poll(&mut [&mut self.serial]);
        self.serial.write(buf).unwrap_or_default()
    }
}
