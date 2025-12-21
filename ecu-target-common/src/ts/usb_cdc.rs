use usb_device::{bus::UsbBus, prelude::*};

/// Generic USB CDC Serial adapter implementing ecu_core TS SerialPort
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
