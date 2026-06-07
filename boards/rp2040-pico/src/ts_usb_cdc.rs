use usb_device::{bus::UsbBus, prelude::*, UsbError};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CdcSerialDirection {
    Read,
    Write,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CdcSerialFault {
    ReadError,
    WriteError,
    ShortWrite { requested: usize, written: usize },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CdcSerialStats {
    pub read_errors: u32,
    pub write_errors: u32,
    pub short_writes: u32,
    pub last_fault: Option<CdcSerialFault>,
    pub last_transfer: Option<(CdcSerialDirection, usize)>,
}

impl CdcSerialStats {
    pub const fn new() -> Self {
        Self {
            read_errors: 0,
            write_errors: 0,
            short_writes: 0,
            last_fault: None,
            last_transfer: None,
        }
    }

    fn record_read(&mut self, result: Result<usize, usb_device::UsbError>) -> usize {
        match result {
            Ok(n) => {
                self.last_transfer = Some((CdcSerialDirection::Read, n));
                n
            }
            Err(UsbError::WouldBlock) => 0,
            Err(_) => {
                self.read_errors = self.read_errors.saturating_add(1);
                self.last_fault = Some(CdcSerialFault::ReadError);
                0
            }
        }
    }

    fn record_write(
        &mut self,
        requested: usize,
        result: Result<usize, usb_device::UsbError>,
    ) -> usize {
        match result {
            Ok(n) => {
                self.last_transfer = Some((CdcSerialDirection::Write, n));
                if n < requested {
                    self.short_writes = self.short_writes.saturating_add(1);
                    self.last_fault = Some(CdcSerialFault::ShortWrite {
                        requested,
                        written: n,
                    });
                }
                n
            }
            Err(UsbError::WouldBlock) => 0,
            Err(_) => {
                self.write_errors = self.write_errors.saturating_add(1);
                self.last_fault = Some(CdcSerialFault::WriteError);
                0
            }
        }
    }
}

impl Default for CdcSerialStats {
    fn default() -> Self {
        Self::new()
    }
}

/// RP2040-local USB CDC Serial adapter using the HAL-compatible usb-device 0.2 line.
#[allow(dead_code)]
pub struct CdcSerial<'a, B: UsbBus> {
    pub serial: usbd_serial::SerialPort<'a, B>,
    pub dev: UsbDevice<'a, B>,
    pub stats: CdcSerialStats,
}

impl<'a, B: UsbBus> ecu_ts::serial::SerialPort for CdcSerial<'a, B> {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let _ = self.dev.poll(&mut [&mut self.serial]);
        self.stats.record_read(self.serial.read(buf))
    }

    fn write(&mut self, buf: &[u8]) -> usize {
        let _ = self.dev.poll(&mut [&mut self.serial]);
        self.stats.record_write(buf.len(), self.serial.write(buf))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stats_record_read_would_block_is_not_a_fault() {
        let mut stats = CdcSerialStats::new();

        assert_eq!(stats.record_read(Err(UsbError::WouldBlock)), 0);
        assert_eq!(stats.read_errors, 0);
        assert_eq!(stats.last_fault, None);
    }

    #[test]
    fn stats_record_read_error_without_losing_observability() {
        let mut stats = CdcSerialStats::new();

        assert_eq!(stats.record_read(Err(UsbError::BufferOverflow)), 0);
        assert_eq!(stats.read_errors, 1);
        assert_eq!(stats.last_fault, Some(CdcSerialFault::ReadError));
    }

    #[test]
    fn stats_record_short_write_as_fault() {
        let mut stats = CdcSerialStats::new();

        assert_eq!(stats.record_write(8, Ok(3)), 3);
        assert_eq!(stats.short_writes, 1);
        assert_eq!(
            stats.last_fault,
            Some(CdcSerialFault::ShortWrite {
                requested: 8,
                written: 3,
            })
        );
    }

    #[test]
    fn stats_record_write_would_block_is_not_a_fault() {
        let mut stats = CdcSerialStats::new();

        assert_eq!(stats.record_write(8, Err(UsbError::WouldBlock)), 0);
        assert_eq!(stats.write_errors, 0);
        assert_eq!(stats.last_fault, None);
    }

    #[test]
    fn stats_record_write_error() {
        let mut stats = CdcSerialStats::new();

        assert_eq!(stats.record_write(8, Err(UsbError::BufferOverflow)), 0);
        assert_eq!(stats.write_errors, 1);
        assert_eq!(stats.last_fault, Some(CdcSerialFault::WriteError));
    }
}
