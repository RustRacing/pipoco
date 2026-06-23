#![cfg(feature = "transport-can")]

use bxcan::{Data, ExtendedId, Frame, Id, StandardId};
use ecu_transport::{CanDevice, CanTransport};
use stm32f4xx_hal::can::Can1;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Stm32CanDeviceError {
    InvalidArbitrationId,
    Transmit,
}

pub struct Stm32CanDevice {
    can: bxcan::Can<Can1>,
}

pub type BoardCanTransport = CanTransport<Stm32CanDevice>;

impl Stm32CanDevice {
    pub fn new(can: bxcan::Can<Can1>) -> Self {
        Self { can }
    }
}

pub fn new_transport(can: bxcan::Can<Can1>) -> BoardCanTransport {
    CanTransport::new(Stm32CanDevice::new(can))
}

impl CanDevice for Stm32CanDevice {
    type Error = Stm32CanDeviceError;

    fn tx_ready(&self) -> bool {
        self.can.is_transmitter_idle()
    }

    fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error> {
        let data = Data::new(data).ok_or(Stm32CanDeviceError::Transmit)?;
        let frame = if id <= 0x7ff {
            Frame::new_data(
                StandardId::new(id as u16).ok_or(Stm32CanDeviceError::InvalidArbitrationId)?,
                data,
            )
        } else {
            Frame::new_data(
                ExtendedId::new(id).ok_or(Stm32CanDeviceError::InvalidArbitrationId)?,
                data,
            )
        };
        self.can
            .transmit(&frame)
            .map(|_| ())
            .map_err(|_| Stm32CanDeviceError::Transmit)
    }

    fn try_receive(&mut self, buf: &mut [u8]) -> Option<(u32, usize)> {
        let frame = self.can.receive().ok()?;
        let id = match frame.id() {
            Id::Standard(id) => id.as_raw() as u32,
            Id::Extended(id) => id.as_raw(),
        };
        let data = frame.data()?;
        let len = core::cmp::min(data.len(), buf.len());
        buf[..len].copy_from_slice(&data[..len]);
        Some((id, len))
    }
}
