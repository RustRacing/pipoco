//! CAN transport for ECU messages
//!
//! Minimal, generic CAN transport that serializes `Message` into a single CAN
//! frame payload. Large messages (e.g., tables) are rejected with
//! `TransportError::MessageTooLarge`. Future work: segmented multi-frame.

use super::{Message, Transport, TransportError, TransportStats};
use postcard::{from_bytes, to_slice};

/// Maximum CAN data payload size (bytes)
/// If the `transport-can-fd` feature is enabled, allow up to 64 bytes.
#[cfg(feature = "transport-can-fd")]
const MAX_CAN_DATA: usize = 64;
#[cfg(not(feature = "transport-can-fd"))]
const MAX_CAN_DATA: usize = 8;

/// Generic CAN device trait
///
/// Target crates should implement this for their CAN peripheral.
pub trait CanDevice {
    type Error;
    /// Returns true when the device is ready to send
    fn tx_ready(&self) -> bool;
    /// Send one CAN frame (standard 11-bit arbitration ID)
    fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error>;
    /// Try receive one CAN frame into the provided buffer
    /// Returns (id, len) on success
    fn try_receive(&mut self, buf: &mut [u8]) -> Option<(u32, usize)>;
}

/// CAN transport wrapper implementing the generic `Transport` trait
pub struct CanTransport<D: CanDevice> {
    dev: D,
    stats: TransportStats,
    // reassembly state (single in-flight)
    re_tid: Option<u8>,
    re_expected: usize,
    re_len: usize,
    re_crc16: u16,
    re_buf: [u8; REASM_BUF],
    // tx sequence
    tx_tid: u8,
}

// Max size we will accept for a segmented message (e.g., full table ~528B)
const REASM_BUF: usize = 1024;

impl<D: CanDevice> CanTransport<D> {
    pub fn new(dev: D) -> Self {
        Self {
            dev,
            stats: TransportStats::default(),
            re_tid: None,
            re_expected: 0,
            re_len: 0,
            re_crc16: 0,
            re_buf: [0; REASM_BUF],
            tx_tid: 0,
        }
    }

    /// Map message to CAN arbitration ID (11-bit)
    fn id_for(msg: &Message) -> u32 {
        match msg {
            Message::Error { .. } => 0x080,
            Message::CmdReset { .. } => 0x050,
            Message::CmdEngineControl { .. } => 0x060,
            Message::TriggerTiming { .. } => 0x100,
            Message::SensorData { .. } => 0x200,
            Message::IpwTable { .. } => 0x300,
            Message::IgnitionTable { .. } => 0x310,
            Message::EngineConfig { .. } => 0x400,
            Message::InjectorConfig { .. } => 0x410,
            Message::CmdCalibrate { .. } => 0x420,
            Message::Heartbeat { .. } => 0x700,
        }
    }
}

impl<D: CanDevice> Transport for CanTransport<D> {
    fn send(&mut self, message: &Message) -> Result<(), TransportError> {
        if !self.dev.tx_ready() {
            self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
            return Err(TransportError::NotReady);
        }

        // Serialize into a temporary buffer
        let mut big = [0u8; REASM_BUF];
        let encoded = match to_slice(message, &mut big[..]) {
            Ok(enc) => enc,
            Err(_) => {
                self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                return Err(TransportError::SerializationFailed);
            }
        };

        // Fast path: fits in one frame
        if encoded.len() < MAX_CAN_DATA {
            let id = Self::id_for(message);
            let mut frame = [0u8; MAX_CAN_DATA];
            let tid = self.tx_tid & 0x3f;
            self.tx_tid = self.tx_tid.wrapping_add(1);
            frame[0] = tid; // single frame marker is implicit (kind=0)
            frame[1..1 + encoded.len()].copy_from_slice(encoded);
            let data = &frame[..1 + encoded.len()];
            match self.dev.send(id, data) {
                Ok(()) => {
                    self.stats.tx_count = self.stats.tx_count.saturating_add(1);
                    Ok(())
                }
                Err(_) => {
                    self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                    Err(TransportError::HardwareError)
                }
            }
        } else {
            // Segmentation: send START with total_len + crc16, then CONT frames, then END
            let total_len = encoded.len();
            if total_len > REASM_BUF {
                self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                return Err(TransportError::MessageTooLarge);
            }
            let crc = crc16_ccitt(encoded);
            let id = Self::id_for(message);
            let tid = self.tx_tid & 0x3f;
            self.tx_tid = self.tx_tid.wrapping_add(1);

            // Start frame
            let mut frame = [0u8; MAX_CAN_DATA];
            frame[0] = (0b01 << 6) | tid; // start
            frame[1] = (total_len & 0xff) as u8;
            frame[2] = ((total_len >> 8) & 0xff) as u8;
            frame[3] = (crc & 0xff) as u8;
            frame[4] = (crc >> 8) as u8;
            let mut sent = 0usize;
            let chunk0 = MAX_CAN_DATA.saturating_sub(5);
            let c0 = core::cmp::min(chunk0, total_len);
            frame[5..5 + c0].copy_from_slice(&encoded[..c0]);
            match self.dev.send(id, &frame[..5 + c0]) {
                Ok(()) => {}
                Err(_) => {
                    self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                    return Err(TransportError::HardwareError);
                }
            }
            sent += c0;

            // Continuation frames
            while sent < total_len {
                let mut fr = [0u8; MAX_CAN_DATA];
                // end if this is last segment
                let remaining = total_len - sent;
                let is_end = remaining <= (MAX_CAN_DATA - 1);
                fr[0] = ((if is_end { 0b11 } else { 0b10 }) << 6) | tid;
                let take = core::cmp::min(remaining, MAX_CAN_DATA - 1);
                fr[1..1 + take].copy_from_slice(&encoded[sent..sent + take]);
                match self.dev.send(id, &fr[..1 + take]) {
                    Ok(()) => {}
                    Err(_) => {
                        self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                        return Err(TransportError::HardwareError);
                    }
                }
                sent += take;
            }
            self.stats.tx_count = self.stats.tx_count.saturating_add(1);
            Ok(())
        }
    }

    fn try_receive(&mut self) -> Option<Message> {
        let mut f = [0u8; MAX_CAN_DATA];
        let (_id, len) = self.dev.try_receive(&mut f[..])?;
        if len == 0 {
            return None;
        }
        let flags = f[0];
        let kind = flags >> 6;
        let tid = flags & 0x3f;

        match kind {
            0 => {
                // single
                if let Ok(msg) = from_bytes::<Message>(&f[1..len]) {
                    self.stats.rx_count = self.stats.rx_count.saturating_add(1);
                    Some(msg)
                } else {
                    self.stats.rx_errors = self.stats.rx_errors.saturating_add(1);
                    None
                }
            }
            1 => {
                // start: reset buffer
                if len < 5 {
                    self.stats.rx_errors = self.stats.rx_errors.saturating_add(1);
                    return None;
                }
                let total = (f[1] as usize) | ((f[2] as usize) << 8);
                let crc = (f[3] as u16) | ((f[4] as u16) << 8);
                if total > REASM_BUF {
                    self.re_tid = None;
                    return None;
                }
                self.re_tid = Some(tid);
                self.re_expected = total;
                self.re_len = 0;
                self.re_crc16 = crc;
                let copy = len.saturating_sub(5);
                if copy > 0 {
                    let take = core::cmp::min(copy, total);
                    self.re_buf[..take].copy_from_slice(&f[5..5 + take]);
                    self.re_len = take;
                }
                None
            }
            2 | 3 => {
                // cont or end
                if self.re_tid != Some(tid) {
                    return None;
                }
                let remaining = self.re_expected.saturating_sub(self.re_len);
                if remaining == 0 {
                    return None;
                }
                let data = &f[1..len];
                let take = core::cmp::min(remaining, data.len());
                self.re_buf[self.re_len..self.re_len + take].copy_from_slice(&data[..take]);
                self.re_len += take;
                if kind == 3 {
                    if self.re_len == self.re_expected
                        && crc16_ccitt(&self.re_buf[..self.re_len]) == self.re_crc16
                    {
                        let out = from_bytes::<Message>(&self.re_buf[..self.re_len]).ok();
                        self.re_tid = None;
                        if out.is_some() {
                            self.stats.rx_count = self.stats.rx_count.saturating_add(1);
                        }
                        out
                    } else {
                        self.re_tid = None;
                        self.stats.rx_errors = self.stats.rx_errors.saturating_add(1);
                        None
                    }
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn poll(&mut self) {}

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn stats(&self) -> TransportStats {
        self.stats
    }

    fn is_ready(&self) -> bool {
        self.dev.tx_ready()
    }
}

// CRC16-CCITT (0x1021), initial 0xFFFF
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::Message;

    struct MockCan {
        // simple single-slot buffers for TX/RX
        rx_id: Option<u32>,
        rx_data: [u8; MAX_CAN_DATA],
        rx_len: usize,
        ready: bool,
    }

    impl MockCan {
        fn new() -> Self {
            Self {
                rx_id: None,
                rx_data: [0; MAX_CAN_DATA],
                rx_len: 0,
                ready: true,
            }
        }
        fn loopback(&mut self, id: u32, data: &[u8]) {
            self.rx_id = Some(id);
            self.rx_data[..data.len()].copy_from_slice(data);
            self.rx_len = data.len();
        }
    }

    impl CanDevice for MockCan {
        type Error = ();
        fn tx_ready(&self) -> bool {
            self.ready
        }
        fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error> {
            self.loopback(id, data);
            Ok(())
        }
        fn try_receive(&mut self, buf: &mut [u8]) -> Option<(u32, usize)> {
            if let Some(id) = self.rx_id.take() {
                let len = self.rx_len;
                buf[..len].copy_from_slice(&self.rx_data[..len]);
                Some((id, len))
            } else {
                None
            }
        }
    }

    #[test]
    fn test_can_send_receive_heartbeat() {
        let mock = MockCan::new();
        let mut can = CanTransport::new(mock);
        let msg = Message::Heartbeat {
            node_id: 1,
            uptime_seconds: 10,
            status: 0,
            error_count: 0,
            cpu_usage: 5,
        };
        let _ = can.send(&msg);
        // Transport loopback: MockCan echoes TX to RX
        let out = can.try_receive();
        assert!(matches!(out, Some(Message::Heartbeat { .. })));
    }

    #[test]
    fn test_can_message_segmented_ok() {
        let mock = MockCan::new();
        let mut can = CanTransport::new(mock);
        let msg = Message::IpwTable {
            version: 1,
            data: [[1000; 16]; 16],
            crc32: 0,
        };
        let res = can.send(&msg);
        assert!(res.is_ok());
    }
}
