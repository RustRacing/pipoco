//! CAN transport for ECU messages.

use crate::message::MAX_ENCODED_MESSAGE_SIZE;
use crate::{Message, Transport, TransportError, TransportStats};
use postcard::{from_bytes, to_slice};

#[cfg(feature = "transport-can-fd")]
const MAX_CAN_DATA: usize = 64;
#[cfg(not(feature = "transport-can-fd"))]
const MAX_CAN_DATA: usize = 8;

const REASM_BUF: usize = MAX_ENCODED_MESSAGE_SIZE;

/// Generic CAN device trait.
pub trait CanDevice {
    type Error;

    /// Returns true when the device is ready to send.
    fn tx_ready(&self) -> bool;

    /// Send one CAN frame.
    fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Try receive one CAN frame into the provided buffer.
    fn try_receive(&mut self, buf: &mut [u8]) -> Option<(u32, usize)>;
}

/// CAN transport wrapper implementing the generic `Transport` trait.
pub struct CanTransport<D: CanDevice> {
    dev: D,
    stats: TransportStats,
    re_tid: Option<u8>,
    re_id: u32,
    re_expected: usize,
    re_len: usize,
    re_crc16: u16,
    re_buf: [u8; REASM_BUF],
    tx_tid: u8,
    last_rx_error: Option<TransportError>,
}

impl<D: CanDevice> CanTransport<D> {
    pub fn new(dev: D) -> Self {
        Self {
            dev,
            stats: TransportStats::default(),
            re_tid: None,
            re_id: 0,
            re_expected: 0,
            re_len: 0,
            re_crc16: 0,
            re_buf: [0; REASM_BUF],
            tx_tid: 0,
            last_rx_error: None,
        }
    }

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

    /// Last receive-side error observed by the lossy `try_receive` API.
    pub fn last_receive_error(&self) -> Option<TransportError> {
        self.last_rx_error
    }

    fn clear_reassembly(&mut self) {
        self.re_tid = None;
        self.re_id = 0;
        self.re_expected = 0;
        self.re_len = 0;
        self.re_crc16 = 0;
    }

    fn rx_error(&mut self, error: TransportError) -> Option<Message> {
        self.stats.rx_errors = self.stats.rx_errors.saturating_add(1);
        self.last_rx_error = Some(error);
        self.clear_reassembly();
        None
    }
}

impl<D: CanDevice> Transport for CanTransport<D> {
    fn send(&mut self, message: &Message) -> Result<(), TransportError> {
        if !self.dev.tx_ready() {
            self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
            return Err(TransportError::NotReady);
        }

        let mut big = [0u8; REASM_BUF];
        let encoded = match to_slice(message, &mut big[..]) {
            Ok(enc) => enc,
            Err(_) => {
                self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                return Err(TransportError::SerializationFailed);
            }
        };

        if encoded.len() <= MAX_CAN_DATA.saturating_sub(1) {
            let id = Self::id_for(message);
            let mut frame = [0u8; MAX_CAN_DATA];
            let tid = self.tx_tid & 0x3f;
            self.tx_tid = self.tx_tid.wrapping_add(1);
            frame[0] = tid;
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
            let total_len = encoded.len();
            if total_len > REASM_BUF {
                self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                return Err(TransportError::MessageTooLarge);
            }

            let crc = crc16_ccitt(encoded);
            let id = Self::id_for(message);
            let tid = self.tx_tid & 0x3f;
            self.tx_tid = self.tx_tid.wrapping_add(1);

            let mut frame = [0u8; MAX_CAN_DATA];
            frame[0] = (0b01 << 6) | tid;
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

            while sent < total_len {
                let mut fr = [0u8; MAX_CAN_DATA];
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
        let (id, len) = self.dev.try_receive(&mut f[..])?;
        if len == 0 {
            return self.rx_error(TransportError::DeserializationFailed);
        }

        let flags = f[0];
        let kind = flags >> 6;
        let tid = flags & 0x3f;

        match kind {
            0 => {
                if self.re_tid.is_some() {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                if let Ok(msg) = from_bytes::<Message>(&f[1..len]) {
                    self.stats.rx_count = self.stats.rx_count.saturating_add(1);
                    self.last_rx_error = None;
                    Some(msg)
                } else {
                    self.rx_error(TransportError::DeserializationFailed)
                }
            }
            1 => {
                if self.re_tid.is_some() {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                if len < 5 {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                let total = (f[1] as usize) | ((f[2] as usize) << 8);
                let crc = (f[3] as u16) | ((f[4] as u16) << 8);
                let copy = len.saturating_sub(5);
                if total == 0 || total > REASM_BUF || copy > total {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                self.re_tid = Some(tid);
                self.re_id = id;
                self.re_expected = total;
                self.re_len = 0;
                self.re_crc16 = crc;
                if copy > 0 {
                    self.re_buf[..copy].copy_from_slice(&f[5..5 + copy]);
                    self.re_len = copy;
                }
                None
            }
            2 | 3 => {
                if self.re_tid != Some(tid) || self.re_id != id {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                let remaining = self.re_expected.saturating_sub(self.re_len);
                if remaining == 0 {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                let data = &f[1..len];
                if data.is_empty() || data.len() > remaining {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                if kind == 2 && data.len() == remaining {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                if kind == 3 && data.len() != remaining {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                self.re_buf[self.re_len..self.re_len + data.len()].copy_from_slice(data);
                self.re_len += data.len();
                if kind == 3 {
                    if self.re_len == self.re_expected
                        && crc16_ccitt(&self.re_buf[..self.re_len]) == self.re_crc16
                    {
                        let out = from_bytes::<Message>(&self.re_buf[..self.re_len]).ok();
                        self.clear_reassembly();
                        if out.is_some() {
                            self.stats.rx_count = self.stats.rx_count.saturating_add(1);
                            self.last_rx_error = None;
                        } else {
                            self.stats.rx_errors = self.stats.rx_errors.saturating_add(1);
                            self.last_rx_error = Some(TransportError::DeserializationFailed);
                        }
                        out
                    } else {
                        self.rx_error(TransportError::DeserializationFailed)
                    }
                } else {
                    None
                }
            }
            _ => self.rx_error(TransportError::DeserializationFailed),
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
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    struct SharedBus {
        frames: VecDeque<(u32, [u8; 64], usize)>,
    }

    #[derive(Clone)]
    struct MockCan {
        bus: Rc<RefCell<SharedBus>>,
        ready: bool,
    }

    impl MockCan {
        fn new(bus: Rc<RefCell<SharedBus>>) -> Self {
            Self { bus, ready: true }
        }
    }

    impl CanDevice for MockCan {
        type Error = ();

        fn tx_ready(&self) -> bool {
            self.ready
        }

        fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error> {
            let mut arr = [0u8; 64];
            let len = core::cmp::min(64, data.len());
            arr[..len].copy_from_slice(&data[..len]);
            self.bus.borrow_mut().frames.push_back((id, arr, len));
            Ok(())
        }

        fn try_receive(&mut self, buf: &mut [u8]) -> Option<(u32, usize)> {
            let mut bus = self.bus.borrow_mut();
            let (id, data, len) = bus.frames.pop_front()?;
            buf[..len].copy_from_slice(&data[..len]);
            Some((id, len))
        }
    }

    fn table_message(version: u32) -> Message {
        let mut table = [[0u16; 16]; 16];
        for (i, row) in table.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (1000 + i * 16 + j) as u16;
            }
        }
        Message::IpwTable {
            version,
            data: table,
            crc32: 0xDEADBEEF,
        }
    }

    fn drain_receive<D: CanDevice>(rx: &mut CanTransport<D>, polls: usize) -> Option<Message> {
        let mut got = None;
        for _ in 0..polls {
            if let Some(msg) = rx.try_receive() {
                got = Some(msg);
                break;
            }
        }
        got
    }

    #[test]
    fn segmented_table_roundtrips() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus));

        let msg = table_message(7);

        tx.send(&msg).expect("send");
        assert_eq!(drain_receive(&mut rx, 1024), Some(msg));
        assert_eq!(rx.last_receive_error(), None);
    }

    #[test]
    fn classic_can_single_frame_boundary_uses_header_payload_capacity() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let msg = Message::CmdReset { target_node_id: 2 };

        tx.send(&msg).expect("send");
        let frames = &bus.borrow().frames;
        assert_eq!(frames.len(), 1);
        let (_, data, len) = frames.front().expect("frame");
        assert_eq!(data[0] >> 6, 0);
        assert!(*len <= MAX_CAN_DATA);
    }

    #[test]
    fn reassembly_rejects_mixed_frame_ids() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));
        let msg = table_message(8);

        tx.send(&msg).expect("send");
        {
            let mut bus = bus.borrow_mut();
            let second = bus.frames.get_mut(1).expect("continuation frame");
            second.0 ^= 0x001;
        }

        assert_eq!(drain_receive(&mut rx, 1024), None);
        assert_eq!(
            rx.last_receive_error(),
            Some(TransportError::DeserializationFailed)
        );
        assert!(rx.stats().rx_errors > 0);
    }

    #[test]
    fn reassembly_rejects_early_end_fragment() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));

        tx.send(&table_message(9)).expect("send");
        {
            let mut bus = bus.borrow_mut();
            let second = bus.frames.get_mut(1).expect("continuation frame");
            second.1[0] = (0b11 << 6) | (second.1[0] & 0x3f);
        }

        assert_eq!(drain_receive(&mut rx, 1024), None);
        assert_eq!(
            rx.last_receive_error(),
            Some(TransportError::DeserializationFailed)
        );
    }

    #[test]
    fn reassembly_recovers_after_invalid_sequence() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));
        let bad = table_message(10);
        let good = table_message(11);

        tx.send(&bad).expect("send bad");
        {
            let mut bus = bus.borrow_mut();
            let second = bus.frames.get_mut(1).expect("continuation frame");
            second.0 ^= 0x001;
        }
        assert_eq!(drain_receive(&mut rx, 1024), None);

        tx.send(&good).expect("send good");
        assert_eq!(drain_receive(&mut rx, 1024), Some(good));
        assert_eq!(rx.last_receive_error(), None);
    }
}
