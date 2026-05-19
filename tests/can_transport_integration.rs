//! Integration test for CAN transport with segmentation/reassembly

#[cfg(feature = "transport-can")]
use ecu_core::transport::can::{CanDevice, CanTransport};
#[cfg(feature = "transport-can")]
use ecu_core::{transport::Message, Transport};

#[cfg(feature = "transport-can")]
struct SharedBus {
    frames: std::collections::VecDeque<(u32, [u8; 64], usize)>,
}

#[cfg(feature = "transport-can")]
impl SharedBus {
    fn new() -> Self {
        Self {
            frames: std::collections::VecDeque::new(),
        }
    }
}

#[cfg(feature = "transport-can")]
#[derive(Clone)]
struct MockCan {
    bus: std::rc::Rc<std::cell::RefCell<SharedBus>>,
    ready: bool,
}

#[cfg(feature = "transport-can")]
impl MockCan {
    fn new(bus: std::rc::Rc<std::cell::RefCell<SharedBus>>) -> Self {
        Self { bus, ready: true }
    }
}

#[cfg(feature = "transport-can")]
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

#[cfg(not(feature = "transport-can"))]
#[test]
fn can_segmented_table_roundtrip() {
    // No CAN transport; nothing to validate in this configuration
}

#[cfg(feature = "transport-can")]
#[test]
fn can_segmented_table_roundtrip() {
    let bus = std::rc::Rc::new(std::cell::RefCell::new(SharedBus::new()));
    let dev_tx = MockCan::new(bus.clone());
    let dev_rx = MockCan::new(bus.clone());
    let mut tx = CanTransport::new(dev_tx);
    let mut rx = CanTransport::new(dev_rx);

    // Create a large IpwTable message that must be segmented
    let mut table = [[0u16; 16]; 16];
    for (i, row) in table.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (1000 + i * 16 + j) as u16;
        }
    }
    let msg = Message::IpwTable {
        version: 7,
        data: table,
        crc32: 0xDEADBEEF,
    };
    assert!(msg.estimated_size() > 500);

    // Send
    tx.send(&msg).expect("send");

    // Drain frames through RX try_receive until message reassembles
    let mut got = None;
    for _ in 0..1024 {
        if let Some(m) = rx.try_receive() {
            got = Some(m);
            break;
        }
    }
    let out = got.expect("should reassemble message");
    if let Message::IpwTable {
        version,
        data,
        crc32,
    } = out
    {
        assert_eq!(version, 7);
        assert_eq!(crc32, 0xDEADBEEF);
        assert_eq!(data[0][0], 1000);
        assert_eq!(data[15][15], 1000 + 255);
    } else {
        panic!("wrong message type");
    }
}
