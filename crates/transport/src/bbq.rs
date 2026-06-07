//! BBQueue-based transport for same-chip communication.

use crate::message::MAX_ENCODED_MESSAGE_SIZE;
use crate::{Message, Transport, TransportError, TransportStats};
use bbqueue::{BBBuffer, Consumer, Producer};
use postcard::{from_bytes, to_slice};

const QUEUE_SIZE: usize = 2048;

/// BBQueue transport for same-chip communication.
pub struct BbqTransport {
    tx_producer: Producer<'static, QUEUE_SIZE>,
    rx_consumer: Consumer<'static, QUEUE_SIZE>,
    stats: TransportStats,
    last_rx_error: Option<TransportError>,
}

impl BbqTransport {
    /// Create a bidirectional transport pair backed by static buffers.
    pub fn create_pair() -> Option<(Self, Self)> {
        static TX_BUFFER: BBBuffer<QUEUE_SIZE> = BBBuffer::new();
        static RX_BUFFER: BBBuffer<QUEUE_SIZE> = BBBuffer::new();

        let (tx_prod_a, tx_cons_a) = TX_BUFFER.try_split().ok()?;
        let (rx_prod_a, rx_cons_a) = RX_BUFFER.try_split().ok()?;

        let transport_a = BbqTransport {
            tx_producer: tx_prod_a,
            rx_consumer: rx_cons_a,
            stats: TransportStats::default(),
            last_rx_error: None,
        };
        let transport_b = BbqTransport {
            tx_producer: rx_prod_a,
            rx_consumer: tx_cons_a,
            stats: TransportStats::default(),
            last_rx_error: None,
        };

        Some((transport_a, transport_b))
    }

    /// Create a transport pair for tests.
    #[cfg(test)]
    pub fn create_test_pair() -> (Self, Self) {
        use std::boxed::Box;

        let tx_buffer: &'static BBBuffer<QUEUE_SIZE> = Box::leak(Box::new(BBBuffer::new()));
        let rx_buffer: &'static BBBuffer<QUEUE_SIZE> = Box::leak(Box::new(BBBuffer::new()));

        let (tx_prod_a, tx_cons_a) = tx_buffer.try_split().unwrap();
        let (rx_prod_a, rx_cons_a) = rx_buffer.try_split().unwrap();

        let transport_a = BbqTransport {
            tx_producer: tx_prod_a,
            rx_consumer: rx_cons_a,
            stats: TransportStats::default(),
            last_rx_error: None,
        };
        let transport_b = BbqTransport {
            tx_producer: rx_prod_a,
            rx_consumer: tx_cons_a,
            stats: TransportStats::default(),
            last_rx_error: None,
        };

        (transport_a, transport_b)
    }

    /// Last receive-side error observed by the lossy `try_receive` API.
    pub fn last_receive_error(&self) -> Option<TransportError> {
        self.last_rx_error
    }
}

impl Transport for BbqTransport {
    fn send(&mut self, message: &Message) -> Result<(), TransportError> {
        let mut encoded_buf = [0u8; MAX_ENCODED_MESSAGE_SIZE];
        let serialized = to_slice(message, &mut encoded_buf).map_err(|err| {
            self.stats.tx_errors += 1;
            if matches!(err, postcard::Error::SerializeBufferFull) {
                TransportError::MessageTooLarge
            } else {
                TransportError::SerializationFailed
            }
        })?;

        let data_len = serialized.len();
        let frame_size = 2 + data_len;
        let mut grant = self.tx_producer.grant_exact(frame_size).map_err(|_| {
            self.stats.tx_errors += 1;
            TransportError::BufferFull
        })?;

        grant[0] = (data_len & 0xFF) as u8;
        grant[1] = ((data_len >> 8) & 0xFF) as u8;
        grant[2..2 + data_len].copy_from_slice(serialized);
        grant.commit(2 + data_len);

        self.stats.tx_count += 1;
        Ok(())
    }

    fn try_receive(&mut self) -> Option<Message> {
        let read_grant = self.rx_consumer.read().ok()?;
        if read_grant.len() < 2 {
            return None;
        }

        let data_len = read_grant[0] as usize | ((read_grant[1] as usize) << 8);
        if read_grant.len() < 2 + data_len {
            return None;
        }

        let data = &read_grant[2..2 + data_len];
        let message = match from_bytes::<Message>(data) {
            Ok(msg) => msg,
            Err(_) => {
                self.stats.rx_errors += 1;
                self.last_rx_error = Some(TransportError::DeserializationFailed);
                read_grant.release(2 + data_len);
                return None;
            }
        };

        read_grant.release(2 + data_len);
        self.stats.rx_count += 1;
        self.last_rx_error = None;
        Some(message)
    }

    fn poll(&mut self) {}

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn stats(&self) -> TransportStats {
        TransportStats {
            tx_buffer_usage: 0,
            rx_buffer_usage: 0,
            avg_latency_us: Some(0),
            ..self.stats
        }
    }

    fn is_ready(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn send_receive_small_message() {
        let (mut tx, mut rx) = BbqTransport::create_test_pair();
        let msg = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 12345,
        };

        tx.send(&msg).expect("send");
        assert_eq!(rx.try_receive(), Some(msg));
        assert_eq!(tx.stats().tx_count, 1);
        assert_eq!(rx.stats().rx_count, 1);
    }
}
