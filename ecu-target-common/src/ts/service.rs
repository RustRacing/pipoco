use ecu_core::ts::serial::{FrameAssembler, SerialPort};
use ecu_core::ts::server::PageStore;
use ecu_core::ts::{OutpcProvider, TunerstudioServer};

/// Minimal TS service with bounded pump and internal buffers.
pub struct TsService<P: OutpcProvider, S: PageStore> {
    pub server: TunerstudioServer<P, S>,
    asm: FrameAssembler,
    inbuf: [u8; 512],
    out: [u8; 512],
}

impl<P: OutpcProvider, S: PageStore> TsService<P, S> {
    pub fn new(signature: &'static [u8], provider: P, store: S) -> Self {
        Self {
            server: TunerstudioServer::new(signature, provider, store),
            asm: FrameAssembler::new(),
            inbuf: [0; 512],
            out: [0; 512],
        }
    }

    pub fn poll_port<SP: SerialPort>(&mut self, port: &mut SP) {
        self.asm.poll_port(port);
    }

    pub fn pump_with_budget<SP: SerialPort>(&mut self, port: &mut SP, mut budget: u8) {
        self.asm.poll_port(port);
        while budget > 0 {
            if let Some(len) = self.asm.try_pop(&mut self.inbuf) {
                if let Some(mr) = self.server.handle(&self.inbuf[..len], &mut self.out) {
                    let _ = port.write(&self.out[..mr]);
                }
                budget -= 1;
            } else {
                break;
            }
        }
    }

    /// Handle a single frame already present in `frame`, writing response into `out`
    pub fn handle_frame(&mut self, frame: &[u8], out: &mut [u8]) -> Option<usize> {
        self.server.handle(frame, out)
    }
}
