use ecu_ts::serial::{FrameAssembler, SerialPort};
use ecu_ts::server::{BenchToolingOwner, OutpcProvider, PageStore, TsServer};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct TsServiceStats {
    pub serial_write_failures: u32,
    pub last_serial_write_expected: usize,
    pub last_serial_write_actual: usize,
}

/// Minimal TS service with bounded pump and internal buffers.
pub struct TsService<P: OutpcProvider, S: PageStore> {
    pub server: TsServer<P, S>,
    asm: FrameAssembler,
    inbuf: [u8; 512],
    out: [u8; 512],
    stats: TsServiceStats,
}

impl<P: OutpcProvider, S: PageStore> TsService<P, S> {
    pub fn new(signature: &'static [u8], provider: P, store: S) -> Self {
        Self {
            server: TsServer::new(signature, provider, store),
            asm: FrameAssembler::new(),
            inbuf: [0; 512],
            out: [0; 512],
            stats: TsServiceStats::default(),
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
                    let written = port.write(&self.out[..mr]);
                    if written != mr {
                        self.stats.serial_write_failures =
                            self.stats.serial_write_failures.saturating_add(1);
                        self.stats.last_serial_write_expected = mr;
                        self.stats.last_serial_write_actual = written;
                    }
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

    /// Handle a single frame already present in `frame`, writing response into
    /// `out`, while routing bounded bench-tooling commands through the supplied
    /// owner.
    pub fn handle_frame_with_bench_tooling<B: BenchToolingOwner>(
        &mut self,
        frame: &[u8],
        out: &mut [u8],
        bench_tooling: &mut B,
    ) -> Option<usize> {
        self.server
            .handle_with_bench_tooling(frame, out, bench_tooling)
    }

    pub fn stats(&self) -> TsServiceStats {
        self.stats
    }

    pub fn reset_stats(&mut self) {
        self.stats = TsServiceStats::default();
    }
}
