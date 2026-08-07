use ecu_atmega2560::{Atmega2560BoardAdapter, Atmega2560StepInput};
use ecu_board_api::TelemetryFrame;
use ecu_domain::{Degrees10, Kpa10, Micros, Percent, Rpm};
use ecu_io::{
    OutputAssemblyCounters, OutputStageSnapshot, SignalAssemblyCounters, SignalStageSnapshot,
};
use serde::Serialize;
use serde_json::Value;
use signal_hook::consts::signal::{SIGINT, SIGTERM};
use signal_hook::flag;
use std::env;
use std::fs::{self, File, OpenOptions};
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::os::unix::net::UnixStream;
use std::panic::{self, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const DEFAULT_FIRMWARE: &str = "aidocs/ref/code/Speeduino-M5x-PCBs/6-cyl firmware files/202305.hex";
const DEFAULT_MAP: &str =
    "aidocs/ref/code/Speeduino-M5x-PCBs/m50-m40-m60_Pnp/Base Tunes/202305/BMW_M50TU_PnP.msq";
const DEFAULT_DURATION_SECS: u64 = 8 * 60 * 60;
const DEFAULT_SAMPLE_SECS: u64 = 60;
const DEFAULT_MAX_RSS_GROWTH_KB: u64 = 256 * 1024;
const DEFAULT_WARMUP_SAMPLES: u64 = 2;
const HOST_TELEMETRY_ARTIFACT: &str = "host_telemetry.json";
const HOST_MAX_PENDING_OUTPUTS: usize = 12;

#[derive(Debug, Clone)]
struct Config {
    firmware: PathBuf,
    map: PathBuf,
    duration_secs: u64,
    sample_secs: u64,
    max_rss_growth_kb: u64,
    warmup_samples: u64,
    run_root: PathBuf,
    pin_smoke: bool,
    pin_drive: bool,
}

#[derive(Debug, Clone, Copy, Default)]
struct ProcessMemory {
    vm_rss_kb: u64,
    vm_hwm_kb: u64,
    vm_size_kb: u64,
}

#[derive(Debug, Clone)]
struct SoakSummary {
    reason: StopReason,
    elapsed: Duration,
    sample_count: u64,
    baseline: Option<ProcessMemory>,
    baseline_sample: Option<u64>,
    last: ProcessMemory,
    host: HostReportSummary,
    pin_drive: PinDriveReportSummary,
}

#[derive(Debug, Clone)]
struct HostReportSummary {
    artifact: String,
    runtime_queue_depth_max: usize,
    runtime_queue_depth_final: usize,
    reset_count: u64,
    output_transition_count: u64,
    aux_command_count: u64,
    clock_monotonicity_violations: usize,
    final_telemetry: Option<TelemetryFrameSummary>,
    signal_counters: HostSignalAssemblyCounters,
    output_counters: HostOutputAssemblyCounters,
    invariant_failures: Vec<String>,
}

#[derive(Debug, Clone)]
struct PinDriveReportSummary {
    enabled: bool,
    events: u64,
    set_failures: u64,
    readback_failures: u64,
    last_level: u8,
}

#[derive(Debug, Clone, Serialize)]
struct HostRuntimeTelemetryArtifact {
    samples: Vec<HostRuntimeTelemetrySample>,
    host_reset_count: u64,
    scheduler_queue_depth_max: usize,
    scheduler_queue_depth_final: usize,
    final_telemetry: Option<TelemetryFrameSummary>,
    signal_assembly_counters: HostSignalAssemblyCounters,
    output_assembly_counters: HostOutputAssemblyCounters,
    output_transition_count: u64,
    aux_command_count: u64,
    clock_monotonicity_violations: usize,
    invariant_failures: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
struct HostRuntimeTelemetrySample {
    sample_index: u64,
    sample_timestamp_ms: u64,
    host_clock_monotonic: bool,
    scheduler_queue_depth: usize,
    signal_assembly_counters: HostSignalAssemblyCounters,
    output_assembly_counters: HostOutputAssemblyCounters,
    output_transition_count: usize,
    aux_command_count: usize,
}

#[derive(Debug, Clone, Copy, Default, Serialize)]
struct TelemetryStageCounterSnapshot {
    seen: u32,
    accepted: u32,
    rejected: u32,
    dropped: u32,
    stale: u32,
    overrun: u32,
    late: u32,
    last_trace_id: u32,
    last_sequence_or_command_id: u32,
    last_timestamp_us: u64,
    last_reason: u8,
}

#[derive(Debug, Clone, Copy, Serialize)]
struct HostSignalAssemblyCounters {
    signal_capture: TelemetryStageCounterSnapshot,
    signal_normalizer: TelemetryStageCounterSnapshot,
    observation_validator: TelemetryStageCounterSnapshot,
    observation_publisher: TelemetryStageCounterSnapshot,
    observation_reader: TelemetryStageCounterSnapshot,
    runtime_snapshot_builder: TelemetryStageCounterSnapshot,
    policy_consumer: TelemetryStageCounterSnapshot,
}

#[derive(Debug, Clone, Copy, Serialize)]
struct HostOutputAssemblyCounters {
    output_intent: TelemetryStageCounterSnapshot,
    output_planner: TelemetryStageCounterSnapshot,
    output_admission: TelemetryStageCounterSnapshot,
    output_armer: TelemetryStageCounterSnapshot,
    output_executor: TelemetryStageCounterSnapshot,
    output_observer: TelemetryStageCounterSnapshot,
}

#[derive(Debug, Clone, Serialize)]
struct TelemetryFrameSummary {
    control_mode: String,
    fault_code: String,
    fault_severity: String,
    ignition_advance_deg10: i16,
    dwell_us: u16,
    injector_pulse_width_us: u32,
}

#[derive(Debug)]
struct PinDriveSharedState {
    events: std::sync::atomic::AtomicU64,
    set_failures: std::sync::atomic::AtomicU64,
    readback_failures: std::sync::atomic::AtomicU64,
    last_level: std::sync::atomic::AtomicU8,
}

impl Default for PinDriveSharedState {
    fn default() -> Self {
        Self {
            events: std::sync::atomic::AtomicU64::new(0),
            set_failures: std::sync::atomic::AtomicU64::new(0),
            readback_failures: std::sync::atomic::AtomicU64::new(0),
            last_level: std::sync::atomic::AtomicU8::new(0),
        }
    }
}

impl PinDriveSharedState {
    fn snapshot(&self) -> PinDriveReportSummary {
        PinDriveReportSummary {
            enabled: true,
            events: self.events.load(Ordering::Relaxed),
            set_failures: self.set_failures.load(Ordering::Relaxed),
            readback_failures: self.readback_failures.load(Ordering::Relaxed),
            last_level: self.last_level.load(Ordering::Relaxed),
        }
    }
}

#[derive(Debug)]
struct PinDriver {
    error: Arc<Mutex<Option<String>>>,
    handle: thread::JoinHandle<()>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StopReason {
    Completed,
    Interrupted,
    QemuExited(i32),
    QemuSignaled,
    MemoryGrowthExceeded,
    HostStepPanicked,
    HostStepError,
    HostClockRollback,
    HostSchedulerQueueDepthInvalid,
    HostInvariantFailure,
}

impl StopReason {
    fn is_success(self) -> bool {
        matches!(self, Self::Completed | Self::Interrupted)
    }

    fn as_str(&self) -> String {
        match self {
            StopReason::Completed => "completed".to_string(),
            StopReason::Interrupted => "interrupted".to_string(),
            StopReason::QemuExited(code) => format!("qemu-exited-{code}"),
            StopReason::QemuSignaled => "qemu-signaled".to_string(),
            StopReason::MemoryGrowthExceeded => "memory-growth-exceeded".to_string(),
            StopReason::HostStepPanicked => "host-runtime-step-panicked".to_string(),
            StopReason::HostStepError => "host-runtime-step-error".to_string(),
            StopReason::HostClockRollback => "host-clock-rollback".to_string(),
            StopReason::HostSchedulerQueueDepthInvalid => {
                "host-scheduler-queue-depth-invalid".to_string()
            }
            StopReason::HostInvariantFailure => "host-invariant-failure".to_string(),
        }
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("qemu soak failed: {error}");
        std::process::exit(1);
    }
}

fn run() -> io::Result<()> {
    let config = Config::parse(env::args().skip(1))?;
    validate_input("firmware", &config.firmware)?;
    validate_input("map", &config.map)?;

    let run_dir = make_run_dir(&config.run_root)?;
    let firmware_bin = run_dir.join("firmware.bin");
    convert_hex_to_binary(&config.firmware, &firmware_bin)?;

    let stop = Arc::new(AtomicBool::new(false));
    flag::register(SIGINT, Arc::clone(&stop))?;
    flag::register(SIGTERM, Arc::clone(&stop))?;

    let qmp_socket = run_dir.join("qmp.sock");
    let mut child = spawn_qemu(&firmware_bin, &run_dir, &qmp_socket)?;
    if config.pin_smoke {
        if let Err(error) = run_pin_smoke(&qmp_socket) {
            stop_qemu(&mut child);
            return Err(error);
        }
    }
    let pin_drive_state = if config.pin_drive {
        Some(Arc::new(PinDriveSharedState::default()))
    } else {
        None
    };
    let pin_driver = pin_drive_state
        .as_ref()
        .map(|state| PinDriver::spawn(&qmp_socket, Arc::clone(&stop), Arc::clone(state)));
    let reason = monitor_soak(
        &config,
        &run_dir,
        &mut child,
        Arc::clone(&stop),
        pin_drive_state.as_deref(),
    )?;
    stop.store(true, Ordering::Relaxed);
    if let Some(driver) = pin_driver {
        driver.join()?;
    }
    stop_qemu(&mut child);

    if reason.is_success() {
        Ok(())
    } else {
        Err(io::Error::other(format!(
            "soak stopped with {}",
            reason.as_str()
        )))
    }
}

fn is_host_clock_monotonic(
    previous_timestamp_ms: Option<u64>,
    sample_timestamp_ms: u64,
    sample_index: u64,
    invariant_failures: &mut Vec<String>,
    host_clock_rollback_count: &mut usize,
) -> bool {
    if let Some(previous) = previous_timestamp_ms {
        if sample_timestamp_ms < previous {
            *host_clock_rollback_count = host_clock_rollback_count.saturating_add(1);
            invariant_failures.push(format!(
                "host clock rollback sample {sample_index}: {sample_timestamp_ms} < {previous}"
            ));
            return false;
        }
    }

    true
}

impl Config {
    fn parse(args: impl Iterator<Item = String>) -> io::Result<Self> {
        let mut config = Self {
            firmware: PathBuf::from(DEFAULT_FIRMWARE),
            map: PathBuf::from(DEFAULT_MAP),
            duration_secs: DEFAULT_DURATION_SECS,
            sample_secs: DEFAULT_SAMPLE_SECS,
            max_rss_growth_kb: DEFAULT_MAX_RSS_GROWTH_KB,
            warmup_samples: DEFAULT_WARMUP_SAMPLES,
            run_root: PathBuf::from("target/qemu-soak/runs"),
            pin_smoke: false,
            pin_drive: false,
        };

        let mut args = args.peekable();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--firmware" => config.firmware = required_path_value(&mut args, "--firmware")?,
                "--map" => config.map = required_path_value(&mut args, "--map")?,
                "--duration-secs" => {
                    config.duration_secs = required_u64_value(&mut args, "--duration-secs")?
                }
                "--sample-secs" => {
                    config.sample_secs = required_u64_value(&mut args, "--sample-secs")?
                }
                "--max-rss-growth-kb" => {
                    config.max_rss_growth_kb = required_u64_value(&mut args, "--max-rss-growth-kb")?
                }
                "--warmup-samples" => {
                    config.warmup_samples = required_u64_value(&mut args, "--warmup-samples")?
                }
                "--run-root" => config.run_root = required_path_value(&mut args, "--run-root")?,
                "--pin-smoke" => config.pin_smoke = true,
                "--pin-drive" => config.pin_drive = true,
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidInput,
                        format!("unknown argument {other}"),
                    ));
                }
            }
        }

        if config.duration_secs == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "--duration-secs must be greater than 0",
            ));
        }
        if config.sample_secs == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "--sample-secs must be greater than 0",
            ));
        }

        Ok(config)
    }
}

fn required_path_value(
    args: &mut std::iter::Peekable<impl Iterator<Item = String>>,
    name: &str,
) -> io::Result<PathBuf> {
    args.next()
        .map(PathBuf::from)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, format!("{name} needs a value")))
}

fn required_u64_value(
    args: &mut std::iter::Peekable<impl Iterator<Item = String>>,
    name: &str,
) -> io::Result<u64> {
    let value = args.next().ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidInput, format!("{name} needs a value"))
    })?;
    value.parse::<u64>().map_err(|_| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("{name} needs an unsigned integer value"),
        )
    })
}

fn print_help() {
    println!(
        "Usage: cargo run -p ecu-qemu-soak -- [--duration-secs N] [--sample-secs N] [--warmup-samples N]\n\
         Defaults:\n\
           --firmware {DEFAULT_FIRMWARE}\n\
           --map {DEFAULT_MAP}\n\
            --duration-secs {DEFAULT_DURATION_SECS}\n\
            --sample-secs {DEFAULT_SAMPLE_SECS}\n\
            --warmup-samples {DEFAULT_WARMUP_SAMPLES}\n\
            --pin-smoke disabled\n\
            --pin-drive disabled"
    );
}

fn validate_input(label: &str, path: &Path) -> io::Result<()> {
    if path.is_file() {
        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("{label} file not found: {}", path.display()),
        ))
    }
}

fn make_run_dir(root: &Path) -> io::Result<PathBuf> {
    let ts = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(io::Error::other)?
        .as_secs();
    let run_dir = root.join(format!("{ts}"));
    fs::create_dir_all(&run_dir)?;
    Ok(run_dir)
}

fn convert_hex_to_binary(firmware_hex: &Path, firmware_bin: &Path) -> io::Result<()> {
    let status = Command::new("objcopy")
        .args(["-I", "ihex", "-O", "binary"])
        .arg(firmware_hex)
        .arg(firmware_bin)
        .status()?;
    if status.success() {
        Ok(())
    } else {
        Err(io::Error::other(format!(
            "objcopy failed for {}",
            firmware_hex.display()
        )))
    }
}

fn spawn_qemu(firmware_bin: &Path, run_dir: &Path, qmp_socket: &Path) -> io::Result<Child> {
    let stderr = File::create(run_dir.join("qemu.stderr.log"))?;
    Command::new("qemu-system-avr")
        .args(["-M", "mega2560"])
        .arg("-bios")
        .arg(firmware_bin)
        .args(["-nographic", "-serial", "none", "-monitor", "none"])
        .arg("-qmp")
        .arg(format!("unix:{},server=on,wait=off", qmp_socket.display()))
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::from(stderr))
        .spawn()
}

fn run_pin_smoke(qmp_socket: &Path) -> io::Result<()> {
    let mut qmp = QmpClient::connect(qmp_socket)?;
    qmp.execute(r#"{"execute":"qmp_capabilities"}"#)?;
    qmp.execute(
        r#"{"execute":"qom-set","arguments":{"path":"/machine/mcu/gpiod","property":"external-level","value":4}}"#,
    )?;
    let high = qmp.execute(
        r#"{"execute":"qom-get","arguments":{"path":"/machine/mcu/gpiod","property":"external-level"}}"#,
    )?;
    if qmp_return_u8(&high)? != 4 {
        return Err(io::Error::other(format!(
            "patched QEMU GPIO qom-get did not report PD2 high: {high}"
        )));
    }
    qmp.execute(
        r#"{"execute":"qom-set","arguments":{"path":"/machine/mcu/gpiod","property":"external-level","value":0}}"#,
    )?;
    let low = qmp.execute(
        r#"{"execute":"qom-get","arguments":{"path":"/machine/mcu/gpiod","property":"external-level"}}"#,
    )?;
    if qmp_return_u8(&low)? != 0 {
        return Err(io::Error::other(format!(
            "patched QEMU GPIO qom-get did not report PD2 low: {low}"
        )));
    }
    Ok(())
}

fn qmp_return_u8(response: &str) -> io::Result<u8> {
    let response = response.trim();
    let parsed = serde_json::from_str::<Value>(response).map_err(|error| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("invalid qmp response: {error}"),
        )
    })?;
    match parsed.get("return") {
        Some(number) => number
            .as_u64()
            .and_then(|value| u8::try_from(value).ok())
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("unexpected qmp return value: {response}"),
                )
            }),
        None => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("qmp response missing return field: {response}"),
        )),
    }
}

impl PinDriver {
    fn spawn(qmp_socket: &Path, stop: Arc<AtomicBool>, state: Arc<PinDriveSharedState>) -> Self {
        let qmp_socket = qmp_socket.to_path_buf();
        let error = Arc::new(Mutex::new(None));
        let thread_error = Arc::clone(&error);
        let thread_state = Arc::clone(&state);
        let handle = thread::spawn(move || {
            if let Err(driver_error) = drive_pins(&qmp_socket, &stop, thread_state) {
                if let Ok(mut slot) = thread_error.lock() {
                    *slot = Some(driver_error.to_string());
                }
                stop.store(true, Ordering::Relaxed);
            }
        });
        Self { error, handle }
    }

    fn join(self) -> io::Result<()> {
        if self.handle.join().is_err() {
            return Err(io::Error::other("pin driver thread panicked"));
        }
        match self.error.lock() {
            Ok(mut error) => {
                if let Some(error) = error.take() {
                    Err(io::Error::other(format!("pin driver failed: {error}")))
                } else {
                    Ok(())
                }
            }
            Err(_) => Err(io::Error::other("pin driver error lock poisoned")),
        }
    }
}

fn drive_pins(
    qmp_socket: &Path,
    stop: &AtomicBool,
    state: Arc<PinDriveSharedState>,
) -> io::Result<()> {
    let mut qmp = QmpClient::connect(qmp_socket)?;
    qmp.execute(r#"{"execute":"qmp_capabilities"}"#)?;
    let mut phase = 0u8;
    while !stop.load(Ordering::Relaxed) {
        let crank = if phase & 1 == 0 { 0x04 } else { 0x00 };
        let cam = if phase & 7 < 4 { 0x08 } else { 0x00 };
        let level = crank | cam;
        state.events.fetch_add(1, Ordering::Relaxed);
        state.last_level.store(level, Ordering::Relaxed);
        if let Err(error) = qmp_set_external_level(&mut qmp, level) {
            state.set_failures.fetch_add(1, Ordering::Relaxed);
            return Err(error);
        }
        let observed = match qmp_return_u8(
            &qmp.execute(
                r#"{"execute":"qom-get","arguments":{"path":"/machine/mcu/gpiod","property":"external-level"}}"#,
            )?,
        ) {
            Ok(observed) => observed,
            Err(error) => {
                state.set_failures.fetch_add(1, Ordering::Relaxed);
                return Err(error);
            }
        };
        if observed != level {
            state.readback_failures.fetch_add(1, Ordering::Relaxed);
        }
        phase = phase.wrapping_add(1);
        thread::sleep(Duration::from_millis(25));
    }
    qmp_set_external_level(&mut qmp, 0)
}

fn qmp_set_external_level(qmp: &mut QmpClient, value: u8) -> io::Result<()> {
    qmp.execute(&format!(
        "{{\"execute\":\"qom-set\",\"arguments\":{{\"path\":\"/machine/mcu/gpiod\",\"property\":\"external-level\",\"value\":{value}}}}}"
    ))?;
    Ok(())
}

struct QmpClient {
    reader: BufReader<UnixStream>,
}

impl QmpClient {
    fn connect(path: &Path) -> io::Result<Self> {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            match UnixStream::connect(path) {
                Ok(stream) => {
                    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
                    stream.set_write_timeout(Some(Duration::from_secs(5)))?;
                    let mut client = Self {
                        reader: BufReader::new(stream),
                    };
                    let greeting = client.read_message()?;
                    if !greeting.contains("\"QMP\"") {
                        return Err(io::Error::other(format!(
                            "unexpected QMP greeting: {greeting}"
                        )));
                    }
                    return Ok(client);
                }
                Err(error) if Instant::now() < deadline => {
                    if error.kind() != io::ErrorKind::NotFound
                        && error.kind() != io::ErrorKind::ConnectionRefused
                    {
                        return Err(error);
                    }
                    thread::sleep(Duration::from_millis(25));
                }
                Err(error) => return Err(error),
            }
        }
    }

    fn execute(&mut self, command: &str) -> io::Result<String> {
        let stream = self.reader.get_mut();
        stream.write_all(command.as_bytes())?;
        stream.write_all(b"\n")?;
        stream.flush()?;
        let response = self.read_message()?;
        if response.contains("\"error\"") {
            return Err(io::Error::other(format!("QMP command failed: {response}")));
        }
        Ok(response)
    }

    fn read_message(&mut self) -> io::Result<String> {
        let mut line = String::new();
        self.reader.read_line(&mut line)?;
        if line.is_empty() {
            Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "QMP socket closed",
            ))
        } else {
            Ok(line)
        }
    }
}

fn monitor_soak(
    config: &Config,
    run_dir: &Path,
    child: &mut Child,
    stop: Arc<AtomicBool>,
    pin_drive_state: Option<&PinDriveSharedState>,
) -> io::Result<StopReason> {
    let mut csv = BufWriter::new(File::create(run_dir.join("samples.csv"))?);
    writeln!(csv, "elapsed_secs,vm_rss_kb,vm_hwm_kb,vm_size_kb")?;

    let started = Instant::now();
    let mut host_adapter = Atmega2560BoardAdapter::m50b25tu_speeduino_m5x_rev23();
    let sample_period = Duration::from_secs(config.sample_secs);
    let duration = Duration::from_secs(config.duration_secs);
    let mut baseline: Option<ProcessMemory> = None;
    let mut baseline_sample: Option<u64> = None;
    let mut last = ProcessMemory::default();
    let mut sample_count = 0u64;
    let mut host_previous_sample_timestamp_ms = None;
    let mut host_previous_signal_counters: Option<HostSignalAssemblyCounters> = None;
    let mut host_previous_output_counters: Option<HostOutputAssemblyCounters> = None;
    let mut host_output_transition_count: u64 = 0;
    let mut host_aux_command_count: u64 = 0;
    let mut host_clock_rollback_count: usize = 0;
    let mut host_queue_depth_max: usize = 0;
    let mut host_invariant_failures: Vec<String> = Vec::new();
    let mut host_signal_counters = HostSignalAssemblyCounters {
        signal_capture: TelemetryStageCounterSnapshot::default(),
        signal_normalizer: TelemetryStageCounterSnapshot::default(),
        observation_validator: TelemetryStageCounterSnapshot::default(),
        observation_publisher: TelemetryStageCounterSnapshot::default(),
        observation_reader: TelemetryStageCounterSnapshot::default(),
        runtime_snapshot_builder: TelemetryStageCounterSnapshot::default(),
        policy_consumer: TelemetryStageCounterSnapshot::default(),
    };
    let mut host_output_counters = HostOutputAssemblyCounters {
        output_intent: TelemetryStageCounterSnapshot::default(),
        output_planner: TelemetryStageCounterSnapshot::default(),
        output_admission: TelemetryStageCounterSnapshot::default(),
        output_armer: TelemetryStageCounterSnapshot::default(),
        output_executor: TelemetryStageCounterSnapshot::default(),
        output_observer: TelemetryStageCounterSnapshot::default(),
    };
    let mut host_final_telemetry = None;
    let mut host_samples = Vec::new();
    let mut host_queue_depth_final = 0usize;
    let mut reason = StopReason::Completed;

    loop {
        if stop.load(Ordering::Relaxed) {
            reason = StopReason::Interrupted;
            break;
        }

        if let Some(status) = child.try_wait()? {
            reason = match status.code() {
                Some(code) => StopReason::QemuExited(code),
                None => StopReason::QemuSignaled,
            };
            break;
        }

        let elapsed = started.elapsed();
        if elapsed >= duration {
            break;
        }

        last = read_process_memory(child.id())?;
        let sample_index = sample_count.saturating_add(1);
        let mut memory_growth_exceeded = false;
        if sample_index > config.warmup_samples {
            let first = *baseline.get_or_insert_with(|| {
                baseline_sample = Some(sample_index);
                last
            });
            if last.vm_rss_kb.saturating_sub(first.vm_rss_kb) > config.max_rss_growth_kb {
                memory_growth_exceeded = true;
            }
        }

        writeln!(
            csv,
            "{},{},{},{}",
            elapsed.as_secs(),
            last.vm_rss_kb,
            last.vm_hwm_kb,
            last.vm_size_kb
        )?;
        csv.flush()?;
        sample_count = sample_index;

        let host_sample_timestamp_ms = monotonic_timestamp_ms(elapsed)?;
        let host_clock_monotonic = is_host_clock_monotonic(
            host_previous_sample_timestamp_ms,
            host_sample_timestamp_ms,
            sample_index,
            &mut host_invariant_failures,
            &mut host_clock_rollback_count,
        );
        host_previous_sample_timestamp_ms = Some(host_sample_timestamp_ms);

        if !host_clock_monotonic {
            reason = StopReason::HostClockRollback;
            break;
        }

        let sample_input =
            host_step_input_for_sample(sample_index, elapsed, pin_drive_state.is_some());
        let host_step = panic::catch_unwind(AssertUnwindSafe(|| host_adapter.step(sample_input)));
        let step_output = match host_step {
            Ok(Ok(step_output)) => step_output,
            Ok(Err(error)) => {
                host_invariant_failures.push(format!("host step error: {error:?}"));
                reason = StopReason::HostStepError;
                break;
            }
            Err(_) => {
                host_invariant_failures.push("host step panicked".to_string());
                reason = StopReason::HostStepPanicked;
                break;
            }
        };

        let queue_depth = host_adapter.runtime().pending_output_count();
        host_queue_depth_max = host_queue_depth_max.max(queue_depth);
        if queue_depth > HOST_MAX_PENDING_OUTPUTS {
            host_invariant_failures.push(format!(
                "host scheduler queue depth {queue_depth} exceeded {HOST_MAX_PENDING_OUTPUTS}"
            ));
            reason = StopReason::HostSchedulerQueueDepthInvalid;
            break;
        }
        host_queue_depth_final = queue_depth;

        let next_signal_counters =
            snapshot_signal_counters(host_adapter.runtime().signal_assembly_counters());
        let next_output_counters =
            snapshot_output_counters(host_adapter.output_assembly_counters());
        if let Some(previous) = host_previous_signal_counters {
            host_invariant_failures.extend(find_counter_regressions(
                "signal",
                "signal_capture",
                &previous.signal_capture,
                &next_signal_counters.signal_capture,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "signal",
                "signal_normalizer",
                &previous.signal_normalizer,
                &next_signal_counters.signal_normalizer,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "signal",
                "observation_validator",
                &previous.observation_validator,
                &next_signal_counters.observation_validator,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "signal",
                "observation_publisher",
                &previous.observation_publisher,
                &next_signal_counters.observation_publisher,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "signal",
                "observation_reader",
                &previous.observation_reader,
                &next_signal_counters.observation_reader,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "signal",
                "runtime_snapshot_builder",
                &previous.runtime_snapshot_builder,
                &next_signal_counters.runtime_snapshot_builder,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "signal",
                "policy_consumer",
                &previous.policy_consumer,
                &next_signal_counters.policy_consumer,
            ));
        }
        if let Some(previous) = host_previous_output_counters {
            host_invariant_failures.extend(find_counter_regressions(
                "output",
                "output_intent",
                &previous.output_intent,
                &next_output_counters.output_intent,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "output",
                "output_planner",
                &previous.output_planner,
                &next_output_counters.output_planner,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "output",
                "output_admission",
                &previous.output_admission,
                &next_output_counters.output_admission,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "output",
                "output_armer",
                &previous.output_armer,
                &next_output_counters.output_armer,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "output",
                "output_executor",
                &previous.output_executor,
                &next_output_counters.output_executor,
            ));
            host_invariant_failures.extend(find_counter_regressions(
                "output",
                "output_observer",
                &previous.output_observer,
                &next_output_counters.output_observer,
            ));
        }

        if !host_invariant_failures.is_empty() {
            reason = StopReason::HostInvariantFailure;
            break;
        }
        host_previous_signal_counters = Some(next_signal_counters);
        host_previous_output_counters = Some(next_output_counters);
        host_signal_counters = next_signal_counters;
        host_output_counters = next_output_counters;

        host_output_transition_count =
            host_output_transition_count.saturating_add(step_output.outputs.len() as u64);
        host_aux_command_count =
            host_aux_command_count.saturating_add(step_output.aux.len() as u64);
        host_final_telemetry = Some(telemetry_frame_summary(step_output.telemetry));
        host_samples.push(HostRuntimeTelemetrySample {
            sample_index,
            sample_timestamp_ms: host_sample_timestamp_ms,
            host_clock_monotonic,
            scheduler_queue_depth: queue_depth,
            signal_assembly_counters: host_signal_counters,
            output_assembly_counters: host_output_counters,
            output_transition_count: usize::try_from(host_output_transition_count)
                .unwrap_or(usize::MAX),
            aux_command_count: usize::try_from(host_aux_command_count).unwrap_or(usize::MAX),
        });

        if memory_growth_exceeded {
            reason = StopReason::MemoryGrowthExceeded;
            break;
        }

        if !host_invariant_failures.is_empty() {
            reason = StopReason::HostInvariantFailure;
            break;
        }

        thread::sleep(sample_period.min(duration.saturating_sub(elapsed)));
    }

    let host_artifact = HostRuntimeTelemetryArtifact {
        samples: host_samples,
        host_reset_count: 0,
        scheduler_queue_depth_max: host_queue_depth_max,
        scheduler_queue_depth_final: host_queue_depth_final,
        final_telemetry: host_final_telemetry,
        signal_assembly_counters: host_signal_counters,
        output_assembly_counters: host_output_counters,
        output_transition_count: host_output_transition_count,
        aux_command_count: host_aux_command_count,
        clock_monotonicity_violations: host_clock_rollback_count,
        invariant_failures: host_invariant_failures,
    };
    write_host_telemetry(run_dir, &host_artifact)?;

    write_report(
        config,
        run_dir,
        SoakSummary {
            reason,
            elapsed: started.elapsed(),
            sample_count,
            baseline,
            baseline_sample,
            last,
            host: HostReportSummary {
                artifact: HOST_TELEMETRY_ARTIFACT.to_string(),
                runtime_queue_depth_max: host_queue_depth_max,
                runtime_queue_depth_final: host_queue_depth_final,
                reset_count: host_artifact.host_reset_count,
                output_transition_count: host_artifact.output_transition_count,
                aux_command_count: host_artifact.aux_command_count,
                clock_monotonicity_violations: host_artifact.clock_monotonicity_violations,
                final_telemetry: host_artifact.final_telemetry,
                signal_counters: host_artifact.signal_assembly_counters,
                output_counters: host_artifact.output_assembly_counters,
                invariant_failures: host_artifact.invariant_failures,
            },
            pin_drive: pin_drive_state
                .map(PinDriveSharedState::snapshot)
                .unwrap_or(PinDriveReportSummary {
                    enabled: false,
                    events: 0,
                    set_failures: 0,
                    readback_failures: 0,
                    last_level: 0,
                }),
        },
    )?;
    Ok(reason)
}

fn monotonic_timestamp_ms(elapsed: Duration) -> io::Result<u64> {
    u64::try_from(elapsed.as_millis()).map_err(io::Error::other)
}

fn host_step_input_for_sample(
    sample_index: u64,
    elapsed: Duration,
    pin_drive_enabled: bool,
) -> Atmega2560StepInput {
    let phase = (sample_index % 8) as u8;
    let crank_high = if pin_drive_enabled {
        phase & 1 == 0
    } else {
        true
    };
    let cam_high = if pin_drive_enabled { phase < 4 } else { true };
    let rpm = if crank_high { 1300u16 } else { 900u16 };
    let load = if cam_high { 1100u16 } else { 750u16 };
    let angle_step = 37u16.saturating_mul(sample_index as u16);

    Atmega2560StepInput {
        now_us: Micros::new(elapsed.as_micros().min(u128::from(u32::MAX)) as u32),
        rpm: Rpm::new(rpm),
        load_kpa10: Kpa10::new(load),
        crank_angle_x10: Degrees10::new(
            i16::try_from(angle_step % 7200).expect("crank angle wrapped to 0..7199"),
        ),
        throttle: Percent::new(if crank_high { 55 } else { 37 }),
        coolant_temp_c10: if cam_high { 820 } else { 900 },
        intake_temp_c10: if cam_high { 640 } else { 710 },
        battery_mv: if cam_high { 13_600 } else { 12_800 },
        lambda: ecu_domain::Lambda100::new(if crank_high { 108 } else { 95 }),
        engine_time_authority: Atmega2560StepInput::expert_manual_authority(),
        launch_armed: sample_index.is_multiple_of(23),
        flat_shift_armed: sample_index.is_multiple_of(37),
    }
}

fn telemetry_frame_summary(frame: TelemetryFrame) -> TelemetryFrameSummary {
    TelemetryFrameSummary {
        control_mode: format!("{:?}", frame.control_mode),
        fault_code: format!("{:?}", frame.fault_code),
        fault_severity: format!("{:?}", frame.fault_severity),
        ignition_advance_deg10: frame.ignition_advance.get(),
        dwell_us: frame.dwell_us.get(),
        injector_pulse_width_us: frame.injector_pulse_width_us.get(),
    }
}

fn snapshot_signal_counters(counters: SignalAssemblyCounters) -> HostSignalAssemblyCounters {
    HostSignalAssemblyCounters {
        signal_capture: snapshot_signal_stage(counters.signal_capture),
        signal_normalizer: snapshot_signal_stage(counters.signal_normalizer),
        observation_validator: snapshot_signal_stage(counters.observation_validator),
        observation_publisher: snapshot_signal_stage(counters.observation_publisher),
        observation_reader: snapshot_signal_stage(counters.observation_reader),
        runtime_snapshot_builder: snapshot_signal_stage(counters.runtime_snapshot_builder),
        policy_consumer: snapshot_signal_stage(counters.policy_consumer),
    }
}

fn snapshot_signal_stage(snapshot: SignalStageSnapshot) -> TelemetryStageCounterSnapshot {
    TelemetryStageCounterSnapshot {
        seen: snapshot.seen,
        accepted: snapshot.accepted,
        rejected: snapshot.rejected,
        dropped: snapshot.dropped,
        stale: snapshot.stale,
        overrun: snapshot.overrun,
        late: 0,
        last_trace_id: snapshot.last_trace_id.get(),
        last_sequence_or_command_id: snapshot.last_sequence_or_command_id,
        last_timestamp_us: u64::from(snapshot.last_timestamp.get()),
        last_reason: snapshot.last_reason,
    }
}

fn snapshot_output_counters(counters: OutputAssemblyCounters) -> HostOutputAssemblyCounters {
    HostOutputAssemblyCounters {
        output_intent: snapshot_output_stage(counters.output_intent),
        output_planner: snapshot_output_stage(counters.output_planner),
        output_admission: snapshot_output_stage(counters.output_admission),
        output_armer: snapshot_output_stage(counters.output_armer),
        output_executor: snapshot_output_stage(counters.output_executor),
        output_observer: snapshot_output_stage(counters.output_observer),
    }
}

fn snapshot_output_stage(snapshot: OutputStageSnapshot) -> TelemetryStageCounterSnapshot {
    TelemetryStageCounterSnapshot {
        seen: snapshot.seen,
        accepted: snapshot.accepted,
        rejected: snapshot.rejected,
        dropped: snapshot.dropped,
        stale: snapshot.stale,
        overrun: snapshot.overrun,
        late: snapshot.late,
        last_trace_id: snapshot.last_trace_id.get(),
        last_sequence_or_command_id: snapshot.last_sequence_or_command_id,
        last_timestamp_us: u64::from(snapshot.last_timestamp.get()),
        last_reason: snapshot.last_reason,
    }
}

fn counter_regressed(previous: u32, current: u32, path: &str) -> Option<String> {
    (current < previous).then_some(format!(
        "{path} regressed: previous={previous} current={current}"
    ))
}

fn find_counter_regressions(
    domain: &str,
    stage: &str,
    previous: &TelemetryStageCounterSnapshot,
    current: &TelemetryStageCounterSnapshot,
) -> Vec<String> {
    let mut failures = Vec::new();
    macro_rules! push_if {
        ($field:ident) => {
            if let Some(failure) = counter_regressed(
                previous.$field as u32,
                current.$field as u32,
                &format!("{domain}.{stage}.{}", stringify!($field)),
            ) {
                failures.push(failure);
            }
        };
    }

    push_if!(seen);
    push_if!(accepted);
    push_if!(rejected);
    push_if!(dropped);
    push_if!(stale);
    push_if!(overrun);
    push_if!(late);
    failures
}

fn write_host_telemetry(run_dir: &Path, artifact: &HostRuntimeTelemetryArtifact) -> io::Result<()> {
    let path = run_dir.join(HOST_TELEMETRY_ARTIFACT);
    let file = File::create(&path)?;
    serde_json::to_writer_pretty(file, artifact).map_err(io::Error::other)?;
    Ok(())
}

fn read_process_memory(pid: u32) -> io::Result<ProcessMemory> {
    let status = fs::read_to_string(format!("/proc/{pid}/status"))?;
    let mut memory = ProcessMemory::default();
    for line in status.lines() {
        if let Some(value) = parse_status_kb(line, "VmRSS:") {
            memory.vm_rss_kb = value;
        } else if let Some(value) = parse_status_kb(line, "VmHWM:") {
            memory.vm_hwm_kb = value;
        } else if let Some(value) = parse_status_kb(line, "VmSize:") {
            memory.vm_size_kb = value;
        }
    }
    Ok(memory)
}

fn parse_status_kb(line: &str, label: &str) -> Option<u64> {
    let rest = line.strip_prefix(label)?;
    rest.split_whitespace().next()?.parse().ok()
}

fn write_report(config: &Config, run_dir: &Path, summary: SoakSummary) -> io::Result<()> {
    let mut report = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(run_dir.join("report.md"))?;
    let memory_growth_kb = summary.baseline.map_or(0, |base| {
        summary.last.vm_rss_kb.saturating_sub(base.vm_rss_kb)
    });
    writeln!(report, "# QEMU Soak Report")?;
    writeln!(report)?;
    writeln!(report, "- result: {}", summary.reason.as_str())?;
    writeln!(report, "- stop_kind: {}", stop_kind(summary.reason))?;
    writeln!(report, "- conclusion: {}", conclusion(summary.reason))?;
    writeln!(report, "- elapsed_secs: {}", summary.elapsed.as_secs())?;
    writeln!(report, "- samples: {}", summary.sample_count)?;
    writeln!(report, "- firmware: {}", config.firmware.display())?;
    writeln!(report, "- map: {}", config.map.display())?;
    writeln!(report, "- warmup_samples: {}", config.warmup_samples)?;
    match (summary.baseline, summary.baseline_sample) {
        (Some(base), Some(sample)) => {
            writeln!(report, "- rss_baseline_status: active")?;
            writeln!(report, "- rss_baseline_sample: {sample}")?;
            writeln!(report, "- baseline_vm_rss_kb: {}", base.vm_rss_kb)?;
        }
        _ => {
            writeln!(
                report,
                "- rss_baseline_status: unavailable: fewer samples than warmup"
            )?;
            writeln!(report, "- rss_baseline_sample: unavailable")?;
            writeln!(report, "- baseline_vm_rss_kb: unavailable")?;
        }
    }
    writeln!(report, "- final_vm_rss_kb: {}", summary.last.vm_rss_kb)?;
    writeln!(report, "- vm_rss_growth_kb: {memory_growth_kb}")?;
    writeln!(report, "- final_vm_hwm_kb: {}", summary.last.vm_hwm_kb)?;
    writeln!(
        report,
        "- firmware_runtime_telemetry: unavailable: external Speeduino firmware telemetry is opaque"
    )?;
    writeln!(report, "- pin_drive_enabled: {}", summary.pin_drive.enabled)?;
    writeln!(
        report,
        "- host_runtime_telemetry: available: host_telemetry.json"
    )?;
    writeln!(
        report,
        "- host_telemetry_artifact: {}",
        summary.host.artifact
    )?;
    writeln!(report, "- pin_drive_events: {}", summary.pin_drive.events)?;
    writeln!(
        report,
        "- pin_drive_set_failures: {}",
        summary.pin_drive.set_failures
    )?;
    writeln!(
        report,
        "- pin_drive_readback_failures: {}",
        summary.pin_drive.readback_failures
    )?;
    writeln!(
        report,
        "- last_pin_external_level: {}",
        summary.pin_drive.last_level
    )?;
    writeln!(
        report,
        "- firmware_clock_monotonicity: unavailable: firmware telemetry not exposed"
    )?;
    writeln!(
        report,
        "- firmware_reset_count: unavailable: firmware telemetry not exposed"
    )?;
    writeln!(
        report,
        "- firmware_scheduler_queue_depth: unavailable: firmware telemetry not exposed"
    )?;
    writeln!(
        report,
        "- firmware_level0_final_state: control_mode=unavailable, fault_code=unavailable, fault_severity=unavailable, ignition_advance_deg10=unavailable, dwell_us=unavailable, injector_pulse_width_us=unavailable"
    )?;
    writeln!(
        report,
        "- firmware_level1_request_final: output_transitions=unavailable, aux_commands=unavailable"
    )?;
    writeln!(
        report,
        "- firmware_level2_stage_counters: unavailable: firmware telemetry not exposed"
    )?;
    writeln!(
        report,
        "- host_level0_final_state: {:?}",
        summary.host.final_telemetry
    )?;
    writeln!(
        report,
        "- host_level1_request_final: output_transitions={}, aux_commands={}, queue_depth_max={}, queue_depth_final={}",
        summary.host.output_transition_count,
        summary.host.aux_command_count,
        summary.host.runtime_queue_depth_max,
        summary.host.runtime_queue_depth_final
    )?;
    writeln!(
        report,
        "- host_level2_stage_counters: signal={:?}, output={:?}",
        summary.host.signal_counters, summary.host.output_counters
    )?;
    writeln!(
        report,
        "- host_clock_monotonicity_violations: {}",
        summary.host.clock_monotonicity_violations
    )?;
    writeln!(report, "- host_runtime_invariant_failures:")?;
    if summary.host.invariant_failures.is_empty() {
        writeln!(report, "  - none")?;
    } else {
        for failure in summary.host.invariant_failures {
            writeln!(report, "  - {failure}")?;
        }
    }
    writeln!(report, "- host_reset_count: {}", summary.host.reset_count)?;
    writeln!(report)?;
    writeln!(
        report,
        "Map handling is host-validated in this runner: the MSQ path is checked and reported, while live TunerStudio upload remains a follow-up."
    )?;
    Ok(())
}

fn stop_kind(reason: StopReason) -> &'static str {
    match reason {
        StopReason::Completed => "clean-duration-completion",
        StopReason::Interrupted => "operator-interrupt",
        StopReason::QemuExited(_) | StopReason::QemuSignaled => "qemu-process-stop",
        StopReason::MemoryGrowthExceeded => "memory-growth-stop",
        StopReason::HostStepPanicked => "host-runtime-step-stop",
        StopReason::HostStepError => "host-runtime-step-stop",
        StopReason::HostClockRollback => "host-runtime-clock-rollback",
        StopReason::HostSchedulerQueueDepthInvalid => "host-scheduler-queue-depth-stop",
        StopReason::HostInvariantFailure => "host-invariant-failure",
    }
}

fn conclusion(reason: StopReason) -> &'static str {
    match reason {
        StopReason::Completed => "no failure observed for configured duration",
        StopReason::Interrupted => "no failure observed before operator interruption",
        StopReason::QemuExited(_) => "failure observed: qemu exited",
        StopReason::QemuSignaled => "failure observed: qemu stopped by signal",
        StopReason::MemoryGrowthExceeded => "failure observed: memory growth exceeded limit",
        StopReason::HostStepPanicked => "failure observed: host runtime panicked while stepping",
        StopReason::HostStepError => "failure observed: host runtime step error",
        StopReason::HostClockRollback => "failure observed: host clock rollback",
        StopReason::HostSchedulerQueueDepthInvalid => {
            "failure observed: host scheduler queue depth exceeded safety bound"
        }
        StopReason::HostInvariantFailure => "failure observed: host runtime invariants failed",
    }
}

fn stop_qemu(child: &mut Child) {
    if matches!(child.try_wait(), Ok(Some(_))) {
        return;
    }
    let _ = child.kill();
    let _ = child.wait();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::{read_to_string, remove_dir_all};

    fn zero_host_signal_counters() -> HostSignalAssemblyCounters {
        HostSignalAssemblyCounters {
            signal_capture: TelemetryStageCounterSnapshot::default(),
            signal_normalizer: TelemetryStageCounterSnapshot::default(),
            observation_validator: TelemetryStageCounterSnapshot::default(),
            observation_publisher: TelemetryStageCounterSnapshot::default(),
            observation_reader: TelemetryStageCounterSnapshot::default(),
            runtime_snapshot_builder: TelemetryStageCounterSnapshot::default(),
            policy_consumer: TelemetryStageCounterSnapshot::default(),
        }
    }

    fn zero_host_output_counters() -> HostOutputAssemblyCounters {
        HostOutputAssemblyCounters {
            output_intent: TelemetryStageCounterSnapshot::default(),
            output_planner: TelemetryStageCounterSnapshot::default(),
            output_admission: TelemetryStageCounterSnapshot::default(),
            output_armer: TelemetryStageCounterSnapshot::default(),
            output_executor: TelemetryStageCounterSnapshot::default(),
            output_observer: TelemetryStageCounterSnapshot::default(),
        }
    }

    fn temporary_run_dir() -> PathBuf {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock available")
            .as_nanos();
        let path = std::env::temp_dir().join(format!("ecu-qemu-soak-report-test-{timestamp}"));
        fs::create_dir_all(&path).expect("temp test run dir");
        path
    }

    fn read_file(path: &Path) -> String {
        read_to_string(path).expect("read report/artifact")
    }

    fn report_summary() -> SoakSummary {
        SoakSummary {
            reason: StopReason::Completed,
            elapsed: Duration::from_secs(60),
            sample_count: 3,
            baseline: Some(ProcessMemory {
                vm_rss_kb: 1_000,
                vm_hwm_kb: 1_000,
                vm_size_kb: 1_000,
            }),
            baseline_sample: Some(1),
            last: ProcessMemory {
                vm_rss_kb: 1_100,
                vm_hwm_kb: 1_250,
                vm_size_kb: 10_000,
            },
            host: HostReportSummary {
                artifact: HOST_TELEMETRY_ARTIFACT.to_string(),
                runtime_queue_depth_max: 2,
                runtime_queue_depth_final: 1,
                reset_count: 0,
                output_transition_count: 12,
                aux_command_count: 4,
                clock_monotonicity_violations: 0,
                final_telemetry: Some(TelemetryFrameSummary {
                    control_mode: "Running".to_string(),
                    fault_code: "None".to_string(),
                    fault_severity: "None".to_string(),
                    ignition_advance_deg10: 123,
                    dwell_us: 200,
                    injector_pulse_width_us: 800,
                }),
                signal_counters: zero_host_signal_counters(),
                output_counters: zero_host_output_counters(),
                invariant_failures: vec!["none".to_string()],
            },
            pin_drive: PinDriveReportSummary {
                enabled: true,
                events: 6,
                set_failures: 1,
                readback_failures: 2,
                last_level: 0x0c,
            },
        }
    }

    #[test]
    fn host_clock_rollback_is_detected() {
        let mut failures = Vec::new();
        let mut rollback_count = 0usize;

        assert!(is_host_clock_monotonic(
            None,
            1_200,
            1,
            &mut failures,
            &mut rollback_count,
        ));
        assert_eq!(rollback_count, 0);
        assert!(failures.is_empty());

        assert!(!is_host_clock_monotonic(
            Some(1_200),
            1_000,
            2,
            &mut failures,
            &mut rollback_count,
        ));
        assert_eq!(rollback_count, 1);
        assert!(failures[0].contains("host clock rollback sample 2"));
    }

    #[test]
    fn counter_regression_ignores_latest_metadata_fields() {
        let previous = TelemetryStageCounterSnapshot {
            seen: 3,
            accepted: 2,
            last_trace_id: 100,
            last_sequence_or_command_id: 200,
            last_reason: 10,
            ..TelemetryStageCounterSnapshot::default()
        };
        let current = TelemetryStageCounterSnapshot {
            seen: 3,
            accepted: 2,
            last_trace_id: 4,
            last_sequence_or_command_id: 5,
            last_reason: 1,
            ..TelemetryStageCounterSnapshot::default()
        };

        let failures = find_counter_regressions("output", "output_planner", &previous, &current);

        assert!(failures.is_empty());
    }

    #[test]
    fn counter_regression_detects_counter_decrease() {
        let previous = TelemetryStageCounterSnapshot {
            seen: 3,
            ..TelemetryStageCounterSnapshot::default()
        };
        let current = TelemetryStageCounterSnapshot {
            seen: 2,
            ..TelemetryStageCounterSnapshot::default()
        };

        let failures = find_counter_regressions("output", "output_planner", &previous, &current);

        assert_eq!(failures.len(), 1);
        assert!(failures[0].contains("output.output_planner.seen regressed"));
    }

    #[test]
    fn pin_drive_counters_are_reported_in_summary() {
        let state = PinDriveSharedState::default();
        state.events.fetch_add(3, Ordering::Relaxed);
        state.set_failures.fetch_add(1, Ordering::Relaxed);
        state.readback_failures.fetch_add(2, Ordering::Relaxed);
        state.last_level.store(0x0a, Ordering::Relaxed);

        let snapshot = state.snapshot();
        assert_eq!(snapshot.events, 3);
        assert_eq!(snapshot.set_failures, 1);
        assert_eq!(snapshot.readback_failures, 2);
        assert_eq!(snapshot.last_level, 0x0a);
    }

    #[test]
    fn host_telemetry_artifact_is_written_and_reported() {
        let run_dir = temporary_run_dir();
        let artifact = HostRuntimeTelemetryArtifact {
            samples: vec![HostRuntimeTelemetrySample {
                sample_index: 1,
                sample_timestamp_ms: 1_000,
                host_clock_monotonic: true,
                scheduler_queue_depth: 2,
                signal_assembly_counters: zero_host_signal_counters(),
                output_assembly_counters: zero_host_output_counters(),
                output_transition_count: 6,
                aux_command_count: 1,
            }],
            host_reset_count: 0,
            scheduler_queue_depth_max: 2,
            scheduler_queue_depth_final: 2,
            final_telemetry: Some(TelemetryFrameSummary {
                control_mode: "Running".to_string(),
                fault_code: "None".to_string(),
                fault_severity: "None".to_string(),
                ignition_advance_deg10: 80,
                dwell_us: 210,
                injector_pulse_width_us: 900,
            }),
            signal_assembly_counters: zero_host_signal_counters(),
            output_assembly_counters: zero_host_output_counters(),
            output_transition_count: 6,
            aux_command_count: 1,
            clock_monotonicity_violations: 0,
            invariant_failures: vec!["none".to_string()],
        };

        write_host_telemetry(&run_dir, &artifact).expect("write host artifact");
        let artifact_path = run_dir.join(HOST_TELEMETRY_ARTIFACT);
        assert!(artifact_path.is_file());
        let artifact_text = read_file(&artifact_path);
        assert!(artifact_text.contains("\"host_reset_count\": 0"));
        assert!(artifact_text.contains("\"output_transition_count\": 6"));

        let config = Config {
            firmware: PathBuf::from("/tmp/firmware.hex"),
            map: PathBuf::from("/tmp/map.msq"),
            duration_secs: 3,
            sample_secs: 1,
            max_rss_growth_kb: 1024,
            warmup_samples: 0,
            run_root: PathBuf::from("/tmp/qemu-soak"),
            pin_smoke: false,
            pin_drive: true,
        };
        write_report(&config, &run_dir, report_summary()).expect("write report");

        let report = read_file(&run_dir.join("report.md"));
        assert!(report.contains("host_runtime_telemetry: available: host_telemetry.json"));
        assert!(report.contains("host_telemetry_artifact: host_telemetry.json"));
        assert!(report.contains("pin_drive_events: 6"));
        assert!(report.contains("pin_drive_set_failures: 1"));
        assert!(report.contains("pin_drive_readback_failures: 2"));
        assert!(report.contains("last_pin_external_level: 12"));

        remove_dir_all(run_dir).ok();
    }
}
