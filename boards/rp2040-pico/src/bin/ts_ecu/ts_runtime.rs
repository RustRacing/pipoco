use core::cell::RefCell;
use core::convert::Infallible;

use cortex_m::interrupt::Mutex;
use ecu_calibration::configs::EcuConfig;
use ecu_control::{AccelerationState, AfterStartState, CrankingGate, DecelFuelCutState};
use ecu_domain::diag::{self, DiagLog, DiagState};
use ecu_domain::{Degrees10, Kpa10, Lambda100, Micros, Rpm};
use ecu_target_common::sensors::adc_pipeline::{
    clamp_slew_u16, clamp_slew_u8, convert_all as ts_convert_all, AdcConfig as TsAdcConfig,
    RawCounts as TsRawCounts, ThermistorBias,
};
use ecu_target_common::{
    control_inputs::{SplitControlSignals, SplitControlSignalsSource},
    sensor_sample::LoadKpa10Source,
};
use ecu_ts::pages::{ExpertTriggerPageState, SystemSnapshot, DIAG_LOG_ENTRY_COUNT};
use embedded_hal::adc::OneShot;
use hal::adc::AdcPin;
use hal::pac;
use rp2040_hal as hal;

#[derive(Copy, Clone)]
pub(crate) struct RpTime;
impl RpTime {
    pub(crate) fn micros(&self) -> u32 {
        unsafe { &*pac::TIMER::ptr() }.timerawl.read().bits()
    }
}
impl ecu_board_api::EcuClock for RpTime {
    fn now_us(&self) -> Micros {
        Micros::new(self.micros())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Rp2040MapLoad;

impl LoadKpa10Source for Rp2040MapLoad {
    type Error = Infallible;

    fn load_kpa10(&mut self) -> Result<Kpa10, Self::Error> {
        let load_kpa10 = with_main_state(|s| s.sens.map_kpa_x10);
        Ok(Kpa10::new(load_kpa10))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Rp2040ControlSignals;

impl SplitControlSignalsSource for Rp2040ControlSignals {
    type Error = Infallible;

    fn signals(&mut self, ignition_rpm: Rpm) -> Result<SplitControlSignals, Self::Error> {
        let (clt_c, lambda_valid, lambda_x100, mapdot_kpa_s, tpsdot_pct_s) = with_main_state(|s| {
            (
                s.sens.clt_c,
                s.sens.lambda_valid,
                s.sens.lambda_x100,
                s.sens.mapdot_kpa_s,
                s.sens.tpsdot_pct_s,
            )
        });
        Ok(SplitControlSignals {
            clt_c,
            lambda_valid,
            measured_lambda100: Lambda100::new(lambda_x100),
            requested_open_loop: false,
            tpsdot_pct_s,
            mapdot_kpa_s,
            spark_advance_x10: Degrees10::new(100),
            ignition_rpm,
        })
    }
}

#[derive(Default)]
pub(crate) struct ClRuntime {
    pub(crate) integ: i32,
}

impl ClRuntime {
    pub(crate) const fn new() -> Self {
        Self { integ: 0 }
    }
}

// CdcSerial is provided by ecu-target-common
pub(crate) struct IdlePwmRuntime {
    pub(crate) last_start_us: u32,
    pub(crate) pin_is_high: bool,
}

impl IdlePwmRuntime {
    pub(crate) const fn new() -> Self {
        Self {
            last_start_us: 0,
            pin_is_high: false,
        }
    }
}

pub(crate) struct Sensors {
    pub(crate) map_kpa_x10: u16,
    pub(crate) tps_percent: u8,
    pub(crate) clt_c: i16,
    pub(crate) iat_c: i16,
    pub(crate) vbatt_mv: u16,
    pub(crate) lambda_valid: bool,
    pub(crate) lambda_x100: u16,
    pub(crate) mapdot_kpa_s: i16,
    pub(crate) tpsdot_pct_s: i16,
    pub(crate) last_map_kpa_x10: u16,
    pub(crate) last_tps_percent: u8,
    pub(crate) last_ts_us: u32,
}

impl Sensors {
    pub(crate) const fn new() -> Self {
        Self {
            map_kpa_x10: 1000,
            tps_percent: 0,
            clt_c: 20,
            iat_c: 25,
            vbatt_mv: 12000,
            lambda_valid: false,
            lambda_x100: 100,
            mapdot_kpa_s: 0,
            tpsdot_pct_s: 0,
            last_map_kpa_x10: 1000,
            last_tps_percent: 0,
            last_ts_us: 0,
        }
    }

    pub(crate) fn update(
        &mut self,
        adc: &mut hal::adc::Adc,
        pins: &mut AdcPins,
        cal: &ecu_calibration::sensors::SensorsCal,
        state: &mut BoardEcuState,
    ) {
        // Read raw counts
        let iat_counts = adc.read(&mut pins.iat).unwrap_or(0);
        let raw = TsRawCounts {
            map: adc.read(&mut pins.map).unwrap_or(0),
            maf: None,
            tps: adc.read(&mut pins.tps).unwrap_or(0),
            clt: adc.read(&mut pins.clt).unwrap_or(0),
            iat: iat_counts,
            // Reuse IAT channel for VBATT when using VSYS/3
            vbatt: iat_counts,
            lambda: None,
            baro: None,
        };

        // When vbatt-vsys is enabled, compute VBATT from VSYS/3 (scale x3).
        // Otherwise, disable VBATT conversion (num=0) and keep previous vbatt_mv.
        #[cfg(feature = "vbatt-vsys")]
        let cfg = TsAdcConfig {
            vref_mv: 3300,
            adc_bits: 12,
            vbatt_scale_num: 3,
            vbatt_scale_den: 1,
            clt_bias: ThermistorBias::PULLUP_2490,
            iat_bias: ThermistorBias::PULLUP_2490,
            lambda_cal:
                ecu_target_common::sensors::adc_pipeline::LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
            baro_cal:
                ecu_target_common::sensors::adc_pipeline::BaroAdcCalibration::LINEAR_0V5_TO_4V5,
        };
        #[cfg(not(feature = "vbatt-vsys"))]
        let cfg = TsAdcConfig {
            vref_mv: 3300,
            adc_bits: 12,
            vbatt_scale_num: 0,
            vbatt_scale_den: 1,
            clt_bias: ThermistorBias::PULLUP_2490,
            iat_bias: ThermistorBias::PULLUP_2490,
            lambda_cal:
                ecu_target_common::sensors::adc_pipeline::LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
            baro_cal:
                ecu_target_common::sensors::adc_pipeline::BaroAdcCalibration::LINEAR_0V5_TO_4V5,
        };
        let out = ts_convert_all(cfg, cal, raw);
        // Clamp and update state diagnostics/emergency
        let now = self.micros();
        let (map_clamped, tps_clamped) = state.process_sensor_update(
            Micros::new(now),
            Kpa10::new(out.map_kpa_x10),
            out.tps_percent,
        );

        // Slew rate limits (conservative defaults): MAP 1000 kPa×10/s, TPS 300 %/s
        let dt_us = if self.last_ts_us == 0 {
            0
        } else {
            now.wrapping_sub(self.last_ts_us)
        };
        let map_slewed = clamp_slew_u16(self.last_map_kpa_x10, map_clamped.raw(), 1000, dt_us);
        let tps_slewed = clamp_slew_u8(self.last_tps_percent, tps_clamped, 300, dt_us);
        self.map_kpa_x10 = map_slewed;
        self.tps_percent = tps_slewed;
        self.clt_c = out.clt_c;
        #[cfg(not(feature = "vbatt-vsys"))]
        {
            self.iat_c = out.iat_c;
        }
        #[cfg(feature = "vbatt-vsys")]
        {
            self.vbatt_mv = out.vbatt_mv;
        }
        self.lambda_valid = out.lambda_valid;
        self.lambda_x100 = out.lambda_x100;

        // Derivatives based on time delta
        let rp = RpTime;
        let now = rp.micros();
        let dt_us = now.wrapping_sub(self.last_ts_us);
        if dt_us > 0 {
            let d_map_x10 = self.map_kpa_x10 as i32 - self.last_map_kpa_x10 as i32;
            let num = d_map_x10.saturating_mul(1_000_000); // scale to per-second
            let den = (10 * (dt_us as i32)).max(1);
            let mapdot = num / den; // kPa/s
            self.mapdot_kpa_s = mapdot.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

            let d_tps = self.tps_percent as i32 - self.last_tps_percent as i32;
            let num_tps = d_tps.saturating_mul(1_000_000);
            let tpsdot = num_tps / (dt_us as i32).max(1);
            self.tpsdot_pct_s = tpsdot.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

            self.last_map_kpa_x10 = self.map_kpa_x10;
            self.last_tps_percent = self.tps_percent;
            self.last_ts_us = now;
        } else if self.last_ts_us == 0 {
            self.last_map_kpa_x10 = self.map_kpa_x10;
            self.last_tps_percent = self.tps_percent;
            self.last_ts_us = now;
        }
    }

    pub(crate) fn micros(&self) -> u32 {
        RpTime.micros()
    }
}

pub(crate) struct AdcPins {
    pub(crate) map: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio26, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
    pub(crate) tps: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio27, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
    pub(crate) clt: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio28, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
    pub(crate) iat: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio29, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
}

pub(crate) const fn default_ecu_config() -> EcuConfig {
    use ecu_calibration::configs::{
        AeConfig, AseConfig, ClConfig, DfcoConfig, FanConfig, IdleConfig, LambdaConfig,
        LoadFailureConfig, PlausibilityConfig, RateConfig, RevLimiterConfig, SensorsLimits,
        WueConfig,
    };
    use ecu_calibration::sensors::SensorsCal;

    EcuConfig {
        ipw_table: [[1000; 16]; 16],
        ve_table: [[100; 16]; 16],
        afr_table: [[147; 16]; 16],
        required_fuel_us: 1000,
        injector_deadtime_us: 800,
        ve_load_source: 0,
        ignition_table: [[15; 16]; 16],
        sensors_cal: SensorsCal::default(),
        sensors_limits: SensorsLimits::default(),
        ae_config: AeConfig::DEFAULT,
        wue_config: WueConfig::DEFAULT,
        ase_config: AseConfig::DEFAULT,
        dfco_config: DfcoConfig::DEFAULT,
        idle_config: IdleConfig::DEFAULT,
        fan_config: FanConfig::DEFAULT,
        cl_config: ClConfig::DEFAULT,
        load_failure_config: LoadFailureConfig::DEFAULT,
        plausibility_config: PlausibilityConfig::DEFAULT,
        rate_config: RateConfig::DEFAULT,
        lambda_config: LambdaConfig::DEFAULT,
        rev_limiter_config: RevLimiterConfig::DEFAULT,
        inj_angle_btdc_x10: [0; 16],
        tdc_per_cyl_x10: [0; 16],
        tooth0_angle_x10: 0,
        cam_missing_timeout_ms: 500,
    }
}

/// Board-owned ECU calibration and diagnostic state.
///
/// Owns the rp2040 TS page surface: the persisted calibration ([`EcuConfig`]),
/// the diagnostic log, the expert-trigger page state, and the
/// sensor-range/emergency tracking that the legacy compatibility-core engine
/// state mutated. The served TS pages are byte-identical to the previous
/// page-store output.
pub(crate) struct BoardEcuState {
    pub(crate) config: EcuConfig,
    pub(crate) diag_log: DiagLog<DIAG_LOG_ENTRY_COUNT>,
    pub(crate) snapshot: SystemSnapshot,
    pub(crate) expert_trigger: ExpertTriggerPageState,
    pub(crate) o2_sensor: ecu_calibration::O2SensorType,
    pub(crate) emerg_trig_map: bool,
    pub(crate) emerg_trig_tps: bool,
    pub(crate) emergency_mode: bool,
    pub(crate) diag_map: DiagState,
    pub(crate) diag_tps: DiagState,
    pub(crate) tooth_count: u8,
    pub(crate) sync_loss_counter: u16,
}

impl BoardEcuState {
    pub(crate) fn new() -> Self {
        use ecu_target_common::ts::page_store::{build_system_snapshot, SnapshotInputs};
        Self {
            config: default_ecu_config(),
            diag_log: DiagLog::new(),
            snapshot: build_system_snapshot(SnapshotInputs {
                rpm: 0,
                synced: false,
                base_pw_us: 0,
                enrich_mult_x100: 100,
                stft_x10: 0,
                fuel_mult_x100: 100,
                final_pw: Micros::new(0),
                last_fault_code: 0,
                isr_count: 0,
                isr_max_us: 0,
                isr_avg_us: 0,
            }),
            expert_trigger: ExpertTriggerPageState::new(),
            o2_sensor: ecu_calibration::O2SensorType::Narrowband,
            emerg_trig_map: false,
            emerg_trig_tps: false,
            emergency_mode: false,
            diag_map: DiagState::new(),
            diag_tps: DiagState::new(),
            tooth_count: 0,
            sync_loss_counter: 0,
        }
    }

    pub(crate) fn emergency_mode(&self) -> bool {
        self.emergency_mode
    }

    /// Clamp sensor values, update diag states, and set/clear emergency mode.
    /// Returns (clamped_map_kpa_x10, clamped_tps_percent). Mirrors the legacy
    /// compatibility-core sensor-update behavior exactly.
    pub(crate) fn process_sensor_update(
        &mut self,
        now_us: Micros,
        raw_map_kpa_x10: Kpa10,
        raw_tps_percent: u8,
    ) -> (Kpa10, u8) {
        let lim = self.config.sensors_limits;
        let now_us = now_us.raw();
        let raw_map_kpa_x10 = raw_map_kpa_x10.raw();
        let map = raw_map_kpa_x10.clamp(lim.map_min_kpa_x10, lim.map_max_kpa_x10);
        let tps = raw_tps_percent.clamp(lim.tps_min_percent, lim.tps_max_percent);

        let map_oob =
            raw_map_kpa_x10 < lim.map_min_kpa_x10 || raw_map_kpa_x10 > lim.map_max_kpa_x10;
        if map_oob {
            if !self.diag_map.is_active() {
                self.diag_map.latch(Micros::new(now_us));
                self.diag_map.start_us = now_us;
                self.diag_map.in_range_since_us = 0;
                if self.emerg_trig_map {
                    self.emergency_mode = true;
                }
            }
        } else if self.diag_map.is_active() {
            if self.diag_map.in_range_since_us == 0 {
                self.diag_map.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_map.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_map.start_us);
                self.diag_map.total_us = self.diag_map.total_us.saturating_add(dur);
                let start_us = self.diag_map.start_us;
                self.diag_log.push(diag::DiagEvent {
                    code: diag::DiagCode::MapRange,
                    timestamp: Micros::new(now_us),
                    source: diag::DiagSource::Sensor,
                    context: Some(raw_map_kpa_x10 as u32),
                    start_us,
                    end_us: now_us,
                });
                self.diag_map.clear(Micros::new(now_us));
                self.diag_map = DiagState::new();
            }
        }

        let tps_oob =
            raw_tps_percent < lim.tps_min_percent || raw_tps_percent > lim.tps_max_percent;
        if tps_oob {
            if !self.diag_tps.is_active() {
                self.diag_tps.latch(Micros::new(now_us));
                self.diag_tps.start_us = now_us;
                self.diag_tps.in_range_since_us = 0;
                if self.emerg_trig_tps {
                    self.emergency_mode = true;
                }
            }
        } else if self.diag_tps.is_active() {
            if self.diag_tps.in_range_since_us == 0 {
                self.diag_tps.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_tps.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_tps.start_us);
                self.diag_tps.total_us = self.diag_tps.total_us.saturating_add(dur);
                let start_us = self.diag_tps.start_us;
                self.diag_log.push(diag::DiagEvent {
                    code: diag::DiagCode::TpsRange,
                    timestamp: Micros::new(now_us),
                    source: diag::DiagSource::Sensor,
                    context: Some(raw_tps_percent as u32),
                    start_us,
                    end_us: now_us,
                });
                self.diag_tps.clear(Micros::new(now_us));
                self.diag_tps = DiagState::new();
            }
        }

        if self.emergency_mode {
            let map_emerg_active = self.emerg_trig_map && self.diag_map.is_active();
            let tps_emerg_active = self.emerg_trig_tps && self.diag_tps.is_active();
            if !(map_emerg_active || tps_emerg_active) {
                self.emergency_mode = false;
            }
        }

        (Kpa10::new(map), tps)
    }
}

pub(crate) struct MainLoopState {
    pub(crate) sens: Sensors,
    pub(crate) ts_outpc_config: crate::ts_pages::Rp2040TsOutpcConfig,
    pub(crate) ae: AccelerationState,
    pub(crate) dfco: DecelFuelCutState,
    pub(crate) ase: AfterStartState,
    pub(crate) crank_gate: CrankingGate,
    pub(crate) cl: ClRuntime,
}

impl MainLoopState {
    const fn new() -> Self {
        Self {
            sens: Sensors::new(),
            ts_outpc_config: crate::ts_pages::Rp2040TsOutpcConfig::new(),
            ae: AccelerationState::new(),
            dfco: DecelFuelCutState::new(),
            ase: AfterStartState::new(),
            crank_gate: CrankingGate::new(),
            cl: ClRuntime::new(),
        }
    }
}

static MAIN_STATE: Mutex<RefCell<MainLoopState>> = Mutex::new(RefCell::new(MainLoopState::new()));

pub(crate) fn with_main_state<R>(f: impl FnOnce(&mut MainLoopState) -> R) -> R {
    cortex_m::interrupt::free(|cs| f(&mut MAIN_STATE.borrow(cs).borrow_mut()))
}
