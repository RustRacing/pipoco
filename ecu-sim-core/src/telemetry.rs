use crate::{
    dyno::DynoFrame,
    sensors::SensorSnapshot,
    state::{CylinderPhysicsState, CylinderState, MisfireReason},
    types::*,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetryFrame<const CYL: usize> {
    pub timestamp_us: Micros,
    pub rpm: Rpm,
    pub crank_angle_deg10: CrankDeg10,
    pub torque_nm_x100: i32,
    pub horsepower_x100: i32,
    pub bmep_bar_x100: BmepBarX100,
    pub imep_bar_x100: ImepBarX100,
    pub pmep_bar_x100: PmepBarX100,
    pub fmep_bar_x100: FmepBarX100,
    pub indicated_torque_nm_x100: TorqueNmX100,
    pub brake_torque_nm_x100: TorqueNmX100,
    pub dyno_load_torque_nm_x100: TorqueNmX100,
    pub lambda_x1000: u16,
    pub afr_x100: u16,
    pub map_kpa10: Kpa10,
    pub tps_x1000: u16,
    pub battery_mv: Millivolts,
    pub ve_x1000: u16,
    pub egt_k_x10: u16,
    pub exhaust_manifold_temp_k_x10: u16,
    pub catalyst_temp_k_x10: u16,
    pub trapped_air_mass_ug: [MassUg; CYL],
    pub delivered_fuel_mass_ug: [MassUg; CYL],
    pub spark_advance_deg10: [i16; CYL],
    pub dwell_us: [Micros; CYL],
    pub injection_pw_us: [Micros; CYL],
    pub soi_deg10: [Option<CrankDeg10>; CYL],
    pub eoi_deg10: [Option<CrankDeg10>; CYL],
    pub combustion_quality_x1000: [u16; CYL],
    pub residual_fraction_x1000: [u16; CYL],
    pub knock_risk_x1000: [u16; CYL],
    pub misfire_flags: [bool; CYL],
    pub pmax_pa: [PressurePa; CYL],
    pub pmax_angle_deg10: [CrankDeg10; CYL],
    pub ca10_deg10: [Option<CrankDeg10>; CYL],
    pub ca50_deg10: [Option<CrankDeg10>; CYL],
    pub ca90_deg10: [Option<CrankDeg10>; CYL],
    pub diagnostic_event_count: u16,
    pub diagnostic_overflow_count: u16,
}

impl<const CYL: usize> TelemetryFrame<CYL> {
    pub const fn empty() -> Self {
        Self {
            timestamp_us: Micros(0),
            rpm: Rpm(0),
            crank_angle_deg10: CrankDeg10(0),
            torque_nm_x100: 0,
            horsepower_x100: 0,
            bmep_bar_x100: BmepBarX100(0),
            imep_bar_x100: ImepBarX100(0),
            pmep_bar_x100: PmepBarX100(0),
            fmep_bar_x100: FmepBarX100(0),
            indicated_torque_nm_x100: TorqueNmX100(0),
            brake_torque_nm_x100: TorqueNmX100(0),
            dyno_load_torque_nm_x100: TorqueNmX100(0),
            lambda_x1000: 1000,
            afr_x100: 1470,
            map_kpa10: Kpa10(0),
            tps_x1000: 0,
            battery_mv: Millivolts(0),
            ve_x1000: 0,
            egt_k_x10: 2930,
            exhaust_manifold_temp_k_x10: 2930,
            catalyst_temp_k_x10: 2930,
            trapped_air_mass_ug: [MassUg(0); CYL],
            delivered_fuel_mass_ug: [MassUg(0); CYL],
            spark_advance_deg10: [0; CYL],
            dwell_us: [Micros(0); CYL],
            injection_pw_us: [Micros(0); CYL],
            soi_deg10: [None; CYL],
            eoi_deg10: [None; CYL],
            combustion_quality_x1000: [0; CYL],
            residual_fraction_x1000: [0; CYL],
            knock_risk_x1000: [0; CYL],
            misfire_flags: [false; CYL],
            pmax_pa: [PressurePa(0); CYL],
            pmax_angle_deg10: [CrankDeg10(0); CYL],
            ca10_deg10: [None; CYL],
            ca50_deg10: [None; CYL],
            ca90_deg10: [None; CYL],
            diagnostic_event_count: 0,
            diagnostic_overflow_count: 0,
        }
    }

    pub fn from_state(
        timestamp_us: Micros,
        rpm: Rpm,
        dyno: DynoFrame,
        sensors: SensorSnapshot,
        cylinders: &[CylinderState; CYL],
        physics: &[CylinderPhysicsState; CYL],
        ve_x1000: u16,
    ) -> Self {
        let mut frame = Self::empty();
        frame.timestamp_us = timestamp_us;
        frame.rpm = rpm;
        frame.crank_angle_deg10 = sensors.crank_angle_deg10;
        frame.torque_nm_x100 = dyno.filtered_torque_nm_x100.0;
        frame.horsepower_x100 = dyno.horsepower_x100;
        frame.bmep_bar_x100 = dyno.bmep_bar_x100;
        frame.imep_bar_x100 = dyno.imep_bar_x100;
        frame.pmep_bar_x100 = dyno.pmep_bar_x100;
        frame.fmep_bar_x100 = dyno.fmep_bar_x100;
        frame.indicated_torque_nm_x100 = dyno.indicated_torque_nm_x100;
        frame.brake_torque_nm_x100 = dyno.brake_torque_nm_x100;
        frame.lambda_x1000 = sensors.lambda_x1000;
        frame.afr_x100 = (sensors.lambda_x1000 as u32 * 1470 / 1000) as u16;
        frame.map_kpa10 = sensors.map_kpa10;
        frame.tps_x1000 = sensors.tps_x1000;
        frame.battery_mv = sensors.battery_mv;
        frame.ve_x1000 = ve_x1000;

        let mut i = 0;
        while i < CYL {
            frame.trapped_air_mass_ug[i] = cylinders[i].air_mass_ug;
            frame.delivered_fuel_mass_ug[i] = cylinders[i].fuel_mass_ug;
            frame.spark_advance_deg10[i] = cylinders[i].last_spark_advance_deg10.0;
            frame.dwell_us[i] = cylinders[i].last_dwell_us;
            frame.injection_pw_us[i] = cylinders[i].last_injection_pw_us;
            frame.soi_deg10[i] = cylinders[i].last_soi_deg10;
            frame.eoi_deg10[i] = cylinders[i].last_eoi_deg10;
            frame.combustion_quality_x1000[i] = cylinders[i].combustion_quality_x1000;
            frame.residual_fraction_x1000[i] = physics[i].residual_fraction_x1000;
            frame.knock_risk_x1000[i] = cylinders[i].knock_risk_x1000;
            frame.misfire_flags[i] = is_misfire(cylinders[i].misfire);
            frame.pmax_pa[i] = physics[i].pmax_pa;
            frame.pmax_angle_deg10[i] = physics[i].pmax_angle_deg10;
            frame.ca10_deg10[i] = physics[i].ca10_deg10;
            frame.ca50_deg10[i] = physics[i].ca50_deg10;
            frame.ca90_deg10[i] = physics[i].ca90_deg10;
            i += 1;
        }

        frame
    }
}

pub fn is_misfire(reason: Option<MisfireReason>) -> bool {
    reason.is_some()
}
