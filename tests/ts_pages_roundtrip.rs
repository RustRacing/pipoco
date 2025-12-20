//! Verify TS page stores for sensors/AE/DFCO and fuel/ign persistence

use ecu_core::persist::KvStore;
use ecu_core::ts::pages::{
    EcuPageStore, EcuStatePageStore, PersistedPageStore, PAGE_AE, PAGE_DFCO, PAGE_SENSORS,
};
use ecu_core::ts::PageStore;
use ecu_target_common::kv::ram::RamKv512;
struct SharedKv {
    inner: std::rc::Rc<std::cell::RefCell<RamKv512>>,
}
impl SharedKv {
    fn new() -> Self {
        Self {
            inner: std::rc::Rc::new(std::cell::RefCell::new(RamKv512::new())),
        }
    }
}
impl Clone for SharedKv {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}
impl KvStore for SharedKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, ecu_core::persist::KvError> {
        self.inner.borrow_mut().read(key, out)
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), ecu_core::persist::KvError> {
        self.inner.borrow_mut().write(key, data)
    }
}
use ecu_core::EcuState;

#[test]
fn sensors_ae_dfco_read_write() {
    let mut state = EcuState::new();
    let mut pages = EcuPageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
        sens: &mut state.sensors_cal,
        idle: &mut state.idle_config,
        fan: &mut state.fan_config,
        cl: &mut state.cl_config,
        wue: &mut state.wue_config,
        ase: &mut state.ase_config,
        ae: &mut state.ae_config,
        dfco: &mut state.dfco_config,
        limits: &mut state.sensors_limits,
        emerg_trig_map: &mut state.emergency_trigger_map_oob,
        emerg_trig_tps: &mut state.emergency_trigger_tps_oob,
        diag_emergency: &state.emergency_mode,
        diag_map: &state.diag_map,
        diag_tps: &state.diag_tps,
        diag_cam: &state.diag_cam,
        diag_log: &state.diag_log,
        angles_inj: &mut state.inj_angle_btdc_x10,
        angles_tdc: &mut state.tdc_per_cyl_x10,
        tooth0_angle_x10: &mut state.tooth0_angle_x10,
        cam_timeout_ms: &mut state.cam_missing_timeout_ms,
    };

    // Build sensors page payload (128 bytes) with modified values
    let mut sdata = [0u8; 128];
    // tps min/max
    sdata[0..2].copy_from_slice(&100u16.to_le_bytes());
    sdata[2..4].copy_from_slice(&3900u16.to_le_bytes());
    // map v0/kpa0, v1/kpa1
    sdata[4..6].copy_from_slice(&600u16.to_le_bytes());
    sdata[6..8].copy_from_slice(&90u16.to_le_bytes());
    sdata[8..10].copy_from_slice(&4600u16.to_le_bytes());
    sdata[10..12].copy_from_slice(&2400u16.to_le_bytes());
    // leave the rest default; write and read back
    pages
        .write_page(PAGE_SENSORS, &sdata)
        .expect("write sensors");
    let mut out = [0u8; 128];
    let n = pages
        .read_page(PAGE_SENSORS, &mut out)
        .expect("read sensors");
    assert_eq!(n, 128);
    assert_eq!(&out[..12], &sdata[..12]);

    // Limits page (10 bytes)
    let mut ldata = [0u8; 10];
    ldata[0..2].copy_from_slice(&150u16.to_le_bytes()); // map_min
    ldata[2..4].copy_from_slice(&2800u16.to_le_bytes()); // map_max
    ldata[4] = 2; // tps_min
    ldata[5] = 98; // tps_max
    ldata[6..8].copy_from_slice(&2u16.to_le_bytes()); // clear_time_s
    ldata[8] = 0b11; // both triggers on
    ldata[9] = 0; // reserved
    pages.write_page(ecu_core::ts::pages::PAGE_LIMITS, &ldata).expect("write limits");
    let mut lread = [0u8; 10];
    let n = pages.read_page(ecu_core::ts::pages::PAGE_LIMITS, &mut lread).expect("read limits");
    assert_eq!(n, 10);
    assert_eq!(&lread[..], &ldata[..]);

    // AE page (16 bytes) write/read
    let mut aedata = [0u8; 16];
    aedata[0..2].copy_from_slice(&(-50i16).to_le_bytes()); // tpsdot_thresh
    aedata[2..4].copy_from_slice(&(25i16).to_le_bytes()); // mapdot_thresh
    aedata[4] = 15; // percent_gain
                    // pad one byte
    aedata[6..10].copy_from_slice(&(250u32).to_le_bytes()); // decay
    aedata[10..14].copy_from_slice(&(500u32).to_le_bytes()); // lockout
    pages.write_page(PAGE_AE, &aedata).expect("write ae");
    let mut aeread = [0u8; 16];
    let n = pages.read_page(PAGE_AE, &mut aeread).expect("read ae");
    assert_eq!(n, 16);
    assert_eq!(&aedata[..], &aeread[..]);

    // DFCO page (16 bytes) write/read
    let mut d = [0u8; 16];
    d[0] = 5; // tps_max_pct
    d[2..4].copy_from_slice(&80u16.to_le_bytes()); // map_max_kpa
    d[4..6].copy_from_slice(&1800u16.to_le_bytes()); // rpm_min
    d[6..8].copy_from_slice(&3500u16.to_le_bytes()); // rpm_max
    d[8..12].copy_from_slice(&(500u32).to_le_bytes()); // delay
    d[12..16].copy_from_slice(&(1000u32).to_le_bytes()); // resume hyst
    pages.write_page(PAGE_DFCO, &d).expect("write dfco");
    let mut dread = [0u8; 16];
    let n = pages.read_page(PAGE_DFCO, &mut dread).expect("read dfco");
    assert_eq!(n, 16);
    assert_eq!(&d[..], &dread[..]);

    // Idle page (6 bytes) write/read
    let mut idle = [0u8; 6];
    idle[0] = 1; // enable
    idle[1..3].copy_from_slice(&(450u16).to_le_bytes()); // 45.0%
    idle[3..5].copy_from_slice(&(100u16).to_le_bytes()); // 100Hz
    pages.write_page(ecu_core::ts::pages::PAGE_IDLE, &idle).expect("write idle");
    let mut idread = [0u8; 6];
    let n = pages.read_page(ecu_core::ts::pages::PAGE_IDLE, &mut idread).expect("read idle");
    assert_eq!(n, 6);
    assert_eq!(&idle[..], &idread[..]);

    // Fan page (6 bytes) write/read
    let mut fan = [0u8; 6];
    fan[0] = 1; // enable
    fan[1..3].copy_from_slice(&(95i16).to_le_bytes()); // on at 95C
    fan[3..5].copy_from_slice(&(90i16).to_le_bytes()); // off at 90C
    pages.write_page(ecu_core::ts::pages::PAGE_FAN, &fan).expect("write fan");
    let mut fnread = [0u8; 6];
    let n = pages.read_page(ecu_core::ts::pages::PAGE_FAN, &mut fnread).expect("read fan");
    assert_eq!(n, 6);
    assert_eq!(&fan[..], &fnread[..]);

    // CL page (8 bytes) write/read
    let mut cl = [0u8; 8];
    cl[0] = 1; // enable
    cl[2..4].copy_from_slice(&(147u16).to_le_bytes());
    cl[4..6].copy_from_slice(&(10u16).to_le_bytes());
    cl[6..8].copy_from_slice(&(2u16).to_le_bytes());
    pages.write_page(ecu_core::ts::pages::PAGE_CL, &cl).expect("write cl");
    let mut clread = [0u8; 8];
    let n = pages.read_page(ecu_core::ts::pages::PAGE_CL, &mut clread).expect("read cl");
    assert_eq!(n, 8);
    assert_eq!(&cl[..], &clread[..]);

    // WUE page (8 bytes) write/read
    let mut wue = [0u8; 8];
    wue[0] = 30; // max percent
    wue[1] = 0;  // min percent
    wue[2..4].copy_from_slice(&(-10i16).to_le_bytes()); // start_c
    wue[4..6].copy_from_slice(&(60i16).to_le_bytes());  // end_c
    pages.write_page(ecu_core::ts::pages::PAGE_WUE, &wue).expect("write wue");
    let mut wread = [0u8; 8];
    let n = pages.read_page(ecu_core::ts::pages::PAGE_WUE, &mut wread).expect("read wue");
    assert_eq!(n, 8);
    assert_eq!(&wue[..], &wread[..]);

    // ASE page (8 bytes) write/read
    let mut ase = [0u8; 8];
    ase[0] = 15; // percent
    ase[2..6].copy_from_slice(&(3000u32).to_le_bytes()); // taper 3s
    ase[6..8].copy_from_slice(&(1000u16).to_le_bytes()); // lockout 1s
    pages.write_page(ecu_core::ts::pages::PAGE_ASE, &ase).expect("write ase");
    let mut aread = [0u8; 8];
    let n = pages.read_page(ecu_core::ts::pages::PAGE_ASE, &mut aread).expect("read ase");
    assert_eq!(n, 8);
    assert_eq!(&ase[..], &aread[..]);
}

#[test]
fn sensors_ae_dfco_negative_sizes_and_ranges() {
    let mut state = EcuState::new();
    let mut pages = EcuPageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
        sens: &mut state.sensors_cal,
        idle: &mut state.idle_config,
        fan: &mut state.fan_config,
        cl: &mut state.cl_config,
        wue: &mut state.wue_config,
        ase: &mut state.ase_config,
        ae: &mut state.ae_config,
        dfco: &mut state.dfco_config,
        limits: &mut state.sensors_limits,
        emerg_trig_map: &mut state.emergency_trigger_map_oob,
        emerg_trig_tps: &mut state.emergency_trigger_tps_oob,
        diag_emergency: &state.emergency_mode,
        diag_map: &state.diag_map,
        diag_tps: &state.diag_tps,
        diag_cam: &state.diag_cam,
        diag_log: &state.diag_log,
        angles_inj: &mut state.inj_angle_btdc_x10,
        angles_tdc: &mut state.tdc_per_cyl_x10,
        tooth0_angle_x10: &mut state.tooth0_angle_x10,
        cam_timeout_ms: &mut state.cam_missing_timeout_ms,
    };

    // Limits page invalid cases
    let mut bad = [0u8; 10];
    // map_min=0 invalid
    bad[0..2].copy_from_slice(&0u16.to_le_bytes());
    bad[2..4].copy_from_slice(&2000u16.to_le_bytes());
    bad[4] = 1; // tps_min
    bad[5] = 50; // tps_max
    bad[6..8].copy_from_slice(&1u16.to_le_bytes());
    bad[8] = 0; bad[9] = 0;
    assert!(pages.write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad).is_err());

    // map_max <= map_min invalid
    bad[0..2].copy_from_slice(&100u16.to_le_bytes());
    bad[2..4].copy_from_slice(&100u16.to_le_bytes());
    assert!(pages.write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad).is_err());

    // tps_max > 100 invalid
    bad[2..4].copy_from_slice(&2000u16.to_le_bytes());
    bad[5] = 150;
    assert!(pages.write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad).is_err());

    // tps_max <= tps_min invalid
    bad[4] = 90; bad[5] = 80;
    assert!(pages.write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad).is_err());

    // clear_time_s == 0 invalid
    bad[4] = 1; bad[5] = 2;
    bad[6..8].copy_from_slice(&0u16.to_le_bytes());
    assert!(pages.write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad).is_err());

    // Wrong size payloads
    assert!(pages.write_page(PAGE_SENSORS, &[0u8; 10]).is_err());
    assert!(pages.write_page(PAGE_AE, &[0u8; 8]).is_err());
    assert!(pages.write_page(PAGE_DFCO, &[0u8; 8]).is_err());

    // Invalid ranges: TPS min >= max, MAP calibration not increasing
    let mut sdata = [0u8; 128];
    sdata[0..2].copy_from_slice(&4000u16.to_le_bytes()); // min
    sdata[2..4].copy_from_slice(&1000u16.to_le_bytes()); // max (invalid)
    assert!(pages.write_page(PAGE_SENSORS, &sdata).is_err());

    let mut sdata2 = [0u8; 128];
    sdata2[0..2].copy_from_slice(&100u16.to_le_bytes());
    sdata2[2..4].copy_from_slice(&200u16.to_le_bytes());
    sdata2[4..6].copy_from_slice(&2000u16.to_le_bytes()); // v0
    sdata2[6..8].copy_from_slice(&500u16.to_le_bytes()); // k0
    sdata2[8..10].copy_from_slice(&1500u16.to_le_bytes()); // v1 <= v0 (invalid)
    sdata2[10..12].copy_from_slice(&400u16.to_le_bytes()); // k1 <= k0
    assert!(pages.write_page(PAGE_SENSORS, &sdata2).is_err());

    // AE: percent_gain > 100
    let mut aedata = [0u8; 16];
    aedata[4] = 150; // percent
    assert!(pages.write_page(PAGE_AE, &aedata).is_err());

    // DFCO: tps_max > 100, rpm_min 0, rpm_max < rpm_min
    let mut dfco = [0u8; 16];
    dfco[0] = 150; // tps_max invalid
    assert!(pages.write_page(PAGE_DFCO, &dfco).is_err());
}

#[test]
fn angles_page_roundtrip() {
    let mut state = EcuState::new();
    let mut pages = EcuPageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
        sens: &mut state.sensors_cal,
        idle: &mut state.idle_config,
        fan: &mut state.fan_config,
        cl: &mut state.cl_config,
        wue: &mut state.wue_config,
        ase: &mut state.ase_config,
        ae: &mut state.ae_config,
        dfco: &mut state.dfco_config,
        limits: &mut state.sensors_limits,
        emerg_trig_map: &mut state.emergency_trigger_map_oob,
        emerg_trig_tps: &mut state.emergency_trigger_tps_oob,
        diag_emergency: &state.emergency_mode,
        diag_map: &state.diag_map,
        diag_tps: &state.diag_tps,
        diag_cam: &state.diag_cam,
        diag_log: &state.diag_log,
        angles_inj: &mut state.inj_angle_btdc_x10,
        angles_tdc: &mut state.tdc_per_cyl_x10,
        tooth0_angle_x10: &mut state.tooth0_angle_x10,
        cam_timeout_ms: &mut state.cam_missing_timeout_ms,
    };

    let mut ang = [0u8; 68];
    for i in 0..16u16 {
        let off = (i as usize) * 2;
        ang[off..off + 2].copy_from_slice(&(i * 10).to_le_bytes());
    }
    for i in 0..16u16 {
        let off = 32 + (i as usize) * 2;
        ang[off..off + 2].copy_from_slice(&(100 + i).to_le_bytes());
    }
    ang[64..66].copy_from_slice(&123u16.to_le_bytes());
    ang[66..68].copy_from_slice(&750u16.to_le_bytes()); // cam timeout ms
    pages
        .write_page(ecu_core::ts::pages::PAGE_ANGLES, &ang)
        .expect("write angles");
    let mut out = [0u8; 68];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_ANGLES, &mut out)
        .expect("read angles");
    assert_eq!(n, 68);
    assert_eq!(&out[..], &ang[..]);
}

#[test]
fn fuel_ign_persist_roundtrip() {
    let mut state = EcuState::new();
    // change a couple cells
    state.ipw_table[0][0] = 1234;
    state.ignition_table[0][0] = -7;

    let kv = SharedKv::new();
    {
        let mut store = PersistedPageStore::new(
            EcuStatePageStore {
                fuel: &mut state.ipw_table,
                ign: &mut state.ignition_table,
            },
            kv.clone(),
        );
        // Burn to KV
        store.burn().expect("burn");
    }
    // Reset state and load from KV using a fresh store (release prior borrows)
    state.ipw_table[0][0] = 0;
    state.ignition_table[0][0] = 0;
    let mut store2 = PersistedPageStore::new(
        EcuStatePageStore {
            fuel: &mut state.ipw_table,
            ign: &mut state.ignition_table,
        },
        kv.clone(),
    );
    store2.try_load();
    assert_eq!(state.ipw_table[0][0], 1234);
    assert_eq!(state.ignition_table[0][0], -7);
}
