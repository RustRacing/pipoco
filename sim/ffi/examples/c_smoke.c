#include <ecu_sim_ffi.h>
#include <stddef.h>
#include <stdio.h>

_Static_assert(sizeof(ecu_sim_status_t) == sizeof(int32_t), "status ABI size");
_Static_assert(sizeof(ecu_sim_inj_mode_t) == sizeof(int32_t),
               "inj mode ABI size");
_Static_assert(sizeof(ecu_sim_ign_mode_t) == sizeof(int32_t),
               "ign mode ABI size");
_Static_assert(sizeof(ecu_sim_output_kind_t) == sizeof(int32_t),
               "output kind ABI size");
_Static_assert(sizeof(ecu_sim_init_cfg_t) == 64, "init cfg ABI size");
_Static_assert(offsetof(ecu_sim_init_cfg_t, cylinders) == 0,
               "init cfg cylinders offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, has_cam) == 1,
               "init cfg has_cam offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, inj_mode) == 4,
               "init cfg inj_mode offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, ign_mode) == 8,
               "init cfg ign_mode offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, firing_len) == 12,
               "init cfg firing_len offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, firing_order) == 13,
               "init cfg firing_order offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, inj_count) == 29,
               "init cfg inj_count offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, inj_channels) == 30,
               "init cfg inj_channels offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, ign_count) == 46,
               "init cfg ign_count offset");
_Static_assert(offsetof(ecu_sim_init_cfg_t, ign_channels) == 47,
               "init cfg ign_channels offset");
_Static_assert(sizeof(ecu_sim_sensor_frame_t) == 32, "sensor frame ABI size");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, now_us) == 0,
               "sensor frame now_us offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, map_kpa10) == 4,
               "sensor frame map_kpa10 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, tps_x100) == 6,
               "sensor frame tps_x100 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, clt_c10) == 8,
               "sensor frame clt_c10 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, iat_c10) == 10,
               "sensor frame iat_c10 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, vbatt_mv) == 12,
               "sensor frame vbatt_mv offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, baro_kpa10) == 14,
               "sensor frame baro_kpa10 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, lambda_x100) == 16,
               "sensor frame lambda_x100 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, lambda_valid) == 18,
               "sensor frame lambda_valid offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, knock_x100) == 20,
               "sensor frame knock_x100 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, vehicle_speed_kph10) == 22,
               "sensor frame vehicle_speed_kph10 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, maf_x100) == 24,
               "sensor frame maf_x100 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, cam_phase_deg10) == 26,
               "sensor frame cam_phase_deg10 offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, cam_phase_valid) == 28,
               "sensor frame cam_phase_valid offset");
_Static_assert(offsetof(ecu_sim_sensor_frame_t, validity_flags) == 29,
               "sensor frame validity_flags offset");
_Static_assert(ECU_SIM_SENSOR_VALID_MAF == (uint8_t)(1u << 0),
               "sensor validity MAF bit");
_Static_assert(ECU_SIM_SENSOR_VALID_KNOCK == (uint8_t)(1u << 1),
               "sensor validity knock bit");
_Static_assert(ECU_SIM_SENSOR_VALID_VEHICLE_SPEED == (uint8_t)(1u << 2),
               "sensor validity VSS bit");
_Static_assert(ECU_SIM_SENSOR_VALID_LAMBDA == (uint8_t)(1u << 3),
               "sensor validity lambda bit");
_Static_assert(sizeof(ecu_sim_output_event_t) == 16, "output event ABI size");
_Static_assert(offsetof(ecu_sim_output_event_t, time_us) == 0,
               "output event time_us offset");
_Static_assert(offsetof(ecu_sim_output_event_t, channel) == 4,
               "output event channel offset");
_Static_assert(offsetof(ecu_sim_output_event_t, kind) == 8,
               "output event kind offset");
_Static_assert(offsetof(ecu_sim_output_event_t, high) == 12,
               "output event high offset");
_Static_assert(sizeof(ecu_sim_snapshot_t) == 40, "snapshot ABI size");
_Static_assert(offsetof(ecu_sim_snapshot_t, now_us) == 0,
               "snapshot now_us offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, rpm) == 4,
               "snapshot rpm offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, synced) == 6,
               "snapshot synced offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, tooth) == 7,
               "snapshot tooth offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, angle_x10) == 8,
               "snapshot angle_x10 offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, load_kpa10) == 10,
               "snapshot load_kpa10 offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, output_overflow_count) == 12,
               "snapshot overflow offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, engine_phase) == 16,
               "snapshot engine_phase offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, fault_code) == 17,
               "snapshot fault_code offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, fault_severity) == 18,
               "snapshot fault_severity offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, cancel_reason) == 19,
               "snapshot cancel_reason offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, control_mode) == 20,
               "snapshot control_mode offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, fuel_pulse_width_us) == 22,
               "snapshot fuel pulse offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, ignition_advance_x10) == 24,
               "snapshot ignition advance offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, dwell_us) == 26,
               "snapshot dwell offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, lambda_target_x100) == 28,
               "snapshot lambda target offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, torque_limit_x100) == 30,
               "snapshot torque limit offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, rev_soft_active) == 32,
               "snapshot rev_soft_active offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, rev_hard_active) == 33,
               "snapshot rev_hard_active offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, launch_active) == 34,
               "snapshot launch_active offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, flat_shift_active) == 35,
               "snapshot flat_shift_active offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, fuel_cut) == 36,
               "snapshot fuel_cut offset");
_Static_assert(offsetof(ecu_sim_snapshot_t, spark_cut) == 37,
               "snapshot spark_cut offset");

int main(void) {
  ecu_sim_init_cfg_t cfg = {0};
  cfg.cylinders = 4;
  cfg.has_cam = 1;
  cfg.inj_mode = ECU_SIM_INJ_BATCH;
  cfg.ign_mode = ECU_SIM_IGN_WASTED;
  cfg.firing_len = 4;
  cfg.firing_order[0] = 1;
  cfg.firing_order[1] = 3;
  cfg.firing_order[2] = 4;
  cfg.firing_order[3] = 2;
  cfg.inj_count = 1;
  cfg.inj_channels[0] = 0;
  cfg.ign_count = 1;
  cfg.ign_channels[0] = 1;

  if (ecu_sim_init(&cfg) != ECU_SIM_OK) {
    return 1;
  }

  ecu_sim_sensor_frame_t sensors = {0};
  sensors.now_us = 1000;
  sensors.map_kpa10 = 900;
  sensors.tps_x100 = 2000;
  sensors.clt_c10 = 850;
  sensors.iat_c10 = 300;
  sensors.vbatt_mv = 13800;
  sensors.baro_kpa10 = 1013;
  sensors.lambda_x100 = 100;
  sensors.lambda_valid = 1;

  ecu_sim_output_event_t events[ECU_SIM_MAX_EVENTS] = {0};
  (void)ecu_sim_set_sensors(&sensors);
  (void)ecu_sim_on_crank_edge(1000);
  (void)ecu_sim_on_crank_edge(1500);
  (void)ecu_sim_on_cam_edge(1500);
  (void)ecu_sim_step(2000);

  ecu_sim_snapshot_t snapshot = {0};
  if (ecu_sim_snapshot(&snapshot) != ECU_SIM_OK) {
    return 2;
  }

  printf(
      "snapshot now_us=%u rpm=%u synced=%u tooth=%u phase=%u mode=%u fault=%u sev=%u cancel=%u\n",
      (unsigned)snapshot.now_us,
      (unsigned)snapshot.rpm,
      (unsigned)snapshot.synced,
      (unsigned)snapshot.tooth,
      (unsigned)snapshot.engine_phase,
      (unsigned)snapshot.control_mode,
      (unsigned)snapshot.fault_code,
      (unsigned)snapshot.fault_severity,
      (unsigned)snapshot.cancel_reason);

  if (snapshot.now_us != 2000 || snapshot.rpm != 2000 || snapshot.synced != 1 ||
      snapshot.tooth != 2 || snapshot.engine_phase != 2 ||
      snapshot.control_mode != 1 || snapshot.fault_code != 0 ||
      snapshot.fault_severity != 0 || snapshot.cancel_reason != 0) {
    return 3;
  }

  size_t copied = ecu_sim_dequeue_events(events, ECU_SIM_MAX_EVENTS);
  if (copied < 4) {
    return 4;
  }

  return 0;
}
