#include <ecu_sim_ffi.h>
#include <stddef.h>
#include <stdio.h>

static ecu_sim_init_cfg_t make_cfg(void) {
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
  cfg.inj_channels[0] = 7;
  cfg.ign_count = 1;
  cfg.ign_channels[0] = 9;
  return cfg;
}

static ecu_sim_sensor_frame_t make_sensors(uint32_t now_us, uint16_t map_kpa10,
                                           uint16_t tps_x100) {
  ecu_sim_sensor_frame_t sensors = {0};
  sensors.now_us = now_us;
  sensors.map_kpa10 = map_kpa10;
  sensors.tps_x100 = tps_x100;
  sensors.clt_c10 = 850;
  sensors.iat_c10 = 300;
  sensors.vbatt_mv = 13800;
  sensors.baro_kpa10 = 1013;
  sensors.lambda_x100 = 100;
  sensors.lambda_valid = 1;
  return sensors;
}

static int drive_singleton(ecu_sim_snapshot_t *snapshot,
                           ecu_sim_output_event_t *events, size_t *copied) {
  ecu_sim_init_cfg_t cfg = make_cfg();
  ecu_sim_sensor_frame_t sensors = make_sensors(1000, 900, 2000);

  ecu_sim_reset();
  if (ecu_sim_init(&cfg) != ECU_SIM_OK) {
    return 10;
  }
  if (ecu_sim_set_sensors(&sensors) != ECU_SIM_OK) {
    return 11;
  }
  if (ecu_sim_set_time(1000) != ECU_SIM_OK) {
    return 12;
  }
  if (ecu_sim_on_crank_edge(1000) != ECU_SIM_OK) {
    return 13;
  }
  if (ecu_sim_on_crank_edge(1500) != ECU_SIM_OK) {
    return 14;
  }
  if (ecu_sim_on_cam_edge(1500) != ECU_SIM_OK) {
    return 15;
  }
  if (ecu_sim_step(2000) != ECU_SIM_OK) {
    return 16;
  }
  if (ecu_sim_snapshot(snapshot) != ECU_SIM_OK) {
    return 17;
  }
  *copied = ecu_sim_dequeue_events(events, ECU_SIM_MAX_EVENTS);
  return 0;
}

static int drive_handle(ecu_sim_handle_t *handle, ecu_sim_snapshot_t *snapshot,
                        ecu_sim_output_event_t *events, size_t *copied) {
  ecu_sim_init_cfg_t cfg = make_cfg();
  ecu_sim_sensor_frame_t sensors = make_sensors(1000, 900, 2000);

  if (ecu_sim_handle_init(handle, &cfg) != ECU_SIM_OK) {
    return 20;
  }
  if (ecu_sim_handle_set_sensors(handle, &sensors) != ECU_SIM_OK) {
    return 21;
  }
  if (ecu_sim_handle_set_time(handle, 1000) != ECU_SIM_OK) {
    return 22;
  }
  if (ecu_sim_handle_on_crank_edge(handle, 1000) != ECU_SIM_OK) {
    return 23;
  }
  if (ecu_sim_handle_on_crank_edge(handle, 1500) != ECU_SIM_OK) {
    return 24;
  }
  if (ecu_sim_handle_on_cam_edge(handle, 1500) != ECU_SIM_OK) {
    return 25;
  }
  if (ecu_sim_handle_step(handle, 2000) != ECU_SIM_OK) {
    return 26;
  }
  if (ecu_sim_handle_snapshot(handle, snapshot) != ECU_SIM_OK) {
    return 27;
  }
  *copied = ecu_sim_handle_dequeue_events(handle, events, ECU_SIM_MAX_EVENTS);
  return 0;
}

int main(void) {
  ecu_sim_handle_t *handle = ecu_sim_handle_create();
  ecu_sim_handle_t *other = ecu_sim_handle_create();
  ecu_sim_init_cfg_t other_cfg = make_cfg();
  ecu_sim_snapshot_t handle_snapshot = {0};
  ecu_sim_snapshot_t singleton_snapshot = {0};
  ecu_sim_snapshot_t reset_snapshot = {0};
  ecu_sim_output_event_t handle_events[ECU_SIM_MAX_EVENTS] = {0};
  ecu_sim_output_event_t singleton_events[ECU_SIM_MAX_EVENTS] = {0};
  size_t handle_copied = 0;
  size_t singleton_copied = 0;

  if (handle == NULL || other == NULL) {
    return 1;
  }
  if (ecu_sim_handle_reset(NULL) != ECU_SIM_ERR_INVALID) {
    return 2;
  }
  if (ecu_sim_handle_snapshot(NULL, &reset_snapshot) != ECU_SIM_ERR_INVALID) {
    return 3;
  }
  if (ecu_sim_handle_dequeue_events(NULL, handle_events, ECU_SIM_MAX_EVENTS) != 0) {
    return 4;
  }

  if (drive_handle(handle, &handle_snapshot, handle_events, &handle_copied) != 0) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 5;
  }
  if (drive_singleton(&singleton_snapshot, singleton_events, &singleton_copied) != 0) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 6;
  }

  if (ecu_sim_handle_init(other, &other_cfg) != ECU_SIM_OK) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 7;
  }
  if (ecu_sim_handle_dequeue_events(other, handle_events, ECU_SIM_MAX_EVENTS) != 0) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 8;
  }

  if (handle_snapshot.now_us != singleton_snapshot.now_us ||
      handle_snapshot.rpm != singleton_snapshot.rpm ||
      handle_snapshot.synced != singleton_snapshot.synced ||
      handle_snapshot.tooth != singleton_snapshot.tooth ||
      handle_snapshot.engine_phase != singleton_snapshot.engine_phase ||
      handle_copied != singleton_copied) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 9;
  }
  if (handle_copied == 0 ||
      handle_events[0].kind != singleton_events[0].kind ||
      handle_events[0].channel != singleton_events[0].channel) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 10;
  }

  if (ecu_sim_handle_reset(handle) != ECU_SIM_OK) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 11;
  }
  if (ecu_sim_handle_snapshot(handle, &reset_snapshot) != ECU_SIM_ERR_NOT_INIT) {
    ecu_sim_handle_destroy(handle);
    ecu_sim_handle_destroy(other);
    return 12;
  }

  printf("handle snapshot now_us=%u rpm=%u synced=%u\n", (unsigned)handle_snapshot.now_us,
         (unsigned)handle_snapshot.rpm, (unsigned)handle_snapshot.synced);
  printf("singleton snapshot now_us=%u rpm=%u synced=%u\n",
         (unsigned)singleton_snapshot.now_us, (unsigned)singleton_snapshot.rpm,
         (unsigned)singleton_snapshot.synced);
  printf("events copied: handle=%u singleton=%u\n", (unsigned)handle_copied,
         (unsigned)singleton_copied);
  printf("reset snapshot status verified\n");

  /* Handles are opaque capability tokens; stale or destroyed ones return ECU_SIM_ERR_INVALID. */
  ecu_sim_handle_destroy(handle);
  if (ecu_sim_handle_reset(handle) != ECU_SIM_ERR_INVALID) {
    ecu_sim_handle_destroy(other);
    return 13;
  }
  ecu_sim_handle_destroy(handle);
  ecu_sim_handle_destroy(other);
  ecu_sim_reset();
  return 0;
}
