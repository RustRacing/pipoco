#ifndef ECU_SIM_FFI_H
#define ECU_SIM_FFI_H

#include <stddef.h>
#include <stdint.h>

#if UINTPTR_MAX != UINT64_MAX
#error "ecu-sim-ffi handle capability tokens require a 64-bit host target"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ECU_SIM_MAX_CHANNELS 16
#define ECU_SIM_MAX_FIRING_ORDER 16
#define ECU_SIM_MAX_EVENTS 128

typedef int32_t ecu_sim_status_t;
#define ECU_SIM_OK ((ecu_sim_status_t)0)
#define ECU_SIM_ERR_INVALID ((ecu_sim_status_t)-1)
#define ECU_SIM_ERR_NOT_INIT ((ecu_sim_status_t)-2)
#define ECU_SIM_ERR_EVENT_OVERFLOW ((ecu_sim_status_t)-3)
#define ECU_SIM_ERR_BUFFER_TOO_SMALL ((ecu_sim_status_t)-4)

typedef int32_t ecu_sim_inj_mode_t;
#define ECU_SIM_INJ_BATCH ((ecu_sim_inj_mode_t)0)
#define ECU_SIM_INJ_SEQUENTIAL ((ecu_sim_inj_mode_t)1)

typedef int32_t ecu_sim_ign_mode_t;
#define ECU_SIM_IGN_WASTED ((ecu_sim_ign_mode_t)0)
#define ECU_SIM_IGN_SEQUENTIAL ((ecu_sim_ign_mode_t)1)

typedef struct {
  uint8_t cylinders;
  uint8_t has_cam;
  ecu_sim_inj_mode_t inj_mode;
  ecu_sim_ign_mode_t ign_mode;
  uint8_t firing_len;
  uint8_t firing_order[ECU_SIM_MAX_FIRING_ORDER];
  uint8_t inj_count;
  uint8_t inj_channels[ECU_SIM_MAX_CHANNELS];
  uint8_t ign_count;
  uint8_t ign_channels[ECU_SIM_MAX_CHANNELS];
} ecu_sim_init_cfg_t;

typedef struct {
  uint32_t now_us;
  uint16_t map_kpa10;
  uint16_t tps_x100;
  int16_t clt_c10;
  int16_t iat_c10;
  uint16_t vbatt_mv;
  uint16_t baro_kpa10;
  uint16_t lambda_x100;
  uint8_t lambda_valid;
  uint16_t knock_x100;
} ecu_sim_sensor_frame_t;

typedef int32_t ecu_sim_output_kind_t;
#define ECU_SIM_OUT_INJECTOR ((ecu_sim_output_kind_t)0)
#define ECU_SIM_OUT_IGNITION ((ecu_sim_output_kind_t)1)
#define ECU_SIM_OUT_IDLE ((ecu_sim_output_kind_t)2)
#define ECU_SIM_OUT_FAN ((ecu_sim_output_kind_t)3)

typedef struct {
  uint32_t time_us;
  uint8_t channel;
  ecu_sim_output_kind_t kind;
  uint8_t high;
} ecu_sim_output_event_t;

typedef struct {
  uint32_t now_us;
  uint16_t rpm;
  uint8_t synced;
  uint8_t tooth;
  int16_t angle_x10;
  uint16_t load_kpa10;
  uint32_t output_overflow_count;
  uint8_t engine_phase;
  uint8_t fault_code;
  uint8_t fault_severity;
  uint8_t cancel_reason;
  uint8_t control_mode;
  uint16_t fuel_pulse_width_us;
  int16_t ignition_advance_x10;
  uint16_t dwell_us;
  uint16_t lambda_target_x100;
  uint16_t torque_limit_x100;
  uint8_t rev_soft_active;
  uint8_t rev_hard_active;
  uint8_t launch_active;
  uint8_t flat_shift_active;
  uint8_t fuel_cut;
  uint8_t spark_cut;
} ecu_sim_snapshot_t;

typedef struct ecu_sim_handle ecu_sim_handle_t;

/* Handles are opaque capability tokens, not raw pointers to state.
 * Destroying a null or stale token is a no-op. Calls that start after destroy
 * begins return ECU_SIM_ERR_INVALID or 0. Destroy waits for any active call
 * that has already entered the handle to finish before it returns. Tokens
 * encode a registry slot, generation, per-registry nonce, and cookie; freed
 * slots may be reused only after generation advances, and exhausted-generation
 * slots retire. Treat handles as bearer tokens: callers must pass back the
 * exact token returned by ecu_sim_handle_create. */
ecu_sim_handle_t *ecu_sim_handle_create(void);
void ecu_sim_handle_destroy(ecu_sim_handle_t *handle);

ecu_sim_status_t ecu_sim_handle_reset(ecu_sim_handle_t *handle);
ecu_sim_status_t ecu_sim_handle_init(ecu_sim_handle_t *handle,
                                     const ecu_sim_init_cfg_t *cfg);
ecu_sim_status_t ecu_sim_handle_set_time(ecu_sim_handle_t *handle, uint32_t now_us);
ecu_sim_status_t ecu_sim_handle_set_sensors(ecu_sim_handle_t *handle,
                                            const ecu_sim_sensor_frame_t *frame);
ecu_sim_status_t ecu_sim_handle_on_crank_edge(ecu_sim_handle_t *handle, uint32_t ts_us);
ecu_sim_status_t ecu_sim_handle_on_cam_edge(ecu_sim_handle_t *handle, uint32_t ts_us);
ecu_sim_status_t ecu_sim_handle_step(ecu_sim_handle_t *handle, uint32_t now_us);
size_t ecu_sim_handle_dequeue_events(ecu_sim_handle_t *handle,
                                     ecu_sim_output_event_t *out,
                                     size_t cap);
ecu_sim_status_t ecu_sim_handle_snapshot(ecu_sim_handle_t *handle, ecu_sim_snapshot_t *out);

ecu_sim_status_t ecu_sim_init(const ecu_sim_init_cfg_t *cfg);
void ecu_sim_reset(void);

ecu_sim_status_t ecu_sim_set_time(uint32_t now_us);
ecu_sim_status_t ecu_sim_set_sensors(const ecu_sim_sensor_frame_t *frame);
ecu_sim_status_t ecu_sim_on_crank_edge(uint32_t ts_us);
ecu_sim_status_t ecu_sim_on_cam_edge(uint32_t ts_us);

ecu_sim_status_t ecu_sim_step(uint32_t now_us);
size_t ecu_sim_dequeue_events(ecu_sim_output_event_t *out, size_t cap);
ecu_sim_status_t ecu_sim_snapshot(ecu_sim_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif
