#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_twai.h"

#ifdef __cplusplus
extern "C" {
#endif

// CANaerospace message IDs assigned to each datum.
// Standard IDs per spec where they exist; custom IDs (1300+) otherwise.
typedef enum {
    CAN_TLM_ID_ACCEL_X       = 304,  // Standard: body normal accel (g)
    CAN_TLM_ID_ACCEL_Y       = 305,  // Standard: body lateral accel (g)
    CAN_TLM_ID_ACCEL_Z       = 306,  // Standard: body longitudinal accel (g)
    CAN_TLM_ID_PITCH         = 320,  // Standard: pitch angle (deg)
    CAN_TLM_ID_ROLL          = 321,  // Standard: roll angle (deg)
    CAN_TLM_ID_YAW           = 322,  // Standard: magnetic heading (deg)
    CAN_TLM_ID_ALTITUDE      = 605,  // Standard: pressure altitude (ft)
    CAN_TLM_ID_FSM_STATE     = 1300, // Custom: FSM State enum value
    CAN_TLM_ID_STATUS_FLAGS  = 1301, // Custom: bitfield, see CAN_TLM_FLAG_*
    CAN_TLM_ID_PYRO_STATUS   = 1302, // Custom: bitfield, mirrors gPyroStatus
    CAN_TLM_ID_TOTAL_ACC     = 1303, // Custom: gTotalAcc magnitude (g)
    CAN_TLM_ID_VERT_VEL      = 1304, // Custom: vertical velocity (ft/s)
    CAN_TLM_ID_EVENT         = 1310, // Custom: event packet (UCHAR2: type, data)
} can_tlm_id_t;

// Status flag bits (sent in CAN_TLM_ID_STATUS_FLAGS as DTC_BCHAR).
#define CAN_TLM_FLAG_SD_LOGGING         (1 << 0)
#define CAN_TLM_FLAG_MAG_CAL_VALID      (1 << 1)
#define CAN_TLM_FLAG_EKF_LOCKED         (1 << 2)
#define CAN_TLM_FLAG_GROUND_PRESS_VALID (1 << 3)
#define CAN_TLM_FLAG_ARMED              (1 << 4)

// Event packet codes (sent in CAN_TLM_ID_EVENT as DTC_UCHAR2: [type, data]).
typedef enum {
    EVT_BOOT          = 0x01, // data = esp_reset_reason() cast to u8
    EVT_ARMED         = 0x02, // data = 0
    EVT_DISARMED      = 0x03, // data = 0
    EVT_LAUNCH        = 0x04, // data = peak gTotalAcc rounded to u8 (g)
    EVT_BURNOUT       = 0x05, // data = 0
    EVT_APOGEE        = 0x06, // data = 0
    EVT_DROGUE_FIRED  = 0x07, // data = 0
    EVT_MAIN_FIRED    = 0x08, // data = 0
    EVT_LANDED        = 0x09, // data = 0
    EVT_SENSOR_FAIL   = 0x10, // data = 1=IMU, 2=MAG, 3=BARO
    EVT_SD_FAIL       = 0x11, // data = 0
    EVT_GENERIC_ERROR = 0xFF, // data = caller's error code
} can_tlm_event_t;

// Default flight computer node ID (1 = primary FC).
#define CAN_TLM_NODE_ID 1

// Packed telemetry packet for classic CAN (split into 4-byte chunks).
#define CAN_TLM_PACKET_SIZE 24
#define CAN_TLM_PACKET_CHUNK_BYTES 4
#define CAN_TLM_PACKET_CHUNKS (CAN_TLM_PACKET_SIZE / CAN_TLM_PACKET_CHUNK_BYTES)
#define CAN_TLM_ID_PACKET_BASE 1400

typedef struct __attribute__((packed)) {
    uint32_t time_ms;
    int16_t  altitude_ft;
    int16_t  vert_vel_fps_x10;
    int16_t  accel_x_mg;
    int16_t  accel_y_mg;
    int16_t  accel_z_mg;
    int16_t  pitch_deg;
    int16_t  roll_deg;
    int16_t  yaw_deg;
    uint8_t  fsm_state;
    uint8_t  status_flags;
    uint8_t  pyro_status;
    uint8_t  reserved;
} can_tlm_packet_t;

#ifdef __cplusplus
static_assert(sizeof(can_tlm_packet_t) == CAN_TLM_PACKET_SIZE, "can_tlm_packet_t size mismatch");
#else
_Static_assert(sizeof(can_tlm_packet_t) == CAN_TLM_PACKET_SIZE, "can_tlm_packet_t size mismatch");
#endif

/*
 * Spawn the periodic telemetry TX task and initialize the internal CANaerospace
 * TX context against the supplied node handle. Must be called once after BSP
 * has brought up the CAN bus. Safe to call from any task context.
 */
void can_telemetry_start(twai_node_handle_t node_hdl);

/*
 * Fire a single event frame. Safe to call from any task or callback. Drops
 * silently if telemetry is not started yet or the TX queue is full (events
 * are best-effort; the periodic status set is the source of truth).
 */
void can_telemetry_event(uint8_t event_type, uint8_t event_data);

/*
 * Update one bit in the latched status-flags byte (CAN_TLM_FLAG_*).
 * Called by main during boot to mark mag-cal/EKF-cal/ground-pressure validity.
 * Thread-safe.
 */
void can_telemetry_set_status_bit(uint8_t flag_mask, bool value);

#ifdef __cplusplus
}
#endif // CAN_TELEMETRY_H