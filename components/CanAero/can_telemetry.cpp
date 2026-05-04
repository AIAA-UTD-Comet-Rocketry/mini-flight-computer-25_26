#include "can_telemetry.h"
#include "canaerospace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "FlightFSM.h"
#include "attitude_ekf.h"

// ----- Externals from sensor_mgr / sd_logger / FlightFSM (sources of truth) -----
// Forward-declared rather than included to avoid pulling main's headers
// (sd_logger.h, sensor_mgr.h) into this component's REQUIRES graph.
extern "C" {
    extern float gAccel[3];
    extern float gAltitude;
    extern float gTotalAcc;
    extern float gVerticalVelocity_fps;
    extern uint8_t gPyroStatus;
    State getCurrentFlightState(void);   // defined in FlightFSM.c (added)
    bool  sd_logger_is_active(void);     // defined in main/sd_logger.c (added)
}

static const char *TAG = "CanTLM";

// Module state. The mutex protects msg_counter inside the TX context (events
// can fire from any task) and the latched status-flag byte.
static canas_tx_ctx_t   g_tx_ctx;
static SemaphoreHandle_t g_tx_mutex = nullptr;
static volatile bool    g_started   = false;
static uint8_t          g_status_flags = 0;

static inline bool fsm_in_flight(State s) {
    return s == STATE_BURNING || s == STATE_RISING || s == STATE_APOGEE
        || s == STATE_DROGUE_DESCENT || s == STATE_MAIN_DESCENT;
}

static inline bool fsm_armed(State s) {
    return s != STATE_IDLE && s != STATE_DISARM;
}

// Single emit guarded by the TX mutex so events and the periodic loop don't
// scramble each other's msg_counter or interleave at the driver layer.
static esp_err_t locked_tx_float(uint16_t id, float v) {
    return canas_tx_float(&g_tx_ctx, id, v);
}
static esp_err_t locked_tx_uchar(uint16_t id, uint8_t v) {
    return canas_tx_uchar(&g_tx_ctx, id, v);
}
static esp_err_t locked_tx_bchar(uint16_t id, uint8_t v) {
    return canas_tx_bchar(&g_tx_ctx, id, v);
}

static void emit_status_set(void) {
    if (xSemaphoreTake(g_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    attitude_t att;
    attitude_ekf_get_attitude(&att);

    // Float telemetry
    locked_tx_float(CAN_TLM_ID_ALTITUDE,  gAltitude);
    locked_tx_float(CAN_TLM_ID_ACCEL_X,   gAccel[0]);
    locked_tx_float(CAN_TLM_ID_ACCEL_Y,   gAccel[1]);
    locked_tx_float(CAN_TLM_ID_ACCEL_Z,   gAccel[2]);
    locked_tx_float(CAN_TLM_ID_TOTAL_ACC, gTotalAcc);
    locked_tx_float(CAN_TLM_ID_VERT_VEL,  gVerticalVelocity_fps);
    locked_tx_float(CAN_TLM_ID_PITCH,     att.pitch_deg);
    locked_tx_float(CAN_TLM_ID_ROLL,      att.roll_deg);
    locked_tx_float(CAN_TLM_ID_YAW,       att.yaw_deg);

    // Discrete telemetry — assemble status flags from latched bits + live FSM
    State fsm = getCurrentFlightState();
    uint8_t flags = g_status_flags;
    if (sd_logger_is_active()) flags |=  CAN_TLM_FLAG_SD_LOGGING;
    else                       flags &= ~CAN_TLM_FLAG_SD_LOGGING;
    if (fsm_armed(fsm))        flags |=  CAN_TLM_FLAG_ARMED;
    else                       flags &= ~CAN_TLM_FLAG_ARMED;

    locked_tx_uchar(CAN_TLM_ID_FSM_STATE,   (uint8_t)fsm);
    locked_tx_bchar(CAN_TLM_ID_STATUS_FLAGS, flags);
    locked_tx_bchar(CAN_TLM_ID_PYRO_STATUS,  gPyroStatus);

    xSemaphoreGive(g_tx_mutex);
}

static void can_tlm_task(void *pv) {
    (void)pv;
    ESP_LOGI(TAG, "Telemetry task started (node %d)", CAN_TLM_NODE_ID);

    while (1) {
        ESP_LOGI(TAG, "test0"); 
        State fsm = getCurrentFlightState();
        ESP_LOGI(TAG, "fsm: %s", fsm);
        // Either 10 Hz when launched or 1 Hz at ground
        // TickType_t period = fsm_in_flight(fsm) ? pdMS_TO_TICKS(100)   // 10 Hz
        //                                        : pdMS_TO_TICKS(1000); // 1 Hz
        ESP_LOGI(TAG, "test1");                                       
        emit_status_set();
        ESP_LOGI(TAG, "test2");
        //vTaskDelay(period);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void can_telemetry_start(twai_node_handle_t node_hdl) {
    if (g_started) return;
    g_tx_mutex = xSemaphoreCreateMutex();
    canas_tx_init(&g_tx_ctx, node_hdl, CAN_TLM_NODE_ID);
    g_started = true;

    BaseType_t r = xTaskCreate(can_tlm_task, "CAN-TLM", configMINIMAL_STACK_SIZE * 6, NULL, 1, NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to spawn telemetry task");
        g_started = false;
    }
    else {
        ESP_LOGI(TAG, "Spawned telemetry task");
    }
}

extern "C" void can_telemetry_event(uint8_t event_type, uint8_t event_data) {
    if (!g_started) return;
    if (xSemaphoreTake(g_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    canas_tx_uchar2(&g_tx_ctx, CAN_TLM_ID_EVENT, event_type, event_data);
    xSemaphoreGive(g_tx_mutex);
}

extern "C" void can_telemetry_set_status_bit(uint8_t flag_mask, bool value) {
    // No mutex needed: single-byte writes on aligned uint8_t are atomic on Xtensa,
    // and the periodic emitter snapshots into a local before sending.
    if (value) g_status_flags |=  flag_mask;
    else       g_status_flags &= ~flag_mask;
}
