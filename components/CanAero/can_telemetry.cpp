#include "can_telemetry.h"
#include "canaerospace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <limits.h>

#include "FlightFSM.h"

// ----- Externals from sensor_mgr / sd_logger / FlightFSM (sources of truth) -----
// Forward-declared rather than included to avoid pulling main's headers
// (sd_logger.h, sensor_mgr.h) into this component's REQUIRES graph.
extern "C" {
    extern float gAccel[3];
    extern float gOrient[3];
    extern float gAltitude;
    extern float gTotalAcc;
    extern float gVerticalVelocity_fps;
    extern uint8_t gPyroStatus;
    State getCurrentFlightState(void);   // defined in FlightFSM.c (added)
    bool  sd_logger_is_active(void);     // defined in main/sd_logger.c (added)
    // Forward-declared with int to avoid pulling BSP.h's sensor headers into this component.
    // Values must stay in sync with led_index_t and pattern_index_t in BSP.h / gpio_mgr.h.
    int LED_setPattern(int led, int pattern);
}
static const int kLedCanTx          = 1; // led_can_tx
static const int kPatternFastFlash  = 8; // pattern_fast_flash

static void pulse_can_tx_led(void) {
    LED_setPattern(kLedCanTx, kPatternFastFlash);
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

static inline int16_t clamp_i16(int32_t v) {
    if (v > INT16_MAX) return INT16_MAX;
    if (v < INT16_MIN) return INT16_MIN;
    return (int16_t)v;
}

static void build_packet(can_tlm_packet_t *pkt, State fsm, uint8_t flags) {
    pkt->time_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    pkt->altitude_ft = clamp_i16((int32_t)lroundf(gAltitude));
    pkt->vert_vel_fps_x10 = clamp_i16((int32_t)lroundf(gVerticalVelocity_fps * 10.0f));
    pkt->accel_x_mg = clamp_i16((int32_t)lroundf(gAccel[0] * 1000.0f));
    pkt->accel_y_mg = clamp_i16((int32_t)lroundf(gAccel[1] * 1000.0f));
    pkt->accel_z_mg = clamp_i16((int32_t)lroundf(gAccel[2] * 1000.0f));
    pkt->yaw_deg   = clamp_i16((int32_t)lroundf(gOrient[0]));
    pkt->pitch_deg = clamp_i16((int32_t)lroundf(gOrient[1]));
    pkt->roll_deg  = clamp_i16((int32_t)lroundf(gOrient[2]));
    pkt->fsm_state = (uint8_t)fsm;
    pkt->status_flags = flags;
    pkt->pyro_status = gPyroStatus;
    pkt->reserved = 0;
}

// Single emit guarded by the TX mutex so events and the periodic loop don't
// scramble each other's msg_counter or interleave at the driver layer.
static esp_err_t locked_tx_uchar4(uint16_t id, uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    return canas_tx_uchar4(&g_tx_ctx, id, a, b, c, d);
}

static void emit_status_set(void) {
    if (xSemaphoreTake(g_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    State fsm = getCurrentFlightState();
    uint8_t flags = g_status_flags;
    if (sd_logger_is_active()) flags |=  CAN_TLM_FLAG_SD_LOGGING;
    else                       flags &= ~CAN_TLM_FLAG_SD_LOGGING;
    if (fsm_armed(fsm))        flags |=  CAN_TLM_FLAG_ARMED;
    else                       flags &= ~CAN_TLM_FLAG_ARMED;

    can_tlm_packet_t pkt;
    build_packet(&pkt, fsm, flags);

    const uint8_t *bytes = (const uint8_t *)&pkt;
    for (int i = 0; i < CAN_TLM_PACKET_CHUNKS; ++i) {
        uint16_t msg_id = CAN_TLM_ID_PACKET_BASE + i;
        const uint8_t *b = &bytes[i * CAN_TLM_PACKET_CHUNK_BYTES];
        locked_tx_uchar4(msg_id, b[0], b[1], b[2], b[3]);
    }

    xSemaphoreGive(g_tx_mutex);
}

static void can_tlm_task(void *pv) {
    (void)pv;
    ESP_LOGI(TAG, "Telemetry task started (node %d)", CAN_TLM_NODE_ID);

    while (1) {
        State fsm = getCurrentFlightState();

        // Either 10 Hz when launched or 1 Hz at ground
        TickType_t period = fsm_in_flight(fsm) ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(1000);
        emit_status_set();
        vTaskDelay(period);
    }
}

extern "C" void can_telemetry_start(twai_node_handle_t node_hdl) {
    if (g_started) return;
    g_tx_mutex = xSemaphoreCreateMutex();
    canas_tx_init(&g_tx_ctx, node_hdl, CAN_TLM_NODE_ID);
    g_tx_ctx.on_tx = pulse_can_tx_led;
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
