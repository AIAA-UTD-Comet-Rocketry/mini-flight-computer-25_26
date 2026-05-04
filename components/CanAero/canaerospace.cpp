#include "canaerospace.h"
#include <string.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_log.h"

// CANaerospace specifies network byte order (big-endian) for multi-byte data.
// ESP32 is little-endian, so we byte-swap on the way out / in.

static const char *TAG = "CanAero";

static inline void be_u16_store(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

static inline void be_u32_store(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

#define CANAS_TX_POOL_SIZE 8

typedef struct {
    twai_frame_t frame;
    uint8_t buf[8];
} canas_tx_slot_t;

#define CANAS_CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))

static QueueHandle_t s_free_q = NULL;
static canas_tx_slot_t s_slots[CANAS_TX_POOL_SIZE];

static bool IRAM_ATTR canas_on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx) {
    (void)handle;
    (void)user_ctx;
    if (!edata || !edata->done_tx_frame || !s_free_q) return false;
    canas_tx_slot_t *slot = CANAS_CONTAINER_OF(edata->done_tx_frame, canas_tx_slot_t, frame);
    BaseType_t higher_woken = pdFALSE;
    xQueueSendFromISR(s_free_q, &slot, &higher_woken);
    
    return higher_woken == pdTRUE;
}

static esp_err_t canas_tx_pool_init(void) {
    if (s_free_q) return ESP_OK;
    s_free_q = xQueueCreate(CANAS_TX_POOL_SIZE, sizeof(canas_tx_slot_t *));
    if (!s_free_q) return ESP_ERR_NO_MEM;
    for (int i = 0; i < CANAS_TX_POOL_SIZE; ++i) {
        canas_tx_slot_t *slot = &s_slots[i];
        xQueueSend(s_free_q, &slot, 0);
    }

    return ESP_OK;
}

bool canas_parse(const twai_frame_t *frame, canas_msg_t *out) {
    if (frame->header.rtr)         return false;  // RTR frames carry no data
    if (frame->header.dlc < 4)     return false;  // CANaerospace minimum is 4 bytes
    if (frame->buffer == NULL)     return false;

    out->msg_id   = (uint16_t)frame->header.id;
    out->node_id  = frame->buffer[0];
    out->dtc      = frame->buffer[1];
    out->svc_code = frame->buffer[2];
    out->msg_code = frame->buffer[3];
    out->data_len = (uint8_t)(frame->header.dlc - 4);

    if (out->data_len > sizeof(out->data)) out->data_len = sizeof(out->data);
    memcpy(out->data, &frame->buffer[4], out->data_len);

    return true;
}

void canas_tx_init(canas_tx_ctx_t *ctx, twai_node_handle_t node_hdl, uint8_t node_id) {
    ctx->node_hdl    = node_hdl;
    ctx->node_id     = node_id;
    ctx->msg_counter = 0;
}

esp_err_t canas_tx_register_callbacks(twai_node_handle_t node_hdl) {
    esp_err_t ret = canas_tx_pool_init();
    if (ret != ESP_OK) return ret;
    twai_event_callbacks_t cbs = {};
    cbs.on_tx_done = canas_on_tx_done;

    return twai_node_register_event_callbacks(node_hdl, &cbs, NULL);
}

// Build the 4-byte header at buf[0..3] and return the start of payload (buf+4).
static uint8_t *fill_header(canas_tx_ctx_t *ctx, uint8_t *buf, canas_dtc_t dtc) {
    buf[0] = ctx->node_id;
    buf[1] = (uint8_t)dtc;
    buf[2] = 0; // svc_code: 0 = normal data message
    buf[3] = ctx->msg_counter++;

    return buf + 4;
}

static canas_tx_slot_t *acquire_slot(TickType_t wait_ticks) {
    if (!s_free_q) return NULL;
    canas_tx_slot_t *slot = NULL;
    if (xQueueReceive(s_free_q, &slot, wait_ticks) != pdTRUE) return NULL;

    return slot;
}

static void release_slot(canas_tx_slot_t *slot) {
    if (!s_free_q || !slot) return;
    xQueueSend(s_free_q, &slot, 0);
}

static esp_err_t send_frame(canas_tx_ctx_t *ctx, canas_tx_slot_t *slot, uint16_t msg_id, uint8_t dlc) {
    twai_frame_t *frame = &slot->frame;
    frame->header.id  = msg_id;
    frame->header.dlc = dlc;
    frame->header.ide = 0; // 11-bit standard ID
    frame->header.rtr = 0;
    frame->header.fdf = 0;
    frame->buffer     = slot->buf;
    frame->buffer_len = dlc;
    esp_err_t ret = twai_node_transmit(ctx->node_hdl, frame, 10);

    //TODO: LED_setPattern(led_can_tx, pattern_fast_blink);

    ESP_LOGI(TAG, "CAN id=0x%03X dlc=%u", frame->header.id, frame->header.dlc);
    ESP_LOG_BUFFER_HEX(TAG, frame->buffer, frame->header.dlc);
    ESP_LOGI(TAG, "node=%u dtc=%u svc=%u msg=%u payload_len=%u",
         frame->buffer[0], frame->buffer[1], frame->buffer[2],
         frame->buffer[3], frame->header.dlc - 4);

    if (ret != ESP_OK) release_slot(slot);

    return ret;
}

esp_err_t canas_tx_float(canas_tx_ctx_t *ctx, uint16_t msg_id, float v) {
    canas_tx_slot_t *slot = acquire_slot(pdMS_TO_TICKS(10));
    if (!slot) return ESP_ERR_TIMEOUT;
    uint8_t *p = fill_header(ctx, slot->buf, CANAS_DTC_FLOAT);
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    be_u32_store(p, bits);

    ESP_LOGI(TAG, "TX id=0x%03X float data=%f", msg_id, v);

    return send_frame(ctx, slot, msg_id, 8);
}

esp_err_t canas_tx_short(canas_tx_ctx_t *ctx, uint16_t msg_id, int16_t v) {
    canas_tx_slot_t *slot = acquire_slot(pdMS_TO_TICKS(10));
    if (!slot) return ESP_ERR_TIMEOUT;
    uint8_t *p = fill_header(ctx, slot->buf, CANAS_DTC_SHORT);
    be_u16_store(p, (uint16_t)v);

    ESP_LOGI(TAG, "TX id=0x%03X short data=%d", msg_id, v);

    return send_frame(ctx, slot, msg_id, 6);
}

esp_err_t canas_tx_uchar(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t v) {
    canas_tx_slot_t *slot = acquire_slot(pdMS_TO_TICKS(10));
    if (!slot) return ESP_ERR_TIMEOUT;
    uint8_t *p = fill_header(ctx, slot->buf, CANAS_DTC_UCHAR);
    p[0] = v;

    ESP_LOGI(TAG, "TX id=0x%03X uchar data=%d", msg_id, v);

    return send_frame(ctx, slot, msg_id, 5);
}

esp_err_t canas_tx_bchar(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t bits) {
    canas_tx_slot_t *slot = acquire_slot(pdMS_TO_TICKS(10));
    if (!slot) return ESP_ERR_TIMEOUT;
    uint8_t *p = fill_header(ctx, slot->buf, CANAS_DTC_BCHAR);
    p[0] = bits;

    ESP_LOGI(TAG, "TX id=0x%03X bchar data=%d", msg_id, bits);

    return send_frame(ctx, slot, msg_id, 5);
}

esp_err_t canas_tx_uchar2(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t a, uint8_t b) {
    canas_tx_slot_t *slot = acquire_slot(pdMS_TO_TICKS(10));
    if (!slot) return ESP_ERR_TIMEOUT;
    uint8_t *p = fill_header(ctx, slot->buf, CANAS_DTC_UCHAR2);
    p[0] = a;
    p[1] = b;

    ESP_LOGI(TAG, "TX id=0x%03X uchar2 data=%d, %d", msg_id, a, b);

    return send_frame(ctx, slot, msg_id, 6);
}

esp_err_t canas_tx_uchar4(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    canas_tx_slot_t *slot = acquire_slot(pdMS_TO_TICKS(10));
    if (!slot) return ESP_ERR_TIMEOUT;
    uint8_t *p = fill_header(ctx, slot->buf, CANAS_DTC_UCHAR4);
    p[0] = a;
    p[1] = b;
    p[2] = c;
    p[3] = d;

    ESP_LOGI(TAG, "TX id=0x%03X uchar4 data=%d, %d, %d, %d", msg_id, a, b, c, d);

    return send_frame(ctx, slot, msg_id, 8);
}