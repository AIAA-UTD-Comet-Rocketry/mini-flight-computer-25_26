#include "canaerospace.h"
#include <string.h>

// CANaerospace specifies network byte order (big-endian) for multi-byte data.
// ESP32 is little-endian, so we byte-swap on the way out / in.

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

// Build the 4-byte header at buf[0..3] and return the start of payload (buf+4).
static uint8_t *fill_header(canas_tx_ctx_t *ctx, uint8_t *buf, canas_dtc_t dtc) {
    buf[0] = ctx->node_id;
    buf[1] = (uint8_t)dtc;
    buf[2] = 0; // svc_code: 0 = normal data message
    buf[3] = ctx->msg_counter++;
    return buf + 4;
}

static esp_err_t send_frame(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t *buf, uint8_t dlc) {
    twai_frame_t frame = {};
    frame.header.id  = msg_id;
    frame.header.dlc = dlc;
    frame.header.ide = 0; // 11-bit standard ID
    frame.header.rtr = 0;
    frame.header.fdf = 0;
    frame.buffer     = buf;
    frame.buffer_len = dlc;
    return twai_node_transmit(ctx->node_hdl, &frame, 10);
}

esp_err_t canas_tx_float(canas_tx_ctx_t *ctx, uint16_t msg_id, float v) {
    uint8_t buf[8];
    uint8_t *p = fill_header(ctx, buf, CANAS_DTC_FLOAT);
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    be_u32_store(p, bits);
    return send_frame(ctx, msg_id, buf, 8);
}

esp_err_t canas_tx_short(canas_tx_ctx_t *ctx, uint16_t msg_id, int16_t v) {
    uint8_t buf[8];
    uint8_t *p = fill_header(ctx, buf, CANAS_DTC_SHORT);
    be_u16_store(p, (uint16_t)v);
    return send_frame(ctx, msg_id, buf, 6);
}

esp_err_t canas_tx_uchar(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t v) {
    uint8_t buf[8];
    uint8_t *p = fill_header(ctx, buf, CANAS_DTC_UCHAR);
    p[0] = v;
    return send_frame(ctx, msg_id, buf, 5);
}

esp_err_t canas_tx_bchar(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t bits) {
    uint8_t buf[8];
    uint8_t *p = fill_header(ctx, buf, CANAS_DTC_BCHAR);
    p[0] = bits;
    return send_frame(ctx, msg_id, buf, 5);
}

esp_err_t canas_tx_uchar2(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t a, uint8_t b) {
    uint8_t buf[8];
    uint8_t *p = fill_header(ctx, buf, CANAS_DTC_UCHAR2);
    p[0] = a;
    p[1] = b;
    return send_frame(ctx, msg_id, buf, 6);
}
