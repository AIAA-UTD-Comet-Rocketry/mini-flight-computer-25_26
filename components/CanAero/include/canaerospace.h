#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CANaerospace Data Type Codes (DTC)
 * Defined in the CANaerospace specification, section 3.
 */
typedef enum {
    CANAS_DTC_NODATA  = 0,   // No data
    CANAS_DTC_ERROR   = 1,   // Error (4-byte error code)
    CANAS_DTC_FLOAT   = 2,   // 32-bit IEEE 754 float
    CANAS_DTC_LONG    = 3,   // 32-bit signed integer
    CANAS_DTC_ULONG   = 4,   // 32-bit unsigned integer
    CANAS_DTC_BLONG   = 5,   // 32-bit bit field
    CANAS_DTC_SHORT   = 6,   // 16-bit signed integer
    CANAS_DTC_USHORT  = 7,   // 16-bit unsigned integer
    CANAS_DTC_BSHORT  = 8,   // 16-bit bit field
    CANAS_DTC_CHAR    = 9,   // 8-bit signed char
    CANAS_DTC_UCHAR   = 10,  // 8-bit unsigned char
    CANAS_DTC_BCHAR   = 11,  // 8-bit bit field
    CANAS_DTC_SHORT2  = 12,  // Two 16-bit signed integers
    CANAS_DTC_USHORT2 = 13,  // Two 16-bit unsigned integers
    CANAS_DTC_BSHORT2 = 14,  // Two 16-bit bit fields
    CANAS_DTC_CHAR4   = 15,  // Four 8-bit signed chars
    CANAS_DTC_UCHAR4  = 16,  // Four 8-bit unsigned chars
    CANAS_DTC_BCHAR4  = 17,  // Four 8-bit bit fields
    CANAS_DTC_CHAR2   = 18,  // Two 8-bit signed chars
    CANAS_DTC_UCHAR2  = 19,  // Two 8-bit unsigned chars
    CANAS_DTC_BCHAR2  = 20,  // Two 8-bit bit fields
} canas_dtc_t;

/*
 * Parsed CANaerospace message.
 *
 * CANaerospace frame layout (CAN data bytes):
 *   [0] node_id   — sender's node ID
 *   [1] dtc       — data type code (see canas_dtc_t)
 *   [2] svc_code  — service code (0 = normal data message)
 *   [3] msg_code  — rolling message counter
 *   [4..7]        — data payload (0–4 bytes, count in data_len)
 */
typedef struct {
    uint16_t msg_id;    // CAN message ID (11-bit, from frame identifier)
    uint8_t  node_id;   // Sender node ID
    uint8_t  dtc;       // Data Type Code
    uint8_t  svc_code;  // Service Code
    uint8_t  msg_code;  // Message counter
    uint8_t  data[4];   // Raw payload bytes
    uint8_t  data_len;  // Number of valid payload bytes (= DLC - 4)
} canas_msg_t;

/*
 * TX context. One per CAN node. Owns the rolling message counter.
 */
typedef struct {
    twai_node_handle_t node_hdl;
    uint8_t            node_id;
    uint8_t            msg_counter;
} canas_tx_ctx_t;

/*
 * Parse a received TWAI frame into a CANaerospace message.
 * Returns false if the frame is an RTR frame or DLC < 4 (minimum CANaerospace size).
 */
bool canas_parse(const twai_frame_t *frame, canas_msg_t *out);

/*
 * Initialize a TX context. Does not allocate; caller owns storage.
 */
void canas_tx_init(canas_tx_ctx_t *ctx, twai_node_handle_t node_hdl, uint8_t node_id);

/*
 * Register the CANaerospace TX-done callback and initialize the TX frame pool.
 * Call this before twai_node_enable() so the driver can accept the callback.
 */
esp_err_t canas_tx_register_callbacks(twai_node_handle_t node_hdl);

/*
 * Per-DTC TX helpers. Each builds one CANaerospace frame, populates the
 * 4-byte header (node_id, dtc, svc=0, msg_counter++), encodes the payload
 * big-endian per spec, and submits via twai_node_transmit with a 10ms timeout.
 *
 * Return ESP_OK on successful enqueue; ESP_ERR_TIMEOUT if TX queue full.
 */
esp_err_t canas_tx_float (canas_tx_ctx_t *ctx, uint16_t msg_id, float    v);
esp_err_t canas_tx_short (canas_tx_ctx_t *ctx, uint16_t msg_id, int16_t  v);
esp_err_t canas_tx_uchar (canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t  v);
esp_err_t canas_tx_bchar (canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t  bits);
esp_err_t canas_tx_uchar2(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t  a, uint8_t b);
esp_err_t canas_tx_uchar4(canas_tx_ctx_t *ctx, uint16_t msg_id, uint8_t  a, uint8_t b, uint8_t c, uint8_t d);

#ifdef __cplusplus
}
#endif // CANAEROSPACE_H

