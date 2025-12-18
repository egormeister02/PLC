#include "plc_dataplane.h"

// Internal hardware abstraction header for platform-specific code.
#include "plc_hal.h"
#include "plc_dataplane_internal.h"

static void plc_memcpy(uint8_t *dst, const uint8_t *src, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        dst[i] = src[i];
    }
}

// Simple CRC8 implementation (polynomial 0x07, initial value 0x00).
static uint8_t plc_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// Byte-wise update helper to avoid temporary buffers.
static uint8_t plc_crc8_update(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        if (crc & 0x80) {
            crc = (uint8_t)((crc << 1) ^ 0x07);
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

plc_tx_handle_t *plc_tx_init(const plc_tx_config_t *config)
{
    if (!config) {
        return NULL;
    }

    if (!plc_hal_init(config)) {
        return NULL;
    }

    plc_tx_handle_t *ctx = plc_hal_alloc_context();
    if (!ctx) {
        plc_hal_deinit();
        return NULL;
    }

    ctx->cfg = *config;
    return ctx;
}

int plc_tx_send(plc_tx_handle_t *handle, const uint8_t *data, size_t length)
{
    if (!handle || !data || length == 0) {
        return -1;
    }

    if (length > PLC_TX_MAX_PAYLOAD_BYTES) {
        return -2;
    }

    // Build framed message: [PREAMBLE][LEN][PAYLOAD][CRC8].
    uint8_t frame[PLC_TX_PREAMBLE_LEN + 1 + PLC_TX_MAX_PAYLOAD_BYTES + 1];
    size_t idx = 0;

    // Preamble: AA 55 AA 55 (fixed pattern for sync).
    frame[idx++] = 0xAA;
    frame[idx++] = 0x55;
    frame[idx++] = 0xAA;
    frame[idx++] = 0x55;

    // Length (1 byte, payload only).
    frame[idx++] = (uint8_t)length;

    // Payload.
    plc_memcpy(&frame[idx], data, length);
    idx += length;

    // CRC8 over [LEN][PAYLOAD][KEY] (key from TX config).
    uint8_t crc = 0x00;
    crc = plc_crc8_update(crc, (uint8_t)length);
    for (size_t i = 0; i < length; ++i) {
        crc = plc_crc8_update(crc, data[i]);
    }
    for (uint8_t i = 0; i < handle->cfg.secret_key_len; ++i) {
        crc = plc_crc8_update(crc, handle->cfg.secret_key[i]);
    }
    frame[idx++] = crc;

    // Delegation to platform-specific implementation: send framed bytes.
    return plc_hal_send(handle, frame, idx);
}

void plc_tx_deinit(plc_tx_handle_t *handle)
{
    if (!handle) {
        return;
    }

    plc_hal_free_context(handle);
    plc_hal_deinit();
}

#define PLC_RX_MAX_FRAME_BYTES (PLC_TX_PREAMBLE_LEN + 1U + PLC_TX_MAX_PAYLOAD_BYTES + 1U)
#define PLC_RX_MAX_FRAME_BITS (PLC_RX_MAX_FRAME_BYTES * 8U)

static plc_rx_debug_t g_plc_rx_last_debug;

int plc_rx_get_last_debug(plc_rx_debug_t *out)
{
    if (!out) {
        return -1;
    }

    *out = g_plc_rx_last_debug;
    return 0;
}

static uint32_t plc_abs_diff_u32(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint8_t plc_edges_to_bit_by_means(uint16_t e, uint32_t mean0, uint32_t mean1)
{
    uint32_t d0 = plc_abs_diff_u32(e, mean0);
    uint32_t d1 = plc_abs_diff_u32(e, mean1);
    return (d1 < d0) ? 1U : 0U;
}

static uint8_t plc_edges_to_byte_by_means(const uint16_t *edges, size_t bit_offset, uint32_t mean0, uint32_t mean1)
{
    uint8_t v = 0;
    for (size_t i = 0; i < 8; ++i) {
        uint8_t bit = plc_edges_to_bit_by_means(edges[bit_offset + i], mean0, mean1);
        v = (uint8_t)((v << 1) | bit);
    }
    return v;
}

static uint8_t plc_preamble_bit(size_t bit_index)
{
    // Preamble bytes are AA 55 AA 55 -> 0xAA55AA55 MSB-first.
    const uint32_t w = 0xAA55AA55UL;
    if (bit_index >= 32U) {
        return 0;
    }
    return (uint8_t)((w >> (31U - (uint32_t)bit_index)) & 0x01U);
}

static int plc_find_preamble_and_means(const uint16_t *edges,
                                      size_t edges_len,
                                      size_t *out_pos,
                                      uint32_t *out_mean0,
                                      uint32_t *out_mean1)
{
    if (!edges || !out_pos || !out_mean0 || !out_mean1 || edges_len < 64U) {
        return -1;
    }

    const size_t pre_bits = 32U;
    size_t best_pos = (size_t)-1;
    uint64_t best_score = 0xFFFFFFFFFFFFFFFFULL;
    uint32_t best_m0 = 0;
    uint32_t best_m1 = 0;

    // Scan a limited window at the start; the HAL capture starts near frame start.
    const size_t scan_limit = (edges_len > 256U) ? 256U : (edges_len - pre_bits);

    for (size_t pos = 0; pos <= scan_limit; ++pos) {
        uint64_t s0 = 0;
        uint64_t s1 = 0;
        uint32_t c0 = 0;
        uint32_t c1 = 0;

        for (size_t i = 0; i < pre_bits; ++i) {
            uint8_t b = plc_preamble_bit(i);
            uint16_t e = edges[pos + i];
            if (b == 0U) {
                s0 += e;
                c0++;
            } else {
                s1 += e;
                c1++;
            }
        }

        if (c0 == 0 || c1 == 0) {
            continue;
        }

        uint32_t m0 = (uint32_t)(s0 / c0);
        uint32_t m1 = (uint32_t)(s1 / c1);

        // Require separation; otherwise classification will be unstable.
        uint32_t sep = (m0 > m1) ? (m0 - m1) : (m1 - m0);
        if (sep < 2U) {
            continue;
        }

        uint64_t score = 0;
        for (size_t i = 0; i < pre_bits; ++i) {
            uint8_t b = plc_preamble_bit(i);
            uint16_t e = edges[pos + i];
            uint32_t mean = (b == 0U) ? m0 : m1;
            score += (uint64_t)plc_abs_diff_u32(e, mean);
        }

        if (score < best_score) {
            best_score = score;
            best_pos = pos;
            best_m0 = m0;
            best_m1 = m1;
        }
    }

    if (best_pos == (size_t)-1) {
        return -2;
    }

    *out_pos = best_pos;
    *out_mean0 = best_m0;
    *out_mean1 = best_m1;
    return 0;
}

plc_rx_handle_t *plc_rx_init(const plc_rx_config_t *config)
{
    if (!config) {
        return NULL;
    }

    plc_rx_config_t cfg = *config;

    // rx_gpio_pin defaults to PLC_RX_DEFAULT_GPIO_PIN when cfg is zero-initialized.
    if (!plc_hal_rx_init(&cfg)) {
        return NULL;
    }

    plc_rx_handle_t *ctx = plc_hal_rx_alloc_context();
    if (!ctx) {
        plc_hal_rx_deinit();
        return NULL;
    }

    ctx->cfg = cfg;
    return ctx;
}

int plc_rx_read(plc_rx_handle_t *handle,
                uint8_t *out_payload,
                size_t out_max_len,
                size_t *out_length,
                uint32_t timeout_ms)
{
    if (!handle || !out_payload || !out_length) {
        return -1;
    }

    *out_length = 0;

    if (handle->cfg.symbol_rate == 0) {
        return -1;
    }

    uint16_t edges[PLC_RX_MAX_FRAME_BITS];

    uint32_t timeout_us = 0;
    if (timeout_ms != 0) {
        timeout_us = timeout_ms * 1000U;
    }

    int rc = plc_hal_rx_capture_edges(handle, edges, PLC_RX_MAX_FRAME_BITS, timeout_us);
    if (rc != 0) {
        // Map HAL timeout to public timeout.
        return (rc == -2) ? -3 : rc;
    }

    const size_t preamble_len_bits = PLC_TX_PREAMBLE_LEN * 8U;
    const size_t min_bits_after_preamble = 8U + 8U; // LEN + CRC (payload may be empty, but we reject len=0)

    // NOTE: capture start can be slightly early due to noise / start-condition jitter.
    // Scan the whole captured window so we can still find the preamble later in the buffer.
    const size_t scan_limit =
        (PLC_RX_MAX_FRAME_BITS > (preamble_len_bits + min_bits_after_preamble))
            ? (PLC_RX_MAX_FRAME_BITS - (preamble_len_bits + min_bits_after_preamble))
            : 0U;

    bool saw_crc_mismatch = false;

    // Track the best candidate for debugging.
    bool have_dbg = false;
    uint32_t best_mism = 0xFFFFFFFFU;
    uint64_t best_score = 0xFFFFFFFFFFFFFFFFULL;
    plc_rx_debug_t dbg = {0};

    // Try to locate a valid frame by scanning candidate preamble positions and validating CRC.
    for (size_t pos = 0; pos <= scan_limit; ++pos) {
        if (pos + preamble_len_bits + min_bits_after_preamble > PLC_RX_MAX_FRAME_BITS) {
            break;
        }

        uint64_t s0 = 0;
        uint64_t s1 = 0;
        uint32_t c0 = 0;
        uint32_t c1 = 0;

        for (size_t i = 0; i < 32U; ++i) {
            uint8_t b = plc_preamble_bit(i);
            uint16_t e = edges[pos + i];
            if (b == 0U) {
                s0 += e;
                c0++;
            } else {
                s1 += e;
                c1++;
            }
        }

        if (c0 == 0U || c1 == 0U) {
            continue;
        }

        uint32_t mean0 = (uint32_t)(s0 / c0);
        uint32_t mean1 = (uint32_t)(s1 / c1);

        // Require separation; otherwise classification will be unstable.
        uint32_t sep = (mean0 > mean1) ? (mean0 - mean1) : (mean1 - mean0);
        if (sep < 2U) {
            continue;
        }

        // Preamble quality metrics (for debug).
        uint32_t mism = 0;
        uint64_t score = 0;

        uint16_t pre0_min = 0xFFFFU;
        uint16_t pre0_max = 0;
        uint16_t pre1_min = 0xFFFFU;
        uint16_t pre1_max = 0;

        for (size_t i = 0; i < 32U; ++i) {
            uint8_t exp_b = plc_preamble_bit(i);
            uint16_t e = edges[pos + i];

            if (exp_b == 0U) {
                if (e < pre0_min) {
                    pre0_min = e;
                }
                if (e > pre0_max) {
                    pre0_max = e;
                }
            } else {
                if (e < pre1_min) {
                    pre1_min = e;
                }
                if (e > pre1_max) {
                    pre1_max = e;
                }
            }

            uint8_t got_b = plc_edges_to_bit_by_means(e, mean0, mean1);
            if (got_b != exp_b) {
                mism++;
            }

            uint32_t mean = (exp_b == 0U) ? mean0 : mean1;
            score += (uint64_t)plc_abs_diff_u32(e, mean);
        }

        uint8_t payload_len = plc_edges_to_byte_by_means(edges, pos + preamble_len_bits, mean0, mean1);
        if (payload_len == 0U || payload_len > PLC_TX_MAX_PAYLOAD_BYTES) {
            continue;
        }

        const size_t payload_start = pos + preamble_len_bits + 8U;
        const size_t payload_bits = (size_t)payload_len * 8U;
        const size_t crc_pos = payload_start + payload_bits;

        if (crc_pos + 8U > PLC_RX_MAX_FRAME_BITS) {
            continue;
        }

        uint8_t tmp_payload[PLC_TX_MAX_PAYLOAD_BYTES];
        for (uint8_t i = 0; i < payload_len; ++i) {
            tmp_payload[i] = plc_edges_to_byte_by_means(edges, payload_start + (size_t)i * 8U, mean0, mean1);
        }

        uint8_t rx_crc = plc_edges_to_byte_by_means(edges, crc_pos, mean0, mean1);

        // Verify CRC8 over [LEN][PAYLOAD][KEY].
        uint8_t calc_crc = 0x00;
        calc_crc = plc_crc8_update(calc_crc, payload_len);
        for (uint8_t i = 0; i < payload_len; ++i) {
            calc_crc = plc_crc8_update(calc_crc, tmp_payload[i]);
        }
        for (uint8_t i = 0; i < handle->cfg.secret_key_len; ++i) {
            calc_crc = plc_crc8_update(calc_crc, handle->cfg.secret_key[i]);
        }

        // Update debug info from the best-scoring candidate.
        if (!have_dbg || (mism < best_mism) || (mism == best_mism && score < best_score)) {
            have_dbg = true;
            best_mism = mism;
            best_score = score;

            dbg.valid = 1;
            dbg.best_pos_bits = (uint16_t)pos;
            dbg.mean0 = (uint16_t)mean0;
            dbg.mean1 = (uint16_t)mean1;
            dbg.sep = (uint16_t)sep;
            dbg.preamble_bit_errors = (uint8_t)mism;

            dbg.decoded_preamble[0] = plc_edges_to_byte_by_means(edges, pos + 0U, mean0, mean1);
            dbg.decoded_preamble[1] = plc_edges_to_byte_by_means(edges, pos + 8U, mean0, mean1);
            dbg.decoded_preamble[2] = plc_edges_to_byte_by_means(edges, pos + 16U, mean0, mean1);
            dbg.decoded_preamble[3] = plc_edges_to_byte_by_means(edges, pos + 24U, mean0, mean1);

            dbg.decoded_len = payload_len;
            dbg.rx_crc = rx_crc;
            dbg.calc_crc = calc_crc;

            dbg.preview_len = (payload_len > PLC_RX_DEBUG_PREVIEW_BYTES) ? PLC_RX_DEBUG_PREVIEW_BYTES : payload_len;
            for (uint8_t i = 0; i < dbg.preview_len; ++i) {
                dbg.preview[i] = tmp_payload[i];
            }

            dbg.pre0_min = pre0_min;
            dbg.pre0_max = pre0_max;
            dbg.pre1_min = pre1_min;
            dbg.pre1_max = pre1_max;
        }

        if (calc_crc != rx_crc) {
            saw_crc_mismatch = true;
            continue;
        }

        if (payload_len > out_max_len) {
            g_plc_rx_last_debug = dbg;
            g_plc_rx_last_debug.last_rc = -2;
            return -2;
        }

        plc_memcpy(out_payload, tmp_payload, payload_len);
        *out_length = payload_len;

        g_plc_rx_last_debug = dbg;
        g_plc_rx_last_debug.last_rc = 0;
        return 0;
    }

    if (have_dbg) {
        g_plc_rx_last_debug = dbg;
        g_plc_rx_last_debug.last_rc = saw_crc_mismatch ? -4 : -3;
    } else {
        g_plc_rx_last_debug.valid = 0;
        g_plc_rx_last_debug.last_rc = saw_crc_mismatch ? -4 : -3;
    }

    return saw_crc_mismatch ? -4 : -3;
}

void plc_rx_deinit(plc_rx_handle_t *handle)
{
    if (!handle) {
        return;
    }

    plc_hal_rx_free_context(handle);
    plc_hal_rx_deinit();
}
