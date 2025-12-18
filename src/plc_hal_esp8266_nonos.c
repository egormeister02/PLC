#if !defined(ARDUINO)

#include "plc_hal.h"

// NOTE: This file is intended for ESP8266 Non-OS SDK implementation.
// For the first prototype we implement a simple blocking FSK
// transmitter using GPIO and busy-wait delays. This is sufficient
// to generate a deterministic high-frequency signal for lab testing.

#include "c_types.h"
#include "osapi.h"
#include "user_interface.h"
#include "gpio.h"
#include "eagle_soc.h"
#include "plc_dataplane_internal.h"

static plc_tx_handle_t g_ctx_storage;
static plc_tx_config_t g_cfg;
static bool g_initialized = false;

static uint32_t g_symbol_time_us = 0;
static uint32_t g_half_period0_us = 0; // For bit 0
static uint32_t g_half_period1_us = 0; // For bit 1

static plc_rx_handle_t g_rx_ctx_storage;
static plc_rx_config_t g_rx_cfg;
static bool g_rx_initialized = false;

static uint32_t g_rx_symbol_time_us = 0;
static uint32_t g_rx_half_period0_us = 0;
static uint32_t g_rx_half_period1_us = 0;
static uint32_t g_rx_expected_edges0 = 0;
static uint32_t g_rx_expected_edges1 = 0;

static bool plc_hal_select_pin_func(uint8_t gpio_pin)
{
    switch (gpio_pin) {
        case 0:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
            return true;
        case 2:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
            return true;
        case 4:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
            return true;
        case 5:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
            return true;
        case 12:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
            return true;
        case 13:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
            return true;
        case 14:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
            return true;
        case 15:
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
            return true;
        default:
            return false;
    }
}

// Configure GPIO and precompute timing parameters for FSK.
bool plc_hal_init(const plc_tx_config_t *config)
{
    if (!config) {
        return false;
    }

    if (config->modulation != PLC_MOD_FSK) {
        // Only FSK is supported in this prototype.
        return false;
    }

    if (config->base_frequency_hz == 0 || config->deviation_hz == 0 || config->symbol_rate == 0) {
        return false;
    }

    // Prevent pin conflict with RX.
    if (g_rx_initialized && (config->tx_gpio_pin == g_rx_cfg.rx_gpio_pin)) {
        return false;
    }

    g_cfg = *config;

    // Compute symbol duration and half-periods in microseconds.
    uint32_t f0 = g_cfg.base_frequency_hz - g_cfg.deviation_hz;
    uint32_t f1 = g_cfg.base_frequency_hz + g_cfg.deviation_hz;

    if (f0 == 0 || f1 == 0) {
        return false;
    }

    g_symbol_time_us = (uint32_t)(1000000UL / g_cfg.symbol_rate);
    g_half_period0_us = (uint32_t)(500000UL / f0); // half-period = 1/(2*f0) seconds
    g_half_period1_us = (uint32_t)(500000UL / f1);

    if (g_half_period0_us == 0 || g_half_period1_us == 0) {
        return false;
    }

    gpio_init();

    // Configure selected GPIO as output.
    if (!plc_hal_select_pin_func(g_cfg.tx_gpio_pin)) {
        return false;
    }

    GPIO_OUTPUT_SET(g_cfg.tx_gpio_pin, 0);

    g_initialized = true;
    return true;
}

void plc_hal_deinit(void)
{
    if (!g_initialized) {
        return;
    }

    // Drive output low and mark as uninitialized.
    GPIO_OUTPUT_SET(g_cfg.tx_gpio_pin, 0);
    g_initialized = false;
}

plc_tx_handle_t *plc_hal_alloc_context(void)
{
    // For the first prototype we use a single static context instance
    // to avoid heap fragmentation in Non-OS environment.
    return &g_ctx_storage;
}

void plc_hal_free_context(plc_tx_handle_t *ctx)
{
    (void)ctx;
}

// Transmit a single FSK bit as a series of GPIO toggles during one symbol period.
static void plc_hal_send_bit(uint8_t bit)
{
    uint32_t half_period_us = bit ? g_half_period1_us : g_half_period0_us;
    uint32_t elapsed = 0;

    while ((elapsed + (2U * half_period_us)) <= g_symbol_time_us) {
        GPIO_OUTPUT_SET(g_cfg.tx_gpio_pin, 1);
        os_delay_us(half_period_us);
        GPIO_OUTPUT_SET(g_cfg.tx_gpio_pin, 0);
        os_delay_us(half_period_us);
        elapsed += 2U * half_period_us;
    }

    // If there is any remaining time in the symbol, keep the line low.
    if (elapsed < g_symbol_time_us) {
        uint32_t remaining = g_symbol_time_us - elapsed;
        os_delay_us(remaining);
    }
}

int plc_hal_send(plc_tx_handle_t *ctx, const uint8_t *data, size_t length)
{
    (void)ctx;

    if (!g_initialized || !data || length == 0) {
        return -1;
    }

    // Iterate over all bytes, MSB first.
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        for (int bit = 7; bit >= 0; --bit) {
            plc_hal_send_bit((byte >> bit) & 0x01U);
        }
    }

    // Ensure line is low after transmission.
    GPIO_OUTPUT_SET(g_cfg.tx_gpio_pin, 0);

    return 0;
}

bool plc_hal_rx_init(const plc_rx_config_t *config)
{
    if (!config) {
        return false;
    }

    if (config->modulation != PLC_MOD_FSK) {
        // Only FSK is supported in this prototype.
        return false;
    }

    if (config->base_frequency_hz == 0 || config->deviation_hz == 0 || config->symbol_rate == 0) {
        return false;
    }

    // Prevent pin conflict with TX.
    if (g_initialized && (config->rx_gpio_pin == g_cfg.tx_gpio_pin)) {
        return false;
    }

    g_rx_cfg = *config;

    uint32_t f0 = g_rx_cfg.base_frequency_hz - g_rx_cfg.deviation_hz;
    uint32_t f1 = g_rx_cfg.base_frequency_hz + g_rx_cfg.deviation_hz;

    if (f0 == 0 || f1 == 0) {
        return false;
    }

    g_rx_symbol_time_us = (uint32_t)(1000000UL / g_rx_cfg.symbol_rate);
    g_rx_half_period0_us = (uint32_t)(500000UL / f0);
    g_rx_half_period1_us = (uint32_t)(500000UL / f1);

    if (g_rx_symbol_time_us == 0 || g_rx_half_period0_us == 0 || g_rx_half_period1_us == 0) {
        return false;
    }

    g_rx_expected_edges0 = g_rx_symbol_time_us / g_rx_half_period0_us;
    g_rx_expected_edges1 = g_rx_symbol_time_us / g_rx_half_period1_us;

    if (g_rx_expected_edges0 == 0 || g_rx_expected_edges1 == 0) {
        return false;
    }

    gpio_init();

    // Configure selected GPIO as input.
    if (!plc_hal_select_pin_func(g_rx_cfg.rx_gpio_pin)) {
        return false;
    }

    GPIO_DIS_OUTPUT(g_rx_cfg.rx_gpio_pin);

    g_rx_initialized = true;
    return true;
}

void plc_hal_rx_deinit(void)
{
    if (!g_rx_initialized) {
        return;
    }

    g_rx_initialized = false;
}

plc_rx_handle_t *plc_hal_rx_alloc_context(void)
{
    return &g_rx_ctx_storage;
}

void plc_hal_rx_free_context(plc_rx_handle_t *ctx)
{
    (void)ctx;
}

static int plc_hal_abs_diff_u32(uint32_t a, uint32_t b)
{
    return (a > b) ? (int)(a - b) : (int)(b - a);
}

static int plc_hal_rx_wait_rising_edge(uint8_t pin, uint32_t timeout_us)
{
    uint32_t start = system_get_time();
    uint8_t prev = (uint8_t)GPIO_INPUT_GET(pin);
    uint32_t iter = 0;

    while (timeout_us == 0 || (uint32_t)(system_get_time() - start) < timeout_us) {
        uint8_t cur = (uint8_t)GPIO_INPUT_GET(pin);
        if (prev == 0 && cur == 1) {
            return 0;
        }
        prev = cur;

        // Prevent WDT reset while waiting for input signal.
        if ((++iter & 0x3FFFU) == 0) {
            system_soft_wdt_feed();
        }
    }

    return -2;
}

int plc_hal_rx_capture_edges(plc_rx_handle_t *ctx,
                             uint16_t *out_edges,
                             size_t num_symbols,
                             uint32_t timeout_us)
{
    (void)ctx;

    if (!g_rx_initialized || !out_edges || num_symbols == 0) {
        return -1;
    }

    if (g_rx_symbol_time_us == 0) {
        return -1;
    }

    const uint32_t idle_low_us = g_rx_symbol_time_us * 4U;

    uint32_t start = system_get_time();
    uint32_t low_start = 0;
    bool armed = false;

    uint32_t t0_us = 0;

    uint8_t prev_v = (uint8_t)GPIO_INPUT_GET(g_rx_cfg.rx_gpio_pin);

    while (timeout_us == 0 || (uint32_t)(system_get_time() - start) < timeout_us) {
        uint8_t v = (uint8_t)GPIO_INPUT_GET(g_rx_cfg.rx_gpio_pin);
        uint32_t now = system_get_time();

        if (v == 0U) {
            if (low_start == 0U) {
                low_start = now;
            }
            if (!armed && (uint32_t)(now - low_start) >= idle_low_us) {
                armed = true;
            }
        } else {
            if (armed && prev_v == 0U && v == 1U) {
                t0_us = now;
                break;
            }
            low_start = 0U;
        }

        prev_v = v;

        if ((now & 0x7FFFU) == 0U) {
            system_soft_wdt_feed();
        }
    }

    if (t0_us == 0U) {
        return -2;
    }

    for (size_t i = 0; i < num_symbols; ++i) {
        uint32_t win_start = t0_us + (uint32_t)(i * g_rx_symbol_time_us);
        uint32_t win_end = win_start + g_rx_symbol_time_us;

        while ((int32_t)(system_get_time() - win_start) < 0) {
            // spin
        }

        uint8_t prev = (uint8_t)GPIO_INPUT_GET(g_rx_cfg.rx_gpio_pin);
        uint16_t edges = 0;

        while ((int32_t)(system_get_time() - win_end) < 0) {
            uint8_t cur = (uint8_t)GPIO_INPUT_GET(g_rx_cfg.rx_gpio_pin);
            if (cur != prev) {
                edges++;
                prev = cur;
            }
        }

        out_edges[i] = edges;

        if ((i & 0x1FU) == 0U) {
            system_soft_wdt_feed();
        }
    }

    return 0;
}

int plc_hal_rx_read_bit(plc_rx_handle_t *ctx, uint8_t *bit, uint32_t timeout_us)
{
    if (!bit) {
        return -1;
    }

    uint16_t edges = 0;
    int rc = plc_hal_rx_capture_edges(ctx, &edges, 1, timeout_us);
    if (rc != 0) {
        return rc;
    }

    int d0 = plc_hal_abs_diff_u32(edges, g_rx_expected_edges0);
    int d1 = plc_hal_abs_diff_u32(edges, g_rx_expected_edges1);

    *bit = (d1 < d0) ? 1U : 0U;
    return 0;
}

#endif // !defined(ARDUINO)
