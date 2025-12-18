#if defined(ARDUINO_ARCH_ESP8266)

#include <Arduino.h>

#include <esp8266_peri.h>

extern "C" {
#include "plc_hal.h"
#include "plc_dataplane_internal.h"
}

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

static inline uint8_t plc_hal_gpio_read_fast(uint32_t mask)
{
    return (GPI & mask) ? 1U : 0U;
}

bool plc_hal_init(const plc_tx_config_t *config)
{
    if (!config) {
        return false;
    }

    if (config->modulation != PLC_MOD_FSK) {
        return false;
    }

    if (config->base_frequency_hz == 0 || config->deviation_hz == 0 || config->symbol_rate == 0) {
        return false;
    }

    if (g_rx_initialized && (config->tx_gpio_pin == g_rx_cfg.rx_gpio_pin)) {
        return false;
    }

    g_cfg = *config;

    uint32_t f0 = g_cfg.base_frequency_hz - g_cfg.deviation_hz;
    uint32_t f1 = g_cfg.base_frequency_hz + g_cfg.deviation_hz;

    if (f0 == 0 || f1 == 0) {
        return false;
    }

    g_symbol_time_us = (uint32_t)(1000000UL / g_cfg.symbol_rate);
    g_half_period0_us = (uint32_t)(500000UL / f0);
    g_half_period1_us = (uint32_t)(500000UL / f1);

    if (g_symbol_time_us == 0 || g_half_period0_us == 0 || g_half_period1_us == 0) {
        return false;
    }

    pinMode(g_cfg.tx_gpio_pin, OUTPUT);
    digitalWrite(g_cfg.tx_gpio_pin, LOW);

    g_initialized = true;
    return true;
}

void plc_hal_deinit(void)
{
    if (!g_initialized) {
        return;
    }

    digitalWrite(g_cfg.tx_gpio_pin, LOW);
    g_initialized = false;
}

plc_tx_handle_t *plc_hal_alloc_context(void)
{
    return &g_ctx_storage;
}

void plc_hal_free_context(plc_tx_handle_t *ctx)
{
    (void)ctx;
}

static void plc_hal_send_bit(uint8_t bit)
{
    uint32_t half_period_us = bit ? g_half_period1_us : g_half_period0_us;

    if (half_period_us == 0 || g_symbol_time_us == 0) {
        return;
    }

    // Fast GPIO writes use GPOS/GPOC and only cover GPIO0..GPIO15.
    if (g_cfg.tx_gpio_pin > 15) {
        // Fallback for unsupported pins.
        uint32_t start_us = micros();
        while ((uint32_t)(micros() - start_us) < g_symbol_time_us) {
            digitalWrite(g_cfg.tx_gpio_pin, HIGH);
            delayMicroseconds(half_period_us);
            digitalWrite(g_cfg.tx_gpio_pin, LOW);
            delayMicroseconds(half_period_us);
        }
        digitalWrite(g_cfg.tx_gpio_pin, LOW);
        return;
    }

    const uint32_t pin_mask = 1U << g_cfg.tx_gpio_pin;
    const uint32_t symbol_cycles = (uint32_t)(((uint64_t)F_CPU * (uint64_t)g_symbol_time_us) / 1000000ULL);
    const uint32_t half_cycles = (uint32_t)(((uint64_t)F_CPU * (uint64_t)half_period_us) / 1000000ULL);

    if (symbol_cycles == 0 || half_cycles == 0) {
        return;
    }

    uint32_t start = ESP.getCycleCount();
    uint32_t end = start + symbol_cycles;
    uint32_t next = start;

    bool level = false;
    uint32_t now = start;

    while ((int32_t)(now - end) < 0) {
        if ((int32_t)(now - next) >= 0) {
            level = !level;
            if (level) {
                GPOS = pin_mask;
            } else {
                GPOC = pin_mask;
            }
            next += half_cycles;
        }
        now = ESP.getCycleCount();
    }

    // Ensure LOW after the symbol.
    GPOC = pin_mask;
}

int plc_hal_send(plc_tx_handle_t *ctx, const uint8_t *data, size_t length)
{
    (void)ctx;

    if (!g_initialized || !data || length == 0) {
        return -1;
    }

    // Fast GPIO writes use GPOS/GPOC and only cover GPIO0..GPIO15.
    if (g_cfg.tx_gpio_pin > 15) {
        for (size_t i = 0; i < length; ++i) {
            uint8_t byte = data[i];
            for (int bit = 7; bit >= 0; --bit) {
                plc_hal_send_bit((byte >> bit) & 0x01U);
            }
        }

        digitalWrite(g_cfg.tx_gpio_pin, LOW);
        return 0;
    }

    const uint32_t pin_mask = 1U << g_cfg.tx_gpio_pin;
    const uint32_t symbol_cycles =
        (uint32_t)(((uint64_t)F_CPU * (uint64_t)g_symbol_time_us) / 1000000ULL);
    const uint32_t half_cycles0 =
        (uint32_t)(((uint64_t)F_CPU * (uint64_t)g_half_period0_us) / 1000000ULL);
    const uint32_t half_cycles1 =
        (uint32_t)(((uint64_t)F_CPU * (uint64_t)g_half_period1_us) / 1000000ULL);

    if (symbol_cycles == 0 || half_cycles0 == 0 || half_cycles1 == 0) {
        return -1;
    }

    // Keep the line LOW in idle and between symbols.
    GPOC = pin_mask;

    // Schedule symbols on an absolute timeline to avoid accumulating drift from per-bit overhead.
    uint32_t symbol_start = ESP.getCycleCount();
    uint32_t bit_index = 0;

    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        for (int bit = 7; bit >= 0; --bit) {
            uint32_t half_cycles = ((byte >> bit) & 0x01U) ? half_cycles1 : half_cycles0;
            uint32_t symbol_end = symbol_start + symbol_cycles;

            uint32_t next = symbol_start;
            bool level = false;

            uint32_t now = ESP.getCycleCount();
            while ((int32_t)(now - symbol_end) < 0) {
                if ((int32_t)(now - next) >= 0) {
                    level = !level;
                    if (level) {
                        GPOS = pin_mask;
                    } else {
                        GPOC = pin_mask;
                    }
                    next += half_cycles;
                }
                now = ESP.getCycleCount();
            }

            // Ensure LOW after the symbol.
            GPOC = pin_mask;

            symbol_start = symbol_end;
            bit_index++;

            // Feed the watchdog occasionally during long frames.
            if ((bit_index & 0x7FU) == 0) {
                ESP.wdtFeed();
            }
        }
    }

    GPOC = pin_mask;
    return 0;
}

bool plc_hal_rx_init(const plc_rx_config_t *config)
{
    if (!config) {
        return false;
    }

    if (config->modulation != PLC_MOD_FSK) {
        return false;
    }

    if (config->base_frequency_hz == 0 || config->deviation_hz == 0 || config->symbol_rate == 0) {
        return false;
    }

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

    // Enable pullup to provide a DC bias when the signal is AC-coupled through a capacitor.
    pinMode(g_rx_cfg.rx_gpio_pin, INPUT_PULLUP);

    // Fast GPIO reads use the GPI register which only covers GPIO0..GPIO15.
    if (g_rx_cfg.rx_gpio_pin > 15) {
        return false;
    }

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

int plc_hal_rx_capture_edges(plc_rx_handle_t *ctx,
                             uint16_t *out_edges,
                             size_t num_symbols,
                             uint32_t timeout_us)
{
    (void)ctx;

    if (!g_rx_initialized || !out_edges || num_symbols == 0) {
        return -1;
    }

    if (g_rx_symbol_time_us == 0 || g_rx_cfg.rx_gpio_pin > 15) {
        return -1;
    }

    const uint32_t pin_mask = 1U << g_rx_cfg.rx_gpio_pin;

    // Wait for an idle-low period before considering activity as a frame start.
    const uint32_t symbol_cycles = (uint32_t)(((uint64_t)F_CPU * (uint64_t)g_rx_symbol_time_us) / 1000000ULL);
    const uint32_t idle_low_cycles = symbol_cycles * 4U;


    const uint32_t cycles_per_us = (F_CPU / 1000000UL);
    uint32_t timeout_cycles = 0;
    if (timeout_us != 0) {
        uint64_t t64 = (uint64_t)timeout_us * (uint64_t)cycles_per_us;
        timeout_cycles = (t64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)t64;
    }

    const uint32_t wait_start = ESP.getCycleCount();
    uint32_t low_start = 0;
    bool armed = false;

    uint32_t t0_cycles = 0;
    uint32_t spin = 0;

    uint8_t prev_v = plc_hal_gpio_read_fast(pin_mask);

    // Reliable start condition for our TX waveform:
    // 1) observe a long idle-low
    // 2) take the first LOW->HIGH transition after that as the start of the first symbol
    while (timeout_us == 0 || (uint32_t)(ESP.getCycleCount() - wait_start) < timeout_cycles) {
        uint8_t v = plc_hal_gpio_read_fast(pin_mask);
        uint32_t now = ESP.getCycleCount();

        if (!armed) {
            if (v == 0U) {
                if (low_start == 0U) {
                    low_start = now;
                }
                if ((uint32_t)(now - low_start) >= idle_low_cycles) {
                    armed = true;
                }
            } else {
                low_start = 0U;
            }
        } else {
            if (prev_v == 0U && v == 1U) {
                t0_cycles = now;
                break;
            }
        }

        prev_v = v;

        if ((++spin & 0x3FFFU) == 0U) {
            ESP.wdtFeed();
        }
    }

    if (t0_cycles == 0U) {
        return -2;
    }

    // Capture consecutive windows aligned to t0_cycles.
    for (size_t i = 0; i < num_symbols; ++i) {
        const uint32_t win_start = t0_cycles + (uint32_t)(i * symbol_cycles);
        const uint32_t win_end = win_start + symbol_cycles;

        // Busy-wait until the window start.
        while ((int32_t)(ESP.getCycleCount() - win_start) < 0) {
            // spin
        }

        uint8_t prev = plc_hal_gpio_read_fast(pin_mask);
        uint16_t edges = 0;

        while ((int32_t)(ESP.getCycleCount() - win_end) < 0) {
            uint8_t cur = plc_hal_gpio_read_fast(pin_mask);
            if (cur != prev) {
                edges++;
                prev = cur;
            }
        }

        out_edges[i] = edges;

        if ((i & 0x1FU) == 0U) {
            ESP.wdtFeed();
        }
    }

    return 0;
}

int plc_hal_rx_read_bit(plc_rx_handle_t *ctx, uint8_t *bit, uint32_t timeout_us)
{
    (void)ctx;

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

#endif // defined(ARDUINO_ARCH_ESP8266)
