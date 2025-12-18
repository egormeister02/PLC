#ifndef PLC_DATAPLANE_H
#define PLC_DATAPLANE_H

#include "c_types.h"

// Maximum payload length (in bytes) accepted by plc_tx_send.
// The library will add its own framing (preamble, length, checksum)
// on top of this payload.
#define PLC_TX_MAX_PAYLOAD_BYTES 128

// Length of preamble in bytes (AA 55 repeated).
#define PLC_TX_PREAMBLE_LEN 4

// Default RX GPIO pin.
// GPIO0 maps to D3 on many ESP8266 dev boards (e.g. NodeMCU).
#define PLC_RX_DEFAULT_GPIO_PIN 0

// Maximum secret key length (bytes) for keyed CRC.
// When secret_key_len > 0 the CRC8 is computed over [LEN][PAYLOAD][KEY].
#define PLC_SECRET_KEY_MAX_LEN 32

#ifdef __cplusplus
extern "C" {
#endif

// Public API for PLC data-plane signal source.
// This library encodes messages into a high-frequency waveform
// suitable for power-line communication (PLC).

// Opaque handle type for a PLC transmitter instance.
typedef struct plc_tx_context plc_tx_handle_t;

// Opaque handle type for a PLC receiver instance.
typedef struct plc_rx_context plc_rx_handle_t;

// Basic modulation type for the first prototype.
typedef enum {
    PLC_MOD_FSK = 0,   // Frequency Shift Keying (simple 2-tone FSK)
    PLC_MOD_ASK = 1    // Amplitude Shift Keying (if supported by hardware)
} plc_modulation_t;

// Configuration for the PLC transmitter.
typedef struct {
    plc_modulation_t modulation;   // Selected modulation scheme
    uint32_t base_frequency_hz;    // Base carrier frequency
    uint32_t deviation_hz;         // Frequency deviation for FSK
    uint32_t symbol_rate;          // Symbols per second
    uint8_t tx_gpio_pin;           // GPIO pin used for output (GPIO number)

    // Keyed-CRC configuration: the same key must be used by TX and RX.
    uint8_t secret_key[PLC_SECRET_KEY_MAX_LEN]; // Application secret
    uint8_t secret_key_len;                     // 0..PLC_SECRET_KEY_MAX_LEN
} plc_tx_config_t;

// Configuration for the PLC receiver.
// Timing parameters (base_frequency_hz, deviation_hz, symbol_rate) must match
// the transmitter settings.
//
// If the structure is zero-initialized, rx_gpio_pin defaults to PLC_RX_DEFAULT_GPIO_PIN.
typedef struct {
    plc_modulation_t modulation;   // Selected modulation scheme
    uint32_t base_frequency_hz;    // Base carrier frequency
    uint32_t deviation_hz;         // Frequency deviation for FSK
    uint32_t symbol_rate;          // Symbols per second
    uint8_t rx_gpio_pin;           // GPIO pin used for input (GPIO number)

    // Keyed-CRC configuration: must match the transmitter.
    uint8_t secret_key[PLC_SECRET_KEY_MAX_LEN];
    uint8_t secret_key_len;        // 0..PLC_SECRET_KEY_MAX_LEN
} plc_rx_config_t;

// Initialize low-level hardware and create transmitter context.
// Returns NULL on failure.
plc_tx_handle_t *plc_tx_init(const plc_tx_config_t *config);

// Send raw bytes over PLC. This function is synchronous in the
// first prototype and blocks until transmission is finished.
// Returns 0 on success, negative value on error.
int plc_tx_send(plc_tx_handle_t *handle, const uint8_t *data, size_t length);

// Free transmitter context and release hardware resources.
void plc_tx_deinit(plc_tx_handle_t *handle);

// Initialize low-level hardware and create receiver context.
// Returns NULL on failure.
plc_rx_handle_t *plc_rx_init(const plc_rx_config_t *config);

// Receive a framed message: [PREAMBLE][LEN][PAYLOAD][CRC8].
//
// On success returns 0 and writes payload to out_payload with its length
// stored in out_length.
//
// On checksum mismatch returns -4.
//
// If timeout_ms is 0, the call blocks indefinitely.
int plc_rx_read(plc_rx_handle_t *handle,
                uint8_t *out_payload,
                size_t out_max_len,
                size_t *out_length,
                uint32_t timeout_ms);

#define PLC_RX_DEBUG_PREVIEW_BYTES 16

typedef struct {
    uint8_t valid;
    int last_rc;
    uint16_t best_pos_bits;
    uint16_t mean0;
    uint16_t mean1;
    uint16_t sep;
    uint8_t preamble_bit_errors;
    uint8_t decoded_preamble[4];
    uint8_t decoded_len;
    uint8_t rx_crc;
    uint8_t calc_crc;
    uint8_t preview_len;
    uint8_t preview[PLC_RX_DEBUG_PREVIEW_BYTES];
    uint16_t pre0_min;
    uint16_t pre0_max;
    uint16_t pre1_min;
    uint16_t pre1_max;
} plc_rx_debug_t;

// Copy last receiver decode debug information.
// Useful to inspect what the decoder sees when CRC does not match.
// Returns 0 on success, negative value on error.
int plc_rx_get_last_debug(plc_rx_debug_t *out);

// Free receiver context and release hardware resources.
void plc_rx_deinit(plc_rx_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // PLC_DATAPLANE_H
