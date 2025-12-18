#ifndef PLC_HAL_H
#define PLC_HAL_H

#include "c_types.h"

#include "plc_dataplane.h"

// Hardware abstraction layer for PLC data-plane (TX/RX).
// This layer isolates ESP8266 Non-OS specific details from
// the generic public API.

bool plc_hal_init(const plc_tx_config_t *config);

void plc_hal_deinit(void);

plc_tx_handle_t *plc_hal_alloc_context(void);

void plc_hal_free_context(plc_tx_handle_t *ctx);

int plc_hal_send(plc_tx_handle_t *ctx, const uint8_t *data, size_t length);

bool plc_hal_rx_init(const plc_rx_config_t *config);

void plc_hal_rx_deinit(void);

plc_rx_handle_t *plc_hal_rx_alloc_context(void);

void plc_hal_rx_free_context(plc_rx_handle_t *ctx);

// Capture edge counts for a sequence of symbol windows.
// The function waits for a start condition (idle-low then activity) and then
// captures num_symbols consecutive windows.
// timeout_us applies to waiting for the start condition; 0 means wait indefinitely.
// Returns 0 on success, negative value on error.
int plc_hal_rx_capture_edges(plc_rx_handle_t *ctx,
                             uint16_t *out_edges,
                             size_t num_symbols,
                             uint32_t timeout_us);

// Read one demodulated bit.
// timeout_us == 0 means wait indefinitely.
// Returns 0 on success, negative value on error.
int plc_hal_rx_read_bit(plc_rx_handle_t *ctx, uint8_t *bit, uint32_t timeout_us);

#endif // PLC_HAL_H
