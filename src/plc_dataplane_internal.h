#ifndef PLC_DATAPLANE_INTERNAL_H
#define PLC_DATAPLANE_INTERNAL_H

#include "plc_dataplane.h"

struct plc_tx_context {
    plc_tx_config_t cfg;
};

struct plc_rx_context {
    plc_rx_config_t cfg;
};

#endif /* PLC_DATAPLANE_INTERNAL_H */
