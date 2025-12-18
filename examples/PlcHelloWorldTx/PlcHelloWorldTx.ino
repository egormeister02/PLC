#include <Arduino.h>
#include <plc_dataplane.h>

static plc_tx_handle_t *s_tx = nullptr;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    yield();
  }

  plc_tx_config_t cfg = {};
  cfg.modulation = PLC_MOD_FSK;
  cfg.base_frequency_hz = 20000;
  cfg.deviation_hz = 5000;
  cfg.symbol_rate = 1000;

  // TX pin must be different from RX default pin (D3 / GPIO0).
  cfg.tx_gpio_pin = D4; // D4 (GPIO2)

  s_tx = plc_tx_init(&cfg);
  if (!s_tx) {
    Serial.println(F("plc_tx_init failed"));
  } else {
    Serial.println(F("PLC TX hello world ready"));
  }
}

void loop() {
  if (!s_tx) {
    delay(1000);
    return;
  }

  static const char msg[] = "hello world";

  int rc = plc_tx_send(s_tx, (const uint8_t *)msg, sizeof(msg) - 1);
  if (rc != 0) {
    Serial.print(F("plc_tx_send failed: "));
    Serial.println(rc);
  } else {
    Serial.println(F("TX: hello world"));
  }

  delay(1000);
}
