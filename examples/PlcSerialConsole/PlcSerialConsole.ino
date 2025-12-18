#include <Arduino.h>
#include <plc_dataplane.h>
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif

static plc_tx_handle_t *s_tx = nullptr;
static char s_buf[PLC_TX_MAX_PAYLOAD_BYTES];
static size_t s_len = 0;

// Shared secret for keyed CRC (TX must match RX)
static const uint8_t SECRET_KEY[] = {'p','l','c','-','k','e','y','1'};

void setup() {
  Serial.begin(115200);
  // For boards with native USB this can block; on ESP8266 it returns immediately.
  while (!Serial) {
    yield();
  }

  // Disable WiFi to reduce RF noise and power usage
#ifdef ARDUINO_ARCH_ESP8266
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
#endif

  plc_tx_config_t cfg = {};
  cfg.modulation = PLC_MOD_FSK;
  cfg.base_frequency_hz = 20000;   // 20 kHz carrier
  cfg.deviation_hz = 5000;         // +/- 5 kHz deviation
  cfg.symbol_rate = 1000;          // 1 ksymbol/s
  cfg.tx_gpio_pin = D4;            // D4 (GPIO2)

  // Keyed CRC configuration
  cfg.secret_key_len = sizeof(SECRET_KEY);
  memcpy(cfg.secret_key, SECRET_KEY, cfg.secret_key_len);

  s_tx = plc_tx_init(&cfg);
  if (!s_tx) {
    Serial.println(F("plc_tx_init failed"));
  } else {
    Serial.println(F("PLC FSK console ready. Type text and press Enter."));
  }
}

void loop() {
  if (!s_tx) {
    delay(1000);
    return;
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    Serial.write(c);  // echo back

    if (c == '\r' || c == '\n') {
      if (s_len == 0) {
        continue;
      }

      int rc = plc_tx_send(s_tx, (const uint8_t *)s_buf, s_len);
      Serial.println();
      if (rc != 0) {
        Serial.println(F("plc_tx_send failed"));
      } else {
        Serial.println(F("Line sent over PLC FSK"));
      }

      s_len = 0;
    } else {
      if (s_len < sizeof(s_buf)) {
        s_buf[s_len++] = c;
      }
    }
  }
}
