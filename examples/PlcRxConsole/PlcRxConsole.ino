#include <Arduino.h>
#include <plc_dataplane.h>
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif

static plc_rx_handle_t *s_rx = nullptr;
static uint8_t s_rx_buf[PLC_TX_MAX_PAYLOAD_BYTES];
static uint8_t s_rx_pin = D3;

// Shared secret for keyed CRC (must match transmitter)
static const uint8_t SECRET_KEY[] = {'p','l','c','-','k','e','y','1'};

void setup() {
  Serial.begin(115200);
  delay(200);

  // Disable WiFi to reduce RF noise and power usage
#ifdef ARDUINO_ARCH_ESP8266
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
#endif

  // Visual liveness indicator.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println();
  Serial.print(F("PlcRxConsole boot "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
  Serial.println(F("Type any character to get an 'alive' response"));

  plc_rx_config_t cfg = {};
  cfg.modulation = PLC_MOD_FSK;
  cfg.base_frequency_hz = 20000;
  cfg.deviation_hz = 5000;
  cfg.symbol_rate = 1000;

  // Default RX pin is D3 (GPIO0). It must be different from TX pin.
  cfg.rx_gpio_pin = D3;
  s_rx_pin = cfg.rx_gpio_pin;

  // Keyed CRC configuration
  cfg.secret_key_len = sizeof(SECRET_KEY);
  memcpy(cfg.secret_key, SECRET_KEY, cfg.secret_key_len);

  Serial.print(F("RX pin (Arduino pin number)="));
  Serial.println((int)s_rx_pin);
  Serial.print(F("RX pin initial read="));
  Serial.println((int)digitalRead(s_rx_pin));

  s_rx = plc_rx_init(&cfg);
  if (!s_rx) {
    Serial.println(F("plc_rx_init failed"));
  } else {
    Serial.println(F("PLC RX console ready"));
  }
}

void loop() {
  static uint32_t last_heartbeat_ms = 0;
  static uint32_t last_led_toggle_ms = 0;
  static bool led_on = false;

  // Visual heartbeat (ESP8266 LED is usually active-low).
  if ((uint32_t)(millis() - last_led_toggle_ms) >= 250) {
    last_led_toggle_ms = millis();
    led_on = !led_on;
    digitalWrite(LED_BUILTIN, led_on ? LOW : HIGH);
  }

  // Liveness check + debug commands from Serial.
  // Any character: prints alive.
  // 'p': print RX pin and its current digitalRead.
  // 'e': counts edges on RX pin for 2000 ms (useful for bursts sent once per second).
  // 'd': print last decoder debug info.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == 'p') {
      Serial.print(F("RX pin="));
      Serial.print((int)s_rx_pin);
      Serial.print(F(", read="));
      Serial.println((int)digitalRead(s_rx_pin));
    } else if (c == 'd') {
      plc_rx_debug_t dbg;
      int drc = plc_rx_get_last_debug(&dbg);
      if (drc != 0 || dbg.valid == 0) {
        Serial.println(F("RX debug: not available"));
      } else {
        Serial.println(F("RX debug:"));
        Serial.print(F("  last_rc="));
        Serial.println(dbg.last_rc);
        Serial.print(F("  best_pos_bits="));
        Serial.println(dbg.best_pos_bits);
        Serial.print(F("  mean0="));
        Serial.print(dbg.mean0);
        Serial.print(F(", mean1="));
        Serial.print(dbg.mean1);
        Serial.print(F(", sep="));
        Serial.println(dbg.sep);
        Serial.print(F("  preamble_bit_errors="));
        Serial.println(dbg.preamble_bit_errors);
        Serial.print(F("  decoded_preamble="));
        for (int i = 0; i < 4; ++i) {
          if (dbg.decoded_preamble[i] < 16) {
            Serial.print('0');
          }
          Serial.print(dbg.decoded_preamble[i], HEX);
          if (i != 3) {
            Serial.print(' ');
          }
        }
        Serial.println();
        Serial.print(F("  decoded_len="));
        Serial.println(dbg.decoded_len);
        Serial.print(F("  rx_crc="));
        if (dbg.rx_crc < 16) {
          Serial.print('0');
        }
        Serial.print(dbg.rx_crc, HEX);
        Serial.print(F(", calc_crc="));
        if (dbg.calc_crc < 16) {
          Serial.print('0');
        }
        Serial.println(dbg.calc_crc, HEX);
        Serial.print(F("  pre0_min="));
        Serial.print(dbg.pre0_min);
        Serial.print(F(", pre0_max="));
        Serial.print(dbg.pre0_max);
        Serial.print(F(", pre1_min="));
        Serial.print(dbg.pre1_min);
        Serial.print(F(", pre1_max="));
        Serial.println(dbg.pre1_max);

        Serial.print(F("  payload_preview_hex="));
        for (int i = 0; i < dbg.preview_len; ++i) {
          if (dbg.preview[i] < 16) {
            Serial.print('0');
          }
          Serial.print(dbg.preview[i], HEX);
          Serial.print(' ');
        }
        Serial.println();

        Serial.print(F("  payload_preview_ascii="));
        for (int i = 0; i < dbg.preview_len; ++i) {
          char ch = (char)dbg.preview[i];
          if (ch >= 32 && ch <= 126) {
            Serial.print(ch);
          } else {
            Serial.print('.');
          }
        }
        Serial.println();
      }
    } else if (c == 'e') {
      Serial.print(F("edge test on RX pin="));
      Serial.println((int)s_rx_pin);
      const uint32_t window_us = 2000000;
      uint32_t start = micros();
      uint8_t prev = (uint8_t)digitalRead(s_rx_pin);
      uint32_t edges = 0;
      uint32_t ones = 0;
      uint32_t samples = 0;

      while ((uint32_t)(micros() - start) < window_us) {
        uint8_t cur = (uint8_t)digitalRead(s_rx_pin);
        if (cur != prev) {
          edges++;
          prev = cur;
        }
        ones += (cur ? 1U : 0U);
        samples++;
        if ((samples & 0x3FFFU) == 0) {
          yield();
        }
      }

      Serial.print(F("edges_2000ms="));
      Serial.print(edges);
      Serial.print(F(", samples="));
      Serial.print(samples);
      Serial.print(F(", ones_pct="));
      Serial.println((samples == 0) ? 0 : (100U * ones / samples));
    } else {
      Serial.print(F("alive, uptime_ms="));
      Serial.println(millis());
    }
  }

  if (!s_rx) {
    // If RX init failed, still keep the console alive.
    if ((uint32_t)(millis() - last_heartbeat_ms) >= 1000) {
      last_heartbeat_ms = millis();
      Serial.println(F("heartbeat (RX not initialized)"));
    }
    delay(10);
    return;
  }

  size_t rx_len = 0;

  // Use a timeout so the loop stays responsive.
  int rc = plc_rx_read(s_rx, s_rx_buf, sizeof(s_rx_buf), &rx_len, 250);

  if (rc == 0) {
    Serial.print(F("RX ("));
    Serial.print(rx_len);
    Serial.print(F(" bytes): "));
    Serial.write(s_rx_buf, rx_len);
    Serial.println();
  } else if (rc == -4) {
    Serial.println(F("RX error: CRC mismatch"));
  } else if (rc != -3) {
    Serial.print(F("RX error: "));
    Serial.println(rc);
  }

  if ((uint32_t)(millis() - last_heartbeat_ms) >= 1000) {
    last_heartbeat_ms = millis();
    Serial.println(F("heartbeat"));
  }

  yield();
}
