#include <Arduino.h>
#include <plc_dataplane.h>

// ============================
// Build-time configuration
// ============================

// 1 = master, 0 = slave
#define ROLE_IS_MASTER 1

// Enable extra logging to Serial
#define DEBUG_LOG 1

// PLC physical/link parameters (must match on both devices)
#define PLC_BASE_FREQUENCY_HZ 20000
#define PLC_DEVIATION_HZ      5000
#define PLC_SYMBOL_RATE       1000

// Pins (ESP8266: use 0..15 only; keep TX != RX)
#define PLC_TX_PIN D4  // e.g., GPIO2
#define PLC_RX_PIN D3  // e.g., GPIO0

// Application opcodes
#define APP_OPCODE_REQ     'Q' // request from master
#define APP_OPCODE_RESP    'R' // response with data
#define APP_OPCODE_NO_DATA 'N' // response: no data

// Payload layout: [OPCODE][SEQ_HI][SEQ_LO][TEXT...]
// PLC payload limit is 128 bytes total -> TEXT <= 125 bytes
#define APP_HEADER_SIZE 3
#define APP_MAX_TEXT (PLC_TX_MAX_PAYLOAD_BYTES - APP_HEADER_SIZE)

// Master timing
//
// NOTE: plc_rx_read() captures a fixed window equal to the maximum possible frame.
// At the default symbol rate (1 ksymbol/s) this makes a successful receive take ~1.07s.
// In a request/response exchange both sides perform one such capture, so the time
// between TX and decoded RX on the master is roughly 2x that (+ jitter).
#define PLC_MAX_FRAME_BITS ((PLC_TX_PREAMBLE_LEN + 1U + PLC_TX_MAX_PAYLOAD_BYTES + 1U) * 8U)
#define PLC_RX_CAPTURE_MS  ((PLC_MAX_FRAME_BITS * 1000UL + (PLC_SYMBOL_RATE - 1U)) / (PLC_SYMBOL_RATE))

#define MASTER_PERIOD_MS       500
#define MASTER_REPLY_TIMEOUTMS (2U * PLC_RX_CAPTURE_MS + 500U)

// ============================
// Globals
// ============================

static plc_tx_handle_t* g_tx = nullptr;
static plc_rx_handle_t* g_rx = nullptr;

static uint8_t g_rx_buf[PLC_TX_MAX_PAYLOAD_BYTES];

// Outgoing line buffer (single pending line)
static char   g_line_buf[APP_MAX_TEXT];
static size_t g_line_len = 0;
static bool   g_line_ready = false;

// Master state
enum MasterState : uint8_t { MS_IDLE = 0, MS_WAIT_REPLY = 1 };
static MasterState g_ms = MS_IDLE;
static uint32_t    g_next_send_due = 0;
static uint32_t    g_wait_deadline = 0;
static uint16_t    g_seq = 0;

// ============================
// Logging helpers
// ============================

#if DEBUG_LOG
  #define DBG(...) do { __VA_ARGS__; } while (0)
#else
  #define DBG(...) do {} while (0)
#endif

// ============================
// Utils
// ============================

static inline void app_build_packet(uint8_t opcode, uint16_t seq,
                                    const uint8_t* text, size_t text_len,
                                    uint8_t* out, size_t* out_len)
{
  out[0] = opcode;
  out[1] = (uint8_t)((seq >> 8) & 0xFF);
  out[2] = (uint8_t)(seq & 0xFF);
  if (text && text_len) {
    memcpy(&out[APP_HEADER_SIZE], text, text_len);
    *out_len = APP_HEADER_SIZE + text_len;
  } else {
    *out_len = APP_HEADER_SIZE;
  }
}

static inline bool app_parse_packet(const uint8_t* buf, size_t len,
                                    uint8_t* opcode, uint16_t* seq,
                                    const uint8_t** text, size_t* text_len)
{
  if (!buf || len < APP_HEADER_SIZE) return false;
  *opcode = buf[0];
  *seq = (uint16_t)((buf[1] << 8) | buf[2]);
  *text = (len > APP_HEADER_SIZE) ? (buf + APP_HEADER_SIZE) : nullptr;
  *text_len = (len > APP_HEADER_SIZE) ? (len - APP_HEADER_SIZE) : 0;
  return true;
}

static inline void print_text_line(const uint8_t* text, size_t len)
{
  if (!text || len == 0) {
    Serial.println(F("(empty)"));
    return;
  }
  Serial.write(text, len);
  Serial.println();
}

// ============================
// Serial line input (non-blocking)
// ============================

static void handle_serial_input()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    Serial.write(c); // local echo

    if (c == '\r' || c == '\n') {
      if (g_line_len == 0) continue;
      if (!g_line_ready) {
        g_line_ready = true; // mark one pending line ready to send
      } else {
        // Already have a pending line; keep the next one in the buffer,
        // but it will be considered after current pending is sent.
      }
      // Do not clear g_line_len here: keep content until it's actually sent.
    } else {
      if (g_line_len < APP_MAX_TEXT) {
        g_line_buf[g_line_len++] = c;
      } else {
        // Line too long; drop extra characters
      }
    }
    yield();
  }
}

// After successful send, clear current pending line
static void consume_pending_line()
{
  g_line_len = 0;
  g_line_ready = false;
}

// ============================
// TX helpers
// ============================

static int app_send_req(uint16_t seq, const char* line, size_t len)
{
  uint8_t frame[PLC_TX_MAX_PAYLOAD_BYTES];
  size_t out_len = 0;
  app_build_packet(APP_OPCODE_REQ, seq,
                   (const uint8_t*)line, len,
                   frame, &out_len);
  int rc = plc_tx_send(g_tx, frame, out_len);
  DBG(Serial.print(F("[TX] REQ seq=")); Serial.print(seq);
      Serial.print(F(" len=")); Serial.println((unsigned)len));
  return rc;
}

static int app_send_resp(uint16_t seq, const uint8_t* text, size_t len)
{
  uint8_t frame[PLC_TX_MAX_PAYLOAD_BYTES];
  size_t out_len = 0;
  app_build_packet(APP_OPCODE_RESP, seq, text, len, frame, &out_len);
  int rc = plc_tx_send(g_tx, frame, out_len);
  DBG(Serial.print(F("[TX] RESP seq=")); Serial.print(seq);
      Serial.print(F(" len=")); Serial.println((unsigned)len));
  return rc;
}

static int app_send_no_data(uint16_t seq)
{
  uint8_t frame[PLC_TX_MAX_PAYLOAD_BYTES];
  size_t out_len = 0;
  app_build_packet(APP_OPCODE_NO_DATA, seq, nullptr, 0, frame, &out_len);
  int rc = plc_tx_send(g_tx, frame, out_len);
  DBG(Serial.print(F("[TX] NO_DATA seq=")); Serial.println(seq));
  return rc;
}

// ============================
// Master logic
// ============================

static void master_tick()
{
  uint32_t now = millis();

  switch (g_ms) {
    case MS_IDLE:
      if ((int32_t)(now - g_next_send_due) >= 0) {
        // Prepare request with pending line (if any)
        const char* line = (g_line_ready ? g_line_buf : nullptr);
        const size_t len = (g_line_ready ? g_line_len : 0);

        const uint32_t tx_start_ms = now;
        int rc = app_send_req(g_seq, line, len);
        const uint32_t tx_end_ms = millis();

        // Next request schedule is based on the TX start time.
        g_next_send_due = tx_start_ms + MASTER_PERIOD_MS;

        if (rc == 0) {
          if (g_line_ready) {
            consume_pending_line();
          }
          // Timeout is measured from the end of transmission (plc_tx_send is synchronous).
          g_wait_deadline = tx_end_ms + MASTER_REPLY_TIMEOUTMS;
          g_ms = MS_WAIT_REPLY;
        } else {
          DBG(Serial.print(F("[ERR] plc_tx_send REQ rc=")); Serial.println(rc));
          // Stay in IDLE and retry on the next period.
        }
      }
      break;

    case MS_WAIT_REPLY: {
      size_t rx_len = 0;
      int rc = plc_rx_read(g_rx, g_rx_buf, sizeof(g_rx_buf), &rx_len, 50 /*ms*/);
      now = millis();

      if (rc == 0) {
        uint8_t  op = 0;
        uint16_t rx_seq = 0;
        const uint8_t* text = nullptr;
        size_t   text_len = 0;
        if (app_parse_packet(g_rx_buf, rx_len, &op, &rx_seq, &text, &text_len)) {
          DBG(Serial.print(F("[RX] op=")); Serial.print((char)op);
              Serial.print(F(" seq=")); Serial.print(rx_seq);
              Serial.print(F(" len=")); Serial.println((unsigned)text_len));

          if (rx_seq == g_seq) {
            if (op == APP_OPCODE_RESP) {
              // Always show received user message
              Serial.print(F("[PEER] "));
              print_text_line(text, text_len);
            } else if (op == APP_OPCODE_NO_DATA) {
              DBG(Serial.println(F("[INFO] Slave has no data")));
            } else {
              DBG(Serial.println(F("[WARN] Unexpected opcode")));
            }

            g_seq++;
            g_ms = MS_IDLE;
          } else {
            // Late/duplicate frames can arrive if previous exchanges timed out.
            // Treat older sequence numbers as stale without spamming warnings.
            if ((uint16_t)(g_seq - rx_seq) < 0x8000U) {
              DBG(Serial.print(F("[INFO] Stale frame ignored (expected seq="));
                  Serial.print(g_seq);
                  Serial.print(F(", got seq="));
                  Serial.print(rx_seq);
                  Serial.println(F(")")));
            } else {
              DBG(Serial.print(F("[WARN] Seq mismatch - ignoring frame (expected seq="));
                  Serial.print(g_seq);
                  Serial.print(F(", got seq="));
                  Serial.print(rx_seq);
                  Serial.println(F(")")));
            }
          }
        } else {
          DBG(Serial.println(F("[WARN] Parse failed")));
        }
      } else if (rc == -3) {
        // no frame within 50ms slice; keep waiting
      } else if (rc == -4) {
        DBG(Serial.println(F("[WARN] CRC mismatch")));
      } else {
        DBG(Serial.print(F("[ERR] plc_rx_read rc=")); Serial.println(rc));
      }

      if ((int32_t)(now - g_wait_deadline) >= 0) {
        DBG(Serial.println(F("[TIMEOUT] No response from slave")));
        g_seq++;
        g_ms = MS_IDLE;
      }
      break;
    }
  }
}

// ============================
// Slave logic
// ============================

static void slave_tick()
{
  size_t rx_len = 0;
  int rc = plc_rx_read(g_rx, g_rx_buf, sizeof(g_rx_buf), &rx_len, 50 /*ms*/);
  if (rc == 0) {
    uint8_t  op = 0;
    uint16_t seq = 0;
    const uint8_t* text = nullptr;
    size_t   text_len = 0;

    if (!app_parse_packet(g_rx_buf, rx_len, &op, &seq, &text, &text_len)) {
      DBG(Serial.println(F("[WARN] Parse failed")));
      return;
    }

    DBG(Serial.print(F("[RX] op=")); Serial.print((char)op);
        Serial.print(F(" seq=")); Serial.print(seq);
        Serial.print(F(" len=")); Serial.println((unsigned)text_len));

    if (op == APP_OPCODE_REQ) {
      // Always show master message if present
      if (text_len > 0) {
        Serial.print(F("[PEER] "));
        print_text_line(text, text_len);
      }

      // Reply with our pending line if any, else NO_DATA
      if (g_line_ready && g_line_len > 0) {
        int txrc = app_send_resp(seq, (const uint8_t*)g_line_buf, g_line_len);
        if (txrc == 0) {
          consume_pending_line();
        } else {
          DBG(Serial.print(F("[ERR] plc_tx_send RESP rc=")); Serial.println(txrc));
          // If sending failed, keep the line for the next request
        }
      } else {
        int txrc = app_send_no_data(seq);
        if (txrc != 0) {
          DBG(Serial.print(F("[ERR] plc_tx_send NO_DATA rc=")); Serial.println(txrc));
        }
      }
    } else {
      DBG(Serial.println(F("[WARN] Unexpected opcode on slave")));
    }
  } else if (rc == -3) {
    // no frame in 50ms window; normal
  } else if (rc == -4) {
    DBG(Serial.println(F("[WARN] CRC mismatch")));
  } else if (rc != 0) {
    DBG(Serial.print(F("[ERR] plc_rx_read rc=")); Serial.println(rc));
  }
}

// ============================
// Setup / Loop
// ============================

void setup()
{
  Serial.begin(115200);
  // For ESP8266 Serial is ready immediately.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // indicator

  // TX init
  plc_tx_config_t tx_cfg = {};
  tx_cfg.modulation       = PLC_MOD_FSK;
  tx_cfg.base_frequency_hz= PLC_BASE_FREQUENCY_HZ;
  tx_cfg.deviation_hz     = PLC_DEVIATION_HZ;
  tx_cfg.symbol_rate      = PLC_SYMBOL_RATE;
  tx_cfg.tx_gpio_pin      = PLC_TX_PIN;

  g_tx = plc_tx_init(&tx_cfg);
  if (!g_tx) {
    Serial.println(F("[FATAL] plc_tx_init failed"));
  } else {
    Serial.println(F("[OK] TX init"));
  }

  // RX init
  plc_rx_config_t rx_cfg = {};
  rx_cfg.modulation       = PLC_MOD_FSK;
  rx_cfg.base_frequency_hz= PLC_BASE_FREQUENCY_HZ;
  rx_cfg.deviation_hz     = PLC_DEVIATION_HZ;
  rx_cfg.symbol_rate      = PLC_SYMBOL_RATE;
  rx_cfg.rx_gpio_pin      = PLC_RX_PIN;

  g_rx = plc_rx_init(&rx_cfg);
  if (!g_rx) {
    Serial.println(F("[FATAL] plc_rx_init failed"));
  } else {
    Serial.println(F("[OK] RX init"));
  }

#if ROLE_IS_MASTER
  Serial.println(F("[ROLE] Master"));
  g_ms = MS_IDLE;
  g_next_send_due = millis(); // start immediately
  g_seq = 0;
#else
  Serial.println(F("[ROLE] Slave"));
#endif

#if DEBUG_LOG
  Serial.println(F("[DEBUG] Enabled"));
#else
  Serial.println(F("[DEBUG] Disabled"));
#endif
}

void loop()
{
  static uint32_t last_led = 0;
  const uint32_t now = millis();

  // LED heartbeat (active LOW on many ESP8266 boards)
  if ((uint32_t)(now - last_led) >= 250) {
    last_led = now;
    static bool led_on = false;
    led_on = !led_on;
    digitalWrite(LED_BUILTIN, led_on ? LOW : HIGH);
  }

  handle_serial_input();

#if ROLE_IS_MASTER
  master_tick();
#else
  slave_tick();
#endif

  yield();
}
