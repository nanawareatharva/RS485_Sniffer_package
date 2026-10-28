// =====================================================================
//  rs485_capture.cpp  —  passive RS485 / Modbus RTU capture
//
//  !!! LISTEN ONLY. This file must NEVER call write(), print(), printf(),
//  !!! println() or flush() on the RS485 UART (Serial1, called RS below).
//  !!! The onboard transceiver drives the bus whenever TX is active, so a
//  !!! single write would put this device on the bus.
//
//  TX (GPIO16) stays assigned to the UART so it idles HIGH. Baud/parity
//  changes go through the IDF driver instead of Serial1.end(), because
//  end() detaches the pins and would leave TX floating.
// =====================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include "driver/uart.h"
#include "config.h"
#include "rs485_capture.h"

static HardwareSerial& RS = Serial1;
static const uart_port_t RS_PORT = UART_NUM_1;     // Serial1's IDF port

const uint32_t RS485_BAUDS[NUM_BAUDS] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
static const char* const PARITY_NAMES[NUM_PARITIES] = {"8N1", "8E1", "8O1", "8N2"};
const char* rs485_parity_name(uint8_t p) { return p < NUM_PARITIES ? PARITY_NAMES[p] : "?"; }

// ------------------------------------------------ capture queue (UART cb -> parser)
#define CHUNK_MAX    256
#define CHUNK_QUEUE  32
struct Chunk {
  uint32_t t_ms;
  uint16_t len;
  uint8_t  gen;            // UART config generation; stale chunks are dropped
  bool     more;           // more bytes of the same burst follow in the next chunk
  uint8_t  data[CHUNK_MAX];
};
static QueueHandle_t        s_chunk_q;
static std::atomic<uint8_t> s_gen{0};
static portMUX_TYPE         s_drop_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t             s_dropped;

// ------------------------------------------------ commands (UI -> parser task)
enum CmdType : uint8_t { CMD_APPLY, CMD_SCAN };
struct Cmd { CmdType type; uint32_t baud; uint8_t parity; };
static QueueHandle_t     s_cmd_q;
static std::atomic<bool> s_scan_cancel{false};

// ------------------------------------------------ shared data, guarded by s_mtx
static SemaphoreHandle_t s_mtx;
static FrameRec*  s_frames;          // PSRAM ring of FRAME_LOG_SIZE
static uint32_t   s_frame_seq;       // seq of the newest record (0 = empty)
static RegEntry*  s_regs;            // PSRAM, sorted by key
static uint32_t   s_reg_count;
static BusStats   s_stats;
static ScanStatus s_scan;
static ModbusDecoder s_dec(MB_RESP_TIMEOUT_MS);

// ------------------------------------------------ parser-task-only state
#define ASM_MAX    (16 * CHUNK_MAX)  // one burst is assembled here before decoding
#define PRINT_MAX  16384             // serial mirror text built while holding the lock
static uint8_t*  s_asm;
static size_t    s_asm_len;
static uint32_t  s_asm_t;
static char*     s_print;
static size_t    s_print_len;
static uint32_t  s_baud;
static uint8_t   s_parity;

static inline void lock()   { xSemaphoreTake(s_mtx, portMAX_DELAY); }
static inline void unlock() { xSemaphoreGive(s_mtx); }

// =====================================================================
//  UART receive callback. Runs in the Arduino UART event task after
//  MB_RX_TIMEOUT_SYMBOLS of bus silence. Only copies bytes into the queue.
// =====================================================================
static void on_uart_rx() {
  static Chunk c;                        // only ever used by this one task
  size_t avail;
  while ((avail = RS.available()) > 0) {
    const size_t n = RS.read(c.data, avail < CHUNK_MAX ? avail : CHUNK_MAX);
    if (n == 0) break;
    c.len  = (uint16_t)n;
    c.t_ms = millis();
    c.gen  = s_gen.load();
    c.more = RS.available() > 0;
    if (xQueueSend(s_chunk_q, &c, 0) != pdTRUE) {
      portENTER_CRITICAL(&s_drop_mux);
      s_dropped += n;
      portEXIT_CRITICAL(&s_drop_mux);
    }
  }
}

// =====================================================================
//  Decoder sinks. Called from s_dec.feed(), which runs under s_mtx.
// =====================================================================
static void print_append(const char* fmt, ...) {
  if (s_print_len >= PRINT_MAX - 1) return;
  va_list ap;
  va_start(ap, fmt);
  const int w = vsnprintf(s_print + s_print_len, PRINT_MAX - s_print_len, fmt, ap);
  va_end(ap);
  if (w > 0) s_print_len += (size_t)w;
  if (s_print_len > PRINT_MAX - 1) s_print_len = PRINT_MAX - 1;
}

static void mirror_line(const FrameInfo& f) {
  // [ms] len=NN crc=OK slave=1 fc=03 REQ : 01 03 00 00 00 07 ...
  print_append("[%lu] len=%u crc=%s slave=%u fc=%02X %s :", (unsigned long)f.t_ms, (unsigned)f.len,
               f.crc_ok ? "OK" : "BAD", (unsigned)f.slave, (unsigned)f.fc, mb_kind_name(f.kind));
  for (size_t i = 0; i < f.len; i++) print_append(" %02X", f.data[i]);
  print_append("\r\n");
}

static void on_frame(void*, const FrameInfo& f) {
  const uint32_t seq = ++s_frame_seq;
  FrameRec& r = s_frames[(seq - 1) % FRAME_LOG_SIZE];
  r.seq = seq;
  r.t_ms = f.t_ms;
  r.kind = f.kind;
  r.crc_ok = f.crc_ok;
  r.paired = f.paired;
  r.slave = f.slave;
  r.fc = f.fc;
  r.len = f.len;
  memcpy(r.bytes, f.data, f.len < FRAME_MAX_BYTES ? f.len : FRAME_MAX_BYTES);
  s_stats.frames_logged = seq;

  if (f.kind != FK_RAW) {
    s_stats.last_valid_ms = f.t_ms ? f.t_ms : 1;
    if (f.slave >= 1 && f.slave <= 247) s_stats.slaves_seen[f.slave >> 3] |= (uint8_t)(1u << (f.slave & 7));
  }
#if SNIFFER_DEBUG
  mirror_line(f);
#endif
}

static inline uint32_t reg_key(uint8_t slave, uint8_t fc, uint16_t addr) {
  return ((uint32_t)slave << 24) | ((uint32_t)fc << 16) | addr;
}

// Binary search; returns index of key or the insert position (found=false).
static uint32_t reg_find(uint32_t key, bool* found) {
  uint32_t lo = 0, hi = s_reg_count;
  while (lo < hi) {
    const uint32_t mid = (lo + hi) / 2;
    const uint32_t k = reg_key(s_regs[mid].slave, s_regs[mid].fc, s_regs[mid].addr);
    if (k == key) { *found = true; return mid; }
    if (k < key) lo = mid + 1; else hi = mid;
  }
  *found = false;
  return lo;
}

static void on_reg(void*, uint8_t slave, uint8_t fc, uint16_t addr, uint16_t value, uint32_t t) {
  bool found;
  const uint32_t i = reg_find(reg_key(slave, fc, addr), &found);
  if (!found) {
    if (s_reg_count >= REG_TABLE_SIZE) return;           // table full: ignore new keys
    memmove(&s_regs[i + 1], &s_regs[i], (s_reg_count - i) * sizeof(RegEntry));
    s_regs[i] = RegEntry{slave, fc, addr, 0, 0, 0};
    s_reg_count++;
    s_stats.reg_count = s_reg_count;
  }
  s_regs[i].value = value;
  s_regs[i].t_ms = t;
  s_regs[i].updates++;
}

// =====================================================================
//  Parser task helpers
// =====================================================================
static uint32_t serial_cfg(uint8_t p) {
  switch (p) {
    case 1:  return SERIAL_8E1;
    case 2:  return SERIAL_8O1;
    case 3:  return SERIAL_8N2;
    default: return SERIAL_8N1;
  }
}

static bool cfg_valid(uint32_t baud, uint8_t parity) {
  if (parity >= NUM_PARITIES) return false;
  for (uint32_t b : RS485_BAUDS) if (b == baud) return true;
  return false;
}

static void load_cfg() {
  Preferences p;
  p.begin("rs485sniff", false);
  s_baud = p.getUInt("baud", MB_DEFAULT_BAUD);
  s_parity = p.getUChar("par", MB_DEFAULT_PARITY);
  p.end();
  if (!cfg_valid(s_baud, s_parity)) { s_baud = MB_DEFAULT_BAUD; s_parity = MB_DEFAULT_PARITY; }
}

static void save_cfg(uint32_t baud, uint8_t parity) {
  Preferences p;
  p.begin("rs485sniff", false);
  p.putUInt("baud", baud);
  p.putUChar("par", parity);
  p.end();
}

// Change baud/parity on the running UART. TX stays attached (idle HIGH).
static void uart_reconfigure(uint32_t baud, uint8_t parity) {
  RS.updateBaudRate(baud);
  uart_set_parity(RS_PORT, parity == 1 ? UART_PARITY_EVEN : parity == 2 ? UART_PARITY_ODD : UART_PARITY_DISABLE);
  uart_set_stop_bits(RS_PORT, parity == 3 ? UART_STOP_BITS_2 : UART_STOP_BITS_1);
  RS.setRxTimeout(MB_RX_TIMEOUT_SYMBOLS);              // re-evaluated for the new frame format
  s_gen.store((uint8_t)(s_gen.load() + 1));            // chunks already queued are now stale
  uart_flush_input(RS_PORT);
  s_asm_len = 0;
  s_baud = baud;
  s_parity = parity;
  lock();
  s_stats.baud = baud;
  s_stats.parity = parity;
  unlock();
}

// Append a chunk to the burst buffer. Returns true when a complete burst is ready.
static bool take_chunk(const Chunk& c) {
  if (c.gen != s_gen.load()) return false;              // received with old settings
  if (s_asm_len + c.len > ASM_MAX) s_asm_len = 0;       // cannot happen (flushed below); be safe
  memcpy(s_asm + s_asm_len, c.data, c.len);
  s_asm_len += c.len;
  s_asm_t = c.t_ms;
  return !c.more || s_asm_len > ASM_MAX - CHUNK_MAX;
}

static void process_burst() {
  uint32_t dropped;
  portENTER_CRITICAL(&s_drop_mux);
  dropped = s_dropped;
  portEXIT_CRITICAL(&s_drop_mux);

  lock();
  s_print_len = 0;
  s_stats.total_bytes += (uint32_t)s_asm_len;
  s_stats.last_byte_ms = s_asm_t ? s_asm_t : 1;
  s_stats.dropped_bytes = dropped;
  s_dec.feed(s_asm, s_asm_len, s_asm_t);
  s_stats.dec = s_dec.counters;
  unlock();
  s_asm_len = 0;

#if SNIFFER_DEBUG
  if (s_print_len) Serial.write((const uint8_t*)s_print, s_print_len);   // USB serial, not RS485
#endif
}

static void publish_scan(const ScanStatus& st) {
  lock();
  s_scan = st;
  s_stats.scanning = st.active;
  unlock();
}

// Auto-baud: listen on every baud/parity combination, count CRC-valid frames.
static void run_scan() {
  static ModbusDecoder sd(MB_RESP_TIMEOUT_MS);   // static keeps it off the task stack
  static Chunk c;
  const uint32_t prev_baud = s_baud;
  const uint8_t prev_parity = s_parity;

  ScanStatus st = {};
  st.active = true;
  st.total = NUM_BAUDS * NUM_PARITIES;
  s_scan_cancel.store(false);
  publish_scan(st);
  Serial.println("Auto-baud scan started (listen only)");

  for (uint8_t i = 0; i < st.total && !s_scan_cancel.load(); i++) {
    const uint32_t baud = RS485_BAUDS[i / NUM_PARITIES];
    const uint8_t parity = i % NUM_PARITIES;
    uart_reconfigure(baud, parity);
    sd.reset();
    st.index = i;
    st.cur_baud = baud;
    st.cur_parity = parity;
    st.cur_valid = st.cur_junk = 0;

    const uint32_t start = millis();
    while (!s_scan_cancel.load() && millis() - start < AUTOBAUD_LISTEN_MS) {
      if (xQueueReceive(s_chunk_q, &c, pdMS_TO_TICKS(20)) == pdTRUE && take_chunk(c)) {
        st.bytes_seen += (uint32_t)s_asm_len;
        sd.feed(s_asm, s_asm_len, s_asm_t);
        s_asm_len = 0;
      }
      st.cur_valid = sd.counters.valid_frames;
      st.cur_junk = sd.counters.junk_bytes;
      st.progress_pct = (uint8_t)(((uint32_t)i * AUTOBAUD_LISTEN_MS + (millis() - start)) * 100u /
                                  ((uint32_t)st.total * AUTOBAUD_LISTEN_MS));
      publish_scan(st);
    }
    Serial.printf("  scan %6lu %s : %lu valid frames, %lu junk bytes\r\n", (unsigned long)baud,
                  rs485_parity_name(parity), (unsigned long)st.cur_valid, (unsigned long)st.cur_junk);
    // More valid frames wins; on a tie, fewer junk bytes. 8N1 is tried before 8N2, so it wins exact ties.
    if (st.cur_valid > st.best_valid ||
        (st.cur_valid > 0 && st.cur_valid == st.best_valid && st.cur_junk < st.best_junk)) {
      st.best_valid = st.cur_valid;
      st.best_junk = st.cur_junk;
      st.best_baud = baud;
      st.best_parity = parity;
    }
  }

  st.active = false;
  st.finished = true;
  st.cancelled = s_scan_cancel.load();
  if (!st.cancelled) st.progress_pct = 100;
  if (!st.cancelled && st.best_valid > 0) {
    uart_reconfigure(st.best_baud, st.best_parity);
    save_cfg(st.best_baud, st.best_parity);
    st.found = true;
    Serial.printf("Auto-baud result: %lu %s (%lu valid frames) - saved\r\n", (unsigned long)st.best_baud,
                  rs485_parity_name(st.best_parity), (unsigned long)st.best_valid);
  } else {
    uart_reconfigure(prev_baud, prev_parity);
    Serial.printf("Auto-baud: %s - keeping %lu %s\r\n",
                  st.cancelled ? "cancelled" : "no valid Modbus frames found",
                  (unsigned long)prev_baud, rs485_parity_name(prev_parity));
  }
  publish_scan(st);
}

static void parser_task(void*) {
  static Chunk c;
  for (;;) {
    Cmd cmd;
    while (xQueueReceive(s_cmd_q, &cmd, 0) == pdTRUE) {
      if (cmd.type == CMD_APPLY) {
        uart_reconfigure(cmd.baud, cmd.parity);
        save_cfg(cmd.baud, cmd.parity);
        Serial.printf("RS485: now %lu %s (saved)\r\n", (unsigned long)cmd.baud, rs485_parity_name(cmd.parity));
      } else if (cmd.type == CMD_SCAN) {
        run_scan();
      }
    }
    if (xQueueReceive(s_chunk_q, &c, pdMS_TO_TICKS(50)) == pdTRUE && take_chunk(c)) process_burst();
  }
}

// =====================================================================
//  Public API
// =====================================================================
void rs485_begin() {
  // Drive TX high before the UART takes the pin, so it is never floating.
  pinMode(RS485_TX_PIN, OUTPUT);
  digitalWrite(RS485_TX_PIN, HIGH);

  s_mtx = xSemaphoreCreateMutex();
  s_chunk_q = xQueueCreate(CHUNK_QUEUE, sizeof(Chunk));
  s_cmd_q = xQueueCreate(4, sizeof(Cmd));
  s_frames = (FrameRec*)ps_calloc(FRAME_LOG_SIZE, sizeof(FrameRec));
  s_regs = (RegEntry*)ps_calloc(REG_TABLE_SIZE, sizeof(RegEntry));
  s_asm = (uint8_t*)ps_malloc(ASM_MAX);
  s_print = (char*)ps_malloc(PRINT_MAX);
  if (!s_mtx || !s_chunk_q || !s_cmd_q || !s_frames || !s_regs || !s_asm || !s_print) {
    Serial.println("RS485 sniffer: out of memory (is PSRAM enabled?) - capture NOT started");
    return;
  }
  s_dec.set_sinks(on_frame, on_reg, nullptr);

  load_cfg();
  s_stats.baud = s_baud;
  s_stats.parity = s_parity;

  RS.setRxBufferSize(4096);                                         // must be before begin()
  RS.begin(s_baud, serial_cfg(s_parity), RS485_RX_PIN, RS485_TX_PIN); // TX assigned -> idles HIGH
  RS.setRxTimeout(MB_RX_TIMEOUT_SYMBOLS);
  RS.onReceive(on_uart_rx, true);                                   // fire only on line silence

  xTaskCreatePinnedToCore(parser_task, "rs485_parse", 8192, nullptr, 3, nullptr, 0);
  Serial.printf("RS485 sniffer: %lu baud %s, listen-only (TX idles high, never written)\r\n",
                (unsigned long)s_baud, rs485_parity_name(s_parity));
}

void rs485_get_stats(BusStats* out) {
  lock();
  *out = s_stats;
  unlock();
}

uint32_t rs485_frames_latest(FrameRec* out, uint32_t max, uint32_t* newest_seq) {
  lock();
  const uint32_t seq = s_frame_seq;
  uint32_t n = seq < FRAME_LOG_SIZE ? seq : FRAME_LOG_SIZE;
  if (n > max) n = max;
  for (uint32_t j = 0; j < n; j++) {
    const uint32_t s = seq - n + 1 + j;
    out[j] = s_frames[(s - 1) % FRAME_LOG_SIZE];
  }
  unlock();
  if (newest_seq) *newest_seq = seq;
  return n;
}

uint32_t rs485_regs_copy(RegEntry* out, uint32_t max, uint32_t* total) {
  lock();
  const uint32_t n = s_reg_count < max ? s_reg_count : max;
  memcpy(out, s_regs, n * sizeof(RegEntry));
  if (total) *total = s_reg_count;
  unlock();
  return n;
}

bool rs485_reg_get(uint8_t slave, uint8_t fc, uint16_t addr, uint16_t* value, uint32_t* t_ms) {
  bool found;
  lock();
  const uint32_t i = reg_find(reg_key(slave, fc, addr), &found);
  if (found) { *value = s_regs[i].value; *t_ms = s_regs[i].t_ms; }
  unlock();
  return found;
}

void rs485_apply(uint32_t baud, uint8_t parity) {
  if (!s_cmd_q || !cfg_valid(baud, parity)) return;
  const Cmd cmd = {CMD_APPLY, baud, parity};
  xQueueSend(s_cmd_q, &cmd, 0);
}

void rs485_scan_start() {
  if (!s_cmd_q) return;
  const Cmd cmd = {CMD_SCAN, 0, 0};
  xQueueSend(s_cmd_q, &cmd, 0);
}

void rs485_scan_cancel() { s_scan_cancel.store(true); }

void rs485_get_scan(ScanStatus* out) {
  lock();
  *out = s_scan;
  unlock();
}

void rs485_clear_frames() {
  lock();
  s_frame_seq = 0;
  s_stats.frames_logged = 0;
  unlock();
}

void rs485_clear_all() {
  portENTER_CRITICAL(&s_drop_mux);
  s_dropped = 0;
  portEXIT_CRITICAL(&s_drop_mux);
  lock();
  s_frame_seq = 0;
  s_reg_count = 0;
  s_dec.reset();
  const uint32_t baud = s_stats.baud;
  const uint8_t parity = s_stats.parity;
  const bool scanning = s_stats.scanning;
  memset(&s_stats, 0, sizeof(s_stats));
  s_stats.baud = baud;
  s_stats.parity = parity;
  s_stats.scanning = scanning;
  unlock();
}
