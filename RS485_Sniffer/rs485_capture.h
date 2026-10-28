#pragma once
// Passive RS485 capture for the ESP32-S3-Touch-LCD-7B (Serial1, RX15 / TX16).
// LISTEN ONLY: nothing in this module ever transmits on the RS485 UART.
//
// All functions here are thread-safe: they copy snapshots out under a mutex.
// They never call LVGL, so the UI must call them from lv_timer callbacks
// (which already run inside the LVGL task) and update widgets itself.

#include <stdint.h>
#include <stddef.h>
#include "modbus_decoder.h"

#define FRAME_LOG_SIZE   200     // frames kept in PSRAM
#define FRAME_MAX_BYTES  256     // bytes stored per frame (longer raw runs are truncated, len keeps the real size)
#define REG_TABLE_SIZE   512     // distinct slave+fc+register keys tracked

#define NUM_BAUDS    8
#define NUM_PARITIES 4
extern const uint32_t RS485_BAUDS[NUM_BAUDS];          // 1200 ... 115200
const char* rs485_parity_name(uint8_t parity);          // "8N1", "8E1", "8O1", "8N2"

struct FrameRec {
  uint32_t seq;            // 1, 2, 3 ... since last clear
  uint32_t t_ms;           // millis() when the burst ended
  uint8_t  kind;           // FrameKind
  bool     crc_ok;
  bool     paired;
  uint8_t  slave, fc;
  uint16_t len;            // real length on the wire
  uint8_t  bytes[FRAME_MAX_BYTES];
};

struct RegEntry {
  uint8_t  slave, fc;
  uint16_t addr;
  uint16_t value;
  uint32_t t_ms;           // last update
  uint32_t updates;
};

struct BusStats {
  DecoderCounters dec;
  uint32_t total_bytes;    // every byte received
  uint32_t dropped_bytes;  // lost because the capture queue was full (should stay 0)
  uint32_t last_byte_ms;   // 0 = never
  uint32_t last_valid_ms;  // 0 = never
  uint32_t frames_logged;  // records written to the frame log since clear (incl. RAW)
  uint32_t reg_count;
  uint8_t  slaves_seen[32];// bitmap, bit n = slave n seen in a valid frame
  uint32_t baud;
  uint8_t  parity;
  bool     scanning;
};

struct ScanStatus {
  bool     active;
  bool     finished;       // a scan has completed (or was cancelled) since boot
  bool     found;          // result applied
  bool     cancelled;
  uint8_t  index, total;   // combo being tested (0-based) / number of combos
  uint8_t  progress_pct;
  uint32_t cur_baud;  uint8_t cur_parity;
  uint32_t cur_valid, cur_junk;
  uint32_t best_baud; uint8_t best_parity;
  uint32_t best_valid, best_junk;
  uint32_t bytes_seen;     // total bytes over the whole scan
};

void rs485_begin();                                    // load NVS, start UART + parser task

void rs485_get_stats(BusStats* out);
// Copies the newest frames (up to max, oldest first). Returns how many were copied.
uint32_t rs485_frames_latest(FrameRec* out, uint32_t max, uint32_t* newest_seq);
// Copies the register table (sorted by slave, fc, addr). Returns how many were copied.
uint32_t rs485_regs_copy(RegEntry* out, uint32_t max, uint32_t* total);
bool rs485_reg_get(uint8_t slave, uint8_t fc, uint16_t addr, uint16_t* value, uint32_t* t_ms);

void rs485_apply(uint32_t baud, uint8_t parity);       // async: re-init UART + save to NVS
void rs485_scan_start();                               // async auto-baud scan (listen only)
void rs485_scan_cancel();
void rs485_get_scan(ScanStatus* out);

void rs485_clear_frames();                             // frame log only
void rs485_clear_all();                                // frames, registers, counters
