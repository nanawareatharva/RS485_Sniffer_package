// Modbus RTU frame decoder (passive). Based on the tested parser in
// rs485_reference/modbus_sniffer.cpp: same CRC, same "try the expected reply
// first, then exception, then request, then orphan reply" order, extended to
// FC 01/02/05/06/0F/10, exception pairing, and raw-byte reporting.

#include "modbus_decoder.h"
#include <string.h>

// ------------------------------------------------------------------ CRC
uint16_t mb_crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (int i = 0; i < 8; i++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}
bool mb_crc_ok(const uint8_t* p, size_t len) {
  if (len < 4) return false;
  uint16_t c = mb_crc16(p, len - 2);
  return p[len - 2] == (c & 0xFF) && p[len - 1] == (c >> 8);
}

const char* mb_kind_name(FrameKind k) {
  switch (k) {
    case FK_REQUEST:   return "REQ";
    case FK_RESPONSE:  return "RSP";
    case FK_EXCEPTION: return "EXC";
    case FK_ORPHAN:    return "ORPH";
    default:           return "RAW";
  }
}

// ------------------------------------------------------------- helpers
static inline uint16_t be16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline bool is_read(uint8_t fc) { return fc >= 1 && fc <= 4; }
static inline bool is_slave(uint8_t a) { return a >= 1 && a <= 247; }
static inline bool known_fc(uint8_t fc) {
  fc &= 0x7F;
  return (fc >= 1 && fc <= 6) || fc == 0x0F || fc == 0x10;
}

// --------------------------------------------------------------- class
ModbusDecoder::ModbusDecoder(uint32_t resp_timeout_ms)
    : timeout_ms_(resp_timeout_ms), frame_fn_(nullptr), reg_fn_(nullptr), ctx_(nullptr) {
  reset();
}

void ModbusDecoder::set_sinks(FrameFn frame_fn, RegFn reg_fn, void* ctx) {
  frame_fn_ = frame_fn;
  reg_fn_ = reg_fn;
  ctx_ = ctx;
}

void ModbusDecoder::reset() {
  memset(&counters, 0, sizeof(counters));
  memset(&pending_, 0, sizeof(pending_));
}

void ModbusDecoder::emit(const FrameInfo& f) {
  if (frame_fn_) frame_fn_(ctx_, f);
}

// Length of the reply the pending request asks for, if p looks like it.
size_t ModbusDecoder::expected_reply_len(const uint8_t* p, size_t n) const {
  switch (pending_.fc) {
    case 1: case 2: case 3: case 4: {
      const size_t bc = (pending_.fc <= 2) ? (pending_.qty + 7) / 8 : pending_.qty * 2;
      if (n < 3 || p[2] != bc) return 0;
      return 5 + bc;
    }
    case 5: case 6:     // reply is an echo of the request
      if (n < 8 || memcmp(p, pending_.req, 6) != 0) return 0;
      return 8;
    case 0x0F: case 0x10:
      if (n < 8 || be16(p + 2) != pending_.start || be16(p + 4) != pending_.qty) return 0;
      return 8;
  }
  return 0;
}

// Register/coil values from a paired request + reply.
void ModbusDecoder::emit_values(const uint8_t* reply, uint32_t t) {
  if (!reg_fn_) return;
  const Pending& q = pending_;
  switch (q.fc) {
    case 3: case 4:
      for (uint16_t i = 0; i < q.qty; i++)
        reg_fn_(ctx_, q.slave, q.fc, (uint16_t)(q.start + i), be16(reply + 3 + 2 * i), t);
      break;
    case 1: case 2:
      for (uint16_t i = 0; i < q.qty; i++)
        reg_fn_(ctx_, q.slave, q.fc, (uint16_t)(q.start + i), (reply[3 + i / 8] >> (i % 8)) & 1, t);
      break;
    case 5:
      reg_fn_(ctx_, q.slave, q.fc, q.start, be16(q.req + 4) == 0xFF00 ? 1 : 0, t);
      break;
    case 6:
      reg_fn_(ctx_, q.slave, q.fc, q.start, be16(q.req + 4), t);
      break;
    case 0x0F:
      for (uint16_t i = 0; i < q.qty; i++)
        reg_fn_(ctx_, q.slave, q.fc, (uint16_t)(q.start + i), (q.req[7 + i / 8] >> (i % 8)) & 1, t);
      break;
    case 0x10:
      for (uint16_t i = 0; i < q.qty; i++)
        reg_fn_(ctx_, q.slave, q.fc, (uint16_t)(q.start + i), be16(q.req + 7 + 2 * i), t);
      break;
  }
}

// Try to recognise one frame at p. Returns its length, or 0 if none.
size_t ModbusDecoder::try_frame(const uint8_t* p, size_t n, uint32_t t, FrameInfo* f) {
  if (n < 5) return 0;                      // shortest valid frame: exception (5 bytes)
  const uint8_t addr = p[0], fc = p[1];
  f->t_ms = t; f->slave = addr; f->fc = fc; f->data = p; f->crc_ok = true; f->paired = false;

  if (pending_.valid && (uint32_t)(t - pending_.t) > timeout_ms_) pending_.valid = false;

  // 1) Reply we are waiting for (checked first: an 8-byte reply looks like a request)
  if (pending_.valid && pending_.slave == addr) {
    if (fc == pending_.fc) {
      const size_t len = expected_reply_len(p, n);
      if (len && n >= len && mb_crc_ok(p, len)) {
        emit_values(p, t);
        pending_.valid = false;
        counters.responses++; counters.valid_frames++;
        f->kind = FK_RESPONSE; f->paired = true; f->len = (uint16_t)len;
        return len;
      }
    }
    if (fc == (pending_.fc | 0x80) && mb_crc_ok(p, 5)) {
      pending_.valid = false;
      counters.exceptions++; counters.valid_frames++;
      f->kind = FK_EXCEPTION; f->paired = true; f->len = 5;
      return 5;
    }
  }

  // 2) Exception reply without a request we saw: addr, fc|0x80, code, crc
  if ((fc & 0x80) && is_slave(addr) && known_fc(fc) && mb_crc_ok(p, 5)) {
    counters.exceptions++; counters.valid_frames++;
    f->kind = FK_EXCEPTION; f->len = 5;
    return 5;
  }

  // 3) Requests from the master
  size_t req_len = 0;
  if (is_read(fc) && is_slave(addr) && n >= 8 && mb_crc_ok(p, 8)) {
    const uint16_t qty = be16(p + 4);
    if (qty >= 1 && qty <= (fc <= 2 ? 2000 : 125)) req_len = 8;
  } else if ((fc == 5 || fc == 6) && addr <= 247 && n >= 8 && mb_crc_ok(p, 8)) {
    req_len = 8;
  } else if ((fc == 0x0F || fc == 0x10) && addr <= 247 && n >= 9) {
    const uint16_t qty = be16(p + 4);
    const size_t bc = p[6], len = 9 + bc;
    const size_t want = (fc == 0x0F) ? (qty + 7u) / 8u : qty * 2u;
    const uint16_t max_qty = (fc == 0x0F) ? 1968 : 123;     // Modbus spec limits
    if (qty >= 1 && qty <= max_qty && bc == want && len <= 256 && n >= len && mb_crc_ok(p, len))
      req_len = len;
  }
  if (req_len) {
    if (addr != 0) {                         // broadcast (addr 0) gets no reply
      pending_.valid = true;
      pending_.slave = addr;
      pending_.fc = fc;
      pending_.start = be16(p + 2);
      pending_.qty = (fc == 5 || fc == 6) ? 1 : be16(p + 4);
      pending_.t = t;
      pending_.len = (uint16_t)req_len;
      memcpy(pending_.req, p, req_len);
    }
    counters.requests++; counters.valid_frames++;
    f->kind = FK_REQUEST; f->len = (uint16_t)req_len;
    return req_len;
  }

  // 4) Replies without a request we saw (started listening mid-poll, or request lost)
  if (is_slave(addr)) {
    if (is_read(fc) && p[2] > 0) {
      const size_t len = 5 + p[2];
      if (n >= len && mb_crc_ok(p, len)) {
        counters.orphans++; counters.valid_frames++;
        f->kind = FK_ORPHAN; f->len = (uint16_t)len;
        return len;
      }
    }
    if ((fc == 0x0F || fc == 0x10) && n >= 8 && mb_crc_ok(p, 8)) {
      counters.orphans++; counters.valid_frames++;
      f->kind = FK_ORPHAN; f->len = 8;
      return 8;
    }
  }
  return 0;
}

// A run of bytes that is not a valid frame: report it raw so non-Modbus
// traffic is still visible, and count it.
void ModbusDecoder::flush_raw(const uint8_t* p, size_t n, uint32_t t) {
  counters.junk_bytes += (uint32_t)n;
  if (n >= 4 && is_slave(p[0]) && known_fc(p[1])) counters.crc_errors++;
  FrameInfo f;
  f.t_ms = t; f.kind = FK_RAW; f.crc_ok = false; f.paired = false;
  f.slave = p[0]; f.fc = n > 1 ? p[1] : 0;
  f.len = (uint16_t)(n > 0xFFFF ? 0xFFFF : n);
  f.data = p;
  emit(f);
}

void ModbusDecoder::feed(const uint8_t* buf, size_t n, uint32_t t_ms) {
  size_t i = 0, raw_start = 0;
  bool in_raw = false;
  while (i < n) {
    FrameInfo f;
    const size_t used = try_frame(buf + i, n - i, t_ms, &f);
    if (used) {
      if (in_raw) { flush_raw(buf + raw_start, i - raw_start, t_ms); in_raw = false; }
      emit(f);
      i += used;
    } else {
      if (!in_raw) { in_raw = true; raw_start = i; }
      i++;
    }
  }
  if (in_raw) flush_raw(buf + raw_start, n - raw_start, t_ms);
}
