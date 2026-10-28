#pragma once
// Modbus RTU frame decoder for a passive sniffer.
// Plain C++ with no Arduino/ESP-IDF dependencies, so the exact same code is
// built into the firmware and into the host-side test (host_test/).

#include <stdint.h>
#include <stddef.h>

uint16_t mb_crc16(const uint8_t* p, size_t n);
bool     mb_crc_ok(const uint8_t* p, size_t len);    // last 2 bytes = CRC (lo, hi)

enum FrameKind : uint8_t {
  FK_REQUEST,     // master -> slave
  FK_RESPONSE,    // slave -> master, paired with the request we saw
  FK_EXCEPTION,   // slave -> master, fc | 0x80
  FK_ORPHAN,      // valid reply but we never saw (or timed out) its request
  FK_RAW          // bytes that are not a valid Modbus frame (CRC error, noise, other protocol)
};
const char* mb_kind_name(FrameKind k);   // "REQ", "RSP", "EXC", "ORPH", "RAW"

struct FrameInfo {
  uint32_t       t_ms;     // time of the burst this frame came in
  FrameKind      kind;
  bool           crc_ok;   // false only for FK_RAW
  bool           paired;   // response/exception matched to a request
  uint8_t        slave;    // first byte (for FK_RAW: just the first byte seen)
  uint8_t        fc;       // second byte
  uint16_t       len;
  const uint8_t* data;     // valid only during the callback
};

struct DecoderCounters {
  uint32_t requests;       // requests seen (all supported FCs)
  uint32_t responses;      // replies paired with a request
  uint32_t orphans;        // valid replies with no matching request (e.g. joined mid-poll)
  uint32_t exceptions;     // exception replies (paired or not)
  uint32_t valid_frames;   // requests + responses + orphans + exceptions
  uint32_t crc_errors;     // unparsed byte runs that look like Modbus (addr + known FC) but fail CRC
  uint32_t junk_bytes;     // bytes not part of any valid frame
};

class ModbusDecoder {
 public:
  // Called for every valid frame and for every run of unparsed bytes.
  typedef void (*FrameFn)(void* ctx, const FrameInfo& f);
  // Called for each register/coil value learned from a request+reply pair.
  // fc is the function code that carried it (1,2,3,4 reads; 5,6,15,16 writes).
  typedef void (*RegFn)(void* ctx, uint8_t slave, uint8_t fc, uint16_t addr,
                        uint16_t value, uint32_t t_ms);

  explicit ModbusDecoder(uint32_t resp_timeout_ms = 1000);
  void set_sinks(FrameFn frame_fn, RegFn reg_fn, void* ctx);
  void reset();                                       // clears counters and pending request

  // Feed one burst of bytes (delimited by bus silence). A burst may contain
  // several frames, e.g. a request and its reply back-to-back.
  void feed(const uint8_t* buf, size_t n, uint32_t t_ms);

  DecoderCounters counters;

 private:
  struct Pending {
    bool     valid;
    uint8_t  slave, fc;
    uint16_t start, qty;
    uint32_t t;
    uint16_t len;
    uint8_t  req[256];     // copy of the request (write values come from here)
  };

  size_t try_frame(const uint8_t* p, size_t n, uint32_t t, FrameInfo* f);
  size_t expected_reply_len(const uint8_t* p, size_t n) const;
  void   emit_values(const uint8_t* reply, uint32_t t);
  void   emit(const FrameInfo& f);
  void   flush_raw(const uint8_t* p, size_t n, uint32_t t);

  Pending  pending_;
  uint32_t timeout_ms_;
  FrameFn  frame_fn_;
  RegFn    reg_fn_;
  void*    ctx_;
};
