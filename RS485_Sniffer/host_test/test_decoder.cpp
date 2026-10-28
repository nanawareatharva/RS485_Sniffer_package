// Host-side test for modbus_decoder.cpp (the same file that runs on the ESP32).
// Arduino does not compile this folder (only the sketch root and src/ are built).
//
// Build + run (Windows, MSVC):   host_test\run_test.bat
// Build + run (g++):             g++ -std=c++17 -I.. test_decoder.cpp ../modbus_decoder.cpp -o t && ./t
//
// Frame bytes/CRCs were generated with an independent Python CRC-16/MODBUS
// implementation; several are well-known published examples.

#include <stdio.h>
#include <string.h>
#include <vector>
#include "../modbus_decoder.h"

typedef std::vector<uint8_t> Bytes;

// ---- real Modbus RTU frames
static const Bytes REQ_03  = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};            // slave 1 read 2 holding regs @0
static const Bytes RSP_03  = {0x01, 0x03, 0x04, 0x00, 0xFA, 0xFF, 0x38, 0x9A, 0x20};      // = 250, -200
static const Bytes REQ_03B = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};            // read reg 0x10
static const Bytes EXC_83  = {0x01, 0x83, 0x02, 0xC0, 0xF1};                              // exception 02 illegal address
static const Bytes ORPH_04 = {0x02, 0x04, 0x02, 0x01, 0x2C, 0xFD, 0x7D};                  // slave 2 FC04 reply = 300
static const Bytes REQ_06  = {0x01, 0x06, 0x00, 0x05, 0x12, 0x34, 0x94, 0xBC};            // write reg 5 = 0x1234 (reply = echo)
static const Bytes REQ_10  = {0x01, 0x10, 0x00, 0x20, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x00, 0x0B, 0x90, 0x72};
static const Bytes RSP_10  = {0x01, 0x10, 0x00, 0x20, 0x00, 0x02, 0x40, 0x02};
static const Bytes REQ_01  = {0x03, 0x01, 0x00, 0x00, 0x00, 0x0A, 0xBD, 0xEF};            // slave 3 read 10 coils
static const Bytes RSP_01  = {0x03, 0x01, 0x02, 0x05, 0x02, 0x42, 0xAD};

// ---- capture what the decoder reports
struct Reg { uint8_t slave, fc; uint16_t addr, value; };
static std::vector<FrameInfo> g_frames;
static std::vector<Reg> g_regs;
static int g_fail = 0, g_pass = 0;

static void on_frame(void*, const FrameInfo& f) {
  printf("    frame: %-4s slave=%-3u fc=%02X len=%-3u crc=%-3s paired=%d :", mb_kind_name(f.kind), f.slave, f.fc,
         f.len, f.crc_ok ? "OK" : "BAD", f.paired);
  for (int i = 0; i < f.len; i++) printf(" %02X", f.data[i]);
  printf("\n");
  g_frames.push_back(f);
}
static void on_reg(void*, uint8_t slave, uint8_t fc, uint16_t addr, uint16_t value, uint32_t) {
  printf("    reg:   slave=%u fc=%02X addr=%u  u16=%u  s16=%d  hex=0x%04X\n", slave, fc, addr, value,
         (int16_t)value, value);
  g_regs.push_back({slave, fc, addr, value});
}

static void check(bool ok, const char* what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  ok ? g_pass++ : g_fail++;
}

static Bytes cat(const Bytes& a, const Bytes& b) { Bytes r = a; r.insert(r.end(), b.begin(), b.end()); return r; }

static void start(ModbusDecoder& d, const char* title) {
  printf("\n=== %s ===\n", title);
  d.reset();
  g_frames.clear();
  g_regs.clear();
}

static void feed(ModbusDecoder& d, const Bytes& b, uint32_t t) {
  printf("  feed @%ums:", t);
  for (uint8_t x : b) printf(" %02X", x);
  printf("\n");
  d.feed(b.data(), b.size(), t);
}

int main() {
  ModbusDecoder d(1000);
  d.set_sinks(on_frame, on_reg, nullptr);

  printf("=== CRC-16/MODBUS reference vectors ===\n");
  const uint8_t v1[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  check(mb_crc16(v1, 9) == 0x4B37, "crc16(\"123456789\") == 0x4B37 (standard check value)");
  const uint8_t v2[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
  check(mb_crc16(v2, 6) == 0xCDC5, "crc16(01 03 00 00 00 0A) == C5 CD on the wire");

  // 1) request + reply in ONE burst
  start(d, "1. FC03 request + reply in one burst");
  feed(d, cat(REQ_03, RSP_03), 1000);
  check(g_frames.size() == 2 && g_frames[0].kind == FK_REQUEST && g_frames[1].kind == FK_RESPONSE,
        "two frames: REQ then RSP");
  check(g_frames.size() == 2 && g_frames[1].paired, "reply is paired with the request");
  check(g_regs.size() == 2 && g_regs[0].addr == 0 && g_regs[0].value == 250 && g_regs[1].addr == 1 &&
            (int16_t)g_regs[1].value == -200, "registers 0 = 250, 1 = -200 (signed)");
  check(d.counters.requests == 1 && d.counters.responses == 1 && d.counters.junk_bytes == 0, "counters req=1 rsp=1 junk=0");

  // 2) request and reply in SEPARATE bursts
  start(d, "2. FC03 request and reply in separate bursts");
  feed(d, REQ_03, 2000);
  feed(d, RSP_03, 2045);
  check(g_frames.size() == 2 && g_frames[1].kind == FK_RESPONSE && g_frames[1].paired, "reply paired across bursts");
  check(g_regs.size() == 2 && g_regs[0].value == 250, "register values decoded");

  // 3) CRC-corrupted frame
  start(d, "3. CRC-corrupted reply (last byte flipped)");
  Bytes bad = RSP_03;
  bad.back() ^= 0xFF;
  feed(d, REQ_03, 3000);
  feed(d, bad, 3040);
  check(g_frames.size() == 2 && g_frames[1].kind == FK_RAW && !g_frames[1].crc_ok, "corrupted reply shown as RAW, crc BAD");
  check(g_frames.size() == 2 && g_frames[1].len == bad.size(), "raw record keeps all 9 bytes for the hex view");
  check(g_regs.empty(), "no register values taken from a bad frame");
  check(d.counters.crc_errors == 1 && d.counters.junk_bytes == bad.size(), "crc_errors=1, junk_bytes=9");

  // 4) exception reply
  start(d, "4. Exception reply (01 83 02 = illegal data address)");
  feed(d, cat(REQ_03B, EXC_83), 4000);
  check(g_frames.size() == 2 && g_frames[1].kind == FK_EXCEPTION && g_frames[1].paired, "EXC paired with its request");
  check(d.counters.exceptions == 1 && g_regs.empty(), "exceptions=1, no register values");

  // 5) orphan reply (we never saw the request)
  start(d, "5. Orphan reply (slave 2 FC04 reply, no request seen)");
  feed(d, ORPH_04, 5000);
  check(g_frames.size() == 1 && g_frames[0].kind == FK_ORPHAN && g_frames[0].crc_ok, "shown as ORPH with crc OK");
  check(d.counters.orphans == 1 && g_regs.empty(), "orphans=1, not put in register table (start address unknown)");

  // extra coverage
  start(d, "6. Reply after the response timeout becomes an orphan");
  feed(d, REQ_03, 6000);
  feed(d, RSP_03, 7500);
  check(g_frames.size() == 2 && g_frames[1].kind == FK_ORPHAN, "late reply (1.5 s) is ORPH");

  start(d, "7. FC06 write + echo, FC10 write + reply");
  feed(d, REQ_06, 8000);
  feed(d, REQ_06, 8030);
  feed(d, cat(REQ_10, RSP_10), 8100);
  check(g_frames.size() == 4 && g_frames[1].kind == FK_RESPONSE && g_frames[3].kind == FK_RESPONSE,
        "echo of FC06 and FC10 reply both paired as RSP");
  check(g_regs.size() == 3 && g_regs[0].value == 0x1234 && g_regs[1].addr == 0x20 && g_regs[1].value == 10 &&
            g_regs[2].value == 11, "written values 0x1234, 10, 11 recorded");

  start(d, "8. FC01 coil read (bits)");
  feed(d, cat(REQ_01, RSP_01), 9000);
  check(g_regs.size() == 10 && g_regs[0].value == 1 && g_regs[1].value == 0 && g_regs[2].value == 1 &&
            g_regs[9].value == 1, "coils 0..9 = 1,0,1,0,0,0,0,0,0,1");

  start(d, "9. Non-Modbus bytes (ASCII text) are kept as RAW");
  const char* txt = "HELLO\r\n";
  feed(d, Bytes(txt, txt + strlen(txt)), 10000);
  check(g_frames.size() == 1 && g_frames[0].kind == FK_RAW && g_frames[0].len == 7, "7 raw bytes in one RAW record");
  check(d.counters.crc_errors == 0, "not counted as a CRC error (does not look like Modbus)");

  start(d, "10. Noise before a valid frame in the same burst");
  feed(d, cat(Bytes{0x00, 0xFF, 0x55}, REQ_03), 11000);
  check(g_frames.size() == 2 && g_frames[0].kind == FK_RAW && g_frames[0].len == 3 && g_frames[1].kind == FK_REQUEST,
        "3 junk bytes then REQ recovered");

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
