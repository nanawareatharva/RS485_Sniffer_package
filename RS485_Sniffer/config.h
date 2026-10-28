// =====================================================================
//  config.h  —  EDIT THIS FILE to match your bus and sensor
//  Waveshare ESP32-S3-Touch-LCD-7B  |  passive RS485 / Modbus RTU sniffer
// =====================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---------- RS485 pins (fixed on the 7B board, do not change) ----------
// Onboard transceiver switches direction automatically: it only drives the
// bus while the UART TX line is active. This code NEVER writes to the UART,
// so the LCD stays a silent listener. Keep the TX pin assigned so it idles
// HIGH (an unassigned/floating TX pin could make the transceiver transmit).
#define RS485_RX_PIN   15
#define RS485_TX_PIN   16

// ---------- Default bus settings ----------
// Used only until you press Apply (or run Auto-baud) on the SETTINGS tab;
// after that the choice is saved in NVS and restored at every boot.
#define MB_DEFAULT_BAUD    9600
#define MB_DEFAULT_PARITY  0          // 0 = 8N1, 1 = 8E1, 2 = 8O1, 3 = 8N2

// Silence (in character times) that marks the end of a Modbus frame.
// Modbus RTU spec = 3.5 chars -> 4 is right for most buses.
#define MB_RX_TIMEOUT_SYMBOLS  4

// Max time between the gateway's request and the sensor's reply (ms).
#define MB_RESP_TIMEOUT_MS     1000

// A value is shown greyed-out as "stale" if not refreshed for this long (ms).
#define VALUE_STALE_MS         10000

// STATS status: GREEN if a valid frame arrived within this window (ms).
#define STATUS_WINDOW_MS       5000

// Auto-baud: how long to listen on each baud/parity combination (ms).
#define AUTOBAUD_LISTEN_MS     3000

// Mirror every frame as one line on the USB serial monitor (115200). 1 = on.
#define SNIFFER_DEBUG          1


// ---------- Temperature / humidity sensors on the RS485 bus ----------
// Map worked out from sniffed traffic (2026-10-12), not from a datasheet:
//   slaves 1 & 2: FC03, reg 0 = 2912 -> 29.12, reg 1 = 4766 -> 47.66  (x0.01)
//   slave 3:      FC04, reg 1 = 294  -> 29.4,  reg 2 = 512  -> 51.2   (x0.1, XY-MD02/SHT20 layout)
// If a breath test shows temperature and humidity swapped for a sensor,
// swap its temp_reg / hum_reg numbers.
// Slave 4 (FC04, 2-register reads at 20-22) has not replied yet, so it is not listed.
// Values only update when the gateway polls these registers (this device only listens).
struct THSensor {
  const char* name;
  uint8_t     slave;      // Modbus slave ID
  uint8_t     fc;         // 3 = holding registers, 4 = input registers
  uint16_t    temp_reg;   // register address as sent on the wire (signed value)
  uint16_t    hum_reg;    // register address as sent on the wire (unsigned value)
  float       scale;      // shown value = raw * scale
  uint8_t     decimals;
};

static const THSensor TH_SENSORS[] = {
  // name        slave fc  temp  hum   scale  decimals
  { "Sensor 1",    1,  3, 0x00, 0x01, 0.01f, 2 },
  { "Sensor 2",    2,  3, 0x00, 0x01, 0.01f, 2 },
  { "Sensor 3",    3,  4, 0x01, 0x02, 0.1f,  1 },
};
static const size_t NUM_TH_SENSORS = sizeof(TH_SENSORS) / sizeof(TH_SENSORS[0]);

// ---------- Door sensor (digital input) ----------
// 3.3 V logic output from the sensor into GPIO6 (the free pin on the board header).
// NEVER connect more than 3.3 V to this pin.
#define DOOR_PIN          6
#define DOOR_OPEN_LEVEL   HIGH            // pin level that means "door open"; use LOW if it shows backwards
#define DOOR_PIN_MODE     INPUT_PULLDOWN  // keeps the pin from floating if the wire comes off (reads "closed")
#define DOOR_DEBOUNCE_MS  100             // level must be stable this long before it counts (filters EMI spikes)
#define DOOR_SAVE_MIN_INTERVAL_MS 10000   // counts are written to flash at most this often (flash writes disturb the display)

// ---------- Door-open alarm: buzzer on a PCF8574T I2C module ----------
// Module on the board's I2C connector (SDA/SCL/3V3/GND, shared with touch + IO expander).
// Address 0x24 is taken by the board's own IO expander - do not set the module's A0-A2 jumpers to that.
#define DOOR_ALARM_MINUTES   30     // buzz when the door has been open this long
#define BUZZER_PCF_ADDR      0      // 0 = auto-detect (tries 0x27, 0x20-0x23, 0x25, 0x26)
#define BUZZER_PCF_PIN       0      // module pin the buzzer is on: P0..P7
#define BUZZER_ACTIVE_LOW    1      // 1 = pin pulled LOW sounds the buzzer (PCF8574 can only sink current)
#define BUZZER_BEEP_MS       500    // beep on/off rhythm

// ---------- Display ----------
#define SCREEN_REFRESH_MS    3000   // full redraw this often: clears any ghost image left by noise or flash writes

#define DASHBOARD_TITLE  "Room Monitor"