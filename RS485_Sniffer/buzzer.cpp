// PCF8574T buzzer output. Uses Waveshare's existing I2C master bus (i2c.cpp)
// instead of creating a second driver on the same pins.
#include <Arduino.h>
#include <atomic>
#include "i2c.h"
#include "config.h"
#include "buzzer.h"

extern DEV_I2C_Port handle;          // Waveshare's I2C bus, created in DEV_I2C_Init() by touch_gt911_init()

static i2c_master_dev_handle_t s_dev;
static std::atomic<bool>       s_ready{false};
static bool                    s_on;   // last state written (only touched by the caller of buzzer_set)

static bool write_pin(bool on) {
  const bool level_high = BUZZER_ACTIVE_LOW ? !on : on;
  uint8_t v = 0xFF;                    // every other pin stays HIGH (PCF8574 "off"/input state)
  if (!level_high) v &= (uint8_t)~(1u << BUZZER_PCF_PIN);
  return i2c_master_transmit(s_dev, &v, 1, 50) == ESP_OK;
}

bool buzzer_begin() {
  if (!handle.bus) {
    Serial.println("Buzzer: I2C bus not started");
    return false;
  }
  uint8_t addr = BUZZER_PCF_ADDR;
  if (addr == 0) {
    static const uint8_t candidates[] = {0x27, 0x20, 0x21, 0x22, 0x23, 0x25, 0x26};   // 0x24 = board IO expander
    for (uint8_t a : candidates) {
      if (i2c_master_probe(handle.bus, a, 50) == ESP_OK) { addr = a; break; }
    }
  } else if (i2c_master_probe(handle.bus, addr, 50) != ESP_OK) {
    addr = 0;
  }
  if (addr == 0) {
    Serial.println("Buzzer: no PCF8574 found on I2C - door alarm will show on screen only");
    return false;
  }

  i2c_device_config_t cfg = {};
  cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  cfg.device_address = addr;
  cfg.scl_speed_hz = 100000;
  if (i2c_master_bus_add_device(handle.bus, &cfg, &s_dev) != ESP_OK) {
    Serial.println("Buzzer: could not add PCF8574 to the I2C bus");
    return false;
  }
  write_pin(false);
  s_on = false;
  s_ready.store(true);
  Serial.printf("Buzzer: PCF8574 at 0x%02X, buzzer on P%d (%s)\r\n", addr, BUZZER_PCF_PIN,
                BUZZER_ACTIVE_LOW ? "active LOW" : "active HIGH");
  return true;
}

void buzzer_set(bool on) {
  if (!s_ready.load() || on == s_on) return;
  if (write_pin(on)) s_on = on;
}

bool buzzer_present() { return s_ready.load(); }
