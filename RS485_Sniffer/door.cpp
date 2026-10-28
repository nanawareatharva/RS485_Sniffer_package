// Door sensor: polls DOOR_PIN every 5 ms, debounces, counts open/close
// transitions, stores the counts in NVS (rate-limited) and drives the
// door-open alarm buzzer.
#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include "config.h"
#include "buzzer.h"
#include "door.h"

#define DOOR_ALARM_MS ((uint32_t)DOOR_ALARM_MINUTES * 60u * 1000u)

static DoorStatus        s_door;
static portMUX_TYPE      s_mux = portMUX_INITIALIZER_UNLOCKED;
static std::atomic<bool> s_reset_req{false};
static std::atomic<bool> s_silence_req{false};

static bool pin_open() { return digitalRead(DOOR_PIN) == DOOR_OPEN_LEVEL; }

static void save_counts(uint32_t opens, uint32_t closes) {
  Preferences p;
  p.begin("door", false);
  p.putUInt("opens", opens);
  p.putUInt("closes", closes);
  p.end();
  portENTER_CRITICAL(&s_mux);
  s_door.saves++;
  portEXIT_CRITICAL(&s_mux);
}

static void door_task(void*) {
  bool stable = pin_open();
  bool candidate = stable;
  uint32_t candidate_since = millis();
  uint32_t since = s_door.since_ms;
  bool silenced = false, alarm = false;
  bool dirty = false;                     // counts changed but not yet in flash
  uint32_t last_save = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5));
    const uint32_t now = millis();

    if (s_reset_req.exchange(false)) {
      portENTER_CRITICAL(&s_mux);
      s_door.opens = s_door.closes = 0;
      portEXIT_CRITICAL(&s_mux);
      save_counts(0, 0);
      last_save = now;
      dirty = false;
      Serial.println("DOOR counts reset");
    }
    if (s_silence_req.exchange(false) && alarm) {
      silenced = true;
      Serial.println("DOOR alarm silenced");
    }

    // ---- debounce
    const bool raw = pin_open();
    if (raw != candidate) {                // level changed: restart the debounce timer
      candidate = raw;
      candidate_since = now;
    } else if (candidate != stable && now - candidate_since >= DOOR_DEBOUNCE_MS) {
      stable = candidate;                  // stable long enough: accept the change
      since = now;
      if (!stable) silenced = false;       // door closed: next alarm buzzes again
      uint32_t opens, closes;
      portENTER_CRITICAL(&s_mux);
      s_door.open = stable;
      if (stable) s_door.opens++;
      else s_door.closes++;
      s_door.last_change_ms = now;
      s_door.since_ms = now;
      opens = s_door.opens;
      closes = s_door.closes;
      portEXIT_CRITICAL(&s_mux);
      dirty = true;
      Serial.printf("[%lu] DOOR %s  (opened %lu, closed %lu)\r\n", (unsigned long)now,
                    stable ? "OPEN" : "CLOSED", (unsigned long)opens, (unsigned long)closes);
    }

    // ---- save to flash, at most every DOOR_SAVE_MIN_INTERVAL_MS
    if (dirty && (last_save == 0 || now - last_save >= DOOR_SAVE_MIN_INTERVAL_MS)) {
      uint32_t opens, closes;
      portENTER_CRITICAL(&s_mux);
      opens = s_door.opens;
      closes = s_door.closes;
      portEXIT_CRITICAL(&s_mux);
      save_counts(opens, closes);
      last_save = now;
      dirty = false;
    }

    // ---- door-open alarm
    const bool new_alarm = stable && now - since >= DOOR_ALARM_MS;
    if (new_alarm != alarm) {
      alarm = new_alarm;
      if (alarm) Serial.printf("DOOR ALARM: open for %d minutes\r\n", DOOR_ALARM_MINUTES);
    }
    buzzer_set(alarm && !silenced && (now / BUZZER_BEEP_MS) % 2 == 0);

    portENTER_CRITICAL(&s_mux);
    s_door.alarm = alarm;
    s_door.silenced = silenced;
    portEXIT_CRITICAL(&s_mux);
  }
}

void door_begin() {
  pinMode(DOOR_PIN, DOOR_PIN_MODE);
  delay(5);

  Preferences p;
  p.begin("door", false);
  s_door.opens = p.getUInt("opens", 0);
  s_door.closes = p.getUInt("closes", 0);
  p.end();
  s_door.open = pin_open();
  s_door.last_change_ms = 0;
  s_door.since_ms = millis();

  xTaskCreatePinnedToCore(door_task, "door", 4096, nullptr, 2, nullptr, 0);
  Serial.printf("Door sensor on GPIO%d: %s (opened %lu, closed %lu so far), alarm after %d min open\r\n",
                DOOR_PIN, s_door.open ? "OPEN" : "CLOSED", (unsigned long)s_door.opens,
                (unsigned long)s_door.closes, DOOR_ALARM_MINUTES);
}

void door_get(DoorStatus* out) {
  portENTER_CRITICAL(&s_mux);
  *out = s_door;
  portEXIT_CRITICAL(&s_mux);
}

void door_reset_counts() { s_reset_req.store(true); }
void door_silence_alarm() { s_silence_req.store(true); }
