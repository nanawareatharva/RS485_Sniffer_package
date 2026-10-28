#pragma once
// Door sensor on a GPIO (3.3 V logic). Debounced in its own task; open/close
// counts are saved in NVS so they survive reboots and power cuts.
// Also runs the door-open alarm (buzzer via buzzer.h).
// Thread-safe snapshot API: never touches LVGL.
#include <stdint.h>

struct DoorStatus {
  bool     open;
  uint32_t opens;            // times the door went closed -> open
  uint32_t closes;           // times the door went open -> closed
  uint32_t last_change_ms;   // millis() of the last change, 0 = none since boot
  uint32_t since_ms;         // millis() the current state started (boot time if unchanged)
  bool     alarm;            // open for DOOR_ALARM_MINUTES or longer
  bool     silenced;         // alarm acknowledged with Silence (until the door closes)
  uint32_t saves;            // flash writes so far (the UI redraws after each one)
};

void door_begin();
void door_get(DoorStatus* out);
void door_reset_counts();    // async: done by the door task, also saved
void door_silence_alarm();   // stop the buzzer until the door closes
