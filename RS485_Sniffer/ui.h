#pragma once
// LVGL 8.4 touch UI for the RS485 sniffer (1024x600, dark theme, 4 tabs).
// Call ONCE from inside esp_lv_adapter_lock()/unlock(), after rs485_begin().
// All later updates run from an lv_timer (inside the LVGL task) and read
// data only through the thread-safe rs485_* snapshot functions.
void ui_create();
