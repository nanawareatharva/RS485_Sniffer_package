/*
 * RS485 / Modbus RTU passive sniffer with LVGL touch UI
 * Waveshare ESP32-S3-Touch-LCD-7B (1024x600), based on Waveshare's 13_lvgl_v8_demo.
 *
 * LISTEN ONLY: the RS485 UART is never written to (see rs485_capture.cpp).
 * Files: rs485_capture.* (UART + storage), modbus_decoder.* (frame parser),
 *        door.* (door sensor on GPIO6 + open-too-long alarm),
 *        buzzer.* (PCF8574T buzzer on I2C), ui.* (LVGL HOME screen),
 *        config.h (bus defaults, sensor register map, door pin).
 */
#include <esp_log.h>

#include "esp_lv_adapter_arduino.h"
#include "config.h"
#include "rs485_capture.h"
#include "door.h"
#include "buzzer.h"
#include "ui.h"

void setup() {
    // Hold RS485 TX (GPIO16) high from the very first moment, before the UART owns it.
    pinMode(RS485_TX_PIN, OUTPUT);
    digitalWrite(RS485_TX_PIN, HIGH);

    Serial.setTxBufferSize(8192);   // USB serial monitor: frame mirror must not stall the parser
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== " DASHBOARD_TITLE " - ESP32-S3-Touch-LCD-7B ===");

    // Start listening first, so no frames are missed while the display comes up.
    rs485_begin();
    door_begin();

    static esp_lcd_panel_handle_t panel_handle = NULL; // Declare a handle for the LCD panel
    static esp_lcd_touch_handle_t tp_handle = NULL;    // Declare a handle for the touch panel

    // Initialize the GT911 touch screen controller (also brings up I2C + IO expander)
    tp_handle = touch_gt911_init();

    // Door-alarm buzzer (PCF8574T) shares the I2C bus that touch_gt911_init() just created
    buzzer_begin();

    // Initialize the Waveshare ESP32-S3 RGB LCD hardware
    panel_handle = waveshare_esp32_s3_rgb_lcd_init();

    // Turn on the LCD backlight
    wavesahre_rgb_lcd_bl_on();

    // Initialize LVGL with the panel and touch handles
    ESP_ERROR_CHECK(esp_lv_adapter_init(panel_handle, tp_handle));

    // Lock the mutex because LVGL APIs are not thread-safe
    if (esp_lv_adapter_lock(-1)) {
        ui_create();
        esp_lv_adapter_unlock();
    }
}

void loop() {
    // Everything runs in the RS485 parser task and the LVGL task.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
