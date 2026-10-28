#pragma once
// Buzzer driven through a PCF8574T I2C I/O expander on the board's I2C bus.
// Call buzzer_begin() once after the board's I2C bus exists (after touch_gt911_init()).
// If no module answers, buzzer_set() does nothing and the alarm is shown on screen only.

bool buzzer_begin();
void buzzer_set(bool on);
bool buzzer_present();
