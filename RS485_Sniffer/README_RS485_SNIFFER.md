# Room Monitor (1ACCORD / COOL CROP): Waveshare ESP32-S3-Touch-LCD-7B

The board does three jobs:

1. **Listens** on the RS485 line between the gateway (Modbus master) and the
   temperature/humidity sensors, and shows the averages and each sensor's values.
   It **never transmits**: nothing in the code writes to the RS485 UART, and TX (GPIO16) idles HIGH.
2. Reads a **door sensor** on **GPIO6** and counts how often the door was opened and closed.
   The counts are kept after a power cut.
3. **Buzzes** through a **PCF8574T** I2C module when the door has been open for **30 minutes**.

## Wiring

### RS485

| Board RS-485 terminal | Connect to |
|---|---|
| **A** | line **A** (D+ / 485+) |
| **B** | line **B** (D- / 485-) |
| **GND** | line **GND** / signal common |

- **Termination DIP switch: OFF**, unless this board is physically at one *end* of the cable.
- The header says *"Sensor connection problem"*? Swap A and B.

### Door sensor (24 V signal → buck converter → 3.3 V)

| From | To |
|---|---|
| buck converter OUT+ (3.3 V) | **GPIO6** |
| buck converter OUT- | **GND** of the board |
| **1 kΩ resistor** | across the buck output (OUT+ to OUT-) |

- **Measure the buck output first: it must be 3.3 V or less.** More than about 3.6 V can damage the ESP32.
- The 1 kΩ resistor drains the buck's output capacitors. Without it, "door closed" can show seconds late.
- GPIO6 has the internal pull-down on, so a loose wire reads **CLOSED**.
- Default: **3.3 V present = door OPEN**. If the screen shows it backwards, set `DOOR_OPEN_LEVEL` to `LOW` in `config.h`.
- A change must be stable for 100 ms to count, which filters electrical spikes.
- Better long-term: replace the buck converter with a **24 V optocoupler module** (e.g. PC817). It switches instantly and isolates the 24 V side.

### Buzzer (PCF8574T module)

| PCF8574T module | Board I2C connector |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | SDA (GPIO8) |
| SCL | SCL (GPIO9) |

- The buzzer goes on module pin **P0**. The PCF8574 can only *sink* current, so wire it as
  **buzzer + → VCC, buzzer − → P0** (active LOW, the default).
  For a buzzer drawing more than about 20 mA, use a buzzer module with its own transistor (a "low-level trigger" type).
- The address is **found automatically** (0x20–0x27). **Do not set the module's A0–A2 jumpers to 0x24**, which the board itself uses.
- Settings in `config.h`: `DOOR_ALARM_MINUTES` (30), `BUZZER_PCF_PIN` (0), `BUZZER_ACTIVE_LOW` (1), `BUZZER_PCF_ADDR` (0 = auto).
- The USB serial monitor shows at start-up whether the module was found:
  `Buzzer: PCF8574 at 0x27, buzzer on P0` or `Buzzer: no PCF8574 found`.

### USB
Use the Type-C port marked **UART**, with the UART selector on **UART1**.

## The screen

- **Start-up:** after power-on or RESET, a splash screen shows 1ACCORD / COOL CROP, a spinner and a loading bar for about 2.5 s.
- **Header:** *Sensors online* (green), *Waiting for sensors* (grey), or *Sensor connection problem* (amber).
- **AVERAGE TEMPERATURE / AVERAGE HUMIDITY:** the average of all sensors updated in the last 10 s, with low and high.
  Tap a tile to see each sensor's own value: temperature only, or humidity only. **Back** returns to HOME.
- **DOOR:**
  - **OPEN** (amber) or **CLOSED** (green), and how long it has been that way.
  - **Opened** / **Closed** counts.
  - **Reset counts** asks for confirmation, then shows a *Resetting…* spinner and a *✓ Counts reset* tick.
  - After 30 min open: **OPEN TOO LONG** in red with a blinking dot, the buzzer beeps (0.5 s on / 0.5 s off), and a red **Silence** button appears.
    Silence stops the buzzer until the door closes. The next long opening alarms again.
- **Bottom edge:** 1ACCORD (left), COOL CROP (right).

## Ghosting / flicker fixes

The display keeps two copies of the picture and only copies the changed parts between them. If one copy is lost,
the screen flickers between an old and a new picture until a reset. Copies get lost through flash writes or electrical noise. The firmware now:

- **Redraws the whole screen every 3 s** (`SCREEN_REFRESH_MS`) and right after every flash write. Any ghost image clears within 3 s by itself.
- **Writes the door counts to flash at most every 10 s** (`DOOR_SAVE_MIN_INTERVAL_MS`). Before, every door change, or every noise spike on the door wire, caused a flash write.
  If power is cut within 10 s of a door change, that last change may not be counted.
- **Debounces the door input for 100 ms**, so electrical noise can't create bursts of fake door events.

**Hardware side**, which software can't fully fix:
- Keep the 24 V wiring and the buck converter **away from the display ribbon cable** and the board.
- Use **twisted pair or shielded cable** for the door signal, and connect **GND** properly to the board.
- A **100 nF capacitor** from GPIO6 to GND, at the board, filters spikes on the door line.
- Power the board from a clean 5 V supply, not one shared with relays or motors.

## Sensor register map (`config.h` → `TH_SENSORS`)

| Sensor | Slave | FC | Temperature reg | Humidity reg | Scale |
|---|---|---|---|---|---|
| Sensor 1 | 1 | 03 | 0 | 1 | ×0.01 |
| Sensor 2 | 2 | 03 | 0 | 1 | ×0.01 |
| Sensor 3 | 3 | 04 | 1 | 2 | ×0.1 |

Worked out from the bus traffic. Check it by breathing on a sensor: humidity jumps, temperature barely moves.
If a sensor's values look swapped, swap its two register numbers.

## Uploading (someone else's PC)

See **HOW_TO_UPLOAD.txt** in the package. In short:

- **Easiest (no Arduino needed):** run `firmware\FLASH.bat COMxx`. It uses the included `esptool.exe` and the ready-made `RS485_Sniffer.merged.bin`.
- **From source in Arduino IDE 2.x:**
  1. Install board package *esp32 by Espressif* **3.1.3**.
  2. Unpack `libraries\LVGL_v8.4.0.7z` into `Documents\Arduino\libraries`.
  3. Open `RS485_Sniffer\RS485_Sniffer.ino`.
  4. Select **ESP32S3 Dev Module** with Flash 16MB, PSRAM OPI, Flash mode QIO, Partition "16M Flash (3MB APP/9.9MB FATFS)".
  5. Upload.

If an upload sticks at `Connecting...`: hold **BOOT**, tap **RESET**, release **BOOT**.

## Files

| File | Purpose |
|---|---|
| `RS485_Sniffer.ino` | start-up order: RS485 → door → touch/I2C → buzzer → display → UI |
| `config.h` | **all settings**: sensors, door pin/level, alarm time, buzzer pin, refresh interval |
| `ui.cpp/.h` | screen (HOME, per-sensor lists, door tile, splash, animations, anti-ghosting redraw) |
| `door.cpp/.h` | door input, counts (saved), 30-min alarm logic |
| `buzzer.cpp/.h` | PCF8574T buzzer on the board's I2C bus |
| `rs485_capture.cpp/.h`, `modbus_decoder.cpp/.h` | listen-only Modbus RTU capture and decoding |
| `host_test\` | PC test for the decoder (`run_test.bat`, needs Visual Studio Build Tools) |
| `esp_lv_adapter_arduino.*`, `gt911.*`, `i2c.*`, `io_extension.*`, `rgb_lcd_port.*`, `touch.*` | Waveshare drivers, unmodified |
