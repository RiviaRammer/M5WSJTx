# Tab5FTx on M5Stack Tab5

This sketch ports the Cardputer FT8 app to M5Stack Tab5.

# Installation

- **Board:** ESP32-P4 / `esp32-p4-evboard`
- **Libraries:** M5Unified, M5GFX

# Setting

- **PSRAM:** enabled
- **USB CDC on boot:** enabled
- **Upload speed:** 1500000
- **Monitor speed:** 115200

The official Tab5 PlatformIO target uses `pioarduino/platform-espressif32`, `board = esp32-p4-evboard`, `board_build.mcu = esp32p4`, `BOARD_HAS_PSRAM`, `ARDUINO_USB_CDC_ON_BOOT=1`, and `ARDUINO_USB_MODE=1`.

# Mode

- FT8
- FT4 selectable from the mode label; the original timing logic is still FT8-oriented, so treat FT4 as an experimental path for now.

# Usage

- **Serial `esc` / `cancel` / `stop`:** Abort queued/current TX
- **Home:** Show the live dashboard
- **Home:** First row shows mode/status, second row shows work logic and `Work Freq`, third row shows UTC and the slot progress bar.
- **Home work logic row:** Tap the left status text to cycle `CQ at ...` / `Answer at ...` / `Only RX, No TX`; tap the frequency button to type a work frequency, or tap `Any Freq` when in Answer mode. CQ always uses a fixed work frequency.
- **Boot default:** Starts in `Only RX, No TX` to avoid accidental transmission.
- **TX:** Show work frequency, TX slot, generated messages, and a manual `TX` button that queues the selected message for the next slot even when the Home logic is `Only RX, No TX`.
- **TX Work Freq field:** Tap the field, type the new value on the Tab5 keyboard or serial monitor, and press Enter.
- **WF:** Temporarily parked while RX performance is being restored; it does not run during RX capture.
- **RX decode:** Uses the Cardputer monitor/decode flow, with Tab5/P4-specific capacity increases and the Tab5 mic channel set for right-channel mono input.
- **Setting:** Edit callsign, grid, and volume percentage. WiFi Config and Sync Time buttons are placeholders for now.
- **Setting fields:** Tap `Callsign` or `Grid`, type the new value, and press Enter.
- **Setting +/- buttons:** Adjust volume percentage.
- **Tab5 keyboard:** If `M5UnitUnified` and `M5UnitUnifiedKEYBOARD` are installed, the sketch reads typed values only after a field is selected.
- **Tap mode label:** Toggle FT8/FT4 display option
- **Tap RX line on Home:** Select that received row for the right-side QSO view. It does not change Work Freq.
- **Tap top-right corner:** Abort queued/current TX
- USB serial commands are disabled; serial output is only used for diagnostics.

# Hardware Notes

- Tab5 uses ESP32-P4 and a 5-inch 1280x720 MIPI-DSI IPS touch display.
- Tab5 includes an RX8130CE RTC. Boot loads system UTC from RTC first; if WiFi/NTP sync later succeeds, the sketch writes the updated UTC back to RTC.
- The display is initialized with `setRotation(3)`, so it is landscape and rotated 180 degrees from rotation `1`.
- Wi-Fi uses the ESP32-C6 SDIO2 link; the sketch calls `WiFi.setPins(12, 13, 11, 10, 9, 8, 15)` before connecting.
- TX audio is routed through `M5.Speaker`.
- RX audio is routed through `M5.Mic`.
- Text commands remain available over the USB serial monitor at 115200 baud.
