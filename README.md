# HORIZON AUDIO DLMS Lite

A compact public build for the classic ESP32 DevKit V4, PCM5102A I2S DAC, and ST7735 128×128 TFT.

## Kept features

- Bluetooth Classic A2DP sink for receiving music from a phone.
- Dedicated audio task and I2S output through PCM5102A.
- Nine-band EQ, factory presets, and custom EQ editing from the device UI.
- Physical button controls, volume adjustment, TFT player and system screens.
- Saved EQ/volume settings and A2DP reconnection behavior.

## Removed or reduced

- Bluetooth SPP phone/serial remote-control profile and its command parser.
- TFT update cadence reduced to about 15 FPS to reduce display workload while audio is active.
- Unused ArduinoJson dependency.

Audio processing, EQ, and the A2DP path are retained from the stable project baseline. Lite means fewer optional features and lower display update load; it does not guarantee zero audio gaps on every board, DAC, power supply, or Bluetooth source.

## Build

This is a PlatformIO project. Open this folder in PlatformIO and build the `esp32dev` environment. The `platformio.ini` pins the ESP32 platform and library versions used by the baseline. Hardware playback still needs to be checked on the target board before release.

## Hardware

The pin map is configured in `include/config.h` and `platformio.ini`. This profile targets a classic ESP32, not ESP32-S3.
