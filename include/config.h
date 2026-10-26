#pragma once
#include <stdint.h>

#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG 0
#endif

#if ENABLE_DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do { } while (0)
#define DEBUG_PRINTLN(...) do { } while (0)
#define DEBUG_PRINTF(...) do { } while (0)
#endif

namespace Horizon {
constexpr char kFirmwareVersion[] = "4.1.0-lite";
constexpr char kBluetoothName[] = "HORIZON AUDIO DLMS LITE";
// Nine-band variant: preserve the existing eight centers and add sub-bass at 25 Hz.
constexpr uint8_t kBandCount = 9;
// PCM5102A I2S output stays fixed at 48 kHz; Bluetooth source PCM is resampled.
constexpr uint32_t kDefaultSampleRate = 48000;
constexpr uint8_t kAudioBitDepth = 16;
constexpr uint32_t kAudioTaskStack = 8192;
constexpr uint32_t kUiTaskStack = 4096;
constexpr uint32_t kInputTaskStack = 3072;
constexpr uint32_t kUiFramePeriodMs = 67;  // ~15 FPS: reduce TFT redraw load during Bluetooth audio.
constexpr uint32_t kUiIdleDelayMs = 20;  // Lower idle polling overhead; input task remains independent.
constexpr uint32_t kInputPollIntervalMs = 10;
constexpr uint16_t kVisualizerPeakHoldMs = 200;
constexpr uint8_t kVisualizerPeakDecayPixelsPerFrame = 2;
constexpr uint8_t kVisualizerAttackShift = 2;   // 25% of the gap per frame.
constexpr uint8_t kVisualizerReleaseShift = 3;  // 12.5% of the gap per frame.
constexpr int16_t kVisualizerDbFloor = -60;  // Meter maps -60..0 dBFS to 0..100%.
constexpr uint16_t kVisualizerClipThreshold = 32767;
constexpr uint8_t kVisualizerDbMinus12Percent = 80;  // -12 dBFS on the -60..0 scale
constexpr uint8_t kVisualizerDbMinus3Percent = 95;   // -3 dBFS on the -60..0 scale
constexpr uint16_t kButtonDebounceMs = 20;
constexpr uint16_t kCenterLongPressMs = 1000;
constexpr uint16_t kUiVolumeOverlayTimeoutMs = 2000;
constexpr int16_t kUiVolumeOverlayX = 14;
constexpr int16_t kUiVolumeOverlayY = 30;
constexpr int16_t kUiVolumeOverlayWidth = 100;
constexpr int16_t kUiVolumeOverlayHeight = 50;

// Modern 128 x 128 RGB565 theme.
constexpr uint16_t kColorBg = 0x0842;           // charcoal
constexpr uint16_t kColorCardBg = 0x18A3;       // slate navy
constexpr uint16_t kColorHeaderBg = kColorBg;
constexpr uint16_t kColorPanel = kColorCardBg;  // compatibility alias
constexpr uint16_t kColorAccent = 0x07FF;       // neon cyan
constexpr uint16_t kColorCyan = kColorAccent;   // compatibility alias
constexpr uint16_t kColorGreen = 0x07E0;
constexpr uint16_t kColorAmber = 0xFFE0;
constexpr uint16_t kColorHighlight = kColorAmber;
constexpr uint16_t kColorText = 0xFFFF;
constexpr uint16_t kColorMuted = 0x8410;
constexpr uint16_t kColorRed = 0xF800;
constexpr uint16_t kColorBorder = 0x4208;
constexpr uint16_t kColorGlow = 0x2D2B;
constexpr uint16_t kColorSelectedBg = 0x1027;
constexpr uint16_t kColorMeterTrack = 0x39C7;
constexpr uint16_t kColorMeterGreen = 0x4F34;  // soft green
constexpr uint16_t kColorMeterCyan = 0x5DFF;   // light cyan
constexpr uint16_t kColorMeterAmber = 0xFDE0;
constexpr uint16_t kColorMeterRed = 0xF800;

// TFT ST7735 128 x 128: configured through TFT_eSPI build flags.
constexpr int8_t kPinTftMosi = 23;
constexpr int8_t kPinTftSclk = 18;
constexpr int8_t kPinTftCs = 5;
constexpr int8_t kPinTftDc = 2;
constexpr int8_t kPinTftReset = 4;

// PCM5102A uses the second ESP32 I2S controller so the A2DP library's
// internal I2S output remains disabled while PCM is processed by our task.
constexpr int8_t kPinI2sBck = 26;
constexpr int8_t kPinI2sLrck = 25;
constexpr int8_t kPinI2sData = 22;

constexpr int8_t kPinUp = 13;
constexpr int8_t kPinDown = 14;
constexpr int8_t kPinLeft = 27;
constexpr int8_t kPinRight = 32;
constexpr int8_t kPinCenter = 33;
constexpr int8_t kPinVolumeUp = 21;
constexpr int8_t kPinVolumeDown = 16;

constexpr float kBandFrequencies[kBandCount] = {
    25.0f, 45.0f, 60.0f, 150.0f, 400.0f, 1000.0f, 2500.0f, 6300.0f, 16000.0f};
// Reference-style peaking EQ: Q=0.9 for the two lowest bands, Q=1.414 elsewhere.
// Adjacent biquad responses still overlap by design; Q cannot isolate bands perfectly.
constexpr float kBandQ[kBandCount] = {
    0.90f, 0.90f, 1.414f, 1.414f, 1.414f, 1.414f, 1.414f, 1.414f, 1.414f};
constexpr const char* kBandShortNames[kBandCount] = {
    "25", "45", "60", "150", "400", "1k", "2k5", "6k3", "16k"};
constexpr const char* kPresetNames[5] = {
    "FLAT", "EXTREME BASS", "VOCAL", "CUSTOM 1", "CUSTOM 2"};
}  // namespace Horizon

enum class InputEvent : uint8_t {
  Up, Down, Left, Right, Center, CenterLong, VolumeUp, VolumeDown
};
