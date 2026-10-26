#include "ui_engine.h"
#include "audio_engine.h"
#include "config.h"
#include <TFT_eSPI.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <esp_system.h>
#include <freertos/task.h>

namespace {
using namespace Horizon;
TFT_eSPI tft;
QueueHandle_t s_eventQueue = nullptr;

enum class UiPage : uint8_t { MainMenu, Dashboard, Preset, EqEditor, System };
UiPage s_page = UiPage::MainMenu;
uint8_t s_menuCursor = 0;
uint8_t s_selectedPreset = 0;
uint8_t s_selectedBand = 0;
bool s_viewDirty = true;
bool s_headerDrawn = false;
bool s_lastBluetoothConnected = false;
uint32_t s_lastDashboardRate = 0;
uint8_t s_lastHeaderPreset = 255;
float s_lastEqGains[Horizon::kBandCount] = {0};
bool s_lastEqGainsValid = false;
uint32_t s_lastSystemRefreshMs = 0;

bool s_volumeOverlayActive = false;
bool s_volumeOverlayNeedsDraw = false;
uint8_t s_volumeOverlayValue = 70;
uint32_t s_volumeOverlayLastActivityMs = 0;

constexpr int kMeterStartX = 22;
constexpr int kMeterTrackWidth = 96;
constexpr int kMeterBarHeight = 8;
constexpr int kMeterRadius = kMeterBarHeight / 2;
constexpr uint8_t kInvalidFillWidth = 0xFF;
constexpr int16_t kInvalidMarkerX = -2;

struct MeterState {
  int16_t smoothedLevelQ2 = 0;  // Percent in Q2 fixed point for shift-based EMA.
  int16_t heldPeakWidth = 0;
  uint32_t peakHoldMs = 0;
  uint32_t decayDeadlineMs = 0;
  uint8_t renderedFillWidth = kInvalidFillWidth;
  uint16_t renderedFillColor = kColorMeterTrack;
  int16_t renderedMarkerX = kInvalidMarkerX;
  bool signalInitialized = false;
  bool clipping = false;
  bool forceRedraw = true;
};

MeterState s_leftMeter;
MeterState s_rightMeter;
uint8_t s_lastDashboardVolume = 255;
bool s_lastDashboardStreamActive = false;
bool s_dashboardStreamStatusValid = false;
const char* presetBadge(uint8_t preset) {
  switch (preset) {
    case 1: return "BASS";
    case 2: return "VOCAL";
    case 3: return "CUSTOM 1";
    case 4: return "CUSTOM 2";
    default: return "FLAT";
  }
}

void drawBatteryIcon() {
  // Battery telemetry is not wired on this board yet, so show an unfilled icon.
  tft.drawRoundRect(109, 4, 13, 8, 2, kColorMuted);
  tft.fillRect(122, 6, 2, 4, kColorMuted);
}

void drawHeader(const AudioSnapshot& audio) {
  tft.fillRect(0, 0, 128, 16, kColorHeaderBg);
  const uint16_t btFill = audio.bluetoothConnected ? kColorGreen : kColorCardBg;
  const uint16_t btText = audio.bluetoothConnected ? kColorBg : kColorMuted;
  tft.fillRoundRect(3, 3, 23, 10, 4, btFill);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(btText, btFill);
  tft.drawCentreString("BT", 14, 4, 1);
  tft.setTextColor(kColorText, kColorHeaderBg);
  tft.drawCentreString(presetBadge(audio.preset), 68, 4, 1);
  drawBatteryIcon();
  tft.drawFastHLine(0, 15, 128, kColorAccent);
  s_lastBluetoothConnected = audio.bluetoothConnected;
  s_lastHeaderPreset = audio.preset;
  s_headerDrawn = true;
}

void drawFooter(bool dashboard = false) {
  tft.fillRect(0, 113, 128, 15, kColorBg);
  tft.drawFastHLine(0, 112, 128, kColorAccent);
  tft.fillRoundRect(3, 115, 59, 11, 4, kColorCardBg);
  tft.drawRoundRect(3, 115, 59, 11, 4, kColorBorder);
  tft.fillRoundRect(66, 115, 59, 11, 4, kColorCardBg);
  tft.drawRoundRect(66, 115, 59, 11, 4, kColorBorder);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(kColorText, kColorCardBg);
  tft.drawCentreString(dashboard ? "[MENU]" : "[SELECT]", 32, 117, 1);
  tft.drawCentreString("[BACK]", 95, 117, 1);
}

void clearViewport() {
  tft.fillRect(0, 16, 128, 96, kColorBg);
}

void drawCard(int x, int y, int w, int h, bool selected = false) {
  const uint16_t fill = selected ? kColorSelectedBg : kColorCardBg;
  tft.fillRoundRect(x, y, w, h, 5, fill);
  tft.drawRoundRect(x, y, w, h, 5, selected ? kColorAccent : kColorBorder);
  if (selected) tft.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 4, kColorGlow);
}

void drawMainMenu() {
  clearViewport();
  const int xs[4] = {4, 66, 4, 66};
  const int ys[4] = {22, 22, 67, 67};
  const char* titles[4] = {"DASHBOARD", "PRESETS", "MANUAL EQ", "SYSTEM"};
  const char* subtitles[4] = {"PLAYER", "5 EQS", "9 BANDS", "INFO"};
  for (uint8_t i = 0; i < 4; ++i) {
    const bool selected = i == s_menuCursor;
    drawCard(xs[i], ys[i], 58, 40, selected);
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(selected ? kColorAccent : kColorText,
                      selected ? kColorSelectedBg : kColorCardBg);
    tft.drawCentreString(titles[i], xs[i] + 29, ys[i] + 10, 1);
    tft.setTextColor(kColorMuted, selected ? kColorSelectedBg : kColorCardBg);
    tft.drawCentreString(subtitles[i], xs[i] + 29, ys[i] + 24, 1);
  }
  drawFooter();
  s_viewDirty = false;
}

uint16_t meterFillColor(uint8_t level, bool clipping) {
  if (clipping) return kColorMeterRed;
  if (level >= kVisualizerDbMinus3Percent) return kColorMeterAmber;
  if (level >= kVisualizerDbMinus12Percent) return kColorMeterCyan;
  return kColorMeterGreen;
}


uint8_t levelToFillWidth(uint8_t level) {
  const uint8_t bounded = std::min<uint8_t>(level, 100);
  return static_cast<uint8_t>(
      (static_cast<uint16_t>(bounded) * kMeterTrackWidth + 99) / 100);
}
uint8_t peakToDbPercent(uint16_t peak) {
  if (peak == 0) return 0;
  const float dbfs = 20.0f * log10f(static_cast<float>(peak) / 32768.0f);
  const float rangeDb = -static_cast<float>(kVisualizerDbFloor);
  const long percent = lroundf((dbfs - kVisualizerDbFloor) * 100.0f / rangeDb);
  return static_cast<uint8_t>(std::max<long>(0, std::min<long>(100, percent)));
}

void updateMeterSignal(MeterState& meter, uint16_t inputPeak, uint32_t now) {
  const uint8_t input = peakToDbPercent(inputPeak);
  const int16_t targetQ2 = static_cast<int16_t>(input) << 2;
  if (!meter.signalInitialized) {
    meter.smoothedLevelQ2 = targetQ2;
    meter.heldPeakWidth = levelToFillWidth(input);
    meter.peakHoldMs = now;
    meter.decayDeadlineMs = now + kVisualizerPeakHoldMs;
    meter.clipping = inputPeak >= kVisualizerClipThreshold;
    meter.signalInitialized = true;
    return;
  }

  // Smooth the bar independently from the raw peak marker.
  const int16_t deltaQ2 = targetQ2 - meter.smoothedLevelQ2;
  if (deltaQ2 >= 0) {
    const int16_t attackStep =
        std::max<int16_t>(1, deltaQ2 >> kVisualizerAttackShift);
    meter.smoothedLevelQ2 += attackStep;
  } else {
    meter.smoothedLevelQ2 += deltaQ2 >> kVisualizerReleaseShift;
  }
  meter.clipping = inputPeak >= kVisualizerClipThreshold;

  // The marker follows the largest raw sample captured for this UI interval.
  const int16_t currentBarWidth = levelToFillWidth(input);
  if (currentBarWidth >= meter.heldPeakWidth) {
    meter.heldPeakWidth = currentBarWidth;
    meter.peakHoldMs = now;
    meter.decayDeadlineMs = now + kVisualizerPeakHoldMs;
    return;
  }

  if (meter.heldPeakWidth <= 0 ||
      now - meter.peakHoldMs < kVisualizerPeakHoldMs) return;
  if (static_cast<int32_t>(now - meter.decayDeadlineMs) < 0) return;

  const uint32_t decayFrames =
      1U + static_cast<uint32_t>(now - meter.decayDeadlineMs) / kUiFramePeriodMs;
  const uint32_t decayPixels =
      decayFrames * kVisualizerPeakDecayPixelsPerFrame;
  meter.heldPeakWidth = static_cast<int16_t>(
      decayPixels >= static_cast<uint32_t>(meter.heldPeakWidth)
          ? 0
          : meter.heldPeakWidth - decayPixels);
  meter.decayDeadlineMs += decayFrames * kUiFramePeriodMs;
}

void redrawMeterTrackAndFill(int y, uint8_t fillWidth, uint16_t fillColor) {
  tft.fillRoundRect(kMeterStartX, y, kMeterTrackWidth, kMeterBarHeight,
                    kMeterRadius, kColorMeterTrack);
  if (fillWidth == 0) return;
  const int fillRadius = std::min(kMeterRadius, static_cast<int>(fillWidth) / 2);
  tft.fillRoundRect(kMeterStartX, y, fillWidth, kMeterBarHeight,
                    fillRadius, fillColor);
}

bool drawMeterFill(MeterState& meter, int y, uint8_t fillWidth,
                   uint16_t fillColor, int* dirtyStart, int* dirtyEnd) {
  const bool force = meter.forceRedraw ||
                     meter.renderedFillWidth == kInvalidFillWidth;
  const bool colorChanged = meter.renderedFillColor != fillColor;
  const uint8_t oldWidth = meter.renderedFillWidth == kInvalidFillWidth
      ? 0 : meter.renderedFillWidth;
  *dirtyStart = kMeterTrackWidth + 1;
  *dirtyEnd = -1;

  if (force || colorChanged) {
    redrawMeterTrackAndFill(y, fillWidth, fillColor);
    *dirtyStart = 0;
    *dirtyEnd = kMeterTrackWidth;
  } else if (oldWidth != fillWidth) {
    const int lower = std::min<int>(oldWidth, fillWidth);
    const int upper = std::max<int>(oldWidth, fillWidth);
    const int start = std::max(0, lower - kMeterRadius - 1);
    const int end = std::min(kMeterTrackWidth, upper);

    // Only repaint the old/new rounded leading edge; keep the fill inside its
    // requested width so the white peak marker aligns with the actual bar tip.
    if (start <= kMeterRadius || end >= kMeterTrackWidth - kMeterRadius) {
      redrawMeterTrackAndFill(y, fillWidth, fillColor);
      *dirtyStart = 0;
      *dirtyEnd = kMeterTrackWidth;
    } else {
      tft.fillRect(kMeterStartX + start, y, end - start, kMeterBarHeight,
                   kColorMeterTrack);
      if (fillWidth > start) {
        const int visibleFillEnd = std::min(end, static_cast<int>(fillWidth));
        const int localWidth = visibleFillEnd - start;
        if (localWidth > 0) {
          const int localRadius = std::min(kMeterRadius, localWidth / 2);
          tft.fillRoundRect(kMeterStartX + start, y, localWidth,
                            kMeterBarHeight, localRadius, fillColor);
          if (localWidth > kMeterRadius) {
            tft.fillRect(kMeterStartX + start, y, kMeterRadius,
                         kMeterBarHeight, fillColor);
          }
        }
      }
      *dirtyStart = start;
      *dirtyEnd = end;
    }
  }

  meter.renderedFillWidth = fillWidth;
  meter.renderedFillColor = fillColor;
  meter.forceRedraw = false;
  return *dirtyEnd >= 0;
}

void restorePeakMarker(int x, int y, uint8_t fillWidth, uint16_t fillColor) {
  tft.drawFastVLine(x, y - 1, kMeterBarHeight + 2, kColorCardBg);
  const int offset = x - kMeterStartX;
  if (offset < 0 || offset >= kMeterTrackWidth) return;
  if (offset <= kMeterRadius || offset >= kMeterTrackWidth - kMeterRadius - 1) {
    redrawMeterTrackAndFill(y, fillWidth, fillColor);
    return;
  }
  tft.drawFastVLine(x, y, kMeterBarHeight,
                    offset < fillWidth ? fillColor : kColorMeterTrack);
}

void drawMeterRow(const char* label, int y, MeterState& meter) {
  const int16_t smoothValue = (meter.smoothedLevelQ2 + 2) >> 2;
  const uint8_t level = static_cast<uint8_t>(
      std::min<int16_t>(100, std::max<int16_t>(0, smoothValue)));
  const uint8_t fillWidth = levelToFillWidth(level);
  const uint16_t fillColor = meterFillColor(level, meter.clipping);

  if (meter.forceRedraw || meter.renderedFillWidth == kInvalidFillWidth) {
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(kColorMuted, kColorCardBg);
    tft.drawString(label, 9, y, 1);
  }

  int dirtyStart = kMeterTrackWidth + 1;
  int dirtyEnd = -1;
  const bool barRedrawn =
      drawMeterFill(meter, y, fillWidth, fillColor, &dirtyStart, &dirtyEnd);

  int16_t markerX = -1;
  if (meter.heldPeakWidth > 0) {
    markerX = static_cast<int16_t>(kMeterStartX + meter.heldPeakWidth);
  }
  const int16_t oldMarkerX = meter.renderedMarkerX;
  if (oldMarkerX >= 0 && oldMarkerX != markerX) {
    restorePeakMarker(oldMarkerX, y, fillWidth, fillColor);
  }

  const bool markerCovered = barRedrawn &&
      markerX >= kMeterStartX + dirtyStart &&
      markerX < kMeterStartX + dirtyEnd;
  if (markerX >= 0 && (markerX != oldMarkerX || markerCovered)) {
    const uint16_t markerColor =
        meter.heldPeakWidth >= kMeterTrackWidth - 1 ? kColorMeterRed : kColorText;
    tft.drawFastVLine(markerX, y - 1, kMeterBarHeight + 2, markerColor);
  }
  meter.renderedMarkerX = markerX;
}

void invalidateMeterRenderCache() {
  s_leftMeter.forceRedraw = true;
  s_rightMeter.forceRedraw = true;
  s_lastDashboardVolume = 255;
}

void updateStereoMeterSignal(const AudioSnapshot& audio, uint32_t now) {
  updateMeterSignal(s_leftMeter, audio.peakLeft, now);
  updateMeterSignal(s_rightMeter, audio.peakRight, now);
}

void drawStereoMeters(const AudioSnapshot& audio) {
  const uint32_t now = millis();
  updateStereoMeterSignal(audio, now);
  drawMeterRow("L", 77, s_leftMeter);
  drawMeterRow("R", 96, s_rightMeter);
}

void drawMeterScale() {
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(kColorMuted, kColorCardBg);
  tft.drawString("-60", kMeterStartX, 69, 1);
  tft.drawCentreString("-36", kMeterStartX + 38, 69, 1);
  tft.drawCentreString("-12", kMeterStartX + 77, 69, 1);
  tft.drawRightString("0", kMeterStartX + kMeterTrackWidth, 69, 1);
}

void drawDashboardStreamStatus(bool streamActive, bool force = false) {
  if (!force && s_dashboardStreamStatusValid &&
      streamActive == s_lastDashboardStreamActive) return;
  tft.fillRect(82, 60, 40, 11, kColorCardBg);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(streamActive ? kColorMeterGreen : kColorMuted, kColorCardBg);
  tft.drawRightString(streamActive ? "STREAM" : "NO PCM", 120, 61, 1);
  s_lastDashboardStreamActive = streamActive;
  s_dashboardStreamStatusValid = true;
}

void drawDashboardVolume(uint8_t volume) {
  if (volume == s_lastDashboardVolume) return;
  tft.fillRect(74, 24, 46, 10, kColorCardBg);
  char label[12];
  snprintf(label, sizeof(label), "VOL %u%%", volume);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(volume >= 80 ? kColorAmber : kColorAccent, kColorCardBg);
  tft.drawRightString(label, 120, 25, 1);
  s_lastDashboardVolume = volume;
}
void drawDashboardStatic(const AudioSnapshot& audio) {
  clearViewport();
  drawCard(4, 21, 120, 34);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(kColorMuted, kColorCardBg);
  tft.drawString("AUDIO OUT", 10, 25, 1);
  const uint32_t rateTenths = audio.sampleRate / 100;
  char format[28];
  snprintf(format, sizeof(format), "%lu.%lukHz / %ubit",
           static_cast<unsigned long>(rateTenths / 10),
           static_cast<unsigned long>(rateTenths % 10),
           static_cast<unsigned>(kAudioBitDepth));
  tft.setTextColor(kColorText, kColorCardBg);
  tft.drawString(format, 10, 39, 1);
  invalidateMeterRenderCache();
  drawDashboardVolume(audio.volume);

  drawCard(4, 58, 120, 52);
  tft.setTextColor(kColorMuted, kColorCardBg);
  tft.drawString("STEREO dBFS", 10, 61, 1);
  drawDashboardStreamStatus(audio.streamActive, true);
  drawMeterScale();
  drawStereoMeters(audio);
  drawFooter(true);
  s_lastDashboardRate = audio.sampleRate;
  s_viewDirty = false;
}

void drawPlayerDynamic(const AudioSnapshot& audio) {
  drawDashboardVolume(audio.volume);
  drawDashboardStreamStatus(audio.streamActive);
  drawStereoMeters(audio);
}

void drawPresetPage(const AudioSnapshot& audio) {
  clearViewport();
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(kColorAccent, kColorBg);
  tft.drawString("SELECT PRESET", 5, 19, 1);
  for (uint8_t i = 0; i < 5; ++i) {
    const int y = 31 + i * 14;
    const bool selected = i == s_selectedPreset;
    drawCard(4, y - 3, 120, 13, selected);
    tft.setTextColor(selected ? kColorAccent : kColorText,
                      selected ? kColorSelectedBg : kColorCardBg);
    tft.drawString(selected ? ">" : " ", 9, y, 1);
    tft.drawString(kPresetNames[i], 21, y, 1);
  }
  tft.setTextColor(kColorMuted, kColorBg);
  tft.drawString("ACTIVE", 5, 103, 1);
  tft.setTextColor(kColorText, kColorBg);
  tft.drawString(presetBadge(audio.preset), 43, 103, 1);
  drawFooter();
  s_viewDirty = false;
}

void drawEqPage(const AudioSnapshot& audio) {
  clearViewport();
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(kColorAccent, kColorBg);
  tft.drawString("MANUAL EQ / 9 BAND", 3, 19, 1);

  char gainLabel[16];
  snprintf(gainLabel, sizeof(gainLabel), "%sHz %+0.1fdB", kBandShortNames[s_selectedBand], audio.bandDb[s_selectedBand]);
  tft.setTextColor(kColorHighlight, kColorBg);
  tft.drawCentreString(gainLabel, 7 + s_selectedBand * 14, 29, 1);

  constexpr int kTrackTop = 41;
  constexpr int kZeroY = 62;
  constexpr int kTrackBottom = 83;
  // Nine compact bars fit the 128-pixel TFT without clipping the 16 kHz edge.
  const char* tinyBandNames[kBandCount] = {
      "25", "45", "60", "150", "400", "1k", "2k", "6k", "16k"};
  for (uint8_t i = 0; i < kBandCount; ++i) {
    const int centerX = 7 + i * 14;
    const bool selected = i == s_selectedBand;
    const float db = constrain(audio.bandDb[i], -18.0f, 18.0f);
    const int knobY = kZeroY - static_cast<int>(lroundf(db * 21.0f / 18.0f));
    if (selected) {
      tft.drawRoundRect(centerX - 6, kTrackTop - 4, 13,
                        kTrackBottom - kTrackTop + 10, 4, kColorGlow);
    }
    tft.drawFastVLine(centerX, kTrackTop, kTrackBottom - kTrackTop + 1, kColorMeterTrack);
    tft.drawFastHLine(centerX - 3, kZeroY, 7, kColorMuted);
    if (knobY < kZeroY)
      tft.fillRect(centerX - 2, knobY, 5, kZeroY - knobY, kColorAccent);
    else if (knobY > kZeroY)
      tft.fillRect(centerX - 2, kZeroY + 1, 5, knobY - kZeroY, kColorAccent);
    if (selected) {
      tft.fillCircle(centerX, knobY, 5, kColorHighlight);
      tft.fillCircle(centerX, knobY, 3, kColorAccent);
      tft.fillCircle(centerX, knobY, 1, kColorText);
    } else {
      tft.fillCircle(centerX, knobY, 2, kColorAccent);
    }
    tft.setTextColor(selected ? kColorHighlight : kColorMuted, kColorBg);
    tft.drawCentreString(tinyBandNames[i], centerX, 88, 1);
  }

  tft.setTextColor(kColorMuted, kColorBg);
  tft.drawCentreString("UP/DOWN: GAIN", 64, 102, 1);
  drawFooter();
  for (uint8_t i = 0; i < kBandCount; ++i) s_lastEqGains[i] = audio.bandDb[i];
  s_lastEqGainsValid = true;
  s_viewDirty = false;
}

void drawSystemPage(const AudioSnapshot& audio) {
  clearViewport();
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(kColorAccent, kColorBg);
  tft.drawString("SYSTEM INFO", 5, 19, 1);
  drawCard(4, 28, 120, 78);
  tft.setTextColor(kColorMuted, kColorCardBg);

  char line[32];
  snprintf(line, sizeof(line), "Firmware: v%s", kFirmwareVersion);
  tft.drawString(line, 10, 34, 1);
  const uint32_t rateTenths = audio.sampleRate / 100;
  snprintf(line, sizeof(line), "Audio %lu.%luk/%ub",
           static_cast<unsigned long>(rateTenths / 10),
           static_cast<unsigned long>(rateTenths % 10),
           static_cast<unsigned>(kAudioBitDepth));
  tft.drawString(line, 10, 47, 1);
  snprintf(line, sizeof(line), "BT %s  %s",
           audio.bluetoothConnected ? "ON" : "OFF",
           audio.streamActive ? "STREAM" : "NO PCM");
  tft.drawString(line, 10, 60, 1);
  snprintf(line, sizeof(line), "Mute:%s Heap:%lu",
           audio.muted ? "ON" : "OFF",
           static_cast<unsigned long>(ESP.getFreeHeap()));
  tft.drawString(line, 10, 73, 1);
  snprintf(line, sizeof(line), "Drop:%lu Max:%lums",
           static_cast<unsigned long>(audio.droppedPackets > 9999 ? 9999 : audio.droppedPackets),
           static_cast<unsigned long>(audio.pcmGapMaxMs > 999 ? 999 : audio.pcmGapMaxMs));
  tft.drawString(line, 10, 86, 1);
  // Gap is the count of >=35 ms packet gaps in the current playback session.
  snprintf(line, sizeof(line), "Gap:%02lu I:%02lu C:%02lu",
           static_cast<unsigned long>(audio.pcmGapEvents > 99 ? 99 : audio.pcmGapEvents),
           static_cast<unsigned long>(audio.i2sWriteFaults > 99 ? 99 : audio.i2sWriteFaults),
           static_cast<unsigned long>(audio.clippedFrames > 99 ? 99 : audio.clippedFrames));
  tft.drawString(line, 10, 97, 1);

  drawFooter();
  s_lastSystemRefreshMs = millis();
  s_viewDirty = false;
}

void drawCurrentPage(const AudioSnapshot& audio) {
  switch (s_page) {
    case UiPage::MainMenu: drawMainMenu(); break;
    case UiPage::Dashboard: drawDashboardStatic(audio); break;
    case UiPage::Preset: drawPresetPage(audio); break;
    case UiPage::EqEditor: drawEqPage(audio); break;
    case UiPage::System: drawSystemPage(audio); break;
  }
}

void drawVolumeOverlay() {
  const int x = kUiVolumeOverlayX;
  const int y = kUiVolumeOverlayY;
  const int w = kUiVolumeOverlayWidth;
  const int h = kUiVolumeOverlayHeight;
  const bool highVolume = s_volumeOverlayValue >= 90;
  const uint16_t levelColor = highVolume ? kColorAmber : kColorAccent;

  // A single crisp outline keeps the overlay distinct without the old double-glow look.
  tft.fillRoundRect(x, y, w, h, 6, kColorCardBg);
  tft.drawRoundRect(x, y, w, h, 6, kColorAccent);

  // Compact speaker glyph with one cyan sound wave.
  const int speakerX = x + 9;
  const int speakerY = y + 15;
  tft.fillRect(speakerX, speakerY + 5, 4, 8, kColorText);
  tft.fillTriangle(speakerX + 4, speakerY + 5,
                   speakerX + 12, speakerY + 1,
                   speakerX + 12, speakerY + 17,
                   kColorText);
  tft.drawLine(speakerX + 15, speakerY + 5,
               speakerX + 18, speakerY + 8, kColorAccent);
  tft.drawFastVLine(speakerX + 18, speakerY + 8, 4, kColorAccent);
  tft.drawLine(speakerX + 18, speakerY + 12,
               speakerX + 15, speakerY + 15, kColorAccent);

  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(kColorMuted, kColorCardBg);
  tft.drawString("VOLUME", x + 30, y + 6, 1);

  char value[8];
  snprintf(value, sizeof(value), "%u%%", s_volumeOverlayValue);
  tft.setTextSize(2);
  tft.setTextColor(levelColor, kColorCardBg);
  tft.drawString(value, x + 30, y + 18, 1);
  tft.setTextSize(1);

  // Thick pill track with an inset fill, sized to retain margins at 0% and 100%.
  const int barX = x + 10;
  const int barY = y + 39;
  const int barW = w - 20;
  const int barH = 7;
  const int innerW = barW - 4;
  tft.fillRoundRect(barX, barY, barW, barH, 3, kColorBorder);
  const int fillW = (innerW * s_volumeOverlayValue + 50) / 100;
  if (fillW > 0)
    tft.fillRoundRect(barX + 2, barY + 2, fillW, 3, 1, levelColor);
}
void restoreOverlayArea(const AudioSnapshot& audio) {
  tft.setViewport(kUiVolumeOverlayX, kUiVolumeOverlayY,
                  kUiVolumeOverlayWidth, kUiVolumeOverlayHeight, false);
  drawCurrentPage(audio);
  tft.resetViewport();
  if (s_page == UiPage::Dashboard) invalidateMeterRenderCache();
}
void checkVolumeOverlayTimer(const AudioSnapshot& audio, uint32_t now) {
  if (!s_volumeOverlayActive ||
      now - s_volumeOverlayLastActivityMs < kUiVolumeOverlayTimeoutMs) return;
  s_volumeOverlayActive = false;
  s_volumeOverlayNeedsDraw = false;
  restoreOverlayArea(audio);
}

void switchPage(UiPage page) {
  s_page = page;
  s_viewDirty = true;
}

void handleInput(InputEvent event, AudioSnapshot& audio) {
  switch (event) {
    case InputEvent::VolumeUp:
    case InputEvent::VolumeDown: {
      const int base = s_volumeOverlayActive ? s_volumeOverlayValue : audio.volume;
      const int delta = event == InputEvent::VolumeUp ? 5 : -5;
      const uint8_t next = static_cast<uint8_t>(constrain(base + delta, 0, 100));
      AudioEngine_SetVolume(next);
      audio.volume = next;
      s_volumeOverlayValue = next;
      s_volumeOverlayActive = true;
      s_volumeOverlayNeedsDraw = true;
      s_volumeOverlayLastActivityMs = millis();
      break;
    }
    case InputEvent::CenterLong:
      switchPage(UiPage::MainMenu);
      break;
    case InputEvent::Center:
      if (s_page == UiPage::Dashboard) {
        switchPage(UiPage::MainMenu);
      } else if (s_page == UiPage::MainMenu) {
        switch (s_menuCursor) {
          case 0: switchPage(UiPage::Dashboard); break;
          case 1: s_selectedPreset = audio.preset; switchPage(UiPage::Preset); break;
          case 2: s_selectedBand = 0; switchPage(UiPage::EqEditor); break;
          case 3: switchPage(UiPage::System); break;
        }
      } else if (s_page == UiPage::Preset) {
        AudioEngine_SetPreset(s_selectedPreset);
        switchPage(UiPage::Dashboard);
      } else if (s_page == UiPage::EqEditor) {
        s_viewDirty = true;
      } else {
        switchPage(UiPage::MainMenu);
      }
      break;
    case InputEvent::Up:
      if (s_page == UiPage::MainMenu) s_menuCursor = (s_menuCursor + 2) % 4;
      else if (s_page == UiPage::Preset) s_selectedPreset = (s_selectedPreset + 4) % 5;
      else if (s_page == UiPage::EqEditor)
        AudioEngine_AdjustBandGain(s_selectedBand, 1.0f);
      else if (s_page == UiPage::System) switchPage(UiPage::MainMenu);
      s_viewDirty = true;
      break;
    case InputEvent::Down:
      if (s_page == UiPage::MainMenu) s_menuCursor = (s_menuCursor + 2) % 4;
      else if (s_page == UiPage::Preset) s_selectedPreset = (s_selectedPreset + 1) % 5;
      else if (s_page == UiPage::EqEditor)
        AudioEngine_AdjustBandGain(s_selectedBand, -1.0f);
      else if (s_page == UiPage::System) switchPage(UiPage::MainMenu);
      s_viewDirty = true;
      break;
    case InputEvent::Left:
      if (s_page == UiPage::MainMenu) s_menuCursor = (s_menuCursor + 3) % 4;
      else if (s_page == UiPage::EqEditor)
        s_selectedBand = (s_selectedBand + kBandCount - 1) % kBandCount;
      else if (s_page == UiPage::Preset) s_selectedPreset = (s_selectedPreset + 4) % 5;
      else switchPage(UiPage::MainMenu);
      s_viewDirty = true;
      break;
    case InputEvent::Right:
      if (s_page == UiPage::MainMenu) s_menuCursor = (s_menuCursor + 1) % 4;
      else if (s_page == UiPage::EqEditor)
        s_selectedBand = (s_selectedBand + 1) % kBandCount;
      else if (s_page == UiPage::Preset) s_selectedPreset = (s_selectedPreset + 1) % 5;
      else if (s_page == UiPage::Dashboard) switchPage(UiPage::MainMenu);
      else switchPage(UiPage::MainMenu);
      s_viewDirty = true;
      break;
  }
}

void render(const AudioSnapshot& audio) {
  const uint32_t now = millis();
  checkVolumeOverlayTimer(audio, now);

  if (s_page == UiPage::EqEditor) {
    bool changed = !s_lastEqGainsValid;
    for (uint8_t i = 0; i < kBandCount && !changed; ++i) {
      if (fabsf(audio.bandDb[i] - s_lastEqGains[i]) > 0.01f) changed = true;
    }
    if (changed) s_viewDirty = true;
  }
  if (s_page == UiPage::Dashboard && audio.sampleRate != s_lastDashboardRate) {
    s_viewDirty = true;
  }
  if (s_page == UiPage::System && now - s_lastSystemRefreshMs >= 1000) {
    s_viewDirty = true;
  }

  const bool pageRedrawn = s_viewDirty;
  if (pageRedrawn) {
    drawCurrentPage(audio);
  } else if (s_page == UiPage::Dashboard) {
    // Keep the meter filter/peak state current while the card covers its lower edge.
    if (s_volumeOverlayActive) updateStereoMeterSignal(audio, now);
    else drawPlayerDynamic(audio);
  }
  if (!s_headerDrawn || audio.preset != s_lastHeaderPreset ||
      audio.bluetoothConnected != s_lastBluetoothConnected) {
    drawHeader(audio);
  }

  if (s_volumeOverlayActive && (s_volumeOverlayNeedsDraw || pageRedrawn)) {
    drawVolumeOverlay();
    s_volumeOverlayNeedsDraw = false;
  }
}

void uiTask(void*) {
  tft.init();
  tft.setRotation(2);  // portrait 128x128; change to 2 if your panel is physically inverted
  tft.setTextWrap(false);
  tft.fillScreen(kColorBg);  // Boot only; normal frames use dirty-region rendering.

  AudioSnapshot audio{};
  AudioEngine_GetSnapshot(&audio);
  drawHeader(audio);
  s_viewDirty = true;
  render(audio);

  uint32_t lastUiFrameMs = millis();
  for (;;) {
    InputEvent event{};
    while (xQueueReceive(s_eventQueue, &event, 0) == pdTRUE) {
      AudioEngine_GetSnapshot(&audio);
      handleInput(event, audio);
    }

    const uint32_t now = millis();
    if (now - lastUiFrameMs >= kUiFramePeriodMs) {
      // Consume the accumulated peak window atomically; rendering never reads
      // live audio state while the Core 1 task is publishing telemetry.
      AudioEngine_GetSnapshotForUi(&audio);
      render(audio);
      lastUiFrameMs = now;
    }

    const bool meterAnimating = s_page == UiPage::Dashboard &&
        (audio.peakLeft != 0 || audio.peakRight != 0 ||
         s_leftMeter.smoothedLevelQ2 != 0 || s_rightMeter.smoothedLevelQ2 != 0 ||
         s_leftMeter.heldPeakWidth != 0 || s_rightMeter.heldPeakWidth != 0);
    const bool needsResponsiveFrames = s_volumeOverlayActive || meterAnimating;
    // Rendering is capped separately by kUiFramePeriodMs. Polling the queue
    // every two RTOS ticks keeps button/overlay response snappy without waking
    // the UI task on every single tick while audio meters are active.
    vTaskDelay(needsResponsiveFrames ? pdMS_TO_TICKS(2)
                                     : pdMS_TO_TICKS(kUiIdleDelayMs));
  }
}
}  // namespace

bool UiEngine_Begin(QueueHandle_t eventQueue) {
  if (eventQueue == nullptr) return false;
  s_eventQueue = eventQueue;
  return xTaskCreatePinnedToCore(uiTask, "UiTask", Horizon::kUiTaskStack,
                                 nullptr, 1, nullptr, 0) == pdPASS;
}
