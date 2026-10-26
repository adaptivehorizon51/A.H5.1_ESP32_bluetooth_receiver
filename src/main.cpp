#include <Arduino.h>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <BluetoothA2DPSink.h>
#include <Preferences.h>
#include <esp_bt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "audio_engine.h"
#include "config.h"
#include "input_manager.h"
#include "ui_engine.h"

namespace {
// The DSP owns I2S1. Give the A2DP library a no-op output so its legacy
// default does not attempt to reconfigure the unused I2S0 during codec setup.
class A2dpNullOutput final : public BluetoothA2DPOutput {
 public:
  bool begin() override { return true; }
  size_t write(const uint8_t*, size_t length) override { return length; }
  void end() override {}
  void set_sample_rate(int) override {}
  void set_output_active(bool) override {}
};
A2dpNullOutput a2dpNullOutput;
BluetoothA2DPSink a2dpSink(a2dpNullOutput);
QueueHandle_t inputEventQueue = nullptr;
bool audioEngineReady = false;
bool a2dpStartRequested = false;
bool settingsRestored = false;
uint32_t nextAudioRetryMs = 0;
uint32_t nextBluetoothRetryMs = 0;
uint32_t lastAutoSavePollMs = 0;
uint32_t settingsDirtySinceMs = 0;
uint32_t settingsSaveRetryMs = 0;
uint32_t settingsIgnoreUntilMs = 0;
Preferences settingsPrefs;
constexpr uint32_t kSettingsMagic = 0x484F5332; // "HOS2"
constexpr uint16_t kSettingsVersion = 1;
struct PersistedSettings {
  uint32_t magic;
  uint16_t version;
  uint8_t preset;
  uint8_t volume;
  uint8_t muted;
  uint8_t reserved[3];
  int16_t customTenths[2][Horizon::kBandCount];
  uint32_t checksum;
};
PersistedSettings savedSettings{};
AudioSnapshot lastObservedSettings{};
bool haveLastObservedSettings = false;
constexpr uint32_t kSettingsSaveDebounceMs = 900;
constexpr uint32_t kSettingsSaveRetryBackoffMs = 5000;

uint32_t settingsChecksum(const PersistedSettings& settings) {
  // FNV-1a over all payload bytes before checksum; deterministic across boots.
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&settings);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < offsetof(PersistedSettings, checksum); ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

void setDefaultSettings() {
  memset(&savedSettings, 0, sizeof(savedSettings));
  savedSettings.magic = kSettingsMagic;
  savedSettings.version = kSettingsVersion;
  savedSettings.preset = 0;
  savedSettings.volume = 70;
  savedSettings.muted = 0;
  const float defaults[2][Horizon::kBandCount] = {
      {3, 2, 0, 0, 0, 0, 0, 0}, {0, 1, 2, -1, 0, 1, 2, 1}};
  for (uint8_t slot = 0; slot < 2; ++slot)
    for (uint8_t band = 0; band < Horizon::kBandCount; ++band)
      savedSettings.customTenths[slot][band] = static_cast<int16_t>(defaults[slot][band] * 10.0f);
  savedSettings.checksum = settingsChecksum(savedSettings);
}

void loadPersistentSettings() {
  setDefaultSettings();
  if (!settingsPrefs.begin("horizon", true)) {
    DEBUG_PRINTLN("[NVS] Settings namespace unavailable; using defaults.");
    return;
  }
  if (settingsPrefs.getBytesLength("settings") != sizeof(PersistedSettings)) {
    DEBUG_PRINTLN("[NVS] No saved settings; using defaults.");
    settingsPrefs.end();
    return;
  }
  PersistedSettings candidate{};
  if (settingsPrefs.getBytes("settings", &candidate, sizeof(candidate)) != sizeof(candidate) ||
      candidate.magic != kSettingsMagic || candidate.version != kSettingsVersion ||
      candidate.preset >= 5 || candidate.volume > 100 || candidate.muted > 1 ||
      candidate.checksum != settingsChecksum(candidate)) {
    DEBUG_PRINTLN("[NVS] Saved settings invalid; using safe defaults.");
    settingsPrefs.end();
    return;
  }
  for (uint8_t slot = 0; slot < 2; ++slot) {
    for (uint8_t band = 0; band < Horizon::kBandCount; ++band) {
      if (candidate.customTenths[slot][band] < -180 || candidate.customTenths[slot][band] > 180) {
        DEBUG_PRINTLN("[NVS] EQ values invalid; using safe defaults.");
        settingsPrefs.end();
        return;
      }
    }
  }
  savedSettings = candidate;
  // Runtime mute is disabled; ignore any mute bit saved by older firmware.
  savedSettings.muted = 0;
  settingsPrefs.end();
  DEBUG_PRINTF("[NVS] Restored preset %u, volume %u%%, mute OFF and both custom EQ slots.\n",
                savedSettings.preset, savedSettings.volume);
}

void loadCustomPresetDataIntoAudioEngine() {
  float gains[Horizon::kBandCount];
  for (uint8_t slot = 0; slot < 2; ++slot) {
    for (uint8_t band = 0; band < Horizon::kBandCount; ++band)
      gains[band] = savedSettings.customTenths[slot][band] / 10.0f;
    AudioEngine_LoadCustomPreset(slot, gains);
  }
}

void applyPersistentSettings() {
  AudioEngine_SetPreset(savedSettings.preset);
  AudioEngine_SetVolume(savedSettings.volume);
  savedSettings.muted = 0;
  settingsRestored = true;
  settingsIgnoreUntilMs = millis() + 1800;
  AudioEngine_GetSnapshot(&lastObservedSettings);
  haveLastObservedSettings = true;
}

bool persistentFieldsChanged(const AudioSnapshot& a, const AudioSnapshot& b) {
  if (a.preset != b.preset || a.volume != b.volume) return true;
  for (uint8_t i = 0; i < Horizon::kBandCount; ++i)
    if (fabsf(a.bandDb[i] - b.bandDb[i]) >= 0.05f) return true;
  return false;
}

// SYS-01: report success only when Preferences accepted the complete record.
// The checksum/version are finalized immediately before writing the NVS blob.
bool savePersistentSettings() {
  savedSettings.magic = kSettingsMagic;
  savedSettings.version = kSettingsVersion;
  savedSettings.checksum = settingsChecksum(savedSettings);

  if (!settingsPrefs.begin("horizon", false)) {
    DEBUG_PRINTLN("[NVS] Cannot open settings namespace for write.");
    return false;
  }

  const size_t written = settingsPrefs.putBytes(
      "settings", &savedSettings, sizeof(savedSettings));
  settingsPrefs.end();
  return written == sizeof(savedSettings);
}

void pollAutoSave() {
  if (!settingsRestored || static_cast<int32_t>(millis() - settingsIgnoreUntilMs) < 0) return;
  AudioSnapshot current{};
  AudioEngine_GetSnapshot(&current);
  if (!haveLastObservedSettings) {
    lastObservedSettings = current;
    haveLastObservedSettings = true;
    return;
  }
  // Keep the in-RAM backup of a custom slot current even if the user switches
  // presets before the flash-save debounce expires. Flash is still written only
  // after settings settle, but a quick slot switch cannot discard the edit.
  if (current.preset == 3 || current.preset == 4) {
    const uint8_t slot = current.preset - 3;
    for (uint8_t band = 0; band < Horizon::kBandCount; ++band) {
      savedSettings.customTenths[slot][band] = static_cast<int16_t>(
          lroundf(constrain(current.bandDb[band], -18.0f, 18.0f) * 10.0f));
    }
  }
  if (persistentFieldsChanged(current, lastObservedSettings)) {
    lastObservedSettings = current;
    settingsDirtySinceMs = millis();
    settingsSaveRetryMs = 0;
    return;
  }
  if (settingsDirtySinceMs == 0 ||
      millis() - settingsDirtySinceMs < kSettingsSaveDebounceMs) return;

  savedSettings.preset = current.preset < 5 ? current.preset : 0;
  savedSettings.volume = current.volume;
  savedSettings.muted = 0;
  if (current.preset == 3 || current.preset == 4) {
    const uint8_t slot = current.preset - 3;
    for (uint8_t band = 0; band < Horizon::kBandCount; ++band) {
      const float clamped = constrain(current.bandDb[band], -18.0f, 18.0f);
      savedSettings.customTenths[slot][band] = static_cast<int16_t>(lroundf(clamped * 10.0f));
    }
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - settingsSaveRetryMs) < 0) return;

  // Do not clear the dirty marker on failure: the current RAM settings remain
  // pending until NVS confirms that the complete record was written.
  if (savePersistentSettings()) {
    DEBUG_PRINTLN("[NVS] Settings auto-saved.");
    settingsDirtySinceMs = 0;
    settingsSaveRetryMs = 0;
  } else {
    DEBUG_PRINTLN("[NVS] WARNING: settings save failed; keeping dirty state and retrying later.");
    settingsSaveRetryMs = millis() + kSettingsSaveRetryBackoffMs;
  }
}

void onA2dpConnectionState(esp_a2d_connection_state_t state, void*) {
  AudioEngine_SetBluetoothConnected(state == ESP_A2D_CONNECTION_STATE_CONNECTED);
}
void onA2dpAudioState(esp_a2d_audio_state_t state, void*) {
  DEBUG_PRINTF("[BT] A2DP audio state=%s\n", a2dpSink.to_str(state));
}
}  // namespace

void startBluetoothProfiles() {
  if (!audioEngineReady) return;

  // Controller initialization is retryable and independent from AudioEngine.
  // Repeated calls only advance the Bluetooth steps that have not succeeded.
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
    const esp_err_t bleRelease = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (bleRelease != ESP_OK) {
      DEBUG_PRINTF("[BT] BLE memory release returned %d; continuing with Classic BT.\n",
                   static_cast<int>(bleRelease));
    }
    esp_bt_controller_config_t btConfig = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    btConfig.mode = ESP_BT_MODE_CLASSIC_BT;
    const esp_err_t initResult = esp_bt_controller_init(&btConfig);
    if (initResult != ESP_OK) {
      DEBUG_PRINTF("[BT] Classic controller init failed: %d; retry scheduled.\n",
                   static_cast<int>(initResult));
      return;
    }
  }

  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    const esp_err_t enableResult = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (enableResult != ESP_OK) {
      DEBUG_PRINTF("[BT] Classic controller enable failed: %d; retry scheduled.\n",
                   static_cast<int>(enableResult));
      return;
    }
  }
  if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
    DEBUG_PRINTLN("[BT] Classic controller not enabled; retry scheduled.");
    return;
  }

  if (!a2dpStartRequested) {
    // This library API returns void from start(); once requested, avoid calling
    // it repeatedly (which can duplicate profile initialization). Controller
    // setup failures above remain retryable; A2DP auto-reconnect handles peers.
    a2dpSink.set_stream_reader(AudioEngine_OnBtPcm, false);
    a2dpSink.set_sample_rate_callback(AudioEngine_OnSampleRate);
    a2dpSink.set_auto_reconnect(true);
    a2dpSink.set_on_connection_state_changed(onA2dpConnectionState, nullptr);
    a2dpSink.set_on_audio_state_changed(onA2dpAudioState, nullptr);
    a2dpSink.start(Horizon::kBluetoothName);
    a2dpStartRequested = true;
    DEBUG_PRINTLN("[BT] A2DP sink start requested; connect a phone when playing music.");
  }
}
void setup() {
  setCpuFrequencyMhz(240);
#if ENABLE_DEBUG
  Serial.begin(115200);
  delay(150);
#endif
  DEBUG_PRINTLN();
  DEBUG_PRINTF("\nHORIZON AUDIO DLMS Feature OS v%s\n", Horizon::kFirmwareVersion);
  DEBUG_PRINTF("Free heap at boot: %lu bytes\n", static_cast<unsigned long>(ESP.getFreeHeap()));
  loadPersistentSettings();
  loadCustomPresetDataIntoAudioEngine();

  // Bring up the standalone operating interface first. A missing DAC, I2S
  // pin fault, or audio-task allocation failure must not trap the device in
  // a fatal loop or prevent the TFT and physical buttons from working.
  inputEventQueue = xQueueCreate(20, sizeof(InputEvent));
  if (inputEventQueue == nullptr) {
    DEBUG_PRINTLN("FATAL: could not allocate input queue; reboot required.");
    return;
  }
  if (!InputManager_Begin(inputEventQueue)) {
    DEBUG_PRINTLN("FATAL: input task failed to start; reboot required.");
    return;
  }
  if (!UiEngine_Begin(inputEventQueue)) {
    DEBUG_PRINTLN("FATAL: UI task failed to start; check TFT setup and heap.");
    return;
  }

  // Audio and Bluetooth are optional subsystems during boot. Retry audio
  // initialization periodically instead of blocking the local OS forever.
  nextAudioRetryMs = millis();
  nextBluetoothRetryMs = millis();
}

void loop() {
  uint32_t now = millis();

  // AudioEngine retry is independent from Bluetooth startup/retry.
  if (!audioEngineReady && static_cast<int32_t>(now - nextAudioRetryMs) >= 0) {
    nextAudioRetryMs = now + 5000;
    audioEngineReady = AudioEngine_Begin();
    if (audioEngineReady) {
      DEBUG_PRINTLN("Audio engine ready.");
      applyPersistentSettings();
      nextBluetoothRetryMs = now;
    } else {
      DEBUG_PRINTLN("WARN: audio engine unavailable; retrying in 5 seconds.");
    }
  }

  now = millis();
  if (audioEngineReady && !a2dpStartRequested &&
      static_cast<int32_t>(now - nextBluetoothRetryMs) >= 0) {
    nextBluetoothRetryMs = now + 5000;
    startBluetoothProfiles();
  }

  if (now - lastAutoSavePollMs >= 100) {
    pollAutoSave();
    lastAutoSavePollMs = now;
  }
  vTaskDelay(pdMS_TO_TICKS(Horizon::kInputPollIntervalMs));
}
