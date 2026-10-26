#pragma once
/* Horizon Audio DLMS shared state/types.
 * Phase 1 compatibility layer for the Phase 2-5 modules in this project.
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace HorizonDLMS {
namespace Config {
constexpr uint32_t AUDIO_SAMPLE_RATE = 48000;
constexpr uint8_t AUDIO_CHANNEL_COUNT = 2;
constexpr uint8_t EQ_BAND_COUNT = 9;
constexpr float EQ_DEFAULT_Q_PEAKING = 1.414f;
constexpr float EQ_MIN_GAIN_DB = -18.0f;
constexpr float EQ_MAX_GAIN_DB = 18.0f;
constexpr uint8_t BUTTON_COUNT = 10;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;
constexpr uint32_t BUTTON_LONG_PRESS_MIN_MS = 650;
constexpr uint32_t BUTTON_SCAN_INTERVAL_MS = 20;
constexpr uint32_t BUTTON_SHORT_PRESS_MAX_MS = 600;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;
constexpr uint8_t VOLUME_MIN = 0;
constexpr uint8_t VOLUME_MAX = 100;
constexpr uint8_t VU_BAR_COUNT = 16;
constexpr size_t MAX_TRACK_TITLE_LENGTH = 64;
constexpr size_t BLUETOOTH_MAC_LENGTH = 24;
constexpr char FIRMWARE_VERSION[] = "1.0.0-phase-integrated";
}
namespace Resources {
constexpr UBaseType_t AUDIO_COMMAND_QUEUE_LENGTH = 16;
constexpr UBaseType_t UI_TELEMETRY_QUEUE_LENGTH = 4;
constexpr UBaseType_t BUTTON_EVENT_QUEUE_LENGTH = 16;
constexpr UBaseType_t BLUETOOTH_EVENT_QUEUE_LENGTH = 8;
constexpr size_t AUDIO_RING_BUFFER_CAPACITY = 32768;
}

enum class EqFilterType : uint8_t { LOW_SHELF, PEAKING, HIGH_SHELF };
enum class AudioPreset : uint8_t { FLAT, ACOUSTIC, BASS_BOOST, VOCAL_ENHANCE, CUSTOM, COUNT };
enum class AudioCommandType : uint8_t { APPLY_SETTINGS, RESET_EQ, SET_EQ_ENABLED, SET_EQ_GAIN, SET_LIMITER_ENABLED, SET_MASTER_VOLUME, SET_MUTE, SET_PRESET };
enum class ButtonId : uint8_t { UP, DOWN, LEFT, RIGHT, CENTER, BACK, VOL_UP, VOL_DOWN, MUTE, EQ_TOGGLE };
enum class ButtonEventType : uint8_t { PRESSED, RELEASED, SHORT_PRESS, LONG_PRESS };
enum class BluetoothEventType : uint8_t { CONNECTED, DISCONNECTED, TRACK_METADATA_UPDATED };
enum class UiScreen : uint8_t { BOOT, DASHBOARD, EQUALIZER, PRESET_MANAGER, SYSTEM_INFO };

struct EqBandDefinition { EqFilterType filterType; float frequencyHz; float q; };
static constexpr EqBandDefinition DEFAULT_EQ_BANDS[Config::EQ_BAND_COUNT] = {
    {EqFilterType::PEAKING, 25.0f, 0.90f},
    {EqFilterType::PEAKING, 45.0f, 0.90f},
    {EqFilterType::PEAKING, 60.0f, 1.414f},
    {EqFilterType::PEAKING, 150.0f, 1.414f},
    {EqFilterType::PEAKING, 400.0f, 1.414f},
    {EqFilterType::PEAKING, 1000.0f, 1.414f},
    {EqFilterType::PEAKING, 2500.0f, 1.414f},
    {EqFilterType::PEAKING, 6300.0f, 1.414f},
    {EqFilterType::PEAKING, 16000.0f, 1.414f}
};

struct BiquadChannelState { float x1=0, x2=0, y1=0, y2=0; };
struct SystemSettings {
    float eqGainDb[Config::EQ_BAND_COUNT] = {0,0,0,0,0,0,0,0,0};
    uint8_t masterVolume = 70;
    bool muted = false;
    bool eqEnabled = true;
    bool limiterEnabled = true;
    AudioPreset activePreset = AudioPreset::FLAT;
};
struct AudioCommand {
    AudioCommandType type = AudioCommandType::APPLY_SETTINGS;
    uint8_t bandIndex = 0;
    float gainDb = 0.0f;
    uint8_t volume = 70;
    bool enabled = false;
    AudioPreset preset = AudioPreset::FLAT;
    SystemSettings settings{};
};
struct ButtonEvent {
    ButtonId id = ButtonId::CENTER;
    ButtonEventType type = ButtonEventType::PRESSED;
    uint32_t timestampMs = 0;
    uint32_t pressDurationMs = 0;
};
struct BluetoothTrackMetadata {
    char title[Config::MAX_TRACK_TITLE_LENGTH] = {0};
    char artist[Config::MAX_TRACK_TITLE_LENGTH] = {0};
    char album[Config::MAX_TRACK_TITLE_LENGTH] = {0};
};
struct BluetoothEvent {
    BluetoothEventType type = BluetoothEventType::DISCONNECTED;
    bool connected = false;
    BluetoothTrackMetadata metadata{};
};
struct AudioTelemetry {
    float rmsLeft = 0, rmsRight = 0, inputPeak = 0, outputPeak = 0;
    bool limiterActive = false;
    uint32_t processedFrames = 0, underrunCount = 0, droppedBlockCount = 0, overflowCount = 0;
    uint32_t sampleRate = Config::AUDIO_SAMPLE_RATE;
    bool streamActive = false;
};
struct UiTelemetry {
    AudioTelemetry audio{};
    float batteryVoltage = 0.0f;
    uint8_t batteryPercent = 0;
    bool bluetoothConnected = false;
    uint8_t masterVolume = 70;
    bool muted = false;
    bool eqEnabled = true;
    bool limiterEnabled = true;
    AudioPreset activePreset = AudioPreset::FLAT;
};
struct UiState {
    UiScreen currentScreen = UiScreen::BOOT;
    UiScreen previousScreen = UiScreen::BOOT;
    bool screenDirty = true;
    uint8_t selectedEqBand = 0;
};
} // namespace HorizonDLMS

/* Shared RTOS objects are global to match the existing phase source files. */
extern QueueHandle_t g_audioCommandQueue;
extern QueueHandle_t g_uiTelemetryQueue;
extern QueueHandle_t g_buttonEventQueue;
extern QueueHandle_t g_bluetoothEventQueue;
extern RingbufHandle_t g_audioPcmRingBuffer;
extern SemaphoreHandle_t g_settingsMutex;
extern SemaphoreHandle_t g_displayMutex;
extern TaskHandle_t g_audioTaskHandle;
extern TaskHandle_t g_uiTaskHandle;
extern TaskHandle_t g_inputTaskHandle;
extern TaskHandle_t g_telemetryTaskHandle;
