#include "audio_engine.h"
#include "resampler.h"  // Fixed 48 kHz output, resampling source PCM as in the reference.
#include <algorithm>
#include <atomic>  // Safe cross-task publication and callback producer guard.
#include <cmath>
#include <cstring>
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

namespace {
using namespace Horizon;
constexpr uint32_t kRingFrameCount = 8192;  // power of two; ~186 ms at 44.1 kHz
constexpr uint32_t kRingMask = kRingFrameCount - 1;
constexpr uint16_t kBlockFrames = 128;
// Inputs may be 16/32/44.1/48 kHz; the resampler supports up to 3x upsampling.
constexpr uint16_t kMaxOutputFrames = kBlockFrames * 3U + 8U;
static StereoResampler s_resampler;  // Owned exclusively by AudioTask.
// Static audio work buffers avoid adding several KB to the FreeRTOS task stack.
static int16_t s_resampledPcm[kMaxOutputFrames * 2];
static int16_t s_outputPcm[kMaxOutputFrames * 2];
static float s_processedPcm[kMaxOutputFrames * 2];
constexpr uint8_t kPresetCount = 5;
constexpr uint32_t kEqTransitionMs = 25;
constexpr uint32_t kLowEqTransitionMs = 25;  // Match reference EQ smoothing target.
constexpr uint32_t kPcmGapResetMs = 35;  // Count only gaps during an active playback session.
constexpr uint32_t kPcmStreamRestartMs = 1200;  // Match streamActive timeout; long idle is not a dropout.
constexpr uint32_t kResumeFadeMs = 12;
constexpr float kSoftClipKnee = 0.90f;  // Reference-style per-sample soft clip knee.
constexpr float kPi = 3.14159265358979323846f;

const float kPresetGains[kPresetCount][kBandCount] = {
    // FLAT
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    // Reference-style bass preset: +14 / +12 / +8 / +4 dB in the sub/low bands.
    {14, 12, 8, 4, 0, 0, 0, 0, 0},
    // Vocal/clarity preset: presence-focused, with modest high-mid/treble lift.
    {0, 0, 0, 0, 0, 4, 6, 4, 0},
    // CUSTOM 1 defaults: old custom curve shifted right to make room for 25 Hz.
    {0, 3, 2, 0, 0, 0, 0, 0, 0},
    // CUSTOM 2 defaults: old custom curve shifted right to make room for 25 Hz.
    {0, 0, 1, 2, -1, 0, 1, 2, 1}};

enum class CommandType : uint8_t { SetBand, AdjustBand, SetPreset, SetVolume };
struct AudioCommand {
  CommandType type;
  uint8_t index;
  float value;
};
struct BiquadCoefficients { float b0, b1, b2, a1, a2; };
struct BiquadState { float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f; };

// Single-producer (A2DP callback), single-consumer (AudioTask) PCM ring.
// All counters and copies are guarded briefly, and frames are kept stereo-aligned.
static int16_t s_pcmRing[kRingFrameCount * 2];
static uint32_t s_head = 0;
static uint32_t s_tail = 0;
static portMUX_TYPE s_ringMux = portMUX_INITIALIZER_UNLOCKED;
// The queue is never deleted after it is published. Atomic publication keeps
// early UI/input calls safe while AudioEngine_Begin() is still initializing.
static std::atomic<QueueHandle_t> s_commandQueue{nullptr};
static TaskHandle_t s_audioTaskHandle = nullptr;
// Callback is the sole PCM producer. A guard rejects an unexpected re-entrant
// callback instead of allowing two writers to reserve the same ring region.
static std::atomic<bool> s_pcmProducerBusy{false};
static std::atomic<bool> s_pcmWritePaused{false};
static std::atomic<bool> s_rateSwitchPending{false};
static uint32_t s_ringEpoch = 0;  // Invalidates in-flight copies across flush/rate changes.
static uint32_t s_sourceRate = 44100U;  // Typical initial A2DP source rate; output remains 48 kHz.
static uint32_t s_rateGeneration = 0;
static portMUX_TYPE s_rateMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_lastPacketMs = 0;
static uint32_t s_droppedPackets = 0;
static uint32_t s_i2sWriteFaults = 0;
static uint32_t s_clippedFrames = 0;
static uint32_t s_dspFaults = 0;
static uint32_t s_pcmGapEvents = 0;
static uint32_t s_pcmGapMaxMs = 0;
static uint32_t s_invalidPcmPackets = 0;
static uint32_t s_commandQueueDrops = 0;
static bool s_filterResetPending = false;
static bool s_i2sOperational = true;  // Owned by AudioTask.
static portMUX_TYPE s_snapshotMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_diagMux = portMUX_INITIALIZER_UNLOCKED;
static AudioSnapshot s_snapshot = {{0}, 0, 70, false, false, kDefaultSampleRate,
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false};
static float s_loadedCustomGains[2][kBandCount] = {
    {0, 3, 2, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 2, -1, 0, 1, 2, 1}};

struct EngineState {
  float targetGain[kBandCount] = {0};
  BiquadCoefficients coeff[kBandCount]{};
  BiquadCoefficients targetCoeff[kBandCount]{};
  float coefficientTargetGain[kBandCount] = {0};
  // No auto pre-gain or block-wide limiter gain: EQ boosts do not duck the full block.
  // Per-sample soft clipping is applied after EQ/volume as in the supplied reference.
  uint32_t transitionRemaining[kBandCount] = {0};
  uint32_t transitionTotalSamples[kBandCount] = {0};
  BiquadState filter[2][kBandCount]{};
  BiquadState transitionFilter[2][kBandCount]{};
  uint8_t preset = 0;
  float customGains[2][kBandCount]{};
  uint8_t volume = 70;
  float appliedVolume = 0.70f;
  float resumeFadeGain = 1.0f;
  float resumeFadeStep = 0.0f;
  uint32_t resumeFadeRemaining = 0;
  uint32_t sampleRate = kDefaultSampleRate;
};

void calculateCoefficients(uint8_t band, float gainDb, uint32_t sampleRate,
                           BiquadCoefficients* c) {
  const float fs = static_cast<float>(std::max<uint32_t>(sampleRate, 8000));
  const float f = std::min(kBandFrequencies[band], fs * 0.45f);
  const float A = powf(10.0f, gainDb / 40.0f);
  const float omega = 2.0f * kPi * f / fs;
  const float sn = sinf(omega);
  const float cs = cosf(omega);
  const float alpha = sn / (2.0f * kBandQ[band]);
  const float a0 = 1.0f + alpha / A;
  c->b0 = (1.0f + alpha * A) / a0;
  c->b1 = (-2.0f * cs) / a0;
  c->b2 = (1.0f - alpha * A) / a0;
  c->a1 = (-2.0f * cs) / a0;
  c->a2 = (1.0f - alpha / A) / a0;
}

void startCoefficientTransition(EngineState* state, uint8_t band) {
  calculateCoefficients(band, state->targetGain[band], state->sampleRate,
                        &state->targetCoeff[band]);
  state->coefficientTargetGain[band] = state->targetGain[band];
  const uint32_t transitionMs = band <= 1 ? kLowEqTransitionMs : kEqTransitionMs;
  const uint32_t transitionSamples = std::max<uint32_t>(
      1, (state->sampleRate * transitionMs) / 1000U);
  state->transitionTotalSamples[band] = transitionSamples;
  state->transitionRemaining[band] = transitionSamples;
  for (uint8_t channel = 0; channel < 2; ++channel)
    state->transitionFilter[channel][band] = BiquadState{};
}

inline float processBiquad(float x, const BiquadCoefficients& c, BiquadState* s) {
  const float y = c.b0 * x + c.b1 * s->x1 + c.b2 * s->x2 -
                  c.a1 * s->y1 - c.a2 * s->y2;
  if (!std::isfinite(y)) {
    *s = BiquadState{};
    ++s_dspFaults;
    return 0.0f;
  }
  s->x2 = s->x1;
  s->x1 = x;
  s->y2 = s->y1;
  s->y1 = y;
  return y;
}

inline float softClip16(float sample) {
  // Reference-style soft knee in normalized float units. Unlike a block-peak
  // limiter, this does not apply one attenuating gain to every sample in a block.
  const float normalized = sample / 32768.0f;
  const float magnitude = fabsf(normalized);
  if (magnitude <= kSoftClipKnee) return sample;
  const float compressed = kSoftClipKnee + (1.0f - kSoftClipKnee) *
      tanhf((magnitude - kSoftClipKnee) / (1.0f - kSoftClipKnee));
  return copysignf(compressed * 32768.0f, normalized);
}

inline int16_t saturate16(float x) {
  if (!std::isfinite(x)) return 0;
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return static_cast<int16_t>(x);
}

bool enqueue(const AudioCommand& command) {
  // The queue remains alive after publication; the acquire load pairs with
  // AudioEngine_Begin()'s release store. Before publication, reject cleanly.
  QueueHandle_t queue = s_commandQueue.load(std::memory_order_acquire);
  if (queue == nullptr) return false;
  const bool queued = xQueueSend(queue, &command, 0) == pdTRUE;
  if (!queued) {
    portENTER_CRITICAL(&s_diagMux);
    ++s_commandQueueDrops;
    portEXIT_CRITICAL(&s_diagMux);
  }
  return queued;
}

void countInvalidPcmPacket() {
  portENTER_CRITICAL(&s_diagMux);
  ++s_invalidPcmPackets;
  portEXIT_CRITICAL(&s_diagMux);
}

void publishSnapshot(const EngineState& state, uint16_t peakLeft, uint16_t peakRight) {
  uint32_t lastPacketMs = 0;
  uint32_t droppedPackets = 0;
  uint32_t pcmGapEvents = 0;
  uint32_t pcmGapMaxMs = 0;
  portENTER_CRITICAL(&s_ringMux);
  lastPacketMs = s_lastPacketMs;
  droppedPackets = s_droppedPackets;
  pcmGapEvents = s_pcmGapEvents;
  pcmGapMaxMs = s_pcmGapMaxMs;
  portEXIT_CRITICAL(&s_ringMux);

  uint32_t invalidPcmPackets = 0;
  uint32_t commandQueueDrops = 0;
  portENTER_CRITICAL(&s_diagMux);
  invalidPcmPackets = s_invalidPcmPackets;
  commandQueueDrops = s_commandQueueDrops;
  portEXIT_CRITICAL(&s_diagMux);

  const uint32_t now = millis();
  portENTER_CRITICAL(&s_snapshotMux);
  for (uint8_t i = 0; i < kBandCount; ++i) s_snapshot.bandDb[i] = state.targetGain[i];
  s_snapshot.preset = state.preset;
  s_snapshot.volume = state.volume;
  s_snapshot.muted = false;
  s_snapshot.sampleRate = state.sampleRate;
  // Keep the largest block peak until the UI consumes it; empty-buffer
  // updates must not erase a transient before the next display frame.
  s_snapshot.peakLeft = std::max(s_snapshot.peakLeft, peakLeft);
  s_snapshot.peakRight = std::max(s_snapshot.peakRight, peakRight);
  s_snapshot.streamActive = lastPacketMs != 0 && (now - lastPacketMs) < 1200;
  s_snapshot.droppedPackets = droppedPackets;
  s_snapshot.i2sWriteFaults = s_i2sWriteFaults;
  s_snapshot.clippedFrames = s_clippedFrames;
  s_snapshot.dspFaults = s_dspFaults;
  s_snapshot.pcmGapEvents = pcmGapEvents;
  s_snapshot.pcmGapMaxMs = pcmGapMaxMs;
  s_snapshot.invalidPcmPackets = invalidPcmPackets;
  s_snapshot.commandQueueDrops = commandQueueDrops;
  portEXIT_CRITICAL(&s_snapshotMux);
}

bool installI2S(uint32_t sampleRate) {
  i2s_config_t config{};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = sampleRate;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = 0;
  config.dma_buf_count = 12;
  config.dma_buf_len = 512;
  config.use_apll = true;
  config.tx_desc_auto_clear = true;
  config.fixed_mclk = 0;

  const esp_err_t installResult = i2s_driver_install(I2S_NUM_1, &config, 0, nullptr);
  if (installResult != ESP_OK) {
    DEBUG_PRINTF("[AUDIO] i2s_driver_install failed: %d\n", static_cast<int>(installResult));
    return false;
  }
  i2s_pin_config_t pins{};
  pins.bck_io_num = kPinI2sBck;
  pins.ws_io_num = kPinI2sLrck;
  pins.data_out_num = kPinI2sData;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  const esp_err_t pinResult = i2s_set_pin(I2S_NUM_1, &pins);
  if (pinResult != ESP_OK) {
    DEBUG_PRINTF("[AUDIO] i2s_set_pin failed: %d\n", static_cast<int>(pinResult));
    i2s_driver_uninstall(I2S_NUM_1);
    return false;
  }
  const esp_err_t zeroResult = i2s_zero_dma_buffer(I2S_NUM_1);
  if (zeroResult != ESP_OK) {
    DEBUG_PRINTF("[AUDIO] i2s_zero_dma_buffer failed: %d\n", static_cast<int>(zeroResult));
    i2s_driver_uninstall(I2S_NUM_1);
    return false;
  }
  return true;
}

// Reset the legacy I2S driver after an alignment fault or failed transfer.
// The fallback fully reinstalls the driver if stop/clock/start does not work.
bool recoverI2S(uint32_t sampleRate) {
  const esp_err_t stopResult = i2s_stop(I2S_NUM_1);
  if (stopResult == ESP_OK) {
    const esp_err_t zeroResult = i2s_zero_dma_buffer(I2S_NUM_1);
    const esp_err_t clockResult = i2s_set_clk(
        I2S_NUM_1, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
    const esp_err_t startResult = i2s_start(I2S_NUM_1);
    if (zeroResult == ESP_OK && clockResult == ESP_OK && startResult == ESP_OK) {
      return true;
    }
    DEBUG_PRINTF("[AUDIO] I2S recovery restart failed: zero=%d clk=%d start=%d\n",
                 static_cast<int>(zeroResult), static_cast<int>(clockResult),
                 static_cast<int>(startResult));
  } else {
    DEBUG_PRINTF("[AUDIO] i2s_stop during recovery returned %d; trying reinstall.\n",
                 static_cast<int>(stopResult));
  }

  const esp_err_t uninstallResult = i2s_driver_uninstall(I2S_NUM_1);
  if (uninstallResult != ESP_OK) {
    DEBUG_PRINTF("[AUDIO] I2S driver uninstall during recovery failed: %d\n",
                 static_cast<int>(uninstallResult));
    return false;
  }
  return installI2S(sampleRate);
}

void flushPcmRingFromAudioTask() {
  // Pause new producers first, then invalidate any copy already in flight.
  // The callback never waits for this task and its memcpy is outside spinlocks.
  s_pcmWritePaused.store(true, std::memory_order_release);
  portENTER_CRITICAL(&s_ringMux);
  ++s_ringEpoch;
  portEXIT_CRITICAL(&s_ringMux);

  while (s_pcmProducerBusy.load(std::memory_order_acquire)) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  portENTER_CRITICAL(&s_ringMux);
  s_tail = s_head;
  s_lastPacketMs = 0;
  s_filterResetPending = false;
  ++s_ringEpoch;
  portEXIT_CRITICAL(&s_ringMux);
}

void beginResumeFade(EngineState* state) {
  state->resumeFadeGain = 0.0f;
  state->resumeFadeRemaining = std::max<uint32_t>(
      1, (state->sampleRate * kResumeFadeMs) / 1000U);
  state->resumeFadeStep = 1.0f / static_cast<float>(state->resumeFadeRemaining);
}

void resetBiquadStates(EngineState* state) {
  for (uint8_t channel = 0; channel < 2; ++channel) {
    for (uint8_t band = 0; band < kBandCount; ++band) {
      state->filter[channel][band] = BiquadState{};
      state->transitionFilter[channel][band] = BiquadState{};
    }
  }
}

void resetFilterPipelineForRate(EngineState* state, uint32_t sampleRate) {
  state->sampleRate = sampleRate;
  for (uint8_t band = 0; band < kBandCount; ++band) {
    calculateCoefficients(band, state->targetGain[band], sampleRate,
                          &state->coeff[band]);
    state->targetCoeff[band] = state->coeff[band];
    state->coefficientTargetGain[band] = state->targetGain[band];
    state->transitionRemaining[band] = 0;
    state->transitionTotalSamples[band] = 0;
  }
  resetBiquadStates(state);
  beginResumeFade(state);
}

void applyCommands(EngineState* state, QueueHandle_t commandQueue) {
  AudioCommand command{};
  while (commandQueue != nullptr && xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
    switch (command.type) {
      case CommandType::SetBand:
      case CommandType::AdjustBand:
        if (command.index < kBandCount) {
          // Editing a factory preset branches to CUSTOM 1 by copying the
          // complete active curve first. Editing CUSTOM 1/2 stays in that slot.
          if (state->preset != 3 && state->preset != 4) {
            for (uint8_t i = 0; i < kBandCount; ++i)
              state->customGains[0][i] = state->targetGain[i];
            state->preset = 3;
          }
          const float requestedGain = command.type == CommandType::AdjustBand
              ? state->targetGain[command.index] + command.value
              : command.value;
          state->targetGain[command.index] = constrain(requestedGain, -18.0f, 18.0f);
          if (state->preset == 3 || state->preset == 4) {
            const uint8_t slot = state->preset - 3;
            state->customGains[slot][command.index] = state->targetGain[command.index];
          }
        }
        break;
      case CommandType::SetPreset:
        if (command.index < kPresetCount) {
          state->preset = command.index;
          for (uint8_t i = 0; i < kBandCount; ++i) {
            state->targetGain[i] = (command.index >= 3)
                ? state->customGains[command.index - 3][i]
                : kPresetGains[command.index][i];
          }
        }
        break;
      case CommandType::SetVolume:
        state->volume = static_cast<uint8_t>(constrain(static_cast<int>(command.value), 0, 100));
        break;
    }
  }
}

// Returns true only when the requested rate has been applied and the PCM
// path can resume. Rate work is deferred out of the Bluetooth callback context.
bool servicePendingSampleRate(EngineState* state, uint32_t* retryAtMs) {
  if (!s_rateSwitchPending.load(std::memory_order_acquire)) return true;
  if (s_pcmProducerBusy.load(std::memory_order_acquire)) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    return false;
  }

  const uint32_t now = millis();
  if (static_cast<int32_t>(now - *retryAtMs) < 0) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
    return false;
  }

  uint32_t requestedRate = kDefaultSampleRate;
  uint32_t generation = 0;
  portENTER_CRITICAL(&s_rateMux);
  requestedRate = s_sourceRate;
  generation = s_rateGeneration;
  portEXIT_CRITICAL(&s_rateMux);

  // Keep input paused while discarding old-rate PCM and reconfiguring the
  // streaming resampler. I2S/DAC remains clocked at 48 kHz at all times.
  flushPcmRingFromAudioTask();
  esp_err_t configResult = ESP_OK;
  if (!s_i2sOperational) {
    s_i2sOperational = recoverI2S(kDefaultSampleRate);
    if (!s_i2sOperational) configResult = ESP_FAIL;
  }
  if (configResult == ESP_OK &&
      !s_resampler.configure(requestedRate, kDefaultSampleRate)) {
    configResult = ESP_ERR_INVALID_ARG;
  }

  if (configResult != ESP_OK) {
    *retryAtMs = millis() + 500;
    DEBUG_PRINTF("[AUDIO] resampler configure %lu -> %lu Hz failed: %d; retrying.\n",
                 static_cast<unsigned long>(requestedRate),
                 static_cast<unsigned long>(kDefaultSampleRate),
                 static_cast<int>(configResult));
    return false;
  }

  resetFilterPipelineForRate(state, kDefaultSampleRate);
  *retryAtMs = 0;

  // Clear the pending flag only if no newer sample-rate event arrived while
  // I2S was being changed. This prevents an older switch from unpausing PCM.
  portENTER_CRITICAL(&s_rateMux);
  if (s_rateGeneration == generation) {
    s_rateSwitchPending.store(false, std::memory_order_release);
    s_pcmWritePaused.store(false, std::memory_order_release);
  }
  portEXIT_CRITICAL(&s_rateMux);
  return !s_rateSwitchPending.load(std::memory_order_acquire);
}

void audioTask(void* parameter) {
  // This task owns its queue handle for life; other tasks only see the
  // atomically published handle once task creation succeeds.
  QueueHandle_t commandQueue = static_cast<QueueHandle_t>(parameter);
  EngineState state;
  for (uint8_t slot = 0; slot < 2; ++slot) {
    for (uint8_t band = 0; band < kBandCount; ++band) {
      state.customGains[slot][band] = constrain(s_loadedCustomGains[slot][band], -18.0f, 18.0f);
    }
  }
  for (uint8_t i = 0; i < kBandCount; ++i) {
    calculateCoefficients(i, 0.0f, state.sampleRate, &state.coeff[i]);
    state.targetCoeff[i] = state.coeff[i];
  }
  // The resampler is task-owned and produces a fixed 48 kHz stream.
  if (!s_resampler.configure(s_sourceRate, kDefaultSampleRate)) {
    DEBUG_PRINTLN("[AUDIO] initial resampler configuration failed; retry will be requested.");
    s_rateSwitchPending.store(true, std::memory_order_release);
    s_pcmWritePaused.store(true, std::memory_order_release);
  }
  int16_t inputBlock[kBlockFrames * 2];
  uint32_t nextRateRetryMs = 0;
  uint32_t nextI2sRecoveryMs = 0;
  TickType_t lastIdleTelemetry = xTaskGetTickCount();
  TickType_t lastAudioTelemetry = lastIdleTelemetry;
  uint32_t audioFramesSinceTelemetry = 0;
  uint32_t telemetryPeakLeft = 0;
  uint32_t telemetryPeakRight = 0;

  for (;;) {
    applyCommands(&state, commandQueue);

    if (s_rateSwitchPending.load(std::memory_order_acquire)) {
      if (!servicePendingSampleRate(&state, &nextRateRetryMs)) continue;
    }

    // A failed I2S transfer leaves the PCM writer paused until a bounded retry
    // has successfully reset/reinstalled the output driver.
    if (!s_i2sOperational) {
      const uint32_t now = millis();
      if (static_cast<int32_t>(now - nextI2sRecoveryMs) < 0) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        continue;
      }
      nextI2sRecoveryMs = now + 1000;
      flushPcmRingFromAudioTask();
      if (recoverI2S(state.sampleRate)) {
        s_i2sOperational = true;
        resetBiquadStates(&state);
        beginResumeFade(&state);
        s_resampler.reset();
        if (!s_rateSwitchPending.load(std::memory_order_acquire)) {
          s_pcmWritePaused.store(false, std::memory_order_release);
        }
      } else {
        DEBUG_PRINTLN("[AUDIO] I2S recovery failed; retrying in 1 second.");
        continue;
      }
    }

    // Fail-safe: no normal path should leave output paused once I2S is healthy
    // and there is no pending sample-rate switch. This cannot override a rate
    // switch or an active I2S recovery, but prevents a stale software pause
    // flag from silently blocking all future PCM callbacks.
    if (s_i2sOperational &&
        !s_rateSwitchPending.load(std::memory_order_acquire) &&
        s_pcmWritePaused.exchange(false, std::memory_order_acq_rel)) {
      DEBUG_PRINTLN("[AUDIO] Cleared stale PCM pause; I2S is operational.");
    }

    // Recompute destinations only when an EQ target changes. The audio loop
    // interpolates coefficients per sample to avoid block-boundary zipper noise.
    for (uint8_t band = 0; band < kBandCount; ++band) {
      if (state.transitionRemaining[band] == 0 &&
          fabsf(state.targetGain[band] - state.coefficientTargetGain[band]) > 0.001f) {
        startCoefficientTransition(&state, band);
      }
    }
    uint32_t frames = 0;
    bool resetFilters = false;
    uint32_t readTail = 0;
    uint32_t readEpoch = 0;
    portENTER_CRITICAL(&s_ringMux);
    resetFilters = s_filterResetPending;
    s_filterResetPending = false;
    readTail = s_tail;
    readEpoch = s_ringEpoch;
    const uint32_t available = (s_head - s_tail) & kRingMask;
    // Do not gate audio on a startup/idle priming state. The previous reserve
    // could leave a quiet/low-rate callback pattern waiting indefinitely for
    // a threshold. Process every available complete PCM frame immediately;
    // ordinary I2S DMA buffering and the resume fade handle start-up smoothing.
    frames = std::min<uint32_t>(available, kBlockFrames);
    portEXIT_CRITICAL(&s_ringMux);

    // The consumer copies outside the spinlock too. s_tail is not advanced
    // until the copy finishes, so the producer cannot overwrite these frames.
    if (frames > 0) {
      const uint32_t firstFrames = std::min<uint32_t>(frames, kRingFrameCount - readTail);
      memcpy(inputBlock, s_pcmRing + readTail * 2, firstFrames * 4);
      const uint32_t remainingFrames = frames - firstFrames;
      if (remainingFrames > 0) {
        memcpy(inputBlock + firstFrames * 2, s_pcmRing, remainingFrames * 4);
      }
    }

    portENTER_CRITICAL(&s_ringMux);
    if (readEpoch != s_ringEpoch ||
        s_rateSwitchPending.load(std::memory_order_acquire) ||
        s_pcmWritePaused.load(std::memory_order_acquire)) {
      // A rate switch/flush invalidated this snapshot. Do not publish tail.
      frames = 0;
      resetFilters = false;
    } else if (frames > 0 && s_tail == readTail) {
      s_tail = (readTail + frames) & kRingMask;
    } else if (frames > 0) {
      // Unexpected consumer cursor change: discard this copied block safely.
      frames = 0;
    }
    portEXIT_CRITICAL(&s_ringMux);

    if (resetFilters) {
      resetBiquadStates(&state);
      s_resampler.reset();
      beginResumeFade(&state);
    }

    if (frames == 0) {
      publishSnapshot(state, 0, 0);
      if (xTaskGetTickCount() - lastIdleTelemetry >= pdMS_TO_TICKS(1000)) {
        lastIdleTelemetry = xTaskGetTickCount();
#if ENABLE_DEBUG
        AudioSnapshot telemetry{};
        AudioEngine_GetSnapshot(&telemetry);
        DEBUG_PRINTF("[AUDIO] waiting-for-PCM stream=%u bt=%u drop=%lu invalid=%lu i2s=%lu heap=%lu\n",
                     telemetry.streamActive ? 1U : 0U,
                     telemetry.bluetoothConnected ? 1U : 0U,
                     static_cast<unsigned long>(telemetry.droppedPackets),
                     static_cast<unsigned long>(telemetry.invalidPcmPackets),
                     static_cast<unsigned long>(telemetry.i2sWriteFaults),
                     static_cast<unsigned long>(ESP.getFreeHeap()));
#endif
      }
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
      continue;
    }

    // Resample source-rate PCM to the fixed 48 kHz output before running the
    // EQ. This mirrors the supplied reference's output-clock strategy.
    const int16_t* samplesToProcess = inputBlock;
    uint32_t processingFrames = frames;
    if (!s_resampler.passthrough()) {
      processingFrames = static_cast<uint32_t>(
          s_resampler.process(inputBlock, frames, s_resampledPcm));
      samplesToProcess = s_resampledPcm;
    }
    if (processingFrames == 0) {
      publishSnapshot(state, 0, 0);
      continue;  // Valid resampler history can consume input without emitting a frame.
    }

    const float targetVolume = static_cast<float>(state.volume) / 100.0f;
    const float volumeStep = 1.0f / std::max(1.0f, state.sampleRate * 0.030f);
    uint32_t maxL = 0, maxR = 0;
    uint32_t softClipFrames = 0;

    for (uint32_t frame = 0; frame < processingFrames; ++frame) {
      // Keep full-scale PCM entering the EQ. Do not auto-attenuate the whole
      // program when one band is boosted; master volume remains its own control.
      float left = static_cast<float>(samplesToProcess[frame * 2]);
      float right = static_cast<float>(samplesToProcess[frame * 2 + 1]);

      for (uint8_t band = 0; band < kBandCount; ++band) {
        const uint32_t remaining = state.transitionRemaining[band];
        if (remaining > 0) {
          const float mixProgress = static_cast<float>(
              state.transitionTotalSamples[band] - remaining) /
              static_cast<float>(state.transitionTotalSamples[band]);
          const float mix = mixProgress * mixProgress * (3.0f - 2.0f * mixProgress);
          const float oldLeft = processBiquad(left, state.coeff[band],
                                              &state.filter[0][band]);
          const float newLeft = processBiquad(left, state.targetCoeff[band],
                                              &state.transitionFilter[0][band]);
          const float oldRight = processBiquad(right, state.coeff[band],
                                               &state.filter[1][band]);
          const float newRight = processBiquad(right, state.targetCoeff[band],
                                               &state.transitionFilter[1][band]);
          left = oldLeft + (newLeft - oldLeft) * mix;
          right = oldRight + (newRight - oldRight) * mix;
          if (--state.transitionRemaining[band] == 0) {
            state.coeff[band] = state.targetCoeff[band];
            state.filter[0][band] = state.transitionFilter[0][band];
            state.filter[1][band] = state.transitionFilter[1][band];
          }
        } else {
          left = processBiquad(left, state.coeff[band], &state.filter[0][band]);
          right = processBiquad(right, state.coeff[band], &state.filter[1][band]);
        }
      }

      const float volumeDiff = targetVolume - state.appliedVolume;
      state.appliedVolume += constrain(volumeDiff, -volumeStep, volumeStep);
      left *= state.appliedVolume;
      right *= state.appliedVolume;
      if (state.resumeFadeRemaining > 0) {
        state.resumeFadeGain = std::min(1.0f, state.resumeFadeGain + state.resumeFadeStep);
        if (--state.resumeFadeRemaining == 0) state.resumeFadeGain = 1.0f;
      }
      left *= state.resumeFadeGain;
      right *= state.resumeFadeGain;

      if (!std::isfinite(left) || !std::isfinite(right)) {
        left = 0.0f;
        right = 0.0f;
        ++s_dspFaults;
      }
      const float absLeft = fabsf(left);
      const float absRight = fabsf(right);
      if (absLeft > kSoftClipKnee * 32768.0f ||
          absRight > kSoftClipKnee * 32768.0f) {
        ++softClipFrames;
      }
      // Soft-limit each sample independently. This avoids the previous behavior
      // where one transient peak reduced every sample in the entire block.
      s_processedPcm[frame * 2] = softClip16(left);
      s_processedPcm[frame * 2 + 1] = softClip16(right);
    }
    s_clippedFrames += softClipFrames;

    for (uint32_t frame = 0; frame < processingFrames; ++frame) {
      s_outputPcm[frame * 2] = saturate16(s_processedPcm[frame * 2]);
      s_outputPcm[frame * 2 + 1] = saturate16(s_processedPcm[frame * 2 + 1]);
      const uint32_t absL = static_cast<uint32_t>(abs(static_cast<int32_t>(s_outputPcm[frame * 2])));
      const uint32_t absR = static_cast<uint32_t>(abs(static_cast<int32_t>(s_outputPcm[frame * 2 + 1])));
      maxL = std::max(maxL, absL);
      maxR = std::max(maxR, absR);
    }

    const size_t bytesRequired = processingFrames * 4;
    size_t bytesSent = 0;
    bool alignmentFault = false;
    bool writeError = false;
    for (uint8_t attempt = 0; attempt < 4 && bytesSent < bytesRequired; ++attempt) {
      size_t justSent = 0;
      const size_t remainingBytes = bytesRequired - bytesSent;
      const char* remaining = reinterpret_cast<const char*>(s_outputPcm) + bytesSent;
      const esp_err_t writeResult = i2s_write(
          I2S_NUM_1, remaining, remainingBytes, &justSent, pdMS_TO_TICKS(10));

      // Legacy I2S writes stereo 16-bit frames: all partial byte counts must
      // remain on a 4-byte frame boundary. Never continue from a split frame.
      if (justSent > remainingBytes || (justSent & 0x03U) != 0U) {
        alignmentFault = true;
        writeError = true;
        DEBUG_PRINTF("[AUDIO] I2S returned invalid partial frame (%u bytes); recovering.\n",
                     static_cast<unsigned>(justSent));
        break;
      }
      bytesSent += justSent;
      if (writeResult != ESP_OK) {
        writeError = true;
        DEBUG_PRINTF("[AUDIO] i2s_write failed: %d\n", static_cast<int>(writeResult));
        break;
      }
      if (justSent == 0) {
        writeError = true;
        break;
      }
    }

    if (bytesSent != bytesRequired || writeError) {
      ++s_i2sWriteFaults;
      if (alignmentFault || writeError) {
        // Do not replay a partly transmitted block: that could duplicate data.
        // Flush stale PCM, reset the driver, and fade in when it is healthy.
        flushPcmRingFromAudioTask();
        if (recoverI2S(state.sampleRate)) {
          s_i2sOperational = true;
          resetBiquadStates(&state);
          beginResumeFade(&state);
          s_resampler.reset();
          if (!s_rateSwitchPending.load(std::memory_order_acquire)) {
            s_pcmWritePaused.store(false, std::memory_order_release);
          }
        } else {
          s_i2sOperational = false;
          nextI2sRecoveryMs = millis() + 1000;
          DEBUG_PRINTLN("[AUDIO] I2S recovery failed; PCM remains paused until retry.");
        }
      }
      if (bytesSent != bytesRequired) {
        // The untransmitted suffix of this output block is deliberately discarded.
        // The next output block remains frame-aligned because only whole frames were sent.
      }
    }

    const uint16_t peakL = static_cast<uint16_t>(std::min<uint32_t>(32768U, maxL));
    const uint16_t peakR = static_cast<uint16_t>(std::min<uint32_t>(32768U, maxR));
    publishSnapshot(state, peakL, peakR);
    audioFramesSinceTelemetry += processingFrames;
    telemetryPeakLeft = std::max(telemetryPeakLeft, maxL);
    telemetryPeakRight = std::max(telemetryPeakRight, maxR);

    if (xTaskGetTickCount() - lastAudioTelemetry >= pdMS_TO_TICKS(1000)) {
      lastAudioTelemetry = xTaskGetTickCount();
#if ENABLE_DEBUG
      AudioSnapshot telemetry{};
      AudioEngine_GetSnapshot(&telemetry);
      DEBUG_PRINTF("[AUDIO] pcm=%lu peak=%lu/%lu vol=%u mute=%u rate=%lu gap=%lu maxGap=%lums drop=%lu invalid=%lu cmdDrop=%lu i2s=%lu clip=%lu dsp=%lu heap=%lu\n",
                   static_cast<unsigned long>(audioFramesSinceTelemetry),
                   static_cast<unsigned long>(telemetryPeakLeft),
                   static_cast<unsigned long>(telemetryPeakRight),
                   static_cast<unsigned>(state.volume), 0U,
                   static_cast<unsigned long>(state.sampleRate),
                   static_cast<unsigned long>(telemetry.pcmGapEvents),
                   static_cast<unsigned long>(telemetry.pcmGapMaxMs),
                   static_cast<unsigned long>(telemetry.droppedPackets),
                   static_cast<unsigned long>(telemetry.invalidPcmPackets),
                   static_cast<unsigned long>(telemetry.commandQueueDrops),
                   static_cast<unsigned long>(telemetry.i2sWriteFaults),
                   static_cast<unsigned long>(telemetry.clippedFrames),
                   static_cast<unsigned long>(telemetry.dspFaults),
                   static_cast<unsigned long>(ESP.getFreeHeap()));
#endif
      audioFramesSinceTelemetry = 0;
      telemetryPeakLeft = 0;
      telemetryPeakRight = 0;
    }
  }
}
}  // namespace

bool AudioEngine_Begin() {
  if (s_audioTaskHandle != nullptr) return true;
  if (!installI2S(kDefaultSampleRate)) return false;

  // Keep the new queue private until task creation succeeds. The task receives
  // its own handle as a parameter, so it never depends on global publication.
  QueueHandle_t newQueue = xQueueCreate(24, sizeof(AudioCommand));
  if (newQueue == nullptr) {
    i2s_driver_uninstall(I2S_NUM_1);
    return false;
  }

  TaskHandle_t newTaskHandle = nullptr;
  const BaseType_t result = xTaskCreatePinnedToCore(
      audioTask, "AudioTask", Horizon::kAudioTaskStack,
      static_cast<void*>(newQueue), 10, &newTaskHandle, 1);
  if (result != pdPASS) {
    vQueueDelete(newQueue);
    i2s_driver_uninstall(I2S_NUM_1);
    return false;
  }

  s_audioTaskHandle = newTaskHandle;
  s_commandQueue.store(newQueue, std::memory_order_release);
  return true;
}

void AudioEngine_OnBtPcm(const uint8_t* data, uint32_t length) {
  // A2DP PCM is interleaved signed 16-bit stereo. Reject malformed or stale
  // input before reserving any ring space.
  if (data == nullptr || length < 4 || (length & 0x03U) != 0) {
    countInvalidPcmPacket();
    return;
  }
  const uint32_t frames = length / 4;
  if (frames >= kRingFrameCount) {
    countInvalidPcmPacket();
    return;
  }
#if ENABLE_DEBUG
  // Log only the first few valid callbacks; never print every real-time packet.
  // If this never appears while music is playing, the problem is upstream of DSP.
  static std::atomic<uint32_t> s_pcmCallbackDiagnostics{0};
  const uint32_t callbackNumber =
      s_pcmCallbackDiagnostics.fetch_add(1, std::memory_order_relaxed);
  if (callbackNumber < 3) {
    DEBUG_PRINTF("[AUDIO] PCM callback #%lu bytes=%lu paused=%u rateSwitch=%u\n",
                 static_cast<unsigned long>(callbackNumber + 1),
                 static_cast<unsigned long>(length),
                 s_pcmWritePaused.load(std::memory_order_acquire) ? 1U : 0U,
                 s_rateSwitchPending.load(std::memory_order_acquire) ? 1U : 0U);
  }
#endif

  bool expectedIdle = false;
  if (!s_pcmProducerBusy.compare_exchange_strong(
          expectedIdle, true, std::memory_order_acq_rel)) {
    // Unexpected re-entry: drop this packet instead of double-reserving head.
    portENTER_CRITICAL(&s_ringMux);
    ++s_droppedPackets;
    portEXIT_CRITICAL(&s_ringMux);
    return;
  }
  struct ProducerReleaseGuard {
    ~ProducerReleaseGuard() {
      s_pcmProducerBusy.store(false, std::memory_order_release);
    }
  } producerGuard;

  if (s_rateSwitchPending.load(std::memory_order_acquire) ||
      s_pcmWritePaused.load(std::memory_order_acquire)) {
    portENTER_CRITICAL(&s_ringMux);
    ++s_droppedPackets;
    portEXIT_CRITICAL(&s_ringMux);
    return;
  }

  uint32_t writeHead = 0;
  uint32_t writeEpoch = 0;
  bool hasCapacity = false;
  portENTER_CRITICAL(&s_ringMux);
  const uint32_t occupied = (s_head - s_tail) & kRingMask;
  const uint32_t freeFrames = kRingFrameCount - 1 - occupied;
  if (frames <= freeFrames &&
      !s_rateSwitchPending.load(std::memory_order_acquire) &&
      !s_pcmWritePaused.load(std::memory_order_acquire)) {
    writeHead = s_head;
    writeEpoch = s_ringEpoch;
    hasCapacity = true;
  } else {
    ++s_droppedPackets;
  }
  portEXIT_CRITICAL(&s_ringMux);
  if (!hasCapacity) return;

  // Critical fix MEM-02: reserve under a short spinlock, then perform the
  // potentially large memcpy without holding a cross-core critical section.
  const uint32_t firstFrames = std::min<uint32_t>(frames, kRingFrameCount - writeHead);
  memcpy(s_pcmRing + writeHead * 2, data, firstFrames * 4);
  const uint32_t remainingFrames = frames - firstFrames;
  if (remainingFrames > 0) {
    memcpy(s_pcmRing, data + firstFrames * 4, remainingFrames * 4);
  }

  bool accepted = false;
  portENTER_CRITICAL(&s_ringMux);
  if (writeEpoch == s_ringEpoch && s_head == writeHead &&
      !s_rateSwitchPending.load(std::memory_order_acquire) &&
      !s_pcmWritePaused.load(std::memory_order_acquire)) {
    const uint32_t now = millis();
    if (s_lastPacketMs != 0) {
      const uint32_t gapMs = now - s_lastPacketMs;
      if (gapMs >= kPcmStreamRestartMs) {
        // A long pause/disconnect is a new playback session, not a short PCM
        // dropout. Clear session telemetry so G/M do not accumulate across songs.
        s_pcmGapEvents = 0;
        s_pcmGapMaxMs = 0;
        s_filterResetPending = true;
      } else if (gapMs >= kPcmGapResetMs) {
        ++s_pcmGapEvents;
        s_pcmGapMaxMs = std::max(s_pcmGapMaxMs, gapMs);
        s_filterResetPending = true;
      }
    }
    // Publish data only after memcpy completes, so the consumer never sees a
    // partially copied packet. The sole-producer guard protects this commit.
    s_head = (writeHead + frames) & kRingMask;
    s_lastPacketMs = now;
    accepted = true;
  } else {
    ++s_droppedPackets;  // An epoch/pause change invalidated this in-flight copy.
  }
  portEXIT_CRITICAL(&s_ringMux);

  if (accepted && s_audioTaskHandle != nullptr) xTaskNotifyGive(s_audioTaskHandle);
}

void AudioEngine_OnSampleRate(uint16_t rate) {
  if (rate != 16000 && rate != 32000 && rate != 44100 && rate != 48000) return;

  bool changed = false;
  portENTER_CRITICAL(&s_rateMux);
  if (rate != s_sourceRate) {
    // Pause input before publishing the new source rate. AudioTask reconfigures
    // the resampler and flushes old-rate PCM; I2S remains fixed at 48 kHz.
    s_rateSwitchPending.store(true, std::memory_order_release);
    s_pcmWritePaused.store(true, std::memory_order_release);
    s_sourceRate = rate;
    ++s_rateGeneration;
    changed = true;
  }
  portEXIT_CRITICAL(&s_rateMux);

  if (changed) {
    portENTER_CRITICAL(&s_ringMux);
    ++s_ringEpoch;  // Invalidate in-flight producer/consumer copies immediately.
    s_filterResetPending = true;
    portEXIT_CRITICAL(&s_ringMux);
    if (s_audioTaskHandle != nullptr) xTaskNotifyGive(s_audioTaskHandle);
  }
}

void AudioEngine_SetBandGain(uint8_t band, float gainDb) {
  if (band >= Horizon::kBandCount || !std::isfinite(gainDb)) return;
  enqueue({CommandType::SetBand, band, constrain(gainDb, -18.0f, 18.0f)});
}

void AudioEngine_AdjustBandGain(uint8_t band, float deltaDb) {
  if (band >= Horizon::kBandCount || !std::isfinite(deltaDb)) return;
  enqueue({CommandType::AdjustBand, band, deltaDb});
}

void AudioEngine_SetPreset(uint8_t preset) {
  if (preset >= 5) return;
  enqueue({CommandType::SetPreset, preset, 0.0f});
}

void AudioEngine_LoadCustomPreset(uint8_t slot, const float gainsDb[Horizon::kBandCount]) {
  if (slot >= 2 || gainsDb == nullptr) return;
  // Intended for boot-time restore before AudioEngine_Begin() starts the task.
  for (uint8_t band = 0; band < Horizon::kBandCount; ++band) {
    if (!std::isfinite(gainsDb[band])) return;
  }
  for (uint8_t band = 0; band < Horizon::kBandCount; ++band) {
    s_loadedCustomGains[slot][band] = constrain(gainsDb[band], -18.0f, 18.0f);
  }
}

void AudioEngine_SetVolume(int volume) {
  enqueue({CommandType::SetVolume, 0, static_cast<float>(constrain(volume, 0, 100))});
}

void AudioEngine_SetBluetoothConnected(bool connected) {
  portENTER_CRITICAL(&s_snapshotMux);
  s_snapshot.bluetoothConnected = connected;
  portEXIT_CRITICAL(&s_snapshotMux);
}

void AudioEngine_GetSnapshot(AudioSnapshot* out) {
  if (out == nullptr) return;
  portENTER_CRITICAL(&s_snapshotMux);
  memcpy(out, &s_snapshot, sizeof(AudioSnapshot));
  portEXIT_CRITICAL(&s_snapshotMux);
}

void AudioEngine_GetSnapshotForUi(AudioSnapshot* out) {
  if (out == nullptr) return;
  portENTER_CRITICAL(&s_snapshotMux);
  memcpy(out, &s_snapshot, sizeof(AudioSnapshot));
  s_snapshot.peakLeft = 0;
  s_snapshot.peakRight = 0;
  portEXIT_CRITICAL(&s_snapshotMux);
}
