#pragma once
#include <Arduino.h>
#include "config.h"

struct AudioSnapshot {
  float bandDb[Horizon::kBandCount];
  uint8_t preset;
  uint8_t volume;
  bool muted;
  bool streamActive;
  uint32_t sampleRate;
  // Post-EQ/output peak sample magnitudes, 0..32768 (signed 16-bit full scale).
  uint16_t peakLeft;
  uint16_t peakRight;
  uint32_t droppedPackets;
  uint32_t i2sWriteFaults;
  // Frames where either channel crossed the per-sample soft-clip knee.
  uint32_t clippedFrames;
  uint32_t dspFaults;
  uint32_t pcmGapEvents;
  uint32_t pcmGapMaxMs;
  uint32_t invalidPcmPackets;
  uint32_t commandQueueDrops;
  bool bluetoothConnected;
};

// Creates I2S, queue and AudioTask. The command queue is published only after
// task creation succeeds; commands issued earlier are safely rejected.
bool AudioEngine_Begin();
// A2DP stream reader callback: expects interleaved signed 16-bit stereo PCM.
// PCM copies happen outside spinlocks; packets crossing a rate/flush epoch drop.
void AudioEngine_OnBtPcm(const uint8_t* data, uint32_t length);
// Requests a supported Bluetooth source rate. AudioTask flushes the ring and
// reconfigures the resampler; the PCM5102A I2S output stays fixed at 48 kHz.
void AudioEngine_OnSampleRate(uint16_t rate);
void AudioEngine_SetBandGain(uint8_t band, float gainDb);
void AudioEngine_AdjustBandGain(uint8_t band, float deltaDb);
void AudioEngine_SetPreset(uint8_t preset);
// Load a custom preset before AudioEngine_Begin(); slot 0=CUSTOM 1, 1=CUSTOM 2.
void AudioEngine_LoadCustomPreset(uint8_t slot, const float gainsDb[Horizon::kBandCount]);
void AudioEngine_SetVolume(int volume);
void AudioEngine_SetBluetoothConnected(bool connected);
// Copies the audio snapshot under the cross-core snapshot lock.
void AudioEngine_GetSnapshot(AudioSnapshot* out);
// Copies the snapshot and clears the peak accumulator for the next UI frame.
void AudioEngine_GetSnapshotForUi(AudioSnapshot* out);
