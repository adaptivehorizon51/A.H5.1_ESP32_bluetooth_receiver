#pragma once
/*
 * StereoResampler - streaming stereo int16 polyphase windowed-sinc (Kaiser).
 *
 * Compatible with Arduino-ESP32 2.x / gnu++11.
 * Threading contract: configure(), reset(), and process() must be called by
 * one task only (audioTask in Horizon Audio DLMS).
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

class StereoResampler {
 public:
  enum {
    TAPS = 32,
    MAX_PHASES = 160,
    MAX_IN_FRAMES = 256,
    HISTORY_FRAMES = TAPS - 1,
    MAX_OUT_FRAMES = MAX_IN_FRAMES * 3 + 8
  };

  bool configure(uint32_t inRate, uint32_t outRate) {
    // Validate before changing the current working configuration.
    if (inRate < 8000U || outRate < inRate || outRate > 96000U ||
        outRate > inRate * 3U) {
      return false;
    }

    if (inRate == outRate) {
      p_ = 1U;
      q_ = 1U;
      passthrough_ = true;
      reset();
      return true;
    }

    const uint32_t g = gcd(inRate, outRate);
    const uint32_t newP = inRate / g;
    const uint32_t newQ = outRate / g;
    if (newQ == 0U || newQ > MAX_PHASES) return false;

    // Keep the 3-phase upsampler in a separate small cache so configuring
    // 16/32 kHz -> 48 kHz does not replace the 160-phase table.
    if (newQ == THREE_PHASE_COUNT) {
      if (!threePhaseTableBuilt_) {
        buildTable(newQ, threePhaseTable_);
        threePhaseTableBuilt_ = true;
      }
    } else if (tablePhasesBuilt_ != newQ) {
      buildTable(newQ, table_);
      tablePhasesBuilt_ = newQ;
    }

    p_ = newP;
    q_ = newQ;
    passthrough_ = false;
    reset();
    return true;
  }

  void reset() {
    pos_ = TAPS / 2 - 1;
    acc_ = 0;
    memset(hist_, 0, sizeof(hist_));
  }

  bool passthrough() const { return passthrough_; }

  // Input/output are stereo-interleaved signed 16-bit PCM. Returns output frames.
  size_t process(const int16_t *in, size_t inFrames, int16_t *out) {
    if (in == nullptr || out == nullptr || inFrames == 0U ||
        inFrames > MAX_IN_FRAMES) {
      return 0U;
    }

    if (passthrough_) {
      memcpy(out, in, inFrames * 2U * sizeof(int16_t));
      return inFrames;
    }

    if (pos_ < (TAPS / 2 - 1)) reset();

    const size_t total = inFrames + HISTORY_FRAMES;
    memcpy(work_, hist_, HISTORY_FRAMES * 2U * sizeof(int16_t));
    memcpy(work_ + HISTORY_FRAMES * 2U, in, inFrames * 2U * sizeof(int16_t));

    const int16_t *activeTable = (q_ == THREE_PHASE_COUNT)
                                     ? threePhaseTable_
                                     : table_;
    size_t produced = 0U;
    while (produced < MAX_OUT_FRAMES && (pos_ + TAPS / 2) < total) {
      const int16_t *c = activeTable + static_cast<size_t>(acc_) * TAPS;
      const int16_t *x = work_ + (pos_ - (TAPS / 2 - 1)) * 2U;

      int32_t sumL = 0;
      int32_t sumR = 0;
      for (size_t k = 0; k < TAPS; k += 4U) {
        int32_t coeff = c[k];
        sumL += (coeff * x[k * 2U]) >> 2;
        sumR += (coeff * x[k * 2U + 1U]) >> 2;

        coeff = c[k + 1U];
        sumL += (coeff * x[(k + 1U) * 2U]) >> 2;
        sumR += (coeff * x[(k + 1U) * 2U + 1U]) >> 2;

        coeff = c[k + 2U];
        sumL += (coeff * x[(k + 2U) * 2U]) >> 2;
        sumR += (coeff * x[(k + 2U) * 2U + 1U]) >> 2;

        coeff = c[k + 3U];
        sumL += (coeff * x[(k + 3U) * 2U]) >> 2;
        sumR += (coeff * x[(k + 3U) * 2U + 1U]) >> 2;
      }

      out[produced * 2U] = clip16((sumL + 8192) >> 13);
      out[produced * 2U + 1U] = clip16((sumR + 8192) >> 13);
      ++produced;

      acc_ += p_;
      if (acc_ >= q_) {
        acc_ -= q_;
        ++pos_;
      }
    }

    // Subtract inFrames from pos_ to make it block-relative for the next call.
    // For upsampling (e.g. 44100→48000) the loop may legitimately exit with
    // pos_ still inside [0, inFrames) when the last phase step did not
    // increment pos_. That state is valid — do NOT reset() here, as that would
    // zero hist_[] mid-stream and produce an audible pop.
    if (pos_ >= inFrames) {
      pos_ -= inFrames;
    }
    // else: pos_ is already a valid carry-over offset into the next block.

    memcpy(hist_, work_ + inFrames * 2U, HISTORY_FRAMES * 2U * sizeof(int16_t));
    return produced;
  }

  float maxAbsCoefficientSum(uint32_t phases) const {
    if (passthrough_) return 0.0f;
    if (phases > q_) phases = q_;
    if (phases > MAX_PHASES) phases = MAX_PHASES;

    const int16_t *activeTable = nullptr;
    if (q_ == THREE_PHASE_COUNT) {
      if (!threePhaseTableBuilt_) return 0.0f;
      activeTable = threePhaseTable_;
    } else {
      if (tablePhasesBuilt_ != q_) return 0.0f;
      activeTable = table_;
    }

    float worst = 0.0f;
    for (uint32_t ph = 0; ph < phases; ++ph) {
      float sum = 0.0f;
      for (size_t k = 0; k < TAPS; ++k) {
        sum += fabsf(static_cast<float>(activeTable[ph * TAPS + k]));
      }
      if (sum > worst) worst = sum;
    }
    return worst / 32768.0f;
  }

 private:
  static float kaiserBeta() { return 7.0f; }

  static uint32_t gcd(uint32_t a, uint32_t b) {
    while (b != 0U) {
      const uint32_t t = a % b;
      a = b;
      b = t;
    }
    return a;
  }

  static int16_t clip16(int32_t v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
  }

  static float besselI0(float x) {
    float sum = 1.0f;
    float term = 1.0f;
    const float q = x * 0.5f;
    for (int k = 1; k < 40; ++k) {
      const float f = q / static_cast<float>(k);
      term *= f * f;
      sum += term;
      if (term < 1e-9f * sum) break;
    }
    return sum;
  }

  void buildTable(uint32_t phases, int16_t *destination) {
    const float half = static_cast<float>(TAPS) * 0.5f;
    const float beta = kaiserBeta();
    const float norm = 1.0f / besselI0(beta);
    float row[TAPS];

    for (uint32_t ph = 0; ph < phases; ++ph) {
      const float frac = static_cast<float>(ph) / static_cast<float>(phases);
      float sum = 0.0f;
      for (int k = 0; k < TAPS; ++k) {
        const float u = static_cast<float>(k - (TAPS / 2 - 1)) - frac;
        float sinc = 1.0f;
        if (fabsf(u) > 1e-6f) {
          const float pu = 3.14159265358979f * u;
          sinc = sinf(pu) / pu;
        }
        const float r = u / half;
        float window = 0.0f;
        if (fabsf(r) < 1.0f) {
          const float inside = 1.0f - r * r;
          window = besselI0(beta * sqrtf(inside)) * norm;
        }
        row[k] = sinc * window;
        sum += row[k];
      }

      for (int k = 0; k < TAPS; ++k) {
        long v = lroundf(row[k] / sum * 32768.0f);
        if (v > 32767L) v = 32767L;
        if (v < -32768L) v = -32768L;
        destination[ph * TAPS + static_cast<size_t>(k)] = static_cast<int16_t>(v);
      }
    }
  }

  enum { THREE_PHASE_COUNT = 3 };

  bool passthrough_ = true;
  uint32_t p_ = 1U;
  uint32_t q_ = 1U;
  uint32_t acc_ = 0U;
  size_t pos_ = TAPS / 2 - 1;
  uint32_t tablePhasesBuilt_ = 0U;
  bool threePhaseTableBuilt_ = false;
  int16_t hist_[HISTORY_FRAMES * 2] = {};
  int16_t work_[(HISTORY_FRAMES + MAX_IN_FRAMES) * 2] = {};
  int16_t table_[MAX_PHASES * TAPS] = {};
  int16_t threePhaseTable_[THREE_PHASE_COUNT * TAPS] = {};
};
