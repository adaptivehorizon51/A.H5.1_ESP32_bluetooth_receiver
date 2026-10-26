#pragma once

/*
 * Horizon Audio DLMS
 * Phase 2: Stereo Biquad DSP Filter
 *
 * Platform: ESP32 / PlatformIO / Arduino-ESP32 / C++17
 *
 * This implementation uses the standard Audio EQ Cookbook
 * coefficient equations and Direct Form I sample history.
 * The two audio channels maintain independent filter history.
 *
 * Threading contract:
 *   - The DSP instance should be owned by the Core 1 audio task.
 *   - Do not call configure(), setGainDb(), reset(), or process*
 *     concurrently from different tasks.
 *   - Send control changes from Core 0 through a FreeRTOS queue.
 */

#include <stdint.h>
#include "dlms_state.h"

namespace HorizonDLMS {

class BiquadFilter final {
public:
    enum class Channel : uint8_t {
        LEFT = 0,
        RIGHT
    };

    /**
     * Complete filter design parameters for one biquad stage.
     * frequencyHz must be below Nyquist (sampleRateHz / 2).
     * For shelf filters, q is interpreted as the cookbook shelf
     * slope S; for peaking filters, q is the conventional Q.
     */
    struct Parameters {
        EqFilterType type = EqFilterType::PEAKING;
        float frequencyHz = 1000.0f;
        float q = Config::EQ_DEFAULT_Q_PEAKING;
        float gainDb = 0.0f;
        float sampleRateHz = static_cast<float>(Config::AUDIO_SAMPLE_RATE);
    };

    /**
     * Raw cookbook coefficients before normalization.
     * a0 is retained here for diagnostics and verification.
     */
    struct RawCoefficients {
        float b0 = 1.0f;
        float b1 = 0.0f;
        float b2 = 0.0f;
        float a0 = 1.0f;
        float a1 = 0.0f;
        float a2 = 0.0f;
    };

    /**
     * Normalized coefficients used by the PCM processing loop.
     * a0 is normalized to 1.0, so no division is required per sample.
     */
    struct NormalizedCoefficients {
        float b0 = 1.0f;
        float b1 = 0.0f;
        float b2 = 0.0f;
        float a1 = 0.0f;
        float a2 = 0.0f;
    };

    BiquadFilter() = default;
    explicit BiquadFilter(const Parameters& parameters);

    /**
     * Configure the filter and immediately initialize its coefficients.
     * Returns false if the supplied parameters are invalid.
     */
    bool configure(const Parameters& parameters);

    /**
     * Calculate cookbook coefficients for low-shelf, peaking, or
     * high-shelf filters. No filter state is changed by this method.
     */
    static bool calculateCoefficients(
        const Parameters& parameters,
        RawCoefficients& output);

    /**
     * Change target gain without abruptly replacing active coefficients.
     * The target is calculated once; normalized coefficients are linearly
     * ramped over transitionSamples during subsequent audio processing.
     *
     * Call this from the owning audio task, typically after receiving
     * an AudioCommand from the Core 0 queue.
     */
    bool setGainDb(float gainDb, uint32_t transitionSamples = 256);

    /**
     * Process one sample for one channel. LEFT and RIGHT have independent
     * delay history. Output is a single-precision floating-point sample.
     */
    float processSample(float input, Channel channel);

    /**
     * Process one stereo frame. The channels never share filter history.
     * Inputs and outputs are float PCM samples.
     */
    void processStereo(
        float inputLeft,
        float inputRight,
        float& outputLeft,
        float& outputRight);

    /**
     * Process an interleaved stereo buffer in-place.
     * sampleCount is the number of float samples, not stereo frames;
     * it must be even (L, R, L, R, ...).
     */
    bool processInterleaved(float* samples, uint32_t sampleCount);

    /**
     * Clear both channels' delay history. Current coefficients and gain
     * settings remain unchanged.
     */
    void reset();

    /**
     * Enable or bypass processing. Bypass returns input samples unchanged
     * while preserving filter history only when processing is enabled.
     */
    void setEnabled(bool enabled);
    bool isEnabled() const;

    float targetGainDb() const;
    const Parameters& parameters() const;
    const NormalizedCoefficients& activeCoefficients() const;

private:
    Parameters parameters_{};

    NormalizedCoefficients current_{};
    NormalizedCoefficients target_{};
    NormalizedCoefficients step_{};

    BiquadChannelState leftState_{};
    BiquadChannelState rightState_{};

    float targetGainDb_ = 0.0f;
    uint32_t transitionSamplesRemaining_ = 0;
    bool enabled_ = true;
    bool configured_ = false;

    /**
     * Normalize raw coefficients once, outside the PCM sample loop.
     */
    static bool normalizeCoefficients(
        const RawCoefficients& raw,
        NormalizedCoefficients& normalized);

    /**
     * Set the coefficient ramp increments for a finite transition.
     */
    void beginCoefficientTransition(
        const NormalizedCoefficients& newTarget,
        uint32_t transitionSamples);

    /**
     * Advance active coefficients by one ramp step.
     */
    void advanceCoefficientTransition();

    /**
     * Apply Direct Form I recurrence to one channel and update history.
     */
    static float processChannel(
        float input,
        BiquadChannelState& state,
        const NormalizedCoefficients& coefficients);
};

} // namespace HorizonDLMS
