#include "biquad_filter.h"

#include <cmath>
#include <algorithm>

namespace HorizonDLMS {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kMinimumFrequencyHz = 1.0f;
constexpr float kMinimumQ = 0.05f;
constexpr float kMinimumA0Magnitude = 1.0e-12f;

/**
 * Check floating-point inputs before using them in coefficient formulas.
 */
bool isFiniteFloat(float value) {
    return std::isfinite(value);
}

/**
 * Clamp gain to the range supported by the Horizon DLMS specification.
 */
float clampGainDb(float gainDb) {
    return std::max(
        Config::EQ_MIN_GAIN_DB,
        std::min(Config::EQ_MAX_GAIN_DB, gainDb));
}

} // namespace

BiquadFilter::BiquadFilter(const Parameters& parameters) {
    // Constructor performs a validated setup; invalid input leaves the
    // object in its safe identity-filter default state.
    configure(parameters);
}

bool BiquadFilter::configure(const Parameters& parameters) {
    RawCoefficients raw{};

    if (!calculateCoefficients(parameters, raw)) {
        return false;
    }

    NormalizedCoefficients normalized{};
    if (!normalizeCoefficients(raw, normalized)) {
        return false;
    }

    parameters_ = parameters;
    parameters_.gainDb = clampGainDb(parameters.gainDb);
    targetGainDb_ = parameters_.gainDb;

    current_ = normalized;
    target_ = normalized;
    step_ = {};
    transitionSamplesRemaining_ = 0;

    configured_ = true;
    reset();
    return true;
}

bool BiquadFilter::calculateCoefficients(
    const Parameters& parameters,
    RawCoefficients& output) {

    const float fs = parameters.sampleRateHz;
    const float frequency = parameters.frequencyHz;
    const float q = parameters.q;
    const float gainDb = clampGainDb(parameters.gainDb);

    // Reject invalid parameters instead of producing NaN/Inf coefficients.
    if (!isFiniteFloat(fs) ||
        !isFiniteFloat(frequency) ||
        !isFiniteFloat(q) ||
        !isFiniteFloat(gainDb) ||
        fs <= 0.0f ||
        frequency < kMinimumFrequencyHz ||
        frequency >= (fs * 0.5f) ||
        q < kMinimumQ) {
        return false;
    }

    const float omega = (2.0f * kPi * frequency) / fs;
    const float cosine = std::cos(omega);
    const float sine = std::sin(omega);
    const float A = std::pow(10.0f, gainDb / 40.0f);

    RawCoefficients c{};

    switch (parameters.type) {
        case EqFilterType::PEAKING: {
            // Audio EQ Cookbook peaking EQ.
            const float alpha = sine / (2.0f * q);

            c.b0 = 1.0f + alpha * A;
            c.b1 = -2.0f * cosine;
            c.b2 = 1.0f - alpha * A;

            c.a0 = 1.0f + alpha / A;
            c.a1 = -2.0f * cosine;
            c.a2 = 1.0f - alpha / A;
            break;
        }

        case EqFilterType::LOW_SHELF: {
            // Audio EQ Cookbook low-shelf.
            // q is used as shelf slope S for this filter type.
            const float slope = q;
            const float shelfTerm =
                ((A + (1.0f / A)) * ((1.0f / slope) - 1.0f)) + 2.0f;

            if (shelfTerm < 0.0f || !isFiniteFloat(shelfTerm)) {
                return false;
            }

            const float alpha =
                (sine * 0.5f) * std::sqrt(shelfTerm);
            const float beta = 2.0f * std::sqrt(A) * alpha;

            c.b0 = A * ((A + 1.0f) - (A - 1.0f) * cosine + beta);
            c.b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosine);
            c.b2 = A * ((A + 1.0f) - (A - 1.0f) * cosine - beta);

            c.a0 = (A + 1.0f) + (A - 1.0f) * cosine + beta;
            c.a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cosine);
            c.a2 = (A + 1.0f) + (A - 1.0f) * cosine - beta;
            break;
        }

        case EqFilterType::HIGH_SHELF: {
            // Audio EQ Cookbook high-shelf.
            // q is used as shelf slope S for this filter type.
            const float slope = q;
            const float shelfTerm =
                ((A + (1.0f / A)) * ((1.0f / slope) - 1.0f)) + 2.0f;

            if (shelfTerm < 0.0f || !isFiniteFloat(shelfTerm)) {
                return false;
            }

            const float alpha =
                (sine * 0.5f) * std::sqrt(shelfTerm);
            const float beta = 2.0f * std::sqrt(A) * alpha;

            c.b0 = A * ((A + 1.0f) + (A - 1.0f) * cosine + beta);
            c.b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosine);
            c.b2 = A * ((A + 1.0f) + (A - 1.0f) * cosine - beta);

            c.a0 = (A + 1.0f) - (A - 1.0f) * cosine + beta;
            c.a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * cosine);
            c.a2 = (A + 1.0f) - (A - 1.0f) * cosine - beta;
            break;
        }

        default:
            return false;
    }

    if (!isFiniteFloat(c.b0) || !isFiniteFloat(c.b1) ||
        !isFiniteFloat(c.b2) || !isFiniteFloat(c.a0) ||
        !isFiniteFloat(c.a1) || !isFiniteFloat(c.a2) ||
        std::fabs(c.a0) < kMinimumA0Magnitude) {
        return false;
    }

    output = c;
    return true;
}

bool BiquadFilter::normalizeCoefficients(
    const RawCoefficients& raw,
    NormalizedCoefficients& normalized) {

    if (!isFiniteFloat(raw.a0) ||
        std::fabs(raw.a0) < kMinimumA0Magnitude) {
        return false;
    }

    // Division occurs only when coefficients are calculated, never in
    // the per-sample PCM recurrence.
    const float inverseA0 = 1.0f / raw.a0;

    NormalizedCoefficients result{};
    result.b0 = raw.b0 * inverseA0;
    result.b1 = raw.b1 * inverseA0;
    result.b2 = raw.b2 * inverseA0;
    result.a1 = raw.a1 * inverseA0;
    result.a2 = raw.a2 * inverseA0;

    if (!isFiniteFloat(result.b0) || !isFiniteFloat(result.b1) ||
        !isFiniteFloat(result.b2) || !isFiniteFloat(result.a1) ||
        !isFiniteFloat(result.a2)) {
        return false;
    }

    normalized = result;
    return true;
}

bool BiquadFilter::setGainDb(
    float gainDb,
    uint32_t transitionSamples) {

    if (!configured_ || !isFiniteFloat(gainDb)) {
        return false;
    }

    Parameters updated = parameters_;
    updated.gainDb = clampGainDb(gainDb);

    RawCoefficients raw{};
    if (!calculateCoefficients(updated, raw)) {
        return false;
    }

    NormalizedCoefficients normalized{};
    if (!normalizeCoefficients(raw, normalized)) {
        return false;
    }

    targetGainDb_ = updated.gainDb;
    parameters_.gainDb = updated.gainDb;

    beginCoefficientTransition(normalized, transitionSamples);
    return true;
}

void BiquadFilter::beginCoefficientTransition(
    const NormalizedCoefficients& newTarget,
    uint32_t transitionSamples) {

    target_ = newTarget;

    if (transitionSamples == 0) {
        current_ = target_;
        step_ = {};
        transitionSamplesRemaining_ = 0;
        return;
    }

    const float inverseLength = 1.0f / static_cast<float>(transitionSamples);

    step_.b0 = (target_.b0 - current_.b0) * inverseLength;
    step_.b1 = (target_.b1 - current_.b1) * inverseLength;
    step_.b2 = (target_.b2 - current_.b2) * inverseLength;
    step_.a1 = (target_.a1 - current_.a1) * inverseLength;
    step_.a2 = (target_.a2 - current_.a2) * inverseLength;

    transitionSamplesRemaining_ = transitionSamples;
}

void BiquadFilter::advanceCoefficientTransition() {
    if (transitionSamplesRemaining_ == 0) {
        return;
    }

    current_.b0 += step_.b0;
    current_.b1 += step_.b1;
    current_.b2 += step_.b2;
    current_.a1 += step_.a1;
    current_.a2 += step_.a2;

    --transitionSamplesRemaining_;

    // Assign exact target values at the end to avoid accumulated
    // floating-point rounding error across the transition.
    if (transitionSamplesRemaining_ == 0) {
        current_ = target_;
        step_ = {};
    }
}

float BiquadFilter::processChannel(
    float input,
    BiquadChannelState& state,
    const NormalizedCoefficients& coefficients) {

    // Standard Direct Form I recurrence using independent x/y history.
    const float output =
        (coefficients.b0 * input) +
        (coefficients.b1 * state.x1) +
        (coefficients.b2 * state.x2) -
        (coefficients.a1 * state.y1) -
        (coefficients.a2 * state.y2);

    state.x2 = state.x1;
    state.x1 = input;
    state.y2 = state.y1;
    state.y1 = output;

    return output;
}

float BiquadFilter::processSample(float input, Channel channel) {
    if (!enabled_ || !configured_) {
        return input;
    }

    // A coefficient transition advances once per audio frame, not once
    // for each channel. For single-channel calls, use processStereo()
    // when processing stereo PCM so both channels share one ramp step.
    if (channel == Channel::LEFT) {
        advanceCoefficientTransition();
    }

    BiquadChannelState& state =
        (channel == Channel::LEFT) ? leftState_ : rightState_;

    return processChannel(input, state, current_);
}

void BiquadFilter::processStereo(
    float inputLeft,
    float inputRight,
    float& outputLeft,
    float& outputRight) {

    if (!enabled_ || !configured_) {
        outputLeft = inputLeft;
        outputRight = inputRight;
        return;
    }

    // Advance once per stereo frame so both channels always use the
    // same coefficients, preventing left/right image movement.
    advanceCoefficientTransition();

    outputLeft = processChannel(inputLeft, leftState_, current_);
    outputRight = processChannel(inputRight, rightState_, current_);
}

bool BiquadFilter::processInterleaved(
    float* samples,
    uint32_t sampleCount) {

    if (samples == nullptr || (sampleCount % 2U) != 0U) {
        return false;
    }

    for (uint32_t i = 0; i < sampleCount; i += 2U) {
        float leftOut = 0.0f;
        float rightOut = 0.0f;

        processStereo(
            samples[i],
            samples[i + 1U],
            leftOut,
            rightOut);

        samples[i] = leftOut;
        samples[i + 1U] = rightOut;
    }

    return true;
}

void BiquadFilter::reset() {
    leftState_ = {};
    rightState_ = {};
}

void BiquadFilter::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool BiquadFilter::isEnabled() const {
    return enabled_;
}

float BiquadFilter::targetGainDb() const {
    return targetGainDb_;
}

const BiquadFilter::Parameters& BiquadFilter::parameters() const {
    return parameters_;
}

const BiquadFilter::NormalizedCoefficients&
BiquadFilter::activeCoefficients() const {
    return current_;
}

} // namespace HorizonDLMS
