#include "sfx.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace engine {
namespace sfx {

namespace {

constexpr double TWO_PI = 6.283185307179586;

// Scale so the loudest sample sits at `peak`, and hard-guarantee [-1, 1].
void normalize(std::vector<float>& frames, float peak) {
    float maxAbs = 0.0f;
    for (float f : frames) maxAbs = std::max(maxAbs, std::fabs(f));
    if (maxAbs <= 0.0f) return;
    float gain = peak / maxAbs;
    for (float& f : frames) f = std::max(-1.0f, std::min(f * gain, 1.0f));
}

}  // namespace

std::vector<float> gunshot(uint32_t sampleRate, uint32_t seed) {
    const auto count = static_cast<size_t>(sampleRate * 0.30);
    std::vector<float> frames(count);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> noise(-1.0, 1.0);

    double lowpass = 0.0;
    for (size_t i = 0; i < count; i++) {
        double t = static_cast<double>(i) / sampleRate;
        // The crack: white noise through a closing low-pass (bright attack
        // that darkens as it decays), dying fast.
        double cutoff = 0.9 * std::exp(-t * 18.0) + 0.05;
        lowpass += cutoff * (noise(rng) - lowpass);
        double crack = lowpass * std::exp(-t * 28.0);
        // The body: a thump sweeping 150 -> 55 Hz.
        double sweep = 150.0 * std::exp(-t * 10.0) + 55.0;
        double thump = 0.8 * std::sin(TWO_PI * sweep * t) * std::exp(-t * 11.0);
        // Soft clip for punch.
        frames[i] = static_cast<float>(std::tanh(2.2 * (crack + thump)));
    }
    normalize(frames, 0.9f);
    return frames;
}

std::vector<float> impact(uint32_t sampleRate, uint32_t seed) {
    const auto count = static_cast<size_t>(sampleRate * 0.22);
    std::vector<float> frames(count);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> noise(-1.0, 1.0);
    std::uniform_real_distribution<double> pitchJitter(0.85, 1.25);

    // The strike pitch varies with the seed; the partial ratios are inharmonic
    // (a knock, not a note).
    double base = 150.0 * pitchJitter(rng);
    const double partials[3] = {1.0, 2.76, 5.4};
    const double gains[3] = {1.0, 0.45, 0.18};

    for (size_t i = 0; i < count; i++) {
        double t = static_cast<double>(i) / sampleRate;
        double knock = 0.0;
        for (int p = 0; p < 3; p++)
            knock += gains[p] * std::sin(TWO_PI * base * partials[p] * t) *
                     std::exp(-t * (16.0 + 14.0 * p));
        // Contact scuff: a few ms of noise at the touch.
        double scuff = 0.25 * noise(rng) * std::exp(-t * 220.0);
        frames[i] = static_cast<float>(knock + scuff);
    }
    normalize(frames, 0.8f);
    return frames;
}

}  // namespace sfx
}  // namespace engine
