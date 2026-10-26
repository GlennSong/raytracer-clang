#ifndef RAYTRACER_ENGINE_AUDIO_SFX_DSP_H
#define RAYTRACER_ENGINE_AUDIO_SFX_DSP_H

// The few DSP pieces the procedural sound files share (sfx_ground.cpp, sfx_motion.cpp): one-pole filters with the
// cutoff in Hz, peak normalising, and the CIRCULAR filters and envelopes that make a loop seamless by construction.
// Internal to src/engine/audio; nothing outside includes it.

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

namespace engine {
namespace sfx {
namespace dsp {

constexpr double TWO_PI = 6.283185307179586;

inline void normalizeTo(std::vector<float>& frames, float peak) {
    float maxAbs = 0.0f;
    for (float f : frames) maxAbs = std::max(maxAbs, std::fabs(f));
    if (maxAbs <= 0.0f) return;
    const float gain = peak / maxAbs;
    for (float& f : frames) f = std::max(-1.0f, std::min(f * gain, 1.0f));
}

// One-pole filters with the cutoff in Hz, so a recipe sounds the same at any device rate.
struct LowPass {
    double a = 0, y = 0;
    LowPass(double hz, double rate) : a(1.0 - std::exp(-TWO_PI * hz / rate)) {}
    double operator()(double x) { y += a * (x - y); return y; }
};
struct HighPass {
    LowPass lp;
    HighPass(double hz, double rate) : lp(hz, rate) {}
    double operator()(double x) { return x - lp(x); }
};
struct BandPass {   // two one-poles: a gentle band, enough for noise colour
    HighPass hp; LowPass lp;
    BandPass(double lo, double hi, double rate) : hp(lo, rate), lp(hi, rate) {}
    double operator()(double x) { return lp(hp(x)); }
};

// Circular box filter and helpers for the LOOPS (same trick as sfx::engine: wrapped windows make the
// filtered noise exactly periodic, so the seam is just another sample).
inline std::vector<double> circBox(const std::vector<double>& in, int window) {
    const auto n = static_cast<int>(in.size());
    const int half = std::max(1, std::min(window / 2, n / 2 - 1));
    std::vector<double> out(in.size());
    double sum = 0.0;
    for (int k = -half; k <= half; k++) sum += in[static_cast<size_t>(((k % n) + n) % n)];
    const double norm = 1.0 / (2 * half + 1);
    for (int i = 0; i < n; i++) {
        out[static_cast<size_t>(i)] = sum * norm;
        sum += in[static_cast<size_t>((i + half + 1) % n)] - in[static_cast<size_t>(((i - half) % n + n) % n)];
    }
    return out;
}
inline std::vector<double> circBand(const std::vector<double>& white, double rate, double loHz, double hiHz) {
    std::vector<double> hi = circBox(white, std::max(1, static_cast<int>(rate / hiHz)));
    const std::vector<double> lo = circBox(white, std::max(2, static_cast<int>(rate / loHz)));
    for (size_t i = 0; i < hi.size(); ++i) hi[i] -= lo[i];
    double s = 0;
    for (double x : hi) s += x * x;
    const double rms = std::sqrt(s / std::max<size_t>(1, hi.size()));
    if (rms > 1e-12) for (double& x : hi) x /= rms;
    return hi;
}
// A smooth periodic random envelope: a few integer-cycle sines, >= 0.
inline std::vector<double> circEnvelope(size_t count, std::mt19937& rng, int minCycles, int maxCycles, int terms) {
    std::uniform_int_distribution<int> cyc(minCycles, maxCycles);
    std::uniform_real_distribution<double> ph(0.0, TWO_PI), amp(0.4, 1.0);
    std::vector<std::pair<int, std::pair<double, double>>> t;
    for (int k = 0; k < terms; ++k) t.push_back({cyc(rng), {ph(rng), amp(rng)}});
    std::vector<double> env(count);
    for (size_t i = 0; i < count; ++i) {
        const double p = static_cast<double>(i) / static_cast<double>(count);
        double v = 0;
        for (const auto& [c, pa] : t) v += pa.second * std::sin(TWO_PI * c * p + pa.first);
        env[i] = v;
    }
    return env;
}

}  // namespace dsp
}  // namespace sfx
}  // namespace engine

#endif
