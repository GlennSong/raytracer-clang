// The ground's sounds (#62 footsteps by surface, #63 jump and landing, #64 grass, #65 water): pure
// (parameters, seed) -> PCM like the rest of sfx.h. No samples anywhere; every surface is a recipe of
// the same few pieces -- a heel transient, a body thump, and a TEXTURE (filtered noise, or grains).

#include "sfx.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace engine {
namespace sfx {

namespace {

constexpr double TWO_PI = 6.283185307179586;

void normalizeTo(std::vector<float>& frames, float peak) {
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

// The per-surface recipe. Times in seconds, levels relative; the builder below renders any of them.
struct Recipe {
    double length;          // clip length
    // heel: a click of band-limited noise at contact
    double clickLevel, clickLo, clickHi, clickDecay;
    // body: a low sine thump (the weight arriving)
    double thumpLevel, thumpHz, thumpDecay;
    // texture: continuous noise through a band, shaped by attack/decay
    double texLevel, texLo, texHi, texAttack, texDecay;
    // grains: discrete micro-impacts (gravel, snow crystals, grass stems) at `grainRate` per second
    // over `grainSpan`, each a tiny ringing click in the band
    double grainLevel, grainRate, grainSpan, grainLo, grainHi;
};

Recipe recipeFor(Ground g) {
    switch (g) {
        //                      len   click lo    hi     dec    thump Hz   dec   tex  lo     hi     att    dec    grain rate  span  lo     hi
        case Ground::Asphalt:  return {0.20, 0.70, 900, 6000, 0.006, 0.40, 95, 0.030, 0.40, 1800, 7000, 0.002, 0.045, 0.25, 900, 0.05, 2500, 8000};
        case Ground::Concrete: return {0.18, 1.00, 1200, 9000, 0.007, 0.28, 110, 0.022, 0.20, 2500, 8000, 0.001, 0.020, 0.00, 0, 0, 0, 0};
        case Ground::Grass:    return {0.30, 0.10, 400, 2500, 0.006, 0.30, 70, 0.045, 0.55, 1500, 6500, 0.012, 0.090, 0.35, 260, 0.16, 2000, 7000};
        case Ground::Dirt:     return {0.22, 0.25, 250, 1800, 0.006, 0.80, 75, 0.050, 0.35, 300, 1800, 0.004, 0.060, 0.20, 180, 0.08, 400, 2500};
        case Ground::Sand:     return {0.32, 0.05, 300, 2000, 0.008, 0.25, 60, 0.050, 0.75, 1200, 4500, 0.030, 0.110, 0.10, 400, 0.18, 1500, 5000};
        case Ground::Rock:     return {0.26, 0.70, 1000, 7000, 0.003, 0.40, 120, 0.020, 0.10, 2000, 8000, 0.002, 0.030, 0.85, 700, 0.12, 1500, 9000};
        case Ground::Snow:     return {0.36, 0.05, 500, 3000, 0.008, 0.35, 65, 0.060, 0.30, 800, 5000, 0.020, 0.120, 0.95, 1400, 0.24, 900, 6000};
        case Ground::Wood:     return {0.24, 0.70, 500, 5000, 0.004, 0.70, 180, 0.060, 0.10, 400, 2500, 0.002, 0.040, 0.00, 0, 0, 0, 0};
        case Ground::Metal:    return {0.40, 0.85, 1500, 9000, 0.003, 0.30, 240, 0.150, 0.20, 2500, 9000, 0.001, 0.200, 0.00, 0, 0, 0, 0};
        default: break;
    }
    return recipeFor(Ground::Concrete);
}

// Render a recipe. `scale`: overall stretch (landings last longer), `weight`: thump multiplier,
// `thumpDrop`: thump pitch multiplier (a heavy fall is lower), `texBoost`: texture multiplier.
std::vector<float> render(const Recipe& r, uint32_t sampleRate, uint32_t seed, double scale, double weight,
                          double thumpDrop, double texBoost, double clickBoost) {
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * r.length * scale);
    std::vector<float> out(count, 0.0f);
    std::mt19937 rng(seed * 2246822519u + 3266489917u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::uniform_real_distribution<double> jit(0.88, 1.14);
    const double pj = jit(rng);   // each step a slightly different shoe, a slightly different spot
    BandPass click(r.clickLo * pj, r.clickHi * pj, rate);
    BandPass tex(r.texLo * pj, r.texHi * pj, rate);
    const double thumpHz = r.thumpHz * pj * thumpDrop;
    // grains: onset times drawn up front (Poisson-ish), each a decaying burst of band noise
    std::vector<std::pair<size_t, double>> grains;
    if (r.grainLevel > 0 && r.grainRate > 0) {
        std::exponential_distribution<double> gap(r.grainRate);
        std::uniform_real_distribution<double> amp(0.3, 1.0);
        const double span = r.grainSpan * scale;
        for (double t = gap(rng) * 0.3; t < span; t += gap(rng)) {
            // denser at the start (the weight rolling onto the foot), thinning out
            const double fall = std::exp(-2.5 * t / std::max(1e-3, span));
            grains.push_back({static_cast<size_t>(t * rate), amp(rng) * fall});
        }
    }
    BandPass grainBand(r.grainLo * pj, std::max(r.grainLo + 1.0, r.grainHi) * pj, rate);
    std::vector<double> grainEnv(count, 0.0);
    const double grainTau = 0.0025;   // each grain rings ~2.5 ms
    for (const auto& [at, a] : grains)
        for (size_t i = at; i < count && i < at + static_cast<size_t>(rate * grainTau * 6); ++i)
            grainEnv[i] += a * std::exp(-(static_cast<double>(i - at) / rate) / grainTau);
    for (size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) / rate;
        const double n = uni(rng);
        double v = 0.0;
        v += clickBoost * r.clickLevel * click(n) * std::exp(-t / r.clickDecay);
        v += weight * r.thumpLevel * std::sin(TWO_PI * thumpHz * t) * std::exp(-t / (r.thumpDecay * std::sqrt(scale)));
        const double att = std::min(1.0, t / std::max(1e-4, r.texAttack));
        v += texBoost * r.texLevel * 3.0 * tex(uni(rng)) * att * std::exp(-t / (r.texDecay * scale));
        v += r.grainLevel * 4.0 * grainBand(uni(rng)) * grainEnv[i];
        // fade the last 8 ms so no clip ends on a step
        const double tail = std::min(1.0, (static_cast<double>(count - i)) / (rate * 0.008));
        out[i] = static_cast<float>(std::tanh(1.4 * v) * tail);
    }
    normalizeTo(out, 0.85f);
    return out;
}

// Circular box filter and helpers for the LOOPS (same trick as sfx::engine: wrapped windows make the
// filtered noise exactly periodic, so the seam is just another sample).
std::vector<double> circBox(const std::vector<double>& in, int window) {
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
std::vector<double> circBand(const std::vector<double>& white, double rate, double loHz, double hiHz) {
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
std::vector<double> circEnvelope(size_t count, std::mt19937& rng, int minCycles, int maxCycles, int terms) {
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

}  // namespace

const char* groundName(Ground g) {
    switch (g) {
        case Ground::Asphalt: return "asphalt";
        case Ground::Concrete: return "concrete";
        case Ground::Grass: return "grass";
        case Ground::Dirt: return "dirt";
        case Ground::Sand: return "sand";
        case Ground::Rock: return "rock";
        case Ground::Snow: return "snow";
        case Ground::Wood: return "wood";
        case Ground::Metal: return "metal";
        default: return "concrete";
    }
}

std::vector<float> footstep(Ground ground, uint32_t sampleRate, uint32_t seed) {
    return render(recipeFor(ground), sampleRate, seed, 1.0, 1.0, 1.0, 1.0, 1.0);
}

std::vector<float> jumpPush(Ground ground, uint32_t sampleRate, uint32_t seed) {
    // a scuff: the surface's texture pushed hard and short, the heel's click, little weight
    return render(recipeFor(ground), sampleRate, seed + 7001u, 0.7, 0.25, 1.2, 1.6, 1.3);
}

std::vector<float> landing(Ground ground, double heavy, uint32_t sampleRate, uint32_t seed) {
    heavy = std::clamp(heavy, 0.0, 1.0);
    // a hop lands like a firm step; a drop off a wall is a deep body thud that rings longer, the
    // surface's texture (sand spraying, snow crushing, gravel scattering) louder with it
    return render(recipeFor(ground), sampleRate, seed + 9001u, 1.0 + 1.2 * heavy, 1.4 + 2.2 * heavy,
                  1.0 - 0.45 * heavy, 1.2 + 0.8 * heavy, 1.0 + 0.3 * heavy);
}

std::vector<float> grassRustle(uint32_t sampleRate, uint32_t seed) {
    // Stems brushing the legs: bright band noise whose loudness flickers fast (each stem) under a slower
    // sway (the stride), all periodic over the buffer.
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 2.0);
    std::mt19937 rng(seed * 747796405u + 2891336453u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);
    const std::vector<double> band = circBand(white, rate, 1800.0, 7500.0);
    std::vector<double> flickWhite(count);
    for (double& w : flickWhite) w = std::fabs(uni(rng));
    const std::vector<double> flicker = circBox(flickWhite, static_cast<int>(rate / 40.0));   // ~25 ms stems
    const std::vector<double> sway = circEnvelope(count, rng, 2, 5, 3);
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        const double env = std::max(0.0, 0.55 + 0.35 * sway[i] / 2.0) * (0.3 + 1.7 * flicker[i]);
        out[i] = static_cast<float>(std::tanh(0.5 * band[i] * env));
    }
    normalizeTo(out, 0.7f);
    return out;
}

std::vector<float> surf(uint32_t sampleRate, uint32_t seed) {
    // Two swells per loop: each rises as a low roar, breaks into a bright hiss, and drains away as a
    // fizz. Two noise colours whose balance follows the swell's phase.
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 11.0);
    std::mt19937 rng(seed * 1664525u + 1013904223u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);
    const std::vector<double> roar = circBand(white, rate, 60.0, 700.0);
    const std::vector<double> hiss = circBand(white, rate, 1500.0, 9000.0);
    const std::vector<double> wobble = circEnvelope(count, rng, 3, 9, 3);
    std::uniform_real_distribution<double> ph(0.0, 1.0);
    const double off = ph(rng) * 0.2;
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        const double p = static_cast<double>(i) / static_cast<double>(count);
        // two swells: phase within the current one, 0..1
        const double w = std::fmod(p * 2.0 + off, 1.0);
        const double rise = std::pow(std::sin(0.5 * TWO_PI * std::min(1.0, w / 0.55)), 2.0);   // build, peak at the break
        const double breakHiss = w > 0.35 ? std::exp(-(w - 0.45) * (w - 0.45) / 0.012) : 0.0;
        const double drain = w > 0.5 ? std::exp(-(w - 0.5) * 4.0) : 0.0;
        const double wob = 1.0 + 0.12 * wobble[i] / 2.0;
        const double v = (0.35 + 0.65 * rise) * 0.8 * roar[i] * wob + (0.9 * breakHiss + 0.35 * drain + 0.05) * hiss[i];
        out[i] = static_cast<float>(std::tanh(0.45 * v));
    }
    normalizeTo(out, 0.7f);
    return out;
}

std::vector<float> river(uint32_t sampleRate, uint32_t seed) {
    // Running water: a mid-band wash that never stops, plus BUBBLES -- short rising sine chirps (the
    // resonance of a closing air pocket), scattered through the loop. Bubbles near the end wrap round.
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 4.0);
    std::mt19937 rng(seed * 22695477u + 1u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);
    const std::vector<double> wash = circBand(white, rate, 250.0, 3000.0);
    const std::vector<double> sparkle = circBand(white, rate, 3000.0, 9000.0);
    const std::vector<double> swell = circEnvelope(count, rng, 2, 7, 4);
    std::vector<double> acc(count, 0.0);
    std::uniform_real_distribution<double> at(0.0, 1.0), hz(350.0, 1400.0), amp(0.2, 1.0), dur(0.008, 0.03);
    const int bubbles = 160;
    for (int b = 0; b < bubbles; ++b) {
        const size_t start = static_cast<size_t>(at(rng) * count);
        const double f0 = hz(rng), a = amp(rng), d = dur(rng);
        const size_t len = static_cast<size_t>(d * rate * 4);
        double phase = 0;
        for (size_t k = 0; k < len; ++k) {
            const double t = static_cast<double>(k) / rate;
            const double f = f0 * (1.0 + 2.5 * t / (d * 4));   // rising
            phase += TWO_PI * f / rate;
            acc[(start + k) % count] += a * std::sin(phase) * std::exp(-t / d);
        }
    }
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        const double s = 1.0 + 0.15 * swell[i] / 2.0;
        const double v = 0.8 * wash[i] * s + 0.25 * sparkle[i] + 0.9 * acc[i];
        out[i] = static_cast<float>(std::tanh(0.45 * v));
    }
    normalizeTo(out, 0.7f);
    return out;
}

}  // namespace sfx
}  // namespace engine
