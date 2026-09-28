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
// HARD ground (asphalt, concrete, rock, wood, metal) is IMPACTS: a heel strike, then the toe ~0.09 s
// later, each a sharp click plus a short RING at the pitch the shoe and the surface make together --
// with next to no noise (#84: built from noise, every surface read as grass). SOFT ground (grass, dirt,
// sand, snow) is one rolled contact carried by its TEXTURE: noise or grains.
struct Recipe {
    double length = 0.25;
    // heel: a click of band-limited noise at contact
    double clickLevel = 0, clickLo = 1000, clickHi = 6000, clickDecay = 0.003;
    // ring: a damped sinusoid pair at the contact (the shoe and the surface ringing together)
    double ringLevel = 0, ringHz = 1500, ringHz2 = 0, ringDecay = 0.006;
    // toe: the same click + ring again, `toeDelay` later at `toeLevel` of the heel (0 = one contact)
    double toeDelay = 0, toeLevel = 0;
    // body: a low sine thump (the weight arriving)
    double thumpLevel = 0, thumpHz = 90, thumpDecay = 0.03;
    // texture: continuous noise through a band, shaped by attack/decay
    double texLevel = 0, texLo = 1500, texHi = 6000, texAttack = 0.002, texDecay = 0.04;
    // grains: discrete micro-impacts (gravel, snow crystals, grass stems) at `grainRate` per second
    // over `grainSpan`, each a tiny ringing click in the band
    double grainLevel = 0, grainRate = 0, grainSpan = 0, grainLo = 2000, grainHi = 8000;
};

Recipe recipeFor(Ground g) {
    Recipe r;
    switch (g) {
        case Ground::Asphalt:   // a dull heel click with grit under it
            r.length = 0.22; r.clickLevel = 0.8; r.clickLo = 1200; r.clickHi = 6000; r.clickDecay = 0.003;
            r.ringLevel = 0.35; r.ringHz = 900; r.ringDecay = 0.005; r.toeDelay = 0.085; r.toeLevel = 0.55;
            r.thumpLevel = 0.30; r.thumpHz = 85; r.thumpDecay = 0.02;
            r.texLevel = 0.06; r.texLo = 2000; r.texHi = 7000; r.texDecay = 0.02;
            r.grainLevel = 0.12; r.grainRate = 700; r.grainSpan = 0.03; r.grainLo = 3000; r.grainHi = 9000; break;
        case Ground::Concrete:  // the clean, bright click-clack of a pavement
            r.length = 0.20; r.clickLevel = 1.0; r.clickLo = 2000; r.clickHi = 10000; r.clickDecay = 0.0022;
            r.ringLevel = 0.55; r.ringHz = 1700; r.ringHz2 = 3100; r.ringDecay = 0.006; r.toeDelay = 0.09; r.toeLevel = 0.6;
            r.thumpLevel = 0.22; r.thumpHz = 100; r.thumpDecay = 0.015; r.texLevel = 0.02; break;
        case Ground::Rock:      // a hard knock on stone, a little loose grit scattering
            r.length = 0.24; r.clickLevel = 0.9; r.clickLo = 1500; r.clickHi = 8000; r.clickDecay = 0.0025;
            r.ringLevel = 0.5; r.ringHz = 2300; r.ringHz2 = 3700; r.ringDecay = 0.004; r.toeDelay = 0.095; r.toeLevel = 0.5;
            r.thumpLevel = 0.35; r.thumpHz = 120; r.thumpDecay = 0.02;
            r.grainLevel = 0.22; r.grainRate = 300; r.grainSpan = 0.06; r.grainLo = 2500; r.grainHi = 9000; break;
        case Ground::Wood:      // a hollow knock
            r.length = 0.26; r.clickLevel = 0.6; r.clickLo = 800; r.clickHi = 5000; r.clickDecay = 0.003;
            r.ringLevel = 0.8; r.ringHz = 420; r.ringHz2 = 1050; r.ringDecay = 0.022; r.toeDelay = 0.1; r.toeLevel = 0.6;
            r.thumpLevel = 0.5; r.thumpHz = 150; r.thumpDecay = 0.04; break;
        case Ground::Metal:     // a clang that rings on
            r.length = 0.45; r.clickLevel = 0.8; r.clickLo = 2000; r.clickHi = 10000; r.clickDecay = 0.002;
            r.ringLevel = 0.6; r.ringHz = 1250; r.ringHz2 = 2890; r.ringDecay = 0.12; r.toeDelay = 0.1; r.toeLevel = 0.5;
            r.thumpLevel = 0.2; r.thumpHz = 240; r.thumpDecay = 0.1; break;
        case Ground::Grass:     // a soft press and the swish of blades
            r.length = 0.30; r.clickLevel = 0.10; r.clickLo = 400; r.clickHi = 2500; r.clickDecay = 0.006;
            r.thumpLevel = 0.30; r.thumpHz = 70; r.thumpDecay = 0.045;
            r.texLevel = 0.55; r.texLo = 1500; r.texHi = 6500; r.texAttack = 0.012; r.texDecay = 0.09;
            r.grainLevel = 0.35; r.grainRate = 260; r.grainSpan = 0.16; r.grainLo = 2000; r.grainHi = 7000; break;
        case Ground::Dirt:      // a dull thud and a crumble
            r.length = 0.22; r.clickLevel = 0.25; r.clickLo = 250; r.clickHi = 1800; r.clickDecay = 0.006;
            r.thumpLevel = 0.80; r.thumpHz = 75; r.thumpDecay = 0.05;
            r.texLevel = 0.35; r.texLo = 300; r.texHi = 1800; r.texAttack = 0.004; r.texDecay = 0.06;
            r.grainLevel = 0.20; r.grainRate = 180; r.grainSpan = 0.08; r.grainLo = 400; r.grainHi = 2500; break;
        case Ground::Sand:      // a soft shush
            r.length = 0.32; r.clickLevel = 0.05; r.clickLo = 300; r.clickHi = 2000; r.clickDecay = 0.008;
            r.thumpLevel = 0.25; r.thumpHz = 60; r.thumpDecay = 0.05;
            r.texLevel = 0.75; r.texLo = 1200; r.texHi = 4500; r.texAttack = 0.03; r.texDecay = 0.11;
            r.grainLevel = 0.10; r.grainRate = 400; r.grainSpan = 0.18; r.grainLo = 1500; r.grainHi = 5000; break;
        case Ground::Snow:      // the compressed crunch
            r.length = 0.36; r.clickLevel = 0.05; r.clickLo = 500; r.clickHi = 3000; r.clickDecay = 0.008;
            r.thumpLevel = 0.35; r.thumpHz = 65; r.thumpDecay = 0.06;
            r.texLevel = 0.30; r.texLo = 800; r.texHi = 5000; r.texAttack = 0.02; r.texDecay = 0.12;
            r.grainLevel = 0.95; r.grainRate = 1400; r.grainSpan = 0.24; r.grainLo = 900; r.grainHi = 6000; break;
        default: return recipeFor(Ground::Concrete);
    }
    return r;
}

// Render a recipe. `scale`: overall stretch (landings last longer), `weight`: thump multiplier,
// `thumpDrop`: thump pitch multiplier (a heavy fall is lower), `texBoost`: texture multiplier,
// `toeDelayScale`: 1 = a walking step; small = heel and toe land together (a landing, flat-footed);
// `scrape`: a burst of noise in the surface's own click band (a push-off drags the sole across it).
std::vector<float> render(const Recipe& r, uint32_t sampleRate, uint32_t seed, double scale, double weight,
                          double thumpDrop, double texBoost, double clickBoost, double toeDelayScale = 1.0,
                          double scrape = 0.0) {
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * r.length * scale);
    std::vector<float> out(count, 0.0f);
    std::mt19937 rng(seed * 2246822519u + 3266489917u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::uniform_real_distribution<double> jit(0.88, 1.14);
    const double pj = jit(rng);   // each step a slightly different shoe, a slightly different spot
    BandPass click(r.clickLo * pj, r.clickHi * pj, rate);
    BandPass scrapeBand(r.clickLo * pj, r.clickHi * pj, rate);
    BandPass tex(r.texLo * pj, r.texHi * pj, rate);
    const double thumpHz = r.thumpHz * pj * thumpDrop;
    const double ringHz = r.ringHz * pj, ringHz2 = r.ringHz2 * pj;
    const double toeAt = r.toeDelay * toeDelayScale * jit(rng);
    const double toeGain = r.toeLevel * (0.85 + 0.3 * (jit(rng) - 0.88) / 0.26);
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
    // one contact (heel, or toe `delay` later at `gain`): click envelope + the surface's ring
    auto contact = [&](double t, double delay, double gain, double noise) {
        if (t < delay || gain <= 0) return 0.0;
        const double u = t - delay;
        double v = clickBoost * r.clickLevel * noise * std::exp(-u / r.clickDecay);
        if (r.ringLevel > 0) {
            double ring = std::sin(TWO_PI * ringHz * u);
            if (ringHz2 > 0) ring = 0.65 * ring + 0.35 * std::sin(TWO_PI * ringHz2 * u);
            v += r.ringLevel * ring * std::exp(-u / r.ringDecay);
        }
        return gain * v;
    };
    for (size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) / rate;
        const double cn = click(uni(rng));
        double v = contact(t, 0.0, 1.0, cn);
        if (r.toeDelay > 0) v += contact(t, toeAt, toeGain, cn);
        v += weight * r.thumpLevel * std::sin(TWO_PI * thumpHz * t) * std::exp(-t / (r.thumpDecay * std::sqrt(scale)));
        const double att = std::min(1.0, t / std::max(1e-4, r.texAttack));
        v += texBoost * r.texLevel * 3.0 * tex(uni(rng)) * att * std::exp(-t / (r.texDecay * scale));
        v += r.grainLevel * 4.0 * grainBand(uni(rng)) * grainEnv[i];
        if (scrape > 0) v += scrape * 2.5 * scrapeBand(uni(rng)) * std::min(1.0, t / 0.004) * std::exp(-t / 0.035);
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
    // a scuff: the sole dragged across the surface in ITS band (bright on stone, dull on earth) with
    // the surface's own texture pushed hard and short, only the toe's contact, little weight
    Recipe r = recipeFor(ground);
    r.toeDelay = 0;   // one contact: the ball of the foot
    return render(r, sampleRate, seed + 7001u, 0.7, 0.25, 1.2, 1.6, 1.0, 1.0, 0.8);
}

std::vector<float> landing(Ground ground, double heavy, uint32_t sampleRate, uint32_t seed) {
    heavy = std::clamp(heavy, 0.0, 1.0);
    // a hop lands like a firm step; a drop off a wall is a deep body thud that rings longer, the
    // surface's texture (sand spraying, snow crushing, gravel scattering) louder with it
    // both feet, flat: heel and toe land nearly together
    return render(recipeFor(ground), sampleRate, seed + 9001u, 1.0 + 1.2 * heavy, 1.4 + 2.2 * heavy,
                  1.0 - 0.45 * heavy, 1.2 + 0.8 * heavy, 1.0 + 0.3 * heavy, 0.25);
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

std::vector<float> river(uint32_t sampleRate, uint32_t seed, double size) {
    // #87 (Glenn: "It's like a low noise ... More of a river sound would be nice"). Water sounds like
    // water because of BUBBLES: an air pocket closing under the surface rings at its Minnaert resonance,
    // f = 3.26 / r (Hz, r in metres), damped fast (van den Doel: d = 0.043 f + 0.0014 f^1.5 per second)
    // and rising a little in pitch as it rises. A stream is thousands of small ones a second; a big river
    // fewer, larger, lower ones over the roar of the whole body of water moving. All wrapped round the
    // loop, so it repeats seamlessly.
    size = std::clamp(size, 0.0, 1.0);
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 4.0);
    std::mt19937 rng(seed * 22695477u + 1u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0), u01(0.0, 1.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);
    // the bed: the water's body moving (low roar, stronger for a big river) and surface hiss
    const std::vector<double> roar = circBand(white, rate, 60.0, 450.0);
    const std::vector<double> hiss = circBand(white, rate, 2500.0, 9000.0);
    // THE RUSH: broad mid-band noise, swelling as the water tumbles -- what a river mostly IS to the ear
    std::vector<double> white2(count);
    for (double& w : white2) w = uni(rng);
    const std::vector<double> rush = circBand(white2, rate, 250.0, 3500.0);
    const std::vector<double> tumble = circEnvelope(count, rng, 6, 18, 5);
    const std::vector<double> swell = circEnvelope(count, rng, 2, 7, 4);
    std::vector<double> acc(count, 0.0);
    // GURGLES, not a boil (Glenn: "sounds like a pot of boiling water"): a boiling pot is exactly an even
    // rain of small independent bubbles. In a river the bubbles come in BURSTS where water tumbles over a
    // stone and traps air -- a few at once, larger (lower) ones, then a gap -- over a rushing wash that
    // carries most of the sound. So: burst events (Poisson), each a cluster of bubbles within ~60 ms.
    const double rMin = 0.0025 + 0.0020 * size, rMax = 0.008 + 0.009 * size;   // metres: ~200-1300 Hz
    const double burstsPerSecond = 13.0 - 8.0 * size;
    std::poisson_distribution<int> perBurst(4.0 + 3.0 * size);
    std::exponential_distribution<double> gap(burstsPerSecond);
    for (double tb = gap(rng); tb < 4.0; tb += gap(rng)) {
        const int n = std::max(1, perBurst(rng));
        const double burstAmp = 0.4 + 0.6 * u01(rng);
        for (int b = 0; b < n; ++b) {
            const double a = -1.5, u = u01(rng);
            const double r = std::pow(std::pow(rMin, a) + u * (std::pow(rMax, a) - std::pow(rMin, a)), 1.0 / a);
            const double f0 = 3.26 / r;
            const double d = 0.043 * f0 + 0.0014 * std::pow(f0, 1.5);
            const double amp = burstAmp * (r / rMax) * (0.5 + 0.5 * u01(rng));
            const double rise = 0.15 * d;
            const size_t start = static_cast<size_t>((tb + 0.06 * u01(rng)) * rate) % count;
            const size_t len = static_cast<size_t>(std::min(0.12, 5.0 / d) * rate);
            double phase = 0;
            for (size_t k = 0; k < len; ++k) {
                const double t = static_cast<double>(k) / rate;
                phase += 6.283185307179586 * f0 * (1.0 + rise * t) / rate;
                acc[(start + k) % count] += amp * std::sin(phase) * std::exp(-d * t);
            }
        }
    }
    // splashes: short broadband bursts where the water breaks over a stone (more in a fast stream)
    const int splashes = static_cast<int>(4.0 * (18.0 - 12.0 * size));
    for (int s2 = 0; s2 < splashes; ++s2) {
        const size_t start = static_cast<size_t>(u01(rng) * count);
        const double a = 0.2 + 0.4 * u01(rng);
        const size_t len = static_cast<size_t>(0.04 * rate);
        for (size_t k = 0; k < len; ++k)
            acc[(start + k) % count] += a * hiss[(start + k) % count] * std::exp(-static_cast<double>(k) / (0.008 * rate));
    }
    double bubbleRms = 0;
    for (double x : acc) bubbleRms += x * x;
    bubbleRms = std::sqrt(bubbleRms / count);
    const double norm = bubbleRms > 1e-9 ? 1.0 / bubbleRms : 1.0;
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        const double sw = 1.0 + 0.15 * swell[i] / 2.0;
        const double tum = std::max(0.2, 0.75 + 0.25 * tumble[i] / 2.5);
        const double v = (0.25 + 0.75 * size) * roar[i] * sw + (0.9 - 0.3 * size) * rush[i] * tum + 0.10 * hiss[i] +
                         0.45 * acc[i] * norm;
        out[i] = static_cast<float>(std::tanh(0.4 * v));
    }
    normalizeTo(out, 0.7f);
    return out;
}

}  // namespace sfx
}  // namespace engine
