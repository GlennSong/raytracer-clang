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
    // dull: a steep extra low-pass (Hz, three poles) on the click and texture -- a one-pole band leaks too much top
    // for a soft sole; the toe also gets its own share of the thump
    double dull = 0;
    // grainDull: the same three-pole low-pass on the grains (Hz; 0 = none) -- grit under a boot, not glass beads
    double grainDull = 0;
};

Recipe recipeFor(Ground g) {
    Recipe r;
    switch (g) {
        // A SHOE ON PAVEMENT IS A LOW THUD (#98 round 2, Glenn: "walking on the sidewalk still sounds like I'm walking
        // on spindly legs"). Measured against a CC0 recording of shoes on concrete (Kenney "Impact Sounds",
        // footstep_concrete_000-004; see CREDITS.md -- used to measure, not shipped): one impact ~20 ms long, the energy at
        // 160-640 Hz, centroid 160-360 Hz, almost nothing over 1.2 kHz. Ours centred at 2.7-4.8 kHz and rang for 85 ms:
        // four octaves too bright, which is what read as small hard legs. Now: a rubber-soled slap in the low mids
        // (the click band low and steepened by `dull`), a thump an octave up from before so small speakers carry it,
        // no ring, and the toe the same slap again, softer. Asphalt a little darker with a trace of grit.
        case Ground::Asphalt:   // a heavy, dull footfall, then the grit under the sole rolling a moment longer
            r.length = 0.18; r.clickLevel = 0.9; r.clickLo = 90; r.clickHi = 400; r.clickDecay = 0.007; r.dull = 700;
            r.toeDelay = 0.09; r.toeLevel = 0.8;
            r.thumpLevel = 0.55; r.thumpHz = 125; r.thumpDecay = 0.011;
            r.texLevel = 0.03; r.texLo = 300; r.texHi = 1200; r.texAttack = 0.003; r.texDecay = 0.02;
            r.grainLevel = 0.10; r.grainRate = 420; r.grainSpan = 0.07; r.grainLo = 500; r.grainHi = 1600; break;
        case Ground::Concrete:  // a firm pavement step: a clean low slap
            r.length = 0.16; r.clickLevel = 0.9; r.clickLo = 170; r.clickHi = 800; r.clickDecay = 0.005; r.dull = 1150;
            r.toeDelay = 0.082; r.toeLevel = 0.8;
            r.thumpLevel = 0.45; r.thumpHz = 170; r.thumpDecay = 0.008;
            r.texLevel = 0.03; r.texLo = 500; r.texHi = 1600; r.texAttack = 0.003; r.texDecay = 0.012; break;
        // OFF THE PAVEMENT, TOO (Glenn, 2026-10-09: "the mountain terrain still has that spindly walking noise (too high
        // pitched)"). The natural grounds were still the first by-ear recipes: rock centred at 5.8 kHz, grass 9.4, sand
        // 8.2, snow 9.9 -- hiss and ticks. Measured against the same CC0 Kenney set (footstep_grass_000-004: centroid
        // 245-425 Hz, 1-6% over 1.25 kHz, 16-39 ms; footstep_snow_000-004: 240-500 Hz, 1-5%, 100-140 ms): a step on
        // anything is a low thud first, and the ground's character is in the low mids -- how long it lasts, how much
        // it crunches -- not up in the treble. No rock or dirt recording in the set: rock is the concrete slap a little
        // brighter with grit, dirt a darker, longer thud.
        case Ground::Rock:      // a firm knock on stone, a little grit under the sole
            r.length = 0.20; r.clickLevel = 0.9; r.clickLo = 300; r.clickHi = 2000; r.clickDecay = 0.005; r.dull = 2200;
            r.toeDelay = 0.09; r.toeLevel = 0.75;
            r.thumpLevel = 0.35; r.thumpHz = 150; r.thumpDecay = 0.02;
            r.grainLevel = 0.20; r.grainRate = 220; r.grainSpan = 0.20; r.grainLo = 500; r.grainHi = 1700; r.grainDull = 1500; break;
        case Ground::Wood:      // a hollow knock
            r.length = 0.26; r.clickLevel = 0.6; r.clickLo = 800; r.clickHi = 5000; r.clickDecay = 0.003;
            r.ringLevel = 0.8; r.ringHz = 420; r.ringHz2 = 1050; r.ringDecay = 0.022; r.toeDelay = 0.1; r.toeLevel = 0.6;
            r.thumpLevel = 0.5; r.thumpHz = 150; r.thumpDecay = 0.04; break;
        case Ground::Metal:     // a clang that rings on
            r.length = 0.45; r.clickLevel = 0.8; r.clickLo = 2000; r.clickHi = 10000; r.clickDecay = 0.002;
            r.ringLevel = 0.6; r.ringHz = 1250; r.ringHz2 = 2890; r.ringDecay = 0.12; r.toeDelay = 0.1; r.toeLevel = 0.5;
            r.thumpLevel = 0.2; r.thumpHz = 240; r.thumpDecay = 0.1; break;
        case Ground::Grass:     // a soft press, the blades brushed under the sole
            r.length = 0.20; r.clickLevel = 0.30; r.clickLo = 150; r.clickHi = 900; r.clickDecay = 0.008; r.dull = 1300;
            r.thumpLevel = 0.40; r.thumpHz = 95; r.thumpDecay = 0.03;
            r.texLevel = 0.45; r.texLo = 350; r.texHi = 1800; r.texAttack = 0.010; r.texDecay = 0.045;
            r.grainLevel = 0.10; r.grainRate = 200; r.grainSpan = 0.10; r.grainLo = 500; r.grainHi = 1800; r.grainDull = 1500; break;
        case Ground::Dirt:      // a dull thud and a crumble
            r.length = 0.22; r.clickLevel = 0.30; r.clickLo = 100; r.clickHi = 700; r.clickDecay = 0.007; r.dull = 800;
            r.thumpLevel = 0.80; r.thumpHz = 80; r.thumpDecay = 0.035;
            r.texLevel = 0.30; r.texLo = 200; r.texHi = 1000; r.texAttack = 0.004; r.texDecay = 0.05;
            r.grainLevel = 0.18; r.grainRate = 160; r.grainSpan = 0.09; r.grainLo = 300; r.grainHi = 1300; r.grainDull = 1200; break;
        case Ground::Sand:      // a soft, sliding shush
            r.length = 0.40; r.clickLevel = 0.10; r.clickLo = 100; r.clickHi = 600; r.clickDecay = 0.010; r.dull = 900;
            r.thumpLevel = 0.30; r.thumpHz = 70; r.thumpDecay = 0.04;
            r.texLevel = 0.75; r.texLo = 200; r.texHi = 1200; r.texAttack = 0.02; r.texDecay = 0.16; break;
        case Ground::Snow:      // the compressed crunch, squeaking down through the pack
            r.length = 0.30; r.clickLevel = 0.10; r.clickLo = 150; r.clickHi = 800; r.clickDecay = 0.008; r.dull = 1000;
            r.thumpLevel = 0.40; r.thumpHz = 75; r.thumpDecay = 0.05;
            r.texLevel = 0.15; r.texLo = 300; r.texHi = 1500; r.texAttack = 0.02; r.texDecay = 0.10;
            r.grainLevel = 0.85; r.grainRate = 900; r.grainSpan = 0.13; r.grainLo = 250; r.grainHi = 1000; r.grainDull = 850; break;
        case Ground::Water:     // wading: the foot pushed through the water, a slosh and a few drops
            r.length = 0.34; r.clickLevel = 0.10; r.clickLo = 400; r.clickHi = 2500; r.clickDecay = 0.006;
            r.thumpLevel = 0.30; r.thumpHz = 90; r.thumpDecay = 0.05;
            r.texLevel = 0.60; r.texLo = 250; r.texHi = 2200; r.texAttack = 0.03; r.texDecay = 0.12;
            r.grainLevel = 0.25; r.grainRate = 60; r.grainSpan = 0.25; r.grainLo = 900; r.grainHi = 3000; break;
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
    const double dullHz = r.dull > 0 ? r.dull * pj : rate;
    LowPass dc1(dullHz, rate), dc2(dullHz, rate), dc3(dullHz, rate), dt1(dullHz, rate), dt2(dullHz, rate), dt3(dullHz, rate);
    const double grainDullHz = r.grainDull > 0 ? r.grainDull * pj : rate;
    LowPass dg1(grainDullHz, rate), dg2(grainDullHz, rate), dg3(grainDullHz, rate);
    LowPass ds1(dullHz, rate), ds2(dullHz, rate), ds3(dullHz, rate);   // the scrape, as dull as the click
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
        const double cn = r.dull > 0 ? dc3(dc2(dc1(click(uni(rng))))) * 2.2 : click(uni(rng));
        double v = contact(t, 0.0, 1.0, cn);
        if (r.toeDelay > 0) v += contact(t, toeAt, toeGain, cn);
        v += weight * r.thumpLevel * std::sin(TWO_PI * thumpHz * t) * std::exp(-t / (r.thumpDecay * std::sqrt(scale)));
        if (r.dull > 0 && r.toeDelay > 0 && t >= toeAt)
            v += 0.6 * toeGain * weight * r.thumpLevel * std::sin(TWO_PI * thumpHz * 1.15 * (t - toeAt)) *
                 std::exp(-(t - toeAt) / (r.thumpDecay * std::sqrt(scale)));
        const double att = std::min(1.0, t / std::max(1e-4, r.texAttack));
        const double tn = r.dull > 0 ? dt3(dt2(dt1(tex(uni(rng))))) * 1.5 : tex(uni(rng));
        v += texBoost * r.texLevel * 3.0 * tn * att * std::exp(-t / (r.texDecay * scale));
        const double gn = r.grainDull > 0 ? dg3(dg2(dg1(grainBand(uni(rng))))) * 2.0 : grainBand(uni(rng));
        v += r.grainLevel * 4.0 * gn * grainEnv[i];
        if (scrape > 0) {
            const double sn = r.dull > 0 ? ds3(ds2(ds1(scrapeBand(uni(rng))))) * 2.2 : scrapeBand(uni(rng));
            v += scrape * 2.5 * sn * std::min(1.0, t / 0.004) * std::exp(-t / 0.035);
        }
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
        case Ground::Water: return "water";
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
    // Stems brushing the legs: band noise whose loudness flickers fast (each stem) under a slower sway (the
    // stride), all periodic over the buffer. A swish in the low mids (2026-10-09: it was 1.8-7.5 kHz, a hiss
    // centred at 4 kHz under every step through a meadow -- the "spindly" sound off the pavement).
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 2.0);
    std::mt19937 rng(seed * 747796405u + 2891336453u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);
    const std::vector<double> band = circBand(circBand(white, rate, 450.0, 2800.0), rate, 450.0, 2800.0);
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

// One LAYER of bubbling for river(): bursts (Poisson, `bursts` a second) of ~`perBurst` bubbles each,
// radii between rMin and rMax (power law: many small), each ringing at its Minnaert frequency with a
// real bubble's damping and a slight rising chirp; wrapped round the loop. Then, for a DISTANT layer,
// low-passed at `lowpassHz` (0 = none) circularly (two passes of a one-pole round the loop, so the
// filter state at the seam is the settled one).
std::vector<double> bubbleLayer(size_t count, double rate, std::mt19937& rng, double bursts, double perBurst,
                                double rMin, double rMax, double lowpassHz) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> acc(count, 0.0);
    std::poisson_distribution<int> nOf(perBurst);
    std::exponential_distribution<double> gap(bursts);
    const double seconds = static_cast<double>(count) / rate;
    for (double tb = gap(rng); tb < seconds; tb += gap(rng)) {
        const int n = std::max(1, nOf(rng));
        const double burstAmp = 0.5 + 0.5 * u01(rng);
        for (int b = 0; b < n; ++b) {
            const double a = -1.5, u = u01(rng);
            const double r = std::pow(std::pow(rMin, a) + u * (std::pow(rMax, a) - std::pow(rMin, a)), 1.0 / a);
            const double f0 = 3.26 / r;
            const double d = 0.043 * f0 + 0.0014 * std::pow(f0, 1.5);
            const double amp = burstAmp * std::sqrt(r / rMax) * (0.5 + 0.5 * u01(rng));
            const double rise = 0.12 * d;
            const size_t start = static_cast<size_t>((tb + 0.08 * u01(rng)) * rate) % count;
            const size_t len = static_cast<size_t>(std::min(0.15, 6.0 / d) * rate);
            double phase = 0;
            for (size_t k = 0; k < len; ++k) {
                const double t = static_cast<double>(k) / rate;
                // a soft onset (~2 ms): a bubble swells into its ring, it doesn't click
                const double on = std::min(1.0, t / 0.002);
                phase += 6.283185307179586 * f0 * (1.0 + rise * t) / rate;
                acc[(start + k) % count] += amp * on * std::sin(phase) * std::exp(-d * t);
            }
        }
    }
    if (lowpassHz > 0) {
        const double al = 1.0 - std::exp(-6.283185307179586 * lowpassHz / rate);
        double y = 0;
        for (int pass = 0; pass < 2; ++pass)
            for (size_t k = 0; k < count; ++k) { y += al * (acc[k] - y); if (pass == 1) acc[k] = y; }
    }
    double rms = 0;
    for (double x : acc) rms += x * x;
    rms = std::sqrt(rms / std::max<size_t>(1, count));
    if (rms > 1e-12) for (double& x : acc) x /= rms;
    return acc;
}

namespace {
// The river's layered bubbling (#87, tuned by ear with Glenn). `open` = heard in the air: the same low bubbles
// un-muffled, plus the surface's own clear sounds -- sparse trickle plinks and a little babble. Closed = heard
// from under the water: everything muffled to the deep bubbling (Glenn: "river5 works good for underwater").
std::vector<float> riverLayers(uint32_t sampleRate, uint32_t seed, double size, bool open) {
    size = std::clamp(size, 0.0, 1.0);
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 4.0);
    std::mt19937 rng(seed * 22695477u + 1u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    // LOW bubbles: big (7-25 mm, ~130-450 Hz), each a deep bloop that rings on; a broad river's bigger
    const double big = 1.0 + 0.35 * size;
    const std::vector<double> nearL = bubbleLayer(count, rate, rng, 3.0 + 1.0 * size, 1.6, 0.0075 * big, 0.020 * big,
                                                  open ? 0.0 : 700.0);
    const std::vector<double> midL = bubbleLayer(count, rate, rng, 14.0 + 8.0 * size, 2.2, 0.0070 * big, 0.022 * big,
                                                 open ? 0.0 : 900.0);
    const std::vector<double> farL = bubbleLayer(count, rate, rng, 60.0 + 50.0 * size, 3.0, 0.0090 * big, 0.025 * big,
                                                 open ? 2200.0 : 450.0 - 150.0 * size);
    // the surface, in the air only: trickles -- small bubbles breaking at the top (2.5-5 mm, ~650-1300 Hz), sparse
    // and clear, so the water sounds lively without the dense fizz of a boil
    const std::vector<double> trickle = open ? bubbleLayer(count, rate, rng, 5.0 - 2.0 * size, 1.5, 0.0025, 0.0050, 0.0)
                                             : std::vector<double>(count, 0.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);
    const std::vector<double> bed = circBand(white, rate, 120.0, 900.0);
    // babble: the surface's mid-band chatter, fast-modulated (not a flat bed, which read as static)
    std::vector<double> white2(count);
    for (double& w : white2) w = uni(rng);
    const std::vector<double> chatter = circBand(white2, rate, 500.0, 2500.0);
    const std::vector<double> chatterAm = circEnvelope(count, rng, 40, 110, 6);
    const std::vector<double> swell = circEnvelope(count, rng, 2, 6, 3);
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        const double sw = 1.0 + 0.2 * swell[i] / 2.0;
        double v = 0.30 * nearL[i] + 0.40 * midL[i] * sw + (0.55 + 0.25 * size) * farL[i] * sw + 0.03 * bed[i] * sw;
        if (open) {
            const double am = std::max(0.0, chatterAm[i] / 3.0);
            v += 0.40 * trickle[i] + 0.18 * chatter[i] * am * am;
        }
        out[i] = static_cast<float>(std::tanh(0.45 * v));
    }
    normalizeTo(out, 0.7f);
    return out;
}
}  // namespace

std::vector<float> simmer(uint32_t sampleRate, uint32_t seed, double boil) {
    // Glenn, of the bubbling river: "It sounds like something cooking ... so you may want to mark the sounds
    // for that, but that's not a river". Kept for a pot on a stove: the layered bubbling heard in the air.
    return riverLayers(sampleRate, seed, boil, true);
}

std::vector<float> river(uint32_t sampleRate, uint32_t seed, double size) {
    // A RIVER, by ear with Glenn (#87): after bubbles (a boiling pot, then cooking), bright noise ("static"),
    // and a swelling roar ("sounds like the sea") -- "maybe it should just be a low static noise". A dark,
    // soft, STEADY noise, no slow swell (that is waves), a faint quick flutter so the water moves.
    // The WIDE river is the dark 60-700 Hz body he picked ("the first one sounds like a wide river"); a
    // NARROW one is lighter, not bigger: the rumble thinned out (a little water doesn't roar), a touch more
    // air and a livelier flutter.
    size = std::clamp(size, 0.0, 1.0);
    const double narrow = 1.0 - size;
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 4.0);
    std::mt19937 rng(seed * 22695477u + 1u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> white(count), white2(count);
    for (double& w : white) w = uni(rng);
    for (double& w : white2) w = uni(rng);
    const std::vector<double> body = circBand(white, rate, 60.0 + 110.0 * narrow, 700.0 + 200.0 * narrow);
    const std::vector<double> lift = circBand(white2, rate, 400.0, 1100.0);
    const std::vector<double> flutter = circEnvelope(count, rng, 30 + static_cast<int>(40 * narrow), 90 + static_cast<int>(60 * narrow), 6);
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        const double fl = 1.0 + (0.06 + 0.08 * narrow) * flutter[i] / 3.0;
        const double v = body[i] * fl + (0.10 + 0.14 * narrow) * lift[i] * fl;
        out[i] = static_cast<float>(std::tanh(0.35 * v));
    }
    normalizeTo(out, 0.6f);
    return out;
}

std::vector<float> splash(double strength, uint32_t sampleRate, uint32_t seed) {
    // A body meeting the water (#43): the SLAP (a dull broadband crack as the surface is struck), the SPRAY
    // (droplets flung up and falling back: a scatter of tiny high plinks over ~0.3-0.8 s), and the GULP (air
    // dragged under: a few low bubbles). A stroke is a soft slap and a little spray; jumping in, all three big.
    strength = std::clamp(strength, 0.0, 1.0);
    const double rate = static_cast<double>(sampleRate);
    const double len = 0.35 + 0.6 * strength;
    const auto count = static_cast<size_t>(rate * len);
    std::mt19937 rng(seed * 2654435761u + 97u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0), u01(0.0, 1.0);
    std::vector<double> acc(count, 0.0);
    BandPass slapBand(250.0, 2500.0 + 2500.0 * strength, rate);
    const double slapDecay = 0.02 + 0.05 * strength;
    for (size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) / rate;
        const double att = std::min(1.0, t / (0.004 + 0.01 * strength));
        acc[i] += (0.5 + 0.8 * strength) * slapBand(uni(rng)) * att * std::exp(-t / slapDecay);
    }
    // droplets: small bubbles ringing as they land (Minnaert, 1-3 mm), strewn over the fall-back time
    const int drops = static_cast<int>(8 + 60 * strength);
    for (int d = 0; d < drops; ++d) {
        const double at = 0.02 + u01(rng) * (0.15 + 0.75 * strength);
        const double r = 0.0010 + 0.0020 * u01(rng);
        const double f0 = 3.26 / r, damp = 0.043 * f0 + 0.0014 * std::pow(f0, 1.5);
        const double a = (0.08 + 0.12 * u01(rng)) * (1.0 + 1.5 * strength) * (1.0 - 0.5 * at / len);
        double ph = 0;
        for (size_t k = static_cast<size_t>(at * rate), n = 0; k < count && n < static_cast<size_t>(rate * 0.03); ++k, ++n) {
            const double t = static_cast<double>(n) / rate;
            ph += 6.283185307179586 * f0 * (1.0 + 0.1 * damp * t) / rate;
            acc[k] += a * std::sin(ph) * std::exp(-damp * t);
        }
    }
    // the gulp: a few big bubbles under the body (only for a real plunge)
    const int gulps = static_cast<int>(6 * strength);
    for (int g = 0; g < gulps; ++g) {
        const double at = 0.05 + 0.3 * u01(rng);
        const double r = 0.010 + 0.012 * u01(rng);
        const double f0 = 3.26 / r, damp = 0.043 * f0 + 0.0014 * std::pow(f0, 1.5);
        double ph = 0;
        for (size_t k = static_cast<size_t>(at * rate), n = 0; k < count && n < static_cast<size_t>(rate * 0.2); ++k, ++n) {
            const double t = static_cast<double>(n) / rate;
            ph += 6.283185307179586 * f0 * (1.0 + 0.15 * damp * t) / rate;
            acc[k] += 0.7 * std::sin(ph) * std::exp(-damp * t);
        }
    }
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        const double tail = std::min(1.0, static_cast<double>(count - i) / (rate * 0.02));
        out[i] = static_cast<float>(std::tanh(1.2 * acc[i]) * tail);
    }
    normalizeTo(out, 0.8f);
    return out;
}

std::vector<float> lap(uint32_t sampleRate, uint32_t seed) {
    // A lake's edge: still water, so no surf -- small wavelets arriving every 1.5-3 s, each a soft low slosh
    // (rising and falling over ~0.5 s) and a few drops as it runs back. Seamless: each wavelet wraps round.
    const double rate = static_cast<double>(sampleRate);
    const auto count = static_cast<size_t>(rate * 12.0);
    std::mt19937 rng(seed * 40503u + 7u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0), u01(0.0, 1.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);
    const std::vector<double> slosh = circBand(white, rate, 120.0, 1400.0);
    std::vector<double> env(count, 0.15);   // the lake is never quite silent
    std::vector<std::pair<size_t, double>> drops;
    for (double t = u01(rng) * 1.5; t < 12.0; t += 1.5 + 1.5 * u01(rng)) {
        const double a = 0.5 + 0.5 * u01(rng), w = 0.45 + 0.25 * u01(rng);
        for (size_t k = 0; k < static_cast<size_t>(w * 2.0 * rate); ++k) {
            const double x = static_cast<double>(k) / (w * rate);   // 0..2
            env[(static_cast<size_t>(t * rate) + k) % count] += a * std::pow(std::sin(0.5 * 3.14159265 * std::min(x, 2.0)), 2.0);
        }
        for (int d = 0; d < 3; ++d) drops.push_back({static_cast<size_t>((t + w + 0.3 * u01(rng)) * rate) % count, 0.05 + 0.08 * u01(rng)});
    }
    std::vector<double> acc(count, 0.0);
    for (size_t i = 0; i < count; ++i) acc[i] = slosh[i] * env[i];
    for (const auto& [at, a] : drops) {
        const double r = 0.0015 + 0.002 * u01(rng);
        const double f0 = 3.26 / r, damp = 0.043 * f0 + 0.0014 * std::pow(f0, 1.5);
        double ph = 0;
        for (size_t n = 0; n < static_cast<size_t>(rate * 0.03); ++n) {
            const double t = static_cast<double>(n) / rate;
            ph += 6.283185307179586 * f0 / rate;
            acc[(at + n) % count] += a * std::sin(ph) * std::exp(-damp * t);
        }
    }
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) out[i] = static_cast<float>(std::tanh(0.6 * acc[i]));
    normalizeTo(out, 0.6f);
    return out;
}

std::vector<float> underwaterRiver(uint32_t sampleRate, uint32_t seed) {
    return riverLayers(sampleRate, seed, 0.5, false);
}

}  // namespace sfx
}  // namespace engine
