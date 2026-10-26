// The body's moves, heard (Glenn, 2026-10-24: "different sfx would help sell these types of motions too. Like
// skateboarding, parachuting, grapple, etc. The bike sounds motorized and it shouldn't. The drifting needs some tire
// squeal"). Pure (parameters, seed) -> PCM like the rest of sfx.h: air rushing past, cloth in the wind, a canopy
// filling, the grapple's shot, bite, rope and winch, a skateboard's wheels, pop and landing, a bicycle's freewheel,
// tyres rolling and squealing. Loops are seamless by construction (circular filters, integer cycles).

#include "sfx.h"
#include "sfx_dsp.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace engine {
namespace sfx {

using namespace dsp;

namespace {

// A damped sine added into `acc` at sample `at` (wrapping round a loop when `wrap`).
void ring(std::vector<double>& acc, double rate, double atSec, double hz, double decaySec, double amp, bool wrap = false) {
    const auto n = acc.size();
    const auto start = static_cast<size_t>(atSec * rate);
    const auto len = static_cast<size_t>(decaySec * 7.0 * rate);
    for (size_t k = 0; k < len; ++k) {
        size_t i = start + k;
        if (i >= n) {
            if (!wrap) break;
            i %= n;
        }
        const double t = static_cast<double>(k) / rate;
        acc[i] += amp * std::sin(TWO_PI * hz * t) * std::exp(-t / decaySec);
    }
}

// A burst of band-limited noise (a click, a scuff) added at `atSec`.
void burst(std::vector<double>& acc, double rate, std::mt19937& rng, double atSec, double lo, double hi, double attackSec,
           double decaySec, double amp, bool wrap = false) {
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    BandPass band(lo, hi, rate);
    const auto n = acc.size();
    const auto start = static_cast<size_t>(atSec * rate);
    const auto len = static_cast<size_t>((attackSec + decaySec * 6.0) * rate);
    const double gain = 1.0 + std::sqrt(rate / (TWO_PI * std::max(hi - lo, 50.0)));   // a narrow band is quieter
    for (size_t k = 0; k < len; ++k) {
        size_t i = start + k;
        if (i >= n) {
            if (!wrap) break;
            i %= n;
        }
        const double t = static_cast<double>(k) / rate;
        const double env = t < attackSec ? t / std::max(attackSec, 1e-6) : std::exp(-(t - attackSec) / decaySec);
        acc[i] += amp * gain * band(uni(rng)) * env;
    }
}

std::vector<double> whiteNoise(size_t count, std::mt19937& rng) {
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> w(count);
    for (double& x : w) x = uni(rng);
    return w;
}

// A circEnvelope scaled to [-1, 1].
std::vector<double> unitEnvelope(size_t count, std::mt19937& rng, int minCycles, int maxCycles, int terms) {
    std::vector<double> e = circEnvelope(count, rng, minCycles, maxCycles, terms);
    double peak = 1e-9;
    for (double x : e) peak = std::max(peak, std::fabs(x));
    for (double& x : e) x /= peak;
    return e;
}

std::vector<float> finish(const std::vector<double>& acc, float peak, double drive = 1.0, double fadeSec = 0.0,
                          double rate = 48000.0) {
    std::vector<float> out(acc.size());
    for (size_t i = 0; i < acc.size(); ++i) {
        double x = drive != 1.0 ? std::tanh(drive * acc[i]) : acc[i];
        if (fadeSec > 0) x *= std::min(1.0, static_cast<double>(acc.size() - i) / (rate * fadeSec));
        out[i] = static_cast<float>(x);
    }
    normalizeTo(out, peak);
    return out;
}

}  // namespace

std::vector<float> windRush(uint32_t sampleRate, uint32_t seed) {
    // Air past the ears: a low roar (the head in the stream) and a thin hiss, both swelling with slow gusts and
    // buffeted a few times a second. The caller raises gain and pitch with airspeed -- faster is louder and brighter.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 4.0);
    std::mt19937 rng(seed * 2246822519u + 3u);
    const std::vector<double> roar = circBand(whiteNoise(count, rng), rate, 30.0, 380.0);
    const std::vector<double> hiss = circBand(whiteNoise(count, rng), rate, 900.0, 6000.0);
    const std::vector<double> gust = unitEnvelope(count, rng, 1, 3, 3);
    const std::vector<double> buffet = unitEnvelope(count, rng, 18, 40, 4);
    std::vector<double> acc(count);
    for (size_t i = 0; i < count; ++i) {
        const double g = (1.0 + 0.35 * gust[i]) * (1.0 + 0.22 * buffet[i]);
        acc[i] = g * (0.9 * roar[i] + 0.22 * hiss[i] * (1.0 + 0.3 * gust[i]));
    }
    return finish(acc, 0.7f);
}

std::vector<float> clothFlutter(double flapsHz, uint32_t sampleRate, uint32_t seed) {
    // Fabric in a hard wind: the trailing edge snapping back and forth -- a slap of low body and a crackle of the cloth
    // per flap, flapsHz times a second with a little jitter (a canopy flaps slowly, a wingsuit buzzes).
    const double rate = sampleRate;
    const double len = 2.0;
    const auto count = static_cast<size_t>(rate * len);
    std::mt19937 rng(seed * 3266489917u + 11u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const int flaps = std::max(1, static_cast<int>(std::lround(flapsHz * len)));
    const double period = len / flaps;
    const std::vector<double> crackle = circBand(whiteNoise(count, rng), rate, 350.0, 4000.0);
    const std::vector<double> body = circBand(whiteNoise(count, rng), rate, 70.0, 420.0);
    std::vector<double> env(count, 0.08);   // never quite still
    for (int f = 0; f < flaps; ++f) {
        const double at = (f + 0.15 * (u01(rng) - 0.5)) * period;
        const double a = 0.6 + 0.4 * u01(rng);
        const double w = period * 0.45;
        for (size_t k = 0; k < static_cast<size_t>(w * 2.0 * rate); ++k) {
            const double x = static_cast<double>(k) / (w * rate) - 1.0;   // -1..1 across the flap
            const double p = std::exp(-x * x * 6.0) * (x < 0 ? 1.0 : 1.0 - 0.6 * x);
            const auto i = (static_cast<size_t>(std::max(0.0, at + len) * rate) + k) % count;
            env[i] += a * p;
        }
    }
    std::vector<double> acc(count);
    for (size_t i = 0; i < count; ++i) acc[i] = env[i] * (0.55 * crackle[i] * env[i] + 0.8 * body[i]);
    return finish(acc, 0.65f, 1.4);
}

std::vector<float> canopyOpen(uint32_t sampleRate, uint32_t seed) {
    // A parachute opening: the pack's rustle as the canopy streams out, the WHUMP of it filling, the lines snapping
    // taut, then the cloth settling into a flutter. ~1.5 s, one-shot.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 1.5);
    std::mt19937 rng(seed * 2654435761u + 41u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> acc(count, 0.0);
    BandPass rustle(400.0, 5000.0, rate), tail(300.0, 3000.0, rate);
    constexpr double kFill = 0.55;
    for (size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) / rate;
        if (t < kFill + 0.05) {
            const double up = std::pow(std::min(1.0, t / kFill), 2.0) * (0.6 + 0.4 * std::sin(TWO_PI * 14.0 * t));
            acc[i] += 0.5 * rustle(uni(rng)) * up * (t > kFill ? 1.0 - (t - kFill) / 0.05 : 1.0);
        }
        if (t > kFill) {
            const double s = t - kFill;
            acc[i] += 0.35 * tail(uni(rng)) * (0.5 + 0.5 * std::sin(TWO_PI * 9.0 * s)) * std::exp(-s / 0.35);
        }
    }
    // the whump: a falling low sweep and a dark burst
    double ph = 0;
    for (size_t i = static_cast<size_t>(kFill * rate); i < count; ++i) {
        const double s = static_cast<double>(i) / rate - kFill;
        ph += TWO_PI * (55.0 + 40.0 * std::exp(-s / 0.05)) / rate;
        acc[i] += 1.1 * std::sin(ph) * std::exp(-s / 0.12) * std::min(1.0, s / 0.004);
    }
    burst(acc, rate, rng, kFill, 60.0, 900.0, 0.003, 0.08, 0.9);
    // the lines: three twangs as they take the weight
    ring(acc, rate, kFill + 0.03, 210.0, 0.15, 0.25);
    ring(acc, rate, kFill + 0.05, 262.0, 0.13, 0.2);
    ring(acc, rate, kFill + 0.08, 318.0, 0.12, 0.18);
    return finish(acc, 0.85f, 1.2, 0.05, rate);
}

std::vector<float> hookFire(uint32_t sampleRate, uint32_t seed) {
    // The grapple's shot: a launcher's thunk, then the line zipping off its spool -- ticks racing and slowing as it
    // pays out -- under a whoosh of the hook going. ~0.5 s.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.5);
    std::mt19937 rng(seed * 2246822519u + 77u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> acc(count, 0.0);
    ring(acc, rate, 0.0, 140.0, 0.03, 0.9);
    burst(acc, rate, rng, 0.0, 2000.0, 8000.0, 0.0005, 0.002, 0.6);
    burst(acc, rate, rng, 0.0, 600.0, 4000.0, 0.04, 0.12, 0.35);
    double phase = 0;
    for (size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) / rate;
        const double tickHz = 420.0 * std::exp(-t / 0.25) + 60.0;
        phase += tickHz / rate;
        if (phase >= 1.0) {
            phase -= 1.0;
            ring(acc, rate, t, 2400.0 * (0.9 + 0.2 * u01(rng)), 0.0015, 0.3 * std::exp(-t / 0.3));
        }
    }
    return finish(acc, 0.8f, 1.0, 0.03, rate);
}

std::vector<float> hookBite(uint32_t sampleRate, uint32_t seed) {
    // The hook biting into something: a sharp click, a metallic clank (inharmonic, steel), a thud and a little grit
    // chipped off. ~0.6 s; seeds vary the steel's pitch.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.6);
    std::mt19937 rng(seed * 3266489917u + 5u);
    std::uniform_real_distribution<double> jitter(0.94, 1.06);
    std::vector<double> acc(count, 0.0);
    const double base = 520.0 * jitter(rng);
    const double ratios[4] = {1.0, 2.41, 3.93, 5.6}, gains[4] = {1.0, 0.6, 0.35, 0.2};
    for (int k = 0; k < 4; ++k) ring(acc, rate, 0.0, base * ratios[k], 0.18 / (1.0 + 0.6 * k), 0.55 * gains[k]);
    burst(acc, rate, rng, 0.0, 3000.0, 9000.0, 0.0003, 0.0015, 0.8);
    ring(acc, rate, 0.0, 110.0, 0.04, 0.7);
    burst(acc, rate, rng, 0.004, 1000.0, 5000.0, 0.001, 0.03, 0.3);
    return finish(acc, 0.85f, 1.0, 0.03, rate);
}

std::vector<float> ropeTwang(uint32_t sampleRate, uint32_t seed) {
    // A slack rope snapping taut under a body: a low, lossy twang (a plucked string, Karplus-Strong, damped fast --
    // a rope, not a guitar string), the fibres creaking as they take the load, a thud at the hand. ~0.7 s.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.7);
    std::mt19937 rng(seed * 2654435761u + 19u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0), u01(0.0, 1.0);
    std::vector<double> acc(count, 0.0);
    const double f0 = 75.0 * (0.92 + 0.16 * u01(rng));
    const auto period = static_cast<size_t>(rate / f0);
    std::vector<double> line(period);
    LowPass soft(900.0, rate);
    for (double& x : line) x = soft(uni(rng));
    for (size_t i = 0; i < count; ++i) {
        // three-tap average (a rope's fibres soak up the overtones fast) and a heavy loss per pass: a thrum, not a ring
        const size_t j = i % period;
        const double prevS = line[(j + period - 1) % period], next = line[(j + 1) % period];
        const double out = line[j];
        line[j] = (0.25 * prevS + 0.5 * out + 0.25 * next) * 0.94;
        const double t = static_cast<double>(i) / rate;
        acc[i] += 0.9 * out * std::min(1.0, t / 0.002) * std::exp(-t / 0.25);
    }
    for (int g = 0; g < 40; ++g) {
        const double at = 0.15 * std::pow(u01(rng), 1.5);
        ring(acc, rate, at, 700.0 + 1800.0 * u01(rng), 0.0015, 0.12 * (1.0 - at / 0.2));
    }
    ring(acc, rate, 0.0, 60.0, 0.06, 0.6);
    return finish(acc, 0.8f, 1.0, 0.05, rate);
}

std::vector<float> reelWinch(uint32_t sampleRate, uint32_t seed) {
    // The grapple's winch reeling: a ratchet clicking 30 times a second over a small motor's whine. A seamless 1 s
    // loop; the caller's pitch follows the reel speed.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate);
    std::mt19937 rng(seed * 40503u + 13u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> acc(count, 0.0);
    constexpr int kClicks = 30;
    for (int c = 0; c < kClicks; ++c) {
        const double at = (c + 0.1 * (u01(rng) - 0.5)) / kClicks + 1.0;
        const double a = 0.7 + 0.3 * u01(rng);
        ring(acc, rate, std::fmod(at, 1.0), 3200.0, 0.0012, 0.5 * a, true);
        ring(acc, rate, std::fmod(at, 1.0), 1100.0, 0.003, 0.3 * a, true);
    }
    const std::vector<double> hiss = circBand(whiteNoise(count, rng), rate, 2000.0, 7000.0);
    for (size_t i = 0; i < count; ++i) {
        const double p = TWO_PI * static_cast<double>(i) / rate;   // integer Hz over a 1 s loop: seamless
        acc[i] += 0.14 * std::sin(180.0 * p) + 0.07 * std::sin(360.0 * p + 0.4) + 0.035 * std::sin(540.0 * p + 1.1) +
                  0.03 * hiss[i];
    }
    return finish(acc, 0.6f);
}

std::vector<float> boardRoll(uint32_t sampleRate, uint32_t seed) {
    // A skateboard's urethane wheels on concrete: a low rumble with the pavement's grit ticking through it and a
    // faint bearing whir. A seamless 2 s loop; gain and pitch by speed (a scooter's small hard wheels: higher).
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 2.0);
    std::mt19937 rng(seed * 2246822519u + 29u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const std::vector<double> rumble = circBand(whiteNoise(count, rng), rate, 50.0, 600.0);
    const std::vector<double> sway = unitEnvelope(count, rng, 2, 6, 3);
    std::vector<double> acc(count, 0.0);
    for (size_t i = 0; i < count; ++i) {
        const double p = TWO_PI * static_cast<double>(i) / rate;
        acc[i] = 0.8 * rumble[i] * (1.0 + 0.15 * sway[i]) + 0.025 * std::sin(820.0 * p) + 0.015 * std::sin(1230.0 * p);
    }
    for (int g = 0; g < 600; ++g)   // grit: ~300 grains a second
        ring(acc, rate, 2.0 * u01(rng), 1500.0 + 3500.0 * u01(rng), 0.0008, 0.06 + 0.1 * u01(rng), true);
    return finish(acc, 0.6f);
}

std::vector<float> boardClack(uint32_t sampleRate, uint32_t seed) {
    // One truck's wheels over a pavement joint: a thud, the deck's knock, a click. ~0.2 s; played once per truck.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.2);
    std::mt19937 rng(seed * 3266489917u + 31u);
    std::uniform_real_distribution<double> jitter(0.92, 1.08);
    std::vector<double> acc(count, 0.0);
    ring(acc, rate, 0.0, 120.0 * jitter(rng), 0.03, 0.8);
    ring(acc, rate, 0.0, 330.0 * jitter(rng), 0.035, 0.45);
    burst(acc, rate, rng, 0.0, 1000.0, 4000.0, 0.0005, 0.002, 0.4);
    return finish(acc, 0.7f, 1.0, 0.02, rate);
}

std::vector<float> ollie(uint32_t sampleRate, uint32_t seed) {
    // The ollie's pop: the tail struck on the ground (a sharp wooden snap) and the front foot's shoe dragging up the
    // grip tape. ~0.35 s.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.35);
    std::mt19937 rng(seed * 2654435761u + 53u);
    std::vector<double> acc(count, 0.0);
    burst(acc, rate, rng, 0.0, 1500.0, 7000.0, 0.0003, 0.0015, 1.0);
    ring(acc, rate, 0.0, 420.0, 0.04, 0.8);
    ring(acc, rate, 0.0, 980.0, 0.025, 0.45);
    ring(acc, rate, 0.0, 150.0, 0.03, 0.5);
    burst(acc, rate, rng, 0.03, 2000.0, 7000.0, 0.05, 0.04, 0.22);
    return finish(acc, 0.85f, 1.0, 0.03, rate);
}

std::vector<float> boardLand(uint32_t sampleRate, uint32_t seed) {
    // Back down: both trucks slapping the ground a hair apart, the deck's knock, the wheels rattling. ~0.4 s.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.4);
    std::mt19937 rng(seed * 2246822519u + 61u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> acc(count, 0.0);
    for (double at : {0.0, 0.018}) {
        ring(acc, rate, at, 90.0, 0.06, 0.9);
        ring(acc, rate, at, 300.0, 0.07, 0.5);
        burst(acc, rate, rng, at, 1000.0, 5000.0, 0.0004, 0.002, 0.5);
    }
    for (int g = 0; g < 50; ++g) {
        const double at = 0.03 + 0.25 * u01(rng);
        ring(acc, rate, at, 1000.0 + 3000.0 * u01(rng), 0.001, 0.15 * std::exp(-at / 0.1));
    }
    return finish(acc, 0.9f, 1.2, 0.03, rate);
}

std::vector<float> handGrab(uint32_t sampleRate, uint32_t seed) {
    // Catching hold: two palms slapping onto rock a hair apart (a dull flesh thud and a bright skin smack), the grit
    // under them, the jacket's cloth swishing as the body's weight swings in. ~0.45 s; seeds vary it.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.45);
    std::mt19937 rng(seed * 2246822519u + 7u);
    std::uniform_real_distribution<double> jitter(0.9, 1.1);
    std::vector<double> acc(count, 0.0);
    for (int h = 0; h < 2; ++h) {
        const double at = h * 0.028 * jitter(rng);
        ring(acc, rate, at, 140.0 * jitter(rng), 0.025, 0.7);                 // the hand's weight
        burst(acc, rate, rng, at, 900.0, 3500.0, 0.0004, 0.008, 0.55);        // the smack
        burst(acc, rate, rng, at + 0.004, 2500.0, 8000.0, 0.001, 0.02, 0.12);   // grit
    }
    burst(acc, rate, rng, 0.05, 500.0, 2500.0, 0.06, 0.08, 0.16);             // the cloth as the body comes in
    return finish(acc, 0.8f, 1.0, 0.03, rate);
}

std::vector<float> handHold(uint32_t sampleRate, uint32_t seed) {
    // One hand set on the next hold, climbing: a soft pat and a little grit. ~0.2 s.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.2);
    std::mt19937 rng(seed * 3266489917u + 41u);
    std::uniform_real_distribution<double> jitter(0.88, 1.12);
    std::vector<double> acc(count, 0.0);
    ring(acc, rate, 0.0, 170.0 * jitter(rng), 0.018, 0.5);
    burst(acc, rate, rng, 0.0, 1100.0, 3200.0, 0.0006, 0.006, 0.35);
    burst(acc, rate, rng, 0.003, 3000.0, 9000.0, 0.001, 0.015, 0.08);
    return finish(acc, 0.6f, 1.0, 0.02, rate);
}

std::vector<float> metalGrab(uint32_t sampleRate, uint32_t seed) {
    // A hand closing on a steel rung or a bar: the palm's slap and the tube's dull ring (a hollow pipe, damped by the
    // grip, so short and low), a faint rattle at its fixing. ~0.4 s.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 0.4);
    std::mt19937 rng(seed * 2654435761u + 67u);
    std::uniform_real_distribution<double> jitter(0.94, 1.06);
    std::vector<double> acc(count, 0.0);
    const double base = 610.0 * jitter(rng);
    const double ratios[3] = {1.0, 2.76, 5.40}, gains[3] = {1.0, 0.45, 0.2};
    for (int k = 0; k < 3; ++k) ring(acc, rate, 0.0, base * ratios[k], 0.06 / (1.0 + 0.5 * k), 0.3 * gains[k]);
    ring(acc, rate, 0.0, 150.0, 0.02, 0.6);
    burst(acc, rate, rng, 0.0, 900.0, 3500.0, 0.0004, 0.006, 0.45);
    burst(acc, rate, rng, 0.012, 2000.0, 6000.0, 0.002, 0.02, 0.07);
    return finish(acc, 0.75f, 1.0, 0.03, rate);
}

std::vector<float> freewheel(uint32_t sampleRate, uint32_t seed) {
    // A bicycle coasting: the freewheel's pawls clicking over the ratchet, 40 clicks a second at pitch 1 (the caller
    // sets pitch from the wheel's revs). A seamless 1 s loop.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate);
    std::mt19937 rng(seed * 40503u + 71u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> acc(count, 0.0);
    constexpr int kClicks = 40;
    for (int c = 0; c < kClicks; ++c) {
        const double at = static_cast<double>(c) / kClicks;
        const double a = 0.75 + 0.25 * u01(rng);
        ring(acc, rate, at, 4200.0, 0.0008, a, true);
        ring(acc, rate, at, 1900.0, 0.0025, 0.5 * a, true);
    }
    return finish(acc, 0.5f);
}

std::vector<float> tyreRoll(uint32_t sampleRate, uint32_t seed) {
    // A pneumatic tyre on asphalt: a soft broadband hum over a low roar. A seamless 2 s loop; gain and pitch by speed.
    const double rate = sampleRate;
    const auto count = static_cast<size_t>(rate * 2.0);
    std::mt19937 rng(seed * 3266489917u + 83u);
    const std::vector<double> hum = circBand(whiteNoise(count, rng), rate, 120.0, 1600.0);
    const std::vector<double> roar = circBand(whiteNoise(count, rng), rate, 40.0, 140.0);
    const std::vector<double> sway = unitEnvelope(count, rng, 2, 5, 3);
    std::vector<double> acc(count);
    for (size_t i = 0; i < count; ++i) acc[i] = (0.5 * hum[i] + 0.7 * roar[i]) * (1.0 + 0.12 * sway[i]);
    return finish(acc, 0.5f);
}

std::vector<float> tyreSqueal(uint32_t sampleRate, uint32_t seed) {
    // Rubber sliding on asphalt: a strained tone near 900 Hz with its overtones, wandering in pitch and chattering
    // (the tread sticking and slipping), over a hiss of the scrub. A seamless 2 s loop: every frequency wander is a
    // sum of whole-cycle sines, integrated exactly, so the phase comes back where it started.
    const double rate = sampleRate;
    constexpr double kLen = 2.0;
    const auto count = static_cast<size_t>(rate * kLen);
    std::mt19937 rng(seed * 2654435761u + 97u);
    std::uniform_real_distribution<double> u01(0.0, 1.0), ph(0.0, TWO_PI);
    const double f0 = std::round((860.0 + 120.0 * u01(rng)) * kLen) / kLen;   // whole cycles in the loop
    struct Term { int cycles; double amp, phase; };
    std::vector<Term> wander;
    for (int k = 0; k < 4; ++k) wander.push_back({3 + static_cast<int>(u01(rng) * 7), 0.04 * (0.4 + 0.6 * u01(rng)), ph(rng)});
    for (int k = 0; k < 3; ++k) wander.push_back({60 + static_cast<int>(u01(rng) * 60), 0.008, ph(rng)});
    std::vector<Term> chatter;
    for (int k = 0; k < 3; ++k) chatter.push_back({50 + static_cast<int>(u01(rng) * 70), 0.2 + 0.15 * u01(rng), ph(rng)});
    const std::vector<double> scrub = circBand(whiteNoise(count, rng), rate, 1500.0, 5000.0);
    std::vector<double> acc(count);
    for (size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) / rate;
        // phase = 2 pi f0 (t + sum a sin(w t + p) integrated) -- the integral of each whole-cycle sine is periodic
        double integral = 0;
        for (const Term& w : wander) {
            const double om = TWO_PI * w.cycles / kLen;
            integral += w.amp * (std::cos(w.phase) - std::cos(om * t + w.phase)) / om;
        }
        const double phase = TWO_PI * f0 * (t + integral);
        double am = 1.0;
        for (const Term& c : chatter) am += c.amp * std::sin(TWO_PI * c.cycles * t / kLen + c.phase);
        acc[i] = am * (std::sin(phase) + 0.5 * std::sin(2.0 * phase + 0.3) + 0.22 * std::sin(3.0 * phase + 1.0)) +
                 0.35 * scrub[i];
    }
    return finish(acc, 0.6f, 1.3);
}

}  // namespace sfx
}  // namespace engine
