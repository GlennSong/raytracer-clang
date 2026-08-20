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

std::vector<float> horn(uint32_t sampleRate, uint32_t seed) {
    // One exact period. Loop-cleanliness comes from quantizing every partial to
    // an integer cycle count over `count` samples — the nominal frequencies are
    // then only approximated (by well under 2 Hz at 0.25 s), which no ear
    // notices, but the wrap seam is mathematically continuous at any rate.
    const auto count = static_cast<size_t>(sampleRate / 4);   // ~0.25 s
    std::vector<float> frames(count);

    // The dyad: real dual-trumpet horns sit around a 400/500 Hz pair (roughly a
    // major third). The seed shifts the whole horn a step so different presses
    // of the same street don't sound cloned from one car.
    const double bases[][2] = {{370.0, 466.0}, {392.0, 494.0},
                               {415.0, 523.0}, {440.0, 554.0}};
    const double* base = bases[seed % 4];

    // Brassy voice per note: falling harmonic series, odd partials favoured.
    const double gains[5] = {1.0, 0.55, 0.40, 0.18, 0.10};

    for (size_t i = 0; i < count; i++) {
        const double phase01 = static_cast<double>(i) / count;   // one period
        double v = 0.0;
        for (int note = 0; note < 2; note++) {
            const double cyclesBase = base[note] * 0.25;   // cycles per period
            for (int h = 0; h < 5; h++) {
                const double cycles = std::round(cyclesBase * (h + 1));
                v += gains[h] * std::sin(TWO_PI * cycles * phase01);
            }
        }
        // Soft clip: the flattened peaks are the honk's bite.
        frames[i] = static_cast<float>(std::tanh(1.6 * 0.28 * v));
    }
    normalize(frames, 0.85f);
    return frames;
}

std::vector<float> engine(uint32_t sampleRate, uint32_t seed) {
    // An engine is not a chord. A stack of steady sines is a HUM (device:
    // "it just sounds like a constant hum") — what makes the real thing read
    // as an engine is that it is a train of discrete COMBUSTION PULSES, each
    // one slightly different from the last, riding a bed of broadband
    // induction noise. All three live here.
    //
    // The loop still has to be seamless, so: tonal partials keep integer
    // cycle counts over the buffer, and the noise is filtered CIRCULARLY
    // (wrapped indices), which makes it exactly periodic by construction
    // rather than approximately so. A 0.75 s period holds 42 firings at
    // kEngineRefHz — long enough that the ear stops hearing the wrap, and
    // long enough to carry per-firing variation.
    const double periodSeconds = 0.75;
    const auto count = static_cast<size_t>(sampleRate * periodSeconds);
    std::vector<float> frames(count);
    const double fireCycles = std::round(kEngineRefHz * periodSeconds);   // 42
    const auto fireCount = static_cast<int>(fireCycles);

    std::mt19937 rng(seed * 2654435761u + 12345u);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::vector<double> white(count);
    for (double& w : white) w = uni(rng);

    // Circular moving average: a box filter whose window wraps the buffer, so
    // the output is periodic to the last sample. Windows are in samples, so
    // the shaped bands land at the same FREQUENCIES on any device rate.
    auto circBox = [&](const std::vector<double>& in, int window) {
        const auto n = static_cast<int>(in.size());
        const int half = std::max(1, std::min(window / 2, n / 2 - 1));
        std::vector<double> out(in.size());
        double sum = 0.0;
        for (int k = -half; k <= half; k++) sum += in[((k % n) + n) % n];
        const double norm = 1.0 / (2 * half + 1);
        for (int i = 0; i < n; i++) {
            out[static_cast<size_t>(i)] = sum * norm;
            sum += in[static_cast<size_t>((i + half + 1) % n)] -
                   in[static_cast<size_t>(((i - half) % n + n) % n)];
        }
        return out;
    };
    auto unitRms = [](std::vector<double>& v) {
        double s = 0.0;
        for (double x : v) s += x * x;
        const double rms = std::sqrt(s / std::max<size_t>(1, v.size()));
        if (rms > 1e-12)
            for (double& x : v) x /= rms;
    };

    // Two noise beds: a deep exhaust/body rumble, and a mid-band induction
    // rasp (wide box minus narrow box = a band, still circular).
    const double rate = static_cast<double>(sampleRate);
    std::vector<double> rumble = circBox(white, static_cast<int>(rate / 220.0));
    std::vector<double> rasp = circBox(white, static_cast<int>(rate / 5200.0));
    std::vector<double> raspLow = circBox(white, static_cast<int>(rate / 900.0));
    for (size_t i = 0; i < count; i++) rasp[i] -= raspLow[i];
    unitRms(rumble);
    unitRms(rasp);

    // CYCLE-TO-CYCLE VARIATION: no two combustion events are equal — this is
    // most of what stops a loop sounding mechanical. One gain per firing, so
    // the pattern repeats only with the whole buffer.
    std::uniform_real_distribution<double> vary(0.74, 1.26);
    std::vector<double> fireGain(static_cast<size_t>(fireCount));
    for (double& g : fireGain) g = vary(rng);

    // The tonal core: the half-order lope (one per revolution — the lumpiness
    // you hear at tickover) plus the firing fundamental and its harmonics.
    struct Partial { double cycles; double gain; };
    const Partial partials[] = {
        {fireCycles * 0.5, 0.50}, {fireCycles, 1.00},
        {fireCycles * 2, 0.55},   {fireCycles * 3, 0.30},
        {fireCycles * 4, 0.20},   {fireCycles * 5, 0.12},
    };
    // Slow breathing over the whole period (integer cycles, so still clean).
    std::uniform_real_distribution<double> phase(0.0, TWO_PI);
    const double breathA = phase(rng), breathB = phase(rng);

    for (size_t i = 0; i < count; i++) {
        const double phase01 = static_cast<double>(i) / static_cast<double>(count);

        // Where we are in the current firing, and which firing it is.
        const double firePos = phase01 * fireCycles;
        const auto fireIdx = static_cast<int>(firePos) % fireCount;
        const double inFire = firePos - std::floor(firePos);
        // Pressure pulse: fast rise, exponential blow-down — a puff, not a
        // sine. Starting at zero keeps every firing boundary continuous.
        const double attack = std::min(1.0, inFire / 0.07);
        const double puff =
            attack * std::exp(-inFire * 5.0) * fireGain[static_cast<size_t>(fireIdx)];

        double tone = 0.0;
        for (const Partial& p : partials)
            tone += p.gain * std::sin(TWO_PI * p.cycles * phase01);

        const double breathe = 1.0 + 0.07 * std::sin(TWO_PI * phase01 + breathA) +
                               0.05 * std::sin(TWO_PI * 3.0 * phase01 + breathB);
        // Noise is GATED by the pulse train: each firing is an audible event
        // rather than a continuous hiss laid over a drone.
        const double bed = rumble[i] * 0.85 + rasp[i] * 0.22;
        const double v = (tone * (0.45 + 0.55 * puff) + bed * (0.25 + 0.95 * puff)) *
                         breathe;
        // Soft clip: combustion is not a sine — the flattened peaks growl.
        frames[i] = static_cast<float>(std::tanh(0.55 * v));
    }
    normalize(frames, 0.80f);
    return frames;
}

}  // namespace sfx
}  // namespace engine
