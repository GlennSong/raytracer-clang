#ifndef RAYTRACER_ENGINE_PASS_BISECT_H
#define RAYTRACER_ENGINE_PASS_BISECT_H

#include "system.h"
#include "frame_stats.h"

#include <string>
#include <vector>

namespace engine {

// Ranks the cost of the screen-space post passes by measurement instead of
// guesswork (ADR-0077).
//
// Why this exists: when a frame is GPU-bound, `gpu_ms` cannot be trusted (the
// timed command buffer carries the present, so it reads as an upper bound),
// and the CPU-side phases are all near zero — so nothing in the ledger says
// WHICH pass costs what. Toggling each pass by hand and squinting at a live
// FPS number is the usual answer; this does the same experiment properly:
// hold a configuration for a fixed window, discard the first frames (the
// pipeline needs to drain after a toggle), take the MEDIAN frame time of the
// rest, restore the original settings, and report a ranked delta.
//
// Median, not mean: a stray OS stall in a 2-second window would otherwise
// swamp the difference being measured.
class PassBisect {
public:
    enum State { Idle, Running, Finished };

    // How long each configuration is held, and how much of the front of that
    // window is thrown away while the GPU pipeline settles.
    static constexpr double STEP_SECONDS = 2.0;
    static constexpr double SETTLE_SECONDS = 0.4;

    State state = Idle;
    std::string result;

    void begin(FrameContext& ctx);
    void update(FrameContext& ctx);
    void cancel(FrameContext& ctx);

    const char* label() const;
    float secondsLeft() const;

private:
    struct Step {
        const char* name;
        bool ssao;
        bool ssr;
        bool bloom;
    };
    // Baseline first, then each pass disabled on its own: the delta from
    // baseline is that pass's cost. Disabling one at a time (rather than
    // enabling one at a time) measures each pass in the context it actually
    // runs in, including whatever it shares with the others.
    static constexpr Step STEPS[] = {
        {"all on (baseline)", true,  true,  true },
        {"SSAO off",          false, true,  true },
        {"SSR off",           true,  false, true },
        {"bloom off",         true,  true,  false},
    };
    static constexpr int STEP_COUNT = 4;

    int step = 0;
    double elapsed = 0.0;
    std::vector<float> samples;
    std::vector<float> medians;   // one per completed step
    bool savedSsao = true, savedSsr = true, savedBloom = true;

    void applyStep(FrameContext& ctx, int index);
    void finish();
};

}  // namespace engine

#endif
