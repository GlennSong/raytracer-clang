#ifndef RAYTRACER_ENGINE_PASS_COST_H
#define RAYTRACER_ENGINE_PASS_COST_H

#include "system.h"
#include "frame_stats.h"

#include <string>
#include <vector>

namespace engine {

// Measures what each screen-space post pass COSTS: turn one off, time the
// difference, put it back. Remove one ingredient and see what changes.
//
// (Originally named "PassBisect", which was simply wrong — bisecting is
// halving a search space, which this does not do. A name that misdescribes
// its own mechanism confuses every later reader, so it was renamed rather
// than left in place.)
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
class PassCost {
public:
    enum State { Idle, Running, Finished };

    // How long each configuration is held, and how much of the front of that
    // window is thrown away while the GPU pipeline settles.
    static constexpr double STEP_SECONDS = 2.0;
    static constexpr double SETTLE_SECONDS = 0.4;

    State state = Idle;
    std::string result;
    // True while the run has presentation sync switched off. Measuring frame
    // time with vsync ON cannot rank passes: the frame is quantised to the
    // refresh interval, so a real 4 ms saving reads as 0.00 — and a frame
    // sitting exactly ON the boundary flips between one and two intervals,
    // which reads as a pass "costing" a negative amount. Both happened on a
    // real machine before this existed.
    bool syncDisabled = false;

    void begin(FrameContext& ctx);
    void update(FrameContext& ctx);
    void cancel(FrameContext& ctx);

    const char* label() const;
    float secondsLeft() const;

    // Number of configurations a run measures (baseline + one per pass).
    static constexpr int STEP_COUNT = 4;

    // Structured results of the last completed run, for a caller that wants to
    // tabulate rather than print (the sweep below). Empty until Finished.
    struct Measured { const char* name; float medianMs; };
    const std::vector<Measured>& measurements() const { return measured; }
    bool trustworthy() const { return resultTrusted; }
    int pixels() const { return framebufferPixels; }

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

    int step = 0;
    double elapsed = 0.0;
    std::vector<float> samples;
    std::vector<float> medians;   // one per completed step
    bool savedSsao = true, savedSsr = true, savedBloom = true;
    int framebufferPixels = 0;    // conditions the run was measured under
    std::vector<Measured> measured;
    bool resultTrusted = false;

    void applyStep(FrameContext& ctx, int index);
    void restore(FrameContext& ctx);
    void finish();
};

// Runs PassCost at several window sizes back to back, with no human in the
// loop: resize, settle, measure every configuration, move on, then write a
// CSV and print a table.
//
// It exists because the manual procedure has too many ways to go wrong —
// forgetting which window size a number came from, collapsing the panel
// mid-run, quitting before it finished, comparing two runs taken under
// different conditions. Every one of those happened. A machine doing the same
// steps every time removes the whole class.
//
// Driven by RT_PASS_SWEEP=<out.csv>; the app runs the sweep from boot and
// quits when it is done, so it is one command and no interaction.
class PassSweep {
public:
    // Fractions of the window size the app launched with. Several sizes make
    // the pixel-scaling visible: a pass whose cost is proportional to area is
    // the one worth attacking at high resolution.
    static constexpr double SCALES[] = {1.0, 0.75, 0.5, 0.35};
    static constexpr int SCALE_COUNT = 4;
    static constexpr double RESIZE_SETTLE_SECONDS = 0.8;

    // `path` is where the CSV goes; empty disables the sweep.
    void begin(const std::string& path);
    bool active() const { return state != Idle; }

    // Drives the sweep; call once per frame. Sets `quit` when finished so an
    // automated run ends by itself.
    void update(FrameContext& ctx, Window& window, bool& quit);

private:
    enum State { Idle, Resizing, Measuring, Done };
    struct Row { int pixels; int width; int height; const char* config;
                 float medianMs; bool trusted; };

    State state = Idle;
    std::string outPath;
    int scaleIndex = 0;
    int baseWidth = 0, baseHeight = 0;
    double settle = 0.0;
    PassCost cost;
    std::vector<Row> rows;

    void writeCsv() const;
    void logSummary() const;
};

}  // namespace engine

#endif
