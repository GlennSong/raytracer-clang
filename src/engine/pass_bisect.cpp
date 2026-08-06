#include "pass_bisect.h"

#include "../log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace engine {

constexpr PassBisect::Step PassBisect::STEPS[];

void PassBisect::begin(FrameContext& ctx) {
    savedSsao = ctx.renderer.ssaoEnabled;
    savedSsr = ctx.renderer.ssrEnabled;
    savedBloom = ctx.renderer.bloomEnabled;
    framebufferPixels = ctx.framebufferWidth * ctx.framebufferHeight;
    // Unlock presentation first: with vsync on, frame time is a multiple of
    // the refresh interval and the whole measurement is meaningless.
    syncDisabled = ctx.renderer.setPresentSync(false);
    if (!syncDisabled)
        LOG_WARN << "pass bisect: renderer would not disable presentation "
                    "sync; results will be lower bounds";
    step = 0;
    elapsed = 0.0;
    samples.clear();
    medians.clear();
    result.clear();
    state = Running;
    applyStep(ctx, 0);
}

void PassBisect::restore(FrameContext& ctx) {
    ctx.renderer.ssaoEnabled = savedSsao;
    ctx.renderer.ssrEnabled = savedSsr;
    ctx.renderer.bloomEnabled = savedBloom;
    if (syncDisabled) {
        ctx.renderer.setPresentSync(true);
        syncDisabled = false;
    }
}

void PassBisect::cancel(FrameContext& ctx) {
    restore(ctx);
    state = Idle;
    samples.clear();
    medians.clear();
}

void PassBisect::applyStep(FrameContext& ctx, int index) {
    ctx.renderer.ssaoEnabled = STEPS[index].ssao;
    ctx.renderer.ssrEnabled = STEPS[index].ssr;
    ctx.renderer.bloomEnabled = STEPS[index].bloom;
}

void PassBisect::update(FrameContext& ctx) {
    if (state != Running) return;

    elapsed += ctx.frameDelta;
    // Discard the settling window: right after a toggle the GPU is still
    // draining the previous configuration's frames.
    if (elapsed > SETTLE_SECONDS)
        samples.push_back(ctx.stats.lastFrame().totalMs);

    if (elapsed < STEP_SECONDS) return;

    float median = 0.0f;
    if (!samples.empty()) {
        std::nth_element(samples.begin(), samples.begin() + samples.size() / 2,
                         samples.end());
        median = samples[samples.size() / 2];
    }
    medians.push_back(median);
    samples.clear();
    elapsed = 0.0;

    if (++step >= STEP_COUNT) {
        restore(ctx);
        finish();
        return;
    }
    applyStep(ctx, step);
}

// Do the measured times look quantised to a display refresh interval? If so
// the numbers describe the display, not the passes, and must not be reported
// as pass costs. Checked against common refresh rates rather than assuming
// 60 Hz — a ProMotion panel is 120.
static bool looksVsyncQuantised(const std::vector<float>& medians) {
    for (double interval : {1000.0 / 60.0, 1000.0 / 120.0, 1000.0 / 90.0}) {
        bool all = true;
        for (float ms : medians) {
            const double k = ms / interval;
            const double nearest = std::floor(k + 0.5);
            if (nearest < 0.9 || std::fabs(k - nearest) > 0.06) { all = false; break; }
        }
        if (all) return true;
    }
    return false;
}

void PassBisect::finish() {
    state = Finished;
    if (medians.empty()) { result = "bisect produced no samples"; return; }

    char buf[640];
    std::snprintf(buf, sizeof(buf), "measured at %.2fM pixels%s\n",
                  framebufferPixels / 1.0e6,
                  syncDisabled ? "" : " (vsync NOT disabled)");
    result = buf;

    // Disabling a pass cannot make the frame SLOWER. If one did, the numbers
    // are not measuring the pass — they are measuring something else moving
    // underneath (typically the frame flipping across a refresh boundary).
    // This catches cases the quantisation test alone misses: a real run read
    // 17.11 / 33.27 / 17.81 / 33.20, where only the boundary was moving.
    bool negativeSaving = false;
    for (int i = 1; i < static_cast<int>(medians.size()); i++)
        if (medians[i] > medians[0] * 1.05f) negativeSaving = true;

    // Failing to disable sync is NOT by itself a reason to discard the run:
    // if the frame is well over the refresh interval the times still vary
    // continuously and rank correctly — they just understate each saving,
    // because part of it is absorbed by the wait. Only genuinely artefactual
    // numbers (quantised, or a negative cost) are refused. Throwing away a
    // sound measurement because one flag was off wasted a real result once.
    if (negativeSaving || looksVsyncQuantised(medians)) {
        result += negativeSaving
            ? "UNRELIABLE: disabling a pass made frames SLOWER, which is\n"
              "impossible — the frame is flipping across a refresh boundary,\n"
              "so these times measure the display, not the passes. Measured:\n"
            : "UNRELIABLE: frame times are quantised to the display refresh,\n"
              "so this cannot rank passes — a real saving reads as 0.00.\n"
              "Measured:\n";
        for (int i = 0; i < static_cast<int>(medians.size()); i++) {
            std::snprintf(buf, sizeof(buf), "  %-18s %6.2f ms\n",
                          STEPS[i].name, medians[i]);
            result += buf;
        }
        result += syncDisabled
            ? "Sync WAS disabled and the times still look like this — the\n"
              "display may be pacing elsewhere; use a GPU capture."
            : "Presentation sync could not be disabled here, which is the\n"
              "likely cause. Shrink the window (fewer pixels moves the frame\n"
              "off the refresh boundary) and re-run, or use a GPU capture.";
        return;
    }

    const float baseline = medians[0];
    std::snprintf(buf, sizeof(buf), "baseline %.2f ms/frame (median)\n",
                  baseline);
    result += buf;

    // Rank the passes by cost, so the answer is the FIRST line read rather
    // than something to work out from four numbers.
    std::vector<std::pair<float, const char*>> ranked;
    for (int i = 1; i < static_cast<int>(medians.size()); i++)
        ranked.emplace_back(baseline - medians[i], STEPS[i].name);
    std::sort(ranked.begin(), ranked.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    for (const auto& [saved, name] : ranked) {
        std::snprintf(buf, sizeof(buf), "%-18s costs %5.2f ms  (%.0f%%)\n",
                      name, saved,
                      baseline > 0 ? 100.0 * saved / baseline : 0.0);
        result += buf;
    }
    if (!syncDisabled)
        result += "note: presentation sync could not be disabled, so each\n"
                  "figure is a LOWER bound — part of a saving is absorbed by\n"
                  "the wait for the display.";
}

const char* PassBisect::label() const {
    return (state == Running && step < STEP_COUNT) ? STEPS[step].name : "done";
}

float PassBisect::secondsLeft() const {
    if (state != Running) return 0.0f;
    const double remaining =
        (STEP_COUNT - step) * STEP_SECONDS - elapsed;
    return static_cast<float>(remaining > 0 ? remaining : 0);
}

}  // namespace engine
