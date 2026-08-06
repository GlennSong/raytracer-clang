#include "pass_bisect.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

    // Refuse to report pass costs that are really display artefacts.
    if (!syncDisabled || negativeSaving || looksVsyncQuantised(medians)) {
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
        result += !syncDisabled
            ? "This backend cannot disable presentation sync, so use a GPU\n"
              "capture (Xcode) to rank passes instead."
            : "Sync was disabled but the times still look quantised — the\n"
              "display may be pacing elsewhere; try a GPU capture.";
        return;
    }

    const float baseline = medians[0];
    std::snprintf(buf, sizeof(buf), "baseline %.2f ms/frame (median)\n",
                  baseline);
    result += buf;
    for (int i = 1; i < static_cast<int>(medians.size()); i++) {
        // Turning a pass OFF makes frames faster; the drop IS that pass's
        // cost. With sync off this is a real measurement, not a boundary flip.
        const float saved = baseline - medians[i];
        std::snprintf(buf, sizeof(buf),
                      "%-18s %6.2f ms  (that pass costs %+.2f ms)\n",
                      STEPS[i].name, medians[i], saved);
        result += buf;
    }
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
