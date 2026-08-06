#include "pass_bisect.h"

#include <algorithm>
#include <cstdio>

namespace engine {

constexpr PassBisect::Step PassBisect::STEPS[];

void PassBisect::begin(FrameContext& ctx) {
    savedSsao = ctx.renderer.ssaoEnabled;
    savedSsr = ctx.renderer.ssrEnabled;
    savedBloom = ctx.renderer.bloomEnabled;
    step = 0;
    elapsed = 0.0;
    samples.clear();
    medians.clear();
    result.clear();
    state = Running;
    applyStep(ctx, 0);
}

void PassBisect::cancel(FrameContext& ctx) {
    ctx.renderer.ssaoEnabled = savedSsao;
    ctx.renderer.ssrEnabled = savedSsr;
    ctx.renderer.bloomEnabled = savedBloom;
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
        ctx.renderer.ssaoEnabled = savedSsao;
        ctx.renderer.ssrEnabled = savedSsr;
        ctx.renderer.bloomEnabled = savedBloom;
        finish();
        return;
    }
    applyStep(ctx, step);
}

void PassBisect::finish() {
    state = Finished;
    if (medians.empty()) { result = "bisect produced no samples"; return; }

    const float baseline = medians[0];
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "baseline %.2f ms/frame (median)\n", baseline);
    result = buf;
    for (int i = 1; i < static_cast<int>(medians.size()); i++) {
        // Turning a pass OFF should make frames FASTER; the drop is that
        // pass's cost. A near-zero drop means the pass is not what's
        // expensive — or the frame is pinned to a display interval, which
        // hides everything smaller than one interval (the report warns about
        // that separately).
        const float saved = baseline - medians[i];
        std::snprintf(buf, sizeof(buf), "%-18s %6.2f ms  (saves %+.2f ms)\n",
                      STEPS[i].name, medians[i], saved);
        result += buf;
    }
    result += "note: if every saving reads ~0 while frames sit on a display\n"
              "interval, vsync is masking the difference — uncap or shrink the\n"
              "window and re-run.";
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
