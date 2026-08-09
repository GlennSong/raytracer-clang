#include "pass_cost.h"

#include "../log.h"
#include "../renderer/window.h"

#include <fstream>
#include <string>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace engine {

constexpr PassCost::Step PassCost::STEPS[];

void PassCost::begin(FrameContext& ctx) {
    savedSsao = ctx.renderer.ssaoEnabled;
    savedSsr = ctx.renderer.ssrEnabled;
    savedBloom = ctx.renderer.bloomEnabled;
    haveSaved = true;
    framebufferPixels = ctx.framebufferWidth * ctx.framebufferHeight;
    // Unlock presentation first: with vsync on, frame time is a multiple of
    // the refresh interval and the whole measurement is meaningless.
    syncDisabled = ctx.renderer.setPresentSync(false);
    if (!syncDisabled)
        LOG_WARN << "pass cost: renderer would not disable presentation "
                    "sync; results will be lower bounds";
    step = 0;
    elapsed = 0.0;
    samples.clear();
    medians.clear();
    result.clear();
    state = Running;
    applyStep(ctx, 0);
}

void PassCost::restore(FrameContext& ctx) {
    // Nothing captured means nothing to put back. Writing the defaults here
    // would be this class inventing a pass configuration for a measurement it
    // never took.
    if (!haveSaved) return;
    ctx.renderer.ssaoEnabled = savedSsao;
    ctx.renderer.ssrEnabled = savedSsr;
    ctx.renderer.bloomEnabled = savedBloom;
    haveSaved = false;
    if (syncDisabled) {
        ctx.renderer.setPresentSync(true);
        syncDisabled = false;
    }
}

void PassCost::cancel(FrameContext& ctx) {
    restore(ctx);
    state = Idle;
    samples.clear();
    medians.clear();
}

void PassCost::applyStep(FrameContext& ctx, int index) {
    ctx.renderer.ssaoEnabled = STEPS[index].ssao;
    ctx.renderer.ssrEnabled = STEPS[index].ssr;
    ctx.renderer.bloomEnabled = STEPS[index].bloom;
}

void PassCost::update(FrameContext& ctx) {
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

void PassCost::finish() {
    state = Finished;
    measured.clear();
    resultTrusted = false;
    for (int i = 0; i < static_cast<int>(medians.size()); i++)
        measured.push_back({STEPS[i].name, medians[i]});
    if (medians.empty()) { result = "pass-cost run produced no samples"; return; }

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

    resultTrusted = true;
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

const char* PassCost::label() const {
    return (state == Running && step < STEP_COUNT) ? STEPS[step].name : "done";
}

float PassCost::secondsLeft() const {
    if (state != Running) return 0.0f;
    const double remaining =
        (STEP_COUNT - step) * STEP_SECONDS - elapsed;
    return static_cast<float>(remaining > 0 ? remaining : 0);
}



// --- PassSweep: the same measurement at several sizes, unattended ---

constexpr double PassSweep::SCALES[];

void PassSweep::begin(const std::string& path) {
    if (path.empty()) return;
    outPath = path;
    scaleIndex = 0;
    rows.clear();
    baseWidth = baseHeight = 0;
    settle = 0.0;
    state = Resizing;
}

void PassSweep::update(FrameContext& ctx, Window& window, bool& quit) {
    if (state == Idle || state == Done) return;

    if (baseWidth == 0) {   // first tick: remember the launch size
        window.getSize(baseWidth, baseHeight);
        if (baseWidth <= 0 || baseHeight <= 0) { state = Done; return; }
        LOG_INFO("pass sweep: %d sizes from %dx%d -> %s",
                 SCALE_COUNT, baseWidth, baseHeight, outPath.c_str());
    }

    if (state == Resizing) {
        if (settle == 0.0) {
            const int w = static_cast<int>(baseWidth * SCALES[scaleIndex]);
            const int h = static_cast<int>(baseHeight * SCALES[scaleIndex]);
            if (!window.setSize(w, h)) {
                LOG_WARN << "pass sweep: window cannot be resized by the app "
                            "(hosted window?); measuring at the current size only";
                // Not fatal — one size still yields a ranking.
            }
        }
        settle += ctx.frameDelta;
        // The framebuffer, swapchain and offscreen targets all resize
        // asynchronously; measuring before they settle times the resize.
        if (settle < RESIZE_SETTLE_SECONDS) return;
        settle = 0.0;
        cost.begin(ctx);
        state = Measuring;
        return;
    }

    cost.update(ctx);
    if (cost.state != PassCost::Finished) return;

    const int fbW = ctx.framebufferWidth;
    const int fbH = ctx.framebufferHeight;
    for (const PassCost::Measured& m : cost.measurements())
        rows.push_back({fbW * fbH, fbW, fbH, m.name, m.medianMs,
                        cost.trustworthy()});

    if (++scaleIndex >= SCALE_COUNT) {
        window.setSize(baseWidth, baseHeight);   // leave the window as found
        writeCsv();
        logSummary();
        state = Done;
        quit = true;                              // unattended run ends itself
        return;
    }
    state = Resizing;
}

void PassSweep::writeCsv() const {
    std::ofstream out(outPath, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG_WARN("pass sweep: cannot write %s", outPath.c_str());
        return;
    }
    out << "framebuffer_w,framebuffer_h,megapixels,config,median_ms,trusted\n";
    for (const Row& r : rows)
        out << r.width << ',' << r.height << ','
            << (r.pixels / 1.0e6) << ',' << r.config << ','
            << r.medianMs << ',' << (r.trusted ? 1 : 0) << '\n';
    LOG_INFO("pass sweep: wrote %s (%zu rows)", outPath.c_str(), rows.size());
}

void PassSweep::logSummary() const {
    // One line per size: what each pass cost there. Printed as well as written
    // so an unattended run is readable straight from the terminal.
    for (int i = 0; i < SCALE_COUNT; i++) {
        const size_t base = static_cast<size_t>(i) * PassCost::STEP_COUNT;
        if (base + PassCost::STEP_COUNT > rows.size()) break;
        const Row& baseline = rows[base];
        if (!baseline.trusted) {
            LOG_WARN("pass sweep %dx%d (%.2fM px): UNRELIABLE at this size",
                     baseline.width, baseline.height, baseline.pixels / 1.0e6);
            continue;
        }
        std::string line;
        char buf[128];
        for (size_t k = 1; k < PassCost::STEP_COUNT; k++) {
            std::snprintf(buf, sizeof(buf), "  %s=%.2fms",
                          rows[base + k].config,
                          baseline.medianMs - rows[base + k].medianMs);
            line += buf;
        }
        LOG_INFO("pass sweep %dx%d (%.2fM px) baseline %.2fms |%s",
                 baseline.width, baseline.height, baseline.pixels / 1.0e6,
                 baseline.medianMs, line.c_str());
    }
}

}  // namespace engine
