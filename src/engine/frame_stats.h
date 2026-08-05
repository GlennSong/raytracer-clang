#ifndef RAYTRACER_ENGINE_FRAME_STATS_H
#define RAYTRACER_ENGINE_FRAME_STATS_H

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>

namespace engine {

// Which slice of the frame a timing bracket covers. Application brackets each
// phase of runFrame; everything not covered (event-bus dispatch, state swaps)
// lands in the frame total but no phase — that gap is itself a signal.
enum class FramePhase { Update, FixedUpdate, Render, Wait };

// One frame of the ledger: CPU phase times plus the renderer's submission
// counters (RenderStats), copied at end of frame so a capture row correlates
// "frame got slow" with "draw calls doubled" without a second tool.
struct FrameSample {
    float totalMs = 0.0f;      // runFrame wall time, sleep included
    float updateMs = 0.0f;     // input + state update()
    float fixedMs = 0.0f;      // all fixed steps this frame
    float renderMs = 0.0f;     // renderFrame(): submit + present
    float waitMs = 0.0f;       // FPS-cap sleep (budget headroom, not cost)
    float hostDeltaMs = 0.0f;  // window-reported frame-to-frame delta —
                               // what the player actually feels
    int fixedSteps = 0;
    uint32_t drawCalls = 0;
    uint32_t instances = 0;
    uint32_t triangles = 0;
};

// The frame ledger (ADR-0077): a fixed ring of per-frame samples every build
// carries, always on. It is NOT a profiler — Tracy (ADR-0068) owns zones,
// threads, and deep dives. The ledger answers the questions Tracy can't when
// no UI is attachable: a glanceable HUD on-device (Vision Pro, where the
// wearer can't see a second machine), and a CSV capture any host can write
// headlessly for diffing before/after a change (tools/frame-report.py).
//
// Cost when idle: a handful of steady_clock reads per frame and one ring
// write. Aggregation (summarize) runs only when someone asks.
class FrameStats {
public:
    static constexpr int HISTORY = 240;   // ~4 s at 60 fps, ~2.7 s at 90 Hz

    // Frame-loop hooks, driven by Application::runFrame. Phase brackets
    // accumulate, so a phase entered twice in one frame sums.
    void beginFrame();
    void beginPhase(FramePhase phase);
    void endPhase(FramePhase phase);
    void endFrame(double hostDeltaSeconds, int fixedSteps,
                  uint32_t drawCalls, uint32_t instances, uint32_t triangles);

    // Clock-free seam: fold a ready-made sample into the ring (and the capture
    // file, if open). endFrame() lands here; tests feed synthetic frames.
    void record(const FrameSample& sample);

    const FrameSample& lastFrame() const { return last; }
    // Frames recorded since construction (not capped by HISTORY).
    long frameCount() const { return totalFrames; }

    // Ring access for plots, oldest first. historyAt(historySize()-1) == last.
    int historySize() const { return count; }
    const FrameSample& historyAt(int index) const {
        return ring[(head + HISTORY - count + index) % HISTORY];
    }

    // Aggregates over the current ring window.
    struct Summary {
        float avgTotalMs = 0.0f;
        float p95TotalMs = 0.0f;   // spikiness — the number avg hides
        float maxTotalMs = 0.0f;
        float avgUpdateMs = 0.0f;
        float avgFixedMs = 0.0f;
        float avgRenderMs = 0.0f;
        float avgWaitMs = 0.0f;
        float avgFps = 0.0f;       // from hostDeltaMs, the felt rate
    };
    Summary summarize() const;

    // CSV capture: one row per recorded frame until stopCapture(). Returns
    // false (and stays off) if the path can't be opened — a read-only bundle
    // dir on visionOS, say; pass a Documents path there. Restarting an active
    // capture closes the old file first.
    bool startCapture(const std::string& path);
    void stopCapture();
    bool capturing() const { return captureFile.is_open(); }
    const std::string& capturePath() const { return captureTarget; }
    long capturedFrames() const { return captureRows; }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr int PHASE_COUNT = 4;

    std::array<FrameSample, HISTORY> ring{};
    int head = 0;    // next write slot
    int count = 0;
    long totalFrames = 0;
    FrameSample last;

    FrameSample current;
    Clock::time_point frameStart;
    std::array<Clock::time_point, PHASE_COUNT> phaseStart{};
    bool frameOpen = false;

    std::ofstream captureFile;
    std::string captureTarget;
    long captureRows = 0;
};

}  // namespace engine

#endif
