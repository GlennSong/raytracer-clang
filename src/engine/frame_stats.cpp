#include "frame_stats.h"

#include <algorithm>

namespace engine {

namespace {
float msBetween(std::chrono::steady_clock::time_point start,
                std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<float, std::milli>(end - start).count();
}

float* phaseField(FrameSample& s, FramePhase phase) {
    switch (phase) {
        case FramePhase::Update:      return &s.updateMs;
        case FramePhase::FixedUpdate: return &s.fixedMs;
        case FramePhase::Render:      return &s.renderMs;
        case FramePhase::Wait:        return &s.waitMs;
    }
    return &s.updateMs;   // unreachable; keeps -Wreturn-type honest
}
}  // namespace

void FrameStats::beginFrame() {
    current = FrameSample{};
    frameStart = Clock::now();
    frameOpen = true;
}

void FrameStats::beginPhase(FramePhase phase) {
    if (!frameOpen) return;   // draw-callback render outside runFrame: untimed
    phaseStart[static_cast<int>(phase)] = Clock::now();
}

void FrameStats::endPhase(FramePhase phase) {
    if (!frameOpen) return;
    *phaseField(current, phase) +=
        msBetween(phaseStart[static_cast<int>(phase)], Clock::now());
}

void FrameStats::endFrame(double hostDeltaSeconds, int fixedSteps,
                          uint32_t drawCalls, uint32_t instances,
                          uint32_t triangles) {
    if (!frameOpen) return;
    frameOpen = false;
    current.totalMs = msBetween(frameStart, Clock::now());
    current.hostDeltaMs = static_cast<float>(hostDeltaSeconds * 1000.0);
    current.fixedSteps = fixedSteps;
    current.drawCalls = drawCalls;
    current.instances = instances;
    current.triangles = triangles;
    record(current);
}

void FrameStats::record(const FrameSample& sample) {
    ring[head] = sample;
    head = (head + 1) % HISTORY;
    count = std::min(count + 1, HISTORY);
    last = sample;
    totalFrames++;

    if (captureFile.is_open()) {
        captureFile << totalFrames << ',' << sample.totalMs << ','
                    << sample.updateMs << ',' << sample.fixedMs << ','
                    << sample.renderMs << ',' << sample.waitMs << ','
                    << sample.hostDeltaMs << ',' << sample.fixedSteps << ','
                    << sample.drawCalls << ',' << sample.instances << ','
                    << sample.triangles << '\n';
        captureRows++;
    }
}

FrameStats::Summary FrameStats::summarize() const {
    Summary s;
    if (count == 0) return s;

    std::array<float, HISTORY> totals;
    double sumTotal = 0, sumUpdate = 0, sumFixed = 0, sumRender = 0,
           sumWait = 0, sumDelta = 0;
    for (int i = 0; i < count; i++) {
        const FrameSample& f = historyAt(i);
        totals[i] = f.totalMs;
        sumTotal += f.totalMs;
        sumUpdate += f.updateMs;
        sumFixed += f.fixedMs;
        sumRender += f.renderMs;
        sumWait += f.waitMs;
        sumDelta += f.hostDeltaMs;
        s.maxTotalMs = std::max(s.maxTotalMs, f.totalMs);
    }
    s.avgTotalMs = static_cast<float>(sumTotal / count);
    s.avgUpdateMs = static_cast<float>(sumUpdate / count);
    s.avgFixedMs = static_cast<float>(sumFixed / count);
    s.avgRenderMs = static_cast<float>(sumRender / count);
    s.avgWaitMs = static_cast<float>(sumWait / count);
    if (sumDelta > 0)
        s.avgFps = static_cast<float>(count / (sumDelta / 1000.0));

    // p95 = the frame time 95% of frames beat; nth_element on the window copy.
    int idx = std::min(count - 1, (count * 95) / 100);
    std::nth_element(totals.begin(), totals.begin() + idx,
                     totals.begin() + count);
    s.p95TotalMs = totals[idx];
    return s;
}

bool FrameStats::startCapture(const std::string& path) {
    stopCapture();
    captureFile.open(path, std::ios::out | std::ios::trunc);
    if (!captureFile.is_open()) return false;
    captureTarget = path;
    captureRows = 0;
    captureFile << "frame,total_ms,update_ms,fixed_ms,render_ms,wait_ms,"
                   "host_delta_ms,fixed_steps,draw_calls,instances,triangles\n";
    return true;
}

void FrameStats::stopCapture() {
    if (captureFile.is_open()) captureFile.close();
    captureTarget.clear();
}

}  // namespace engine
