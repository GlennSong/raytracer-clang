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
        case FramePhase::Update:        return &s.updateMs;
        case FramePhase::FixedUpdate:   return &s.fixedMs;
        case FramePhase::Render:        return &s.renderMs;
        case FramePhase::Wait:          return &s.waitMs;
        case FramePhase::RenderAcquire: return &s.acquireMs;
        case FramePhase::RenderEncode:  return &s.encodeMs;
        case FramePhase::RenderSubmit:  return &s.submitMs;
        case FramePhase::Poll:          return &s.pollMs;
        case FramePhase::Dispatch:      return &s.dispatchMs;
        case FramePhase::StateSwap:     return &s.stateSwapMs;
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
                          uint32_t triangles, float gpuMs,
                          uint32_t meshUploads, uint32_t textureUploads) {
    if (!frameOpen) return;
    frameOpen = false;
    current.totalMs = msBetween(frameStart, Clock::now());
    current.hostDeltaMs = static_cast<float>(hostDeltaSeconds * 1000.0);
    current.fixedSteps = fixedSteps;
    current.drawCalls = drawCalls;
    current.instances = instances;
    current.triangles = triangles;
    current.gpuMs = gpuMs;
    current.meshUploads = meshUploads;
    current.textureUploads = textureUploads;
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
                    << sample.triangles << ',' << sample.acquireMs << ','
                    << sample.encodeMs << ',' << sample.submitMs << ','
                    << sample.gpuMs << ',' << sample.meshUploads << ','
                    << sample.textureUploads << ',' << sample.pollMs << ','
                    << sample.dispatchMs << ',' << sample.stateSwapMs << '\n';
        captureRows++;
    }
}

FrameStats::Summary FrameStats::summarize() const {
    Summary s;
    if (count == 0) return s;

    std::array<float, HISTORY> totals;
    double sumTotal = 0, sumUpdate = 0, sumFixed = 0, sumRender = 0,
           sumWait = 0, sumDelta = 0, sumAcquire = 0, sumEncode = 0,
           sumSubmit = 0, sumGpu = 0, sumOther = 0;
    for (int i = 0; i < count; i++) {
        const FrameSample& f = historyAt(i);
        totals[i] = f.totalMs;
        sumTotal += f.totalMs;
        sumUpdate += f.updateMs;
        sumFixed += f.fixedMs;
        sumRender += f.renderMs;
        sumWait += f.waitMs;
        sumDelta += f.hostDeltaMs;
        sumAcquire += f.acquireMs;
        sumEncode += f.encodeMs;
        sumSubmit += f.submitMs;
        sumGpu += f.gpuMs;
        // Time inside the frame that no bracket claimed. Watching it is how a
        // missing bracket announces itself instead of hiding a stall.
        sumOther += std::max(0.0f, f.totalMs - (f.updateMs + f.fixedMs +
                                                f.acquireMs + f.encodeMs +
                                                f.submitMs + f.waitMs +
                                                f.pollMs + f.dispatchMs +
                                                f.stateSwapMs));
        s.maxTotalMs = std::max(s.maxTotalMs, f.totalMs);
    }
    s.avgTotalMs = static_cast<float>(sumTotal / count);
    s.avgUpdateMs = static_cast<float>(sumUpdate / count);
    s.avgFixedMs = static_cast<float>(sumFixed / count);
    s.avgRenderMs = static_cast<float>(sumRender / count);
    s.avgWaitMs = static_cast<float>(sumWait / count);
    s.avgAcquireMs = static_cast<float>(sumAcquire / count);
    s.avgEncodeMs = static_cast<float>(sumEncode / count);
    s.avgSubmitMs = static_cast<float>(sumSubmit / count);
    s.avgGpuMs = static_cast<float>(sumGpu / count);
    s.avgOtherMs = static_cast<float>(sumOther / count);
    if (sumDelta > 0)
        s.avgFps = static_cast<float>(count / (sumDelta / 1000.0));

    // p95 = the frame time 95% of frames beat; nth_element on the window copy.
    int idx = std::min(count - 1, (count * 95) / 100);
    std::nth_element(totals.begin(), totals.begin() + idx,
                     totals.begin() + count);
    s.p95TotalMs = totals[idx];
    return s;
}

bool FrameStats::startCapture(const std::string& path,
                              const std::string& context) {
    stopCapture();
    captureFile.open(path, std::ios::out | std::ios::trunc);
    if (!captureFile.is_open()) return false;
    captureTarget = path;
    captureRows = 0;
    if (!context.empty()) captureFile << "# " << context << '\n';
    captureFile << "frame,total_ms,update_ms,fixed_ms,render_ms,wait_ms,"
                   "host_delta_ms,fixed_steps,draw_calls,instances,triangles,"
                   "acquire_ms,encode_ms,submit_ms,gpu_ms,mesh_uploads,"
                   "texture_uploads,poll_ms,dispatch_ms,state_swap_ms\n";
    return true;
}

void FrameStats::stopCapture() {
    if (captureFile.is_open()) captureFile.close();
    captureTarget.clear();
}

}  // namespace engine
