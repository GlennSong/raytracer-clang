#include "test_framework.h"

#include "../src/engine/frame_stats.h"

#include <cstdio>
#include <fstream>
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

namespace {
FrameSample sampleWithTotal(float totalMs, float hostDeltaMs = 0.0f) {
    FrameSample s;
    s.totalMs = totalMs;
    s.hostDeltaMs = hostDeltaMs;
    return s;
}
}

TEST_CASE(frame_stats_ring_keeps_newest_window) {
    FrameStats fs;
    CHECK(fs.historySize() == 0);
    CHECK(fs.frameCount() == 0);

    // Overfill the ring: only the newest HISTORY frames remain, oldest first.
    const int overshoot = 10;
    for (int i = 0; i < FrameStats::HISTORY + overshoot; i++)
        fs.record(sampleWithTotal(static_cast<float>(i)));

    CHECK(fs.historySize() == FrameStats::HISTORY);
    CHECK(fs.frameCount() == FrameStats::HISTORY + overshoot);
    CHECK_APPROX(fs.historyAt(0).totalMs, overshoot, 1e-6);
    CHECK_APPROX(fs.historyAt(FrameStats::HISTORY - 1).totalMs,
                 FrameStats::HISTORY + overshoot - 1, 1e-6);
    CHECK_APPROX(fs.lastFrame().totalMs, FrameStats::HISTORY + overshoot - 1,
                 1e-6);
}

TEST_CASE(frame_stats_summary_aggregates) {
    FrameStats fs;
    // 99 steady frames at 10 ms plus one 50 ms spike; host delta 10 ms/frame.
    for (int i = 0; i < 99; i++) fs.record(sampleWithTotal(10.0f, 10.0f));
    fs.record(sampleWithTotal(50.0f, 10.0f));

    FrameStats::Summary s = fs.summarize();
    CHECK_APPROX(s.avgTotalMs, (99 * 10.0 + 50.0) / 100.0, 1e-4);
    CHECK_APPROX(s.maxTotalMs, 50.0, 1e-6);
    // The spike is the slowest 1% — p95 must NOT report it.
    CHECK_APPROX(s.p95TotalMs, 10.0, 1e-6);
    CHECK_APPROX(s.avgFps, 100.0, 0.1);   // 10 ms host delta = 100 fps
}

TEST_CASE(frame_stats_summary_empty_is_zero) {
    FrameStats fs;
    FrameStats::Summary s = fs.summarize();
    CHECK(s.avgTotalMs == 0.0f);
    CHECK(s.p95TotalMs == 0.0f);
    CHECK(s.maxTotalMs == 0.0f);
    CHECK(s.avgFps == 0.0f);
}

TEST_CASE(frame_stats_render_split_phases_are_distinct) {
    FrameStats fs;
    fs.beginFrame();
    // The render split (ADR-0077): each sub-phase lands in its own field, so a
    // frame blocked on the GPU (acquire) is never confused with CPU work.
    fs.beginPhase(FramePhase::RenderAcquire);
    fs.endPhase(FramePhase::RenderAcquire);
    fs.beginPhase(FramePhase::RenderEncode);
    fs.endPhase(FramePhase::RenderEncode);
    fs.beginPhase(FramePhase::RenderSubmit);
    fs.endPhase(FramePhase::RenderSubmit);
    fs.endFrame(1.0 / 60.0, 1, 0, 0, 0, 12.5f);

    const FrameSample& f = fs.lastFrame();
    CHECK(f.acquireMs >= 0.0f);
    CHECK(f.encodeMs >= 0.0f);
    CHECK(f.submitMs >= 0.0f);
    // Sub-phases are tracked separately from the aggregate render bracket,
    // which this frame never opened — so it must stay zero, not absorb them.
    CHECK(f.renderMs == 0.0f);
    CHECK_APPROX(f.gpuMs, 12.5, 1e-4);

    FrameStats::Summary s = fs.summarize();
    CHECK_APPROX(s.avgGpuMs, 12.5, 1e-4);
}

TEST_CASE(frame_stats_gpu_defaults_to_zero_when_unsupported) {
    FrameStats fs;
    fs.beginFrame();
    fs.endFrame(1.0 / 60.0, 1, 0, 0, 0);   // backend reports no GPU timing
    CHECK(fs.lastFrame().gpuMs == 0.0f);
    CHECK(fs.summarize().avgGpuMs == 0.0f);
}

TEST_CASE(frame_stats_phase_brackets_fill_sample) {
    FrameStats fs;
    fs.beginFrame();
    fs.beginPhase(FramePhase::Update);
    fs.endPhase(FramePhase::Update);
    // A phase entered twice in one frame accumulates (fixedUpdate steps).
    fs.beginPhase(FramePhase::FixedUpdate);
    fs.endPhase(FramePhase::FixedUpdate);
    fs.beginPhase(FramePhase::FixedUpdate);
    fs.endPhase(FramePhase::FixedUpdate);
    fs.endFrame(1.0 / 60.0, 2, 7, 100, 9000);

    const FrameSample& f = fs.lastFrame();
    CHECK(fs.frameCount() == 1);
    CHECK(f.totalMs >= 0.0f);
    CHECK(f.totalMs >= f.updateMs);       // phases nest inside the frame
    CHECK(f.updateMs >= 0.0f);
    CHECK(f.fixedMs >= 0.0f);
    CHECK(f.fixedSteps == 2);
    CHECK(f.drawCalls == 7);
    CHECK(f.instances == 100);
    CHECK(f.triangles == 9000);
    CHECK_APPROX(f.hostDeltaMs, 1000.0 / 60.0, 1e-3);
}

TEST_CASE(frame_stats_phase_outside_frame_is_ignored) {
    FrameStats fs;
    // The modal-resize draw callback renders outside runFrame: no open frame,
    // so brackets and endFrame must be inert rather than corrupt the ring.
    fs.beginPhase(FramePhase::Render);
    fs.endPhase(FramePhase::Render);
    fs.endFrame(0.016, 1, 0, 0, 0);
    CHECK(fs.frameCount() == 0);
    CHECK(fs.historySize() == 0);
}

TEST_CASE(frame_stats_csv_capture_round_trips) {
    const std::string path = "frame_stats_test_capture.csv";
    FrameStats fs;
    CHECK(fs.startCapture(path));
    CHECK(fs.capturing());
    CHECK(fs.capturePath() == path);

    FrameSample s = sampleWithTotal(12.5f, 16.6f);
    s.updateMs = 1.5f;
    s.fixedSteps = 3;
    s.drawCalls = 42;
    fs.record(s);
    fs.record(sampleWithTotal(13.0f));
    CHECK(fs.capturedFrames() == 2);
    fs.stopCapture();
    CHECK(!fs.capturing());

    std::ifstream in(path);
    CHECK(in.is_open());
    std::string header, row1, row2, extra;
    std::getline(in, header);
    std::getline(in, row1);
    std::getline(in, row2);
    bool hasExtra = static_cast<bool>(std::getline(in, extra));
    CHECK(header ==
          "frame,total_ms,update_ms,fixed_ms,render_ms,wait_ms,"
          "host_delta_ms,fixed_steps,draw_calls,instances,triangles,"
          "acquire_ms,encode_ms,submit_ms,gpu_ms");
    CHECK(row1.substr(0, 2) == "1,");
    CHECK(row1.find(",42,") != std::string::npos);   // draw calls column
    CHECK(row2.substr(0, 2) == "2,");
    CHECK(!hasExtra);
    in.close();
    std::remove(path.c_str());
}

TEST_CASE(frame_stats_capture_to_bad_path_fails_closed) {
    FrameStats fs;
    CHECK(!fs.startCapture("no_such_dir/frame_stats.csv"));
    CHECK(!fs.capturing());
    fs.record(sampleWithTotal(1.0f));   // must not crash or write
    CHECK(fs.capturedFrames() == 0);
}
