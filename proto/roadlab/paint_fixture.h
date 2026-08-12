#ifndef ROADLAB_PAINT_FIXTURE_H
#define ROADLAB_PAINT_FIXTURE_H

// A corpus of marking-evaluator inputs, with the CPU's answers, for comparing
// against a GPU.
//
// rl_paint.h claims to be one evaluator that runs on four backends. Everything
// so far checks that claim structurally: the WGSL is generated from the header,
// a digest catches drift, naga parses it and lowers it to SPIR-V. None of that
// says the two compute the same numbers, which is the only part that matters —
// a translation can be perfectly valid WGSL and still paint a different road.
//
// So: dump the inputs and the CPU's outputs, run the same inputs through the
// shader on any WebGPU device, and compare. tools/roadlab-wgsl-run.py does the
// second half against llvmpipe, which needs no hardware at all.
//
// The file is little-endian float32/int32 with no padding beyond what is written
// here — a format worth exactly as much as the one script that reads it.

#include "paint_bake.h"
#include <string>

namespace roadlab {

struct PaintCase {
    float s = 0, t = 0, fw = 0;
    float roadWear = 0, wearMask = 0, wheel = 0;
    int count = 0;
    int pad = 0;   // 32 bytes, so the GPU's array stride matches this struct
};

struct PaintFixture {
    std::vector<PaintCase> cases;
    std::vector<RlBoundary> bounds;    // cases.size() * kMaxBakedBoundaries
    std::vector<float> expected;       // cases.size() * 4 — coverage, r, g, b
};

// Real cross-sections from every demo, plus synthetic rows that force the styles
// the demos happen not to use. Deterministic: same corpus every run, so a
// mismatch is reproducible.
PaintFixture buildPaintFixture();

bool writePaintFixture(const PaintFixture& fx, const std::string& path, std::string& err);

}  // namespace roadlab

#endif
