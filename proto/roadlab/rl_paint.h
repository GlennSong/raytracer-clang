#ifndef ROADLAB_RL_PAINT_H
#define ROADLAB_RL_PAINT_H

// The lane-marking evaluator, in a subset that compiles as C++, MSL and GLSL
// unchanged.
//
// This is the piece that has to run on a GPU. Everything else in the surface
// shader — asphalt grain, concrete, wear blotching — the engine already has and
// already affords; what it does not have is markings positioned in METRES from a
// cross-section that changes along the road. That is the part worth porting, and
// it is small: a handful of abs() and smoothstep() per boundary.
//
// Three constraints shape the interface, all of them from the GPU side:
//
//   1. No containers. The CPU reference walks a std::vector<Boundary> built per
//      fragment from the cross-section. A fragment cannot. So the boundaries
//      arrive as a flat, fixed-size array that something else filled in — from
//      vertex attributes, or a profile texture, or (on CPU) directly.
//   2. No noise. Wear needs a blotch mask, but every backend already ships value
//      noise and they do not agree bit-for-bit. The mask is an INPUT. That keeps
//      this header exactly about paint, and lets each backend spend its own
//      noise. Wear is dressing; nothing downstream measures it.
//   3. Float, not double. GPUs have no doubles worth using. Coverage differences
//      against the double reference are ~1e-7, which no test should care about
//      and no eye can see.
//
// The engine already shades roads this way — analytic markings from a road-local
// UV, under Surface::RoadMarkings (ADR-0044). The difference is the coordinate:
// road_mesh.cpp bakes `2 + lateral/halfWidth`, a FRACTION of the road's width, so
// paint stretches when the road widens and a lane that appears cannot be drawn
// at all. Here `t` is metres.
//
// WGSL is the one backend this cannot be shared with verbatim — different struct
// and function syntax — so it is GENERATED from this file by
// tools/roadlab-wgsl.py, and tests.cpp fails if the two drift. Metal and GLSL
// take it as-is with the two macros below.
//
// Writing for that generator is why this file has no ternaries and no pointer
// out-parameters: both translate badly, and the alternatives (rlMax/rlMin, and
// returning a small struct) are no worse in C++. Shrinking the shared subset was
// cheaper than growing the translator, and it is the reason the translator can
// refuse anything it does not recognise instead of guessing.

// Address space for the boundary array. Metal wants `device const` or
// `constant`; C++ and GLSL want nothing.
#ifndef RL_BUF
#define RL_BUF const
#endif
// Function linkage. C++ needs `inline` in a header or every translation unit
// that includes it defines the symbol again; GLSL has no such keyword and Metal
// is happy with either. One macro rather than four copies of the file.
#ifndef RL_FN
#  ifdef __cplusplus
#    define RL_FN inline
#  else
#    define RL_FN
#  endif
#endif

// How many painted boundaries a station carries. Fixed because a GPU record has
// to be, and 12 because the widest cross-section in the demos paints nine. This
// is the array length WGSL needs and the cap paint_bake.h enforces — one number,
// so they cannot disagree.
#define RL_MAX_BOUNDS 12

// Marking styles. Kept as an int rather than an enum so the value can travel
// through a float texture channel without a cast that varies by language.
#define RL_MARK_NONE 0
#define RL_MARK_SOLID 1
#define RL_MARK_DASHED 2
#define RL_MARK_DOUBLE 3
#define RL_MARK_SOLID_DASHED 4
#define RL_MARK_DASHED_SOLID 5
#define RL_MARK_WIDE_DASHED 6
#define RL_MARK_BOTTS 7
#define RL_MARK_HATCH 8
#define RL_MARK_CHEVRON 9

// Paint colours, as an index rather than a triple: three floats per boundary
// saved, and every backend already needs the same small palette.
#define RL_PAINT_WHITE 0
#define RL_PAINT_YELLOW 1
#define RL_PAINT_RED 2
#define RL_PAINT_BLUE 3
#define RL_PAINT_GREEN 4

// Eight floats — two RGBA texels, or eight interpolated vertex channels.
struct RlBoundary {
    float t;         // lateral offset from the reference line, IN METRES
    float style;     // RL_MARK_*
    float width;     // stripe width (m)
    float gap;       // gap between the pair, for the double styles (m)
    float dashOn;    // dash length (m)
    float dashOff;   // gap length (m)
    float wear;      // extra wear on this marking, over the road's own
    float color;     // RL_PAINT_*
};

struct RlPaint {
    float coverage;
    float r;
    float g;
    float b;
};

struct RlRgb {
    float r;
    float g;
    float b;
};

// --- primitives -------------------------------------------------------------
//
// The only two functions the WGSL generator does not translate: it emits
// `max`/`min` for them and checks they are still here under these names. Every
// comparison in this file goes through them, which is what keeps the rest
// ternary-free.

RL_FN float rlMax(float a, float b) { return a > b ? a : b; }
RL_FN float rlMin(float a, float b) { return a < b ? a : b; }

// --- helpers ----------------------------------------------------------------

RL_FN float rlSaturate(float x) { return rlMin(rlMax(x, 0.0f), 1.0f); }

// Analytic coverage of a signed distance under a filter of width `fw`: the
// fraction of the pixel the shape covers. This is what makes a 100 mm stripe
// legible at 200 m without a mip chain.
RL_FN float rlCoverage(float d, float fw) {
    float h = 0.5f * rlMax(fw, 1e-6f);
    return rlSaturate((h - d) / (2.0f * h));
}

// x mod period, never negative. The expression is already in [0, period) in
// exact arithmetic; the clamp is for the float edge where floor() rounds the
// wrong way and hands back a value a few ulp below zero.
RL_FN float rlWrap(float x, float period) {
    return rlMax(x - period * floor(x / period), 0.0f);
}

RL_FN struct RlRgb rlPaintColor(float idx) {
    struct RlRgb c;
    c.r = 0.16f; c.g = 0.46f; c.b = 0.26f;   // green, the last band
    if (idx < 0.5f) {          // white
        c.r = 0.86f; c.g = 0.86f; c.b = 0.84f;
    } else if (idx < 1.5f) {   // yellow
        c.r = 0.83f; c.g = 0.68f; c.b = 0.16f;
    } else if (idx < 2.5f) {   // red
        c.r = 0.62f; c.g = 0.16f; c.b = 0.14f;
    } else if (idx < 3.5f) {   // blue
        c.r = 0.18f; c.g = 0.30f; c.b = 0.62f;
    }
    return c;
}

// Dash duty with an anti-alias fallback. Below the resolution of the pattern the
// stripe fades to its duty cycle rather than aliasing between dash and gap —
// which is what stops a dashed line crawling as the camera pulls back.
RL_FN float rlDashGate(float s, float on, float off, float fw) {
    float onLen = rlMax(on, 0.2f);
    float offLen = rlMax(off, 0.2f);
    float period = onLen + offLen;
    float u = rlWrap(s, period);
    float d = rlMax(-u, u - onLen);
    float c = rlCoverage(d, fw);
    float blend = rlSaturate(fw / (onLen * 0.7f));
    return c + (onLen / period - c) * blend;
}

// --- the evaluator ----------------------------------------------------------
//
// `bounds` holds only the PAINTED boundaries, ordered by t, most negative first.
// Nothing here depends on that order — the bake resolved the one thing that did
// — but the sim and the export read the same array and do.
//
// `wearMask` is a 0..1 blotch field the caller supplies (see note 2 above);
// `wheel` is how close this fragment is to a wheel path, where paint wears
// fastest.
RL_FN struct RlPaint rlEvaluateMarkings(RL_BUF struct RlBoundary* bounds, int count, float s, float t,
                                  float fw, float roadWear, float wearMask, float wheel) {
    struct RlPaint out;
    out.coverage = 0.0f;
    out.r = 0.8f;
    out.g = 0.8f;
    out.b = 0.8f;

    for (int i = 0; i < count; ++i) {
        float style = bounds[i].style;
        if (style < 0.5f) continue;   // RL_MARK_NONE

        float bt = bounds[i].t;
        float dt = t - bt;
        // Nothing is painted more than a metre from its own boundary, so most
        // fragments reject every marking on the road in one compare each.
        if (dt > 1.0f + fw || dt < -(1.0f + fw)) continue;

        float w = rlMax(bounds[i].width, 0.06f);
        // Anti-shimmer: never draw a stripe thinner than the filter, and scale
        // its contribution so average brightness is preserved. Without this a
        // receding lane line sparkles instead of fading.
        float effW = rlMax(w, fw * 1.6f);
        float energy = w / effW;

        float off = (bounds[i].gap + w) * 0.5f;
        float dash = rlDashGate(s, bounds[i].dashOn, bounds[i].dashOff, fw);
        float cov = 0.0f;

        if (style < 1.5f) {                    // SOLID
            cov = rlCoverage(fabs(dt) - effW * 0.5f, fw);
        } else if (style < 2.5f) {             // DASHED
            cov = rlCoverage(fabs(dt) - effW * 0.5f, fw) * dash;
        } else if (style < 3.5f) {             // DOUBLE
            float a = rlCoverage(fabs(dt + off) - effW * 0.5f, fw);
            float b = rlCoverage(fabs(dt - off) - effW * 0.5f, fw);
            cov = rlMax(a, b);
        } else if (style < 4.5f) {             // SOLID_DASHED
            float a = rlCoverage(fabs(dt + off) - effW * 0.5f, fw);
            float b = rlCoverage(fabs(dt - off) - effW * 0.5f, fw) * dash;
            cov = rlMax(a, b);
        } else if (style < 5.5f) {             // DASHED_SOLID
            float a = rlCoverage(fabs(dt + off) - effW * 0.5f, fw) * dash;
            float b = rlCoverage(fabs(dt - off) - effW * 0.5f, fw);
            cov = rlMax(a, b);
        } else if (style < 6.5f) {             // WIDE_DASHED (ramp extension)
            float on = 0.6f;
            float period = 1.8f;
            float u = rlWrap(s, period);
            float d = rlMax(-u, u - on);
            float g = rlCoverage(d, fw);
            g = g + (on / period - g) * rlSaturate(fw / (on * 0.7f));
            float wide = rlMax(0.20f, fw * 1.6f);
            cov = rlCoverage(fabs(dt) - wide * 0.5f, fw) * g;
        } else if (style < 7.5f) {             // BOTTS
            float pitch = 1.2f;
            float u = rlWrap(s, pitch) - pitch * 0.5f;
            cov = rlCoverage(sqrt(dt * dt + u * u) - 0.055f, fw);
        } else {                               // HATCH / CHEVRON
            // Areas, not lines — which is also why OpenDRIVE cannot express them
            // as a roadMark. The far edge was resolved at bake time (the bake can
            // see unpainted boundaries; the shader is only given painted ones)
            // and travels in `gap`, which the area styles do not otherwise use.
            float far = bounds[i].gap;
            float lo = rlMin(bt, far);
            float hi = rlMax(bt, far);
            if (t >= lo && t <= hi) {
                float d = rlWrap((s + t) * 0.7071f, 1.4f) - 0.7f;
                cov = rlCoverage(fabs(d) - 0.07f, fw);
            }
        }

        if (cov <= 1e-4f) continue;
        cov *= energy;

        float wearAmt = rlSaturate(roadWear + bounds[i].wear);
        if (wearAmt > 0.001f) {
            float erode = rlSaturate((wearAmt * (0.55f + 0.9f * wheel) - wearMask + 0.35f) * 2.2f);
            cov *= (1.0f - 0.95f * erode);
        }

        // Painted over painted keeps the newest colour; coverage saturates.
        if (cov > out.coverage * 0.5f) {
            struct RlRgb c = rlPaintColor(bounds[i].color);
            out.r = c.r;
            out.g = c.g;
            out.b = c.b;
        }
        out.coverage = rlMin(out.coverage + cov, 1.0f);
    }
    return out;
}

#endif
