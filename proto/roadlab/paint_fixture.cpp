#include "paint_fixture.h"

#include "scene.h"

#include <cstdio>

namespace roadlab {

namespace {

// A small deterministic generator. Not rl_math's noise: the corpus has to be
// identical on any machine that builds this, and a shared RNG that someone
// retunes later would silently change the fixture.
struct Lcg {
    uint32_t s;
    explicit Lcg(uint32_t seed) : s(seed ? seed : 1u) {}
    float unit() {
        s = s * 1664525u + 1013904223u;
        return float((s >> 8) & 0xffffff) / float(0x1000000);
    }
    float range(float lo, float hi) { return lo + (hi - lo) * unit(); }
};

void addCase(PaintFixture& fx, const RlBoundary* row, int count, const PaintCase& c) {
    PaintCase cc = c;
    cc.count = count;
    fx.cases.push_back(cc);
    for (int k = 0; k < kMaxBakedBoundaries; ++k) {
        RlBoundary b = k < count ? row[k] : RlBoundary{};
        if (k >= count) b.style = float(RL_MARK_NONE);
        fx.bounds.push_back(b);
    }
    RlPaint p = rlEvaluateMarkings(row, count, c.s, c.t, c.fw, c.roadWear, c.wearMask, c.wheel);
    fx.expected.push_back(p.coverage);
    fx.expected.push_back(p.r);
    fx.expected.push_back(p.g);
    fx.expected.push_back(p.b);
}

// Filter widths spanning the whole regime: sharp enough that a stripe is solid,
// wide enough that the dash pattern and then the stripe itself go sub-pixel.
// The far field is where the energy-preservation arithmetic lives, and it is the
// part most likely to differ between a compiler's float and a GPU's.
const float kFilters[] = {0.002f, 0.01f, 0.05f, 0.2f, 0.8f, 2.5f, 8.0f};

}  // namespace

PaintFixture buildPaintFixture() {
    PaintFixture fx;
    Lcg rng(20260812u);

    // Real cross-sections. These carry the style mixes and the boundary spacings
    // the demos actually produce, which no synthetic row would think to make.
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);
        for (const Road& r : sc.net.roads()) {
            double len = r.activeLength();
            if (len < 8.0) continue;
            for (int k = 0; k < 6; ++k) {
                double s = r.begin() + len * (0.07 + 0.16 * double(k));
                RlBoundary row[kMaxBakedBoundaries];
                int n = bakeBoundaries(r, s, row, kMaxBakedBoundaries);
                if (n == 0) continue;
                double le = r.xs.leftExtentAt(s), re = r.xs.rightExtentAt(s);
                for (int j = 0; j <= 18; ++j) {
                    PaintCase c;
                    c.s = float(s);
                    c.t = float(re + (le - re) * double(j) / 18.0);
                    c.fw = kFilters[size_t(j) % (sizeof(kFilters) / sizeof(kFilters[0]))];
                    c.roadWear = rng.range(0.0f, 1.0f);
                    c.wearMask = rng.range(0.0f, 1.0f);
                    c.wheel = rng.range(0.0f, 1.0f);
                    addCase(fx, row, n, c);
                }
            }
        }
    }

    // Synthetic rows for every style code, including the ones no demo reaches.
    // A style the corpus never exercises is a branch the comparison never
    // covers, and the area styles are exactly where the two implementations
    // would be most likely to disagree.
    for (int style = RL_MARK_SOLID; style <= RL_MARK_CHEVRON; ++style) {
        for (int variant = 0; variant < 3; ++variant) {
            RlBoundary row[kMaxBakedBoundaries];
            for (RlBoundary& b : row) {
                b = RlBoundary{};
                b.style = float(RL_MARK_NONE);
            }
            int n = 3;
            for (int k = 0; k < n; ++k) {
                row[k].t = float(-3.5 + 3.5 * k);
                row[k].style = float(style);
                row[k].width = rng.range(0.08f, 0.30f);
                row[k].gap = style == RL_MARK_HATCH || style == RL_MARK_CHEVRON
                                 ? row[k].t + rng.range(1.0f, 3.0f)
                                 : rng.range(0.08f, 0.25f);
                row[k].dashOn = rng.range(0.5f, 6.0f);
                row[k].dashOff = rng.range(0.5f, 12.0f);
                row[k].wear = rng.range(0.0f, 0.6f);
                row[k].color = float(variant % 5);
            }
            for (int i = 0; i < 240; ++i) {
                PaintCase c;
                c.s = rng.range(-40.0f, 400.0f);
                c.t = rng.range(-6.0f, 6.0f);
                c.fw = kFilters[size_t(i) % (sizeof(kFilters) / sizeof(kFilters[0]))];
                c.roadWear = rng.range(0.0f, 1.0f);
                c.wearMask = rng.range(0.0f, 1.0f);
                c.wheel = rng.range(0.0f, 1.0f);
                addCase(fx, row, n, c);
            }
        }
    }

    // Degenerate inputs, because a shader gets these too: no boundaries, a
    // boundary the fragment is nowhere near, a zero-width stripe.
    {
        RlBoundary row[kMaxBakedBoundaries];
        for (RlBoundary& b : row) {
            b = RlBoundary{};
            b.style = float(RL_MARK_NONE);
        }
        PaintCase c;
        c.fw = 0.02f;
        c.wearMask = 1.0f;
        addCase(fx, row, 0, c);
        row[0].style = float(RL_MARK_SOLID);
        row[0].t = 0.0f;
        row[0].width = 0.0f;
        for (float t : {-50.0f, -1.0f, 0.0f, 1.0f, 50.0f}) {
            c.t = t;
            addCase(fx, row, 1, c);
        }
    }
    return fx;
}

bool writePaintFixture(const PaintFixture& fx, const std::string& path, std::string& err) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        err = "cannot open " + path;
        return false;
    }
    int32_t header[2] = {0x524c5058 /*'RLPX'*/, int32_t(fx.cases.size())};
    bool ok = std::fwrite(header, sizeof(int32_t), 2, f) == 2;
    ok = ok && std::fwrite(fx.cases.data(), sizeof(PaintCase), fx.cases.size(), f) ==
                   fx.cases.size();
    ok = ok && std::fwrite(fx.bounds.data(), sizeof(RlBoundary), fx.bounds.size(), f) ==
                   fx.bounds.size();
    ok = ok && std::fwrite(fx.expected.data(), sizeof(float), fx.expected.size(), f) ==
                   fx.expected.size();
    std::fclose(f);
    if (!ok) err = "short write to " + path;
    return ok;
}

}  // namespace roadlab
