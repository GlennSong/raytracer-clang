#include "paint_bake.h"

#include <algorithm>

namespace roadlab {

float markStyleCode(MarkStyle style) {
    switch (style) {
        case MarkStyle::None: return float(RL_MARK_NONE);
        case MarkStyle::Solid: return float(RL_MARK_SOLID);
        case MarkStyle::Dashed: return float(RL_MARK_DASHED);
        case MarkStyle::Double: return float(RL_MARK_DOUBLE);
        case MarkStyle::SolidDashed: return float(RL_MARK_SOLID_DASHED);
        case MarkStyle::DashedSolid: return float(RL_MARK_DASHED_SOLID);
        case MarkStyle::WideDashed: return float(RL_MARK_WIDE_DASHED);
        case MarkStyle::Botts: return float(RL_MARK_BOTTS);
        case MarkStyle::Hatch: return float(RL_MARK_HATCH);
        case MarkStyle::Chevron: return float(RL_MARK_CHEVRON);
    }
    return float(RL_MARK_NONE);
}

float paintColorCode(PaintColor color) {
    switch (color) {
        case PaintColor::White: return float(RL_PAINT_WHITE);
        case PaintColor::Yellow: return float(RL_PAINT_YELLOW);
        case PaintColor::Red: return float(RL_PAINT_RED);
        case PaintColor::Blue: return float(RL_PAINT_BLUE);
        case PaintColor::Green: return float(RL_PAINT_GREEN);
    }
    return float(RL_PAINT_WHITE);
}

int bakeBoundaries(const Road& road, double s, RlBoundary* out, int maxCount) {
    static thread_local std::vector<Boundary> scratch;
    road.xs.boundariesAt(s, scratch);
    int n = 0;
    for (size_t bi = 0; bi < scratch.size(); ++bi) {
        const Boundary& b = scratch[bi];
        const Marking& m = b.mark;
        // Only PAINTED boundaries are baked. A cross-section has up to 17
        // boundaries once kerbs, gutters, verges and slopes are counted, but
        // never more than nine carry paint — and it is the painted ones the
        // shader needs. Carrying the rest would triple the per-station cost for
        // nothing.
        //
        // The one thing an unpainted boundary was used for is the hatch styles,
        // which fill outward to their neighbour. That extent is resolved HERE,
        // while the full list is still in hand, and stored in `gap` (which the
        // area styles do not otherwise use). The shader then needs no neighbour
        // at all, so filtering cannot change what it draws.
        if (m.style == MarkStyle::None) continue;
        if (n >= maxCount) break;
        if (m.style == MarkStyle::Hatch || m.style == MarkStyle::Chevron) {
            double far = b.t + (b.t >= 0 ? 3.0 : -3.0);
            if (b.t >= 0 && bi + 1 < scratch.size()) far = scratch[bi + 1].t;
            if (b.t < 0 && bi > 0) far = scratch[bi - 1].t;
            out[n].gap = float(far);
        }
        out[n].t = float(b.t);
        out[n].style = markStyleCode(m.style);
        out[n].width = m.width;
        if (m.style != MarkStyle::Hatch && m.style != MarkStyle::Chevron) out[n].gap = m.gap;
        out[n].dashOn = m.dashOn;
        out[n].dashOff = m.dashOff;
        out[n].wear = m.wear;
        out[n].color = paintColorCode(m.color);
        ++n;
    }
    return n;
}

BoundaryStrip bakeBoundaryStrip(const Road& road, double step) {
    BoundaryStrip strip;
    if (step < 0.05) step = 0.05;
    double s0 = road.begin(), s1 = road.end();

    // Regular rings, PLUS a pair straddling every lane-section boundary.
    //
    // The pair is the whole trick. Slot k of the baked array is only the same
    // physical boundary at both ends of a step if the boundary SET is the same,
    // and it changes discontinuously at a section boundary — a lane appears and
    // every slot outboard of it shifts one place. Interpolating across that
    // blends two unrelated boundaries and drags paint metres sideways. Landing a
    // ring on either side of the seam means no step ever straddles one, which is
    // a requirement on the road mesher too: it must emit a ring pair there.
    std::vector<double> at;
    for (double s = s0; s < s1 - 1e-6; s += step) at.push_back(s);
    at.push_back(s1);
    const double kSeam = 1e-3;
    for (const LaneSection& sec : road.xs.sections) {
        if (sec.s0 > s0 + kSeam && sec.s0 < s1 - kSeam) {
            at.push_back(sec.s0 - kSeam);
            at.push_back(sec.s0 + kSeam);
        }
        // A short section needs rings of its own, not just the pair at its ends.
        //
        // The seam pair keeps a step from STRADDLING a discontinuity; it says
        // nothing about resolving what happens between two of them. Widths are
        // cubics, and a gore nose is a section a metre or two long over which a
        // lane goes from nothing to full width — linear interpolation between
        // its two edges is 6 cm out near the ends, which is more than a stripe
        // is wide. Subdividing by the section rather than by the road is what
        // makes the strip's error a property of the geometry instead of a
        // property of how short someone made a taper.
        double inner0 = std::max(s0, sec.s0) + kSeam;
        double inner1 = std::min(s1, sec.s0 + sec.length) - kSeam;
        double span = inner1 - inner0;
        if (span <= 0 || span >= step) continue;   // the regular grid already covers it
        for (int k = 1; k < 4; ++k) at.push_back(inner0 + span * double(k) * 0.25);
    }
    std::sort(at.begin(), at.end());
    at.erase(std::unique(at.begin(), at.end(),
                         [](double a, double b) { return std::fabs(a - b) < 1e-9; }),
             at.end());

    strip.stations = at;
    strip.rows.assign(at.size() * size_t(kMaxBakedBoundaries), RlBoundary{});
    for (size_t i = 0; i < at.size(); ++i) {
        RlBoundary* row = &strip.rows[i * size_t(kMaxBakedBoundaries)];
        // Padding is style -1, not 0. An UNMARKED boundary is a real boundary
        // with style None (0) — it holds a slot its neighbours are numbered
        // against — while a padding slot is a boundary that does not exist. Both
        // paint nothing, so conflating them is invisible until the interpolator
        // silently renumbers every lane past the first unmarked one.
        for (int k = 0; k < kMaxBakedBoundaries; ++k) {
            row[k] = RlBoundary{};
            row[k].style = -1.0f;
        }
        bakeBoundaries(road, at[i], row, kMaxBakedBoundaries);
    }

    // Hold the offset across dead slots.
    //
    // A slot that is dead at one end of a step and live at the other is a lane
    // appearing or ending inside it. The dead end's `t` is whatever the padding
    // left there — zero, the reference line — so interpolating it sweeps the
    // marking from the middle of the road out to its real position over the
    // length of the step. Copying the nearest live offset into the dead slot
    // makes that interpolation a no-op instead.
    //
    // This is not cosmetic. The profile texture (paint_texture.h) hands the
    // offset axis to the hardware's linear filter, which cannot be taught about
    // liveness; the only way the filter can be trusted is if a dead slot already
    // holds a harmless value. Fixing it here fixes it for both paths at once.
    const int stride = kMaxBakedBoundaries;
    for (int k = 0; k < stride; ++k) {
        float held = 0.0f;
        bool have = false;
        for (size_t i = 0; i < at.size(); ++i) {
            RlBoundary& b = strip.rows[i * size_t(stride) + size_t(k)];
            if (b.style > -0.5f) { held = b.t; have = true; }
            else if (have) b.t = held;
        }
        have = false;
        for (size_t i = at.size(); i-- > 0;) {
            RlBoundary& b = strip.rows[i * size_t(stride) + size_t(k)];
            if (b.style > -0.5f) { held = b.t; have = true; }
            else if (have) b.t = held;
        }
    }
    return strip;
}

int BoundaryStrip::sampleInterpolated(double s, RlBoundary* out, int maxCount) const {
    if (stations.empty()) return 0;
    // Which pair of rows the station falls between. A rasteriser gets this for
    // free from the vertex it is between; here it is a search.
    size_t hi = size_t(std::lower_bound(stations.begin(), stations.end(), s) - stations.begin());
    if (hi == 0) hi = 1;
    if (hi >= stations.size()) hi = stations.size() - 1;
    size_t lo = hi - 1;
    double span = stations[hi] - stations[lo];
    double u = span > 1e-9 ? clampd((s - stations[lo]) / span, 0.0, 1.0) : 0.0;

    const RlBoundary* a = &rows[lo * size_t(stride)];
    const RlBoundary* b = &rows[hi * size_t(stride)];
    // Offsets blend; everything else takes the nearer row whole.
    //
    // Only `t` is continuous in s. Style, width and colour are piecewise
    // constant — they change at lane-section seams and nowhere else — and a
    // blend between two of them is not a third style, it is a number that means
    // nothing. Both ends of an ordinary step sit in the same section and agree,
    // so nearest is exact there; the only steps where the two differ are the
    // 2 mm ones straddling a seam.
    //
    // Nearest is also what a GPU can do without help: one sampler set to linear
    // for the offset texture, one set to nearest for the style texture. Anything
    // cleverer here would be a CPU path the shader could not follow.
    const RlBoundary* near = u < 0.5 ? a : b;
    int n = 0;
    for (int k = 0; k < stride && n < maxCount; ++k) {
        if (near[k].style < -0.5f) continue;   // padding: this slot has no boundary
        RlBoundary r = near[k];
        r.t = float(lerp(double(a[k].t), double(b[k].t), u));
        out[n++] = r;
    }
    return n;
}

}  // namespace roadlab
