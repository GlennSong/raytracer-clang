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
        if (sec.s0 <= s0 + kSeam || sec.s0 >= s1 - kSeam) continue;
        at.push_back(sec.s0 - kSeam);
        at.push_back(sec.s0 + kSeam);
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
    int n = 0;
    for (int k = 0; k < stride && n < maxCount; ++k) {
        // A boundary that exists at one end and not the other is a lane
        // appearing or ending inside this step. Interpolating its offset from a
        // zero would drag paint across the carriageway, so the slot takes the
        // end that has it and only its POSITION is blended.
        bool liveA = a[k].style > -0.5f;
        bool liveB = b[k].style > -0.5f;
        if (!liveA && !liveB) continue;
        const RlBoundary& src = (u < 0.5 && liveA) || !liveB ? a[k] : b[k];
        RlBoundary r = src;
        r.t = float(lerp(double(a[k].t), double(b[k].t), u));
        out[n++] = r;
    }
    return n;
}

}  // namespace roadlab
