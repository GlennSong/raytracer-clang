#include "roof_plant.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

// One item's demand for space: its wanted size and the size below which it is
// not worth placing at all (a 1 m "water tank" is a crate).
struct Demand {
    RoofPlantItem::Kind kind;
    Real w, d, minW, minD;
};

struct Cell { Real u, v, w, d; };

}  // namespace

std::vector<RoofPlantItem> planRoofPlant(const Poly2* topPlan,
                                         const Vec2& frameOrigin,
                                         const Vec2& axisU, const Vec2& axisV,
                                         Real width, Real depth, int floors,
                                         bool curtainWall,
                                         const std::function<Real()>& unit01) {
    std::vector<RoofPlantItem> out;
    // Mirrors Rng::range exactly, so the call sequence — and therefore every
    // building already in the world — is unchanged by the move.
    auto range = [&](Real a, Real b) { return a + (b - a) * unit01(); };
    const Real inset = 0.7;                       // parapet clearance
    const Real uw = width - 2 * inset, ud = depth - 2 * inset;
    if (uw < 3.5 || ud < 3.5) return out;
    const Real roofArea = uw * ud;

    // --- what this roof carries ------------------------------------------
    std::vector<Demand> want;
    if (floors >= 4 && width > 5 && depth > 5) {          // roof access: always 1
        const Real bw = std::min(std::max(width * 0.28, Real(3.0)), Real(5.0));
        const Real bd = std::min(std::max(depth * 0.24, Real(2.4)), Real(3.8));
        want.push_back({RoofPlantItem::Kind::Access, bw, bd, Real(2.4), Real(2.0)});
    }
    if (floors >= 3 && floors <= 20 && !curtainWall && width > 4 && depth > 4 &&
        unit01() < 0.65) {                              // water tanks: 1-2
        const int nTank = (roofArea > 170.0 && unit01() < 0.35) ? 2 : 1;
        for (int i = 0; i < nTank; ++i) {
            const Real tr =
                std::min(range(1.2, 1.7), std::min(width, depth) * 0.22);
            want.push_back({RoofPlantItem::Kind::WaterTank, 2 * tr + 1.0,
                            2 * tr + 1.0, Real(2.4), Real(2.4)});
        }
    }
    if (floors >= 4 && width > 6 && depth > 5 && unit01() < 0.85) {
        const int nHvac = std::max(1, std::min(4, static_cast<int>(roofArea / 70.0)));
        for (int i = 0; i < nHvac; ++i) {
            const Real aw = std::min(std::max(width * 0.26, Real(2.6)), Real(5.2));
            const Real ad = std::min(std::max(depth * 0.20, Real(2.0)), Real(3.2));
            want.push_back({RoofPlantItem::Kind::Hvac, aw, ad, Real(2.2), Real(1.8)});
        }
    }
    if (want.empty()) return out;

    // --- carve the roof into one cell per item ----------------------------
    std::vector<Cell> cells{{inset, inset, uw, ud}};
    while (cells.size() < want.size()) {
        std::size_t bi = 0;                       // split the biggest cell
        for (std::size_t i = 1; i < cells.size(); ++i)
            if (cells[i].w * cells[i].d > cells[bi].w * cells[bi].d) bi = i;
        Cell c = cells[bi];
        if (std::max(c.w, c.d) < 4.5) break;      // roof full: extras are skipped
        const Real t = range(0.4, 0.6);
        Cell a = c, b = c;
        if (c.w >= c.d) { a.w = c.w * t; b.u = c.u + a.w; b.w = c.w - a.w; }
        else            { a.d = c.d * t; b.v = c.v + a.d; b.d = c.d - a.d; }
        cells[bi] = a;
        cells.push_back(b);
    }
    // Biggest item takes the biggest cell.
    std::sort(want.begin(), want.end(), [](const Demand& a, const Demand& b) {
        return a.w * a.d > b.w * b.d;
    });
    std::sort(cells.begin(), cells.end(), [](const Cell& a, const Cell& b) {
        return a.w * a.d > b.w * b.d;
    });

    // A seat is only real if all four corners are ON the roof polygon — an L or
    // a courtyard plan has notches its bounding box does not know about.
    auto onRoof = [&](Real u, Real v, Real w2, Real d2) {
        if (!topPlan) return true;
        for (int cu = 0; cu <= 1; ++cu)
            for (int cv = 0; cv <= 1; ++cv) {
                const Vec2 q = frameOrigin + axisU * (u + w2 * cu) +
                               axisV * (v + d2 * cv);
                if (!pointInPolygon(*topPlan, q)) return false;
            }
        return true;
    };

    const std::size_t n = std::min(want.size(), cells.size());
    for (std::size_t i = 0; i < n; ++i) {
        const Demand& it = want[i];
        const Cell& c = cells[i];
        const Real clear = 0.5;
        const Real w2 = std::min(it.w, c.w - clear);
        const Real d2 = std::min(it.d, c.d - clear);
        if (w2 < it.minW || d2 < it.minD) continue;   // cell too small: skip it
        for (int attempt = 0; attempt < 4; ++attempt) {
            const Real u = c.u + (c.w - w2) * range(0.2, 0.8);
            const Real v = c.v + (c.d - d2) * range(0.2, 0.8);
            if (!onRoof(u, v, w2, d2)) continue;      // in a notch: jitter again
            RoofPlantItem p;
            p.kind = it.kind;
            p.u = u;
            p.v = v;
            p.w = w2;
            p.d = d2;
            out.push_back(p);
            break;
        }
    }
    return out;
}

}  // namespace engine
