#include "facade_plan.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

// A stable hash of a surface's identity. Element weights are resolved against
// THIS rather than a running RNG stream, so adding an element to a registry
// cannot reshuffle where the previously registered ones landed — a property
// that matters enormously once recipes are data an author edits repeatedly.
std::uint32_t surfaceHash(std::uint32_t seed, int level, int edge, int bay,
                          int spec) {
    std::uint32_t h = seed * 2654435761u;
    h ^= static_cast<std::uint32_t>(level + 1) * 2246822519u;
    h = (h << 13) | (h >> 19);
    h ^= static_cast<std::uint32_t>(edge + 1) * 3266489917u;
    h = (h << 7) | (h >> 25);
    h ^= static_cast<std::uint32_t>(bay + 2) * 668265263u;
    h ^= static_cast<std::uint32_t>(spec + 1) * 374761393u;
    h ^= h >> 15;
    h *= 2246822519u;
    h ^= h >> 13;
    return h;
}

Real hashUnit(std::uint32_t h) { return (h >> 8) * (1.0 / 16777216.0); }

}  // namespace

// ---------------------------------------------------------------------------
// Bay grid
// ---------------------------------------------------------------------------

BayGrid bayGridFor(Real wallLength, Real preferredBayWidth) {
    BayGrid g;
    g.wallLength = std::max(Real(0), wallLength);
    const Real want = std::max(Real(0.5), preferredBayWidth);
    // Whole bays, so the rhythm closes on the wall exactly. A wall too short
    // for even one bay still reports one — a 2 m wall has a single narrow bay
    // rather than zero, which would make it invisible to every selector.
    g.bays = std::max(1, static_cast<int>(std::llround(g.wallLength / want)));
    g.bayWidth = g.bays > 0 ? g.wallLength / g.bays : g.wallLength;
    return g;
}

void BayGrid::bayspan(int i, Real& t0, Real& t1) const {
    if (bays <= 0) { t0 = 0; t1 = 1; return; }
    i = std::max(0, std::min(bays - 1, i));
    t0 = static_cast<Real>(i) / bays;
    t1 = static_cast<Real>(i + 1) / bays;
}

Real BayGrid::bayCentre(int i) const {
    Real a, b;
    bayspan(i, a, b);
    return (a + b) * 0.5;
}

int BayGrid::bayAt(Real t) const {
    if (bays <= 0) return 0;
    const int i = static_cast<int>(t * bays);
    return std::max(0, std::min(bays - 1, i));
}

int BayGrid::centreBay() const { return bays > 0 ? bays / 2 : 0; }

// ---------------------------------------------------------------------------
// Fenestration
// ---------------------------------------------------------------------------

WallRole roleForTag(EdgeTag tag, int floor, bool retail) {
    switch (tag) {
        case EdgeTag::Party:
            // Blank BY CONSTRUCTION. The whole reason edges are tagged once at
            // plan time rather than re-derived: a shared wall can never
            // accidentally sprout windows into the neighbour's building.
            return WallRole::Blank;
        case EdgeTag::Rear:
        case EdgeTag::Lane:
            // A lane IS access, but it is the back of the building: service
            // doors and a few small openings, never the show facade.
            return WallRole::Sparse;
        case EdgeTag::Street:
            return (retail && floor == 0) ? WallRole::Storefront
                                          : WallRole::Regular;
        case EdgeTag::Court:
        case EdgeTag::Side:
        case EdgeTag::None:
            break;
    }
    return WallRole::Regular;
}

std::vector<Opening> fenestrate(const BayGrid& grid, WallRole role,
                                const FenestrationStyle& style, int floor,
                                bool entrance) {
    std::vector<Opening> out;
    if (grid.bays <= 0 || role == WallRole::Blank) return out;

    if (role == WallRole::Curtain) {
        // One banded ribbon per bay: the curtain wall's vision strip.
        for (int b = 0; b < grid.bays; ++b) {
            Real t0, t1;
            grid.bayspan(b, t0, t1);
            const Real inset = (t1 - t0) * 0.04;
            Opening o;
            o.t0 = t0 + inset;
            o.t1 = t1 - inset;
            o.sill = 0.55;
            o.head = 2.9;
            o.kind = OpeningKind::Window;
            o.bay = b;
            out.push_back(o);
        }
        return out;
    }

    // Windows DIMINISH WITH HEIGHT in a classical facade — an upper storey's
    // opening is shorter than the piano nobile's below it. Small, but it is
    // what stops a tall masonry building reading as a stack of identical
    // photocopied floors.
    const Real headScale =
        floor <= 0 ? Real(1.0)
                   : std::max(Real(0.80), Real(1.0) - static_cast<Real>(floor) * 0.022);

    const int doorBay = grid.centreBay();
    for (int b = 0; b < grid.bays; ++b) {
        Real t0, t1;
        grid.bayspan(b, t0, t1);
        const Real span = t1 - t0;

        // The DOOR splits its run rather than living inside one glazing
        // strategy — every wall that carries an entrance gets one, whatever
        // the style, which is what stops a door being trapped in one branch.
        if (entrance && b == doorBay) {
            const Real half = (style.doorWidth / std::max(grid.wallLength, Real(0.01))) * 0.5;
            const Real c = (t0 + t1) * 0.5;
            Opening d;
            d.t0 = std::max(t0, c - half);
            d.t1 = std::min(t1, c + half);
            d.sill = 0;
            d.head = style.doorHead;
            d.kind = OpeningKind::Door;
            d.bay = b;
            out.push_back(d);
            continue;
        }

        if (role == WallRole::Sparse) {
            // Service walls get a few small openings, not a full rhythm.
            if (b % 2 == 1) continue;
            Opening o;
            o.t0 = t0 + span * 0.38;
            o.t1 = t1 - span * 0.38;
            o.sill = style.sill + 0.35;
            o.head = style.head - 0.25;
            o.kind = OpeningKind::Window;
            o.bay = b;
            out.push_back(o);
            continue;
        }

        if (role == WallRole::Storefront) {
            Opening o;
            o.t0 = t0 + span * 0.06;
            o.t1 = t1 - span * 0.06;
            o.sill = 0.35;
            o.head = 2.9;
            o.kind = OpeningKind::Storefront;
            o.bay = b;
            out.push_back(o);
            continue;
        }

        const Real w = span * std::max(Real(0.15), std::min(Real(0.9),
                                                            style.windowWidthFrac));
        const Real pad = (span - w) * 0.5;
        Opening o;
        o.t0 = t0 + pad;
        o.t1 = t1 - pad;
        o.sill = style.sill;
        o.head = style.sill + (style.head - style.sill) * headScale;
        o.kind = OpeningKind::Window;
        o.bay = b;
        out.push_back(o);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Element registry
// ---------------------------------------------------------------------------

const char* elementKindName(ElementKind k) {
    switch (k) {
        case ElementKind::Opening:    return "opening";
        case ElementKind::Balcony:    return "balcony";
        case ElementKind::Cornice:    return "cornice";
        case ElementKind::Quoin:      return "quoin";
        case ElementKind::Pilaster:   return "pilaster";
        case ElementKind::BaseCourse: return "base-course";
        case ElementKind::Awning:     return "awning";
        case ElementKind::Portico:    return "portico";
        case ElementKind::Porch:      return "porch";
        case ElementKind::Steps:      return "steps";
        case ElementKind::BayWindow:  return "bay-window";
        case ElementKind::Parapet:    return "parapet";
        case ElementKind::Steeple:    return "steeple";
        case ElementKind::Spire:      return "spire";
        case ElementKind::Dome:       return "dome";
        case ElementKind::RoofDeck:   return "roof-deck";
        case ElementKind::Sign:       return "sign";
        case ElementKind::Planter:    return "planter";
        case ElementKind::Lamp:       return "lamp";
    }
    return "?";
}

bool ElementSelector::matchesEdge(EdgeTag t) const {
    return anyTag || t == tag;
}

bool ElementSelector::matchesFloor(int floor) const {
    if (floorMin >= 0 && floor < floorMin) return false;
    if (floorMax >= 0 && floor > floorMax) return false;
    return true;
}

bool ElementSelector::matchesTier(int t, int topTier) const {
    if (tier == -1) return true;
    if (tier == kTopTier) return t == topTier;
    return t == tier;
}

bool ElementSelector::matchesBay(int bay, const BayGrid& grid) const {
    switch (bays) {
        case Bays::All:        return true;
        case Bays::None:       return bay < 0;
        case Bays::Centre:     return bay == grid.centreBay();
        case Bays::Ends:       return bay == 0 || bay == grid.bays - 1;
        case Bays::Alternate:  return bay >= 0 && (bay % 2) == 0;
        case Bays::Boundaries: return true;   // handled by the resolver's span
    }
    return true;
}

BuildingSurfaces surfacesFor(const MassStack& stack, Real preferredBay) {
    BuildingSurfaces s;
    s.levels = expandLevels(stack);
    s.grids.reserve(s.levels.size());
    for (const Level& lv : s.levels) {
        std::vector<BayGrid> perEdge;
        // Only the OUTER loop of each plan carries facade; a courtyard's inner
        // walls are addressed separately (their edges carry EdgeTag::Court).
        for (const Shape2& plan : lv.plans) {
            for (std::size_t i = 0; i < plan.outer.size(); ++i) {
                const ArcGeom g = arcGeom(plan.outer.start(i), plan.outer.end(i),
                                          plan.outer.bulge(i));
                const Real len = g.straight
                                     ? (plan.outer.end(i) - plan.outer.start(i)).length()
                                     : std::fabs(g.radius * g.sweep);
                perEdge.push_back(bayGridFor(len, preferredBay));
            }
            for (const Loop2& h : plan.holes)
                for (std::size_t i = 0; i < h.size(); ++i)
                    perEdge.push_back(
                        bayGridFor((h.end(i) - h.start(i)).length(), preferredBay));
        }
        s.grids.push_back(std::move(perEdge));
        s.topTier = std::max(s.topTier, lv.tier);
    }
    return s;
}

namespace {

// The tag of plan-edge `edge` on a level, walking the same order surfacesFor
// used to build the grids. One traversal order, defined once, so grids and tags
// cannot drift apart.
EdgeTag tagOfEdge(const Level& lv, int edge) {
    int i = 0;
    for (const Shape2& plan : lv.plans) {
        for (std::size_t e = 0; e < plan.outer.size(); ++e, ++i)
            if (i == edge) return plan.outer.edges[e].tag;
        for (const Loop2& h : plan.holes)
            for (std::size_t e = 0; e < h.size(); ++e, ++i)
                if (i == edge) return h.edges[e].tag;
    }
    return EdgeTag::None;
}

}  // namespace

std::vector<ElementPlacement> resolveElements(const ElementRegistry& reg,
                                              const BuildingSurfaces& surfaces,
                                              std::uint32_t seed) {
    std::vector<ElementPlacement> out;
    // Which level index is the first of each tier, so "floor" can mean "floor
    // within this tier" — what a recipe actually means by "floors 1 and up".
    std::vector<int> tierStart(surfaces.topTier + 2, 0);
    for (std::size_t i = 0; i < surfaces.levels.size(); ++i) {
        const int t = surfaces.levels[i].tier;
        if (t + 1 < static_cast<int>(tierStart.size()) && tierStart[t] == 0 &&
            (i == 0 || surfaces.levels[i - 1].tier != t))
            tierStart[t] = static_cast<int>(i);
    }

    for (std::size_t si = 0; si < reg.elements.size(); ++si) {
        const ElementSpec& spec = reg.elements[si];
        for (std::size_t li = 0; li < surfaces.levels.size(); ++li) {
            const Level& lv = surfaces.levels[li];
            const int floorInTier =
                static_cast<int>(li) -
                (lv.tier < static_cast<int>(tierStart.size()) ? tierStart[lv.tier] : 0);
            if (!spec.on.matchesTier(lv.tier, surfaces.topTier)) continue;
            if (!spec.on.matchesFloor(floorInTier)) continue;
            const std::vector<BayGrid>& grids = surfaces.grids[li];

            for (int e = 0; e < static_cast<int>(grids.size()); ++e) {
                const EdgeTag tag = tagOfEdge(lv, e);
                if (!spec.on.matchesEdge(tag)) continue;
                const BayGrid& grid = grids[e];

                auto emit = [&](int bay, Real t0, Real t1) {
                    // The weight is resolved against a STABLE per-surface hash,
                    // so an occasional element appears on the same bays every
                    // rebuild and adding another element does not move it.
                    if (spec.weight < 1.0) {
                        const Real r = hashUnit(surfaceHash(
                            seed, static_cast<int>(li), e, bay, static_cast<int>(si)));
                        if (r >= spec.weight) return;
                    }
                    ElementPlacement p;
                    p.kind = spec.kind;
                    p.level = static_cast<int>(li);
                    p.tier = lv.tier;
                    p.edge = e;
                    p.tag = tag;
                    p.bay = bay;
                    p.t0 = t0;
                    p.t1 = t1;
                    p.depth = spec.depth;
                    p.height = spec.height;
                    p.width = spec.width;
                    p.style = spec.style;
                    p.specIndex = static_cast<int>(si);
                    out.push_back(p);
                };

                if (spec.on.bays == ElementSelector::Bays::None) {
                    emit(-1, 0, 1);            // a whole-wall element
                } else if (spec.on.bays == ElementSelector::Bays::Boundaries) {
                    // Pilasters land BETWEEN bays, never through a window —
                    // the thing a private bay count made impossible.
                    for (int b = 0; b <= grid.bays; ++b) {
                        const Real t = static_cast<Real>(b) / std::max(1, grid.bays);
                        emit(b, t, t);
                    }
                } else {
                    for (int b = 0; b < grid.bays; ++b) {
                        if (!spec.on.matchesBay(b, grid)) continue;
                        Real t0, t1;
                        grid.bayspan(b, t0, t1);
                        emit(b, t0, t1);
                    }
                }
            }
        }
    }
    return out;
}

}  // namespace engine
