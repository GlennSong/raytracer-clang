#include "lot_program.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;

}  // namespace

const char* lotShapeName(LotShape s) {
    switch (s) {
        case LotShape::MidBlock: return "mid-block";
        case LotShape::Corner:   return "corner";
        case LotShape::Through:  return "through";
        case LotShape::Island:   return "island";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

Real maxStoreysFor(Real shortSide, Real plateArea) {
    // ONE height model above the tables, replacing six recipes' copy-pasted
    // `coreness * N` guesses. Lifts, cores and structure scale with the storeys
    // they serve: a tall building needs both a wide enough plate to fit a core
    // and enough floor area to make the core worth its footprint. Both numbers
    // are placeholders to be tuned; the point is that there is only one of them.
    if (shortSide <= 0 || plateArea <= 0) return 0;
    return std::min(shortSide / 0.55, plateArea / 26.0);
}

void inscribedRect(const Shape2& shape, Real& w, Real& d, Shape2* outRect) {
    w = d = 0;
    if (shape.outer.size() < 3) return;
    const Poly2 ring = tessellate(shape.outer, 0.08);
    if (ring.size() < 3) return;
    const OBB2 box = orientedBoundingBox(ring);

    // Work in the box's own frame, so a lot at any street angle measures the
    // same as one squared to the axes — a diagonal parcel is not less buildable
    // than a rectilinear one of the same size.
    const Vec2 ax = box.axis[0], az = box.axis[1];
    auto toLocal = [&](const Vec2& p) {
        const Vec2 r = p - box.center;
        return Vec2(dot(r, ax), dot(r, az));
    };
    auto toWorld = [&](const Vec2& p) {
        return box.center + ax * p.x + az * p.y;
    };

    // Binary search the largest scale of the box that still fits, then trim
    // each axis independently — the same shrink-to-fit that already seats a
    // house, used here as a QUALIFYING MEASUREMENT rather than a placement.
    auto fits = [&](Real hw, Real hd, const Vec2& c) {
        if (hw <= 0.05 || hd <= 0.05) return false;
        // Sample the candidate rectangle's boundary and interior corners; the
        // lot is convex-ish at this scale, so corners plus edge midpoints catch
        // every real violation.
        const Vec2 pts[8] = {
            Vec2(c.x - hw, c.y - hd), Vec2(c.x + hw, c.y - hd),
            Vec2(c.x + hw, c.y + hd), Vec2(c.x - hw, c.y + hd),
            Vec2(c.x, c.y - hd),      Vec2(c.x + hw, c.y),
            Vec2(c.x, c.y + hd),      Vec2(c.x - hw, c.y)};
        for (const Vec2& p : pts)
            if (!contains(shape, toWorld(p))) return false;
        return true;
    };

    const Vec2 localCentroid = toLocal(centroid(shape));
    Real bestW = 0, bestD = 0;
    Vec2 bestC = localCentroid;
    // Try a few centres: the centroid, and the box centre, plus small offsets.
    const Vec2 candidates[5] = {localCentroid, Vec2(0, 0),
                                Vec2(localCentroid.x * 0.5, localCentroid.y * 0.5),
                                Vec2(localCentroid.x, 0), Vec2(0, localCentroid.y)};
    for (const Vec2& c : candidates) {
        Real lo = 0, hi = 1.0;
        for (int i = 0; i < 22; ++i) {
            const Real mid = (lo + hi) * 0.5;
            if (fits(box.half[0] * mid, box.half[1] * mid, c)) lo = mid;
            else hi = mid;
        }
        Real hw = box.half[0] * lo, hd = box.half[1] * lo;
        if (hw <= 0.05 || hd <= 0.05) continue;
        // Grow each axis independently now that a seed rectangle fits.
        for (int axis = 0; axis < 2; ++axis) {
            Real g0 = 1.0, g1 = 3.0;
            for (int i = 0; i < 18; ++i) {
                const Real m = (g0 + g1) * 0.5;
                const bool okFit = axis == 0 ? fits(hw * m, hd, c) : fits(hw, hd * m, c);
                if (okFit) g0 = m; else g1 = m;
            }
            if (axis == 0) hw *= g0; else hd *= g0;
        }
        if (hw * hd > bestW * bestD) { bestW = hw; bestD = hd; bestC = c; }
    }
    if (bestW <= 0 || bestD <= 0) return;
    w = bestW * 2;
    d = bestD * 2;
    if (w < d) std::swap(w, d);          // report width >= depth, consistently
    if (outRect) {
        Poly2 r = {toWorld(Vec2(bestC.x - bestW, bestC.y - bestD)),
                   toWorld(Vec2(bestC.x + bestW, bestC.y - bestD)),
                   toWorld(Vec2(bestC.x + bestW, bestC.y + bestD)),
                   toWorld(Vec2(bestC.x - bestW, bestC.y + bestD))};
        *outRect = shapeFromPoly(r);
    }
}

LotTags measureLot(const Shape2& lot, const std::vector<Vec2>& streetDirs,
                   StreetClass klass, bool enclosed, Real coreness) {
    LotTags t;
    t.area = area(lot);
    t.streetClass = klass;
    t.enclosed = enclosed;
    t.coreness = std::max(Real(0), std::min(Real(1), coreness));
    inscribedRect(lot, t.inscribedW, t.inscribedD);
    t.maxStoreys = maxStoreysFor(std::min(t.inscribedW, t.inscribedD),
                                 t.inscribedW * t.inscribedD);

    // Frontage: total length of edges tagged Street, and how many DISTINCT
    // directions they face — which is what makes a lot a corner. Lane frontage
    // is accumulated separately: it grants ACCESS without granting cornerness.
    Real front = 0, lane = 0;
    std::vector<Vec2> dirs;
    for (std::size_t i = 0; i < lot.outer.size(); ++i) {
        if (lot.outer.edges[i].tag == EdgeTag::Lane) {
            lane += (lot.outer.end(i) - lot.outer.start(i)).length();
            continue;
        }
        if (lot.outer.edges[i].tag != EdgeTag::Street) continue;
        const Vec2 d = lot.outer.end(i) - lot.outer.start(i);
        const Real len = d.length();
        if (len < 1e-6) continue;
        front += len;
        const Vec2 outward = normalize(Vec2(d.y, -d.x));
        bool seen = false;
        for (const Vec2& e : dirs)
            if (dot(e, outward) > 0.72) { seen = true; break; }   // within ~44 deg
        if (!seen) dirs.push_back(outward);
    }
    if (dirs.empty() && !streetDirs.empty()) dirs = streetDirs;
    t.frontWidth = front;
    t.laneWidth = lane;
    t.frontages = static_cast<int>(dirs.size());

    if (t.frontages >= 4) t.shape = LotShape::Island;
    else if (t.frontages == 3) t.shape = LotShape::Corner;
    else if (t.frontages == 2) {
        // Two frontages facing OPPOSITE ways is a through lot; at right angles
        // it is a corner. The distinction decides where entrances and ornament
        // go, so it is worth getting from geometry rather than a coin flip.
        t.shape = dot(dirs[0], dirs[1]) < -0.5 ? LotShape::Through
                                               : LotShape::Corner;
    } else {
        t.shape = LotShape::MidBlock;
    }
    return t;
}

// ---------------------------------------------------------------------------
// Programs
// ---------------------------------------------------------------------------

bool lotFitsProgram(const LotTags& tags, const LotProgram& p) {
    // The inversion in one function: everything that used to be a rejection
    // counter downstream is a precondition here.
    if (tags.area < p.minArea) return false;
    // ACCESS, not show frontage: a mews house reached from a lane is perfectly
    // buildable. Requiring street frontage alone is what leaves a block's
    // interior landlocked and green by accident rather than by design.
    if (tags.frontWidth + tags.laneWidth < p.minFrontage) return false;
    // The lot must contain the program's minimum rectangle in EITHER
    // orientation — a 20x8 lot can hold an 8x20 building.
    const Real lw = std::max(tags.inscribedW, tags.inscribedD);
    const Real ld = std::min(tags.inscribedW, tags.inscribedD);
    const Real pw = std::max(p.minW, p.minD);
    const Real pd = std::min(p.minW, p.minD);
    if (lw < pw || ld < pd) return false;
    if (p.requiresCorner && tags.shape != LotShape::Corner) return false;
    if (p.requiresRim && tags.enclosed) return false;
    if (static_cast<int>(tags.streetClass) < static_cast<int>(p.minStreetClass))
        return false;
    // A program that needs more storeys than the plate can carry is not
    // eligible — §17.2's height model doing real work rather than decorating.
    if (p.minStoreys > 1 && tags.maxStoreys < p.minStoreys) return false;
    return true;
}

void ProgramSet::reset() { placed.assign(programs.size(), 0); }

int ProgramSet::bestFor(const LotTags& tags) const {
    // The highest-weighted eligible program, with no RNG and no quota consumed
    // — a stable answer to "what is this land for", which is what a parceller
    // needs to decide how finely to cut it.
    std::vector<std::pair<int, Real>> el = eligible(tags);
    int best = -1;
    Real bestW = -1;
    for (const auto& [i, w] : el)
        if (w > bestW) { bestW = w; best = i; }
    return best;
}

std::vector<std::pair<int, Real>> ProgramSet::eligible(const LotTags& tags) const {
    std::vector<std::pair<int, Real>> out;
    for (std::size_t i = 0; i < programs.size(); ++i) {
        const LotProgram& p = programs[i];
        if (!lotFitsProgram(tags, p)) continue;
        if (p.quota > 0 && i < placed.size() && placed[i] >= p.quota) continue;
        // Tag biasing: the facts the parceller measured, tilting the odds
        // rather than gating them.
        Real w = std::max(Real(0), p.weight);
        // SCALE FIT. A cottage technically fits on a 25 000 m2 rim parcel, and
        // if that counts as an equal candidate then the smallest program in a
        // mix drives every decision — the parceller cuts a campus block into
        // cottages and the coarse grain the rim exists for never appears.
        //
        // The fit PEAKS where the land matches the lot this program is built on
        // and falls away on both sides. A one-sided penalty (only punishing the
        // too-small) decays every candidate by the same factor, so it changes
        // the magnitudes and never the ORDER — which meant the highest base
        // weight won at every size and the largest program in a mix could never
        // be reached at all. Measured: `glass_tower` lost to `office_tower` on
        // a 1600 m2 parcel and on a 48 000 m2 one alike, so the cutter always
        // aimed at the smaller plate and the skyscraper program was dead
        // weight in the list.
        if (tags.area > 1.0 && p.targetArea() > 1.0) {
            const Real ratio = tags.area / p.targetArea();
            // Symmetric in log space: half the land, or twice it, costs the
            // same. Gentle, because this is a bias among survivors and not a
            // gate — `lotFitsProgram` already threw out what does not fit.
            const Real lg = std::log(ratio);
            w *= std::max(Real(0.02), std::exp(-(lg * lg) * 0.30));
        }
        if (tags.shape == LotShape::Corner && p.requiresCorner) w *= 3.0;
        if (tags.streetClass == StreetClass::Arterial &&
            p.frontage == FrontageKind::Direct)
            w *= 1.8;                                   // retail wants an arterial
        if (tags.streetClass == StreetClass::Lane &&
            p.service == ServiceKind::Loading)
            w *= 1.6;                                   // a lane gets service
        if (!tags.enclosed && p.requiresRim) w *= 2.5;
        // Downtown: bias toward whatever the plate can actually carry, so the
        // skyline rises with coreness instead of by dice.
        if (p.minStoreys >= 8) w *= 0.25 + tags.coreness * 2.5;
        if (w > 0) out.push_back({static_cast<int>(i), w});
    }
    return out;
}

int ProgramSet::pick(const LotTags& tags, std::uint32_t seed) {
    if (placed.size() != programs.size()) reset();

    // STAGE 1 — quotas run first. A quota'd program takes the best eligible lot
    // it sees rather than waiting for a die to favour it, which is the fix for
    // rarity-as-probability yielding either none or a dozen.
    std::vector<std::pair<int, Real>> el = eligible(tags);
    if (el.empty()) return -1;
    for (const auto& [idx, w] : el) {
        const LotProgram& p = programs[idx];
        if (p.quota <= 0 || placed[idx] >= p.quota) continue;
        // Only place a quota'd program on a lot that comfortably exceeds its
        // minimum — a landmark should not land on the worst site that merely
        // qualifies.
        const Real lw = std::max(tags.inscribedW, tags.inscribedD);
        const Real ld = std::min(tags.inscribedW, tags.inscribedD);
        if (lw >= std::max(p.minW, p.minD) * 1.25 &&
            ld >= std::min(p.minW, p.minD) * 1.25) {
            ++placed[idx];
            return idx;
        }
    }

    // STAGE 2 — weighted pick among the survivors.
    Real total = 0;
    for (const auto& [idx, w] : el) total += w;
    if (total <= 0) return -1;
    Rng rng(seed);
    Real r = rng.unit() * total;
    for (const auto& [idx, w] : el) {
        r -= w;
        if (r <= 0) {
            ++placed[idx];
            return idx;
        }
    }
    ++placed[el.back().first];
    return el.back().first;
}

// ---------------------------------------------------------------------------
// Stock program sets
// ---------------------------------------------------------------------------

namespace {

LotProgram makeProgram(const char* name, Real minW, Real minD, Real minArea,
                       Real coverage, Real front, Real side, Real rear,
                       FrontageKind fk, OpenKind ok, int stalls,
                       int minS, int maxS, Real weight,
                       std::vector<std::string> recipes = {}) {
    LotProgram p;
    p.name = name;
    p.minW = minW;
    p.minD = minD;
    p.minArea = minArea;
    p.minFrontage = std::min(minW, minD) * 0.7;
    p.coverage = coverage;
    p.frontSetback = front;
    p.sideSetback = side;
    p.rearSetback = rear;
    p.frontage = fk;
    p.open = ok;
    p.parkingStalls = stalls;
    p.minStoreys = minS;
    p.maxStoreys = maxS;
    p.weight = weight;
    p.recipes = std::move(recipes);
    return p;
}

// The lot a program is BUILT on. Stated for anything whose normal site is
// meaningfully bigger than the smallest one it would take — which is every
// program that wants a shaped plan, because a plan can only grow wings on a
// plate wide enough to have them.
// The verge between the carriageway and the first lot, counted once per side
// of a block. Matches BlockParcelParams::roadMargin, which is what the
// parceller will actually take off again.
constexpr Real kRoadVerge = 2.0;
// How many lots a block face carries before it stops reading as a block.
constexpr Real kLotsPerFrontage = 4.0;

LotProgram& built(LotProgram& p, Real w, Real d) {
    p.targetW = w;
    p.targetD = d;
    return p;
}

}  // namespace

BlockGrain blockGrainFor(const ProgramSet& mix) {
    const LotProgram* drive = nullptr;
    for (const LotProgram& p : mix.programs) {
        if (p.quota > 0) continue;                     // landmarks size nothing
        if (!drive || p.targetArea() > drive->targetArea()) drive = &p;
    }
    BlockGrain g;
    if (!drive) return g;
    // Street to street: a row of lots, back to back with the row behind it,
    // plus the verge between the carriageway and the first lot on each side.
    const Real lotDepth = drive->targetDepth() + drive->frontSetback +
                          drive->rearSetback;
    g.depth = 2 * lotDepth + 2 * kRoadVerge;
    // Along the street: enough lots to read as a block rather than a pair.
    g.length = std::max(g.depth, kLotsPerFrontage * drive->targetWidth());
    return g;
}

ProgramSet residentialPrograms() {
    ProgramSet s;
    // A cottage will squeeze onto 7 x 7 m; it is BUILT on a modest house plot,
    // and the difference is the whole grain of a neighbourhood. Deriving the
    // target from the minimum would ask for an 89 m2 lot, which is not a house
    // plot anywhere.
    LotProgram cottage = makeProgram("cottage", 7, 7, 90, 0.35, 5, 2, 6,
                                     FrontageKind::Garden, OpenKind::Lawn, 1,
                                     1, 2, 3.0, {"cottage_house", "craftsman"});
    s.programs.push_back(built(cottage, 13, 22));
    LotProgram villa = makeProgram("villa", 12, 11, 320, 0.30, 8, 4, 8,
                                   FrontageKind::Garden, OpenKind::Garden, 2,
                                   1, 3, 1.6, {"villa", "craftsman"});
    s.programs.push_back(built(villa, 20, 24));
    // A terrace is NARROW and deep — the densest thing on this list, and the
    // reason a neighbourhood block can hold four times the lots a downtown one
    // does on the same land.
    LotProgram terrace = makeProgram("townhouse", 6, 11, 80, 0.62, 2, 0, 5,
                                     FrontageKind::Patio, OpenKind::Lawn, 0,
                                     2, 4, 2.2, {"townhouse", "rowhouse"});
    s.programs.push_back(built(terrace, 7, 24));
    LotProgram walkup = makeProgram("walkup", 14, 14, 260, 0.55, 3, 3, 5,
                                    FrontageKind::Forecourt, OpenKind::Court, 2,
                                    3, 6, 1.2, {"walkup", "apartment_slab"});
    s.programs.push_back(built(walkup, 22, 26));
    LotProgram corner = makeProgram("corner_shopfront", 9, 10, 130, 0.75, 0, 0, 3,
                                    FrontageKind::Direct, OpenKind::None, 0,
                                    2, 4, 1.0, {"corner_shopfront"});
    corner.requiresCorner = true;
    built(corner, 11, 15);
    s.programs.push_back(corner);
    s.reset();
    return s;
}

ProgramSet commercialPrograms() {
    ProgramSet s;
    LotProgram shop = makeProgram("shopfront", 8, 12, 120, 0.85, 0, 0, 2,
                                  FrontageKind::Direct, OpenKind::None, 0,
                                  1, 3, 3.0, {"shopfront", "big_box"});
    s.programs.push_back(built(shop, 9, 26));      // the high-street plot
    s.programs.push_back(makeProgram("mixed_use", 12, 16, 240, 0.78, 0, 0, 3,
                                     FrontageKind::Direct, OpenKind::None, 0,
                                     3, 6, 2.0, {"mixed_use", "flatiron"}));
    // An office block is the smallest program that can carry a courtyard or an
    // atrium ring, and it only can on a plate wide enough for one.
    LotProgram block = makeProgram("office_block", 18, 18, 420, 0.70, 2, 2, 4,
                                   FrontageKind::Forecourt, OpenKind::None, 4,
                                   4, 10, 1.4, {"office_block", "hotel"});
    s.programs.push_back(built(block, 34, 38));
    LotProgram bank = makeProgram("bank", 14, 14, 260, 0.72, 1, 1, 3,
                                  FrontageKind::Plaza, OpenKind::None, 0,
                                  2, 5, 0.8, {"bank", "library"});
    bank.requiresCorner = true;
    s.programs.push_back(bank);
    s.reset();
    return s;
}

ProgramSet downtownPrograms() {
    ProgramSet s;
    s.programs.push_back(makeProgram("mixed_use", 12, 16, 240, 0.80, 0, 0, 2,
                                     FrontageKind::Direct, OpenKind::None, 0,
                                     4, 8, 2.2, {"mixed_use", "flatiron"}));
    // A TOWER PLATE. The minimum (26 x 26) is what a tower will squeeze onto;
    // the target is what one is actually built on, and the difference is the
    // whole reason a downtown lot can carry a shaped plan instead of a box.
    LotProgram tower = makeProgram("office_tower", 26, 26, 900, 0.68, 3, 3, 4,
                                   FrontageKind::Plaza, OpenKind::None, 0,
                                   12, 45, 1.5,
                                   {"office_tower", "stepped_tower",
                                    "terraced_tower"});
    s.programs.push_back(built(tower, 44, 46));
    // A slender tower needs a plate that can carry a core: §17.2 gates this,
    // not a per-recipe constant.
    LotProgram glass = makeProgram("glass_tower", 32, 30, 1300, 0.62, 4, 4, 5,
                                   FrontageKind::Plaza, OpenKind::None, 0,
                                   22, 70, 1.0,
                                   {"glass_tower", "profile_tower",
                                    "arch_tower", "pyramid_landmark"});
    s.programs.push_back(built(glass, 56, 54));
    LotProgram cathedral = makeProgram("cathedral", 26, 42, 1200, 0.55, 8, 6, 8,
                                       FrontageKind::Plaza, OpenKind::Park, 0,
                                       1, 3, 0.5, {"cathedral", "church"});
    built(cathedral, 46, 64);
    cathedral.quota = 1;              // exactly one, not a one-in-fifty chance
    s.programs.push_back(cathedral);
    LotProgram hall = makeProgram("civic_hall", 24, 30, 900, 0.55, 10, 8, 8,
                                  FrontageKind::Plaza, OpenKind::Park, 0,
                                  2, 5, 0.5, {"civic_hall", "capitol", "museum"});
    built(hall, 48, 56);
    hall.quota = 1;
    s.programs.push_back(hall);
    s.reset();
    return s;
}

ProgramSet rimPrograms() {
    ProgramSet s;
    // §17.6: a rim block has no far-side street, so its depth is unbounded —
    // which is not a special case but a TAG that admits a different program
    // set. The coarse grain falls out of the minimums, not out of a branch.
    LotProgram campus = makeProgram("university_campus", 70, 80, 8000, 0.28,
                                    16, 12, 16, FrontageKind::Forecourt,
                                    OpenKind::Park, 30, 2, 6, 1.0,
                                    {"university_campus", "school"});
    campus.requiresRim = true;
    s.programs.push_back(campus);
    LotProgram park = makeProgram("office_park", 55, 60, 4500, 0.32, 14, 10, 12,
                                  FrontageKind::Forecourt, OpenKind::Lawn, 40,
                                  2, 5, 1.4, {"office_park", "hospital"});
    park.requiresRim = true;
    s.programs.push_back(park);
    LotProgram box = makeProgram("big_box", 48, 44, 3200, 0.52, 10, 6, 8,
                                 FrontageKind::Yard, OpenKind::None, 60, 1, 1, 1.6,
                                 {"big_box", "warehouse"});
    box.requiresRim = true;
    box.service = ServiceKind::Loading;
    s.programs.push_back(box);
    LotProgram works = makeProgram("works", 40, 40, 2400, 0.48, 8, 6, 8,
                                   FrontageKind::Yard, OpenKind::None, 20, 1, 3, 1.2,
                                   {"works", "factory", "warehouse"});
    works.requiresRim = true;
    works.service = ServiceKind::Loading;
    s.programs.push_back(works);
    // Something small must remain eligible, or a rim block with an ordinary
    // parcel on it would have no program at all and silently become lawn.
    s.programs.push_back(makeProgram("cottage", 7, 7, 90, 0.35, 5, 2, 6,
                                     FrontageKind::Garden, OpenKind::Lawn, 1,
                                     1, 2, 0.8, {"cottage_house"}));
    s.reset();
    return s;
}

}  // namespace engine
