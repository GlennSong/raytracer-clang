#include "street_furniture.h"

#include <unordered_map>

#include <algorithm>
#include <cmath>
#include <map>

namespace engine {

StreetFurniturePlan planStreetFurniture(
    const NavGraph& nav, const std::function<Real(Real, Real)>& ground,
    const StreetFurnitureParams& p) {
    StreetFurniturePlan out;
    auto gy = [&](Real x, Real z) { return ground ? ground(x, z) : Real(0); };
    // Grade-separated links ride a bridge deck this far up per layer — the same
    // constant CityRenderSystem uses, so poles on a bridge stand on its deck.
    const Real kLayerLift = 5.8;

    // SIGNALS: one per (junction, approach BEARING) — matching the sim's
    // SignalController for real this time. Two mismatches used to break the
    // "poles and phases agree" claim:
    //   - the controller only signalises 4+-approach junctions (T-junctions
    //     stay uncontrolled, by device feedback), but poles were planted at
    //     EVERY junction — a dark, dead head on every T;
    //   - the unified graph can hold OVERLAPPING COLLINEAR edges (a long
    //     edge plus a stub riding it), and per-LINK placement stood two heads
    //     on one arm (device: "double stoplights which is bizarre").
    // So: skip junctions the controller leaves uncontrolled, quantise each
    // approach bearing to a 15-degree bin per node, and keep ONE pole per
    // bin — the widest approach, then the longest, then the lowest link
    // index (deterministic). Phases stay per-LINK in the sim (stateForLink),
    // so a deduped twin still resolves its phase; only its redundant pole is
    // gone. The pole stands on the near-right curb corner, backed off by the
    // widest crossing road (and a knot-merged junction's spread) so it never
    // lands in a carriageway.
    std::map<std::pair<int, int>, int> armBest;   // (node, bearing bin) -> link
    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& L = nav.links[li];
        // Semantic signal predicate (#17/S5), shared with the controller and
        // the render markings so all three agree exactly.
        if (!(L.access & road_access::kSignalable)) continue;
        if (!nav.signalControlled(L.to)) continue;
        Vec2 d = nav.direction(li);
        int bin = static_cast<int>(std::lround(std::atan2(d.y, d.x) /
                                               (PI / 12.0)));
        bin = ((bin % 24) + 24) % 24;
        const auto key = std::make_pair(L.to, bin);
        auto it = armBest.find(key);
        if (it != armBest.end()) {
            const NavLink& W = nav.links[it->second];
            const bool better =
                L.width > W.width ||
                (L.width == W.width &&
                 (L.length > W.length ||
                  (L.length == W.length && li < it->second)));
            if (!better) continue;
            it->second = li;
        } else {
            armBest.emplace(key, li);
        }
    }
    // Every ARM at every node — outward unit direction + carriageway half —
    // from ALL links (an inbound-only one-way arm is not in outLinks).
    struct Arm { Vec2 dir; Real half; };
    std::vector<std::vector<Arm>> armsAt(nav.nodes.size());
    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& K = nav.links[li];
        const Vec2 kd = nav.direction(li);
        if (K.from >= 0 && K.from < static_cast<int>(armsAt.size()))
            armsAt[K.from].push_back({kd, K.width * Real(0.5)});
        if (K.to >= 0 && K.to < static_cast<int>(armsAt.size()))
            armsAt[K.to].push_back({Vec2(-kd.x, -kd.y), K.width * Real(0.5)});
    }
    for (const auto& kv : armBest) {
        const int li = kv.second;
        const NavLink& L = nav.links[li];
        Vec2 d = nav.direction(li);
        Vec2 node = nav.nodes[L.to];
        Vec2 right(d.y, -d.x);
        Real thisHalf = L.width * 0.5;
        Real crossHalf = thisHalf;
        for (int ol : nav.outLinks[L.to])
            crossHalf = std::max(crossHalf, nav.links[ol].width * 0.5);
        Real spread = L.to < static_cast<int>(nav.nodeSpread.size())
                          ? nav.nodeSpread[L.to] : Real(0);
        // ON the kerb, not in the box: the drawn junction pad fills the disc
        // out to (crossHalf + sidewalk), so the back-off must clear THAT — the
        // old (crossHalf + curbGap) planted the pole ~a sidewalk-width inside
        // the asphalt. Laterally the pole hugs the kerb on the sidewalk band;
        // the street_kit mast arm (4.2 m) then reaches over the near lane,
        // which is the whole point of the arm.
        const Real w = thisHalf + p.curbGap;
        Real t = crossHalf + p.sidewalkWidth + spread + p.curbGap;
        // ACUTE-CORNER CLEARANCE (device, metro: "stoplights show up in the
        // middle of the street and not on the corners"). The back-off above
        // assumes a right-angle cross: it clears the pad DISC and hugs the
        // pole's OWN kerb, but says nothing about the NEIGHBOURING arm. Where
        // two arms meet at an acute angle the neighbour's carriageway sweeps
        // through the "corner" — at 60 degrees the pole stood ~5.5 m from
        // its centreline, inside a 6 m half-width, squarely on the pad. The
        // kerb corner of an acute block is FURTHER back along the approach
        // (the same reason road_net's acute-pair trim extends a body until
        // its centreline clears the sibling's ribbon), so: walk the pole back
        // along its own kerb line until it clears EVERY arm at the node by
        // (that arm's half + curbGap). Per arm the clearance |a t + b| < h is
        // one interval in t; raising t past one arm's interval can land in
        // another's, so iterate to a fixed point (a few passes suffice).
        const std::vector<Arm>& arms = armsAt[L.to];
        for (int pass = 0; pass < 4; ++pass) {
            Real tNext = t;
            for (const Arm& arm : arms) {
                // P(t) - node = (-d) t + right w; perp to arm = a t + b.
                const Real a = arm.dir.x * (-d.y) - arm.dir.y * (-d.x);
                const Real b = (arm.dir.x * right.y - arm.dir.y * right.x) * w;
                const Real h = arm.half + p.curbGap;
                if (std::fabs(a) < 1e-6) continue;   // own / opposite arm
                const Real along = arm.dir.x * (-d.x * t + right.x * w) +
                                   arm.dir.y * (-d.y * t + right.y * w);
                if (along <= 0) continue;            // behind the node
                if (std::fabs(a * t + b) >= h) continue;
                const Real root = -b / a, halfBad = h / std::fabs(a);
                tNext = std::max(tNext, root + halfBad);
            }
            if (tNext <= t + 1e-9) break;
            t = std::min(tNext, Real(30.0));         // a fork this sharp is a
                                                     // gore, not a corner
        }
        Vec2 corner = node - d * t + right * w;
        SignalSpot s;
        s.base = Vec3(corner.x, gy(corner.x, corner.y) + L.layer * kLayerLift,
                      corner.y);
        s.face = Vec3(-d.x, 0, -d.y);   // head faces its approaching traffic
        s.link = li;
        out.signals.push_back(s);
    }

    // LAMPS: march each link's RIGHT sidewalk — a two-way road contributes one
    // directed link per direction, so both sides light up and the pattern
    // alternates naturally. Freeway-width carriageways have no sidewalk to
    // stand on; junction mouths are left to the signal poles.
    // LAMPS every `lampSpacing` metres of KERB — measured along the street,
    // NOT per graph link. A nav link is a tessellation segment: on this city
    // they average 6.7 m, and the old per-link rule ("one lamp at the middle
    // of any link longer than 0.6 * spacing") therefore threw away 7120 of
    // 7288 links and lit only the rare long straight. That is why the city
    // was dark. Sampling each kerb finely and enforcing the spacing with a
    // grid gives even lighting whatever the tessellation does.
    //
    // The spacing test is DIRECTION AWARE: the two kerbs of a two-way street
    // are only a carriageway apart (12-17 m, inside the spacing), so a plain
    // radius test would let one side suppress the other and light every
    // street down one side only. Lamps facing opposite ways are different
    // kerbs and never suppress each other.
    struct PlacedLamp { Vec2 at; Vec2 dir; };
    std::unordered_map<long long, std::vector<PlacedLamp>> grid;
    const Real cell = std::max(Real(1), p.lampSpacing);
    auto cellKey = [](int cx, int cz) {
        return static_cast<long long>(cx) * 73856093LL ^
               static_cast<long long>(cz) * 19349663LL;
    };
    auto tooClose = [&](Vec2 v, Vec2 d) {
        const int cx = static_cast<int>(std::floor(v.x / cell));
        const int cz = static_cast<int>(std::floor(v.y / cell));
        for (int dz = -1; dz <= 1; ++dz)
            for (int dx = -1; dx <= 1; ++dx) {
                auto it = grid.find(cellKey(cx + dx, cz + dz));
                if (it == grid.end()) continue;
                for (const PlacedLamp& q : it->second)
                    if (dot(q.dir, d) > 0 &&
                        (q.at - v).length() < p.lampSpacing * Real(0.85))
                        return true;
            }
        return false;
    };

    // EVERY carriageway, hashed, so a candidate can ask "am I inside ANY
    // road's ribbon?" (device, metro map: "some of the street lights are
    // actually sitting right in the middle of the road"). The junction-clear
    // radius below is measured from the NODE; at a T or an acute corner the
    // neighbouring arm's carriageway reaches past that radius, and a kerb
    // lamp of one street stood in the middle of the other — 75 of the
    // metro's 1739 lamps, the worst 5.8 m inside a 12 m road. The rule the
    // signal poles already obey (9176f6a) applied to lamps: clear every
    // link's half-width by curbGap. A lamp's OWN link passes trivially
    // (it stands lampVerge > curbGap beyond that kerb).
    const Real lcell = 20.0;   // >= widest half-width + curbGap, so 3x3 covers
    std::unordered_map<long long, std::vector<int>> linkGrid;
    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& K = nav.links[li];
        const Vec2 a = nav.nodes[K.from], b = nav.nodes[K.to];
        const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) / lcell)) - 1;
        const int x1 = static_cast<int>(std::floor(std::max(a.x, b.x) / lcell)) + 1;
        const int z0 = static_cast<int>(std::floor(std::min(a.y, b.y) / lcell)) - 1;
        const int z1 = static_cast<int>(std::floor(std::max(a.y, b.y) / lcell)) + 1;
        for (int cz = z0; cz <= z1; ++cz)
            for (int cx = x0; cx <= x1; ++cx)
                linkGrid[cellKey(cx, cz)].push_back(li);
    }
    auto insideAnyCarriageway = [&](const Vec2& v) {
        const int cx = static_cast<int>(std::floor(v.x / lcell));
        const int cz = static_cast<int>(std::floor(v.y / lcell));
        auto it = linkGrid.find(cellKey(cx, cz));
        if (it == linkGrid.end()) return false;
        for (int li : it->second) {
            const NavLink& K = nav.links[li];
            const Vec2 a = nav.nodes[K.from], b = nav.nodes[K.to];
            const Vec2 ab = b - a;
            const Real l2 = ab.lengthSquared();
            Real t = l2 > 1e-12 ? dot(v - a, ab) / l2 : Real(0);
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            if ((a + ab * t - v).length() < K.width * Real(0.5) + p.curbGap)
                return true;
        }
        return false;
    };

    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& L = nav.links[li];
        if (L.klass == RoadClass::Freeway || L.klass == RoadClass::Ramp)
            continue;   // corridor lighting is its own pass, not lamps
        if (L.width > p.maxLampRoadWidth) continue;
        const Vec2 a = nav.nodes[L.from], b = nav.nodes[L.to];
        const Real len = (b - a).length();
        if (len < Real(0.5)) continue;
        const Vec2 dir = (b - a) * (Real(1) / len);
        // Sample finely along the segment; the grid decides what survives.
        const int steps = std::max(1, static_cast<int>(std::ceil(len / Real(2.5))));
        for (int k = 0; k <= steps; ++k) {
            const Real t = static_cast<Real>(k) / steps;
            const Vec2 sp = nav.sidewalkPoint(li, t, p.lampVerge);
            // Clear of real JUNCTIONS only — a nav node is usually just a
            // curve vertex, and treating those as junctions blanked the
            // curved avenues that make up most of this city.
            const bool nearJunction =
                (nav.isJunction(L.to) &&
                 (sp - nav.nodes[L.to]).length() < p.junctionClear) ||
                (nav.isJunction(L.from) &&
                 (sp - nav.nodes[L.from]).length() < p.junctionClear);
            if (nearJunction) continue;
            if (insideAnyCarriageway(sp)) continue;
            if (tooClose(sp, dir)) continue;
            const Real y = gy(sp.x, sp.y) + L.layer * kLayerLift;
            out.lampBases.push_back(Vec3(sp.x, y, sp.y));
            out.lampHeads.push_back(Vec3(sp.x, y + p.lampHeight, sp.y));
            const int cx = static_cast<int>(std::floor(sp.x / cell));
            const int cz = static_cast<int>(std::floor(sp.y / cell));
            grid[cellKey(cx, cz)].push_back({sp, dir});
        }
    }
    return out;
}

}  // namespace engine
