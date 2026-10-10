#include "street_furniture.h"

#include <unordered_map>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>

namespace engine {

const char* streetSpeciesName(StreetSpecies s) {
    static const char* const n[] = {"maple", "oak", "beech", "royal_palm", "coconut_palm", "monkeypod", "poinciana", "jacaranda", "plumeria"};
    const int i = static_cast<int>(s);
    return i >= 0 && i < static_cast<int>(StreetSpecies::Count) ? n[i] : "maple";
}

const char* plantingSchemeName(StreetPlanting::Scheme s) {
    static const char* const n[] = {"none", "avenue", "mixed", "sparse", "saplings", "palms", "flowering"};
    return s <= StreetPlanting::Flowering ? n[s] : "?";
}

StreetPlanting choosePlanting(const StreetCharacter& c, RoadClass klass, Real u, Real v) {
    using S = StreetSpecies;
    StreetPlanting pl;
    auto one = [&](StreetPlanting::Scheme sc, S a, Real spacing, Real lo, Real hi) {
        pl.scheme = sc; pl.species[0] = a; pl.kinds = 1; pl.spacing = spacing; pl.scaleLo = lo; pl.scaleHi = hi;
    };
    auto mix = [&](StreetPlanting::Scheme sc, S a, S b, S d, int kinds, Real spacing, Real lo, Real hi) {
        pl.scheme = sc; pl.species[0] = a; pl.species[1] = b; pl.species[2] = d; pl.kinds = kinds;
        pl.spacing = spacing; pl.scaleLo = lo; pl.scaleHi = hi;
    };
    const bool main = klass != RoadClass::Local;
    const S shade[4] = {S::Monkeypod, S::Oak, S::Maple, S::Beech};
    const S bloom[3] = {S::Poinciana, S::Jacaranda, S::Plumeria};
    const S shadeK = shade[std::min(3, static_cast<int>(v * 4))];
    const S bloomK = bloom[std::min(2, static_cast<int>(v * 3))];
    const S palmK = c.coastal ? S::CoconutPalm : S::RoyalPalm;
    switch (c.kind) {
        case StreetCharacter::Rural:
            pl.scheme = StreetPlanting::None;
            break;
        case StreetCharacter::Industrial:   // yards and loading bays: bare kerbs, a few scrappy survivors
            if (u < 0.75) pl.scheme = StreetPlanting::None;
            else { mix(StreetPlanting::Sparse, S::Monkeypod, S::Plumeria, S::Maple, 3, 20.0, 0.7, 1.1); pl.gaps = 0.4; }
            break;
        case StreetCharacter::Downtown:     // the boulevards get their palms or a formal row; the side streets pits of young trees
            if (main) {
                if (u < 0.45) one(StreetPlanting::Palms, palmK, 9.0, 0.95, 1.1);
                else if (u < 0.75) one(StreetPlanting::Avenue, shadeK, 9.0, 0.95, 1.1);
                else pl.scheme = StreetPlanting::None;   // a plaza of glass and nothing green
            } else {
                if (u < 0.35) pl.scheme = StreetPlanting::None;
                else one(StreetPlanting::Saplings, v < 0.5 ? S::Plumeria : shadeK, 11.0, 0.5, 0.65);
            }
            break;
        case StreetCharacter::Commercial:   // the low strips: open to the signs, now and then a row of palms
            if (u < 0.4) pl.scheme = StreetPlanting::None;
            else if (u < 0.7) one(StreetPlanting::Palms, palmK, 12.0, 0.9, 1.1);
            else { mix(StreetPlanting::Sparse, S::Plumeria, S::Monkeypod, S::Poinciana, 3, 16.0, 0.7, 1.0); pl.gaps = 0.25; }
            break;
        case StreetCharacter::OldTown:
        case StreetCharacter::Residential:
        default: {
            const Real age = c.kind == StreetCharacter::OldTown ? std::max(c.age, Real(0.8)) : c.age;
            if (c.coastal && u < 0.45) { one(StreetPlanting::Palms, S::CoconutPalm, 12.0 + 4.0 * v, 0.85, 1.15); pl.gaps = 0.1; break; }
            if (age < 0.3) {   // a new estate: one kind, evenly spaced, still small
                if (u < 0.2) pl.scheme = StreetPlanting::None;
                else one(StreetPlanting::Saplings, v < 0.4 ? bloomK : shadeK, 9.0 + 2.0 * v, 0.42, 0.58);
                break;
            }
            if (c.wealth > 0.65) {   // the leafy streets: a closed canopy, or a street famous for its blossom
                if (u < 0.55) one(StreetPlanting::Avenue, shadeK, 8.0 + 2.0 * v, 1.05, 1.3);
                else one(StreetPlanting::Flowering, bloomK, 9.0 + 2.0 * v, 0.95, 1.2);
                pl.gaps = 0.03;
            } else if (c.wealth < 0.35) {   // the poorer streets: mostly bare, what was planted half gone
                if (u < 0.45) pl.scheme = StreetPlanting::None;
                else { mix(StreetPlanting::Sparse, shadeK, S::Plumeria, S::Monkeypod, 3, 15.0 + 7.0 * v, 0.7, 1.2); pl.gaps = 0.45; }
            } else {   // in between: a mix at a looser spacing, now and then one side only
                mix(StreetPlanting::Mixed, shadeK, bloomK, v < 0.5 ? S::Monkeypod : S::Maple, 3, 11.0 + 3.0 * v, 0.8, 1.1);
                pl.gaps = 0.12;
                pl.oneSide = !main && u > 0.75;
            }
            if (age > 0.7 && pl.scheme != StreetPlanting::None) {   // old streets: bigger trees, more of them lost
                pl.scaleLo *= 1.12; pl.scaleHi *= 1.15; pl.gaps = std::min(Real(0.6), pl.gaps + 0.1);
            }
            break;
        }
    }
    return pl;
}

StreetFurniturePlan planStreetFurniture(
    const NavGraph& nav, const std::function<Real(Real, Real)>& ground,
    const StreetFurnitureParams& p) {
    StreetFurniturePlan out;
    auto gy = [&](Real x, Real z) { return ground ? ground(x, z) : Real(0); };
    // Grade-separated links ride a bridge deck this far up per layer — the same
    // constant CityRenderSystem uses, so poles on a bridge stand on its deck.
    const Real kLayerLift = 5.8;
    // How far a pole looks for asphalt that is not its own junction's, and how far it may be
    // walked to escape it before the placement is abandoned as not-a-corner.
    const Real kPoleSearch = 26.0, kPoleMaxWalk = 12.0, kPoleDropInside = 1.5;

    // WALK A PLACEMENT OFF THE ASPHALT. `out` is the direction away from the road it
    // belongs to (its kerb normal); stepping along that keeps a pole on its own corner
    // and a lamp on its own verge instead of teleporting it somewhere arbitrary.
    // Returns how deep it still is when it gives up — 0 means clear.
    auto walkOffDeck = [&](Vec2& at, const Vec2& out, Real limit) -> Real {
        if (!p.deck) return 0;
        Real moved = 0;
        for (int i = 0; i < 24; ++i) {
            const Real depth = static_cast<Real>(p.deck->depthInside(at.x, at.y));
            if (depth <= 0) return 0;
            const Real step = std::min(Real(0.5) + depth, limit - moved);
            if (step <= Real(1e-3)) return depth;
            at = at + out * step;
            moved += step;
        }
        return static_cast<Real>(p.deck->depthInside(at.x, at.y));
    };

    // Every link bucketed by cell, so "what asphalt is near this point" is a few dozen
    // segment tests rather than the city's thousands.
    const Real kLinkCell = 32.0;
    std::unordered_map<long long, std::vector<int>> poleGrid;
    auto cellOf = [](int cx, int cz) {
        return static_cast<long long>(cx) * 73856093LL ^ static_cast<long long>(cz) * 19349663LL;
    };
    for (int li2 = 0; li2 < nav.linkCount(); ++li2) {
        const NavLink& L2 = nav.links[li2];
        const Vec2 a = nav.nodes[L2.from], b = nav.nodes[L2.to];
        const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) / kLinkCell));
        const int x1 = static_cast<int>(std::floor(std::max(a.x, b.x) / kLinkCell));
        const int z0 = static_cast<int>(std::floor(std::min(a.y, b.y) / kLinkCell));
        const int z1 = static_cast<int>(std::floor(std::max(a.y, b.y) / kLinkCell));
        for (int cx = x0; cx <= x1; ++cx)
            for (int cz = z0; cz <= z1; ++cz) poleGrid[cellOf(cx, cz)].push_back(li2);
    }
    auto forEachLinkNear = [&](const Vec2& q, Real radius, const std::function<void(int)>& fn) {
        const int r = static_cast<int>(std::ceil(radius / kLinkCell));
        const int cx = static_cast<int>(std::floor(q.x / kLinkCell));
        const int cz = static_cast<int>(std::floor(q.y / kLinkCell));
        for (int dx = -r; dx <= r; ++dx)
            for (int dz = -r; dz <= r; ++dz) {
                auto it = poleGrid.find(cellOf(cx + dx, cz + dz));
                if (it == poleGrid.end()) continue;
                for (int li2 : it->second) fn(li2);
            }
    };

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
        // TRY BOTH KERBS (Glenn, 2026-09-21: "the stoplights aren't showing up in some
        // places"). A pole goes on the approach's right-hand corner; where that corner lies
        // inside somebody else's carriageway — a freeway past the end of a street, a ramp
        // gore — it used to be DROPPED, and the junction was left with no drawn signal while
        // the sim went on phasing it (16 of metro_lanes' 292). The left-hand corner, across
        // the same approach, is usually fine. Only when both are buried is there no pole.
        auto tryCorner = [&](Real side, Real reach, Vec2& out) -> bool {
            const Vec2 sideDir = right * side;
            Vec2 corner = node + d * reach + sideDir * w;
            // ...AND OUT OF EVERY OTHER ROAD'S CARRIAGEWAY. The back-off above reasons about
            // the ARMS AT THIS NODE, which is all a lattice city has near a corner. A lane-built
            // city puts a freeway carriageway or a ramp gore right past the end of a street —
            // no arm of this node, and 18 m of asphalt the pole stood in the middle of. Push it
            // clear of any link it is inside, iterating because clearing one can enter another,
            // and giving up rather than walking a pole across the city.
            Real residual = 0;
            for (int pass = 0; pass < 5; ++pass) {
                Vec2 push(0, 0);
                Real worst = 0;
                forEachLinkNear(corner, kPoleSearch, [&](int ol) {
                    if (ol == li) return;
                    const engine::NavLink& O = nav.links[ol];
                    const Vec2 a = nav.nodes[O.from], b = nav.nodes[O.to];
                    const Vec2 ab = b - a;
                    const Real L2 = ab.lengthSquared();
                    const Real u = L2 < Real(1e-9) ? Real(0) : std::clamp(dot(corner - a, ab) / L2, Real(0), Real(1));
                    const Vec2 foot = a + ab * u;
                    Vec2 away = corner - foot;
                    const Real dist = away.length();
                    const Real need = O.width * Real(0.5) + p.curbGap;
                    if (dist >= need) return;
                    if (dist < Real(1e-6)) {
                        const Vec2 n2(-ab.y, ab.x);
                        const Real nl = n2.length();
                        away = nl > Real(1e-9) ? n2 * (Real(1) / nl) : Vec2(1, 0);
                    } else {
                        away = away * (Real(1) / dist);
                    }
                    const Real gap = need - dist;
                    if (gap > worst) { worst = gap; push = away * gap; }
                });
                residual = worst;
                if (worst <= Real(1e-3)) break;
                if ((corner + push - node).length() > kPoleMaxWalk) break;   // not a corner any more
                corner = corner + push;
                residual = 0;
            }
            // A corner a few centimetres inside its OWN street is just a kerb — a two-way road
            // is two links sharing one centreline, so every correct pole reads as marginally
            // inside the opposite direction's carriageway. Only a pole genuinely BURIED fails.
            if (residual > kPoleDropInside) return false;
            // ...and off the DRAWN asphalt, which the link widths above only approximate —
            // ALL the way off. The kerb allowance is for the link-width estimate (two links on
            // one centreline); the deck has no such ambiguity, and every lattice pole ends at
            // exactly 0 here. A pole still inside it (one on metro_lanes, 0.30 m into a
            // carriageway after a 12 m walk) tries the other corner instead.
            if (walkOffDeck(corner, sideDir, kPoleMaxWalk) > Real(0)) return false;
            out = corner;
            return true;
        };
        Vec2 corner;
        // ...and then ACROSS the junction. A far-side signal — on the exit corner, facing the
        // approaching traffic — is the ordinary North American arrangement, and it is where a
        // light goes when both near corners are asphalt (a ramp gore on one side, a freeway
        // past the end of the street on the other).
        const Real farReach = crossHalf + p.sidewalkWidth + spread + p.curbGap;
        if (!tryCorner(Real(1), -t, corner) && !tryCorner(Real(-1), -t, corner) &&
            !tryCorner(Real(1), farReach, corner) && !tryCorner(Real(-1), farReach, corner)) {
            out.unpoledApproaches.push_back(node);
            continue;
        }
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
            // ...and off the DRAWN asphalt. insideAnyCarriageway reasons from link
            // WIDTHS, which is its own road's width — a lamp clears the street it is
            // marching along and stands on the junction pad or on the wider road
            // crossing behind it. Lamps are sampled every 2.5 m along a kerb, so
            // dropping one costs nothing; a signal pole gets walked out instead.
            if (p.deck && p.deck->depthInside(sp.x, sp.y) > 0.0) continue;
            if (tooClose(sp, dir)) continue;
            const Real y = gy(sp.x, sp.y) + L.layer * kLayerLift;
            out.lampBases.push_back(Vec3(sp.x, y, sp.y));
            out.lampHeads.push_back(Vec3(sp.x, y + p.lampHeight, sp.y));
            const int cx = static_cast<int>(std::floor(sp.x / cell));
            const int cz = static_cast<int>(std::floor(sp.y / cell));
            grid[cellKey(cx, cz)].push_back({sp, dir});
        }
    }
    if (!p.kerbLife) return out;

    // KERB LIFE: every city street that has a pavement and fronts lots (not a freeway, a ramp, a bridge, the road
    // between towns): street trees in pits at treeSpacing, kept off the lamps, the signal poles and the junctions'
    // sightlines; a litter bin by each corner; bike racks and news boxes along the main streets. The same sampling and
    // the same "off the asphalt" rules as the lamps, each kind spaced along its own kerb (direction-aware).
    struct Placed { Vec2 at, dir; };
    auto keyOf = [&](const Vec2& v, Real c) { return cellKey(static_cast<int>(std::floor(v.x / c)), static_cast<int>(std::floor(v.y / c))); };
    struct Spaced {
        Real spacing;
        std::unordered_map<long long, std::vector<Placed>> g;
    };
    auto near = [&](Spaced& s, Vec2 v, Vec2 d, Real r, bool sameKerb) {
        const int cx = static_cast<int>(std::floor(v.x / s.spacing)), cz = static_cast<int>(std::floor(v.y / s.spacing));
        for (int dz = -1; dz <= 1; ++dz)
            for (int dx = -1; dx <= 1; ++dx) {
                auto it = s.g.find(cellKey(cx + dx, cz + dz));
                if (it == s.g.end()) continue;
                for (const Placed& q : it->second)
                    if ((!sameKerb || dot(q.dir, d) > 0) && (q.at - v).length() < r) return true;
            }
        return false;
    };
    auto add = [&](Spaced& s, Vec2 v, Vec2 d) { s.g[keyOf(v, s.spacing)].push_back({v, d}); };
    Spaced trees{p.character ? Real(24) : std::max(p.treeSpacing, Real(4)), {}}, bins{30.0, {}}, props{std::max(p.propSpacing, Real(10)), {}};
    Spaced lamps{12.0, {}}, poles{12.0, {}};
    for (const Vec3& b : out.lampBases) add(lamps, Vec2(b.x, b.z), Vec2(0, 0));
    for (const SignalSpot& s : out.signals) add(poles, Vec2(s.base.x, s.base.z), Vec2(0, 0));
    auto streetWithPavement = [&](const NavLink& L) {
        return (L.klass == RoadClass::Local || L.klass == RoadClass::Collector || L.klass == RoadClass::Arterial) &&
               L.layer == 0 && !L.elevAbsolute && L.walkable && (L.access & road_access::kFrontage) && L.width <= p.maxLampRoadWidth;
    };
    auto clearOfRoad = [&](const Vec2& v, Real margin) {
        if (insideAnyCarriageway(v)) return false;
        if (!p.deck) return true;
        // depthInside is 0 off the deck: the point and four around it at `margin` must all be off it
        for (const Vec2 o : {Vec2(0, 0), Vec2(margin, 0), Vec2(-margin, 0), Vec2(0, margin), Vec2(0, -margin)})
            if (p.deck->depthInside(v.x + o.x, v.y + o.y) > 0.0) return false;
        return true;
    };
    auto distToJunction = [&](const NavLink& L, const Vec2& v) {
        Real d = 1e30;
        if (nav.isJunction(L.to)) d = std::min(d, (v - nav.nodes[L.to]).length());
        if (nav.isJunction(L.from)) d = std::min(d, (v - nav.nodes[L.from]).length());
        return d;
    };
    // THE KERB AS DRAWN: a link's width is its travel lanes, and a street with parking bands has its kerb a
    // band further out -- so walk out from the centreline until off the asphalt, then add the verge
    auto kerbPoint = [&](int li, Real t, Real verge) {
        const Vec2 c = nav.pointOnLink(li, t);
        const Vec2 s1 = nav.sidewalkPoint(li, t, 1.0);
        if (!p.deck || (s1 - c).length() < 1e-6) return nav.sidewalkPoint(li, t, verge);
        const Vec2 n = normalize(s1 - c);
        Real s = nav.links[li].width * 0.5;
        while (s < 16.0 && p.deck->depthInside(c.x + n.x * s, c.y + n.y * s) > 0.0) s += 0.25;
        return c + n * (s + verge);
    };
    uint32_t h = 0x5EED7u;
    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& L = nav.links[li];
        if (!streetWithPavement(L)) continue;
        const Vec2 a = nav.nodes[L.from], b = nav.nodes[L.to];
        const Real len = (b - a).length();
        if (len < Real(0.5)) continue;
        const Vec2 dir = (b - a) * (Real(1) / len);
        const bool main = L.klass != RoadClass::Local;
        const int steps = std::max(1, static_cast<int>(std::ceil(len / Real(1.5))));
        // THIS STREET'S PLANTING: by its character, and the same all along it -- the draw is keyed on the street's
        // LINE (its bearing to 10 degrees, its offset across to 30 m, 700 m stretches along it, its class), so both
        // directions of a two-way street and every block of a straight one agree, and the next street over may not
        StreetPlanting pl;
        pl.spacing = p.treeSpacing;
        Vec2 lineDir = dir;
        if (p.character) {
            constexpr Real kPi = 3.14159265358979;
            const Real kBear = kPi / 18.0;
            Real ang = std::atan2(dir.y, dir.x);
            if (ang < 0) ang += kPi;
            if (ang >= kPi) ang -= kPi;
            const int ab = static_cast<int>(std::lround(ang / kBear)) % 18;
            lineDir = Vec2(std::cos(ab * kBear), std::sin(ab * kBear));
            const Vec2 mid = (a + b) * Real(0.5);
            const long long off = static_cast<long long>(std::llround(dot(mid, Vec2(-lineDir.y, lineDir.x)) / 30.0));
            const long long along = static_cast<long long>(std::floor(dot(mid, lineDir) / 700.0));
            uint64_t k = 0x9E3779B97F4A7C15ull ^ (static_cast<uint64_t>(ab) * 0xBF58476D1CE4E5B9ull) ^
                         (static_cast<uint64_t>(off) * 0x94D049BB133111EBull) ^ (static_cast<uint64_t>(along) * 0xD6E8FEB86659FD93ull) ^
                         static_cast<uint64_t>(L.klass) * 0x2545F4914F6CDD1Dull;
            k ^= k >> 31; k *= 0xBF58476D1CE4E5B9ull; k ^= k >> 29;
            pl = choosePlanting(p.character(mid), L.klass, (k & 0xFFFF) / 65536.0, ((k >> 16) & 0xFFFF) / 65536.0);
            ++out.planted[std::min<int>(pl.scheme, 7)];
        }
        const bool plantHere = !p.character || (pl.scheme != StreetPlanting::None && (!pl.oneSide || dot(dir, lineDir) > 0));
        for (int k = 0; k <= steps; ++k) {
            const Real t = static_cast<Real>(k) / steps;
            const Real dj = distToJunction(L, kerbPoint(li, t, p.treeVerge));
            // A STREET TREE
            {
                const Vec2 sp = kerbPoint(li, t, p.treeVerge);
                if (plantHere && dj >= p.treeJunctionClear && clearOfRoad(sp, 0.4) && !near(trees, sp, dir, pl.spacing * 0.92, true) &&
                    !near(lamps, sp, dir, 3.2, false) && !near(poles, sp, dir, 3.5, false) && !near(bins, sp, dir, 2.0, false) &&
                    !near(props, sp, dir, 2.5, false)) {
                    h = h * 1664525u + 1013904223u;
                    if (!p.character) {
                        const uint32_t v = (h >> 16) % 3u;
                        out.trees.push_back({Vec3(sp.x, gy(sp.x, sp.y), sp.y), Real(0.75) + (h >> 8 & 0xFF) / 255.0 * 0.35, v,
                                             static_cast<StreetSpecies>(v)});
                    } else if ((h >> 24) / 256.0 >= pl.gaps) {   // (a gap keeps its place in the row: the pit stays empty)
                        const uint32_t h2 = h * 2654435761u;
                        out.trees.push_back({Vec3(sp.x, gy(sp.x, sp.y), sp.y),
                                             pl.scaleLo + (pl.scaleHi - pl.scaleLo) * ((h >> 8) & 0xFF) / 255.0, (h2 >> 20) % 3u,
                                             pl.species[(h >> 16) % static_cast<uint32_t>(std::max(1, pl.kinds))]});
                    }
                    add(trees, sp, dir);
                }
            }
            // A LITTER BIN by the corner, just past the junction's clear zone, near the kerb
            {
                const Vec2 sp = kerbPoint(li, t, 1.0);   // (past curbGap: insideAnyCarriageway pads every kerb by it)
                if (dj >= p.junctionClear && dj < p.junctionClear + 4.0 && clearOfRoad(sp, 0.2) && !near(bins, sp, dir, 25.0, true) &&
                    !near(trees, sp, dir, 1.6, false) && !near(lamps, sp, dir, 1.2, false) && !near(poles, sp, dir, 1.5, false)) {
                    out.props.push_back({Vec3(sp.x, gy(sp.x, sp.y), sp.y), normalize(sp - nav.pointOnLink(li, t)), KerbProp::Bin});
                    add(bins, sp, dir);
                }
            }
            // A BIKE RACK or NEWS BOXES along a main street, mid-block
            if (main && dj > 25.0) {
                const Vec2 sp = kerbPoint(li, t, 1.0);
                if (clearOfRoad(sp, 0.3) && !near(props, sp, dir, p.propSpacing, true) && !near(trees, sp, dir, 2.6, false) &&
                    !near(lamps, sp, dir, 2.0, false) && !near(poles, sp, dir, 2.5, false)) {
                    h = h * 1664525u + 1013904223u;
                    out.props.push_back({Vec3(sp.x, gy(sp.x, sp.y), sp.y), normalize(sp - nav.pointOnLink(li, t)), static_cast<uint8_t>((h >> 12) % 3u ? KerbProp::BikeRack : KerbProp::NewsBoxes)});
                    add(props, sp, dir);
                }
            }
        }
    }
    return out;
}

}  // namespace engine
