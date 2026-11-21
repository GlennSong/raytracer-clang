#include "street_furniture.h"
#include "roads/road_model.h"
#include "../../../proc_rng.h"

#include <unordered_map>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>

namespace engine {

const char* streetSpeciesName(StreetSpecies s) {
    static const char* const n[] = {"maple", "oak", "beech", "royal_palm", "coconut_palm", "monkeypod", "poinciana", "jacaranda", "plumeria",
                                    "pine", "spruce", "birch"};
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
    // THE HIGH GROUND (Glenn, 2026-11-04, of a hamlet 350 m up: "palm trees? That doesn't seem to make sense on a
    // mountain ... I would imagine we'd have more pine trees"): above the upland line no palm and no tropical tree --
    // the mountain's own pines, spruces and birches instead -- and the higher the street, the more of its broadleaves
    // are conifers too
    if (c.highland > 0) {
        const S hill[3] = {S::Pine, S::Spruce, S::Birch};
        const Real w = u * 7.31 + v * 3.17, draw = w - std::floor(w);   // a third draw of its own
        for (int k = 0; k < 3; ++k) {
            S& sp = pl.species[k];
            const bool tropical = sp == S::RoyalPalm || sp == S::CoconutPalm || sp == S::Monkeypod || sp == S::Poinciana ||
                                  sp == S::Jacaranda || sp == S::Plumeria;
            if (tropical || draw < c.highland) sp = hill[(k + static_cast<int>(v * 3)) % 3];
        }
        if (pl.scheme == StreetPlanting::Palms || pl.scheme == StreetPlanting::Flowering) pl.scheme = StreetPlanting::Avenue;
    }
    return pl;
}

KerbLine::KerbLine(const std::vector<Poly2>* loops, Real reach) : loops_(loops), reach_(reach) {
    if (!loops_) return;
    for (const Poly2& L : *loops_) {
        Box b{Real(1e30), Real(1e30), Real(-1e30), Real(-1e30)};
        for (const Vec2& q : L) { b.x0 = std::min(b.x0, q.x); b.z0 = std::min(b.z0, q.y); b.x1 = std::max(b.x1, q.x); b.z1 = std::max(b.z1, q.y); }
        box_.push_back(b);
    }
}

Real KerbLine::depth(const Vec2& at) const {
    if (!loops_) return 0;
    bool inside = false;
    for (std::size_t k = 0; k < box_.size(); ++k) {
        const Box& b = box_[k];
        if (at.x < b.x0 || at.x > b.x1 || at.y < b.z0 || at.y > b.z1) continue;
        if (pointInPolygon((*loops_)[k], at)) inside = !inside;
    }
    if (!inside) return 0;
    Real best = reach_;   // (the edges further off than this cannot be nearest to what is being asked about)
    for (std::size_t k = 0; k < box_.size(); ++k) {
        const Box& b = box_[k];
        if (at.x < b.x0 - reach_ || at.x > b.x1 + reach_ || at.y < b.z0 - reach_ || at.y > b.z1 + reach_) continue;
        const Poly2& L = (*loops_)[k];
        for (std::size_t i = 0; i < L.size(); ++i) {
            const Vec2 a = L[i], c = L[(i + 1) % L.size()], ac = c - a;
            const Real l2 = ac.lengthSquared();
            const Real u = l2 < Real(1e-12) ? Real(0) : std::clamp(dot(at - a, ac) / l2, Real(0), Real(1));
            best = std::min(best, (a + ac * u - at).length());
        }
    }
    return best;
}

namespace {

// ONE PIECE OF A STREET'S CENTRE LINE (two neighbouring points of its line, RoadModel::linePoints): the planner marches
// along these, a kerb at a time. Read off the road model; nothing is kept but what the placements ask.
struct StreetPiece {
    Vec2 a, b;
    Real half = 0;            // the spine to the carriageway's edge (ModelSection::carriagewayHalfWidth)
    RoadClass klass = RoadClass::Local;
    bool cars = true;         // cars drive it (not a footway or a track)
    bool sidewalk[2] = {false, false};   // a sidewalk along the +1 kerb, the -1 kerb (ModelSection::sidewalkOnKerb)
    int city = 0, street = -1;
};

uint64_t mix64(uint64_t k) {
    k ^= k >> 31; k *= 0xBF58476D1CE4E5B9ull; k ^= k >> 29;
    return k;
}

}  // namespace

StreetFurniturePlan planStreetFurniture(
    const std::vector<FurnitureCity>& cities, const std::function<Real(Real, Real)>& ground,
    const StreetFurnitureParams& p) {
    StreetFurniturePlan out;
    auto gy = [&](Real x, Real z) { return ground ? ground(x, z) : Real(0); };
    // How far a pole may be walked to escape asphalt before the placement is abandoned as not-a-corner.
    const Real kPoleMaxWalk = 12.0, kPoleDropInside = 1.5, kPoleSearch = 26.0;

    // THE STREETS AS PIECES, and every junction's centre and arm mouths (lamps, trees and kerb props keep clear)
    std::vector<StreetPiece> pieces;
    std::vector<Vec2> junctionPoints;
    for (const FurnitureCity& fc : cities) {
        if (!fc.model || !fc.model->built) continue;
        const RoadModel& m = *fc.model;
        for (uint32_t s = 0; s < m.streets.size(); ++s) {
            const ModelStreet& S = m.streets[s];
            if (S.line.count < 2) continue;
            for (uint32_t i = S.line.first; i + 1 < S.line.end(); ++i) {
                StreetPiece pc;
                pc.a = m.linePoints[i];
                pc.b = m.linePoints[i + 1];
                if ((pc.b - pc.a).length() < Real(1e-3)) continue;
                pc.half = S.section.carriagewayHalfWidth();
                pc.klass = S.section.klass;
                pc.cars = S.section.carriesCars();
                pc.sidewalk[0] = S.section.sidewalkOnKerb(1);
                pc.sidewalk[1] = S.section.sidewalkOnKerb(-1);
                pc.city = fc.city;
                pc.street = static_cast<int>(s);
                pieces.push_back(pc);
            }
        }
        for (uint32_t j = 0; j < m.junctions.size(); ++j) {
            const ModelJunction& J = m.junctions[j];
            junctionPoints.push_back(J.at);
            for (std::size_t ai = 0; ai < J.arms.size(); ++ai) {
                Vec2 at;
                if (armMouth(m, J, ai, at)) junctionPoints.push_back(at);
            }
        }
    }

    // WALK A PLACEMENT OFF THE ASPHALT. `outDir` is the direction away from the road it belongs to (its kerb normal);
    // stepping along that keeps a pole on its own corner and a lamp on its own verge. Returns how deep it still is when
    // it gives up -- 0 means clear.
    const KerbLine kerb(p.kerbLoops);
    auto depthAt = [&](const Vec2& q) {
        return std::max(p.deck ? static_cast<Real>(p.deck->depthInside(q.x, q.y)) : Real(0), kerb.depth(q));
    };
    auto walkOffDeck = [&](Vec2& at, const Vec2& outDir, Real limit) -> Real {
        if (!p.deck && !p.kerbLoops) return 0;
        Real moved = 0;
        for (int i = 0; i < 24; ++i) {
            const Real depth = depthAt(at);
            if (depth <= 0) return 0;
            const Real step = std::min(Real(0.5) + depth, limit - moved);
            if (step <= Real(1e-3)) return depth;
            at = at + outDir * step;
            moved += step;
        }
        return depthAt(at);
    };

    // EVERY CARRIAGEWAY, bucketed by cell, so a candidate can ask "am I inside ANY road's carriageway?" (device, metro
    // map: "some of the street lights are actually sitting right in the middle of the road") -- clear every street's
    // half-width by curbGap
    auto cellKey = [](int cx, int cz) { return static_cast<long long>(cx) * 73856093LL ^ static_cast<long long>(cz) * 19349663LL; };
    const Real lcell = 20.0;
    std::unordered_map<long long, std::vector<int>> pieceGrid;
    for (int i = 0; i < static_cast<int>(pieces.size()); ++i) {
        const StreetPiece& K = pieces[static_cast<std::size_t>(i)];
        const int x0 = static_cast<int>(std::floor(std::min(K.a.x, K.b.x) / lcell)) - 1;
        const int x1 = static_cast<int>(std::floor(std::max(K.a.x, K.b.x) / lcell)) + 1;
        const int z0 = static_cast<int>(std::floor(std::min(K.a.y, K.b.y) / lcell)) - 1;
        const int z1 = static_cast<int>(std::floor(std::max(K.a.y, K.b.y) / lcell)) + 1;
        for (int cz = z0; cz <= z1; ++cz)
            for (int cx = x0; cx <= x1; ++cx) pieceGrid[cellKey(cx, cz)].push_back(i);
    }
    auto forEachPieceNear = [&](const Vec2& q, Real radius, const std::function<void(int)>& fn) {
        const int r = static_cast<int>(std::ceil(radius / lcell));
        const int cx = static_cast<int>(std::floor(q.x / lcell)), cz = static_cast<int>(std::floor(q.y / lcell));
        std::vector<int> seen;
        for (int dx = -r; dx <= r; ++dx)
            for (int dz = -r; dz <= r; ++dz) {
                auto it = pieceGrid.find(cellKey(cx + dx, cz + dz));
                if (it == pieceGrid.end()) continue;
                for (int i : it->second)
                    if (std::find(seen.begin(), seen.end(), i) == seen.end()) {
                        seen.push_back(i);
                        fn(i);
                    }
            }
    };
    auto insideAnyCarriageway = [&](const Vec2& v) {
        auto it = pieceGrid.find(cellKey(static_cast<int>(std::floor(v.x / lcell)), static_cast<int>(std::floor(v.y / lcell))));
        if (it == pieceGrid.end()) return false;
        for (int i : it->second) {
            const StreetPiece& K = pieces[static_cast<std::size_t>(i)];
            const Vec2 ab = K.b - K.a;
            const Real l2 = ab.lengthSquared();
            const Real t = l2 > 1e-12 ? std::clamp(dot(v - K.a, ab) / l2, Real(0), Real(1)) : Real(0);
            if ((K.a + ab * t - v).length() < K.half + p.curbGap) return true;
        }
        return false;
    };

    // SIGNALS: one pole per ARRIVING ARM of every lit junction (its stated control, junctionLit; with none stated, a
    // junction of four arms or more), on the arm's kerb corner at its mouth, facing the traffic arriving; where that
    // corner is somebody else's asphalt, the other kerb, then the far side of the junction. (Was: one per street-map
    // link entering a lit node, deduplicated to one per arm.)
    for (const FurnitureCity& fc : cities) {
        if (!fc.model || !fc.model->built) continue;
        const RoadModel& m = *fc.model;
        for (uint32_t j = 0; j < m.junctions.size(); ++j) {
            const ModelJunction& J = m.junctions[j];
            const bool lit = p.controlsStated ? junctionLit(J) : J.arms.size() >= 4;
            if (!lit) continue;
            for (std::size_t ai = 0; ai < J.arms.size(); ++ai) {
                const ModelArm& arm = J.arms[ai];
                if (!arm.street.valid() || arm.street.index >= m.streets.size()) {
                    ++out.unarmedApproaches;
                    continue;
                }
                const ModelSection& sec = m.streets[arm.street.index].section;
                if (!sec.carriesCars() || sec.lanesArriving(arm.spineIn) == 0) continue;   // nothing arrives by it
                // THE MOUTH: where the junction's area begins along the arm's street (ModelArm::boxEdge; not stated:
                // 10 m back from the arm's end), on the street's centre line (frontageOn)
                const Real back = arm.spineIn ? Real(-1) : Real(1);   // (along the spine, away from the junction)
                const Real mouthStation = !std::isnan(arm.boxEdge) && std::fabs(arm.boxEdge - arm.station) > Real(0.5)
                                              ? arm.boxEdge : arm.station + back * 10.0;
                const RoadFrontage mf = m.frontageOn(arm.street, mouthStation, 1);
                const RoadFrontage af = m.frontageOn(arm.street, mouthStation + back * 5.0, 1);
                if (!mf.valid() || !af.valid()) {
                    ++out.unarmedApproaches;
                    continue;
                }
                const Vec2 mouth = mf.on;
                Vec2 d = mf.on - af.on;   // the way traffic arrives (toward the junction)
                const Real step = d.length();
                if (step < Real(1e-3)) {
                    ++out.unarmedApproaches;
                    continue;
                }
                d = d * (Real(1) / step);
                const Real dl = (J.at - mouth).length();
                const Vec2 right = rightOf(d);
                const Real w = sec.carriagewayHalfWidth() + p.curbGap;
                auto tryCorner = [&](Real side, Vec2 from, Vec2& at) -> bool {
                    const Vec2 sideDir = right * side;
                    Vec2 corner = from + sideDir * w;
                    // ...out of every other street's carriageway, pushed clear (iterating: clearing one can enter
                    // another), giving up rather than walking a pole across the city
                    Real residual = 0;
                    for (int pass = 0; pass < 5; ++pass) {
                        Vec2 push(0, 0);
                        Real worst = 0;
                        forEachPieceNear(corner, kPoleSearch, [&](int oi) {
                            const StreetPiece& O = pieces[static_cast<std::size_t>(oi)];
                            if (O.city == fc.city && O.street == static_cast<int>(arm.street.index)) return;
                            const Vec2 ab = O.b - O.a;
                            const Real L2 = ab.lengthSquared();
                            const Real u = L2 < Real(1e-9) ? Real(0) : std::clamp(dot(corner - O.a, ab) / L2, Real(0), Real(1));
                            Vec2 away = corner - (O.a + ab * u);
                            const Real dist = away.length();
                            const Real need = O.half + p.curbGap;
                            if (dist >= need) return;
                            away = dist < Real(1e-6) ? normalize(Vec2(-ab.y, ab.x)) : away * (Real(1) / dist);
                            if (need - dist > worst) {
                                worst = need - dist;
                                push = away * worst;
                            }
                        });
                        residual = worst;
                        if (worst <= Real(1e-3)) break;
                        if ((corner + push - from).length() > kPoleMaxWalk) break;   // not a corner any more
                        corner = corner + push;
                        residual = 0;
                    }
                    if (residual > kPoleDropInside) return false;
                    if (walkOffDeck(corner, sideDir, kPoleMaxWalk) > Real(0)) return false;
                    at = corner;
                    return true;
                };
                // back from the mouth past the corner's kerb round (a sidewalk's width and the kerb gap), on the arm
                const Vec2 near = mouth - d * (p.sidewalkWidth + p.curbGap);
                const Vec2 far = J.at + d * dl;            // the far side, where the arm's line runs on across
                Vec2 corner;
                if (!tryCorner(Real(1), near, corner) && !tryCorner(Real(-1), near, corner) && !tryCorner(Real(1), far, corner) &&
                    !tryCorner(Real(-1), far, corner)) {
                    out.unpoledApproaches.push_back(J.at);
                    continue;
                }
                SignalSpot s;
                s.base = Vec3(corner.x, gy(corner.x, corner.y), corner.y);
                s.face = Vec3(-d.x, 0, -d.y);   // the head faces its arriving traffic
                s.arm = packFurnitureArm(fc.city, static_cast<int>(j), static_cast<int>(ai));
                s.city = fc.city;
                s.junction = static_cast<int>(j);
                s.armIndex = static_cast<int>(ai);
                out.signals.push_back(s);
            }
        }
    }

    // HOW FAR a point is from the nearest junction's centre or arm mouth, bucketed by 20 m cells
    constexpr Real kJunctionCell = 20.0;
    std::map<std::pair<int, int>, std::vector<Vec2>> junctionCells;
    for (const Vec2& j : junctionPoints)
        junctionCells[{static_cast<int>(std::floor(j.x / kJunctionCell)), static_cast<int>(std::floor(j.y / kJunctionCell))}].push_back(j);
    auto junctionDist = [&](const Vec2& v) {
        Real d = 1e30;
        const int cx = static_cast<int>(std::floor(v.x / kJunctionCell)), cz = static_cast<int>(std::floor(v.y / kJunctionCell));
        for (int dx = -2; dx <= 2; ++dx)
            for (int dz = -2; dz <= 2; ++dz) {
                const std::map<std::pair<int, int>, std::vector<Vec2>>::const_iterator it = junctionCells.find({cx + dx, cz + dz});
                if (it == junctionCells.end()) continue;
                for (const Vec2& j : it->second) d = std::min(d, (v - j).length());
            }
        return d;   // (exact within 40 m: every clearance here is under that)
    };
    // THE KERB AS DRAWN: from the centre line out past the carriageway (and any parking band the deck holds), then the
    // verge. `n` the unit normal toward the kerb.
    auto kerbPoint = [&](const StreetPiece& K, Vec2 c, Vec2 n, Real verge) {
        Real s = K.half;
        if (p.deck)
            while (s < 16.0 && p.deck->depthInside(c.x + n.x * s, c.y + n.y * s) > 0.0) s += 0.25;
        return c + n * (s + verge);
    };

    // LAMPS every `lampSpacing` metres of KERB, measured along the street; each kerb of a street with a sidewalk there,
    // sampled finely and spaced with a grid (direction-aware: the two kerbs of a street are lamps facing opposite ways and
    // never suppress each other). No lamps on freeways and ramps (their lighting is its own pass), near junctions (the
    // signal poles' corners), in any carriageway or on the drawn asphalt.
    struct PlacedLamp { Vec2 at; Vec2 dir; };
    std::unordered_map<long long, std::vector<PlacedLamp>> grid;
    const Real cell = std::max(Real(1), p.lampSpacing);
    auto tooClose = [&](Vec2 v, Vec2 d) {
        const int cx = static_cast<int>(std::floor(v.x / cell)), cz = static_cast<int>(std::floor(v.y / cell));
        for (int dz = -1; dz <= 1; ++dz)
            for (int dx = -1; dx <= 1; ++dx) {
                auto it = grid.find(cellKey(cx + dx, cz + dz));
                if (it == grid.end()) continue;
                for (const PlacedLamp& q : it->second)
                    if (dot(q.dir, d) > 0 && (q.at - v).length() < p.lampSpacing * Real(0.85)) return true;
            }
        return false;
    };
    auto pieceDir = [](const StreetPiece& K) { return normalize(K.b - K.a); };
    for (const StreetPiece& K : pieces) {
        if (K.klass == RoadClass::Freeway || K.klass == RoadClass::Ramp || !K.cars) continue;
        if (K.half * 2 > p.maxLampRoadWidth) continue;
        const Real len = (K.b - K.a).length();
        const Vec2 dir = pieceDir(K);
        const int steps = std::max(1, static_cast<int>(std::ceil(len / Real(2.5))));
        for (int side = 0; side < 2; ++side) {
            if (!K.sidewalk[side]) continue;
            const Real sgn = side == 0 ? Real(1) : Real(-1);
            const Vec2 n = rightOf(dir) * sgn, kerbDir = dir * sgn;
            for (int k = 0; k <= steps; ++k) {
                const Vec2 c = K.a + (K.b - K.a) * (static_cast<Real>(k) / steps);
                const Vec2 sp = c + n * (K.half + p.lampVerge);
                if (junctionDist(sp) < p.junctionClear) continue;
                if (insideAnyCarriageway(sp)) continue;
                if (p.deck && p.deck->depthInside(sp.x, sp.y) > 0.0) continue;
                if (kerb.depth(sp) > 0.0) continue;   // (the junction corners the deck field does not hold)
                if (tooClose(sp, kerbDir)) continue;
                const Real y = gy(sp.x, sp.y);
                out.lampBases.push_back(Vec3(sp.x, y, sp.y));
                out.lampHeads.push_back(Vec3(sp.x, y + p.lampHeight, sp.y));
                out.lampStreets.push_back(LampStreet{K.city, K.street});
                grid[cellKey(static_cast<int>(std::floor(sp.x / cell)), static_cast<int>(std::floor(sp.y / cell)))].push_back({sp, kerbDir});
            }
        }
    }
    if (!p.kerbLife) return out;

    // KERB LIFE: every city street that has a pavement (not a freeway, a ramp, the road between towns): street trees in
    // pits at their spacing, kept off the lamps, the signal poles and the junctions' sightlines; a litter bin by each
    // corner; bike racks and news boxes along the main streets. The same sampling and the same "off the asphalt" rules as
    // the lamps, each kind spaced along its own kerb (direction-aware).
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
    auto streetWithPavement = [&](const StreetPiece& K) {
        return (K.klass == RoadClass::Local || K.klass == RoadClass::Collector || K.klass == RoadClass::Arterial) && K.cars &&
               K.half * 2 <= p.maxLampRoadWidth;
    };
    auto clearOfRoad = [&](const Vec2& v, Real margin) {
        if (insideAnyCarriageway(v)) return false;
        if (!p.deck) return true;
        for (const Vec2 o : {Vec2(0, 0), Vec2(margin, 0), Vec2(-margin, 0), Vec2(0, margin), Vec2(0, -margin)})
            if (p.deck->depthInside(v.x + o.x, v.y + o.y) > 0.0) return false;
        return true;
    };
    ProcRng hRng(0x5EED7u);
    for (const StreetPiece& K : pieces) {
        if (!streetWithPavement(K)) continue;
        const Real len = (K.b - K.a).length();
        const Vec2 dir = pieceDir(K);
        const bool main = K.klass != RoadClass::Local;
        const int steps = std::max(1, static_cast<int>(std::ceil(len / Real(1.5))));
        // THIS STREET'S PLANTING: by its character, and the same all along its stated run (streetRun), else along its
        // line (its bearing to 10 degrees, its offset across to 30 m, 700 m stretches along it, its class)
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
            const Vec2 mid = (K.a + K.b) * Real(0.5);
            const long long off = static_cast<long long>(std::llround(dot(mid, Vec2(-lineDir.y, lineDir.x)) / 30.0));
            const long long along = static_cast<long long>(std::floor(dot(mid, lineDir) / 700.0));
            const uint64_t run = p.streetRun ? p.streetRun(K.city, K.street) : 0;
            const uint64_t k = mix64(run != 0
                ? 0x9E3779B97F4A7C15ull ^ (run * 0xBF58476D1CE4E5B9ull) ^ static_cast<uint64_t>(K.klass) * 0x2545F4914F6CDD1Dull
                : 0x9E3779B97F4A7C15ull ^ (static_cast<uint64_t>(ab) * 0xBF58476D1CE4E5B9ull) ^
                  (static_cast<uint64_t>(off) * 0x94D049BB133111EBull) ^ (static_cast<uint64_t>(along) * 0xD6E8FEB86659FD93ull) ^
                  static_cast<uint64_t>(K.klass) * 0x2545F4914F6CDD1Dull);
            pl = choosePlanting(p.character(mid), K.klass, (k & 0xFFFF) / 65536.0, ((k >> 16) & 0xFFFF) / 65536.0);
            ++out.planted[std::min<int>(pl.scheme, 7)];
        }
        for (int side = 0; side < 2; ++side) {
            if (!K.sidewalk[side]) continue;
            const Real sgn = side == 0 ? Real(1) : Real(-1);
            const Vec2 n = rightOf(dir) * sgn, kerbDir = dir * sgn;
            const bool plantHere = !p.character || (pl.scheme != StreetPlanting::None && (!pl.oneSide || dot(kerbDir, lineDir) > 0));
            for (int k = 0; k <= steps; ++k) {
                const Vec2 c = K.a + (K.b - K.a) * (static_cast<Real>(k) / steps);
                const Real dj = junctionDist(kerbPoint(K, c, n, p.treeVerge));
                {   // A STREET TREE
                    const Vec2 sp = kerbPoint(K, c, n, p.treeVerge);
                    if (plantHere && dj >= p.treeJunctionClear && clearOfRoad(sp, 0.4) && !near(trees, sp, kerbDir, pl.spacing * 0.92, true) &&
                        !near(lamps, sp, kerbDir, 3.2, false) && !near(poles, sp, kerbDir, 3.5, false) && !near(bins, sp, kerbDir, 2.0, false) &&
                        !near(props, sp, kerbDir, 2.5, false)) {
                        const uint32_t h = hRng.bits();
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
                        add(trees, sp, kerbDir);
                    }
                }
                {   // A LITTER BIN by the corner, just past the junction's clear zone, near the kerb
                    const Vec2 sp = kerbPoint(K, c, n, 1.0);
                    if (dj >= p.junctionClear && dj < p.junctionClear + 4.0 && clearOfRoad(sp, 0.2) && !near(bins, sp, kerbDir, 25.0, true) &&
                        !near(trees, sp, kerbDir, 1.6, false) && !near(lamps, sp, kerbDir, 1.2, false) && !near(poles, sp, kerbDir, 1.5, false)) {
                        out.props.push_back({Vec3(sp.x, gy(sp.x, sp.y), sp.y), n, KerbProp::Bin});
                        add(bins, sp, kerbDir);
                    }
                }
                if (main && dj > 25.0) {   // A BIKE RACK or NEWS BOXES along a main street, mid-block
                    const Vec2 sp = kerbPoint(K, c, n, 1.0);
                    if (clearOfRoad(sp, 0.3) && !near(props, sp, kerbDir, p.propSpacing, true) && !near(trees, sp, kerbDir, 2.6, false) &&
                        !near(lamps, sp, kerbDir, 2.0, false) && !near(poles, sp, kerbDir, 2.5, false)) {
                        const uint32_t h = hRng.bits();
                        out.props.push_back({Vec3(sp.x, gy(sp.x, sp.y), sp.y), n,
                                             static_cast<uint8_t>((h >> 12) % 3u ? KerbProp::BikeRack : KerbProp::NewsBoxes)});
                        add(props, sp, kerbDir);
                    }
                }
            }
        }
    }
    // THE DATA KINDS (street_props.lua, ADR-0354), in a pass of their own after the march above, so the planner's own
    // pieces stand where they always did: each kind at its spacing a side, clear of everything placed
    std::vector<Spaced> kinds;
    for (const StreetFurnitureParams::KerbKind& kk : p.kerbKinds) kinds.push_back(Spaced{std::max(kk.every, Real(8)), {}});
    for (std::size_t ki = 0; ki < p.kerbKinds.size(); ++ki) {
        const StreetFurnitureParams::KerbKind& kk = p.kerbKinds[ki];
        for (const StreetPiece& K : pieces) {
            if (!streetWithPavement(K) || (kk.mainOnly && K.klass == RoadClass::Local)) continue;
            const Real len = (K.b - K.a).length();
            const Vec2 dir = pieceDir(K);
            const int steps = std::max(1, static_cast<int>(std::ceil(len / Real(1.5))));
            for (int side = 0; side < 2; ++side) {
                if (!K.sidewalk[side]) continue;
                const Real sgn = side == 0 ? Real(1) : Real(-1);
                const Vec2 n = rightOf(dir) * sgn, kerbDir = dir * sgn;
                for (int k = 0; k <= steps; ++k) {
                    const Vec2 c = K.a + (K.b - K.a) * (static_cast<Real>(k) / steps);
                    const Vec2 sp = kerbPoint(K, c, n, kk.verge);
                    if (junctionDist(sp) < kk.junction || !clearOfRoad(sp, 0.25)) continue;
                    bool crowded = near(kinds[ki], sp, kerbDir, kk.every, true) || near(trees, sp, kerbDir, 1.6, false) ||
                                   near(lamps, sp, kerbDir, 1.4, false) || near(poles, sp, kerbDir, 1.8, false) ||
                                   near(bins, sp, kerbDir, 1.6, false) || near(props, sp, kerbDir, 2.0, false);
                    for (std::size_t kj = 0; kj < kinds.size() && !crowded; ++kj)
                        if (kj != ki) crowded = near(kinds[kj], sp, kerbDir, 2.0, false);
                    if (crowded) continue;
                    out.props.push_back({Vec3(sp.x, gy(sp.x, sp.y), sp.y), n, static_cast<uint8_t>(KerbProp::kFirstDataKind + ki)});
                    add(kinds[ki], sp, kerbDir);
                }
            }
        }
    }
    return out;
}

}  // namespace engine
