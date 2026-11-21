// Street furniture placement (build-time poles, sim-time phases), planned from the road model (#306: the street map it
// was planned on went). The claims under test:
//   - one signal pole per arriving arm of a lit junction, standing on its kerb corner, clear of every carriageway;
//   - no pole at a junction that is not lit;
//   - lamps, street trees and kerb props off every carriageway, on both kerbs, clear of the junctions;
//   - each street planted by its character.
// The scenes are road files built as a level's roads are (model_scene.h).
#include "test_framework.h"

#include <set>

#include "model_scene.h"
#include "../src/engine/procgen/city/street_furniture.h"

using namespace engine;

namespace {

// How far `v` is inside the nearest carriageway of the model's car streets (a street piece's travel half-width less the
// distance to its centre line; negative: clear of all by that much).
Real intoCarriageway(const RoadModel& m, Vec2 v) {
    Real worst = -1e9;
    for (const ModelStreet& S : m.streets) {
        if (!S.section.carriesCars() || S.line.count < 2) continue;
        for (uint32_t i = S.line.first; i + 1 < S.line.end(); ++i) {
            const Vec2 a = m.linePoints[i], b = m.linePoints[i + 1], ab = b - a;
            const Real l2 = ab.lengthSquared();
            const Real t = l2 > 1e-12 ? std::clamp(dot(v - a, ab) / l2, Real(0), Real(1)) : Real(0);
            worst = std::max(worst, S.section.carriagewayHalfWidth() - (a + ab * t - v).length());
        }
    }
    return worst;
}

StreetFurniturePlan plan(const citytest::ModelScene& s, StreetFurnitureParams fp = {}) {
    return planStreetFurniture({FurnitureCity{s.model.get(), 0}}, [](Real, Real) { return Real(0); }, fp);
}

}  // namespace

TEST_CASE(signal_poles_one_per_arriving_arm_of_a_lit_junction_on_its_corner) {
    // a 4-way cross of 80 m local roads, its junction lit (no control stated: a junction of four arms or more is)
    const citytest::ModelScene& s = citytest::crossScene(80);
    StreetFurnitureParams fp;
    fp.controlsStated = false;
    const StreetFurniturePlan p = plan(s, fp);
    std::printf("    [poles] %zu poles, %zu unpoled, %d unarmed\n", p.signals.size(), p.unpoledApproaches.size(), p.unarmedApproaches);
    CHECK(p.signals.size() == 4u);
    std::set<int> arms;
    for (const SignalSpot& sp : p.signals) {
        arms.insert(sp.armIndex);
        CHECK(intoCarriageway(*s.model, Vec2(sp.base.x, sp.base.z)) < -fp.curbGap + 1e-6);   // clear of every carriageway
        CHECK(Vec2(sp.base.x, sp.base.z).length() < 30.0);                                  // at the junction's corner
        CHECK(sp.arm == packFurnitureArm(0, sp.junction, sp.armIndex));
    }
    CHECK(arms.size() == 4u);   // one per arm
}

TEST_CASE(no_signal_poles_at_a_junction_that_is_not_lit) {
    const citytest::ModelScene& s = citytest::teeScene(80);
    StreetFurnitureParams fp;
    fp.controlsStated = false;   // (a T has three arms: not lit)
    CHECK(plan(s, fp).signals.empty());
}

TEST_CASE(lamps_never_stand_in_any_streets_carriageway) {
    const citytest::ModelScene& s = citytest::gridScene(600, 90, 5);
    const StreetFurniturePlan p = plan(s);
    std::printf("    [lamps] %zu lamps\n", p.lampBases.size());
    CHECK(p.lampBases.size() > 50u);
    CHECK(p.lampStreets.size() == p.lampBases.size());
    int inRoad = 0;
    for (const Vec3& b : p.lampBases) inRoad += intoCarriageway(*s.model, Vec2(b.x, b.z)) > 0.0 ? 1 : 0;
    CHECK(inRoad == 0);
}

TEST_CASE(kerb_life_lines_the_pavements_and_keeps_off_the_road) {
    RoadGraph g;
    const int w = g.addNode({-300, 0}), e = g.addNode({300, 0}), n = g.addNode({0, 300}), so = g.addNode({0, -300}), c = g.addNode({0, 0});
    g.addEdge(w, c, 12, RoadClass::Collector); g.addEdge(c, e, 12, RoadClass::Collector);
    g.addEdge(so, c, 10, RoadClass::Local); g.addEdge(c, n, 10, RoadClass::Local);
    const citytest::ModelScene& s = citytest::graphScene("furniture_kerb_life", g);
    StreetFurnitureParams fp;
    const StreetFurniturePlan p = plan(s, fp);
    int bins = 0, mid = 0;
    for (const KerbProp& kp : p.props) (kp.kind == KerbProp::Bin ? bins : mid)++;
    std::printf("    [kerb] %zu trees, %d bins, %d racks/news boxes, %zu lamps\n", p.trees.size(), bins, mid, p.lampBases.size());
    CHECK(p.trees.size() > 100);
    CHECK(bins >= 4);   // a corner bin on each approach's kerb
    CHECK(mid >= 4);
    int onRoad = 0, byLamp = 0, atCorner = 0;
    for (const StreetTree& t : p.trees) {
        const Vec2 v(t.base.x, t.base.z);
        if (intoCarriageway(*s.model, v) > -0.3) ++onRoad;
        if (v.length() < fp.treeJunctionClear - 0.5) ++atCorner;
        for (const Vec3& l : p.lampBases) if ((Vec2(l.x, l.z) - v).length() < 3.0) { ++byLamp; break; }
    }
    for (const KerbProp& kp : p.props) if (intoCarriageway(*s.model, Vec2(kp.base.x, kp.base.z)) > -0.3) ++onRoad;
    CHECK(onRoad == 0);
    CHECK(byLamp == 0);
    CHECK(atCorner == 0);
    // trees on BOTH kerbs of the collector
    int northKerb = 0, southKerb = 0;
    for (const StreetTree& t : p.trees) if (std::fabs(t.base.x) > 30) (t.base.z > 0 ? northKerb : southKerb)++;
    CHECK(northKerb > 20 && southKerb > 20);
    // off: lamps and signals only
    fp.kerbLife = false;
    const StreetFurniturePlan bare = plan(s, fp);
    CHECK(bare.trees.empty() && bare.props.empty());
    CHECK(bare.lampBases.size() == p.lampBases.size());
}

// EACH STREET ITS OWN PLANTING (Glenn: "some have trees, some don't, some have them spaced differently ... suburbs vs
// downtown, poor vs wealthy areas, newly developed areas vs old areas"): the scheme follows the street's character,
// and one street keeps one planting along its length.
TEST_CASE(streets_are_planted_by_their_character) {
    using C = StreetCharacter;
    auto many = [](C c, RoadClass k) {
        int n[8] = {0};
        Real spacing = 0, scale = 0, gaps = 0;
        int planted = 0;
        for (int i = 0; i < 400; ++i) {
            const StreetPlanting pl = choosePlanting(c, k, (i % 20 + 0.5) / 20.0, (i / 20 + 0.5) / 20.0);
            ++n[pl.scheme];
            if (pl.scheme == StreetPlanting::None) continue;
            ++planted; spacing += pl.spacing; scale += 0.5 * (pl.scaleLo + pl.scaleHi); gaps += pl.gaps;
        }
        struct R { int none, planted; Real spacing, scale, gaps; int palms; };
        return R{n[StreetPlanting::None], planted, planted ? spacing / planted : 0, planted ? scale / planted : 0,
                 planted ? gaps / planted : 0, n[StreetPlanting::Palms]};
    };
    C rich; rich.wealth = 0.85; rich.age = 0.6;
    C poor; poor.wealth = 0.15; poor.age = 0.6;
    C fresh; fresh.wealth = 0.5; fresh.age = 0.1;
    C old; old.wealth = 0.5; old.age = 0.95;
    C works; works.kind = C::Industrial;
    C coast; coast.coastal = true; coast.wealth = 0.5;
    C core; core.kind = C::Downtown;
    const auto r = many(rich, RoadClass::Local), pp = many(poor, RoadClass::Local), f = many(fresh, RoadClass::Local);
    const auto o = many(old, RoadClass::Local), i = many(works, RoadClass::Local), s = many(coast, RoadClass::Local);
    const auto d = many(core, RoadClass::Arterial);
    CHECK(r.none == 0 && pp.none > 100);            // the leafy streets are all planted; the poor ones often bare
    CHECK(r.spacing < pp.spacing - 4.0);            // ...close-set against far apart
    CHECK(pp.gaps > r.gaps + 0.25);                 // ...and the poor streets' rows are broken
    CHECK(f.scale < 0.6 && o.scale > f.scale + 0.35);   // saplings on the new estates, big old trees in the old town
    CHECK(i.none > 250);                            // the works: bare kerbs
    CHECK(s.palms > 120);                           // palms by the sea
    CHECK(d.palms > 100 && d.none > 50);            // downtown boulevards: palms, or nothing

    // ON THE MAP: a wealthy east side and a poor industrial west on one crossroads -- the east arm planted close and
    // of ONE kind all along it (both kerbs), the west arm bare or nearly
    RoadGraph g;
    const int w = g.addNode({-400, 0}), e = g.addNode({400, 0}), c = g.addNode({0, 0}), n = g.addNode({0, 300});
    g.addEdge(w, c, 10, RoadClass::Local); g.addEdge(c, e, 10, RoadClass::Local); g.addEdge(c, n, 10, RoadClass::Local);
    const citytest::ModelScene& scene = citytest::graphScene("furniture_character", g);
    StreetFurnitureParams fp;
    fp.character = [](Vec2 at) {
        C ch;
        if (at.x < -20) { ch.kind = C::Industrial; return ch; }
        ch.wealth = 0.9; ch.age = 0.6;
        return ch;
    };
    const StreetFurniturePlan p = plan(scene, fp);
    int west = 0, east = 0;
    std::set<int> eastKinds;
    for (const StreetTree& t : p.trees) {
        if (std::fabs(t.base.z) > 20) continue;   // (the north arm)
        if (t.base.x < -20) ++west;
        else if (t.base.x > 20) { ++east; eastKinds.insert(static_cast<int>(t.species)); }
    }
    std::printf("    [planting] crossroads: west (industrial) %d trees, east (well off) %d trees of %zu kind(s)\n", west, east, eastKinds.size());
    CHECK(east > 40);
    CHECK(west < east / 3);
    CHECK(eastKinds.size() == 1);
}
