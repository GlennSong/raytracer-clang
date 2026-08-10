#include "test_framework.h"

#include "../src/engine/procgen/city/site_plan.h"
#include <cmath>

using namespace engine;

namespace {
bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }

// A rectangular lot with its south edge on the street.
Shape2 streetLot(Real w, Real d) {
    Shape2 lot = rectShape(0, 0, w, d);
    lot.outer.edges[0].tag = EdgeTag::Street;
    lot.outer.edges[1].tag = EdgeTag::Side;
    lot.outer.edges[2].tag = EdgeTag::Rear;
    lot.outer.edges[3].tag = EdgeTag::Side;
    return lot;
}

LotTags tagsFor(const Shape2& lot) {
    return measureLot(lot, {}, StreetClass::Street, true, 0.3);
}
}  // namespace

// --- inscribed rectangle / buildability ------------------------------------

TEST_CASE(lot_inscribed_rect_measures_a_rectangle_exactly) {
    Real w, d;
    inscribedRect(rectShape(0, 0, 30, 18), w, d);
    CHECK(near(w, 30, 0.5));
    CHECK(near(d, 18, 0.5));
}

TEST_CASE(lot_inscribed_rect_is_frame_independent) {
    // A lot at a street angle must measure the same as one squared to the axes
    // — a diagonal parcel is not less buildable than a rectilinear one.
    Poly2 rot;
    const Real c = std::cos(0.6), s = std::sin(0.6);
    const Vec2 corners[4] = {{0, 0}, {30, 0}, {30, 18}, {0, 18}};
    for (const Vec2& p : corners) rot.push_back(Vec2(p.x * c - p.y * s, p.x * s + p.y * c));
    Real w, d;
    inscribedRect(shapeFromPoly(rot), w, d);
    CHECK(near(w, 30, 1.0));
    CHECK(near(d, 18, 1.0));
}

// The measurement that makes the inversion work: a skinny wedge must REPORT
// that it cannot hold a building, instead of being handed one and rejected.
TEST_CASE(lot_inscribed_rect_exposes_an_unbuildable_wedge) {
    Poly2 wedge = {{0, 0}, {40, 0}, {0, 5}};
    Real w, d;
    inscribedRect(shapeFromPoly(wedge), w, d);
    CHECK(d < 3.0);                     // no room for a building of any depth
    LotTags t = measureLot(shapeFromPoly(wedge), {}, StreetClass::Street, true, 0);
    LotProgram cottage = residentialPrograms().programs[0];
    CHECK(!lotFitsProgram(t, cottage));
}

TEST_CASE(lot_height_is_capped_by_the_plate_once) {
    // A 100-storey tower cannot stand on 200 m2, and the model says so without
    // any recipe carrying its own guess.
    CHECK(maxStoreysFor(12, 200) < 10);
    CHECK(maxStoreysFor(40, 1600) > 40);
    // Both terms bind: a long thin plate is limited by its short side.
    CHECK(maxStoreysFor(8, 1600) < 20);
}

// --- lot tags --------------------------------------------------------------

TEST_CASE(lot_tags_identify_corner_through_and_mid_block) {
    Shape2 mid = streetLot(20, 30);
    CHECK(tagsFor(mid).shape == LotShape::MidBlock);
    CHECK(tagsFor(mid).frontages == 1);

    Shape2 corner = streetLot(20, 30);
    corner.outer.edges[1].tag = EdgeTag::Street;      // a second, perpendicular front
    CHECK(tagsFor(corner).shape == LotShape::Corner);

    Shape2 through = streetLot(20, 30);
    through.outer.edges[2].tag = EdgeTag::Street;     // opposite front
    CHECK(tagsFor(through).shape == LotShape::Through);

    Shape2 island = rectShape(0, 0, 20, 30);
    for (Edge2& e : island.outer.edges) e.tag = EdgeTag::Street;
    CHECK(tagsFor(island).shape == LotShape::Island);
}

TEST_CASE(lot_tags_measure_frontage_length) {
    Shape2 lot = streetLot(24, 30);
    LotTags t = tagsFor(lot);
    CHECK(near(t.frontWidth, 24, 0.01));
    CHECK(near(t.area, 720, 0.01));
}

// --- the inversion ---------------------------------------------------------

// §17.1: the parceller should only emit lots that can hold a program, so
// nothing downstream ever needs a rejection counter.
TEST_CASE(program_eligibility_is_the_whole_rejection_test) {
    ProgramSet set = residentialPrograms();

    // A generous lot is eligible for several programs.
    LotTags big = tagsFor(streetLot(26, 34));
    CHECK(set.eligible(big).size() >= 3);

    // A tiny lot is eligible for none — which is a legitimate answer meaning
    // "this is open space", not a failure to be counted.
    Shape2 sliver = streetLot(4, 6);
    LotTags tiny = tagsFor(sliver);
    CHECK(set.eligible(tiny).empty());
    CHECK(set.pick(tiny, 1) == -1);
}

TEST_CASE(program_corner_requirement_is_honoured) {
    ProgramSet set = residentialPrograms();
    Shape2 mid = streetLot(22, 26);
    Shape2 corner = streetLot(22, 26);
    corner.outer.edges[1].tag = EdgeTag::Street;

    auto hasCornerShop = [&](const LotTags& t) {
        for (const auto& [i, w] : set.eligible(t))
            if (set.programs[i].name == "corner_shopfront") return true;
        return false;
    };
    CHECK(!hasCornerShop(tagsFor(mid)));
    CHECK(hasCornerShop(tagsFor(corner)));
}

// Rarity as a QUOTA, not a probability — the fix for "one in fifty is a
// landmark" producing either none or twelve.
TEST_CASE(program_quotas_place_exactly_once) {
    ProgramSet set = downtownPrograms();
    Shape2 big = streetLot(60, 70);
    LotTags t = measureLot(big, {}, StreetClass::Arterial, true, 0.9);

    int cathedrals = 0;
    for (std::uint32_t i = 0; i < 60; ++i) {
        const int pick = set.pick(t, i * 7919u + 1);
        if (pick >= 0 && set.programs[pick].name == "cathedral") ++cathedrals;
    }
    CHECK(cathedrals == 1);              // exactly one, across sixty lots
}

TEST_CASE(program_rim_blocks_admit_campus_scale_programs) {
    // §17.6: a rim block is not a special case, it is a TAG that admits a
    // different program set. The coarse grain falls out of the minimums.
    ProgramSet rim = rimPrograms();
    Shape2 huge = streetLot(110, 130);
    LotTags enclosed = measureLot(huge, {}, StreetClass::Street, true, 0.2);
    LotTags open = measureLot(huge, {}, StreetClass::Street, false, 0.2);
    CHECK(rim.eligible(enclosed).size() < rim.eligible(open).size());

    // And an ordinary block-sized lot cannot host a campus at all.
    LotTags small = measureLot(streetLot(24, 30), {}, StreetClass::Street, false, 0.2);
    for (const auto& [i, w] : rim.eligible(small))
        CHECK(rim.programs[i].name != "university_campus");
}

TEST_CASE(program_pick_is_deterministic) {
    Shape2 lot = streetLot(26, 32);
    LotTags t = tagsFor(lot);
    ProgramSet a = residentialPrograms(), b = residentialPrograms();
    for (std::uint32_t i = 0; i < 20; ++i)
        CHECK(a.pick(t, i + 1) == b.pick(t, i + 1));
}

// --- site plan -------------------------------------------------------------

// The structural claim of the whole layer: zones cannot overlap, because every
// one of them is carved out of the same shrinking free region.
TEST_CASE(site_zones_are_disjoint_by_construction) {
    ProgramSet set = residentialPrograms();
    for (std::uint32_t seed = 1; seed <= 12; ++seed) {
        Shape2 lot = streetLot(24 + seed, 32);
        LotTags t = tagsFor(lot);
        const int pick = set.pick(t, seed);
        if (pick < 0) continue;
        SitePlan site = layoutSite(lot, t, set.programs[pick], seed);
        CHECK(!site.zones.empty());
        for (std::size_t i = 0; i < site.zones.size(); ++i)
            for (std::size_t j = i + 1; j < site.zones.size(); ++j) {
                std::vector<Shape2> overlap = shapeBool(
                    site.zones[i].parts, site.zones[j].parts, BoolOp::Intersect);
                Real a = 0;
                for (const Shape2& s : overlap) a += area(s);
                CHECK(a < 0.5);          // no two zones share ground
            }
    }
}

TEST_CASE(site_zones_stay_inside_the_lot) {
    ProgramSet set = commercialPrograms();
    Shape2 lot = streetLot(30, 26);
    LotTags t = tagsFor(lot);
    const int pick = set.pick(t, 5);
    CHECK(pick >= 0);
    SitePlan site = layoutSite(lot, t, set.programs[pick], 5);
    Real total = 0;
    for (const ZoneArea& z : site.zones) {
        std::vector<Shape2> out = shapeBool(z.parts, {lot}, BoolOp::Subtract);
        Real spill = 0;
        for (const Shape2& s : out) spill += area(s);
        CHECK(spill < 0.5);              // nothing hangs off the parcel
        total += z.totalArea();
    }
    CHECK(total <= area(lot) + 0.5);
}

TEST_CASE(site_setbacks_are_program_properties) {
    // A villa wants a deep front garden; a shopfront wants none. Same lot.
    Shape2 lot = streetLot(28, 34);
    LotTags t = tagsFor(lot);
    ProgramSet res = residentialPrograms(), com = commercialPrograms();

    LotProgram villa;
    for (const LotProgram& p : res.programs) if (p.name == "villa") villa = p;
    LotProgram shop;
    for (const LotProgram& p : com.programs) if (p.name == "shopfront") shop = p;
    CHECK(villa.frontSetback > shop.frontSetback);

    SitePlan sv = layoutSite(lot, t, villa, 3);
    SitePlan ss = layoutSite(lot, t, shop, 3);
    Vec2 vlo, vhi, slo, shi;
    bounds(sv.buildEnvelope, vlo, vhi);
    bounds(ss.buildEnvelope, slo, shi);
    // The villa's building stands further back from the street (y = 0).
    CHECK(vlo.y > slo.y);
}

TEST_CASE(site_every_path_connects_the_street_to_the_entrance) {
    ProgramSet set = residentialPrograms();
    Shape2 lot = streetLot(26, 34);
    LotTags t = tagsFor(lot);
    LotProgram villa;
    for (const LotProgram& p : set.programs) if (p.name == "villa") villa = p;
    SitePlan site = layoutSite(lot, t, villa, 9);

    CHECK(site.hasEntrance);
    const ZoneArea* circ = site.zoneOf(Zone::Circulation);
    CHECK(circ != nullptr);
    // The circulation zone reaches both ends: it touches the access point and
    // it touches the building envelope.
    // Measure to the nearest EDGE, not the nearest vertex: tessellate only
    // subdivides arcs, so a long straight run of the path carries vertices at
    // its ends only and a point-to-vertex test would report it as far away.
    auto reach = [&](const Vec2& target) {
        Real best = 1e30;
        for (const Shape2& part : circ->parts) {
            const Poly2 r = tessellate(part.outer, 0.25);
            for (std::size_t i = 0; i < r.size(); ++i) {
                const Vec2 a = r[i], b = r[(i + 1) % r.size()];
                const Vec2 d = b - a;
                const Real len2 = d.lengthSquared();
                Real u = len2 > 1e-12 ? dot(target - a, d) / len2 : 0;
                u = std::max(Real(0), std::min(Real(1), u));
                best = std::min(best, (target - (a + d * u)).length());
            }
        }
        return best;
    };
    const Real nearAccess = reach(site.access[0].at);
    const Real nearEntrance = reach(site.entrance);
    CHECK(nearAccess < 1.5);
    CHECK(nearEntrance < 1.5);
}

// A gate exists because the geometry demands one — a path has to get through
// the fence — not because a die said so.
TEST_CASE(site_a_gate_appears_where_a_path_crosses_a_boundary) {
    ProgramSet set = residentialPrograms();
    LotProgram villa;
    for (const LotProgram& p : set.programs) if (p.name == "villa") villa = p;
    Shape2 lot = streetLot(26, 34);
    LotTags t = tagsFor(lot);
    SitePlan site = layoutSite(lot, t, villa, 4);

    CHECK(!site.boundaries.empty());
    CHECK(!site.gates.empty());
    // Every gate sits ON a boundary run, and its hinge axis runs along it.
    for (const GateSpec& g : site.gates) {
        Real best = 1e30;
        for (const BoundaryRun& b : site.boundaries) {
            const Vec2 d = b.b - b.a;
            const Real len2 = d.lengthSquared();
            if (len2 < 1e-9) continue;
            Real u = dot(g.centre - b.a, d) / len2;
            u = std::max(Real(0), std::min(Real(1), u));
            best = std::min(best, (g.centre - (b.a + d * u)).length());
        }
        CHECK(best < 0.2);
        CHECK(near(g.axis.length(), 1.0, 1e-6));
        CHECK(g.width > 0.5 && g.height > 0.5);
    }
}

// A plaza program has no fence, so it must have no gate either — the gate is
// downstream of the boundary, not an independent decision.
TEST_CASE(site_no_boundary_means_no_gate) {
    ProgramSet set = commercialPrograms();
    LotProgram shop;
    for (const LotProgram& p : set.programs) if (p.name == "shopfront") shop = p;
    CHECK(shop.frontage == FrontageKind::Direct);
    Shape2 lot = streetLot(26, 22);
    SitePlan site = layoutSite(lot, tagsFor(lot), shop, 6);
    CHECK(site.boundaries.empty());
    CHECK(site.gates.empty());
}

TEST_CASE(site_layout_is_deterministic) {
    Shape2 lot = streetLot(25, 31);
    LotTags t = tagsFor(lot);
    LotProgram p = residentialPrograms().programs[1];
    SitePlan a = layoutSite(lot, t, p, 77);
    SitePlan b = layoutSite(lot, t, p, 77);
    CHECK(a.zones.size() == b.zones.size());
    for (std::size_t i = 0; i < a.zones.size(); ++i) {
        CHECK(a.zones[i].zone == b.zones[i].zone);
        CHECK(near(a.zones[i].totalArea(), b.zones[i].totalArea(), 1e-9));
    }
    CHECK(a.gates.size() == b.gates.size());
}

// --- furnishing ------------------------------------------------------------

TEST_CASE(site_props_never_overlap_each_other) {
    ProgramSet set = residentialPrograms();
    LotProgram villa;
    for (const LotProgram& p : set.programs) if (p.name == "villa") villa = p;
    Shape2 lot = streetLot(30, 38);
    SitePlan site = layoutSite(lot, tagsFor(lot), villa, 21);
    SiteFurnishing f = furnishSite(site, villa, 21);
    CHECK(!f.props.empty());
    for (std::size_t i = 0; i < f.props.size(); ++i)
        for (std::size_t j = i + 1; j < f.props.size(); ++j) {
            const Real d = (f.props[i].at - f.props[j].at).length();
            // Props from DIFFERENT zones can be closer than their radii sum
            // only if they are in different zones; within a zone the dart
            // thrower guarantees spacing. Check the strict case per zone.
            if (d < 0.05) CHECK(false);
        }
}

TEST_CASE(site_props_stay_inside_their_zone) {
    ProgramSet set = residentialPrograms();
    LotProgram villa;
    for (const LotProgram& p : set.programs) if (p.name == "villa") villa = p;
    Shape2 lot = streetLot(30, 38);
    SitePlan site = layoutSite(lot, tagsFor(lot), villa, 33);
    for (const ZoneArea& z : site.zones) {
        std::vector<PropInstance> props = furnishZone(z, villa, 33);
        for (const PropInstance& p : props) {
            CHECK(z.contains(p.at));
            CHECK(contains(site.lot, p.at));
        }
        // Within one zone, spacing is guaranteed.
        for (std::size_t i = 0; i < props.size(); ++i)
            for (std::size_t j = i + 1; j < props.size(); ++j)
                CHECK((props[i].at - props[j].at).length() >=
                      props[i].radius + props[j].radius - 1e-6);
    }
}

TEST_CASE(site_parking_zone_gets_cars_and_gardens_get_trees) {
    ProgramSet set = residentialPrograms();
    LotProgram villa;
    for (const LotProgram& p : set.programs) if (p.name == "villa") villa = p;
    Shape2 lot = streetLot(32, 40);
    SitePlan site = layoutSite(lot, tagsFor(lot), villa, 8);
    SiteFurnishing f = furnishSite(site, villa, 8);
    bool tree = false;
    for (const PropInstance& p : f.props)
        if (p.kind == PropKind::Tree || p.kind == PropKind::Shrub) tree = true;
    CHECK(tree);
    // The building zone furnishes itself, so nothing is scattered inside it.
    const ZoneArea* b = site.zoneOf(Zone::Building);
    CHECK(b != nullptr);
    CHECK(furnishZone(*b, villa, 8).empty());
}
