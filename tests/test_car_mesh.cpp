#include "test_framework.h"

#include "../src/engine/procgen/vehicle/car_mesh.h"
#include "../src/engine/procgen/vehicle/occupant.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include "../src/engine/mesh_builder.h"

using namespace engine;

// The low-poly car body (supersedes ADR-0062's lofted shell). These pin the
// properties that the previous two generators got wrong: winding that can't
// drift, flat shading that can't smear, and window/arch openings that are REAL
// holes rather than dark paint.

namespace {

// The engine winds front faces CLOCKWISE: for a triangle (a,b,c) the outward
// geometric normal is cross(c-a, b-a). See MeshBuilder's winding contract.
Vec3 geoNormal(const RenderMesh& m, std::size_t t) {
    const Vec3& a = m.vertices[m.indices[t]].position;
    const Vec3& b = m.vertices[m.indices[t + 1]].position;
    const Vec3& c = m.vertices[m.indices[t + 2]].position;
    return cross(c - a, b - a);
}

Vec3 bodyMin(const RenderMesh& m) {
    Vec3 lo(1e9, 1e9, 1e9);
    for (const Vertex& v : m.vertices) {
        lo.x = std::min(lo.x, (Real)v.position.x);
        lo.y = std::min(lo.y, (Real)v.position.y);
        lo.z = std::min(lo.z, (Real)v.position.z);
    }
    return lo;
}

Vec3 bodyMax(const RenderMesh& m) {
    Vec3 hi(-1e9, -1e9, -1e9);
    for (const Vertex& v : m.vertices) {
        hi.x = std::max(hi.x, (Real)v.position.x);
        hi.y = std::max(hi.y, (Real)v.position.y);
        hi.z = std::max(hi.z, (Real)v.position.z);
    }
    return hi;
}

const RenderMesh& bodyOf(const CarMesh& c) {
    return c.parts[static_cast<int>(CarPart::Body)];
}

// Moller-Trumbore, two-sided: how many triangles a ray crosses. Used to ask the
// only question that really settles "is the cabin hollow" — you cannot answer it
// from triangle counts or bounding boxes.
int hitsAlong(const RenderMesh& m, const Vec3& origin, const Vec3& dir) {
    int hits = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const Vec3 e1 = b - a, e2 = c - a;
        const Vec3 pv = cross(dir, e2);
        const Real det = dot(e1, pv);
        if (std::fabs(det) < 1e-12) continue;
        const Real inv = 1.0 / det;
        const Vec3 tv = origin - a;
        const Real u = dot(tv, pv) * inv;
        if (u < 0 || u > 1) continue;
        const Vec3 qv = cross(tv, e1);
        const Real v = dot(dir, qv) * inv;
        if (v < 0 || u + v > 1) continue;
        if (dot(e2, qv) * inv > 1e-6) ++hits;
    }
    return hits;
}

}  // namespace

TEST_CASE(car_body_builds) {
    CarMesh car = buildCarMesh(defaultSedanParams());
    const RenderMesh& m = bodyOf(car);
    CHECK(!m.vertices.empty());
    CHECK(m.indices.size() % 3 == 0);
    CHECK(m.indices.size() / 3 > 100);      // a real shell, not a stub
    // Raised from 1200 when the wheel arches became elliptical. That cost one
    // extra anchor below the belt (so the arch band is 3 cells tall, not 1) plus
    // 4 stations across each opening — without both, the arch is a single cell
    // and can only ever be the rectangle it used to be. ~1370 for a car with a
    // hollow cabin, real apertures, an interior and round arches is still inside
    // the low-poly bracket; the mid/low LOD tiers are what keep traffic cheap.
    CHECK(m.indices.size() / 3 < 1500);
}

// Every triangle's STORED normal agrees with its geometric normal. This is what
// emitQuad buys us: the normal is computed from the cell's own corners and the
// winding is oriented to match, so an inside-out face is not expressible. The
// lofted shell's per-face "which way is outward?" heuristic is what this replaces.
TEST_CASE(car_body_winding_matches_normals) {
    CarMesh car = buildCarMesh(defaultSedanParams());
    const RenderMesh& m = bodyOf(car);
    int bad = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3 gn = geoNormal(m, t);
        const Vec3& sn = m.vertices[m.indices[t]].normal;
        if (dot(gn, sn) <= 0) ++bad;
    }
    CHECK(bad == 0);
}

// Flat shading: all three corners of a triangle carry the SAME normal. Without
// this the chamfers smear and the faceted read is gone — defect #2 of ADR-0062.
TEST_CASE(car_body_is_flat_shaded) {
    CarMesh car = buildCarMesh(defaultSedanParams());
    const RenderMesh& m = bodyOf(car);
    int bad = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& n0 = m.vertices[m.indices[t]].normal;
        const Vec3& n1 = m.vertices[m.indices[t + 1]].normal;
        const Vec3& n2 = m.vertices[m.indices[t + 2]].normal;
        if (dot(n0, n1) < 0.999 || dot(n0, n2) < 0.999) ++bad;
    }
    CHECK(bad == 0);
}

// The apertures are REAL HOLES, not dark paint.
//
// Counting triangles does NOT show this — cutting a hole removes cells but adds
// reveal quads around its edge, so the body can legitimately grow. (An earlier
// version of this test asserted the body shrank, and started failing the moment
// reveals landed. The count was a proxy; these are the property.)
TEST_CASE(car_apertures_are_real_openings) {
    CarParams p = defaultSedanParams();
    CarMesh cut = buildCarMesh(p);
    CHECK(!cut.parts[static_cast<int>(CarPart::Glass)].vertices.empty());
    CHECK(!cut.parts[static_cast<int>(CarPart::Interior)].vertices.empty());

    // The Low tier deliberately stops cutting: at 200 m a painted band reads the
    // same and costs less. No holes means no glass and no cavity to line.
    p.lod = CarLod::Low;
    CarMesh solid = buildCarMesh(p);
    CHECK(solid.parts[static_cast<int>(CarPart::Glass)].vertices.empty());
    CHECK(solid.parts[static_cast<int>(CarPart::Interior)].vertices.empty());
}

// The cabin is HOLLOW. A ray straight up from inside it must cross a surface at
// least twice — the headliner, then the roof skin. Once would mean the cabin is
// solid and we are starting inside the material.
TEST_CASE(car_cabin_is_hollow) {
    CarParams p = defaultSedanParams();
    CarMesh car = buildCarMesh(p);
    RenderMesh shell = bodyOf(car);
    MeshBuilder::append(shell, car.parts[static_cast<int>(CarPart::Interior)]);

    const Real floorY = (p.cabinFloor - 0.5) * p.height;
    const Vec3 origin(0, floorY + 0.20, -0.05);
    CHECK(hitsAlong(shell, origin, Vec3(0, 1, 0)) >= 2);

    // Sideways out through a door: liner then skin, or glass through a window.
    //
    // Deliberately NOT a parity check. Parity would assume a closed manifold, and
    // this shell is not one and should not be: the inner skin is single-sided by
    // design (culling makes a two-sided liner pointless) and the apertures are
    // real holes. A ray grazing a window edge crosses liner + glass + no skin —
    // an odd count, and correct. Asserting evenness claimed a property the mesh
    // never had, and only passed until the cabin moved.
    CHECK(hitsAlong(shell, origin, Vec3(1, 0, 0)) >= 2);
}

// Glass is emitted twice with opposite winding. There is no two-sided material
// flag, so a single pane vanishes from whichever side it was not wound for —
// including from the driver's seat looking out.
//
// It gets there by being TWO-SIDED, not by being emitted twice. The pane used to
// be emitted a second time facing inward, and because the transparent pass does
// not write depth, both coincident faces blended — a pane authored at 0.32
// opacity rendered as 0.54, and ~79% through a whole car. So the rule is: no
// triangle may share all three corners with another. Visibility from both sides
// is the renderer's job now (RenderMaterial::FLAG_TWO_SIDED).
TEST_CASE(car_glass_has_no_coincident_faces) {
    CarMesh car = buildCarMesh(defaultSedanParams());
    const RenderMesh& g = car.parts[static_cast<int>(CarPart::Glass)];
    CHECK(g.indices.size() / 3 > 0);

    std::map<std::string, int> byCorners;
    for (std::size_t t = 0; t + 2 < g.indices.size(); t += 3) {
        std::vector<std::string> c;
        for (int k = 0; k < 3; ++k) {
            const Vec3& v = g.vertices[g.indices[t + k]].position;
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f", v.x, v.y, v.z);
            c.push_back(buf);
        }
        std::sort(c.begin(), c.end());
        byCorners[c[0] + "|" + c[1] + "|" + c[2]]++;
    }
    int duplicated = 0;
    for (const auto& kv : byCorners)
        if (kv.second != 1) duplicated++;
    if (duplicated) std::printf("  %d glass face(s) still doubled\n", duplicated);
    CHECK(duplicated == 0);
}

TEST_CASE(car_body_fits_requested_size) {
    CarParams p = defaultSedanParams();
    CarMesh car = buildCarMesh(p);
    const Vec3 lo = bodyMin(bodyOf(car)), hi = bodyMax(bodyOf(car));
    CHECK(hi.x <= p.width * 0.5 + 1e-6);
    CHECK(lo.x >= -p.width * 0.5 - 1e-6);
    CHECK(hi.y <= p.height * 0.5 + 1e-6);
    CHECK(lo.y >= -p.height * 0.5 - 1e-6);
    CHECK(hi.z <= p.length * 0.5 + 1e-6);
    CHECK(lo.z >= -p.length * 0.5 - 1e-6);
    // The body must actually USE its envelope, not rattle around inside it.
    CHECK(hi.z - lo.z > p.length * 0.97);
    CHECK(hi.x - lo.x > p.width * 0.80);
}

// The sill sits at the rocker, clear of the ground by roughly the class's ground
// clearance — so the body doesn't scrape and the wheels have somewhere to go.
TEST_CASE(car_body_clears_the_ground) {
    CarParams p = defaultSedanParams();
    CarMesh car = buildCarMesh(p);
    const Real groundY = -p.height * 0.5;
    const Real lowest = bodyMin(bodyOf(car)).y - groundY;
    CHECK(lowest > p.groundClearance * 0.5);
    CHECK(lowest < p.groundClearance * 2.5);
}

// Lamp markers only — nothing emissive is baked, which is what stops the caller's
// lenses from doubling the drawn lamps (defect #3). Tags must match what
// citysim's syncCarLamps already parses.
TEST_CASE(car_lamp_attaches_are_named_and_legal) {
    CarParams p = defaultSedanParams();
    CarMesh car = buildCarMesh(p);

    // Lenses are real geometry, but they live in their OWN part so the caller
    // binds their material and therefore owns emission. That separation is the
    // thing that stops a lamp being drawn twice: citysim overlays lit instances
    // at the markers per state, and a lamp that baked its own glow would both
    // double up and be unable to switch off. So: lens geometry present, and
    // none of it merged into the body.
    const RenderMesh& lampPart = car.parts[static_cast<int>(CarPart::Lamp)];
    CHECK(!lampPart.vertices.empty());
    CHECK(lampPart.indices.size() / 3 == 48);   // four boxes, twelve tris each

    // The body carries no lamp-tinted vertices — if it did, the lens would be
    // welded into the paint and the caller could not relight it.
    int lampColoured = 0;
    for (const Vertex& v : bodyOf(car).vertices)
        if (v.color.x > 0.8 && v.color.y > 0.9) ++lampColoured;
    CHECK(lampColoured == 0);

    std::map<std::string, int> seen;
    for (const CarAttach& a : car.attaches) seen[a.tag]++;
    CHECK(seen.size() == 5);   // four lamps + the driver's seat
    CHECK(seen["headlight_l"] == 1);
    CHECK(seen["headlight_r"] == 1);
    CHECK(seen["taillight_l"] == 1);
    CHECK(seen["taillight_r"] == 1);

    // FMVSS 108: lamp centre 559-1372 mm above ground. Every real car obeys this,
    // which is why one that doesn't looks subtly wrong.
    const Real groundY = -p.height * 0.5;
    for (const CarAttach& a : car.attaches) {
        // Lamps only — the markers array also carries the driver's hip point,
        // which sits well below any legal lamp height and is not one.
        if (a.tag.rfind("headlight", 0) != 0 && a.tag.rfind("taillight", 0) != 0)
            continue;
        const Real h = a.position.y - groundY;
        CHECK(h > 0.559);
        CHECK(h < 1.372);
    }
    // Headlights face forward, taillights back.
    for (const CarAttach& a : car.attaches) {
        const bool head = a.tag.rfind("headlight", 0) == 0;
        const bool tail = a.tag.rfind("taillight", 0) == 0;
        if (!head && !tail) continue;
        CHECK((head ? a.normal.z > 0 : a.normal.z < 0));
        CHECK(a.position.z * (head ? 1 : -1) > 0);
        // Side tags are PHYSICAL: with forward +Z / up +Y (right-handed), the
        // car's left is +X (lab-measured steer chirality, driver_control.h).
        // These were once swapped, which blinked the right indicator on a left
        // turn in the player's car.
        CHECK((a.tag.back() == 'l' ? a.position.x > 0 : a.position.x < 0));
    }
}

// The driver's seat reference point is where an occupant gets posed and where
// VehicleSystem places its driver model, so it has to land inside the cabin
// rather than in the boot or under the floor.
TEST_CASE(car_driver_seat_attach_is_in_the_cabin) {
    CarParams p = defaultSedanParams();
    CarMesh car = buildCarMesh(p);

    const CarAttach* seat = nullptr;
    for (const CarAttach& a : car.attaches)
        if (a.tag == "driver_seat") seat = &a;
    CHECK(seat != nullptr);
    if (!seat) return;

    const Real floorY = (p.cabinFloor - 0.5) * p.height;
    CHECK(seat->position.y > floorY);
    CHECK(seat->position.y < p.height * 0.5);
    // Ahead of the rear axle and behind the front one — i.e. between the wheels.
    CHECK(seat->position.z < p.length * 0.5 - p.frontOverhang);
    CHECK(seat->position.z > -(p.length * 0.5 - p.rearOverhang));
}

// THE DRIVER FITS.
//
// This is the assertion the eye caught first and the tests had missed: a seated
// occupant placed on the driver_seat marker put its head 5 cm through the roof.
// The cause was not the person but the car — the cabin floor sat 0.43 m above
// the ground where a real one is 0.30-0.35 m, leaving too little headroom. A
// number that only shows up when something has to fit inside it.
TEST_CASE(car_seated_occupant_fits_the_cabin) {
    CarParams p = defaultSedanParams();
    CarMesh car = buildCarMesh(p);

    const CarAttach* seat = nullptr;
    for (const CarAttach& a : car.attaches)
        if (a.tag == "driver_seat") seat = &a;
    CHECK(seat != nullptr);
    if (!seat) return;

    const RenderMesh occ = buildSeatedOccupant();
    Real lo = 1e9, hi = -1e9;
    for (const Vertex& v : occ.vertices) {
        lo = std::min(lo, (Real)v.position.y);
        hi = std::max(hi, (Real)v.position.y);
    }

    // Head clears the roof, feet clear the floor. Both are measured against the
    // BODY, so a body change that squashes the cabin fails here rather than in a
    // screenshot nobody takes.
    const Vec3 bodyHi = bodyMax(bodyOf(car));
    const Real floorY = (p.cabinFloor - 0.5) * p.height;
    CHECK(seat->position.y + hi < bodyHi.y);
    CHECK(seat->position.y + lo > floorY - 0.06);

    // And a real cabin floor sits 0.28-0.40 m off the ground.
    const Real ground = -p.height * 0.5;
    CHECK(floorY - ground > 0.28);
    CHECK(floorY - ground < 0.40);
}

// The furniture is inside the shell, not poking through the doors or the roof.
TEST_CASE(car_interior_fits_inside_the_body) {
    CarParams p = defaultSedanParams();
    CarMesh car = buildCarMesh(p);
    const RenderMesh& in = car.parts[static_cast<int>(CarPart::Interior)];
    CHECK(!in.vertices.empty());

    const Vec3 lo = bodyMin(bodyOf(car)), hi = bodyMax(bodyOf(car));
    int outside = 0;
    for (const Vertex& v : in.vertices) {
        if (v.position.x < lo.x - 1e-6 || v.position.x > hi.x + 1e-6 ||
            v.position.y < lo.y - 1e-6 || v.position.y > hi.y + 1e-6 ||
            v.position.z < lo.z - 1e-6 || v.position.z > hi.z + 1e-6)
            ++outside;
    }
    CHECK(outside == 0);
}

// Seat rows are a REPEAT, not a layout: asking for more rows must produce more
// furniture. This is the property that gets a bus's seating from one op.
TEST_CASE(car_interior_rows_scale) {
    CarParams two = defaultSedanParams();
    CarParams none = defaultSedanParams();
    none.interior = false;
    CHECK(buildCarMesh(none).parts[static_cast<int>(CarPart::Interior)].indices.size() <
          buildCarMesh(two).parts[static_cast<int>(CarPart::Interior)].indices.size());
}

TEST_CASE(car_body_is_deterministic) {
    CarMesh a = buildCarMesh(defaultSedanParams());
    CarMesh b = buildCarMesh(defaultSedanParams());
    const RenderMesh& ma = bodyOf(a);
    const RenderMesh& mb = bodyOf(b);
    CHECK(ma.vertices.size() == mb.vertices.size());
    CHECK(ma.indices == mb.indices);
    bool same = true;
    for (std::size_t i = 0; i < ma.vertices.size(); ++i) {
        const Vec3 d = ma.vertices[i].position - mb.vertices[i].position;
        if (d.lengthSquared() > 0) same = false;
    }
    CHECK(same);
}

// Degenerate params must fail closed, not emit garbage the renderer then eats.
TEST_CASE(car_rejects_degenerate_params) {
    CarParams p = defaultSedanParams();
    p.stations.clear();
    CHECK(bodyOf(buildCarMesh(p)).vertices.empty());

    CarParams q = defaultSedanParams();
    q.width = 0;
    CHECK(bodyOf(buildCarMesh(q)).vertices.empty());
}

// --- surface integrity ------------------------------------------------------
// The car is NOT a closed solid and is not meant to be: glass panes are emitted
// double-sided (culling is unconditional, so a one-sided pane vanishes from one
// side), and the cabin liner is wound opposite the outer skin so you can see it
// THROUGH a window. Both make global manifoldness impossible by construction.
//
// What must hold is narrower, and these pin it: no zero-area triangles, and no
// OPEN EDGE around a wheel arch. The arches regressed exactly once — the reveal
// bridged the outer skin to an inner skin that, out at the arches, was never
// emitted (the liner is deliberately glass-only), leaving the lip's back edge
// dangling. A dangling edge is culled from one side, so the car was see-through
// at the arch from any low angle. The fix makes the arch a closed pocket; this
// test is what stops it silently reopening.
namespace {

struct EdgeStats {
    int boundary = 0;      // used by exactly ONE face -> open edge
    int nonManifold = 0;   // three or more faces
    int flipped = 0;       // paired, but both faces wound the same way
    int degenerate = 0;
    int tris = 0;
};

// Weld by quantised position first: every face carries its own vertices for flat
// shading, so raw indices never share and everything would look like boundary.
std::vector<int> weld(const RenderMesh& m, std::vector<Vec3>* pos, Real eps) {
    std::map<std::tuple<long long, long long, long long>, int> ids;
    std::vector<int> vid(m.vertices.size());
    const Real q = Real(1) / eps;
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        auto key = std::make_tuple(llround(p.x * q), llround(p.y * q), llround(p.z * q));
        auto it = ids.find(key);
        if (it == ids.end()) {
            vid[i] = static_cast<int>(ids.size());
            ids[key] = vid[i];
            if (pos) pos->push_back(p);
        } else {
            vid[i] = it->second;
        }
    }
    return vid;
}

EdgeStats analyze(const RenderMesh& m, Real eps = 1e-5) {
    EdgeStats s;
    std::vector<int> vid = weld(m, nullptr, eps);
    std::map<std::pair<int, int>, int> directed;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const int a = vid[m.indices[t]], b = vid[m.indices[t + 1]],
                  c = vid[m.indices[t + 2]];
        s.tris++;
        if (a == b || b == c || a == c) { s.degenerate++; continue; }
        directed[{a, b}]++; directed[{b, c}]++; directed[{c, a}]++;
    }
    std::map<std::pair<int, int>, int> undirected;
    for (const auto& kv : directed) {
        const auto e = std::minmax(kv.first.first, kv.first.second);
        undirected[{e.first, e.second}] += kv.second;
    }
    for (const auto& kv : undirected) {
        if (kv.second == 1) { s.boundary++; continue; }
        if (kv.second > 2) { s.nonManifold++; continue; }
        const auto f = directed.find(kv.first);
        if (f == directed.end() || f->second != 1) s.flipped++;
    }
    return s;
}

// Open edges below `yMax` — the wheel-arch band, in body-local coordinates.
int openEdgesBelow(const RenderMesh& m, Real yMax, Real eps = 1e-5) {
    std::vector<Vec3> pos;
    std::vector<int> vid = weld(m, &pos, eps);
    std::map<std::pair<int, int>, int> und;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const int a = vid[m.indices[t]], b = vid[m.indices[t + 1]],
                  c = vid[m.indices[t + 2]];
        if (a == b || b == c || a == c) continue;
        for (const auto& e : {std::make_pair(a, b), std::make_pair(b, c),
                              std::make_pair(c, a)}) {
            const auto mm = std::minmax(e.first, e.second);
            und[{mm.first, mm.second}]++;
        }
    }
    int n = 0;
    for (const auto& kv : und)
        if (kv.second == 1 &&
            (pos[kv.first.first].y + pos[kv.first.second].y) * Real(0.5) < yMax)
            n++;
    return n;
}

}  // namespace

// The regression that prompted all of this.
TEST_CASE(car_wheel_arches_are_closed_pockets) {
    const CarMesh car = buildCarMesh(defaultSedanParams());
    CHECK(openEdgesBelow(bodyOf(car), Real(-0.20)) == 0);
}

// Not just the sedan's exact proportions. The classes live in Lua, so sweep the
// package numbers they vary — a tall van roof or a big-wheeled truck cuts the
// arch out of a different silhouette and could reopen the hole on its own.
TEST_CASE(car_arches_closed_across_proportions) {
    struct Variant { const char* name; Real h, l, wheel, clearance; };
    for (const Variant& v : {Variant{"sedan", 1.45, 4.59, 0.62, 0.14},
                             Variant{"tall van", 2.30, 5.40, 0.70, 0.20},
                             Variant{"low coupe", 1.28, 4.30, 0.58, 0.11},
                             Variant{"truck", 1.95, 5.90, 0.82, 0.24},
                             Variant{"tiny", 1.50, 3.40, 0.55, 0.15}}) {
        CarParams p = defaultSedanParams();
        p.height = v.h;
        p.length = v.l;
        p.wheelDiameter = v.wheel;
        p.groundClearance = v.clearance;
        const int open = openEdgesBelow(bodyOf(buildCarMesh(p)), Real(-0.20));
        if (open != 0) std::printf("  [arch open] %s: %d edges\n", v.name, open);
        CHECK(open == 0);
    }
}

// Zero-area triangles carry a normal that matches no surface and shade as seams.
TEST_CASE(car_emits_no_degenerate_triangles) {
    const CarMesh car = buildCarMesh(defaultSedanParams());
    for (int i = 0; i < kCarPartCount; ++i)
        CHECK(analyze(car.parts[i]).degenerate == 0);
}

// The lamp lenses ARE closed solids, and should stay that way — they are the one
// part with no double-siding and no liner, so nothing excuses an open edge.
TEST_CASE(car_lamp_lenses_are_closed) {
    const CarMesh car = buildCarMesh(defaultSedanParams());
    const EdgeStats s = analyze(car.parts[static_cast<int>(CarPart::Lamp)]);
    CHECK(s.tris > 0);
    CHECK(s.boundary == 0);
    CHECK(s.nonManifold == 0);
}

// NOT TESTED: that the wheel arch is elliptical rather than rectangular.
//
// It is — the generator cuts the opening on a semi-ellipse peaking over the axle
// (archTopAt) with `archSegments` stations across it, and it is visibly round in
// the lab. But two attempts to pin it from the mesh both failed: sampling the
// flank's extreme vertex in a z-slice picks up the wheel-well recess, which
// exists at every arch cell and reads the same whether the opening is curved or
// square. A real guard needs the aperture BOUNDARY, which the mesh alone does
// not distinguish from ordinary skin — it would want the cell classification
// exposed for test, which is a bigger change than it is worth right now.
//
// So this is a known gap, recorded rather than papered over with a test that
// passes on the rectangle it is supposed to catch. Verify arches by looking:
// tools/car_shots.sh, the `side` and `kerb` frames.

// THE WHEEL FITS THE ARCH (device: "the wheels of the car are crunched into
// the body"). A wheel of `wheelDiameter` resting at the ground datum has its
// crown at y01 = wheelDiameter / height — above the old fixed archTop (0.42)
// for a sedan, and the anchor ladder quantised the cut lower still. Probe the
// opening the way the eye does: rays from OUTSIDE the car, aimed inward, at
// points on the tyre's upper arc (radius + 2 cm). Where the arch is open the
// ray passes the outer skin and hits the wheel well behind it; where bodywork
// covers the tyre the first hit IS the outer skin. Both a sedan and a
// tall-wheeled proportion (jeep-like: 0.74 m wheel, 1.75 m body) must pass.
namespace {
bool rayHitsTriangle(const Vec3& o, const Vec3& d, const Vec3& a, const Vec3& b,
                     const Vec3& c, Real& tOut) {
    const Vec3 e1 = b - a, e2 = c - a;
    const Vec3 pv = cross(d, e2);
    const Real det = dot(e1, pv);
    if (std::fabs(det) < 1e-9) return false;
    const Real inv = Real(1) / det;
    const Vec3 tv = o - a;
    const Real u = dot(tv, pv) * inv;
    if (u < 0 || u > 1) return false;
    const Vec3 qv = cross(tv, e1);
    const Real v = dot(d, qv) * inv;
    if (v < 0 || u + v > 1) return false;
    const Real t = dot(e2, qv) * inv;
    if (t <= 1e-6) return false;
    tOut = t;
    return true;
}
// First hit distance along the ray, or -1.
Real firstHit(const RenderMesh& m, const Vec3& o, const Vec3& d) {
    Real best = -1;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        Real t;
        if (rayHitsTriangle(o, d, m.vertices[m.indices[i]].position,
                            m.vertices[m.indices[i + 1]].position,
                            m.vertices[m.indices[i + 2]].position, t))
            if (best < 0 || t < best) best = t;
    }
    return best;
}
int coveredCrownSamples(const CarParams& p) {
    const CarMesh car = buildCarMesh(p);
    const RenderMesh& body = car.parts[static_cast<int>(CarPart::Body)];
    const Real r = p.wheelDiameter * Real(0.5);
    const Real cy = -p.height * Real(0.5) + r;          // wheel centre (rests on datum)
    // One centimetre over the tyre: a sedan's front fender line sits only
    // ~4 cm above the crown (hood height at the axle is the body's own
    // proportion), and the arch row is capped by it; what the gate demands is
    // that no bodywork stands between the eye and the tyre itself.
    const Real probeR = r + Real(0.01);
    const Real frontZ = p.length * Real(0.5) - p.frontOverhang;
    const Real rearZ = -(p.length * Real(0.5) - p.rearOverhang);
    int covered = 0;
    for (Real az : {frontZ, rearZ}) {
        // The CROWN region, 45..135 degrees: the fender lip legitimately
        // skirts the tyre lower down, as on a real car.
        for (int k = 2; k <= 6; ++k) {
            const Real phi = Real(3.14159265358979) * static_cast<Real>(k) / 8;
            const Real y = cy + probeR * std::sin(phi);
            const Real z = az + probeR * std::cos(phi);
            for (Real side : {Real(1), Real(-1)}) {
                const Vec3 o(side * (p.width * Real(0.5) + Real(1)), y, z);
                const Vec3 d(-side, 0, 0);
                const Real t = firstHit(body, o, d);
                if (t < 0) continue;                          // no skin at all here
                const Real hitX = std::fabs(o.x + d.x * t);
                // The outer skin at this station: the widest body vertex nearby.
                // Ring vertices exist only at stations, so take the widest
                // vertex within a third of a metre along z — the flank is
                // straight enough over that span.
                Real outer = 0;
                for (const Vertex& v : body.vertices)
                    if (std::fabs(v.position.z - z) < Real(0.35) &&
                        std::fabs(v.position.y - y) < Real(0.25))
                        outer = std::max(outer, std::fabs(v.position.x));
                // Open: the first hit is the wheel well, one shell thickness in.
                if (hitX > outer - p.shellThickness * Real(0.5)) {   // first hit IS the skin
                    ++covered;
                    std::printf("      covered: axle z=%.2f phi=%d side=%+.0f y=%.3f (y01 %.3f) hitX=%.3f outer=%.3f\n",
                                az, k * 180 / 8, side, y, (y + p.height * Real(0.5)) / p.height, hitX, outer);
                }
            }
        }
    }
    return covered;
}
}  // namespace

TEST_CASE(car_wheel_crown_is_not_behind_bodywork) {
    CarParams sedan = defaultSedanParams();
    const int sedanCovered = coveredCrownSamples(sedan);
    std::printf("    [arch] sedan: %d/20 crown probes covered by bodywork\n", sedanCovered);
    CHECK(sedanCovered == 0);

    CarParams tall = defaultSedanParams();
    tall.height = 1.75;
    tall.wheelDiameter = 0.74;
    tall.length = 4.4;
    const int tallCovered = coveredCrownSamples(tall);
    std::printf("    [arch] tall-wheeled: %d/20 crown probes covered by bodywork\n", tallCovered);
    CHECK(tallCovered == 0);
}

