#include "test_framework.h"

#include "../src/engine/procgen/vehicle_mesh.h"

#include <algorithm>
#include <cmath>

using namespace engine;

// The lofted car shell (ADR-0062): curved automotive bodies with painted glass —
// the fix for a fleet of box compositions that all read as cybertrucks. These pin
// the properties that make it read as a CAR: curvature (no box silhouette),
// windows, style differentiation, correct bounds, determinism.

namespace {
const Vec3 kPaint(0.72, 0.10, 0.10);
const Vec3 kGlass(0.05, 0.07, 0.10);

bool isGlass(const Vertex& v) {
    return std::fabs(v.color.x - kGlass.x) < 1e-6 && std::fabs(v.color.y - kGlass.y) < 1e-6;
}

// Widest |x| among vertices in a height band (paint+glass shell only — skip
// wheels/lamps by ignoring near-black tyre verts).
Real widthInBand(const RenderMesh& m, Real yMin, Real yMax) {
    Real w = 0;
    for (const Vertex& v : m.vertices) {
        if (v.color.x < 0.06 && v.color.y < 0.08) continue;   // tyres/glass edges
        if (v.position.y >= yMin && v.position.y <= yMax)
            w = std::max(w, std::fabs((Real)v.position.x));
    }
    return w;
}
}  // namespace

TEST_CASE(car_shell_is_curved_not_a_box) {
    RenderMesh m = buildCarShell(CarShellStyle::Sedan, kPaint, Vec3(1.8, 1.3, 4.2));
    CHECK(!m.vertices.empty());
    CHECK(m.indices.size() % 3 == 0);

    // Tumblehome: the body is widest at the belt line and narrows toward the
    // roof — a box would be equally wide at every height.
    Real beltW = widthInBand(m, -0.20, 0.05);
    Real roofW = widthInBand(m, 0.42, 0.70);
    CHECK(beltW > 0.80);              // a real body side
    CHECK(roofW < beltW * 0.85);      // ...that curves inward going up

    // Plan taper: the nose is narrower than the cabin (box: identical).
    Real noseW = 0, midW = 0;
    for (const Vertex& v : m.vertices) {
        if (v.color.x < 0.06 && v.color.y < 0.08) continue;
        if (v.position.z > 1.85) noseW = std::max(noseW, std::fabs((Real)v.position.x));
        if (std::fabs(v.position.z) < 0.5) midW = std::max(midW, std::fabs((Real)v.position.x));
    }
    CHECK(noseW < midW * 0.90);
}

TEST_CASE(car_shell_has_windows_and_paint) {
    RenderMesh m = buildCarShell(CarShellStyle::Sedan, kPaint, Vec3(1.8, 1.3, 4.2));
    int glass = 0, paint = 0;
    Real glassYMin = 1e9;
    for (const Vertex& v : m.vertices) {
        if (isGlass(v)) { ++glass; glassYMin = std::min(glassYMin, (Real)v.position.y); }
        else if (std::fabs(v.color.x - kPaint.x) < 1e-6) ++paint;
    }
    CHECK(glass > 8);                 // a real greenhouse, not a stray vertex
    CHECK(paint > glass);             // mostly body
    CHECK(glassYMin > -0.1);          // windows live above the belt, never on the hull
}

TEST_CASE(car_shell_styles_differ) {
    Vec3 size(1.9, 1.7, 4.6);
    RenderMesh sedan = buildCarShell(CarShellStyle::Sedan, kPaint, Vec3(1.8, 1.3, 4.2));
    RenderMesh suv = buildCarShell(CarShellStyle::SUV, kPaint, size);
    RenderMesh van = buildCarShell(CarShellStyle::Van, kPaint, Vec3(2.0, 2.1, 5.4));

    auto roofAt = [](const RenderMesh& m, Real z0, Real z1) {
        Real top = -1e9;
        for (const Vertex& v : m.vertices)
            if (v.position.z > z0 && v.position.z < z1)
                top = std::max(top, (Real)v.position.y);
        return top;
    };
    // A sedan's rear deck drops well below its roof; a van carries its roof aft.
    Real sedanRoof = roofAt(sedan, -0.6, 0.2), sedanDeck = roofAt(sedan, -2.0, -1.7);
    CHECK(sedanDeck < sedanRoof - 0.3);
    Real vanRoof = roofAt(van, -0.5, 0.5), vanRear = roofAt(van, -2.5, -2.0);
    CHECK(vanRear > vanRoof - 0.3);   // the slab profile
    // A panel van has glass only at the cab: no glass in its rear half.
    for (const Vertex& v : van.vertices)
        if (isGlass(v)) CHECK(v.position.z > 0.0);
    (void)suv;
}

TEST_CASE(car_shell_fits_its_requested_size) {
    Vec3 size(1.8, 1.3, 4.2);
    RenderMesh m = buildCarShell(CarShellStyle::Sedan, kPaint, size);
    Real maxX = 0, maxY = -1e9, minY = 1e9, maxZ = 0;
    for (const Vertex& v : m.vertices) {
        maxX = std::max(maxX, std::fabs((Real)v.position.x));
        maxY = std::max(maxY, (Real)v.position.y);
        minY = std::min(minY, (Real)v.position.y);
        maxZ = std::max(maxZ, std::fabs((Real)v.position.z));
    }
    CHECK(maxX <= size.x * 0.55);            // width respected (wheels slightly proud)
    CHECK(maxY <= size.y * 0.52);            // roof at ~+H/2
    CHECK(minY < -size.y * 0.45);            // wheels reach below the hull
    CHECK(maxZ <= size.z * 0.52);            // length respected
    CHECK(maxZ > size.z * 0.45);
}

TEST_CASE(car_shell_is_deterministic_and_wheels_optional) {
    Vec3 size(1.8, 1.3, 4.2);
    RenderMesh a = buildCarShell(CarShellStyle::Hatchback, kPaint, size);
    RenderMesh b = buildCarShell(CarShellStyle::Hatchback, kPaint, size);
    CHECK(a.vertices.size() == b.vertices.size());
    CHECK(a.indices.size() == b.indices.size());
    bool same = true;
    for (std::size_t i = 0; i < a.vertices.size(); ++i)
        if (a.vertices[i].position.x != b.vertices[i].position.x ||
            a.vertices[i].position.y != b.vertices[i].position.y ||
            a.vertices[i].position.z != b.vertices[i].position.z)
            same = false;
    CHECK(same);
    // The player's car renders physics wheels as separate entities: withWheels
    // false must drop the tyre geometry.
    RenderMesh noWheels = buildCarShell(CarShellStyle::Sedan, kPaint, size, false);
    RenderMesh withWheels = buildCarShell(CarShellStyle::Sedan, kPaint, size, true);
    CHECK(noWheels.vertices.size() < withWheels.vertices.size());
}
