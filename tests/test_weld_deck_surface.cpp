// The welded deck is a DRIVING SURFACE, so it must SIT ON the road it was
// authored from.
//
// weldSolid builds the deck top by triangulating the ribbon's 2D outline with
// earcut and lifting each vertex to the road height. Ear clipping returns a VALID
// triangulation but never a well-shaped one: on a long serpentine corridor it
// emits faces spanning hundreds of metres. A triangle is a FLAT plane through its
// three corners, so a face spanning a grade break chords straight across it —
// the corners sit on the road while the middle of the face hangs metres below it.
// That is the "dark wedge" at freeway_lab's gores (task #60), and a cliff in the
// surface cars and the player drive on.
//
// The invariant is deviation from the authored profile, NOT triangle size or drop:
// a 200 m face on a 3.5% grade legitimately drops 7 m and is perfectly correct.
// What is never correct is the SHEET departing from the road between its corners,
// so this samples each face's interior (edge midpoints + centroid) — exactly where
// a chord across a curve deviates most — and compares against the corridor's own
// vertical profile.
#include "test_framework.h"

#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/alignment.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {

// freeway_lab.json's corridor, verbatim — the level the wedges were seen on. Its
// profile has real grade BREAKS (a +4% climb to a crest at s=560, a 0% run, then
// a -5% fall), which is what a long chord cuts across.
CorridorDef freewayLabCorridor() {
    CorridorDef c;
    std::vector<Vec2> pts = {{-560, -420}, {-180, -240}, {120, 40},
                             {180, 420},   {520, 560},   {820, 700}};
    c.horizontal = Alignment::fromPolyline(pts, 240.0, 70.0);
    c.designSpeed = 30.0;
    c.laneWidth = 3.6;
    c.lanes.throughLanes = 4;
    c.medianWidth = 1.4;
    c.vertical.pvis = {{0, 13, 0},     {140, 14, 100}, {260, 14, 140},
                       {560, 26, 160}, {860, 26, 160}, {1120, 12, 140},
                       {1500, 10, 0},  {1900, 10, 0}};
    return c;
}

bool isDeckTop(const RenderMesh& m, std::size_t i, const Vec3& topColor) {
    const Vertex& v = m.vertices[m.indices[i]];
    return v.normal.y > 0.5 && std::fabs(v.color.x - topColor.x) < 0.01 &&
           std::fabs(v.color.y - topColor.y) < 0.01 &&
           std::fabs(v.color.z - topColor.z) < 0.01;
}

}  // namespace

TEST_CASE(weld_deck_surface_sits_on_the_road) {
    CorridorDef c = freewayLabCorridor();
    auto ground = [](Real, Real) -> Real { return 0.0; };
    std::vector<UnionSpine> spines = corridorDeckSpines(c, ground, 3.0);
    WeldSolidParams wp;
    wp.barriers = true;
    wp.thickness = 0.5;
    wp.heightAt = ground;
    RenderMesh m = weldSolid(spines, wp);
    CHECK(!m.vertices.empty());

    // Authored centreline height nearest a world XZ (coarse scan + refine).
    const Real L = c.horizontal.length();
    auto roadYAt = [&](double x, double z) {
        const Vec2 q(x, z);
        Real bs = 0, bd = 1e30;
        for (Real s = 0; s <= L; s += 4.0) {
            Real d = (c.horizontal.pos(s) - q).lengthSquared();
            if (d < bd) { bd = d; bs = s; }
        }
        for (Real s = std::max(Real(0), bs - 4); s <= std::min(L, bs + 4); s += 0.25) {
            Real d = (c.horizontal.pos(s) - q).lengthSquared();
            if (d < bd) { bd = d; bs = s; }
        }
        return (double)c.vertical.elevation(bs);
    };

    // Superelevation tilts the deck by up to hw*7% (~1.7 m at the verge) off the
    // centreline height, so only a departure well beyond that is a real defect.
    const double tol = 2.5;
    int checked = 0, bad = 0;
    double worst = 0.0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        if (!isDeckTop(m, i, wp.topColor)) continue;
        const Vec3& a = m.vertices[m.indices[i]].position;
        const Vec3& b = m.vertices[m.indices[i + 1]].position;
        const Vec3& d = m.vertices[m.indices[i + 2]].position;
        // Interior probes: a plane through 3 points on a curve deviates most
        // BETWEEN them, never at the corners (which are sampled onto the road).
        const Vec3 probes[4] = {(a + b) * 0.5f, (b + d) * 0.5f, (d + a) * 0.5f,
                                (a + b + d) * (1.0f / 3.0f)};
        for (const Vec3& q : probes) {
            ++checked;
            const double dev = std::fabs((double)q.y - roadYAt(q.x, q.z));
            worst = std::max(worst, dev);
            if (dev > tol) ++bad;
        }
    }
    std::printf("[deck] tris=%zu probes=%d off-road=%d worstDev=%.2f m\n",
                m.indices.size() / 3, checked, bad, worst);
    CHECK(checked > 400);
    CHECK(worst < tol);
    CHECK(bad == 0);
}
