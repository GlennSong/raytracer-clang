#include "ground_clutter.h"

#include "proc_rng.h"
#include "stylized_rock.h"
#include "../mesh_builder.h"

#include <cmath>
#include <vector>

namespace engine {

bool clutterKindFromName(const std::string& n, ClutterKind& out) {
    if (n == "stones") out = ClutterKind::Stones;
    else if (n == "twigs") out = ClutterKind::Twigs;
    else return false;
    return true;
}

namespace {
// stones: the rock library's pebbles (a cluster of small faceted stones), sunk a third of their height
RenderMesh stones(uint32_t seed, double size) {
    StylizedRockParams p;
    p.family = RockFamily::Pebbles;
    p.material = RockMaterial::Granite;
    p.size = size;
    p.moss = 0.0;
    RenderMesh m = stylizedRock(seed, p);
    double hi = 0.0;
    for (const Vertex& v : m.vertices) hi = std::max(hi, static_cast<double>(v.position.y));
    const double sink = 0.35 * hi;
    MeshBuilder::deform(m, [&](const Vec3& q) { return Vec3(q.x, q.y - sink, q.z); });
    return m;
}

// twigs: one to three crooked sticks lying on the ground, each a tapered tube with a side shoot
RenderMesh twigs(uint32_t seed, double size) {
    ProcRng rng(seed);
    RenderMesh m;
    const int sticks = 1 + rng.below(3);
    for (int k = 0; k < sticks; ++k) {
        const double len = size * rng.in(0.45, 1.0), r = size * rng.in(0.012, 0.028);
        const double a = rng.in(0.0, 6.2831853);
        const Vec3 dir(std::cos(a), 0.0, std::sin(a)), side(-std::sin(a), 0.0, std::cos(a));
        const Vec3 start = side * (size * rng.in(-0.25, 0.25)) - dir * (0.5 * len);
        std::vector<Vec3> pts;
        std::vector<double> radii;
        std::vector<Vec3> cols;
        const Vec3 bark = Vec3(0.085, 0.062, 0.042) * rng.in(0.8, 1.25);
        const int seg = 5;
        double bend = 0.0;
        for (int i = 0; i <= seg; ++i) {
            const double t = static_cast<double>(i) / seg;
            bend += rng.in(-0.08, 0.08);
            // lying on the ground, the middle a little proud of it (a stick rests on its ends and its knots)
            pts.push_back(start + dir * (t * len) + side * (bend * len * 0.3) + Vec3(0, r * (0.6 + 0.6 * std::sin(t * 3.14159)), 0));
            radii.push_back(r * (1.0 - 0.55 * t));
            cols.push_back(bark * (0.85 + 0.3 * rng.next()));
        }
        MeshBuilder::append(m, MeshBuilder::tube(pts, radii, 5, cols));
        if (rng.next() < 0.7) {   // a side shoot from somewhere along it
            const int at = 1 + rng.below(seg - 2);
            const double sa = a + (rng.next() < 0.5 ? 1.0 : -1.0) * rng.in(0.4, 0.9);
            const Vec3 sd(std::cos(sa), 0.0, std::sin(sa));
            const double sl = len * rng.in(0.2, 0.4);
            std::vector<Vec3> sp = {pts[static_cast<std::size_t>(at)], pts[static_cast<std::size_t>(at)] + sd * (0.5 * sl),
                                    pts[static_cast<std::size_t>(at)] + sd * sl + Vec3(0, -r * 0.3, 0)};
            std::vector<double> sr = {radii[static_cast<std::size_t>(at)] * 0.6, radii[static_cast<std::size_t>(at)] * 0.4,
                                      radii[static_cast<std::size_t>(at)] * 0.2};
            MeshBuilder::append(m, MeshBuilder::tube(sp, sr, 4, {bark, bark, bark}));
        }
    }
    return m;
}
}  // namespace

RenderMesh groundClutter(ClutterKind kind, uint32_t seed, double size) {
    return kind == ClutterKind::Twigs ? twigs(seed, size) : stones(seed, size);
}

}  // namespace engine
