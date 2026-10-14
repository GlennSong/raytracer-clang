// THE MESH LIBRARY's solids and fills (mesh_builder.h): boxes, blocks, polygons, plan prisms, cylinders, and the one
// triangulator. Every face here goes out through emitQuad / emitTri / triFacing, wound to the normal it is given.
#include "mesh_builder.h"
#include "procgen/city/triangulate.h"

#include <cmath>

namespace engine {

namespace {
Vec3 centroid(const Vec3* p, int n) {
    Vec3 c(0, 0, 0);
    for (int i = 0; i < n; ++i) c = c + p[i];
    return c * (1.0 / n);
}
// The normal of a planar-ish quad a b c d (Newell), turned away from `inside`.
Vec3 faceNormalAway(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& inside) {
    const Vec3 q[4] = {a, b, c, d};
    Vec3 n(0, 0, 0);
    for (int i = 0; i < 4; ++i) {
        const Vec3& u = q[i];
        const Vec3& v = q[(i + 1) % 4];
        n.x += (u.y - v.y) * (u.z + v.z);
        n.y += (u.z - v.z) * (u.x + v.x);
        n.z += (u.x - v.x) * (u.y + v.y);
    }
    if (n.lengthSquared() < 1e-24) return Vec3(0, 0, 0);
    n = normalize(n);
    const Vec3 mid = (a + b + c + d) * 0.25;
    return dot(n, mid - inside) < 0 ? n * -1.0 : n;
}
}  // namespace

std::vector<std::array<int, 3>> MeshBuilder::triangulate(const std::vector<Vec2>& outline, const std::vector<std::vector<Vec2>>& holes) {
    return triangulateIndices(outline, holes);
}

void MeshBuilder::emitBlock(RenderMesh& mesh, const Vec3 (&foot)[4], const Vec3 (&head)[4], const Vec3& color, uint8_t faces) {
    Vec3 all[8] = {foot[0], foot[1], foot[2], foot[3], head[0], head[1], head[2], head[3]};
    const Vec3 inside = centroid(all, 8);
    auto quad = [&](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        const Vec3 n = faceNormalAway(a, b, c, d, inside);
        if (n.lengthSquared() > 0) emitQuad(mesh, a, b, c, d, n, color);
    };
    if (faces & kFaceBottom) quad(foot[0], foot[1], foot[2], foot[3]);
    if (faces & kFaceTop) quad(head[0], head[1], head[2], head[3]);
    static const uint8_t side[4] = {kFaceNegZ, kFacePosX, kFacePosZ, kFaceNegX};
    for (int k = 0; k < 4; ++k)
        if (faces & side[k]) {
            const int j = (k + 1) % 4;
            quad(foot[k], foot[j], head[j], head[k]);
        }
}

void MeshBuilder::emitBox(RenderMesh& mesh, const MeshFrame& f, const Vec3& color, uint8_t faces) {
    const Vec3 foot[4] = {f.at(0, 0, 0), f.at(1, 0, 0), f.at(1, 0, 1), f.at(0, 0, 1)};
    const Vec3 head[4] = {f.at(0, 1, 0), f.at(1, 1, 0), f.at(1, 1, 1), f.at(0, 1, 1)};
    emitBlock(mesh, foot, head, color, faces);
}

void MeshBuilder::emitBox(RenderMesh& mesh, const Vec3& lo, const Vec3& hi, const Vec3& color, uint8_t faces) {
    emitBox(mesh, MeshFrame::aabb(lo, hi), color, faces);
}

void MeshBuilder::emitPolygon(RenderMesh& mesh, const std::vector<Vec3>& ring, const std::vector<std::vector<Vec3>>& holes,
                              const Vec3& normalIn, const Vec3& color, double uvPerMetre) {
    if (ring.size() < 3 || normalIn.lengthSquared() < 1e-24) return;
    const Vec3 n = normalize(normalIn);
    const Vec3 ref = std::fabs(n.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    const Vec3 u = normalize(cross(ref, n)), v = cross(n, u);
    std::vector<Vec3> pts(ring.begin(), ring.end());
    std::vector<Vec2> outline;
    for (const Vec3& p : ring) outline.push_back(Vec2(dot(p, u), dot(p, v)));
    std::vector<std::vector<Vec2>> h2;
    for (const auto& h : holes) {
        h2.emplace_back();
        for (const Vec3& p : h) { h2.back().push_back(Vec2(dot(p, u), dot(p, v))); pts.push_back(p); }
    }
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (const Vec3& p : pts) {
        Vertex vx(p, n, u, uvPerMetre > 0 ? static_cast<float>(dot(p, u) * uvPerMetre) : 0.0f,
                  uvPerMetre > 0 ? static_cast<float>(dot(p, v) * uvPerMetre) : 0.0f);
        vx.color = color;
        mesh.vertices.push_back(vx);
    }
    for (const std::array<int, 3>& t : triangulate(outline, h2))
        triFacing(mesh, base + static_cast<uint32_t>(t[0]), base + static_cast<uint32_t>(t[1]), base + static_cast<uint32_t>(t[2]), n);
}

void MeshBuilder::emitPlanPrism(RenderMesh& mesh, const std::vector<Vec2>& outline, const std::vector<std::vector<Vec2>>& holes,
                                double y0, double y1, const Vec3& color, uint8_t faces) {
    if (outline.size() < 3 || y1 <= y0) return;
    auto at = [](const Vec2& p, double y) { return Vec3(p.x, y, p.y); };
    std::vector<std::vector<Vec3>> topHoles, botHoles;
    for (const auto& h : holes) {
        topHoles.emplace_back(); botHoles.emplace_back();
        for (const Vec2& p : h) { topHoles.back().push_back(at(p, y1)); botHoles.back().push_back(at(p, y0)); }
    }
    std::vector<Vec3> top, bot;
    for (const Vec2& p : outline) { top.push_back(at(p, y1)); bot.push_back(at(p, y0)); }
    if (faces & kFaceTop) emitPolygon(mesh, top, topHoles, Vec3(0, 1, 0), color);
    if (faces & kFaceBottom) emitPolygon(mesh, bot, botHoles, Vec3(0, -1, 0), color);
    if (!(faces & kFaceSides)) return;
    // walls: an edge's outward side is away from the polygon's area (the outline) or toward the hole's inside
    auto signedArea = [](const std::vector<Vec2>& r) {
        double a = 0;
        for (std::size_t i = 0; i < r.size(); ++i) a += cross(r[i], r[(i + 1) % r.size()]);
        return a * 0.5;
    };
    auto walls = [&](const std::vector<Vec2>& r, bool hole) {
        const double ccw = signedArea(r) > 0 ? 1.0 : -1.0;   // CCW in (x, z): the interior on the left
        for (std::size_t i = 0; i < r.size(); ++i) {
            const Vec2 a = r[i], b = r[(i + 1) % r.size()], d = b - a;
            const double l = d.length();
            if (l < 1e-9) continue;
            Vec2 out(d.y / l, -d.x / l);   // the right of a->b
            out = out * ccw * (hole ? -1.0 : 1.0);
            emitQuad(mesh, at(a, y0), at(b, y0), at(b, y1), at(a, y1), Vec3(out.x, 0, out.y), color);
        }
    };
    walls(outline, false);
    for (const auto& h : holes)
        if (h.size() >= 3) walls(h, true);
}

void MeshBuilder::emitCylinder(RenderMesh& mesh, const Vec3& base, double radius, double height, int sides, const Vec3& color,
                               bool faceted, uint8_t faces) {
    sides = std::max(3, sides);
    std::vector<Vec3> ringLo, ringHi;
    for (int i = 0; i < sides; ++i) {
        const double a = 6.283185307179586 * i / sides;
        const Vec3 r(std::cos(a) * radius, 0, std::sin(a) * radius);
        ringLo.push_back(base + r);
        ringHi.push_back(base + r + Vec3(0, height, 0));
    }
    for (int i = 0; i < sides; ++i) {
        const int j = (i + 1) % sides;
        const double a0 = 6.283185307179586 * i / sides, a1 = 6.283185307179586 * j / sides, am = 6.283185307179586 * (i + 0.5) / sides;
        if (faceted) {
            emitQuad(mesh, ringLo[i], ringLo[j], ringHi[j], ringHi[i], Vec3(std::cos(am), 0, std::sin(am)), color);
        } else {
            const uint32_t v0 = vertex(mesh, ringLo[i], Vec3(std::cos(a0), 0, std::sin(a0)), color),
                           v1 = vertex(mesh, ringLo[j], Vec3(std::cos(a1), 0, std::sin(a1)), color),
                           v2 = vertex(mesh, ringHi[j], Vec3(std::cos(a1), 0, std::sin(a1)), color),
                           v3 = vertex(mesh, ringHi[i], Vec3(std::cos(a0), 0, std::sin(a0)), color);
            const Vec3 out(std::cos(am), 0, std::sin(am));
            triFacing(mesh, v0, v1, v2, out);
            triFacing(mesh, v0, v2, v3, out);
        }
    }
    if (faces & kFaceTop) emitPolygon(mesh, ringHi, {}, Vec3(0, 1, 0), color);
    if (faces & kFaceBottom) emitPolygon(mesh, ringLo, {}, Vec3(0, -1, 0), color);
}

}  // namespace engine
