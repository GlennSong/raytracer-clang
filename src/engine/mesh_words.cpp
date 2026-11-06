// THE MESH LIBRARY's solids and fills (mesh_builder.h): boxes, blocks, polygons, plan prisms, cylinders, and the one
// triangulator. Every face here goes out through emitQuad / emitTri / triFacing, wound to the normal it is given.
#include "mesh_builder.h"
#include "procgen/city/triangulate.h"

#include <algorithm>
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
    auto flat = [y0](const Vec2&, const Vec2&) { return Vec2(y0, y0); };
    emitSkirt(mesh, outline, y1, flat, color);
    for (const auto& h : holes)
        if (h.size() >= 3) emitSkirt(mesh, h, y1, flat, color, true);
}

void MeshBuilder::emitSkirt(RenderMesh& mesh, const std::vector<Vec2>& outline, double topY,
                            const std::function<Vec2(const Vec2&, const Vec2&)>& bottoms, const Vec3& color, bool facingIn) {
    if (outline.size() < 3) return;
    // an edge's outward side is away from the outline's area (toward a hole's inside when `facingIn`)
    double area2 = 0;
    for (std::size_t i = 0; i < outline.size(); ++i) area2 += cross(outline[i], outline[(i + 1) % outline.size()]);
    const double side = (area2 > 0 ? 1.0 : -1.0) * (facingIn ? -1.0 : 1.0);   // CCW in (x, z): the interior on the left
    for (std::size_t i = 0; i < outline.size(); ++i) {
        const Vec2 a = outline[i], b = outline[(i + 1) % outline.size()], d = b - a;
        const double l = d.length();
        if (l < 1e-9) continue;
        const Vec2 out = Vec2(d.y / l, -d.x / l) * side;   // the right of a->b
        const Vec2 lo = bottoms(a, b);
        emitQuad(mesh, Vec3(a.x, lo.x, a.y), Vec3(b.x, lo.y, b.y), Vec3(b.x, topY, b.y), Vec3(a.x, topY, a.y), Vec3(out.x, 0, out.y),
                 color);
    }
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

namespace {
// one corner of a flat shape: at the plane point p of frame f, facing n, uv the plane's metres
uint32_t flatVertex(RenderMesh& mesh, const MeshFrame& f, const Vec2& p, const Vec3& n, const Vec3& color) {
    const uint32_t i = MeshBuilder::vertex(mesh, f.at(p.x, 0, p.y), n, color);
    Vertex& v = mesh.vertices[i];
    v.u = static_cast<float>(p.x);
    v.v = static_cast<float>(p.y);
    v.tangent = f.x.lengthSquared() > 1e-24 ? normalize(f.x) : Vec3(1, 0, 0);
    return i;
}
}  // namespace

void MeshBuilder::emitBand(RenderMesh& mesh, const MeshFrame& f, const MeshProfile& outer, const MeshProfile& inner,
                           const Vec3& color) {
    const std::size_t n = std::min(outer.pts.size(), inner.pts.size());
    if (n < 2 || f.y.lengthSquared() < 1e-24) return;
    const Vec3 up = normalize(f.y);
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (std::size_t k = 0; k < n; ++k) {
        flatVertex(mesh, f, outer.pts[k], up, color);
        flatVertex(mesh, f, inner.pts[k], up, color);
    }
    const std::size_t spans = outer.closed ? n : n - 1;
    for (std::size_t k = 0; k < spans; ++k) {
        const uint32_t a = base + static_cast<uint32_t>(2 * k), b = base + static_cast<uint32_t>(2 * ((k + 1) % n));
        quadFacing(mesh, a, b, b + 1, a + 1, up);
    }
}

void MeshBuilder::emitFill(RenderMesh& mesh, const MeshFrame& f, const MeshProfile& outline, const Vec3& color,
                           const std::vector<MeshProfile>& holes) {
    if (outline.pts.size() < 3 || f.y.lengthSquared() < 1e-24) return;
    const Vec3 up = normalize(f.y);
    std::vector<std::vector<Vec2>> h2;
    for (const MeshProfile& h : holes) h2.push_back(h.pts);
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (const Vec2& p : outline.pts) flatVertex(mesh, f, p, up, color);
    for (const std::vector<Vec2>& h : h2)
        for (const Vec2& p : h) flatVertex(mesh, f, p, up, color);
    for (const std::array<int, 3>& t : triangulate(outline.pts, h2))
        triFacing(mesh, base + static_cast<uint32_t>(t[0]), base + static_cast<uint32_t>(t[1]), base + static_cast<uint32_t>(t[2]), up);
}

void MeshBuilder::emitAnnulus(RenderMesh& mesh, const MeshFrame& f, double rIn, double rOut, const Vec3& color, int sides,
                              double a0, double a1, int rings) {
    if (rOut <= rIn || f.y.lengthSquared() < 1e-24) return;
    sides = std::max(3, sides);
    rings = std::max(1, rings);
    const bool whole = std::fabs(a1 - a0) >= TAU - 1e-9;
    auto arc = [&](double r) {   // the ring at radius r: closed for a whole turn, else its sides + 1 points
        MeshProfile p;
        p.closed = whole;
        const int pts = whole ? sides : sides + 1;
        for (int k = 0; k < pts; ++k) {
            const double a = a0 + (a1 - a0) * k / sides;
            p.pts.push_back(Vec2(std::cos(a) * r, std::sin(a) * r));
        }
        return p;
    };
    const Vec3 up = normalize(f.y);
    for (int ri = 0; ri < rings; ++ri) {
        const double r0 = rIn + (rOut - rIn) * ri / rings, r1 = rIn + (rOut - rIn) * (ri + 1) / rings;
        const MeshProfile outer = arc(r1);
        if (r0 > 1e-12) {
            emitBand(mesh, f, outer, arc(r0), color);
            continue;
        }
        // the disc's middle: a fan from the centre
        const uint32_t c = flatVertex(mesh, f, Vec2(0, 0), up, color), base = static_cast<uint32_t>(mesh.vertices.size());
        for (const Vec2& p : outer.pts) flatVertex(mesh, f, p, up, color);
        const std::size_t n = outer.pts.size(), spans = whole ? n : n - 1;
        for (std::size_t k = 0; k < spans; ++k)
            triFacing(mesh, c, base + static_cast<uint32_t>(k), base + static_cast<uint32_t>((k + 1) % n), up);
    }
}

}  // namespace engine
