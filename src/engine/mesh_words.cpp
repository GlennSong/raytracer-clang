// THE MESH LIBRARY's solids and fills (mesh_builder.h): boxes, blocks, polygons, plan prisms, cylinders, the one
// triangulator, and the drape on a mesh. Every face here goes out through emitQuad / emitTri / triFacing, wound to the
// normal it is given.
#include "mesh_builder.h"
#include "../hash.h"
#include "procgen/city/triangulate.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

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

// THE DRAPE ON A MESH ---------------------------------------------------------------------------------------------
namespace {
// The surface triangles are found through a grid of this many metres a cell. It is a lookup only: any size gives the
// same pieces, this one keeps a road's few-metre triangles to a handful a cell.
constexpr double kDrapeCell = 4.0;
// Twice the plan area (m^2) under which a triangle stands on edge: no plane to drape on, nothing flat to drape.
constexpr double kDrapeNoPlan2 = 1e-10;
// A piece of less plan area than this (m^2) is a sliver left by a cut along a shared edge: dropped.
constexpr double kDrapeSliver = 1e-8;
// A flat triangle counts as wholly over the surface when its pieces cover it to within this share of its area.
constexpr double kDrapeWholeShare = 1e-6;

// A surface triangle as the drape reads it: its plan corners counter-clockwise, its plane (height at a plan point)
// and the normal of that plane facing up.
struct DrapeUnder {
    Vec2 p[3];
    Vec2 lo, hi;
    double h = 0, hx = 0, hz = 0;   // height h at p[0], rising hx per metre of x and hz per metre of z
    Vec3 up;
    double heightAt(const Vec2& q) const { return h + hx * (q.x - p[0].x) + hz * (q.y - p[0].y); }
};

long long drapeKey(long i, long j) {
    return (static_cast<long long>(i) << 32) ^ static_cast<long long>(static_cast<uint32_t>(j));
}
long drapeCellOf(double x) { return std::lround(std::floor(x / kDrapeCell)); }

bool boxesMeet(const Vec2& lo, const Vec2& hi, const Vec2& lo2, const Vec2& hi2) {
    return lo.x <= hi2.x && lo2.x <= hi.x && lo.y <= hi2.y && lo2.y <= hi.y;
}

// The convex polygon `poly` clipped to the inside of the convex counter-clockwise ring `ring`, one edge's half-plane
// at a time (polygon.h's clipHalfPlane). Empty when less than a sliver is left.
Poly2 clipToConvex(Poly2 poly, const Vec2* ring, std::size_t n) {
    for (std::size_t e = 0; e < n && poly.size() >= 3; ++e) {
        const Vec2 d = ring[(e + 1) % n] - ring[e];
        if (d.lengthSquared() < kDrapeNoPlan2) continue;
        const Vec2 out = normalize(Vec2(d.y, -d.x));   // the edge's outward side (the ring runs counter-clockwise)
        poly = clipHalfPlane(poly, out, dot(out, ring[e]));
    }
    if (poly.size() < 3 || area(poly) < kDrapeSliver) return {};
    return poly;
}

// The convex pieces of convex `poly` that lie outside the convex counter-clockwise `hole`, appended to `out`: the part
// beyond the hole's first edge, then of what is left the part beyond its second, and so on.
void subtractConvex(const Poly2& poly, const Poly2& hole, std::vector<Poly2>& out) {
    Poly2 rest = poly;
    for (std::size_t e = 0; e < hole.size() && rest.size() >= 3; ++e) {
        const Vec2 d = hole[(e + 1) % hole.size()] - hole[e];
        if (d.lengthSquared() < kDrapeNoPlan2) continue;
        const Vec2 n = normalize(Vec2(d.y, -d.x));
        const double c = dot(n, hole[e]);
        const Poly2 beyond = clipHalfPlane(rest, n * -1.0, -c);
        if (beyond.size() >= 3 && area(beyond) >= kDrapeSliver) out.push_back(beyond);
        rest = clipHalfPlane(rest, n, c);
    }
}
}  // namespace

RenderMesh MeshBuilder::drapeOnMesh(const RenderMesh& flat, const std::vector<std::array<Vec3, 3>>& surface, double lift,
                                    double reach, double flush) {
    RenderMesh out;
    out.materialIndex = flat.materialIndex;
    // the surface triangles, planes and grid
    std::vector<DrapeUnder> under;
    under.reserve(surface.size());
    std::unordered_map<long long, std::vector<uint32_t>> grid;
    for (const std::array<Vec3, 3>& s : surface) {
        Vec2 a(s[0].x, s[0].z), b(s[1].x, s[1].z), c(s[2].x, s[2].z);
        double ha = s[0].y, hb = s[1].y, hc = s[2].y;
        double det = cross(b - a, c - a);
        if (std::fabs(det) < kDrapeNoPlan2) continue;
        if (det < 0) {
            std::swap(b, c);
            std::swap(hb, hc);
            det = -det;
        }
        DrapeUnder u;
        u.p[0] = a;
        u.p[1] = b;
        u.p[2] = c;
        const Vec2 ab = b - a, ac = c - a;
        u.h = ha;
        u.hx = ((hb - ha) * ac.y - (hc - ha) * ab.y) / det;
        u.hz = ((hc - ha) * ab.x - (hb - ha) * ac.x) / det;
        u.up = normalize(Vec3(-u.hx, 1.0, -u.hz));
        bounds(Poly2{a, b, c}, u.lo, u.hi);
        const uint32_t id = static_cast<uint32_t>(under.size());
        for (long i = drapeCellOf(u.lo.x); i <= drapeCellOf(u.hi.x); ++i)
            for (long j = drapeCellOf(u.lo.y); j <= drapeCellOf(u.hi.y); ++j) grid[drapeKey(i, j)].push_back(id);
        under.push_back(u);
    }
    std::vector<uint32_t> seenBy(under.size(), UINT32_MAX);   // which flat triangle last listed each surface triangle
    std::vector<uint32_t> cands;
    std::vector<std::pair<double, uint32_t>> ranked;
    std::vector<Poly2> taken, pieces, next;   // taken: the pieces kept so far, over surface triangles takenBy
    std::vector<uint32_t> takenBy;
    for (std::size_t t = 0; t + 2 < flat.indices.size(); t += 3) {
        const Vertex& A = flat.vertices[flat.indices[t]];
        const Vertex& B = flat.vertices[flat.indices[t + 1]];
        const Vertex& C = flat.vertices[flat.indices[t + 2]];
        const Vec2 pa(A.position.x, A.position.z), pb(B.position.x, B.position.z), pc(C.position.x, C.position.z);
        const double det = cross(pb - pa, pc - pa);
        if (std::fabs(det) < kDrapeNoPlan2) continue;   // standing on edge: nothing flat to lay down
        // the side the flat triangle faces (its front, mesh_builder.h): the pieces face the same way
        const double side = cross(C.position - A.position, B.position - A.position).y >= 0 ? 1.0 : -1.0;
        // where a plan point falls in the flat triangle (its weights on A, B, C): what every attribute is read through
        auto weights = [&](const Vec2& q, double& wa, double& wb, double& wc) {
            wb = cross(q - pa, pc - pa) / det;
            wc = cross(pb - pa, q - pa) / det;
            wa = 1.0 - wb - wc;
        };
        auto flatHeight = [&](const Vec2& q) {
            double wa = 0, wb = 0, wc = 0;
            weights(q, wa, wb, wc);
            return wa * A.position.y + wb * B.position.y + wc * C.position.y;
        };
        Poly2 tri{pa, pb, pc};
        if (det < 0) std::swap(tri[1], tri[2]);   // counter-clockwise, as the pieces cut from it stay
        Vec2 lo, hi;
        bounds(tri, lo, hi);
        // the surface triangles near it, nearest in height (at its middle) first
        cands.clear();
        const uint32_t tag = static_cast<uint32_t>(t / 3);
        for (long i = drapeCellOf(lo.x); i <= drapeCellOf(hi.x); ++i)
            for (long j = drapeCellOf(lo.y); j <= drapeCellOf(hi.y); ++j) {
                std::unordered_map<long long, std::vector<uint32_t>>::const_iterator it = grid.find(drapeKey(i, j));
                if (it == grid.end()) continue;
                for (uint32_t id : it->second)
                    if (seenBy[id] != tag && boxesMeet(lo, hi, under[id].lo, under[id].hi)) {
                        seenBy[id] = tag;
                        cands.push_back(id);
                    }
            }
        const Vec2 mid = (pa + pb + pc) * (1.0 / 3.0);
        const double midH = flatHeight(mid);
        ranked.clear();
        for (uint32_t id : cands) ranked.push_back({std::fabs(under[id].heightAt(mid) - midH), id});
        std::sort(ranked.begin(), ranked.end());
        // each surface triangle in turn takes the part of the flat triangle over it that no nearer one took
        taken.clear();
        takenBy.clear();
        for (const std::pair<double, uint32_t>& rk : ranked) {
            const DrapeUnder& U = under[rk.second];
            const Poly2 over = clipToConvex(tri, U.p, 3);
            if (over.empty()) continue;
            pieces.assign(1, over);
            for (const Poly2& hole : taken) {
                Vec2 hlo, hhi;
                bounds(hole, hlo, hhi);
                next.clear();
                for (const Poly2& piece : pieces) {
                    Vec2 plo, phi;
                    bounds(piece, plo, phi);
                    if (!boxesMeet(plo, phi, hlo, hhi) || clipToConvex(piece, hole.data(), hole.size()).empty()) {
                        next.push_back(piece);   // they do not overlap: kept whole (cutting it along the hole's edges would only shred it)
                        continue;
                    }
                    subtractConvex(piece, hole, next);
                }
                pieces.swap(next);
                if (pieces.empty()) break;
            }
            for (const Poly2& piece : pieces) {
                if (piece.size() < 3 || area(piece) < kDrapeSliver) continue;
                const Vec2 at = centroid(piece);
                if (std::fabs(U.heightAt(at) - flatHeight(at)) > reach) continue;   // another level's surface
                taken.push_back(piece);
                takenBy.push_back(rk.second);
            }
        }
        if (taken.empty()) continue;
        // a vertex of the drape at plan point q, `lift` over surface height h, its attributes read from the flat triangle
        auto put = [&](const Vec2& q, double h, const Vec3& n) {
            double wa = 0, wb = 0, wc = 0;
            weights(q, wa, wb, wc);
            const uint32_t vi = vertex(out, Vec3(q.x, h + lift, q.y), n, A.color * wa + B.color * wb + C.color * wc);
            Vertex& v = out.vertices[vi];
            v.u = static_cast<float>(wa * A.u + wb * B.u + wc * C.u);
            v.v = static_cast<float>(wa * A.v + wb * B.v + wc * C.v);
            v.tangent = A.tangent * wa + B.tangent * wb + C.tangent * wc;
            return vi;
        };
        // KEPT WHOLE when the surface does not fold under it: the triangle fully covered, and every corner of every
        // piece within `flush` of the plane through the surface at the triangle's own corners. (Between the corners of
        // the pieces the gap runs straight, so that bounds it everywhere.) A flat road keeps its paint's triangles.
        double coveredArea = 0;
        for (const Poly2& piece : taken) coveredArea += area(piece);
        double cornerH[3] = {0, 0, 0};
        bool whole = std::fabs(coveredArea - 0.5 * std::fabs(det)) <= kDrapeWholeShare * 0.5 * std::fabs(det);
        const Vec2 corner[3] = {pa, pb, pc};
        for (int k = 0; k < 3 && whole; ++k) {
            whole = false;
            for (std::size_t pi = 0; pi < taken.size() && !whole; ++pi)
                for (const Vec2& q : taken[pi])
                    if ((q - corner[k]).lengthSquared() < kDrapeNoPlan2) {
                        cornerH[k] = under[takenBy[pi]].heightAt(corner[k]);
                        whole = true;
                        break;
                    }
        }
        auto cornerPlane = [&](const Vec2& q) {
            double wa = 0, wb = 0, wc = 0;
            weights(q, wa, wb, wc);
            return wa * cornerH[0] + wb * cornerH[1] + wc * cornerH[2];
        };
        for (std::size_t pi = 0; pi < taken.size() && whole; ++pi)
            for (const Vec2& q : taken[pi])
                if (std::fabs(under[takenBy[pi]].heightAt(q) - cornerPlane(q)) > flush) { whole = false; break; }
        if (whole) {
            const Vec3 a3(pa.x, cornerH[0], pa.y), b3(pb.x, cornerH[1], pb.y), c3(pc.x, cornerH[2], pc.y);
            Vec3 n = cross(c3 - a3, b3 - a3);
            if (n.y * side < 0) n = n * -1.0;
            const uint32_t ia = put(pa, cornerH[0], n), ib = put(pb, cornerH[1], n), ic = put(pc, cornerH[2], n);
            triFacing(out, ia, ib, ic, n);
            continue;
        }
        for (std::size_t pi = 0; pi < taken.size(); ++pi) {
            const DrapeUnder& U = under[takenBy[pi]];
            const Vec3 n = U.up * side;
            const uint32_t base = static_cast<uint32_t>(out.vertices.size());
            for (const Vec2& q : taken[pi]) put(q, U.heightAt(q), n);
            for (std::size_t k = 1; k + 1 < taken[pi].size(); ++k)
                triFacing(out, base, base + static_cast<uint32_t>(k), base + static_cast<uint32_t>(k + 1), n);
        }
    }
    return weld(out);   // every piece was given its own corners: the ones neighbours share become one
}

namespace {
// The weld's lookup grid: cells this many tolerances wide, so a vertex within the tolerance of another is in one of
// the (at most two per axis) cells its own tolerance box touches.
constexpr double kWeldCellTolerances = 4.0;
constexpr uint32_t kWeldUnset = 0xFFFFFFFFu;

bool weldNear(const Vec3& a, const Vec3& b, double tol) {
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}
bool weldSame(const Vertex& a, const Vertex& b, const MeshBuilder::WeldTolerance& t) {
    if (!weldNear(a.position, b.position, t.position)) return false;
    if (t.positionOnly) return true;
    return weldNear(a.normal, b.normal, t.direction) && weldNear(a.tangent, b.tangent, t.direction) &&
           std::fabs(static_cast<double>(a.u) - b.u) <= t.uv && std::fabs(static_cast<double>(a.v) - b.v) <= t.uv &&
           weldNear(a.color, b.color, t.color);
}
uint32_t weldCellKey(int64_t x, int64_t y, int64_t z) {
    return hashPair(hashCell(static_cast<uint32_t>(x), static_cast<uint32_t>(y), 0u), static_cast<uint32_t>(z));
}
}  // namespace

RenderMesh MeshBuilder::weld(const RenderMesh& mesh) { return weld(mesh, WeldTolerance()); }

RenderMesh MeshBuilder::weld(const RenderMesh& mesh, const WeldTolerance& tol) {
    const double cell = std::max(tol.position, 1e-9) * kWeldCellTolerances;
    // 1. Each vertex's keeper: the earliest vertex it is the same as (itself, when there is none).
    std::vector<Vertex> kept;
    std::vector<uint32_t> keeperOf(mesh.vertices.size(), kWeldUnset);
    std::unordered_map<uint32_t, std::vector<uint32_t>> grid;   // cell -> kept vertices in it (looked up, never walked)
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vertex& v = mesh.vertices[i];
        const Vec3& p = v.position;
        const int64_t lo[3] = {static_cast<int64_t>(std::floor((p.x - tol.position) / cell)),
                               static_cast<int64_t>(std::floor((p.y - tol.position) / cell)),
                               static_cast<int64_t>(std::floor((p.z - tol.position) / cell))};
        const int64_t hi[3] = {static_cast<int64_t>(std::floor((p.x + tol.position) / cell)),
                               static_cast<int64_t>(std::floor((p.y + tol.position) / cell)),
                               static_cast<int64_t>(std::floor((p.z + tol.position) / cell))};
        uint32_t found = kWeldUnset;
        for (int64_t x = lo[0]; x <= hi[0]; ++x)
            for (int64_t y = lo[1]; y <= hi[1]; ++y)
                for (int64_t z = lo[2]; z <= hi[2]; ++z) {
                    const std::unordered_map<uint32_t, std::vector<uint32_t>>::const_iterator it = grid.find(weldCellKey(x, y, z));
                    if (it == grid.end()) continue;
                    for (uint32_t k : it->second)
                        if (k < found && weldSame(kept[k], v, tol)) found = k;   // the earliest, whatever order cells come in
                }
        if (found == kWeldUnset) {
            found = static_cast<uint32_t>(kept.size());
            kept.push_back(v);
            grid[weldCellKey(static_cast<int64_t>(std::floor(p.x / cell)), static_cast<int64_t>(std::floor(p.y / cell)),
                             static_cast<int64_t>(std::floor(p.z / cell)))].push_back(found);
        }
        keeperOf[i] = found;
    }
    // 2. The triangles on their keepers, without the ones that have no area; then only the vertices still used.
    RenderMesh out;
    out.materialIndex = mesh.materialIndex;
    out.tangentIsData = mesh.tangentIsData;
    std::vector<uint32_t> slot(kept.size(), kWeldUnset);
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t a = keeperOf[mesh.indices[t]], b = keeperOf[mesh.indices[t + 1]], c = keeperOf[mesh.indices[t + 2]];
        if (a == b || b == c || a == c) continue;
        const Vec3 n = cross(kept[b].position - kept[a].position, kept[c].position - kept[a].position);
        if (0.5 * n.length() < tol.sliver) continue;
        for (uint32_t k : {a, b, c}) {
            if (slot[k] == kWeldUnset) {
                slot[k] = static_cast<uint32_t>(out.vertices.size());
                out.vertices.push_back(kept[k]);
            }
            out.indices.push_back(slot[k]);
        }
    }
    return out;
}

}  // namespace engine
