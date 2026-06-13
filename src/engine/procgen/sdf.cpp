#include "sdf.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
double mix(double a, double b, double t) { return a + (b - a) * t; }
}  // namespace

// --- primitives ---

Sdf sdfSphere(const Vec3& center, double radius) {
    return [center, radius](const Vec3& p) { return (p - center).length() - radius; };
}

Sdf sdfBox(const Vec3& center, const Vec3& halfExtent) {
    return [center, halfExtent](const Vec3& p) {
        Vec3 q(std::abs(p.x - center.x) - halfExtent.x,
               std::abs(p.y - center.y) - halfExtent.y,
               std::abs(p.z - center.z) - halfExtent.z);
        Vec3 qPos(std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0));
        double inside = std::min(std::max(q.x, std::max(q.y, q.z)), 0.0);
        return qPos.length() + inside;
    };
}

Sdf sdfCapsule(const Vec3& a, const Vec3& b, double radius) {
    return [a, b, radius](const Vec3& p) {
        Vec3 pa = p - a, ba = b - a;
        double denom = dot(ba, ba);
        double h = denom > 0.0 ? clamp01(dot(pa, ba) / denom) : 0.0;
        return (pa - ba * h).length() - radius;
    };
}

// --- combinators ---

Sdf sdfUnion(Sdf a, Sdf b) {
    return [a, b](const Vec3& p) { return std::min(a(p), b(p)); };
}

Sdf sdfIntersect(Sdf a, Sdf b) {
    return [a, b](const Vec3& p) { return std::max(a(p), b(p)); };
}

Sdf sdfSubtract(Sdf a, Sdf b) {
    return [a, b](const Vec3& p) { return std::max(a(p), -b(p)); };
}

Sdf sdfSmoothUnion(Sdf a, Sdf b, double k) {
    return [a, b, k](const Vec3& p) {
        double da = a(p), db = b(p);
        if (k <= 0.0) return std::min(da, db);
        double h = clamp01(0.5 + 0.5 * (db - da) / k);
        return mix(db, da, h) - k * h * (1.0 - h);
    };
}

Sdf sdfUnion(const std::vector<Sdf>& parts) {
    return [parts](const Vec3& p) {
        double d = 1e30;
        for (const Sdf& s : parts) d = std::min(d, s(p));
        return d;
    };
}

Sdf sdfSmoothUnion(const std::vector<Sdf>& parts, double k) {
    return [parts, k](const Vec3& p) {
        double d = 1e30;
        bool first = true;
        for (const Sdf& s : parts) {
            double da = s(p);
            if (first) { d = da; first = false; continue; }
            if (k <= 0.0) { d = std::min(d, da); continue; }
            double h = clamp01(0.5 + 0.5 * (d - da) / k);
            d = mix(d, da, h) - k * h * (1.0 - h);
        }
        return d;
    };
}

Vec3 sdfGradient(const Sdf& field, const Vec3& p, double eps) {
    double gx = field(Vec3(p.x + eps, p.y, p.z)) - field(Vec3(p.x - eps, p.y, p.z));
    double gy = field(Vec3(p.x, p.y + eps, p.z)) - field(Vec3(p.x, p.y - eps, p.z));
    double gz = field(Vec3(p.x, p.y, p.z + eps)) - field(Vec3(p.x, p.y, p.z - eps));
    Vec3 g(gx, gy, gz);
    return g.lengthSquared() > 0.0 ? normalize(g) : Vec3(0, 1, 0);
}

// --- Surface Nets polygonization ---

RenderMesh polygonizeSdf(const Sdf& field, const SdfBounds& bounds, int resolution) {
    RenderMesh mesh;
    const int res = std::max(1, resolution);
    const int N = res + 1;                       // grid corners per axis
    const Vec3 span = bounds.max - bounds.min;
    const Vec3 cell(span.x / res, span.y / res, span.z / res);

    auto cornerPos = [&](int x, int y, int z) {
        return Vec3(bounds.min.x + x * cell.x, bounds.min.y + y * cell.y,
                    bounds.min.z + z * cell.z);
    };

    // Sample the field at every grid corner.
    std::vector<double> s(static_cast<size_t>(N) * N * N);
    auto sIdx = [&](int x, int y, int z) { return (static_cast<size_t>(z) * N + y) * N + x; };
    for (int z = 0; z < N; z++)
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++)
                s[sIdx(x, y, z)] = field(cornerPos(x, y, z));

    // One vertex per sign-changing cell, at the average of its edge crossings.
    static const int CORNER[8][3] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},
                                     {0,0,1},{1,0,1},{1,1,1},{0,1,1}};
    static const int EDGE[12][2] = {{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},
                                    {6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
    std::vector<int> cellVert(static_cast<size_t>(res) * res * res, -1);
    auto cIdx = [&](int x, int y, int z) { return (static_cast<size_t>(z) * res + y) * res + x; };

    for (int cz = 0; cz < res; cz++)
        for (int cy = 0; cy < res; cy++)
            for (int cx = 0; cx < res; cx++) {
                double d[8];
                bool hasNeg = false, hasPos = false;
                for (int i = 0; i < 8; i++) {
                    d[i] = s[sIdx(cx + CORNER[i][0], cy + CORNER[i][1], cz + CORNER[i][2])];
                    (d[i] < 0.0 ? hasNeg : hasPos) = true;
                }
                if (!(hasNeg && hasPos)) continue;   // cell doesn't straddle the surface

                Vec3 sum;
                int count = 0;
                for (int e = 0; e < 12; e++) {
                    int a = EDGE[e][0], b = EDGE[e][1];
                    if ((d[a] < 0.0) == (d[b] < 0.0)) continue;
                    double t = d[a] / (d[a] - d[b]);   // linear crossing along the edge
                    Vec3 pa = cornerPos(cx + CORNER[a][0], cy + CORNER[a][1], cz + CORNER[a][2]);
                    Vec3 pb = cornerPos(cx + CORNER[b][0], cy + CORNER[b][1], cz + CORNER[b][2]);
                    sum += pa + (pb - pa) * t;
                    count++;
                }
                Vec3 v = sum * (1.0 / count);
                cellVert[cIdx(cx, cy, cz)] = static_cast<int>(mesh.vertices.size());
                mesh.vertices.push_back(Vertex(v, sdfGradient(field, v)));   // outward normal
            }

    // Stitch a quad across every grid edge whose endpoints differ in sign; the
    // four cells around it own the corners. Winding must be GLOBALLY consistent
    // (every shared edge traversed oppositely by its two quads) or adjacent
    // triangles disagree and half get backface-culled — the stripey holes.
    // Orienting each quad independently by a normal can't guarantee that, so we
    // use a deterministic rule: each axis ring below is ordered with the same
    // handedness (forward right-hand normal = +axis), and we flip purely by the
    // edge's sign (`flip` when the lower corner is inside), giving the engine's
    // clockwise-front orientation uniformly. Per-vertex normals (gradient) give
    // the smooth shading regardless.
    auto emitQuad = [&](int i0, int i1, int i2, int i3, bool flip) {
        if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0) return;
        auto tri = [&](int a, int b, int c) {
            mesh.indices.push_back((uint32_t)a);
            mesh.indices.push_back((uint32_t)b);
            mesh.indices.push_back((uint32_t)c);
        };
        // The four dual vertices rarely form a planar/convex quad on thin or
        // pinched features; splitting along the *shorter* diagonal avoids the
        // slivers and folds that read as little holes. Winding is preserved.
        double d02 = (mesh.vertices[i0].position - mesh.vertices[i2].position).lengthSquared();
        double d13 = (mesh.vertices[i1].position - mesh.vertices[i3].position).lengthSquared();
        if (d02 <= d13) {
            if (!flip) { tri(i0, i1, i2); tri(i0, i2, i3); }
            else       { tri(i0, i3, i2); tri(i0, i2, i1); }
        } else {
            if (!flip) { tri(i1, i2, i3); tri(i1, i3, i0); }
            else       { tri(i1, i0, i3); tri(i1, i3, i2); }
        }
    };

    for (int z = 0; z < N; z++)
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++) {
                double here = s[sIdx(x, y, z)];
                bool flip = here < 0.0;   // lower corner inside -> reverse
                // The 4 surrounding cells span the two perpendicular axes in
                // [k-1, k], so all four exist only for those coords in [1,res-1].
                // +X ring (in y,z): forward right-hand normal = +X.
                if (x < res && y >= 1 && y < res && z >= 1 && z < res &&
                    (here < 0.0) != (s[sIdx(x+1,y,z)] < 0.0)) {
                    emitQuad(cellVert[cIdx(x, y-1, z-1)], cellVert[cIdx(x, y, z-1)],
                             cellVert[cIdx(x, y, z)],     cellVert[cIdx(x, y-1, z)], flip);
                }
                // +Y ring (in x,z): ordered z-then-x so forward RH normal = +Y.
                if (y < res && x >= 1 && x < res && z >= 1 && z < res &&
                    (here < 0.0) != (s[sIdx(x,y+1,z)] < 0.0)) {
                    emitQuad(cellVert[cIdx(x-1, y, z-1)], cellVert[cIdx(x-1, y, z)],
                             cellVert[cIdx(x, y, z)],     cellVert[cIdx(x, y, z-1)], flip);
                }
                // +Z ring (in x,y): forward RH normal = +Z.
                if (z < res && x >= 1 && x < res && y >= 1 && y < res &&
                    (here < 0.0) != (s[sIdx(x,y,z+1)] < 0.0)) {
                    emitQuad(cellVert[cIdx(x-1, y-1, z)], cellVert[cIdx(x, y-1, z)],
                             cellVert[cIdx(x, y, z)],     cellVert[cIdx(x-1, y, z)], flip);
                }
            }

    return mesh;
}

}  // namespace engine
