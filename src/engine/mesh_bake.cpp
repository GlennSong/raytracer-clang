#include "mesh_bake.h"

#include "mesh_builder.h"
#include "../proc_rng.h"
#include "../triangle_bvh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace engine {

namespace {

bool tagged(const Vertex& v) { return v.u <= -999.0f; }

// The detailed mesh as the path tracer's triangles under its BVH: positions, vertex normals, colours, each corner's
// mean curvature in its uv slot (rec.u at a hit), its part in its material index (rec.materialIndex)
struct HighSurface {
    std::vector<Triangle> tris;
    TriangleBVH bvh;
    HighSurface(const RenderMesh& m, const std::vector<int>* parts, bool curvature) {
        // MEAN CURVATURE per welded point: over its edges, -2 n . (q - p) / |q - p|^2 averaged (1/R on a sphere of R)
        std::vector<float> k(m.vertices.size(), 0.0f);
        if (curvature) {
            std::map<std::array<long long, 3>, int> ids;
            std::vector<int> weld(m.vertices.size());
            for (std::size_t i = 0; i < m.vertices.size(); ++i) {
                const Vec3& p = m.vertices[i].position;
                const std::array<long long, 3> key{std::llround(p.x * 1e5), std::llround(p.y * 1e5), std::llround(p.z * 1e5)};
                weld[i] = ids.emplace(key, static_cast<int>(ids.size())).first->second;
            }
            std::vector<Vec3> nsum(ids.size(), Vec3(0, 0, 0)), pos(ids.size());
            for (std::size_t i = 0; i < m.vertices.size(); ++i) {
                nsum[static_cast<std::size_t>(weld[i])] = nsum[static_cast<std::size_t>(weld[i])] + m.vertices[i].normal;
                pos[static_cast<std::size_t>(weld[i])] = m.vertices[i].position;
            }
            std::vector<double> ksum(ids.size(), 0.0);
            std::vector<int> kn(ids.size(), 0);
            for (std::size_t f = 0; f + 2 < m.indices.size(); f += 3)
                for (int e = 0; e < 3; ++e) {
                    const int a = weld[m.indices[f + static_cast<std::size_t>(e)]];
                    const int b = weld[m.indices[f + static_cast<std::size_t>((e + 1) % 3)]];
                    if (a == b) continue;
                    for (const auto& [p, q] : {std::pair<int, int>{a, b}, std::pair<int, int>{b, a}}) {
                        const Vec3 d = pos[static_cast<std::size_t>(q)] - pos[static_cast<std::size_t>(p)];
                        const double l2 = d.lengthSquared();
                        const Vec3& ns = nsum[static_cast<std::size_t>(p)];
                        if (l2 < 1e-14 || ns.lengthSquared() < 1e-20) continue;
                        ksum[static_cast<std::size_t>(p)] += -2.0 * dot(normalize(ns), d) / l2;
                        ++kn[static_cast<std::size_t>(p)];
                    }
                }
            for (std::size_t i = 0; i < m.vertices.size(); ++i) {
                const std::size_t w = static_cast<std::size_t>(weld[i]);
                k[i] = kn[w] > 0 ? static_cast<float>(ksum[w] / kn[w]) : 0.0f;
            }
        }
        tris.reserve(m.indices.size() / 3);
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            const uint32_t ia = m.indices[i], ib = m.indices[i + 1], ic = m.indices[i + 2];
            const Vertex& a = m.vertices[ia];
            const Vertex& b = m.vertices[ib];
            const Vertex& c = m.vertices[ic];
            if (cross(b.position - a.position, c.position - a.position).lengthSquared() < 1e-20) continue;
            Triangle t;
            t.v0 = a.position; t.v1 = b.position; t.v2 = c.position;
            t.n0 = a.normal; t.n1 = b.normal; t.n2 = c.normal;
            t.c0 = a.color; t.c1 = b.color; t.c2 = c.color;
            t.uv0[0] = k[ia]; t.uv1[0] = k[ib]; t.uv2[0] = k[ic];
            t.materialIndex = parts ? (*parts)[i / 3] : -1;
            tris.push_back(t);
        }
        if (!tris.empty()) bvh.build(tris);
    }
    bool cast(const Vec3& o, const Vec3& d, double tMax, HitRecord& rec) const {
        return !tris.empty() && bvh.intersect(tris, Ray(o, d), 1e-7, tMax, rec);
    }
    // the nearest hit OF `part` (any when < 0): a hit on another part is stepped past and the ray goes on
    bool castPart(const Vec3& o, const Vec3& d, double tMax, int part, HitRecord& rec) const {
        Vec3 from = o;
        double gone = 0;
        for (int step = 0; step < 16; ++step) {
            if (!cast(from, d, tMax - gone, rec)) return false;
            if (part < 0 || rec.materialIndex == part) { rec.t += gone; return true; }
            gone += rec.t + 1e-6;
            from = o + d * gone;
            if (gone >= tMax) return false;
        }
        return false;
    }
    // ...and a ray that runs exactly along an edge of the detailed mesh (a point on a grid line) can round outside
    // both triangles there: nudged a hair sideways and tried again
    bool castNudged(const Vec3& o, const Vec3& d, double tMax, const Vec3& t, const Vec3& b, int part, HitRecord& rec) const {
        if (castPart(o, d, tMax, part, rec)) return true;
        const double e = 1e-5;
        for (const Vec3& off : {t * e, b * e, t * -e, b * -e, (t + b) * e})
            if (castPart(o + off, d, tMax, part, rec)) return true;
        return false;
    }
};

// One point's bake
struct Sample {
    Vec3 tangentN, worldN, color;
    double ao = 1, curvature = 0, height = 0, thickness = 0;
};

}  // namespace

BakedMaps bakeMaps(const RenderMesh& low, const RenderMesh& high, const BakeOptions& opts) {
    BakedMaps out;
    const int S = std::max(1, opts.size);
    out.size = S;
    const std::size_t N = static_cast<std::size_t>(S) * static_cast<std::size_t>(S);
    out.normal.assign(N, Vec3(0, 0, 1));
    out.color.assign(N, Vec3(1, 1, 1));
    out.ao.assign(N, 1.0f);
    out.curvature.assign(N, 0.0f);
    out.height.assign(N, 0.0f);
    out.thickness.assign(N, 0.0f);
    out.covered.assign(N, 0);
    out.position.assign(N, Vec3(0, 0, 0));
    out.world.assign(N, Vec3(0, 0, 0));
    const bool byPart = opts.lowParts && opts.highParts && opts.lowParts->size() * 3 == low.indices.size() &&
                        opts.highParts->size() * 3 == high.indices.size();
    const HighSurface hs(high, byPart ? opts.highParts : nullptr, opts.curvature);
    const int ss = std::clamp(opts.samples, 1, 8);
    const double cage = std::max(1e-4, opts.cage), back = 1e-4;

    for (std::size_t f = 0; f + 2 < low.indices.size(); f += 3) {
        const Vertex* v[3] = {&low.vertices[low.indices[f]], &low.vertices[low.indices[f + 1]], &low.vertices[low.indices[f + 2]]};
        if (tagged(*v[0]) || tagged(*v[1]) || tagged(*v[2])) continue;
        const int part = byPart ? (*opts.lowParts)[f / 3] : -1;
        // the triangle in texel units (a texel x spans [x, x + 1))
        double qx[3], qy[3];
        for (int k = 0; k < 3; ++k) { qx[k] = v[k]->u * S; qy[k] = v[k]->v * S; }
        const double A = (qx[1] - qx[0]) * (qy[2] - qy[0]) - (qx[2] - qx[0]) * (qy[1] - qy[0]);
        if (std::fabs(A) < 1e-12) continue;
        const Vec3 e1 = v[1]->position - v[0]->position, e2 = v[2]->position - v[0]->position;
        const double du1 = v[1]->u - v[0]->u, dv1 = v[1]->v - v[0]->v, du2 = v[2]->u - v[0]->u, dv2 = v[2]->v - v[0]->v;
        const double det = du1 * dv2 - du2 * dv1;
        if (std::fabs(det) < 1e-20) continue;
        const Vec3 dPdu = (e1 * dv2 - e2 * dv1) * (1.0 / det);
        const int x0 = std::max(0, static_cast<int>(std::floor(std::min({qx[0], qx[1], qx[2]}))));
        const int x1 = std::min(S - 1, static_cast<int>(std::floor(std::max({qx[0], qx[1], qx[2]}))));
        const int y0 = std::max(0, static_cast<int>(std::floor(std::min({qy[0], qy[1], qy[2]}))));
        const int y1 = std::min(S - 1, static_cast<int>(std::floor(std::max({qy[0], qy[1], qy[2]}))));
        // one point at (sx, sy) in texel units: false when it is not on this triangle
        auto bakeAt = [&](double sx, double sy, std::size_t seedIdx, Sample& s, Vec3& pOut) {
            const double w1 = ((sx - qx[0]) * (qy[2] - qy[0]) - (qx[2] - qx[0]) * (sy - qy[0])) / A;
            const double w2 = ((qx[1] - qx[0]) * (sy - qy[0]) - (sx - qx[0]) * (qy[1] - qy[0])) / A;
            const double w0 = 1 - w1 - w2;
            const double tol = -1e-3;
            if (w0 < tol || w1 < tol || w2 < tol) return 0;   // 0: not here
            const Vec3 p = v[0]->position * w0 + v[1]->position * w1 + v[2]->position * w2;
            pOut = p;
            Vec3 n = v[0]->normal * w0 + v[1]->normal * w1 + v[2]->normal * w2;
            if (n.lengthSquared() < 1e-20) n = cross(e1, e2);
            n = normalize(n);
            // THE FRAME AS THE SHADER BUILDS IT: the vertices' tangents interpolated, square to n, B = cross(n, T)
            // (mesh.frag); dP/du stands in only where the mesh carries none
            const Vec3 Tv = v[0]->tangent * w0 + v[1]->tangent * w1 + v[2]->tangent * w2;
            Vec3 T = Tv.lengthSquared() > 1e-12 ? Tv : dPdu;
            T = T - n * dot(n, T);
            if (T.lengthSquared() < 1e-20) T = cross(std::fabs(n.x) < 0.6 ? Vec3(1, 0, 0) : Vec3(0, 1, 0), n);
            T = normalize(T);
            const Vec3 B = cross(n, T);
            // THE NEAREST SURFACE (of the part) to the point, out along n or in against it, facing the same way; each
            // cast starts a hair behind the point so a surface lying exactly on it (t = 0) counts
            HitRecord rec, outHit, inHit;
            const bool gotOut = hs.castNudged(p - n * back, n, cage + back, T, B, part, outHit);
            const bool gotIn = hs.castNudged(p + n * back, n * -1.0, cage + back, T, B, part, inHit);
            auto facing = [&](const HitRecord& h) { return dot(h.frontFace ? h.normal : h.normal * -1.0, n) > 0.2; };
            const bool okOut = gotOut && facing(outHit), okIn = gotIn && facing(inHit);
            if (!okOut && !okIn) return 1;   // 1: on the triangle, nothing found
            rec = (okOut && (!okIn || std::fabs(outHit.t - back) <= std::fabs(inHit.t - back))) ? outHit : inHit;
            const Vec3 Nw = rec.frontFace ? rec.normal : rec.normal * -1.0;
            s.worldN = Nw;
            s.tangentN = normalize(Vec3(dot(Nw, T), dot(Nw, B), dot(Nw, n)));
            s.color = rec.color;
            s.curvature = rec.u;
            s.height = dot(rec.point - p, n);
            if (opts.thickness) {
                HitRecord th;
                s.thickness = hs.cast(rec.point - Nw * 1e-4, Nw * -1.0, opts.thicknessMax, th) ? th.t : opts.thicknessMax;
            }
            if (opts.aoRays > 0) {
                // cosine-weighted rays round the detailed normal, seeded by the texel (deterministic)
                ProcRng rng(static_cast<uint32_t>(seedIdx) * 2654435761u + 7u);
                Vec3 t2 = normalize(std::fabs(Nw.x) < 0.6 ? cross(Vec3(1, 0, 0), Nw) : cross(Vec3(0, 1, 0), Nw));
                const Vec3 b2 = cross(Nw, t2);
                int open = 0;
                for (int r = 0; r < opts.aoRays; ++r) {
                    const double r1 = rng.next(), r2 = rng.next();
                    const double phi = 6.283185307179586 * r1, sr = std::sqrt(r2);
                    const Vec3 d = normalize(t2 * (std::cos(phi) * sr) + b2 * (std::sin(phi) * sr) + Nw * std::sqrt(1 - r2));
                    HitRecord occ;
                    if (!hs.cast(rec.point + Nw * 1e-4, d, opts.aoDistance, occ)) ++open;
                }
                s.ao = static_cast<double>(open) / opts.aoRays;
            }
            return 2;   // 2: found
        };
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(S) + static_cast<std::size_t>(x);
                Vec3 tn(0, 0, 0), wn(0, 0, 0), col(0, 0, 0), pos(0, 0, 0);
                double ao = 0, curv = 0, hgt = 0, thk = 0;
                int on = 0, found = 0;
                for (int sy = 0; sy < ss; ++sy)
                    for (int sx = 0; sx < ss; ++sx) {
                        Sample smp;
                        Vec3 p;
                        const int r = bakeAt(x + (sx + 0.5) / ss, y + (sy + 0.5) / ss, i * 64u + static_cast<std::size_t>(sy * ss + sx), smp, p);
                        if (r == 0) continue;
                        ++on;
                        pos = pos + p;
                        if (r < 2) continue;
                        ++found;
                        tn = tn + smp.tangentN; wn = wn + smp.worldN; col = col + smp.color;
                        ao += smp.ao; curv += smp.curvature; hgt += smp.height; thk += smp.thickness;
                    }
                if (on == 0) continue;
                if (out.covered[i] == 0) ++out.texels;
                out.covered[i] = 1;
                out.position[i] = pos * (1.0 / on);
                if (found == 0) continue;
                const double k = 1.0 / found;
                out.world[i] = normalize(wn);
                out.normal[i] = normalize(tn);
                out.color[i] = col * k;
                out.ao[i] = static_cast<float>(ao * k);
                out.curvature[i] = static_cast<float>(curv * k);
                out.height[i] = static_cast<float>(hgt * k);
                out.thickness[i] = static_cast<float>(thk * k);
            }
    }
    // (a texel on a shared edge is visited by both its triangles: counted once each way)
    for (std::size_t i = 0; i < N; ++i) if (out.covered[i] == 1 && out.world[i].lengthSquared() > 0) ++out.hits;
    // DILATE: each empty texel beside a filled one takes their mean, a ring at a time
    for (int ring = 0; ring < opts.dilate; ++ring) {
        std::vector<std::size_t> fill;
        for (int y = 0; y < S; ++y)
            for (int x = 0; x < S; ++x) {
                const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(S) + static_cast<std::size_t>(x);
                if (out.covered[i]) continue;
                bool next = false;
                for (int dy = -1; dy <= 1 && !next; ++dy)
                    for (int dx = -1; dx <= 1 && !next; ++dx) {
                        const int xx = x + dx, yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= S || yy >= S) continue;
                        if (out.covered[static_cast<std::size_t>(yy) * static_cast<std::size_t>(S) + static_cast<std::size_t>(xx)]) next = true;
                    }
                if (next) fill.push_back(i);
            }
        if (fill.empty()) break;
        for (std::size_t i : fill) {
            const int x = static_cast<int>(i % static_cast<std::size_t>(S)), y = static_cast<int>(i / static_cast<std::size_t>(S));
            Vec3 n(0, 0, 0), w(0, 0, 0), c(0, 0, 0);
            double a = 0, cu = 0, h = 0, t = 0;
            int k = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx, yy = y + dy;
                    if (xx < 0 || yy < 0 || xx >= S || yy >= S) continue;
                    const std::size_t j = static_cast<std::size_t>(yy) * static_cast<std::size_t>(S) + static_cast<std::size_t>(xx);
                    if (!out.covered[j] || out.covered[j] == 3) continue;   // (3: filled this ring -- not a source yet)
                    n = n + out.normal[j]; w = w + out.world[j]; c = c + out.color[j];
                    a += out.ao[j]; cu += out.curvature[j]; h += out.height[j]; t += out.thickness[j];
                    ++k;
                }
            if (k == 0) continue;
            out.normal[i] = n.lengthSquared() > 1e-20 ? normalize(n) : Vec3(0, 0, 1);
            out.world[i] = w.lengthSquared() > 1e-20 ? normalize(w) : Vec3(0, 0, 0);
            out.color[i] = c * (1.0 / k);
            out.ao[i] = static_cast<float>(a / k);
            out.curvature[i] = static_cast<float>(cu / k);
            out.height[i] = static_cast<float>(h / k);
            out.thickness[i] = static_cast<float>(t / k);
            out.covered[i] = 3;
        }
        for (std::size_t i : fill) if (out.covered[i] == 3) out.covered[i] = 2;
    }
    return out;
}

BakedModel bakeParts(const std::vector<BakePart>& parts, const UnwrapOptions& unwrap, BakeOptions bake) {
    BakedModel out;
    RenderMesh high;
    std::vector<int> lowParts, highParts;
    // the parts not baked keep their uvs: set aside, tagged so the unwrapper and the baker pass them by, put back after
    std::vector<std::pair<uint32_t, std::pair<float, float>>> keep;
    for (std::size_t p = 0; p < parts.size(); ++p) {
        const BakePart& part = parts[p];
        out.firstIndex.push_back(out.mesh.indices.size());
        const uint32_t base = static_cast<uint32_t>(out.mesh.vertices.size());
        MeshBuilder::append(out.mesh, part.low);
        for (std::size_t f = 0; f < part.low.indices.size() / 3; ++f) lowParts.push_back(static_cast<int>(p));
        if (!part.bake)
            for (uint32_t i = base; i < out.mesh.vertices.size(); ++i) {
                Vertex& v = out.mesh.vertices[i];
                if (v.u <= -999.0f) continue;   // (already a lens's tag)
                keep.push_back({i, {v.u, v.v}});
                v.u = -999.5f;
            }
        if (part.bake) {
            MeshBuilder::append(high, part.high);
            for (std::size_t f = 0; f < part.high.indices.size() / 3; ++f) highParts.push_back(static_cast<int>(p));
        }
    }
    UnwrapOptions uo = unwrap;
    uo.faceGroups = &lowParts;
    // (the unwrap keeps face order, so lowParts still lines up with the faces after it)
    const std::vector<uint32_t> before = out.mesh.indices;
    out.unwrap = unwrapMesh(out.mesh, uo);
    // the unwrap rebuilds the vertices: the kept parts' uvs go back by the faces' corners
    std::map<uint32_t, std::pair<float, float>> kept(keep.begin(), keep.end());
    for (std::size_t c = 0; c < before.size() && c < out.mesh.indices.size(); ++c) {
        const auto it = kept.find(before[c]);
        if (it == kept.end()) continue;
        Vertex& v = out.mesh.vertices[out.mesh.indices[c]];
        v.u = it->second.first;
        v.v = it->second.second;
    }
    // ...and the baker passes them by: a part id no high face has
    std::vector<int> bakeLow = lowParts;
    for (std::size_t f = 0; f < bakeLow.size(); ++f)
        if (!parts[static_cast<std::size_t>(bakeLow[f])].bake) bakeLow[f] = -2;
    bake.size = out.unwrap.size;
    bake.lowParts = &bakeLow;
    bake.highParts = &highParts;
    // (unbaked faces carry real uvs again: drawn into the atlas they would claim texels; bake a copy with them tagged)
    RenderMesh baking = out.mesh;
    for (std::size_t f = 0; f < bakeLow.size(); ++f)
        if (bakeLow[f] == -2)
            for (int k = 0; k < 3; ++k) baking.vertices[baking.indices[f * 3 + static_cast<std::size_t>(k)]].u = -999.5f;
    out.maps = bakeMaps(baking, high, bake);
    return out;
}

namespace {
uint8_t q8(double x) { return static_cast<uint8_t>(std::clamp(std::lround(x * 255.0), 0L, 255L)); }
}  // namespace

std::vector<uint8_t> normalMapBytes(const BakedMaps& m) {
    std::vector<uint8_t> b(m.normal.size() * 3);
    for (std::size_t i = 0; i < m.normal.size(); ++i) {
        b[i * 3] = q8(m.normal[i].x * 0.5 + 0.5); b[i * 3 + 1] = q8(m.normal[i].y * 0.5 + 0.5); b[i * 3 + 2] = q8(m.normal[i].z * 0.5 + 0.5);
    }
    return b;
}
std::vector<uint8_t> worldNormalMapBytes(const BakedMaps& m) {
    std::vector<uint8_t> b(m.world.size() * 3);
    for (std::size_t i = 0; i < m.world.size(); ++i) {
        b[i * 3] = q8(m.world[i].x * 0.5 + 0.5); b[i * 3 + 1] = q8(m.world[i].y * 0.5 + 0.5); b[i * 3 + 2] = q8(m.world[i].z * 0.5 + 0.5);
    }
    return b;
}
std::vector<uint8_t> colorMapBytes(const BakedMaps& m) {
    std::vector<uint8_t> b(m.color.size() * 3);
    for (std::size_t i = 0; i < m.color.size(); ++i) {
        b[i * 3] = q8(m.color[i].x); b[i * 3 + 1] = q8(m.color[i].y); b[i * 3 + 2] = q8(m.color[i].z);
    }
    return b;
}
std::vector<uint8_t> aoMapBytes(const BakedMaps& m) {
    std::vector<uint8_t> b(m.ao.size());
    for (std::size_t i = 0; i < m.ao.size(); ++i) b[i] = q8(m.ao[i]);
    return b;
}
std::vector<uint8_t> curvatureMapBytes(const BakedMaps& m, double radius) {
    std::vector<uint8_t> b(m.curvature.size());
    for (std::size_t i = 0; i < m.curvature.size(); ++i) b[i] = q8(0.5 + 0.5 * std::clamp(m.curvature[i] * radius, -1.0, 1.0));
    return b;
}
std::vector<uint8_t> heightMapBytes(const BakedMaps& m, double range) {
    std::vector<uint8_t> b(m.height.size());
    const double r = std::max(range, 1e-9);
    for (std::size_t i = 0; i < m.height.size(); ++i) b[i] = q8(0.5 + 0.5 * m.height[i] / r);
    return b;
}
std::vector<uint8_t> thicknessMapBytes(const BakedMaps& m, double max) {
    std::vector<uint8_t> b(m.thickness.size());
    const double r = std::max(max, 1e-9);
    for (std::size_t i = 0; i < m.thickness.size(); ++i) b[i] = q8(m.thickness[i] / r);
    return b;
}

}  // namespace engine
