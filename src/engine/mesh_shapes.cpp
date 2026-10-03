// The shape kit (MeshBuilder, mesh_builder.h): icosphere, noise displacement, plane cuts,
// faceting, volume normals, colour functions, tubes.
#include "mesh_builder.h"
#include "procgen/noise.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace engine {

RenderMesh MeshBuilder::icosphere(int subdiv) {
    const double t = (1.0 + std::sqrt(5.0)) / 2.0;
    const double raw[12][3] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                               {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    std::vector<Vec3> v;
    for (const auto& r : raw) v.push_back(normalize(Vec3(r[0], r[1], r[2])));
    std::vector<std::array<uint32_t, 3>> f = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
                                              {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8},
                                              {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (int k = 0; k < std::clamp(subdiv, 0, 5); ++k) {
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> mid;
        auto midpoint = [&](uint32_t a, uint32_t b) {
            const auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it = mid.find(key);
            if (it != mid.end()) return it->second;
            v.push_back(normalize((v[a] + v[b]) * 0.5));
            return mid[key] = static_cast<uint32_t>(v.size() - 1);
        };
        std::vector<std::array<uint32_t, 3>> nf;
        for (const auto& t3 : f) {
            const uint32_t a = midpoint(t3[0], t3[1]), b = midpoint(t3[1], t3[2]), c = midpoint(t3[2], t3[0]);
            nf.push_back({t3[0], a, c}); nf.push_back({t3[1], b, a}); nf.push_back({t3[2], c, b}); nf.push_back({a, b, c});
        }
        f = std::move(nf);
    }
    RenderMesh m;
    for (const Vec3& p : v) {
        Vertex vx(p, p, Vec3(1, 0, 0), 0.0f, 0.0f);
        vx.color = Vec3(1, 1, 1);
        m.vertices.push_back(vx);
    }
    for (const auto& t3 : f) {   // wound as emitTri winds a face whose normal points out
        const Vec3 &A = v[t3[0]], &B = v[t3[1]], &C = v[t3[2]];
        if (dot(cross(C - A, B - A), A + B + C) >= 0) m.indices.insert(m.indices.end(), {t3[0], t3[1], t3[2]});
        else m.indices.insert(m.indices.end(), {t3[0], t3[2], t3[1]});
    }
    return m;
}

void MeshBuilder::displaceNoise(RenderMesh& mesh, const Vec3& centre, double amp, double freq, uint32_t seed) {
    const Noise n(seed);
    const double off = (seed % 997) * 0.37;
    for (Vertex& v : mesh.vertices) {
        const Vec3 d = v.position - centre;
        const double len = d.length();
        if (len < 1e-12) continue;
        const Vec3 dir = d / len;
        const double k = n.noise3(dir.x * freq + off, dir.y * freq, dir.z * freq - off);
        v.position = centre + dir * (len * (1.0 + amp * k));
    }
}

void MeshBuilder::cutByPlane(RenderMesh& mesh, const Vec3& centre, const Vec3& nIn, double d) {
    const Vec3 n = normalize(nIn);
    for (Vertex& v : mesh.vertices) {
        const double k = dot(v.position - centre, n) - d;
        if (k > 0) v.position = v.position - n * k;
    }
}

void MeshBuilder::facet(RenderMesh& mesh, const Vec3& centre) {
    RenderMesh out;
    out.materialIndex = mesh.materialIndex;
    out.tangentIsData = mesh.tangentIsData;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const Vertex &a = mesh.vertices[mesh.indices[i]], &b = mesh.vertices[mesh.indices[i + 1]], &c = mesh.vertices[mesh.indices[i + 2]];
        Vec3 fn = cross(c.position - a.position, b.position - a.position);
        if (fn.lengthSquared() < 1e-20) continue;
        fn = normalize(fn);
        const Vec3 fc = (a.position + b.position + c.position) * (1.0 / 3.0);
        const bool flip = dot(fn, fc - centre) < 0;
        if (flip) fn = fn * -1.0;
        const uint32_t base = static_cast<uint32_t>(out.vertices.size());
        for (const Vertex* src : {&a, &b, &c}) {
            Vertex v = *src;
            v.normal = fn;
            out.vertices.push_back(v);
        }
        if (!flip) out.indices.insert(out.indices.end(), {base, base + 1, base + 2});
        else out.indices.insert(out.indices.end(), {base, base + 2, base + 1});
    }
    mesh = std::move(out);
}

void MeshBuilder::leanNormals(RenderMesh& mesh, const Vec3& c, const Vec3& r, double t) {
    for (Vertex& v : mesh.vertices) {
        const Vec3 q((v.position.x - c.x) / (r.x * r.x), (v.position.y - c.y) / (r.y * r.y), (v.position.z - c.z) / (r.z * r.z));
        if (q.lengthSquared() < 1e-20) continue;
        v.normal = normalize(v.normal * (1.0 - t) + normalize(q) * t);
    }
}

void MeshBuilder::deform(RenderMesh& mesh, const std::function<Vec3(const Vec3&)>& fn) {
    for (Vertex& v : mesh.vertices) v.position = fn(v.position);
}

void MeshBuilder::colorBy(RenderMesh& mesh, const std::function<Vec3(const Vertex&)>& fn) {
    for (Vertex& v : mesh.vertices) v.color = fn(v);
}

RenderMesh MeshBuilder::crossCards(double width, double height, int planes, const Vec3& normalIn) {
    RenderMesh m;
    const Vec3 n = normalize(normalIn);
    const int k = std::max(1, planes);
    for (int p = 0; p < k; ++p) {
        const double a = 3.141592653589793 * p / k;   // half a turn covers every direction (two-sided)
        const Vec3 side(std::cos(a) * width * 0.5, 0.0, std::sin(a) * width * 0.5);
        const Vec3 tan = normalize(side);
        const uint32_t base = static_cast<uint32_t>(m.vertices.size());
        auto put = [&](const Vec3& pos, float u, float v) {
            Vertex vx(pos, n, tan, u, v);
            vx.color = Vec3(1, 1, 1);
            m.vertices.push_back(vx);
        };
        put(Vec3(0, 0, 0) - side, 0.0f, 0.0f);
        put(Vec3(0, 0, 0) + side, 1.0f, 0.0f);
        put(Vec3(0, height, 0) + side, 1.0f, 1.0f);
        put(Vec3(0, height, 0) - side, 0.0f, 1.0f);
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return m;
}

RenderMesh MeshBuilder::ribbon(const std::vector<Vec3>& pts, const std::vector<double>& hw, const Vec3& upIn,
                               const std::vector<Vec3>& colours) {
    RenderMesh m;
    const std::size_t n = pts.size();
    if (n < 2 || hw.size() < n) return m;
    const Vec3 up = normalize(upIn);
    auto flat = [&](Vec3 v) { return v - up * dot(v, up); };
    double along = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (i > 0) along += (pts[i] - pts[i - 1]).length();
        // The side direction from a tangent averaged over a few points (two neighbours flip on a
        // kink), flattened against `up`.
        const std::size_t a = i >= 3 ? i - 3 : 0, b = std::min(n - 1, i + 3);
        Vec3 t = flat(pts[b] - pts[a]);
        if (t.lengthSquared() < 1e-18) t = flat((i + 1 < n ? pts[i + 1] : pts[i]) - (i > 0 ? pts[i - 1] : pts[i]));
        if (t.lengthSquared() < 1e-18) t = Vec3(1, 0, 0);
        t = normalize(t);
        const Vec3 side = normalize(cross(up, t));
        // No wider than the bend is round: on a curve of radius R a half-width past R folds the
        // strip over itself (a fan of crossed triangles).
        double halfW = hw[i];
        if (i > 0 && i + 1 < n) {
            const Vec3 d0 = flat(pts[i] - pts[i - 1]), d1 = flat(pts[i + 1] - pts[i]);
            const double l0 = d0.length(), l1 = d1.length();
            if (l0 > 1e-9 && l1 > 1e-9) {
                const double ang = std::acos(std::clamp(dot(d0 / l0, d1 / l1), -1.0, 1.0));
                if (ang > 1e-4) halfW = std::min(halfW, 0.85 * 0.5 * (l0 + l1) / ang);
            }
        }
        const Vec3 col = i < colours.size() ? colours[i] : Vec3(1, 1, 1);
        for (int k = 0; k < 2; ++k) {
            Vertex v(pts[i] + side * (k == 0 ? -halfW : halfW), up, t, static_cast<float>(k), static_cast<float>(along));
            v.color = col;
            m.vertices.push_back(v);
        }
    }
    for (std::size_t i = 0; i + 1 < n; ++i) {
        const uint32_t a = static_cast<uint32_t>(2 * i), b = a + 1, c = a + 2, d = a + 3;
        triFacing(m, a, b, d, up);
        triFacing(m, a, d, c, up);
    }
    return m;
}

uint32_t MeshBuilder::vertex(RenderMesh& mesh, const Vec3& p, const Vec3& n, const Vec3& color) {
    Vertex v(p, n.lengthSquared() > 1e-20 ? normalize(n) : Vec3(0, 1, 0), Vec3(1, 0, 0), 0.0f, 0.0f);
    v.color = color;
    mesh.vertices.push_back(v);
    return static_cast<uint32_t>(mesh.vertices.size() - 1);
}

void MeshBuilder::triFacing(RenderMesh& mesh, uint32_t a, uint32_t b, uint32_t c, const Vec3& out) {
    const Vec3 &A = mesh.vertices[a].position, &B = mesh.vertices[b].position, &C = mesh.vertices[c].position;
    if (dot(cross(C - A, B - A), out) >= 0) mesh.indices.insert(mesh.indices.end(), {a, b, c});
    else mesh.indices.insert(mesh.indices.end(), {a, c, b});
}

RenderMesh MeshBuilder::tube(const std::vector<Vec3>& pts, const std::vector<double>& radii, int sides,
                             const std::vector<Vec3>& colours) {
    RenderMesh m;
    const std::size_t n = pts.size();
    sides = std::max(3, sides);
    if (n < 2 || radii.size() < n) return m;
    std::vector<uint32_t> base(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 d = normalize(i + 1 < n ? pts[i + 1] - pts[i] : pts[i] - pts[i - 1]);
        const Vec3 a = std::fabs(d.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        const Vec3 u = normalize(cross(d, a)), w = cross(d, u);
        base[i] = static_cast<uint32_t>(m.vertices.size());
        for (int k = 0; k < sides; ++k) {
            const double t = 6.283185307179586 * k / sides;
            const Vec3 r = u * std::cos(t) + w * std::sin(t);
            Vertex vx(pts[i] + r * radii[i], r, d, 0.0f, static_cast<float>(i) / static_cast<float>(n - 1));
            vx.color = i < colours.size() ? colours[i] : Vec3(1, 1, 1);
            m.vertices.push_back(vx);
        }
    }
    for (std::size_t i = 0; i + 1 < n; ++i)
        for (int k = 0; k < sides; ++k) {
            const uint32_t a = base[i] + k, b = base[i] + (k + 1) % sides, c = base[i + 1] + k, d = base[i + 1] + (k + 1) % sides;
            const Vec3 out = m.vertices[a].normal;
            for (const auto& t3 : {std::array<uint32_t, 3>{a, b, d}, std::array<uint32_t, 3>{a, d, c}}) {
                const Vec3 &A = m.vertices[t3[0]].position, &B = m.vertices[t3[1]].position, &C = m.vertices[t3[2]].position;
                if (dot(cross(C - A, B - A), out) >= 0) m.indices.insert(m.indices.end(), {t3[0], t3[1], t3[2]});
                else m.indices.insert(m.indices.end(), {t3[0], t3[2], t3[1]});
            }
        }
    return m;
}

RenderMesh MeshBuilder::roundedBox(const Vec3& size, double radius, int segs, double uvPerMetre) {
    RenderMesh m;
    const Vec3 h = size * 0.5;
    const double r = std::max(0.0, std::min(radius, std::min({h.x, h.y, h.z}) * 0.999));
    segs = std::max(1, segs);
    // The coordinates along one axis: the flat middle plus `segs` arc steps toward each end.
    auto axis = [&](double e) {
        std::vector<double> c;
        if (r <= 1e-9) return std::vector<double>{-e, e};
        for (int k = segs; k >= 1; --k) c.push_back(-(e - r) - r * std::sin(1.5707963267948966 * k / segs));
        c.push_back(-(e - r));
        c.push_back(e - r);
        for (int k = 1; k <= segs; ++k) c.push_back((e - r) + r * std::sin(1.5707963267948966 * k / segs));
        return c;
    };
    const std::vector<double> ax = axis(h.x), ay = axis(h.y), az = axis(h.z);
    const Vec3 inner(h.x - r, h.y - r, h.z - r);
    // Each face: its axis, sign, and the two in-plane axes (u, v).
    struct Face { int n; double sgn; int u, v; };
    const Face faces[6] = {{0, 1, 2, 1}, {0, -1, 2, 1}, {1, 1, 0, 2}, {1, -1, 0, 2}, {2, 1, 0, 1}, {2, -1, 0, 1}};
    const std::vector<double>* lists[3] = {&ax, &ay, &az};
    const double half[3] = {h.x, h.y, h.z};
    for (const Face& f : faces) {
        const std::vector<double>& lu = *lists[f.u];
        const std::vector<double>& lv = *lists[f.v];
        const uint32_t base = static_cast<uint32_t>(m.vertices.size());
        Vec3 outN(0, 0, 0);
        (f.n == 0 ? outN.x : f.n == 1 ? outN.y : outN.z) = f.sgn;
        for (double cv : lv)
            for (double cu : lu) {
                double p[3];
                p[f.n] = f.sgn * half[f.n]; p[f.u] = cu; p[f.v] = cv;
                const double q[3] = {std::clamp(p[0], -inner.x, inner.x), std::clamp(p[1], -inner.y, inner.y),
                                     std::clamp(p[2], -inner.z, inner.z)};
                Vec3 d(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
                const double l = d.length();
                d = l > 1e-12 ? d * (1.0 / l) : outN;
                const Vec3 pos = r > 1e-9 ? Vec3(q[0], q[1], q[2]) + d * r : Vec3(p[0], p[1], p[2]);
                Vertex vx(pos, d, Vec3(f.u == 0 ? 1 : 0, f.u == 1 ? 1 : 0, f.u == 2 ? 1 : 0),
                          static_cast<float>(cu * uvPerMetre), static_cast<float>(cv * uvPerMetre));
                vx.color = Vec3(1, 1, 1);
                m.vertices.push_back(vx);
            }
        const uint32_t nu = static_cast<uint32_t>(lu.size()), nv = static_cast<uint32_t>(lv.size());
        for (uint32_t j = 0; j + 1 < nv; ++j)
            for (uint32_t i = 0; i + 1 < nu; ++i) {
                const uint32_t a = base + j * nu + i, b = a + 1, c = a + nu, d = c + 1;
                triFacing(m, a, b, d, outN);
                triFacing(m, a, d, c, outN);
            }
    }
    return m;
}

RenderMesh MeshBuilder::lathe(const std::vector<std::pair<double, double>>& prof, int segs) {
    RenderMesh m;
    const std::size_t n = prof.size();
    struct P2 { double x, y; };
    std::vector<P2> profile(n);
    for (std::size_t i = 0; i < n; ++i) profile[i] = {prof[i].first, prof[i].second};
    auto len = [](const P2& a, const P2& b) { return std::hypot(b.x - a.x, b.y - a.y); };
    segs = std::max(3, segs);
    if (n < 2) return m;
    std::vector<double> vlen(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) vlen[i] = vlen[i - 1] + len(profile[i - 1], profile[i]);
    std::vector<uint32_t> base(n);
    for (std::size_t i = 0; i < n; ++i) {
        // The profile's outward normal in (r, y): its tangent turned a right angle, averaged over both neighbours.
        const P2& t1 = profile[std::min(i + 1, n - 1)];
        const P2& t0 = profile[i > 0 ? i - 1 : 0];
        double pnx = t1.y - t0.y, pny = -(t1.x - t0.x);
        const double pl = std::hypot(pnx, pny);
        if (pl > 1e-12) { pnx /= pl; pny /= pl; } else { pnx = 1; pny = 0; }
        base[i] = static_cast<uint32_t>(m.vertices.size());
        for (int k = 0; k <= segs; ++k) {
            const double a = 6.283185307179586 * k / segs, ca = std::cos(a), sa = std::sin(a);
            Vertex vx(Vec3(profile[i].x * ca, profile[i].y, profile[i].x * sa), normalize(Vec3(pnx * ca, pny, pnx * sa)),
                      Vec3(-sa, 0, ca), static_cast<float>(static_cast<double>(k) / segs), static_cast<float>(vlen[i]));
            vx.color = Vec3(1, 1, 1);
            m.vertices.push_back(vx);
        }
    }
    for (std::size_t i = 0; i + 1 < n; ++i)
        for (int k = 0; k < segs; ++k) {
            const uint32_t a = base[i] + k, b = a + 1, c = base[i + 1] + k, d = c + 1;
            const Vec3 out = normalize(m.vertices[a].normal + m.vertices[d].normal);
            if ((m.vertices[a].position - m.vertices[b].position).length() > 1e-9) triFacing(m, a, b, d, out);
            if ((m.vertices[c].position - m.vertices[d].position).length() > 1e-9) triFacing(m, a, d, c, out);
        }
    return m;
}

}  // namespace engine
