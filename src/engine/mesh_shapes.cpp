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

}  // namespace engine
