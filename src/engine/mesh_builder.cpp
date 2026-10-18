#include <unordered_map>
#include "mesh_builder.h"
#include "../renderer/cube_faces.h"   // cubeFaceDirection — the cubemap convention
#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>

namespace engine {

RenderMesh MeshBuilder::shape(const std::string& name, Vec3 size) {
    float x = static_cast<float>(size.x);
    float y = static_cast<float>(size.y);
    if (name == "box")      return box(size);
    if (name == "sphere")   return sphere(x);
    if (name == "cylinder") return cylinder(x, y);
    if (name == "plane")    return plane(x, y);
    if (name == "cone")     return cone(x, y);
    if (name == "wedge")    return wedge(size);
    if (name == "torus")    return torus(x, y);
    if (name == "capsule")  return capsule(x, y);
    return RenderMesh{};
}

RenderMesh MeshBuilder::box(Vec3 size) {
    RenderMesh mesh;
    float hx = static_cast<float>(size.x * 0.5);
    float hy = static_cast<float>(size.y * 0.5);
    float hz = static_cast<float>(size.z * 0.5);

    struct Face { Vec3 normal; Vec3 tangent; Vec3 verts[4]; float u[4]; float v[4]; };
    Face faces[] = {
        {Vec3(0,0,-1), Vec3(1,0,0),
         {Vec3(-hx,-hy,-hz), Vec3(hx,-hy,-hz), Vec3(hx,hy,-hz), Vec3(-hx,hy,-hz)},
         {0,1,1,0}, {0,0,1,1}},
        {Vec3(0,0,1), Vec3(-1,0,0),
         {Vec3(hx,-hy,hz), Vec3(-hx,-hy,hz), Vec3(-hx,hy,hz), Vec3(hx,hy,hz)},
         {0,1,1,0}, {0,0,1,1}},
        {Vec3(-1,0,0), Vec3(0,0,-1),
         {Vec3(-hx,-hy,hz), Vec3(-hx,-hy,-hz), Vec3(-hx,hy,-hz), Vec3(-hx,hy,hz)},
         {0,1,1,0}, {0,0,1,1}},
        {Vec3(1,0,0), Vec3(0,0,1),
         {Vec3(hx,-hy,-hz), Vec3(hx,-hy,hz), Vec3(hx,hy,hz), Vec3(hx,hy,-hz)},
         {0,1,1,0}, {0,0,1,1}},
        {Vec3(0,-1,0), Vec3(1,0,0),
         {Vec3(-hx,-hy,hz), Vec3(hx,-hy,hz), Vec3(hx,-hy,-hz), Vec3(-hx,-hy,-hz)},
         {0,1,1,0}, {0,0,1,1}},
        {Vec3(0,1,0), Vec3(1,0,0),
         {Vec3(-hx,hy,-hz), Vec3(hx,hy,-hz), Vec3(hx,hy,hz), Vec3(-hx,hy,hz)},
         {0,1,1,0}, {0,0,1,1}},
    };

    for (auto& f : faces) {
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int i = 0; i < 4; i++)
            mesh.vertices.push_back(Vertex(f.verts[i], f.normal, f.tangent, f.u[i], f.v[i]));
        mesh.indices.insert(mesh.indices.end(),
            {base, base+1, base+2, base, base+2, base+3});
    }
    return mesh;
}

RenderMesh MeshBuilder::sphere(float radius, int stacks, int slices) {
    RenderMesh mesh;
    for (int i = 0; i <= stacks; i++) {
        float theta = static_cast<float>(PI) * i / stacks;
        for (int j = 0; j <= slices; j++) {
            float phi = 2.0f * static_cast<float>(PI) * j / slices;
            float x = radius * std::sin(theta) * std::cos(phi);
            float y = radius * std::cos(theta);
            float z = radius * std::sin(theta) * std::sin(phi);
            Vec3 pos(x, y, z);
            Vec3 norm = normalize(pos);
            Vec3 tan = normalize(Vec3(-std::sin(phi), 0, std::cos(phi)));
            float u = static_cast<float>(j) / slices;
            float v = static_cast<float>(i) / stacks;
            mesh.vertices.push_back(Vertex(pos, norm, tan, u, v));
        }
    }
    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            uint32_t a = i * (slices + 1) + j;
            uint32_t b = a + slices + 1;
            mesh.indices.insert(mesh.indices.end(),
                {a, b, a+1, b, b+1, a+1});
        }
    }
    dropDegenerate(mesh);
    return mesh;
}

RenderMesh MeshBuilder::cubeSphere(float radius, int faceRes, bool warp) {
    RenderMesh mesh;
    if (faceRes < 1) faceRes = 1;

    // Weld vertices coincident across the shared face seams (and the 8 corners each
    // shared by 3 faces) so the result is ONE closed manifold, not 6 loose sheets.
    // Quantise position to a grid far finer than the vertex spacing (smallest
    // ~R·(π/2)/faceRes) but coarser than the cross-face float error — so seam pairs
    // land on the same key while genuinely distinct grid points do not.
    const double eps = std::max(1e-6, static_cast<double>(radius) * 1e-4);
    struct Key {
        long long x, y, z;
        bool operator<(const Key& o) const {
            return x != o.x ? x < o.x : (y != o.y ? y < o.y : z < o.z);
        }
    };
    std::map<Key, uint32_t> weld;

    auto tangentFor = [](const Vec3& dir) {
        Vec3 t = cross(Vec3(0, 1, 0), dir);
        return t.lengthSquared() > 1e-12 ? normalize(t) : Vec3(1, 0, 0);
    };
    auto addVert = [&](const Vec3& dir) -> uint32_t {
        Vec3 pos = dir * static_cast<double>(radius);
        Key k{std::llround(pos.x / eps), std::llround(pos.y / eps),
              std::llround(pos.z / eps)};
        auto it = weld.find(k);
        if (it != weld.end()) return it->second;
        uint32_t idx = static_cast<uint32_t>(mesh.vertices.size());
        // Equirectangular UV as a sane default; per-face cube maps are sampled by
        // direction in later phases (planet plan), so this is not on the hot path.
        double y = std::max(-1.0, std::min(1.0, dir.y));
        float u = static_cast<float>(0.5 + std::atan2(dir.z, dir.x) / (2.0 * PI));
        float v = static_cast<float>(0.5 - std::asin(y) / PI);
        // Normal is the outward radial (exact for the undisplaced sphere).
        mesh.vertices.push_back(Vertex(pos, dir, tangentFor(dir), u, v));
        weld.emplace(k, idx);
        return idx;
    };

    // COBE tangent adjustment: uniform grid steps map to tan(s·π/4) in cube space,
    // so after projection the angular spacing (and thus triangle area) evens out.
    auto warpCoord = [warp](double s) { return warp ? std::tan(s * PI * 0.25) : s; };
    auto faceDir = [&](int face, int i, int j) {
        double s = warpCoord(2.0 * i / faceRes - 1.0);
        double t = warpCoord(2.0 * j / faceRes - 1.0);
        // cube_faces.h takes (u, v) in [0,1]; feed the warped cube coords back so the
        // mesh directions match the per-face cubemap texel convention exactly.
        return cubeFaceDirection(face, (s + 1.0) * 0.5, (t + 1.0) * 0.5);
    };

    // Emit a triangle wound so its front face (outward normal cross(c-a,b-a), the
    // engine's clockwise-front convention) points away from the origin.
    auto addTri = [&](uint32_t a, uint32_t b, uint32_t c) {
        const Vec3& pa = mesh.vertices[a].position;
        const Vec3& pb = mesh.vertices[b].position;
        const Vec3& pc = mesh.vertices[c].position;
        if (dot(cross(pc - pa, pb - pa), pa) >= 0)
            mesh.indices.insert(mesh.indices.end(), {a, b, c});
        else
            mesh.indices.insert(mesh.indices.end(), {a, c, b});
    };

    const int stride = faceRes + 1;
    std::vector<uint32_t> grid(static_cast<size_t>(stride) * stride);
    for (int face = 0; face < 6; face++) {
        for (int j = 0; j <= faceRes; j++)
            for (int i = 0; i <= faceRes; i++)
                grid[j * stride + i] = addVert(faceDir(face, i, j));
        for (int j = 0; j < faceRes; j++) {
            for (int i = 0; i < faceRes; i++) {
                uint32_t a = grid[j * stride + i];
                uint32_t b = grid[j * stride + i + 1];
                uint32_t c = grid[(j + 1) * stride + i + 1];
                uint32_t d = grid[(j + 1) * stride + i];
                addTri(a, b, c);
                addTri(a, c, d);
            }
        }
    }
    return mesh;
}

RenderMesh MeshBuilder::cylinder(float radius, float height, int slices) {
    RenderMesh mesh;
    float halfH = height * 0.5f;

    for (int i = 0; i <= slices; i++) {
        float phi = 2.0f * static_cast<float>(PI) * i / slices;
        float x = radius * std::cos(phi);
        float z = radius * std::sin(phi);
        Vec3 normal = normalize(Vec3(x, 0, z));
        Vec3 tan = normalize(Vec3(-std::sin(phi), 0, std::cos(phi)));
        float u = static_cast<float>(i) / slices;
        mesh.vertices.push_back(Vertex(Vec3(x, -halfH, z), normal, tan, u, 0));
        mesh.vertices.push_back(Vertex(Vec3(x, halfH, z), normal, tan, u, 1));
    }
    for (int i = 0; i < slices; i++) {
        uint32_t a = i * 2, b = a + 1, c = a + 2, d = a + 3;
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
    }

    Vec3 capTan(1, 0, 0);

    // Top cap
    uint32_t topCenter = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(Vertex(Vec3(0, halfH, 0), Vec3(0, 1, 0), capTan, 0.5f, 0.5f));
    for (int i = 0; i <= slices; i++) {
        float phi = 2.0f * static_cast<float>(PI) * i / slices;
        float x = radius * std::cos(phi);
        float z = radius * std::sin(phi);
        mesh.vertices.push_back(Vertex(
            Vec3(x, halfH, z), Vec3(0, 1, 0), capTan,
            0.5f + 0.5f * std::cos(phi), 0.5f + 0.5f * std::sin(phi)));
    }
    // Clockwise-front so the +Y cap faces up (a fan around the centre).
    for (int i = 0; i < slices; i++)
        mesh.indices.insert(mesh.indices.end(),
            {topCenter, topCenter + 1 + static_cast<uint32_t>(i),
             topCenter + 1 + static_cast<uint32_t>(i + 1)});

    // Bottom cap
    uint32_t botCenter = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(Vertex(Vec3(0, -halfH, 0), Vec3(0, -1, 0), capTan, 0.5f, 0.5f));
    for (int i = 0; i <= slices; i++) {
        float phi = 2.0f * static_cast<float>(PI) * i / slices;
        float x = radius * std::cos(phi);
        float z = radius * std::sin(phi);
        mesh.vertices.push_back(Vertex(
            Vec3(x, -halfH, z), Vec3(0, -1, 0), capTan,
            0.5f + 0.5f * std::cos(phi), 0.5f - 0.5f * std::sin(phi)));
    }
    // Clockwise-front for the -Y cap (reverse of the top fan).
    for (int i = 0; i < slices; i++)
        mesh.indices.insert(mesh.indices.end(),
            {botCenter, botCenter + 1 + static_cast<uint32_t>(i + 1),
             botCenter + 1 + static_cast<uint32_t>(i)});

    dropDegenerate(mesh);
    return mesh;
}

RenderMesh MeshBuilder::plane(float width, float depth) {
    RenderMesh mesh;
    float hw = width * 0.5f;
    float hd = depth * 0.5f;
    Vec3 normal(0, 1, 0);
    Vec3 tan(1, 0, 0);
    mesh.vertices = {
        Vertex(Vec3(-hw, 0, -hd), normal, tan, 0, 0),
        Vertex(Vec3( hw, 0, -hd), normal, tan, 1, 0),
        Vertex(Vec3( hw, 0,  hd), normal, tan, 1, 1),
        Vertex(Vec3(-hw, 0,  hd), normal, tan, 0, 1),
    };
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

RenderMesh MeshBuilder::cone(float radius, float height, int slices) {
    RenderMesh mesh;
    float halfH = height * 0.5f;
    float slopeLen = std::sqrt(radius * radius + height * height);
    float ny = radius / slopeLen;
    float nr = height / slopeLen;

    // Apex + side vertices
    for (int i = 0; i < slices; i++) {
        float phi0 = 2.0f * static_cast<float>(PI) * i / slices;
        float phi1 = 2.0f * static_cast<float>(PI) * (i + 1) / slices;
        float midPhi = (phi0 + phi1) * 0.5f;

        Vec3 n(nr * std::cos(midPhi), ny, nr * std::sin(midPhi));
        n = normalize(n);
        Vec3 tan = normalize(Vec3(-std::sin(midPhi), 0, std::cos(midPhi)));

        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back(Vertex(Vec3(0, halfH, 0), n, tan,
            static_cast<float>(i + 0.5f) / slices, 0));
        mesh.vertices.push_back(Vertex(
            Vec3(radius * std::cos(phi0), -halfH, radius * std::sin(phi0)), n, tan,
            static_cast<float>(i) / slices, 1));
        mesh.vertices.push_back(Vertex(
            Vec3(radius * std::cos(phi1), -halfH, radius * std::sin(phi1)), n, tan,
            static_cast<float>(i + 1) / slices, 1));
        mesh.indices.insert(mesh.indices.end(), {base, base+1, base+2});
    }

    // Bottom cap
    Vec3 capTan(1, 0, 0);
    uint32_t botCenter = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(Vertex(Vec3(0, -halfH, 0), Vec3(0, -1, 0), capTan, 0.5f, 0.5f));
    for (int i = 0; i <= slices; i++) {
        float phi = 2.0f * static_cast<float>(PI) * i / slices;
        mesh.vertices.push_back(Vertex(
            Vec3(radius * std::cos(phi), -halfH, radius * std::sin(phi)),
            Vec3(0, -1, 0), capTan,
            0.5f + 0.5f * std::cos(phi), 0.5f - 0.5f * std::sin(phi)));
    }
    // Clockwise-front for the -Y cap so it faces down.
    for (int i = 0; i < slices; i++)
        mesh.indices.insert(mesh.indices.end(),
            {botCenter, botCenter + 1 + static_cast<uint32_t>(i + 1),
             botCenter + 1 + static_cast<uint32_t>(i)});

    dropDegenerate(mesh);
    return mesh;
}

RenderMesh MeshBuilder::wedge(Vec3 size) {
    RenderMesh mesh;
    float hx = static_cast<float>(size.x * 0.5);
    float hy = static_cast<float>(size.y);
    float hz = static_cast<float>(size.z * 0.5);

    // Vertices: bottom quad + top edge (triangular cross-section in YZ)
    //   bottom: (-hx,0,-hz), (hx,0,-hz), (hx,0,hz), (-hx,0,hz)
    //   top:    (-hx,hy,-hz), (hx,hy,-hz)
    // The slope face goes from the top edge down to the bottom back edge.

    Vec3 bl0(-hx, 0, -hz), bl1(hx, 0, -hz);
    Vec3 bl2(hx, 0, hz),   bl3(-hx, 0, hz);
    Vec3 tl0(-hx, hy, -hz), tl1(hx, hy, -hz);

    auto addQuad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n) {
        Vec3 tan = normalize(b - a);
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back(Vertex(a, n, tan, 0, 0));
        mesh.vertices.push_back(Vertex(b, n, tan, 1, 0));
        mesh.vertices.push_back(Vertex(c, n, tan, 1, 1));
        mesh.vertices.push_back(Vertex(d, n, tan, 0, 1));
        mesh.indices.insert(mesh.indices.end(),
            {base, base+1, base+2, base, base+2, base+3});
    };

    auto addTri = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 n) {
        Vec3 tan = normalize(b - a);
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back(Vertex(a, n, tan, 0, 0));
        mesh.vertices.push_back(Vertex(b, n, tan, 1, 0));
        mesh.vertices.push_back(Vertex(c, n, tan, 0.5f, 1));
        mesh.indices.insert(mesh.indices.end(), {base, base+1, base+2});
    };

    // Bottom face (y=0, normal down)
    addQuad(bl3, bl2, bl1, bl0, Vec3(0, -1, 0));

    // Back face (z=-hz, vertical)
    addQuad(bl0, bl1, tl1, tl0, Vec3(0, 0, -1));

    // Slope face (from top edge down to bottom back edge)
    Vec3 slopeEdge1 = tl0 - tl1;
    Vec3 slopeEdge2 = bl2 - tl1;
    Vec3 slopeN = normalize(cross(slopeEdge1, slopeEdge2));
    addQuad(tl0, tl1, bl2, bl3, slopeN);

    // Left triangle (-x side)
    addTri(bl0, tl0, bl3, Vec3(-1, 0, 0));

    // Right triangle (+x side)
    addTri(bl1, bl2, tl1, Vec3(1, 0, 0));

    return mesh;
}

RenderMesh MeshBuilder::torus(float majorRadius, float minorRadius,
                              int majorSegments, int minorSegments) {
    RenderMesh mesh;
    for (int i = 0; i <= majorSegments; i++) {
        float theta = 2.0f * static_cast<float>(PI) * i / majorSegments;
        float ct = std::cos(theta), st = std::sin(theta);
        for (int j = 0; j <= minorSegments; j++) {
            float phi = 2.0f * static_cast<float>(PI) * j / minorSegments;
            float cp = std::cos(phi), sp = std::sin(phi);

            float x = (majorRadius + minorRadius * cp) * ct;
            float y = minorRadius * sp;
            float z = (majorRadius + minorRadius * cp) * st;

            Vec3 center(majorRadius * ct, 0, majorRadius * st);
            Vec3 pos(x, y, z);
            Vec3 norm = normalize(pos - center);
            Vec3 tan = normalize(Vec3(-st, 0, ct));

            float u = static_cast<float>(i) / majorSegments;
            float v = static_cast<float>(j) / minorSegments;
            mesh.vertices.push_back(Vertex(pos, norm, tan, u, v));
        }
    }
    for (int i = 0; i < majorSegments; i++) {
        for (int j = 0; j < minorSegments; j++) {
            uint32_t a = i * (minorSegments + 1) + j;
            uint32_t b = a + minorSegments + 1;
            mesh.indices.insert(mesh.indices.end(),
                {a, b, a+1, b, b+1, a+1});
        }
    }
    dropDegenerate(mesh);
    return mesh;
}

RenderMesh MeshBuilder::capsule(float radius, float height, int stacks, int slices) {
    RenderMesh mesh;
    float halfH = height * 0.5f;
    int halfStacks = stacks / 2;

    // Top hemisphere
    for (int i = 0; i <= halfStacks; i++) {
        float theta = static_cast<float>(PI) * 0.5f * i / halfStacks;
        float r = radius * std::sin(theta);
        float y = halfH + radius * std::cos(theta);
        for (int j = 0; j <= slices; j++) {
            float phi = 2.0f * static_cast<float>(PI) * j / slices;
            Vec3 pos(r * std::cos(phi), y, r * std::sin(phi));
            Vec3 norm = normalize(Vec3(pos.x, pos.y - halfH, pos.z));
            Vec3 tan = normalize(Vec3(-std::sin(phi), 0, std::cos(phi)));
            float u = static_cast<float>(j) / slices;
            float v = static_cast<float>(i) / (stacks + 1);
            mesh.vertices.push_back(Vertex(pos, norm, tan, u, v));
        }
    }

    // Bottom hemisphere
    for (int i = 0; i <= halfStacks; i++) {
        float theta = static_cast<float>(PI) * 0.5f + static_cast<float>(PI) * 0.5f * i / halfStacks;
        float r = radius * std::sin(theta);
        float y = -halfH + radius * std::cos(theta);
        for (int j = 0; j <= slices; j++) {
            float phi = 2.0f * static_cast<float>(PI) * j / slices;
            Vec3 pos(r * std::cos(phi), y, r * std::sin(phi));
            Vec3 norm = normalize(Vec3(pos.x, pos.y + halfH, pos.z));
            Vec3 tan = normalize(Vec3(-std::sin(phi), 0, std::cos(phi)));
            float u = static_cast<float>(j) / slices;
            float v = static_cast<float>(halfStacks + 1 + i) / (stacks + 1);
            mesh.vertices.push_back(Vertex(pos, norm, tan, u, v));
        }
    }

    int totalRings = 2 * (halfStacks + 1);
    for (int i = 0; i < totalRings - 1; i++) {
        for (int j = 0; j < slices; j++) {
            uint32_t a = i * (slices + 1) + j;
            uint32_t b = a + slices + 1;
            mesh.indices.insert(mesh.indices.end(),
                {a, b, a+1, b, b+1, a+1});
        }
    }
    dropDegenerate(mesh);
    return mesh;
}

// ---------------------------------------------------------------------------
// Procgen-grade assembly ops (ROADMAP 3.3)
// ---------------------------------------------------------------------------

// reserve(size + n) on every append is an exact fit: it defeats the vector's
// geometric growth, so a city appended one building at a time copies the
// whole buffer per building — O(n^2). piedmont_roads (an 8 km metro) spent
// 25+ minutes and 3.8 GB in here. Grow by doubling instead.
template <typename T>
static void growFor(std::vector<T>& v, std::size_t extra) {
    const std::size_t need = v.size() + extra;
    if (need > v.capacity()) v.reserve(std::max(need, v.capacity() * 2));
}

void MeshBuilder::append(RenderMesh& dst, const RenderMesh& src) {
    const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    growFor(dst.vertices, src.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    growFor(dst.indices, src.indices.size());
    for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
}

void MeshBuilder::appendTransformed(RenderMesh& dst, const RenderMesh& src,
                                    const Mat4& xform) {
    const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    const Mat4 normalMatrix = xform.inverse().transpose();
    growFor(dst.vertices, src.vertices.size());
    for (const Vertex& v : src.vertices) {
        Vertex out = v;
        out.position = xform.transformPoint(v.position);
        out.normal = normalize(normalMatrix.transformDirection(v.normal));
        out.tangent = normalize(xform.transformDirection(v.tangent));
        dst.vertices.push_back(out);
    }
    growFor(dst.indices, src.indices.size());
    for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
}

void MeshBuilder::transform(RenderMesh& mesh, const Mat4& xform) {
    const Mat4 normalMatrix = xform.inverse().transpose();
    for (Vertex& v : mesh.vertices) {
        v.position = xform.transformPoint(v.position);
        v.normal = normalize(normalMatrix.transformDirection(v.normal));
        v.tangent = normalize(xform.transformDirection(v.tangent));
    }
}

RenderMesh MeshBuilder::merged(const std::vector<RenderMesh>& parts) {
    RenderMesh out;
    if (!parts.empty()) out.materialIndex = parts.front().materialIndex;
    for (const RenderMesh& part : parts) append(out, part);
    return out;
}

void MeshBuilder::recomputeNormals(RenderMesh& mesh) {
    for (Vertex& v : mesh.vertices) v.normal = Vec3(0, 0, 0);
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        uint32_t ia = mesh.indices[i], ib = mesh.indices[i + 1], ic = mesh.indices[i + 2];
        // This engine winds front faces clockwise (a box's +Y face has explicit
        // normal +Y but a right-hand winding normal of -Y), so the outward
        // normal is cross(c-a, b-a) — the reverse of the textbook right-hand
        // rule. Area-weighted (the un-normalized cross scales with face area).
        Vec3 faceNormal = cross(mesh.vertices[ic].position - mesh.vertices[ia].position,
                                mesh.vertices[ib].position - mesh.vertices[ia].position);
        mesh.vertices[ia].normal += faceNormal;
        mesh.vertices[ib].normal += faceNormal;
        mesh.vertices[ic].normal += faceNormal;
    }
    for (Vertex& v : mesh.vertices) v.normal = normalize(v.normal);
}

void MeshBuilder::dropDegenerate(RenderMesh& mesh) {
    std::size_t w = 0;
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t a = mesh.indices[t], b = mesh.indices[t + 1], c = mesh.indices[t + 2];
        const Vec3 f = cross(mesh.vertices[c].position - mesh.vertices[a].position,
                             mesh.vertices[b].position - mesh.vertices[a].position);
        if (f.lengthSquared() < kNoArea2) continue;
        mesh.indices[w++] = a; mesh.indices[w++] = b; mesh.indices[w++] = c;
    }
    mesh.indices.resize(w);
}

void MeshBuilder::flatShade(RenderMesh& mesh) {
    RenderMesh flat;
    flat.materialIndex = mesh.materialIndex;
    flat.tangentIsData = mesh.tangentIsData;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Vertex a = mesh.vertices[mesh.indices[i]], b = mesh.vertices[mesh.indices[i + 1]], c = mesh.vertices[mesh.indices[i + 2]];
        const Vec3 n = cross(c.position - a.position, b.position - a.position);
        if (n.lengthSquared() < kNoArea2) continue;
        a.normal = b.normal = c.normal = normalize(n);
        const uint32_t base = static_cast<uint32_t>(flat.vertices.size());
        flat.vertices.push_back(a); flat.vertices.push_back(b); flat.vertices.push_back(c);
        triFacing(flat, base, base + 1, base + 2, a.normal);
    }
    mesh = std::move(flat);
}

void MeshBuilder::autosmooth(RenderMesh& mesh, double creaseDeg) {
    const double cosCrease = std::cos(creaseDeg * 3.141592653589793 / 180.0);
    const std::size_t nt = mesh.indices.size() / 3;
    std::vector<Vec3> face(nt), unit(nt);
    std::vector<std::vector<uint32_t>> around(mesh.vertices.size());   // the faces round each vertex
    for (std::size_t t = 0; t < nt; ++t) {
        const Vertex &a = mesh.vertices[mesh.indices[3 * t]], &b = mesh.vertices[mesh.indices[3 * t + 1]],
                     &c = mesh.vertices[mesh.indices[3 * t + 2]];
        face[t] = cross(c.position - a.position, b.position - a.position);   // outward (clockwise front), by area
        unit[t] = face[t].lengthSquared() > 1e-30 ? normalize(face[t]) : Vec3(0, 0, 0);
        for (int k = 0; k < 3; ++k) around[mesh.indices[3 * t + k]].push_back(static_cast<uint32_t>(t));
    }
    // Each corner's normal; corners of one vertex whose normals agree share it, the rest get a copy.
    std::vector<Vertex> out = mesh.vertices;
    std::vector<std::vector<std::pair<Vec3, uint32_t>>> made(mesh.vertices.size());
    for (std::size_t t = 0; t < nt; ++t)
        for (int k = 0; k < 3; ++k) {
            const uint32_t vi = mesh.indices[3 * t + k];
            Vec3 n(0, 0, 0);
            for (uint32_t f : around[vi])
                if (dot(unit[f], unit[t]) >= cosCrease) n += face[f];
            n = n.lengthSquared() > 1e-30 ? normalize(n) : (unit[t].lengthSquared() > 0 ? unit[t] : mesh.vertices[vi].normal);
            uint32_t idx = UINT32_MAX;
            for (const auto& [mn, mi] : made[vi])
                if (dot(mn, n) > 0.9999) { idx = mi; break; }
            if (idx == UINT32_MAX) {
                if (made[vi].empty()) idx = vi;
                else { idx = static_cast<uint32_t>(out.size()); out.push_back(mesh.vertices[vi]); }
                out[idx].normal = n;
                made[vi].push_back({n, idx});
            }
            mesh.indices[3 * t + k] = idx;
        }
    mesh.vertices = std::move(out);
}

void MeshBuilder::emitTri(RenderMesh& mesh, const Vec3& a, const Vec3& b,
                          const Vec3& c, const Vec3& normal, const Vec3& color) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 edge = b - a;
    Vec3 tan = edge.lengthSquared() > 1e-12 ? normalize(edge) : Vec3(1, 0, 0);
    auto mk = [&](const Vec3& p) {
        Vertex v(p, normal, tan, 0, 0);
        v.color = color;
        return v;
    };
    mesh.vertices.push_back(mk(a));
    mesh.vertices.push_back(mk(b));
    mesh.vertices.push_back(mk(c));
    triFacing(mesh, base, base + 1, base + 2, normal);
}

void MeshBuilder::emitQuad(RenderMesh& mesh, const Vec3& a, const Vec3& b,
                           const Vec3& c, const Vec3& d, const Vec3& normal,
                           const Vec3& color) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 edge = b - a;
    Vec3 tan = edge.lengthSquared() > 1e-12 ? normalize(edge) : Vec3(1, 0, 0);
    auto vtx = [&](const Vec3& p, float u, float vv) {
        Vertex v(p, normal, tan, u, vv);
        v.color = color;
        return v;
    };
    mesh.vertices.push_back(vtx(a, 0, 0));
    mesh.vertices.push_back(vtx(b, 1, 0));
    mesh.vertices.push_back(vtx(c, 1, 1));
    mesh.vertices.push_back(vtx(d, 0, 1));
    triFacing(mesh, base, base + 1, base + 2, normal);
    triFacing(mesh, base, base + 2, base + 3, normal);
}

void MeshBuilder::emitTriUV(RenderMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
                            const Vec3& normal, const Vec3& color,
                            float ua, float va, float ub, float vb, float uc, float vc) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 edge = b - a;
    Vec3 tan = edge.lengthSquared() > 1e-12 ? normalize(edge) : Vec3(1, 0, 0);
    auto mk = [&](const Vec3& p, float u, float vv) {
        Vertex v(p, normal, tan, u, vv);
        v.color = color;
        return v;
    };
    mesh.vertices.push_back(mk(a, ua, va));
    mesh.vertices.push_back(mk(b, ub, vb));
    mesh.vertices.push_back(mk(c, uc, vc));
    triFacing(mesh, base, base + 1, base + 2, normal);
}

void MeshBuilder::emitQuadUV(RenderMesh& mesh, const Vec3& a, const Vec3& b,
                             const Vec3& c, const Vec3& d, const Vec3& normal,
                             const Vec3& color, float ua, float va, float ub, float vb,
                             float uc, float vc, float ud, float vd) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 edge = b - a;
    Vec3 tan = edge.lengthSquared() > 1e-12 ? normalize(edge) : Vec3(1, 0, 0);
    auto vtx = [&](const Vec3& p, float u, float vv) {
        Vertex v(p, normal, tan, u, vv);
        v.color = color;
        return v;
    };
    mesh.vertices.push_back(vtx(a, ua, va));
    mesh.vertices.push_back(vtx(b, ub, vb));
    mesh.vertices.push_back(vtx(c, uc, vc));
    mesh.vertices.push_back(vtx(d, ud, vd));
    triFacing(mesh, base, base + 1, base + 2, normal);
    triFacing(mesh, base, base + 2, base + 3, normal);
}

void MeshBuilder::gridIndices(RenderMesh& mesh, int cols, int rows,
                              uint32_t base) {
    if (cols < 2 || rows < 2) return;
    mesh.indices.reserve(mesh.indices.size() +
                         static_cast<size_t>(cols - 1) * (rows - 1) * 6);
    for (int j = 0; j + 1 < rows; ++j) {
        for (int i = 0; i + 1 < cols; ++i) {
            uint32_t a = base + static_cast<uint32_t>(j * cols + i);
            uint32_t b = a + 1;                              // +x
            uint32_t c = a + static_cast<uint32_t>(cols);    // +z
            uint32_t d = c + 1;                              // +x, +z
            gridCell(mesh, a, b, d, c);
        }
    }
}

void MeshBuilder::gridCell(RenderMesh& mesh, uint32_t a00, uint32_t a10, uint32_t a11, uint32_t a01) {
    // clockwise-front for an up-facing (+Y) surface: (a00, a10, a11) and (a00, a11, a01) seen from above; a lattice
    // over a height field can never turn a cell over in plan, so the order is the winding
    mesh.indices.insert(mesh.indices.end(), {a00, a10, a11, a00, a11, a01});
}

void MeshBuilder::emitLattice(RenderMesh& mesh, const LatticeSpec& spec) {
    const int R = spec.rings, P = spec.profilePts;
    if (R < 2 || P < 2 || !spec.verts) return;

    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.insert(mesh.vertices.end(), spec.verts,
                         spec.verts + static_cast<size_t>(R) * P);
    auto at = [&](int i, int j) { return base + static_cast<uint32_t>(i * P + j); };
    auto pos = [&](int i, int j) { return spec.verts[i * P + j].position; };

    // Fix winding ONCE for the whole lattice. The canonical cell winding is
    // {a,b,d, a,d,c} with a=(i,j), b=(i,j+1) [+column], c=(i+1,j) [+ring]; its
    // geometric front normal is cross(d-a, b-a). Whether that agrees with the
    // surface the caller built (banked, curving, either handedness of column
    // order) is data, not something to assume — so read it from the supplied
    // vertex normals of the first non-degenerate cell and apply that one choice
    // to every cell. A per-face test can't live on shared vertices.
    bool flip = false;
    for (int i = 0; i + 1 < R && flip == false; ++i) {
        bool decided = false;
        for (int j = 0; j + 1 < P && !decided; ++j) {
            const Vec3 a = pos(i, j), b = pos(i, j + 1), d = pos(i + 1, j + 1);
            const Vec3 gn = cross(d - a, b - a);
            if (gn.lengthSquared() < 1e-12) continue;        // degenerate cell
            Vec3 sn = spec.verts[i * P + j].normal + spec.verts[i * P + j + 1].normal +
                      spec.verts[(i + 1) * P + j].normal + spec.verts[(i + 1) * P + j + 1].normal;
            if (sn.lengthSquared() < 1e-12) { decided = true; break; }  // no normal hint
            flip = dot(gn, sn) < 0.0;
            decided = true;
        }
        if (decided) break;
    }

    mesh.indices.reserve(mesh.indices.size() +
                         static_cast<size_t>(R - 1) * (P - 1) * 6);
    // Skip a zero-area TRIANGLE rather than emit it: a swept profile with a
    // pinched column (a collapsed curb, a Coons pad whose interior folds where a
    // branch is embedded) yields a needle a car never sees but the mesh-quality
    // and degenerate checks rightly flag. Each cell is two tris; drop only those
    // that collapse, keeping the rest of the quad.
    auto area2 = [&](uint32_t x, uint32_t y, uint32_t z) {
        const Vec3& A = mesh.vertices[x].position;
        const Vec3& B = mesh.vertices[y].position;
        const Vec3& C = mesh.vertices[z].position;
        return cross(B - A, C - A).lengthSquared();
    };
    auto emit = [&](uint32_t x, uint32_t y, uint32_t z) {
        if (area2(x, y, z) > 1e-14) mesh.indices.insert(mesh.indices.end(), {x, y, z});
    };
    // The global choice above is the DEFAULT, not the verdict. It reads one cell,
    // which is right for a surface with a consistent handedness (a swept body, a
    // banked deck) but wrong where the sheet FOLDS: a Coons junction pad on sloped
    // ground can reverse its own orientation partway across, and one choice then
    // leaves the folded cells inverted — they cull away in the viewer. Measured on
    // a 3x3 street grid over a sine grade: 24 of 11575 road faces.
    //
    // So decide per CELL from that cell's own four corner normals, and fall back to
    // the global choice only when the corners give no usable hint. This is not the
    // per-face normal test the header warns against — that one compares a face to a
    // single shared (averaged) vertex normal and false-positives on every crease.
    // Here the comparison is a whole cell against the sum of its own corners, which
    // is exactly the test the global pass already performs, just not thrown away
    // after the first hit. A consistently-wound sheet gets the identical result.
    for (int i = 0; i + 1 < R; ++i) {
        for (int j = 0; j + 1 < P; ++j) {
            const uint32_t a = at(i, j), b = at(i, j + 1),
                           c = at(i + 1, j), d = at(i + 1, j + 1);
            bool cellFlip = flip;
            const Vec3 gn = cross(pos(i + 1, j + 1) - pos(i, j),
                                  pos(i, j + 1) - pos(i, j));
            if (gn.lengthSquared() > 1e-12) {
                const Vec3 sn = spec.verts[i * P + j].normal +
                                spec.verts[i * P + j + 1].normal +
                                spec.verts[(i + 1) * P + j].normal +
                                spec.verts[(i + 1) * P + j + 1].normal;
                if (sn.lengthSquared() > 1e-12) cellFlip = dot(gn, sn) < 0.0;
            }
            if (!cellFlip) { emit(a, b, d); emit(a, d, c); }
            else           { emit(a, d, b); emit(a, c, d); }
        }
    }
}

void MeshBuilder::bakeHeightColor(RenderMesh& mesh, const Vec3& low, const Vec3& high) {
    if (mesh.vertices.empty()) return;
    double minY = 1e30, maxY = -1e30;
    for (const Vertex& v : mesh.vertices) {
        minY = std::min(minY, static_cast<double>(v.position.y));
        maxY = std::max(maxY, static_cast<double>(v.position.y));
    }
    double range = maxY - minY;
    for (Vertex& v : mesh.vertices) {
        double t = range > 1e-6 ? (v.position.y - minY) / range : 0.0;
        v.color = low + (high - low) * t;
    }
}

void MeshBuilder::generatePlanarUVs(RenderMesh& mesh, int axis, float scale) {
    for (Vertex& v : mesh.vertices) {
        const Vec3& p = v.position;
        float uu, vv;
        if (axis == 0)      { uu = static_cast<float>(p.z); vv = static_cast<float>(p.y); }
        else if (axis == 2) { uu = static_cast<float>(p.x); vv = static_cast<float>(p.y); }
        else                { uu = static_cast<float>(p.x); vv = static_cast<float>(p.z); }
        v.u = uu * scale;
        v.v = vv * scale;
    }
}

}  // namespace engine

namespace engine {

std::vector<MeshBuilder::CellChunk> MeshBuilder::chunkByCell(const RenderMesh& m, double cell) {
    std::vector<CellChunk> out;
    if (cell <= 0.0 || m.indices.size() < 3) { CellChunk c; c.mesh = m; out.push_back(std::move(c)); return out; }
    std::map<std::pair<int, int>, std::size_t> slot;
    std::vector<std::unordered_map<uint32_t, uint32_t>> remap;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const uint32_t i0 = m.indices[t], i1 = m.indices[t + 1], i2 = m.indices[t + 2];
        const Vec3& a = m.vertices[i0].position; const Vec3& b = m.vertices[i1].position; const Vec3& c = m.vertices[i2].position;
        const double cx = (a.x + b.x + c.x) / 3.0, cz = (a.z + b.z + c.z) / 3.0;
        const std::pair<int, int> key{static_cast<int>(std::floor(cx / cell)), static_cast<int>(std::floor(cz / cell))};
        auto it = slot.find(key);
        if (it == slot.end()) {
            it = slot.emplace(key, out.size()).first;
            CellChunk cc; cc.cx = key.first; cc.cz = key.second; cc.mesh.materialIndex = m.materialIndex; out.push_back(std::move(cc)); remap.emplace_back();
        }
        RenderMesh& dst = out[it->second].mesh; auto& rm = remap[it->second];
        for (uint32_t src : {i0, i1, i2}) {
            auto ri = rm.find(src);
            if (ri == rm.end()) { ri = rm.emplace(src, static_cast<uint32_t>(dst.vertices.size())).first; dst.vertices.push_back(m.vertices[src]); }
            dst.indices.push_back(ri->second);
        }
    }
    return out;
}

std::vector<std::array<uint32_t, 3>> MeshBuilder::convexHull(const std::vector<Vec3>& pts) {
    using Face = std::array<uint32_t, 3>;
    std::vector<Face> faces;
    const std::size_t n = pts.size();
    if (n < 4) return faces;
    double scale = 0.0;
    for (const Vec3& p : pts) scale = std::max(scale, (p - pts[0]).length());
    if (scale <= 0.0) return faces;
    const double eps = 1e-9 * scale;
    // a first tetrahedron: the farthest point from 0, from that line, from that plane
    uint32_t i0 = 0, i1 = 0, i2 = 0, i3 = 0;
    double best = -1.0;
    for (uint32_t i = 1; i < n; ++i)
        if (double d = (pts[i] - pts[i0]).length(); d > best) { best = d; i1 = i; }
    best = -1.0;
    const Vec3 ab = pts[i1] - pts[i0];
    for (uint32_t i = 0; i < n; ++i)
        if (double d = cross(ab, pts[i] - pts[i0]).length(); d > best) { best = d; i2 = i; }
    if (best <= eps * scale) return faces;
    const Vec3 nrm = cross(ab, pts[i2] - pts[i0]);
    best = -1.0;
    for (uint32_t i = 0; i < n; ++i)
        if (double d = std::fabs(dot(nrm, pts[i] - pts[i0])); d > best) { best = d; i3 = i; }
    if (best <= eps * scale * scale) return faces;
    auto normalOf = [&](const Face& f) { return cross(pts[f[1]] - pts[f[0]], pts[f[2]] - pts[f[0]]); };
    const Vec3 inside = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) * 0.25;
    for (Face f : {Face{i0, i1, i2}, Face{i0, i1, i3}, Face{i0, i2, i3}, Face{i1, i2, i3}}) {
        if (dot(normalOf(f), pts[f[0]] - inside) < 0) std::swap(f[1], f[2]);
        faces.push_back(f);
    }
    for (uint32_t p = 0; p < n; ++p) {
        if (p == i0 || p == i1 || p == i2 || p == i3) continue;
        std::vector<char> seen(faces.size(), 0);
        bool any = false;
        for (std::size_t f = 0; f < faces.size(); ++f) {
            const Vec3 fn = normalOf(faces[f]);
            const double len = fn.length();
            if (len > 0 && dot(fn, pts[p] - pts[faces[f][0]]) / len > eps) { seen[f] = 1; any = true; }
        }
        if (!any) continue;
        // the horizon: the visible faces' edges whose other side is not visible, kept in their winding
        std::vector<std::pair<uint32_t, uint32_t>> edges;
        for (std::size_t f = 0; f < faces.size(); ++f) {
            if (!seen[f]) continue;
            for (int e = 0; e < 3; ++e) edges.push_back({faces[f][e], faces[f][(e + 1) % 3]});
        }
        std::vector<Face> next;
        for (std::size_t f = 0; f < faces.size(); ++f)
            if (!seen[f]) next.push_back(faces[f]);
        for (const auto& [a, b] : edges) {
            bool twin = false;
            for (const auto& [c, d] : edges) twin = twin || (c == b && d == a);
            if (!twin) next.push_back(Face{a, b, p});
        }
        faces.swap(next);
    }
    return faces;
}


std::size_t MeshBuilder::simplify(RenderMesh& mesh, std::size_t targetTris, double maxError) {
    const std::size_t nv = mesh.vertices.size();
    std::vector<std::array<uint32_t, 3>> F;
    for (std::size_t k = 0; k + 2 < mesh.indices.size(); k += 3) F.push_back({mesh.indices[k], mesh.indices[k + 1], mesh.indices[k + 2]});
    std::size_t live = F.size();
    if (live <= targetTris || nv < 4) return live;
    std::vector<Vec3> P(nv);
    for (std::size_t i = 0; i < nv; ++i) P[i] = mesh.vertices[i].position;
    // a symmetric 4x4 quadric: xx xy xz xw yy yz yw zz zw ww
    using Quad = std::array<double, 10>;
    std::vector<Quad> Q(nv, Quad{});
    auto addPlane = [](Quad& q, const Vec3& n, double d, double w) {
        q[0] += w * n.x * n.x; q[1] += w * n.x * n.y; q[2] += w * n.x * n.z; q[3] += w * n.x * d;
        q[4] += w * n.y * n.y; q[5] += w * n.y * n.z; q[6] += w * n.y * d;
        q[7] += w * n.z * n.z; q[8] += w * n.z * d; q[9] += w * d * d;
    };
    auto err = [](const Quad& q, const Vec3& p) {
        return q[0] * p.x * p.x + 2 * q[1] * p.x * p.y + 2 * q[2] * p.x * p.z + 2 * q[3] * p.x + q[4] * p.y * p.y +
               2 * q[5] * p.y * p.z + 2 * q[6] * p.y + q[7] * p.z * p.z + 2 * q[8] * p.z + q[9];
    };
    std::vector<std::vector<uint32_t>> vf(nv);
    std::map<std::pair<uint32_t, uint32_t>, int> edgeUse;
    for (uint32_t f = 0; f < F.size(); ++f) {
        const Vec3 &a = P[F[f][0]], &b = P[F[f][1]], &c = P[F[f][2]];
        const Vec3 cr = cross(b - a, c - a);
        const double len = cr.length();
        if (len > 1e-20) {
            const Vec3 n = cr * (1.0 / len);
            for (int e = 0; e < 3; ++e) addPlane(Q[F[f][e]], n, -dot(n, a), 0.5 * len);
        }
        for (int e = 0; e < 3; ++e) {
            vf[F[f][e]].push_back(f);
            const uint32_t x = F[f][e], y = F[f][(e + 1) % 3];
            ++edgeUse[{std::min(x, y), std::max(x, y)}];
        }
    }
    // an open edge holds its line: a plane through it, square to its face, weighted heavy
    for (uint32_t f = 0; f < F.size(); ++f) {
        const Vec3 &a = P[F[f][0]], &b = P[F[f][1]], &c = P[F[f][2]];
        const Vec3 fn = cross(b - a, c - a);
        if (fn.length() < 1e-20) continue;
        for (int e = 0; e < 3; ++e) {
            const uint32_t x = F[f][e], y = F[f][(e + 1) % 3];
            if (edgeUse[{std::min(x, y), std::max(x, y)}] != 1) continue;
            const Vec3 ed = P[y] - P[x];
            Vec3 n = cross(ed, fn);
            if (n.length() < 1e-20) continue;
            n = normalize(n);
            const double w = 100.0 * ed.lengthSquared();
            addPlane(Q[x], n, -dot(n, P[x]), w);
            addPlane(Q[y], n, -dot(n, P[x]), w);
        }
    }
    std::vector<char> deadV(nv, 0), deadF(F.size(), 0);
    std::vector<uint32_t> version(nv, 0);
    struct Cand { double cost; uint32_t u, v, vu, vv; Vec3 p; };
    auto worse = [](const Cand& a, const Cand& b) { return a.cost > b.cost; };
    std::priority_queue<Cand, std::vector<Cand>, decltype(worse)> heap(worse);
    auto plan = [&](uint32_t u, uint32_t v) {
        Quad q;
        for (int i = 0; i < 10; ++i) q[i] = Q[u][i] + Q[v][i];
        // the point minimising the summed quadric (a 3x3 solve), else the better of the ends and the middle
        const double A[3][3] = {{q[0], q[1], q[2]}, {q[1], q[4], q[5]}, {q[2], q[5], q[7]}};
        const double bb[3] = {-q[3], -q[6], -q[8]};
        const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                           A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        Vec3 best = (P[u] + P[v]) * 0.5;
        double cost = err(q, best);
        const double scale = std::max({std::fabs(A[0][0]), std::fabs(A[1][1]), std::fabs(A[2][2]), 1e-30});
        if (std::fabs(det) > 1e-9 * scale * scale * scale) {
            auto solve = [&](int col) {
                double M[3][3];
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) M[r][c] = c == col ? bb[r] : A[r][c];
                return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
            };
            const Vec3 opt(solve(0), solve(1), solve(2));
            // (only near the edge: a far optimum on a nearly flat patch is a spike)
            const double reach = (P[u] - P[v]).length();
            if ((opt - (P[u] + P[v]) * 0.5).length() < 1.5 * reach && err(q, opt) < cost) { best = opt; cost = err(q, opt); }
        }
        for (const Vec3& c : {P[u], P[v]})
            if (err(q, c) < cost) { best = c; cost = err(q, c); }
        heap.push({std::max(0.0, cost), u, v, version[u], version[v], best});
    };
    for (const auto& [e, n] : edgeUse) plan(e.first, e.second);
    auto neighbours = [&](uint32_t x, std::set<uint32_t>& out) {
        for (uint32_t f : vf[x])
            if (!deadF[f])
                for (uint32_t w : F[f])
                    if (w != x) out.insert(w);
    };
    while (live > targetTris && !heap.empty()) {
        const Cand c = heap.top();
        heap.pop();
        if (deadV[c.u] || deadV[c.v] || c.vu != version[c.u] || c.vv != version[c.v]) continue;
        if (c.cost > maxError) break;
        const uint32_t u = c.u, v = c.v;
        // manifold: the ends share exactly the neighbours their shared faces give them
        std::set<uint32_t> nu, nvv;
        neighbours(u, nu);
        neighbours(v, nvv);
        int sharedFaces = 0;
        for (uint32_t f : vf[u])
            if (!deadF[f] && (F[f][0] == v || F[f][1] == v || F[f][2] == v)) ++sharedFaces;
        int common = 0;
        for (uint32_t w : nu) common += nvv.count(w) ? 1 : 0;
        if (sharedFaces == 0 || common != sharedFaces) continue;
        // no face turned over or squashed flat
        bool ok = true;
        for (uint32_t x : {u, v})
            for (uint32_t f : vf[x]) {
                if (deadF[f] || !ok) continue;
                const auto& t = F[f];
                if ((t[0] == u || t[1] == u || t[2] == u) && (t[0] == v || t[1] == v || t[2] == v)) continue;
                Vec3 q[3];
                for (int e = 0; e < 3; ++e) q[e] = (t[e] == u || t[e] == v) ? c.p : P[t[e]];
                const Vec3 n0 = cross(P[t[1]] - P[t[0]], P[t[2]] - P[t[0]]), n1 = cross(q[1] - q[0], q[2] - q[0]);
                const double l0 = n0.length(), l1 = n1.length();
                if (l1 < 1e-12 || dot(n0, n1) < 0.2 * l0 * l1) ok = false;
                // (nor made a sliver: shape 2 sqrt(3) area / the edges' squares, 1 equilateral -- under 0.15 and worse
                // than the face it was, a long thin triangle that streaks the shading)
                auto shape = [](const Vec3& a, const Vec3& b, const Vec3& c, double crossLen) {
                    const double e2 = (b - a).lengthSquared() + (c - b).lengthSquared() + (a - c).lengthSquared();
                    return e2 > 1e-30 ? 2.0 * std::sqrt(3.0) * crossLen / e2 : 0.0;
                };
                const double s1 = shape(q[0], q[1], q[2], l1);
                if (s1 < 0.15 && s1 < shape(P[t[0]], P[t[1]], P[t[2]], l0)) ok = false;
            }
        if (!ok) continue;
        for (uint32_t f : vf[v]) {
            if (deadF[f]) continue;
            auto& t = F[f];
            if (t[0] == u || t[1] == u || t[2] == u) { deadF[f] = 1; --live; continue; }
            for (uint32_t& w : t) if (w == v) w = u;
            vf[u].push_back(f);
        }
        deadV[v] = 1;
        P[u] = c.p;
        for (int i = 0; i < 10; ++i) Q[u][i] += Q[v][i];
        ++version[u];
        std::set<uint32_t> again;
        neighbours(u, again);
        for (uint32_t w : again) plan(std::min(u, w), std::max(u, w));
    }
    // the kept vertices, in order, at their moved places
    std::vector<uint32_t> remap(nv, ~0u);
    RenderMesh out;
    out.materialIndex = mesh.materialIndex;
    out.tangentIsData = mesh.tangentIsData;
    for (uint32_t f = 0; f < F.size(); ++f) {
        if (deadF[f]) continue;
        for (uint32_t w : F[f]) {
            if (remap[w] == ~0u) {
                remap[w] = static_cast<uint32_t>(out.vertices.size());
                Vertex vx = mesh.vertices[w];
                vx.position = P[w];
                out.vertices.push_back(vx);
            }
            out.indices.push_back(remap[w]);
        }
    }
    mesh = std::move(out);
    return live;
}


std::size_t MeshBuilder::splitNonManifold(RenderMesh& mesh) {
    const std::size_t nf = mesh.indices.size() / 3;
    auto ekey = [](uint32_t a, uint32_t b) { return (static_cast<uint64_t>(std::min(a, b)) << 32) | std::max(a, b); };
    std::unordered_map<uint64_t, int> use;
    for (std::size_t f = 0; f < nf; ++f)
        for (int e = 0; e < 3; ++e) ++use[ekey(mesh.indices[f * 3 + e], mesh.indices[f * 3 + (e + 1) % 3])];
    std::vector<std::vector<uint32_t>> vf(mesh.vertices.size());
    for (std::size_t f = 0; f < nf; ++f)
        for (int e = 0; e < 3; ++e) vf[mesh.indices[f * 3 + e]].push_back(static_cast<uint32_t>(f));
    std::size_t added = 0;
    const std::size_t nv = mesh.vertices.size();
    for (uint32_t v = 0; v < nv; ++v) {
        const std::vector<uint32_t>& fs = vf[v];
        if (fs.size() < 2) continue;
        // the fans: faces round v joined through an edge from v that has exactly two faces
        std::vector<int> comp(fs.size(), -1);
        int comps = 0;
        for (std::size_t i = 0; i < fs.size(); ++i) {
            if (comp[i] >= 0) continue;
            comp[i] = comps;
            std::vector<std::size_t> stack{i};
            while (!stack.empty()) {
                const std::size_t a = stack.back();
                stack.pop_back();
                for (std::size_t j = 0; j < fs.size(); ++j) {
                    if (comp[j] >= 0) continue;
                    // do faces a and j share an edge (v, w) with two faces?
                    bool joined = false;
                    for (int e = 0; e < 3 && !joined; ++e) {
                        const uint32_t w = mesh.indices[fs[a] * 3 + e];
                        if (w == v) continue;
                        bool inJ = false;
                        for (int k = 0; k < 3; ++k) inJ |= mesh.indices[fs[j] * 3 + k] == w;
                        joined = inJ && use[ekey(v, w)] == 2;
                    }
                    if (joined) { comp[j] = comps; stack.push_back(j); }
                }
            }
            ++comps;
        }
        for (int c = 1; c < comps; ++c) {   // every fan past the first its own copy of v
            mesh.vertices.push_back(mesh.vertices[v]);
            const uint32_t nvx = static_cast<uint32_t>(mesh.vertices.size() - 1);
            for (std::size_t i = 0; i < fs.size(); ++i)
                if (comp[i] == c)
                    for (int k = 0; k < 3; ++k)
                        if (mesh.indices[fs[i] * 3 + k] == v) mesh.indices[fs[i] * 3 + k] = nvx;
            ++added;
        }
    }
    return added;
}

void MeshBuilder::remesh(RenderMesh& mesh, const std::function<double(const Vec3&)>& edgeAt,
                         const std::function<Vec3(const Vec3&)>& project, int iterations) {
    std::vector<Vec3> P;
    P.reserve(mesh.vertices.size());
    for (const Vertex& v : mesh.vertices) P.push_back(v.position);
    std::vector<std::array<uint32_t, 3>> F;
    for (std::size_t k = 0; k + 2 < mesh.indices.size(); k += 3) F.push_back({mesh.indices[k], mesh.indices[k + 1], mesh.indices[k + 2]});
    auto ekey = [](uint32_t a, uint32_t b) { return (static_cast<uint64_t>(std::min(a, b)) << 32) | std::max(a, b); };
    auto faceN = [&](const std::array<uint32_t, 3>& t) { return cross(P[t[1]] - P[t[0]], P[t[2]] - P[t[0]]); };
    for (int it = 0; it < iterations; ++it) {
        // SPLIT every edge over 4/3 of its length (its middle on the surface), a face by how many of its edges split
        for (int pass = 0; pass < 3; ++pass) {
            std::unordered_map<uint64_t, uint32_t> mid;
            for (const auto& t : F)
                for (int e = 0; e < 3; ++e) {
                    const uint32_t a = t[e], b = t[(e + 1) % 3];
                    const uint64_t k = ekey(a, b);
                    if (mid.count(k)) continue;
                    const Vec3 m = (P[a] + P[b]) * 0.5;
                    if ((P[a] - P[b]).length() > 4.0 / 3.0 * edgeAt(m)) {
                        P.push_back(project(m));
                        mid[k] = static_cast<uint32_t>(P.size() - 1);
                    }
                }
            if (mid.empty()) break;
            std::vector<std::array<uint32_t, 3>> next;
            next.reserve(F.size() * 2);
            for (const auto& t : F) {
                int64_t m[3];
                for (int e = 0; e < 3; ++e) {
                    auto f = mid.find(ekey(t[e], t[(e + 1) % 3]));
                    m[e] = f == mid.end() ? int64_t{-1} : static_cast<int64_t>(f->second);
                }
                const int n = (m[0] >= 0) + (m[1] >= 0) + (m[2] >= 0);
                if (n == 0) { next.push_back(t); continue; }
                if (n == 3) {
                    const uint32_t ab = static_cast<uint32_t>(m[0]), bc = static_cast<uint32_t>(m[1]), ca = static_cast<uint32_t>(m[2]);
                    next.push_back({t[0], ab, ca}); next.push_back({ab, t[1], bc}); next.push_back({ca, bc, t[2]}); next.push_back({ab, bc, ca});
                    continue;
                }
                int r0 = 0;
                while (m[r0] < 0) ++r0;
                if (n == 2 && m[(r0 + 2) % 3] >= 0) r0 = (r0 + 2) % 3;   // (the two split edges r0, r0 + 1)
                const uint32_t a = t[r0], b = t[(r0 + 1) % 3], c = t[(r0 + 2) % 3];
                const uint32_t m1 = static_cast<uint32_t>(m[r0]);
                if (n == 1) { next.push_back({a, m1, c}); next.push_back({m1, b, c}); continue; }
                const uint32_t m2 = static_cast<uint32_t>(m[(r0 + 1) % 3]);
                // (the quad a m1 m2 c split along its shorter diagonal)
                next.push_back({m1, b, m2});
                if ((P[a] - P[m2]).length() < (P[m1] - P[c]).length()) { next.push_back({a, m1, m2}); next.push_back({a, m2, c}); }
                else { next.push_back({a, m1, c}); next.push_back({m1, m2, c}); }
            }
            F.swap(next);
        }
        // COLLAPSE every edge under 4/5 of its length, shortest first, into its middle (on the surface)
        {
            std::vector<std::vector<uint32_t>> vf(P.size());
            for (uint32_t f = 0; f < F.size(); ++f)
                for (uint32_t w : F[f]) vf[w].push_back(f);
            std::vector<char> deadF(F.size(), 0), touched(P.size(), 0);
            std::vector<std::pair<double, uint64_t>> shortE;
            {
                std::unordered_map<uint64_t, char> seen;
                for (const auto& t : F)
                    for (int e = 0; e < 3; ++e) {
                        const uint32_t a = t[e], b = t[(e + 1) % 3];
                        const uint64_t k = ekey(a, b);
                        if (seen.emplace(k, 1).second) {
                            const double L = (P[a] - P[b]).length();
                            if (L < 0.8 * edgeAt((P[a] + P[b]) * 0.5)) shortE.push_back({L, k});
                        }
                    }
            }
            std::sort(shortE.begin(), shortE.end());
            auto ring = [&](uint32_t x, std::vector<uint32_t>& out) {
                out.clear();
                for (uint32_t f : vf[x])
                    if (!deadF[f])
                        for (uint32_t w : F[f])
                            if (w != x && std::find(out.begin(), out.end(), w) == out.end()) out.push_back(w);
            };
            std::vector<uint32_t> nu, nv;
            for (const auto& [L, k] : shortE) {
                const uint32_t u = static_cast<uint32_t>(k >> 32), v = static_cast<uint32_t>(k & 0xffffffffu);
                if (touched[u] || touched[v]) continue;
                ring(u, nu);
                ring(v, nv);
                if (std::find(nu.begin(), nu.end(), v) == nu.end()) continue;
                int shared = 0, common = 0;
                for (uint32_t f : vf[u])
                    if (!deadF[f] && (F[f][0] == v || F[f][1] == v || F[f][2] == v)) ++shared;
                for (uint32_t w : nu) common += std::find(nv.begin(), nv.end(), w) != nv.end();
                if (shared != 2 || common != 2) continue;   // (manifold: the link condition)
                const Vec3 p = project((P[u] + P[v]) * 0.5);
                bool ok = true;
                for (uint32_t w : nu) if (w != v && (P[w] - p).length() > 4.0 / 3.0 * edgeAt((P[w] + p) * 0.5)) ok = false;
                for (uint32_t w : nv) if (ok && w != u && (P[w] - p).length() > 4.0 / 3.0 * edgeAt((P[w] + p) * 0.5)) ok = false;
                for (uint32_t x : {u, v})
                    for (uint32_t f : vf[x]) {
                        if (!ok || deadF[f]) continue;
                        const auto& t = F[f];
                        if ((t[0] == u || t[1] == u || t[2] == u) && (t[0] == v || t[1] == v || t[2] == v)) continue;
                        Vec3 q[3];
                        for (int e = 0; e < 3; ++e) q[e] = (t[e] == u || t[e] == v) ? p : P[t[e]];
                        const Vec3 n0 = faceN(t), n1 = cross(q[1] - q[0], q[2] - q[0]);
                        if (n1.length() < 1e-14 || dot(n0, n1) < 0.3 * n0.length() * n1.length()) ok = false;
                    }
                if (!ok) continue;
                for (uint32_t f : vf[v]) {
                    if (deadF[f]) continue;
                    auto& t = F[f];
                    if (t[0] == u || t[1] == u || t[2] == u) { deadF[f] = 1; continue; }
                    for (uint32_t& w : t) if (w == v) w = u;
                    vf[u].push_back(f);
                }
                P[u] = p;
                touched[u] = touched[v] = 1;
                for (uint32_t w : nu) touched[w] = 1;
                for (uint32_t w : nv) touched[w] = 1;
            }
            std::vector<std::array<uint32_t, 3>> kept;
            for (uint32_t f = 0; f < F.size(); ++f) if (!deadF[f]) kept.push_back(F[f]);
            F.swap(kept);
        }
        // FLIP an edge where its four corners' valences come nearer six (and neither face turns over)
        {
            std::vector<int> val(P.size(), 0);
            std::unordered_map<uint64_t, std::array<int, 2>> ef;
            for (int f = 0; f < static_cast<int>(F.size()); ++f)
                for (int e = 0; e < 3; ++e) {
                    const uint64_t k = ekey(F[f][e], F[f][(e + 1) % 3]);
                    auto [itr, fresh] = ef.emplace(k, std::array<int, 2>{f, -1});
                    if (fresh) { ++val[F[f][e]]; ++val[F[f][(e + 1) % 3]]; }
                    else if (itr->second[1] < 0) itr->second[1] = f;
                    else itr->second[1] = -2;   // (three or more: leave it)
                }
            std::vector<char> busy(F.size(), 0);
            for (const auto& [k, fs] : ef) {
                if (fs[1] < 0 || busy[fs[0]] || busy[fs[1]]) continue;
                const uint32_t a = static_cast<uint32_t>(k >> 32), b = static_cast<uint32_t>(k & 0xffffffffu);
                auto other = [&](int f) { for (uint32_t w : F[f]) if (w != a && w != b) return w; return a; };
                const uint32_t c = other(fs[0]), d = other(fs[1]);
                if (c == d || ef.count(ekey(c, d))) continue;
                auto dev = [](int x) { return (x - 6) * (x - 6); };
                const int before = dev(val[a]) + dev(val[b]) + dev(val[c]) + dev(val[d]);
                const int after = dev(val[a] - 1) + dev(val[b] - 1) + dev(val[c] + 1) + dev(val[d] + 1);
                if (after >= before) continue;
                // the two faces as they would be, wound as the old ones: (a b c) -> (c d? ) -- by the old winding of a -> b
                const auto& t0 = F[fs[0]];
                int ia = 0;
                while (t0[ia] != a) ++ia;
                const bool abOrder = t0[(ia + 1) % 3] == b;   // face 0 runs a -> b -> c
                std::array<uint32_t, 3> n0 = abOrder ? std::array<uint32_t, 3>{a, d, c} : std::array<uint32_t, 3>{a, c, d};
                std::array<uint32_t, 3> n1 = abOrder ? std::array<uint32_t, 3>{b, c, d} : std::array<uint32_t, 3>{b, d, c};
                const Vec3 o = faceN(F[fs[0]]) + faceN(F[fs[1]]);
                const Vec3 m0 = faceN(n0), m1 = faceN(n1);
                if (dot(m0, o) <= 0.0 || dot(m1, o) <= 0.0 || dot(m0, m1) < 0.5 * m0.length() * m1.length()) continue;
                F[fs[0]] = n0;
                F[fs[1]] = n1;
                busy[fs[0]] = busy[fs[1]] = 1;
                --val[a]; --val[b]; ++val[c]; ++val[d];
            }
        }
        // RELAX: each vertex toward its neighbours' middle within its tangent plane, back onto the surface
        {
            std::vector<Vec3> sum(P.size(), Vec3(0, 0, 0)), nrm(P.size(), Vec3(0, 0, 0));
            std::vector<int> cnt(P.size(), 0);
            for (const auto& t : F) {
                const Vec3 n = faceN(t);
                for (int e = 0; e < 3; ++e) {
                    nrm[t[e]] = nrm[t[e]] + n;
                    sum[t[e]] = sum[t[e]] + P[t[(e + 1) % 3]];
                    ++cnt[t[e]];
                }
            }
            for (std::size_t v = 0; v < P.size(); ++v) {
                if (!cnt[v] || nrm[v].length() < 1e-14) continue;
                const Vec3 n = normalize(nrm[v]);
                const Vec3 d = sum[v] * (1.0 / cnt[v]) - P[v];
                P[v] = project(P[v] + (d - n * dot(d, n)) * 0.5);
            }
        }
    }
    // the vertices still used, in order
    std::vector<uint32_t> remap(P.size(), ~0u);
    RenderMesh out;
    out.materialIndex = mesh.materialIndex;
    const Vertex proto = mesh.vertices.empty() ? Vertex() : mesh.vertices.front();
    for (const auto& t : F) {
        for (uint32_t w : t) {
            if (remap[w] == ~0u) {
                remap[w] = static_cast<uint32_t>(out.vertices.size());
                Vertex vx = proto;
                vx.position = P[w];
                out.vertices.push_back(vx);
            }
            out.indices.push_back(remap[w]);
        }
    }
    mesh = std::move(out);
}


std::size_t MeshBuilder::fillHoles(RenderMesh& mesh, std::size_t maxEdges, const std::function<Vec3(const Vec3&)>& project) {
    // the open half-edges: a -> b on a face with no face running b -> a
    std::unordered_map<uint64_t, int> half;
    auto hkey = [](uint32_t a, uint32_t b) { return (static_cast<uint64_t>(a) << 32) | b; };
    for (std::size_t k = 0; k + 2 < mesh.indices.size(); k += 3)
        for (int e = 0; e < 3; ++e) ++half[hkey(mesh.indices[k + e], mesh.indices[k + (e + 1) % 3])];
    std::unordered_multimap<uint32_t, uint32_t> next;   // open a -> b, by a (a vertex two holes meet at has two)
    for (const auto& [k, n] : half) {
        const uint32_t a = static_cast<uint32_t>(k >> 32), b = static_cast<uint32_t>(k & 0xffffffffu);
        if (!half.count(hkey(b, a))) next.emplace(a, b);
    }
    std::size_t filled = 0;
    std::unordered_map<uint64_t, char> used;   // (open edges walked, not vertices: a loop may pass a vertex twice)
    for (const auto& [start, first] : next) {
        if (used.count(hkey(start, first))) continue;
        std::vector<uint32_t> loop{start};
        used[hkey(start, first)] = 1;
        uint32_t cur = first;
        bool closed = false;
        while (loop.size() <= maxEdges) {
            if (cur == start) { closed = true; break; }
            loop.push_back(cur);
            // on along an open edge not yet walked
            uint32_t to = cur;
            auto range = next.equal_range(cur);
            for (auto it = range.first; it != range.second; ++it)
                if (!used.count(hkey(cur, it->second))) { to = it->second; break; }
            if (to == cur) break;
            used[hkey(cur, to)] = 1;
            cur = to;
        }
        if (!closed || loop.size() < 3) continue;
        Vec3 c(0, 0, 0);
        for (uint32_t v : loop) c = c + mesh.vertices[v].position;
        Vertex mid = mesh.vertices[loop.front()];
        mid.position = project(c * (1.0 / loop.size()));
        mesh.vertices.push_back(mid);
        const uint32_t m = static_cast<uint32_t>(mesh.vertices.size() - 1);
        for (std::size_t i = 0; i < loop.size(); ++i) {   // (the face beside a -> b runs it so; the fill runs b -> a)
            const uint32_t a = loop[i], b = loop[(i + 1) % loop.size()];
            mesh.indices.push_back(b);
            mesh.indices.push_back(a);
            mesh.indices.push_back(m);
        }
        ++filled;
    }
    return filled;
}

}  // namespace engine
