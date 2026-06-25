#include "mesh_builder.h"
#include <cmath>

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
    return mesh;
}

// ---------------------------------------------------------------------------
// Procgen-grade assembly ops (ROADMAP 3.3)
// ---------------------------------------------------------------------------

void MeshBuilder::append(RenderMesh& dst, const RenderMesh& src) {
    const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    dst.indices.reserve(dst.indices.size() + src.indices.size());
    for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
}

void MeshBuilder::appendTransformed(RenderMesh& dst, const RenderMesh& src,
                                    const Mat4& xform) {
    const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    const Mat4 normalMatrix = xform.inverse().transpose();
    dst.vertices.reserve(dst.vertices.size() + src.vertices.size());
    for (const Vertex& v : src.vertices) {
        Vertex out = v;
        out.position = xform.transformPoint(v.position);
        out.normal = normalize(normalMatrix.transformDirection(v.normal));
        out.tangent = normalize(xform.transformDirection(v.tangent));
        dst.vertices.push_back(out);
    }
    dst.indices.reserve(dst.indices.size() + src.indices.size());
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
    // Keep (a,b,c) when its outward normal cross(c-a,b-a) agrees with `normal`,
    // else swap b/c so the front face points the way the shading normal does.
    if (dot(cross(c - a, b - a), normal) >= 0)
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2});
    else
        mesh.indices.insert(mesh.indices.end(), {base, base + 2, base + 1});
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
    if (dot(cross(c - a, b - a), normal) >= 0)
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 1, base + 2, base, base + 2, base + 3});
    else
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 2, base + 1, base, base + 3, base + 2});
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
    if (dot(cross(c - a, b - a), normal) >= 0)
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 1, base + 2, base, base + 2, base + 3});
    else
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 2, base + 1, base, base + 3, base + 2});
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
            // Clockwise-front for an up-facing (+Y) surface.
            mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
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
