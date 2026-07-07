#include "debug_draw.h"

#include "mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

void DebugDraw::line(const Vec3& a, const Vec3& b, const Vec3& color,
                     Real seconds) {
    entries.push_back({a, b, color, seconds});
}

void DebugDraw::arrow(const Vec3& from, const Vec3& to, const Vec3& color,
                      Real seconds) {
    line(from, to, color, seconds);
    Vec3 dir = to - from;
    Real len = dir.length();
    if (len <= 0) return;
    dir = dir * (1.0 / len);
    // Two perpendicular directions for the four head fins.
    Vec3 ref = std::fabs(dir.y) < 0.99 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    Vec3 side = normalize(cross(dir, ref));
    Vec3 up = cross(side, dir);
    Real head = std::min(len * 0.25, static_cast<Real>(0.5)) * 0.5;
    Vec3 back = to - dir * (head * 2.0);
    line(to, back + side * head, color, seconds);
    line(to, back - side * head, color, seconds);
    line(to, back + up * head, color, seconds);
    line(to, back - up * head, color, seconds);
}

void DebugDraw::aabb(const Vec3& min, const Vec3& max, const Vec3& color,
                     Real seconds) {
    const Vec3 c[8] = {
        {min.x, min.y, min.z}, {max.x, min.y, min.z},
        {max.x, min.y, max.z}, {min.x, min.y, max.z},
        {min.x, max.y, min.z}, {max.x, max.y, min.z},
        {max.x, max.y, max.z}, {min.x, max.y, max.z},
    };
    static const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},   // bottom
        {4, 5}, {5, 6}, {6, 7}, {7, 4},   // top
        {0, 4}, {1, 5}, {2, 6}, {3, 7},   // verticals
    };
    for (const auto& e : edges) line(c[e[0]], c[e[1]], color, seconds);
}

void DebugDraw::box(const Vec3& center, const Vec3& halfExtent,
                    const Quat& orientation, const Vec3& color, Real seconds) {
    static const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };
    Vec3 c[8];
    int i = 0;
    for (int y = -1; y <= 1; y += 2)
        for (const auto& xz : {Vec3(-1, 0, -1), Vec3(1, 0, -1),
                               Vec3(1, 0, 1), Vec3(-1, 0, 1)}) {
            Vec3 local(xz.x * halfExtent.x, y * halfExtent.y,
                       xz.z * halfExtent.z);
            c[i++] = center + orientation.rotate(local);
        }
    for (const auto& e : edges) line(c[e[0]], c[e[1]], color, seconds);
}

void DebugDraw::sphere(const Vec3& center, Real radius, const Vec3& color,
                       Real seconds, int segments) {
    circle(center, Vec3(0, 0, 1), radius, color, seconds, segments);   // XY
    circle(center, Vec3(0, 1, 0), radius, color, seconds, segments);   // XZ
    circle(center, Vec3(1, 0, 0), radius, color, seconds, segments);   // YZ
}

void DebugDraw::circle(const Vec3& center, const Vec3& normal, Real radius,
                       const Vec3& color, Real seconds, int segments) {
    if (segments < 3) segments = 3;
    Vec3 n = normalize(normal);
    Vec3 ref = std::fabs(n.y) < 0.99 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    Vec3 u = normalize(cross(n, ref));
    Vec3 v = cross(n, u);
    Vec3 prev = center + u * radius;
    for (int i = 1; i <= segments; i++) {
        Real angle = 2.0 * M_PI * i / segments;
        Vec3 p = center + (u * std::cos(angle) + v * std::sin(angle)) * radius;
        line(prev, p, color, seconds);
        prev = p;
    }
}

void DebugDraw::axes(const Mat4& transform, Real size, Real seconds) {
    Vec3 origin = transform.transformPoint(Vec3(0, 0, 0));
    line(origin, transform.transformPoint(Vec3(size, 0, 0)),
         Vec3(0.9, 0.2, 0.2), seconds);
    line(origin, transform.transformPoint(Vec3(0, size, 0)),
         Vec3(0.2, 0.9, 0.2), seconds);
    line(origin, transform.transformPoint(Vec3(0, 0, size)),
         Vec3(0.2, 0.4, 0.9), seconds);
}

void DebugDraw::update(Real dt) {
    for (auto& entry : entries) entry.ttl -= dt;
}

void DebugDraw::endFrame() {
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [](const Line& l) { return l.ttl <= 0; }),
                  entries.end());
}

RenderMesh buildDebugLineMesh(const std::vector<DebugDraw::Line>& lines,
                              const Vec3& cameraPosition, Real pixelScale) {
    RenderMesh mesh;
    mesh.vertices.reserve(lines.size() * 4);
    mesh.indices.reserve(lines.size() * 6);
    for (const auto& l : lines) {
        Vec3 dir = l.b - l.a;
        Real len = dir.length();
        if (len <= 0) continue;
        dir = dir * (1.0 / len);

        Vec3 mid = (l.a + l.b) * 0.5;
        Vec3 toCam = cameraPosition - mid;
        Real dist = toCam.length();
        if (dist <= 0) continue;
        toCam = toCam * (1.0 / dist);

        // Ribbon side vector: perpendicular to both the line and the view
        // ray, so the quad faces the camera. A line pointing straight at the
        // camera has no such side — fall back to any perpendicular of dir.
        Vec3 side = cross(dir, toCam);
        Real sideLen = side.length();
        if (sideLen < 1e-6) {
            Vec3 ref = std::fabs(dir.y) < 0.99 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
            side = cross(dir, ref);
            sideLen = side.length();
        }
        side = side * (1.0 / sideLen);

        Real halfWidth = std::max(dist * pixelScale, static_cast<Real>(1e-4));
        Vec3 offset = side * halfWidth;
        // Perimeter order a..d; emitQuad winds the two triangles to front-face
        // `toCam` per the engine's clockwise convention.
        MeshBuilder::emitQuad(mesh, l.a - offset, l.a + offset, l.b + offset,
                              l.b - offset, toCam, l.color);
    }
    return mesh;
}

}  // namespace engine
