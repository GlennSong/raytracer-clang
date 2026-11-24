// A stamped model's shapes, markers and joints as debug lines (#191). See model_overlay.h.
#include "model_overlay.h"

#include "procgen/proc_model.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// A direction square to `axis` (unit), the same one each time for the same axis.
Vec3 squareTo(const Vec3& axis) {
    const Vec3 other = std::abs(axis.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    return normalize(cross(axis, other));
}

void capsule(DebugDraw& out, const ProcFrame& at, Real radius, Real halfHeight, const Vec3& colour) {
    // (its length runs along its own y)
    const Vec3 up = at.rotation.rotate(Vec3(0, 1, 0)), a = at.rotation.rotate(Vec3(1, 0, 0)), b = at.rotation.rotate(Vec3(0, 0, 1));
    const Vec3 top = at.position + up * halfHeight, bottom = at.position - up * halfHeight;
    out.sphere(top, radius, colour, 0, 16);
    out.sphere(bottom, radius, colour, 0, 16);
    for (const Vec3& side : {a, b, a * -1.0, b * -1.0}) out.line(top + side * radius, bottom + side * radius, colour);
}

}  // namespace

ModelOverlay& modelOverlay() {
    static ModelOverlay switches;
    return switches;
}

void drawModelOverlay(World& world, Entity instance, const ModelOverlay& what, DebugDraw& out) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    const Transform* at = world.get<Transform>(instance);
    if (mi == nullptr || mi->model == nullptr || at == nullptr || !what.any()) return;
    const ProcModel& model = *mi->model;
    const ProcFrame place{at->position, at->orientation};
    const Vec3 still(0.25, 0.95, 0.35), moving(1.0, 0.6, 0.15), yellow(1.0, 0.9, 0.2), white(0.95, 0.95, 0.95);

    if (what.shapes) {
        for (const ProcCollider& c : model.colliders) {
            const ProcFrame node = place * model.nodeFrame(c.node, mi->pose);
            const ProcFrame f = node * c.frame();
            const Vec3 colour = model.nodeMoves(c.node) ? moving : still;
            switch (c.kind) {
                case ProcCollider::Kind::Box: out.box(f.position, c.halfExtent, f.rotation, colour); break;
                case ProcCollider::Kind::Sphere: out.sphere(f.position, c.radius, colour); break;
                case ProcCollider::Kind::Capsule: capsule(out, f, c.radius, c.halfHeight, colour); break;
                case ProcCollider::Kind::Mesh:
                case ProcCollider::Kind::Hull: {
                    // the box round its points, in the space the points are in (a mesh's are the node's own)
                    if (c.mesh.vertices.empty()) break;
                    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
                    for (const Vertex& v : c.mesh.vertices) {
                        lo = Vec3(std::min<Real>(lo.x, v.position.x), std::min<Real>(lo.y, v.position.y), std::min<Real>(lo.z, v.position.z));
                        hi = Vec3(std::max<Real>(hi.x, v.position.x), std::max<Real>(hi.y, v.position.y), std::max<Real>(hi.z, v.position.z));
                    }
                    const ProcFrame in = c.kind == ProcCollider::Kind::Hull ? f : node;
                    out.box(in.point((lo + hi) * 0.5), (hi - lo) * 0.5, in.rotation, colour * 0.7);
                    break;
                }
            }
        }
    }

    for (std::size_t i = 0; i < model.nodes.size(); ++i) {
        const ProcNode& n = model.nodes[i];
        const ProcFrame node = place * model.nodeFrame(static_cast<int>(i), mi->pose);
        if (what.markers)
            for (const ProcMarker& mk : n.markers) out.axes((node * mk.frame).matrix(), std::max<Real>(0.25, mk.size));
        if (!what.joints || n.motion.kind == ProcMotion::Kind::None) continue;
        // the node as placed, before its own move: what the limits are measured from
        const ProcFrame parent = n.parent >= 0 ? place * model.nodeFrame(n.parent, mi->pose) : place;
        const ProcFrame rest = parent * n.frame;
        const Vec3 axis = normalize(rest.rotation.rotate(n.motion.axis));
        const Real value = n.motion.clamped(mi->pose.of(n.name));
        if (n.motion.kind == ProcMotion::Kind::Turn) {
            const Real r = 0.5;
            out.arrow(rest.position, rest.position + axis * 0.8, yellow);
            out.circle(rest.position, axis, r, yellow * 0.6);
            const Vec3 spoke = squareTo(n.motion.axis);
            auto spokeAt = [&](Real angle, const Vec3& colour, Real length) {
                out.line(rest.position, rest.point(Quat::fromAxisAngle(n.motion.axis, angle).rotate(spoke) * length), colour);
            };
            if (n.motion.limited) {
                spokeAt(n.motion.min, white, r);
                spokeAt(n.motion.max, white, r);
            }
            spokeAt(value, yellow, r * 1.25);
        } else {
            const Vec3 from = rest.position + axis * n.motion.min, to = rest.position + axis * n.motion.max;
            out.line(from, to, yellow * 0.6);
            const Vec3 across = rest.rotation.rotate(squareTo(n.motion.axis)) * 0.12;
            out.line(from - across, from + across, white);
            out.line(to - across, to + across, white);
            out.sphere(rest.position + axis * value, 0.06, yellow, 0, 12);
        }
    }
}

}  // namespace engine
