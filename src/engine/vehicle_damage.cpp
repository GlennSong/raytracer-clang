#include "vehicle_damage.h"
#include "mesh_builder.h"
#include <algorithm>
#include <cmath>

namespace engine {

const char* damageZoneName(DamageZone z) {
    switch (z) {
        case DamageZone::Front: return "front";
        case DamageZone::Rear: return "rear";
        case DamageZone::Left: return "left";
        case DamageZone::Right: return "right";
        case DamageZone::Roof: return "roof";
        case DamageZone::Under: return "under";
        default: return "?";
    }
}

DamageZone damageZone(const Vec3& local, const Vec3& h) {
    // the axis the point is furthest out along, each as a share of its half-extent; the ends win a tie with the
    // sides (a corner knock is a nose or a tail knock), and the roof only when the point is truly on top
    const Real ex = std::fabs(local.x) / std::max(Real(1e-3), h.x);
    const Real ey = local.y / std::max(Real(1e-3), h.y);
    const Real ez = std::fabs(local.z) / std::max(Real(1e-3), h.z);
    if (ey < -0.85 && ex < 0.9 && ez < 0.9) return DamageZone::Under;
    if (ey > 0.85 && ex < 0.9 && ez < 0.9) return DamageZone::Roof;
    if (ez >= ex * 0.92) return local.z >= 0 ? DamageZone::Front : DamageZone::Rear;
    return local.x >= 0 ? DamageZone::Left : DamageZone::Right;
}

Real touchHarm(const DamageRule& r, bool persisted, Real speed, Real slide, Real dt, Real mass) {
    const Real share = r.refMass / std::max(Real(200), mass);
    if (persisted) return std::max(Real(0), slide) * dt * r.scrapePerMetre * share;
    if (speed <= r.knockFrom) return 0;
    const Real t = std::min(Real(1), (speed - r.knockFrom) / std::max(Real(1e-3), r.knockFull - r.knockFrom));
    return std::pow(t, 1.3) * share;
}

int touchSparks(const DamageRule& r, bool persisted, Real speed, Real slide) {
    if (persisted) return slide >= r.sparkSlide ? std::min(6, 1 + static_cast<int>(slide / 4.0)) : 0;
    return speed >= r.sparkKnock ? std::min(40, static_cast<int>(speed * 1.5)) : 0;
}

bool dentMesh(RenderMesh& mesh, const Vec3& point, const Vec3& inward, Real radius, Real depth, Real scuff) {
    if (radius <= 0 || depth <= 0) return false;
    bool moved = false;
    const Real r2 = radius * radius;
    const Vec3 bare(0.30, 0.29, 0.28);   // scraped through to dark metal
    for (Vertex& v : mesh.vertices) {
        const Vec3 d = v.position - point;
        const Real dd = d.lengthSquared();
        if (dd >= r2) continue;
        const Real t = 1.0 - std::sqrt(dd) / radius;   // 1 at the point, 0 at the rim
        const Real k = t * t * (3.0 - 2.0 * t);
        v.position = v.position + inward * (depth * k);
        if (v.u <= -999.0f) {
            if (k > 0.5) {   // a lens crushed in: the lamp is broken
                v.u = 0.0f;
                v.color = v.color * 0.25;
            }
        } else if (scuff > 0) {
            const Real a = std::min(Real(1), k * scuff);
            v.color = v.color * (1.0 - a) + bare * a;
        }
        moved = true;
    }
    if (moved) MeshBuilder::recomputeNormals(mesh);
    return moved;
}

}  // namespace engine
