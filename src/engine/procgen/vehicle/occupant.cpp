#include "occupant.h"

#include "../../mesh_builder.h"

#include <cmath>

namespace engine {

namespace {

// A coloured box PITCHED about an arbitrary (y, z) pivot POINT.
//
// citysim's walker has the same idea but pivots only about z = 0, which is fine
// for a hip or shoulder on a standing body. A seated figure needs the general
// form: the knee is not on z = 0, and it MOVES with the thigh, so the shin has
// to rotate about wherever the thigh left it.
void pitchedBox(RenderMesh& out, const Vec3& size, const Vec3& centre,
                const Vec3& color, Real pivotY, Real pivotZ, Real angle) {
    RenderMesh b = MeshBuilder::box(size);
    const Real ca = std::cos(angle), sa = std::sin(angle);
    const std::uint32_t base = static_cast<std::uint32_t>(out.vertices.size());
    for (Vertex v : b.vertices) {
        Vec3 p = v.position + centre;
        const Real ry = p.y - pivotY, rz = p.z - pivotZ;
        p.y = pivotY + ry * ca - rz * sa;
        p.z = pivotZ + ry * sa + rz * ca;
        v.position = p;
        const Vec3 n = v.normal;
        v.normal = Vec3(n.x, n.y * ca - n.z * sa, n.y * sa + n.z * ca);
        v.color = color;
        out.vertices.push_back(v);
    }
    for (std::uint32_t i : b.indices) out.indices.push_back(base + i);
}

void box(RenderMesh& out, const Vec3& size, const Vec3& centre, const Vec3& color) {
    pitchedBox(out, size, centre, color, 0, 0, 0);
}

}  // namespace

RenderMesh buildSeatedOccupant(const OccupantParams& p) {
    RenderMesh m;

    // Torso and head recline about the hip by the SEAT's back angle, so the
    // occupant leans with the seat instead of sitting bolt upright inside it.
    // Negative because a positive pitch here swings the top FORWARD.
    const Real recline = -degreesToRadians(p.backAngle);
    const Real rise = degreesToRadians(p.thighRise);

    // Thighs: forward from the hip along +Z, angled slightly up to the knee.
    for (Real sx : {Real(-1), Real(1)})
        pitchedBox(m, Vec3(0.17, 0.17, p.thigh),
                   Vec3(sx * p.hipHalfWidth, 0, p.thigh * Real(0.5)), p.trousers,
                   0, 0, rise);

    // The knee, where the thigh actually ended — the shin pivots about THIS.
    const Real kneeY = p.thigh * std::sin(rise);
    const Real kneeZ = p.thigh * std::cos(rise);
    // Solve the shin's forward lean so the heel lands exactly H30 below the hip.
    const Real drop = std::min(kneeY + p.hipToHeel, p.lowerLeg * Real(0.999));
    const Real shinLean = -std::acos(std::max(drop / p.lowerLeg, Real(0)));
    for (Real sx : {Real(-1), Real(1)}) {
        pitchedBox(m, Vec3(0.15, p.lowerLeg, 0.15),
                   Vec3(sx * p.hipHalfWidth, kneeY - p.lowerLeg * Real(0.5), kneeZ),
                   p.trousers, kneeY, kneeZ, shinLean);
        // Foot at the heel the lean put us at, flat on the floor.
        const Real heelZ = kneeZ + p.lowerLeg * std::sin(-shinLean);
        box(m, Vec3(0.14, 0.07, 0.26),
            Vec3(sx * p.hipHalfWidth, -p.hipToHeel + Real(0.035), heelZ + Real(0.06)),
            p.trousers);
    }

    // Torso and head are sized to `sittingHeight`, NOT fixed. The leg segments
    // above are J826 data and stay put; sitting height is not in that data, and a
    // 1.45 m sedan with a 0.32 m floor only leaves ~0.78 m from hip to headliner.
    // Sizing to fit is why the driver's head stopped coming out of the roof.
    const Real torso = p.sittingHeight * Real(0.68);
    const Real headSize = p.sittingHeight * Real(0.30);
    pitchedBox(m, Vec3(0.44, torso, 0.26), Vec3(0, torso * Real(0.5), 0), p.shirt,
               0, 0, recline);
    pitchedBox(m, Vec3(headSize, headSize, headSize),
               Vec3(0, torso + headSize * Real(0.5), 0), p.skin, 0, 0, recline);

    // Arms reach forward to the wheel. The shoulder moved with the recline, so
    // the arm pitches about where it ended up, not about where it started.
    //
    // NEGATIVE armReach. pitchedBox swings +Y toward +Z, and the arm hangs BELOW
    // the shoulder (ry < 0), so a positive angle drives its far end to negative
    // z — backwards, toward the boot. The driver was reaching over his own
    // shoulders for the back seat.
    const Real shoulderStand = p.sittingHeight * Real(0.62);
    const Real shoulderY = shoulderStand * std::cos(recline);
    const Real shoulderZ = shoulderStand * std::sin(recline);
    for (Real sx : {Real(-1), Real(1)})
        pitchedBox(m, Vec3(0.12, 0.56, 0.16),
                   Vec3(sx * 0.25, shoulderY - Real(0.28), shoulderZ), p.shirt,
                   shoulderY, shoulderZ, degreesToRadians(-p.armReach));
    return m;
}

}  // namespace engine
