#include "occupant.h"

#include "../../block_body.h"
#include "../../player/driver_seat.h"
#include "../person.h"

#include <algorithm>

#include <cmath>

namespace engine {

// The jointed body (block_body.h) seated by the player's own seat rule (driver_seat.h seatedPose, ADR-0254): the
// manikin's numbers say where the seat puts the feet and the hands, and how far the back reclines.
OccupantParams chairOccupant() {
    OccupantParams p;
    p.hipToHeel = 0.45;
    p.sittingHeight = 0.86;
    p.backAngle = 8.0;
    p.armReach = 20.0;
    return p;
}

OccupantParams carPassenger() {
    OccupantParams p;
    p.armReach = 20.0;
    return p;
}

BodyPose seatedOccupantPose(const OccupantParams& p, Real bodyScale) {
    // Scaled to the cabin, never up: the manikin's torso and head took 0.98 of the sitting height along the
    // reclined back; the body's pelvis to the top of its head is 0.88 m.
    const Real headTop = BlockBody::parts()[BlockBody::Head].joint.y + BlockBody::parts()[BlockBody::Head].size.y -
                         BlockBody::parts()[BlockBody::Torso].joint.y;
    const Real scale = std::min(bodyScale, Real(0.98) * p.sittingHeight / headTop);
    SeatGeometry seat;
    seat.lean = -p.backAngle;   // (seatedPose: -ve leans back)
    seat.wheel = false;         // the hands where the arms reach, not round a wheel
    // The feet: the thigh out from the hip rising by thighRise, the shin leaning forward so the heel lands
    // hipToHeel under the hip (the manikin's solve, with the body's own bones).
    const BlockBody::PartSpec& thighSpec = BlockBody::parts()[BlockBody::LegL];
    const BlockBody::PartSpec& shinSpec = BlockBody::parts()[BlockBody::ShinL];
    const Real thigh = 2 * thighSpec.rest.length() * scale, shin = 2 * shinSpec.rest.length() * scale;
    const Real rise = degreesToRadians(p.thighRise);
    const Real kneeY = thigh * std::sin(rise), kneeZ = thigh * std::cos(rise);
    const Real drop = std::min(kneeY + p.hipToHeel, shin * Real(0.999));
    seat.foot = Vec3(thighSpec.joint.x * scale, kneeY - drop, kneeZ + std::sqrt(shin * shin - drop * drop));
    // The hands: nine tenths of the arm out from the shoulder at armReach from hanging, the elbow taking the rest.
    const BlockBody::PartSpec& armSpec = BlockBody::parts()[BlockBody::ArmL];
    const Real pitch = degreesToRadians(seat.lean);
    const Vec3 shoulderUp = (armSpec.joint - BlockBody::parts()[BlockBody::Torso].joint) * scale;
    const Vec3 shoulder(shoulderUp.x, shoulderUp.y * std::cos(pitch) - shoulderUp.z * std::sin(pitch),
                        shoulderUp.y * std::sin(pitch) + shoulderUp.z * std::cos(pitch));
    const Real arm = 4 * armSpec.rest.length() * scale;   // the upper arm and the forearm
    const Real reach = degreesToRadians(p.armReach);
    seat.grip = shoulder + Vec3(0, -std::cos(reach), std::sin(reach)) * (Real(0.9) * arm);
    return seatedPose(seat, SeatSway{}, 0, 0, scale);
}

RenderMesh buildSeatedOccupant(const OccupantParams& p) {
    PersonOutfit outfit;
    outfit.shirt = p.shirt;
    outfit.pants = p.trousers;
    outfit.skin = p.skin;
    return buildPosedPerson(seatedOccupantPose(p), outfit);
}

}  // namespace engine
