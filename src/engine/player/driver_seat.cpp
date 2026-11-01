#include "driver_seat.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {

namespace {

constexpr Real kDeg = 3.14159265358979 / 180.0;
constexpr Real kTau = 6.283185307179586;

}  // namespace

Real ejectTolerance(bool twoWheeler, bool openSeat, bool windscreen) {
    if (twoWheeler) return 7.0;
    if (openSeat) return 11.0;
    return windscreen ? 22.0 : std::numeric_limits<Real>::infinity();
}

Vec3 ejectThrow(const Vec3& before, bool hit) {
    return Vec3(before.x, std::max(Real(0), before.y), before.z) * (hit ? 0.8 : 0.6) + Vec3(0, hit ? 3.0 : 1.0, 0);
}

Vec3 seatGrip(const SeatGeometry& seat, int side, Real steer) {
    const Real sx = side == 0 ? 1.0 : -1.0;
    if (!seat.wheel) {
        // the bars turn about the head tube: one grip comes back as the other goes on
        const Real turn = std::clamp(steer, Real(-1), Real(1)) * 0.35;
        const Vec3 g(seat.grip.x * sx, seat.grip.y, seat.grip.z);
        return Vec3(g.x * std::cos(turn), g.y, g.z + g.x * std::sin(turn));
    }
    // the wheel: its centre between the hands, its face raked back 24 degrees; the hands at quarter to three, turned
    // with it as far as hands go before they would cross (the wheel turns further under them)
    const Vec3 centre(0, seat.grip.y, seat.grip.z);
    const Real r = std::abs(seat.grip.x);
    const Real rake = 24.0 * kDeg;
    const Vec3 across(1, 0, 0), up(0, std::cos(rake), -std::sin(rake));
    const Real a = (side == 0 ? 0.0 : 3.14159265358979) + std::clamp(steer, Real(-1), Real(1)) * 0.9;   // right: the left hand rises
    return centre + across * (r * std::cos(a)) + up * (r * std::sin(a));
}

Vec3 seatFoot(const SeatGeometry& seat, int side, Real crank) {
    const Real sx = side == 0 ? 1.0 : -1.0;
    if (!seat.pedals) return Vec3(seat.foot.x * sx, seat.foot.y, seat.foot.z);
    // round the crank, the two feet half a turn apart
    const Real a = kTau * (crank + (side == 0 ? 0.0 : 0.5));
    return Vec3(seat.foot.x * sx, seat.foot.y + seat.crank * std::cos(a), seat.foot.z + seat.crank * std::sin(a));
}

BodyPose seatedPose(const SeatGeometry& seat, const SeatSway& sway, Real steer, Real crank, Real scale) {
    BodyPose p;
    p.scale = scale;
    constexpr Real kShoulders = 0.55;   // the hip to the shoulders: what the lean turns the torso by
    const Real pitch = seat.lean * kDeg + std::atan(sway.lean.z / kShoulders);
    const Real roll = -std::atan(sway.lean.x / kShoulders);
    p.orientation = Quat::fromAxisAngle(Vec3(0, 0, 1), roll) * Quat::fromAxisAngle(Vec3(1, 0, 0), pitch);
    // the pelvis on the hip point
    p.position = p.orientation.rotate(BlockBody::parts()[BlockBody::Torso].joint * scale) * -1.0;
    // the head, on the neck (its offset taken into the body's frame)
    const Vec3 nod = p.orientation.conjugate().rotate(sway.nod);
    constexpr Real kNeck = 0.15;
    p.neck = Quat::fromAxisAngle(Vec3(0, 0, 1), -std::atan(nod.x / kNeck)) *
             Quat::fromAxisAngle(Vec3(1, 0, 0), std::atan(nod.z / kNeck));
    reachTo(p, BlockBody::ArmL, seatGrip(seat, 0, steer));
    reachTo(p, BlockBody::ArmR, seatGrip(seat, 1, steer));
    reachTo(p, BlockBody::LegL, seatFoot(seat, 0, crank));
    reachTo(p, BlockBody::LegR, seatFoot(seat, 1, crank));
    return p;
}

}  // namespace engine
