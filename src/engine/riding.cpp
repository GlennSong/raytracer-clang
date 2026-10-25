#include "riding.h"

#include "camera/fly_camera_controller.h"
#include "physics/physics_world.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
Vec3 flatForward(const Mat4& m) {
    Vec3 f(m.m[0][2], 0, m.m[2][2]);
    const Real l = std::sqrt(f.x * f.x + f.z * f.z);
    return l > 1e-6 ? f * (1.0 / l) : Vec3(0, 0, 1);
}

Entity theRider(World& world) {
    Entity rider;
    world.each<Transform, CharacterController, ControlledBy, Passenger>(
        [&](Entity e, Transform&, CharacterController&, ControlledBy&, Passenger&) {
            if (!rider.valid()) rider = e;
        });
    return rider;
}
}  // namespace

// The carrier's own up, in metres: its matrix's up column normalised (the matrix may carry a scale, so a height in
// its raw frame is not metres). A rider sits and stands in the carrier: the eye over the seat leans as the saloon
// pitches and rolls -- added straight up in the world it swayed against the cabin round it.
Vec3 carrierUp(const Mat4& carrier) {
    const Vec3 u(carrier.m[0][1], carrier.m[1][1], carrier.m[2][1]);
    const Real l = u.length();
    return l > 1e-6 ? u * (1.0 / l) : Vec3(0, 1, 0);
}

Vec3 riderEye(const Mat4& carrier, const Vec3& local, Real eyeAbove) {
    return carrier.transformPoint(local) + carrierUp(carrier) * eyeAbove;
}

Vec3 riderCentre(const Mat4& carrier, const Vec3& local, Real eyeAbove) {
    return riderEye(carrier, local, eyeAbove) - Vec3(0, kRiderEyeAboveCentre, 0);   // (the capsule stays upright)
}

Real headingDeg(const Mat4& m) {
    const Vec3 f = flatForward(m);
    return std::atan2(f.x, -f.z) * 57.29577951308232;
}

Real wrapDeg(Real d) {
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

void rideStep(World& world, PhysicsWorld* physics, Entity rider, FlyCameraController* fly) {
    Passenger* p = world.get<Passenger>(rider);
    Transform* t = world.get<Transform>(rider);
    if (!p || !t) return;
    // the step before, for the frame to draw between: none on boarding, nor across a jump (a gangway into the next
    // car is a new frame, not a 20 m glide)
    const bool jumped = !p->haveLast || (Vec3(p->carrier.m[0][3], p->carrier.m[1][3], p->carrier.m[2][3]) -
                                         Vec3(p->carrierLast.m[0][3], p->carrierLast.m[1][3], p->carrierLast.m[2][3]))
                                                .length() > 5.0;
    p->carrierPrev = jumped ? p->carrier : p->carrierLast;
    p->carrierLast = p->carrier;
    p->haveLast = true;
    const Vec3 at = riderCentre(p->carrier, p->local, p->eyeAbove);
    if (PrevTransform* prev = world.get<PrevTransform>(rider)) prev->value = *t;
    t->position = at;
    if (PrevTransform* prev = world.get<PrevTransform>(rider))
        if ((at - prev->value.position).length() > 2.0) prev->value.position = at;   // boarding, a seat change: no streak
    if (physics)
        if (const CharacterController* cc = world.get<CharacterController>(rider))
            if (cc->characterId != INVALID_CHARACTER) physics->setCharacterPosition(cc->characterId, at);
    p->onFoot = !p->sitting;   // (the footsteps: seated, silent)
    p->facing = flatForward(p->carrier);
    // the view turns WITH the carrier, a step at a time here (RidingSystem adds the rest of the way each frame)
    const Real yaw = headingDeg(p->carrier);
    if (fly && p->haveYaw) fly->yaw += wrapDeg(yaw - p->lastYaw);
    p->lastYaw = yaw;
    p->haveYaw = true;
}

// THE FRAME'S ORDER is update, then the fixed steps, then render: the camera is framed (first person's eye, third
// person's shoulder rig) in the update, from the body as the LAST frame's steps left it. So the update only notes
// where that was; the render -- after this frame's steps, with this frame's interpolation -- puts the body on the
// carrier as drawn and carries the camera on by exactly what the body moved and the carrier turned since it was
// framed. (Placing the rider in the update used the last frame's poses and fraction: a moving carrier a frame out.)
void RidingSystem::update(FrameContext& ctx) {
    const Entity rider = theRider(ctx.world);
    if (!rider.valid()) return;
    Passenger* p = ctx.world.get<Passenger>(rider);
    const Transform* t = ctx.world.get<Transform>(rider);
    if (!p || !t) return;
    p->framedAt = t->position;
    p->framedYaw = fly_ ? fly_->yaw : 0.0;
    p->framed = true;
}

void RidingSystem::render(FrameContext& ctx) {
    const Entity rider = theRider(ctx.world);
    if (!rider.valid()) return;
    Passenger* p = ctx.world.get<Passenger>(rider);
    Transform* t = ctx.world.get<Transform>(rider);
    if (!p || !t || !p->haveLast || !p->framed) return;
    p->framed = false;
    // the carrier as drawn: between its last two steps by this frame's interpolation (the renderer's blend of the
    // vehicle's instances -- InstanceGroup::ids -- and of every entity's PrevTransform)
    const Real a = std::clamp(ctx.interpolation, 0.0, 1.0);
    Mat4 drawn = p->carrier;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) drawn.m[r][c] = p->carrierPrev.m[r][c] + (p->carrier.m[r][c] - p->carrierPrev.m[r][c]) * a;
    const Vec3 at = riderCentre(drawn, p->local, p->eyeAbove);
    t->position = at;
    if (PrevTransform* prev = ctx.world.get<PrevTransform>(rider)) prev->value = *t;   // drawn where it is put
    // the view: the rest of the way round with the drawn carrier (rideStep turned it to the step's heading)
    const Real shown = wrapDeg(headingDeg(drawn) - headingDeg(p->carrier));
    if (fly_) fly_->yaw += wrapDeg(shown - p->yawShown);
    p->yawShown = shown;
    // THE FIRST-PERSON EYE, carried on rigidly from where it was framed: moved with the body and turned by the
    // carrier's turn since (measured: 0.00 mm against the saloon, from ~15 mm framed a frame late). A third-person
    // rig is left as framed: it springs toward its target with a velocity lead that already runs a frame ahead, and
    // carrying it as well overshot by a frame every other frame (71 mm, against 2.3 mm left alone).
    if ((at - p->framedAt).length() >= 2.0) return;   // a boarding or a step off is a jump: framed afresh next frame
    CameraState& cam = ctx.view.camera;
    const Vec3 framedEye = p->framedAt + Vec3(0, kRiderEyeAboveCentre, 0);
    if ((cam.position - framedEye).length() > 0.5) return;   // not the eye: a shoulder rig, or a placed camera
    const Real turn = fly_ ? (fly_->yaw - p->framedYaw) * 0.017453292519943295 : 0.0;
    const Real c = std::cos(turn), s = std::sin(turn);   // (yaw grows clockwise from above: -Z to +X)
    auto carried = [&](const Vec3& q) {
        const Vec3 r = q - p->framedAt;
        return at + Vec3(r.x * c - r.z * s, r.y, r.x * s + r.z * c);
    };
    cam.position = carried(cam.position);
    cam.target = carried(cam.target);
}

}  // namespace engine
