#ifndef RAYTRACER_ENGINE_RIDING_H
#define RAYTRACER_ENGINE_RIDING_H

#include "components.h"
#include "system.h"

namespace engine {

class PhysicsWorld;
class FlyCameraController;

// RIDING (ADR-0243; Glenn, 2026-10-24: "How general of a thing should this component be made where we are going from
// one relative space to worldspace and back again? I suppose cars, trains, buses would use this. ... ships and dare I
// say it some kind of spaceship"). A rider is a Passenger: a point in a CARRIER's own frame. The system that owns the
// vehicle -- the bus and the car (CityPlayerTransitSystem), the train (TrainSystem) -- keeps the rules that are its
// own (who boards where, the aisle, the gangways, the seats, the doors) and hands the rest here:
//
//  rideStep()     each fixed step, after the owner set Passenger::carrier / local / sitting / eyeAbove: the capsule
//                 pinned at carrier x local, the step before kept (carrierPrev), what the footsteps and the body read
//                 (onFoot, facing), and the view turned with the carrier (a corner swings the saloon, not the world).
//  RidingSystem   each frame, after the fixed steps (its render; the update only notes where the camera was framed
//                 from): the rider's Transform on the carrier between its last two steps by this frame's
//                 interpolation -- exactly how the renderer draws the vehicle -- the view turned the rest of the way
//                 with it, and the camera, first person or third, carried by what the body moved since it was framed.
//
// It is the kinematic level: inside, the rider walks by the owner's bounds. A ship's deck with stairs would add
// collision in the carrier's frame (the carrier publishing its interior, the rider colliding in local space);
// nothing here assumes the carrier's up is the world's.

// The eye above the capsule centre (PlayerSystem::eyeHeight): a rider's centre sits that far under its eye.
constexpr Real kRiderEyeAboveCentre = 0.7;

// A rider at `local` in `carrier`: its eye, `eyeAbove` over that point along the carrier's own up, and its capsule's
// centre (upright, kRiderEyeAboveCentre under the eye).
Vec3 riderEye(const Mat4& carrier, const Vec3& local, Real eyeAbove);
Vec3 riderCentre(const Mat4& carrier, const Vec3& local, Real eyeAbove);

// One fixed step of a ride (see above). `fly` may be null (no view to turn: a test, a headless host).
void rideStep(World& world, PhysicsWorld* physics, Entity rider, FlyCameraController* fly);

// (its render runs before the other systems' renders that read the camera: registered early)
class RidingSystem : public System {
public:
    explicit RidingSystem(FlyCameraController* fly) : fly_(fly) {}
    void setFly(FlyCameraController* fly) { fly_ = fly; }
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

private:
    FlyCameraController* fly_;
};

}  // namespace engine

#endif
