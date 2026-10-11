#ifndef RAYTRACER_APPS_CITYSIM_TRAIN_SYSTEM_H
#define RAYTRACER_APPS_CITYSIM_TRAIN_SYSTEM_H

// THE TRAINS, in the world (rail_sim.h runs them; this draws them and carries the player).
//
// It finds the RailTracks the road builder left on the level's road entity, runs a RailSim on them, and draws every
// car as an instance of its kind's body, glass and door leaves -- the head drawn between its last two steps, so every
// carriage moves along the rails' own curve at the frame's moment.
//
// ABOARD (Glenn: "like the bus where you can move around the carriages and between them and they shouldn't be
// jittery"): the bus's answer, carried over. The player is a PASSENGER standing at a floor point in one carriage's
// own frame, walked by the ordinary move keys within the aisle, the vestibules and the doorways; through the gangway
// into the next carriage (the point re-expressed in that car's frame -- both ends of a coupling are the same place on
// the rail); out of a door onto the platform while the doors are open on that side. The eye is put on that floor
// point in the carriage AS DRAWN this frame, in render(), after the steps -- so the carriage and the view cannot
// disagree, which is what jitter was on the bus.

#include "../../engine/system.h"
#include "../../engine/components.h"
#include "../../engine/procgen/vehicle/train_mesh.h"
#include "rail_sim.h"

#include <string>
#include <vector>

namespace engine {
class PhysicsSystem;
class FlyCameraController;
}

namespace citysim {

class TrainSystem : public engine::System {
public:
    TrainSystem(engine::PhysicsSystem& physics, engine::FlyCameraController& fly) : physics_(physics), fly_(fly) {}
    void fixedUpdate(engine::FrameContext& ctx) override;
    void render(engine::FrameContext& ctx) override;

    const RailSim& sim() const { return sim_; }
    int ridingTrain() const { return train_; }

private:
    void build(engine::FrameContext& ctx);
    void ride(engine::FrameContext& ctx, engine::Real dt);
    bool walkableAt(int car, const engine::Vec3& local, int doorSide) const;
    const engine::TrainBodyMesh& model(int car) const { return car == 0 ? loco_ : coach_; }

    engine::PhysicsSystem& physics_;
    engine::FlyCameraController& fly_;
    bool tried_ = false;
    RailSim sim_;
    engine::TrainBodyMesh loco_, coach_;
    // instance groups: locomotive body + glass, carriage body + glass, door leaves
    engine::Entity locoBody_, locoGlass_, coachBody_, coachGlass_, leaves_;
    int tick_ = 0;
    // aboard: which train and car, the floor point in that car's frame
    int train_ = -1, car_ = -1;
    engine::Vec3 local_{0, 0, 0};
    bool haveYaw_ = false;
    engine::Real lastYaw_ = 0;
    engine::Vec3 pinned_{0, 0, 0};
};

}  // namespace citysim

#endif
