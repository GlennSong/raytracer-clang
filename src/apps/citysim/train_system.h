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
#include "../../engine/audio/audio_engine.h"
#include "rail_sim.h"

#include <string>
#include <vector>

namespace engine {
class PhysicsSystem;
class FlyCameraController;
}

namespace citysim {

class CityRenderSystem;

class TrainSystem : public engine::System {
public:
    // `city`: the level's city, if it has one -- its people ride these trains (CitySim::setRail)
    TrainSystem(engine::PhysicsSystem& physics, engine::FlyCameraController& fly, CityRenderSystem* city = nullptr)
        : physics_(physics), fly_(fly), city_(city) {}
    void update(engine::FrameContext& ctx) override;   // the seat offers (the interaction broker's E)
    void fixedUpdate(engine::FrameContext& ctx) override;
    void render(engine::FrameContext& ctx) override;

    const RailSim& sim() const { return sim_; }
    int ridingTrain() const { return train_; }

private:
    void build(engine::FrameContext& ctx);
    void sounds(engine::FrameContext& ctx);   // clatter, rumble, horn, door chimes (sfx.h railClack ...)
    void ride(engine::FrameContext& ctx, engine::Real dt);
    bool walkableAt(int train, int car, const engine::Vec3& local, int doorSide) const;
    // THE STOCK each train is built from (its kind's locomotive and carriage), by train
    const engine::TrainBodyMesh& coachOf(int train) const { return stock_[static_cast<std::size_t>(stockOf_[static_cast<std::size_t>(train)])].coach; }
    const engine::TrainBodyMesh& model(int train, int car) const {
        const Stock& k = stock_[static_cast<std::size_t>(stockOf_[static_cast<std::size_t>(train)])];
        return car == 0 ? k.loco : k.coach;
    }

    std::vector<engine::Entity> entrances_;   // the station entrances' groups (station_entrance.h)
    engine::PhysicsSystem& physics_;
    engine::FlyCameraController& fly_;
    CityRenderSystem* city_ = nullptr;
    bool tried_ = false;
    RailSim sim_;
    // one per kind of train running (intercity on the main line, light rail in the cities): its models and instance
    // groups -- locomotive body + glass, carriage body + glass, door leaves
    struct Stock {
        std::string name;
        engine::TrainBodyMesh loco, coach;
        engine::Entity locoBody, locoGlass, coachBody, coachGlass, leaves;
    };
    std::vector<Stock> stock_;
    std::vector<int> stockOf_;      // per train
    engine::Entity riders_[3];      // the city's people in the seats (CitySim's train riders), three outfits
    std::vector<engine::Entity> boards_;   // the stations' name boards (station_boards.h): frames, then one group per name
    // THE SOUNDS: clips made once; per train its rumble voice and what it has already sounded
    bool audioTried_ = false;
    engine::AudioClipHandle clack_[3], rumble_, horn_, chimeOpen_, chimeShut_;
    struct TrainEar {
        engine::AudioVoiceHandle rumble;
        double lastOdo = -1;     // the odometer at the last frame: axles crossing a joint since then clack
        int hornedFor = -1;      // the stop it has sounded its horn for
        int lastDoor = 0;        // doorSide at the last frame
        bool beeped = false;     // the doors-closing warning, this dwell
    };
    std::vector<TrainEar> ears_;
    int tick_ = 0;
    // aboard: which train and car, the floor point in that car's frame
    int train_ = -1, car_ = -1;
    engine::Vec3 local_{0, 0, 0};
    int seat_ = -1;              // the seat held in car_ (its coach's seats), -1 standing
    int sitRequest_ = -2;        // from the broker: a seat to take, -1 stand up, -2 nothing
    engine::Vec3 pinned_{0, 0, 0};
};

}  // namespace citysim

#endif
