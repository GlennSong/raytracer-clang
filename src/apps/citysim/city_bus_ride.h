#ifndef RAYTRACER_APPS_CITYSIM_CITY_BUS_RIDE_H
#define RAYTRACER_APPS_CITYSIM_CITY_BUS_RIDE_H

// THE PLAYER ON A BUS (ADR-0486; the passenger seat ADR-0459 deleted with the old buses, back on the new ones). The bus's
// own rules -- where you get on, where you may stand, its seats, when its doors let you off -- are kept here; being
// carried in its frame is engine/riding.h's (a Passenger), as on the trains.
//   - ON: walk into one of its doorways from the pavement while it stands at a stop (CityBusRuns::standing). Or
//     `bus_board <bus>` (a probe's way in).
//   - ABOARD: the move keys walk the aisle and the door wells, in the bus's frame; every seat within reach is an E
//     offer (the interaction broker), and seated, E stands you up again. Sat, you face the way the seat faces.
//   - OFF: walk out through a doorway while the bus stands; you land in its mouth, and are not taken back on until you
//     have stepped out of it. Moved by something else (a teleport): off.
// The carrier is the bus as drawn this bake (CityRenderSystem::busCabinOf), so the bus and the view cannot disagree.

#include "city_render.h"

#include "../../engine/control/city_requests.h"
#include "../../engine/event_bus.h"
#include "../../engine/system.h"

namespace engine {
class PhysicsSystem;
class FlyCameraController;
}

namespace citysim {

// ---- The bus's rules, in its body frame (+Z forward, +X the kerb side where the doors are) -----------------------------
using BusCabin = CityRenderSystem::BusCabin;
// The floor (the doors' floor points' height).
engine::Real busFloor(const BusCabin& c);
// The door well `local` stands in (-1: none): within half a doorway of a door along the bus, between the aisle's middle
// and just past the door's floor point.
int busDoorWell(const BusCabin& c, const engine::Vec3& local);
// May a standing passenger be at `local`: in the aisle (between the seats, the saloon's length) or a door well.
bool busWalkable(const BusCabin& c, const engine::Vec3& local);
// Feet at `local` (the pavement, outside): standing in a doorway's mouth, at about the floor's height -- this gets you
// on. The door's index (-1: not at one).
int busDoorMouth(const BusCabin& c, const engine::Vec3& feetLocal);
// Past the wall in a doorway: this gets you off (when the bus stands). The door's index (-1: not out of one).
int busStepsOff(const BusCabin& c, const engine::Vec3& local);
// Where you stand inside door `door` on getting on, and outside it on getting off (body frame, the floor's height).
engine::Vec3 busInside(const BusCabin& c, int door);
engine::Vec3 busOutside(const BusCabin& c, int door);

class BusRideSystem : public engine::System {
public:
    BusRideSystem(CityRenderSystem& city, engine::PhysicsSystem& physics, engine::FlyCameraController& fly)
        : city_(city), physics_(physics), fly_(fly) {}
    void onStart(engine::FrameContext& ctx) override;    // hears BusBoardRequest
    void update(engine::FrameContext& ctx) override;     // the seat offers (the interaction broker's E)
    void fixedUpdate(engine::FrameContext& ctx) override;

    engine::Entity riding() const { return bus_; }   // the bus the player is on (none: on foot)
    const engine::Vec3& aboardAt() const { return local_; }
    int seat() const { return seat_; }

private:
    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    engine::FlyCameraController& fly_;
    engine::EventBus::Subscription onBoard_;
    bool boardPending_ = false;
    engine::BusBoardRequest board_;
    engine::Entity bus_{};
    engine::Entity leftBus_{};   // the bus last stepped off: not boarded again until you are out of its doorway
    engine::Vec3 local_{0, 0, 0};
    engine::Vec3 pinned_{0, 0, 0};
    int seat_ = -1;
    int sitRequest_ = -2;   // -2 none, -1 stand up, >= 0 sit on that seat
    void ride(engine::FrameContext& ctx, engine::Real dt);
    void getOff(engine::FrameContext& ctx, engine::Entity player);
};

}  // namespace citysim

#endif
