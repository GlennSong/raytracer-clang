#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_DRIVING_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_DRIVING_H

// THE REAL DRIVE SKILL, AND THE ROAD QUESTIONS (ADR-0440). Two small adapters between the brain's interfaces
// (brain_skills.h) and what already exists:
//
//   RoadPlaces    the road model's kerb parking spaces as places the brain can hold (ADR-0439's questions:
//                 RoadModel::frontageAt, kerbSpacesNear, kerbSpacePose, kerbLaneBeside). The road part of a World; a
//                 host's World adds its doors, its cars and its clock.
//   DrivingSkill  the brain's DriveSkill over the driving library's front door (apps/driving/driving.h), call for call,
//                 and the driving's events handed to the machine as the data's events of the same names.
//
//   LaneRoom      whether a lane is empty where a car leaving its kerb space would be put, looked at back along the
//                 paths that lead into it (ADR-0442).
//
// The city sim's side is city_brain.h (ADR-0441). NOT BUILT: beginning a drive part-way along its route (a person raised from
// far in mid-drive starts from wherever the car is placed, or from the start), and taking a car off the road when its
// driver stops being watched.

#include "brain.h"

#include "../../driving/driving.h"
#include "../../../engine/procgen/city/roads/road_model.h"

#include <string>
#include <vector>

namespace citysim {
namespace brain {

// Where a place meets the lanes: the kerb lane path beside it and how far along. What DrivingSkill asks.
class KerbLanes {
public:
    virtual ~KerbLanes() = default;
    virtual bool kerbLane(PlaceRef place, driving::PathPoint& out) const = 0;
};

class RoadPlaces : public KerbLanes {
public:
    // `tag` is OR-ed into every handle given out, so a host can tell these places from its own (its doors). The low
    // kIndexBits bits count the spaces handed out.
    static constexpr uint32_t kIndexBits = 24;
    RoadPlaces(const engine::RoadModel& model, uint32_t tag);

    bool owns(PlaceRef place) const;
    // The handle for a space: the same handle every time for the same space.
    PlaceRef placeOf(const engine::KerbSpace& space);
    bool spaceOf(PlaceRef place, engine::KerbSpace& out) const;
    // Where a car in the space stands (RoadModel::kerbSpacePose).
    bool positionOf(PlaceRef place, Vec2& out) const;
    // The kerb lane beside the space (RoadModel::kerbLaneBeside): where its car pulls out to and stops to turn in.
    bool kerbLane(PlaceRef place, driving::PathPoint& out) const override;
    // The kerb spaces near a door, nearest first by walking distance (RoadModel::frontageAt, then kerbSpacesNear), no
    // farther than `reach` metres on foot and at most `most`. The caller picks the first that is free.
    std::vector<PlaceRef> spacesNear(Vec2 door, Real reach, std::size_t most);

private:
    const engine::RoadModel& model_;
    uint32_t tag_;
    std::vector<engine::KerbSpace> spaces_;
};

class DrivingSkill : public DriveSkill {
public:
    // Every car is placed as vehicle class `vehicleClass` of the driving's table (one class for now).
    DrivingSkill(driving::Driving& driving, const KerbLanes& lanes, uint32_t vehicleClass);
    // The data's events the driving's six kinds of event arrive as, by name. False, with `err` naming brain.lua, when
    // the table lacks one.
    bool bindEvents(const BrainTable& table, std::string* err);

    bool pullOut(CarRef car, PlaceRef space) override;                                                  // placeCar
    bool beginTrip(CarRef car, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real fractionDone) override;   // beginTrip
    void endTrip(CarRef car) override;                                                                  // endTrip
    void park(CarRef car, PlaceRef space) override;                                                     // removeCar
    // changeDestination. When the driving has no route there it keeps the old one and reports "no route" itself; that
    // report is not passed on (the answer here is false, which is what the brain acts on).
    bool changeDestination(CarRef car, PlaceRef toSpace) override;

    // After the driving's tick: its events posted to the machine, each to the person at the wheel. An "ended early"
    // that this skill asked for itself (endTrip above) is not passed on: the brain already knows.
    void report(Machine& machine);
    // The same for a host that drains the driving's events itself and shares them out (the city sim's hand-over,
    // which other cars' drivers read too): `events` are the ones for this skill's cars, in the driving's order.
    void report(Machine& machine, const std::vector<driving::Event>& events);
    // The data's event one of the driving's kinds arrives as (bindEvents), for a host that must raise one itself (a
    // trip it could only begin later, and then could not).
    EventId eventFor(driving::EventKind kind) const { return events_[static_cast<uint8_t>(kind)]; }

private:
    driving::Driving& driving_;
    const KerbLanes& lanes_;
    uint32_t vehicleClass_;
    EventId events_[driving::kEventKindCount];
    std::vector<driving::CarId> endedHere_;   // cars whose trip this skill ended since the last report
    std::vector<driving::CarId> refusedHere_; // cars whose new destination the driving refused since the last report
};

// IS THERE ROOM TO PUT A CAR INTO A LANE (ADR-0442)? A car leaving a kerb space is put in the lane beside it with its
// front at a point of a lane path. That needs the lane empty from `behind` metres behind where the car's rear will be
// to `ahead` metres ahead of its front. BEHIND is measured back along the lane path and, where the path begins nearer
// than that, on back along EVERY path that leads into it (a turn out of a junction, and the lanes arriving at that
// turn), until the distance is used up: a car coming round the corner is as much in the way as one in the lane. AHEAD
// is measured on the lane path alone. "Empty" is the driving's own lists of whose body is on which path
// (traffic_state.h carsOnStretch), so a car's tail counts as well as its front.
//
// Made once for a road (it lists, for every path, the paths that lead into it; the road does not change).
class LaneRoom {
public:
    explicit LaneRoom(const driving::Driving& driving);
    bool clear(const driving::PathPoint& front, Real length, Real behind, Real ahead) const;
    // The paths that lead into `path`, in path order.
    std::vector<engine::PathId> pathsInto(engine::PathId path) const;

private:
    const driving::Driving& driving_;
    std::vector<uint32_t> firstInto_;   // by path: where its list begins in into_ (one more entry than paths)
    std::vector<engine::PathId> into_;
};

}  // namespace brain
}  // namespace citysim

#endif
