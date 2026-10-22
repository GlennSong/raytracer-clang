#ifndef RAYTRACER_APPS_CITYSIM_RAIL_SIM_H
#define RAYTRACER_APPS_CITYSIM_RAIL_SIM_H

// TRAINS (Glenn, 2026-10-10: "it makes sense if it follows the highway in a loop and there's a station. There should
// be trains going in each direction ... Maybe we should just have the train run 24 hours a day at an interval. There
// should be carriages and they should be like the bus where you can move around the carriages and between them and
// they shouldn't be jittery ... Start simple.")
//
// The road builder lays the track (a track class's lanes, rail_track.h); this runs trains on it. Each LINE is one
// track followed end to end -- the tracks of consecutive edges joined where one ends and the next begins -- so a
// double-track loop is two lines, one each way, and nothing ever crosses or switches. A train is a position along its
// line (its head's front), a speed and a stop it is heading for. It runs around the clock: up to the line speed and
// the speed a curve allows, braking for the next platform and for the train ahead, a dwell at each platform with its
// doors open on the platform's side. The timetable is the spacing: a line's trains start evenly spread, and the same
// running keeps them so.
//
// A carriage's pose is a function of the head's position alone (each car's front a fixed distance behind it, its body
// laid between two bogie points on the rail), so a renderer that interpolates the head between two steps moves every
// carriage -- and a rider in it -- along exactly the curve the rails draw. That is the whole anti-jitter argument.

#include "../../engine/procgen/city/rail_track.h"
#include "../../rt_math.h"

#include <functional>
#include <string>
#include <vector>

namespace citysim {

// What a train is made of and how it runs. Lengths are bodies; `gap` the coupling between them.
struct TrainKind {
    std::string name = "intercity";
    double locoLength = 19.0;     // the locomotive (a modern electric: no steam)
    double carLength = 24.0;      // each passenger carriage
    int cars = 3;                 // carriages behind the locomotive
    double gap = 0.9;             // coupler to coupler, where the gangway bridges
    double width = 2.9, floor = 1.1;   // body width; floor above the railhead (a platform's top)
    double vmax = 33.0;           // m/s (120 km/h) on the straight
    double accel = 0.7, decel = 0.8;   // service rates, m/s^2
    double lateral = 1.0;         // the curve speed: v^2 / r <= this
    double dwell = 30.0;          // seconds at a platform, doors open for all but the first and last few
    double length() const { return locoLength + cars * carLength + cars * gap; }
};

// The stock a line can run (Glenn: "subway, lightrail, amtrak ... a few carriages + the locomotive").
TrainKind trainKindNamed(const std::string& name);

struct RailLine {
    std::string name;
    std::string cls;                  // its track class (rail, light_rail, metro): what stock runs it
    std::vector<engine::Vec3> pts;    // railhead centreline, running order
    std::vector<double> s;            // station of each point
    double length = 0;
    bool loop = false;
    std::vector<engine::RailStop> stops;   // in this line's stations, sorted
    std::vector<double> vlim;         // per point: the speed the track allows (curves, braking ahead of them)
    // the railhead point and the running direction at station `st` (wrapped on a loop, clamped otherwise)
    engine::Vec3 at(double st) const;
    double wrap(double st) const;
    double speedLimit(double st) const;
};

// A line's colour on the map and the platform board: the main line charcoal, each light-rail or metro RING its own
// (both ways of a ring share it -- keyed by the ring's first station name in sort order).
engine::Vec3 lineColour(const RailLine& line);

struct Train {
    int line = -1;
    TrainKind kind;
    double s = 0;             // the head's front, along the line
    double prevS = 0;         // ...at the previous step, for drawing between steps
    double v = 0;
    int nextStop = 0;         // index into the line's stops
    bool dwelling = false;
    double dwellLeft = 0;
    int doorSide = 0;         // while the doors are open: +1 the train's left, -1 its right; 0 shut
    double odometer = 0;
};

// One carriage's place: where its body stands and which way it faces. Local frame: +z forward (toward the head),
// +x the train's LEFT, +y up; the origin on the railhead line under the middle of the body.
struct CarPose {
    engine::Mat4 m;
    double length = 0;
    bool loco = false;
};

class RailSim {
public:
    // Join `tracks` into lines (a track whose end meets another's start within `joinTol`, running on the same way,
    // continues into it). Every line gets `trainsPerLine` trains of `kind`, evenly spaced.
    void build(const std::vector<engine::RailTrack>& tracks, const TrainKind& kind, int trainsPerLine,
               double joinTol = 3.0);
    // ...each line with the stock its track class runs (`kindOf`) and as many trains as `countOf` gives for its length
    void build(const std::vector<engine::RailTrack>& tracks, const std::function<TrainKind(const std::string& cls)>& kindOf,
               const std::function<int(double length, const TrainKind& kind)>& countOf, double joinTol = 3.0);
    void step(double dt);

    const std::vector<RailLine>& lines() const { return lines_; }
    const std::vector<Train>& trains() const { return trains_; }
    // The poses of every car of train `ti`, the head drawn at `alpha` of the way from its last step to this one.
    std::vector<CarPose> carPoses(int ti, double alpha) const;
    // The same for one car (0 = the locomotive).
    CarPose carPose(int ti, int car, double alpha) const;
    int carCount(int ti) const { return 1 + trains_[static_cast<std::size_t>(ti)].kind.cars; }
    double headAt(int ti, double alpha) const;   // the head's station, interpolated (wrap-safe)
    std::string describe() const;                // `trains?`

private:
    double gapAhead(int ti) const;   // metres from this train's head to the tail of the one ahead on its line
    std::vector<RailLine> lines_;
    std::vector<Train> trains_;
};

}  // namespace citysim

#endif
