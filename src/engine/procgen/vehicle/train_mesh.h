#ifndef RAYTRACER_ENGINE_PROCGEN_VEHICLE_TRAIN_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_VEHICLE_TRAIN_MESH_H

// TRAIN BODIES (Glenn, 2026-10-10: "passenger trains like a subway, lightrail, amtrak ... a few carriages + the
// locomotive. It should just look like a modern locomotive, so no steam engine stuff").
//
// One modern electric locomotive and one passenger carriage per kind, built in the body's own frame: +z forward,
// +x the train's LEFT, +y up, the origin on the railhead line under the middle of the body (rail_sim.h CarPose).
// A carriage is a hollow you can walk through: floor, seats, walls with window openings and door openings, end walls
// with a gangway through to the next car and the diaphragm that bridges the coupling. The door LEAVES are a mesh of
// their own (the renderer slides them along the body's skin when the doors open) and so is the glass.

#include "../../../renderer/renderer.h"
#include "../../../rt_math.h"

#include <string>
#include <vector>

namespace engine {

struct TrainBodySpec {
    std::string kind = "intercity";   // intercity | subway | light_rail
    double length = 24.0, width = 2.9, floor = 1.1, roof = 4.1;
    double gap = 0.9;                 // coupler to coupler: each end carries half of the diaphragm
    Vec3 livery{0.10, 0.45, 0.52};    // the stripe and the locomotive's nose
};

struct TrainBodyMesh {
    RenderMesh body;        // shell, frame, bogies, interior
    RenderMesh glass;       // window panes (clear)
    RenderMesh doorLeaf;    // ONE leaf, centred on its own origin, its face toward +x
    double length = 0, width = 0, floor = 0;
    // the walkable floor, in the body frame: a door's opening (z centre, half width), the aisle's half width, and
    // where seats are (hip points, facing +z or -z by their sign of w: w > 0 faces forward)
    std::vector<double> doorZ;
    double doorHalf = 0.65, doorHeight = 2.0;
    double aisleHalf = 0.4;
    double gangwayHalf = 0.55;
    std::vector<Vec3> seats;
    bool walkable = false;  // a carriage (true) or the locomotive (false: its cab is not ours)
};

TrainBodyMesh buildTrainCarriage(const TrainBodySpec& spec);
TrainBodyMesh buildTrainLocomotive(const TrainBodySpec& spec);

}  // namespace engine

#endif
