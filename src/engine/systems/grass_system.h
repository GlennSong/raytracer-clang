#ifndef RAYTRACER_ENGINE_GRASS_SYSTEM_H
#define RAYTRACER_ENGINE_GRASS_SYSTEM_H

#include "../system.h"
#include "../world.h"

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace engine {

// Keeps the level's GrassField (components.h) planted around the camera: square tiles of
// instanced clumps, made as they come within the field's radius and dropped as they leave.
// A tile's clumps are a pure function of its coordinates and the field's seed, so a tile
// made again is the same tile -- the field never swims as the camera moves.
//
// Tiles are BUILT ON THE JOB SYSTEM, nearest first, and committed on the render thread: a
// tile's placement (ground, slope and density for ~2,800 clumps) cost 10-30 ms, and on the
// render thread that was the hitch in a walk. A tile changing ring keeps its old clumps until
// the new ones arrive, and tiles are built a tile beyond the field's radius so they are ready
// before they are due. The first frame builds everything at once.
class GrassSystem : public System {
public:
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    // One tile's clumps, per variant: what a job hands back.
    struct Built {
        std::pair<int, int> key;
        int lod = 0;
        uint64_t generation = 0;
        std::vector<std::vector<Mat4>> perVariant;
        Vec3 centre{0, 0, 0};
    };

private:
    struct Tile {
        int lod = -1;                 // the committed ring (-1: nothing yet)
        int building = -1;            // the ring in flight (-1: none)
        std::vector<Entity> groups;
    };
    struct Inbox {                    // shared with the jobs, so a late one never writes freed memory
        std::mutex m;
        std::vector<Built> done;
    };
    std::map<std::pair<int, int>, Tile> tiles_;
    std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
    uint64_t generation_ = 1;        // bumped on stop: results from before are dropped
    int inFlight_ = 0;
    bool first_ = true;
    bool commitAll_ = false;         // the first frame's commit: no budget
    void drop(World& world, Tile& t);
};

}  // namespace engine

#endif
