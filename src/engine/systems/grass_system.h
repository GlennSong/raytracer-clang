#ifndef RAYTRACER_ENGINE_GRASS_SYSTEM_H
#define RAYTRACER_ENGINE_GRASS_SYSTEM_H

#include "../system.h"
#include "../world.h"

#include <map>
#include <utility>
#include <vector>

namespace engine {

// Keeps the level's GrassField (components.h) planted around the camera: square tiles of
// instanced clumps, made as they come within the field's radius and dropped as they leave.
// A tile's clumps are a pure function of its coordinates and the field's seed, so a tile
// made again is the same tile -- the field never swims as the camera moves. Tiles are built
// nearest first under a small per-frame budget (all at once on the first frame).
class GrassSystem : public System {
public:
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    struct Tile { int lod = 0; std::vector<Entity> groups; };
    std::map<std::pair<int, int>, Tile> tiles_;
    bool first_ = true;
    void drop(World& world, Tile& t);
};

}  // namespace engine

#endif
