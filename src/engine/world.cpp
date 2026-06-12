#include "world.h"

namespace engine {

Entity World::create() {
    return entities.insert(EntityMeta{});
}

void World::destroy(Entity entity) {
    if (!entities.erase(entity)) return;
    for (auto& entry : pools) entry.second->removeIndex(entity.index);
}

bool World::alive(Entity entity) const {
    return entities.contains(entity);
}

void World::destroyAll() {
    std::vector<Entity> live;
    entities.forEach([&](Entity e, EntityMeta&) { live.push_back(e); });
    for (Entity e : live) destroy(e);
}

}  // namespace engine

