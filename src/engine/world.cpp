#include "world.h"

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
