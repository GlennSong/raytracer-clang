#ifndef RAYTRACER_ENGINE_BUNDLE_CACHE_USE_H
#define RAYTRACER_ENGINE_BUNDLE_CACHE_USE_H

// WHICH LEVELS USE A CACHE FILE (ADR-0256). The terrain files (weather, maps, pyramid, world maps) and the
// population are named by a content hash, so levels that build the same ground share one file, and nothing in the
// name says whose it is. Each now has a record beside it, `<file>.uses`:
//     {"kind": "pyramid", "levels": {"/abs/assets/levels/river_town.json": 1761555600, ...}}
// stamped with the time the level's load began each time a load reads or writes the file. A file is stale once,
// for every level that used it, a newer file of its kind has been used since (the level moved on) or the level file
// is gone; `rt_bake --prune` deletes those. A file without a record (written before ADR-0256, or by a test) is never pruned here.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace engine {
namespace bundle {

// The level whose load is under way: later noteCacheUse calls name it, stamped with this call's time. loadLevel
// calls it first thing; an empty path stops the stamping.
void setCacheLevel(const std::string& levelPath);
// Record that the current level uses `file`, of `kind` ("weather", "maps", "pyramid", "wmap:normal", "population"...).
// No-op without a current level. Thread-safe.
void noteCacheUse(const std::string& file, const std::string& kind);

struct CacheUse {
    std::string kind;
    std::map<std::string, int64_t> levels;   // absolute level path -> the load that last used the file (unix s)
};
bool readCacheUse(const std::string& file, CacheUse& out);

// Housekeeping over cache directories (cache/terrain, cache/population). `owned` = it has a record; `stale` is the
// rule above. In-flight `*.tmp*` files are skipped. `apply` deletes the stale files with their records, and a
// record whose file is gone.
struct CacheFileEntry { std::string path, kind; uint64_t bytes = 0; bool owned = false, stale = false; std::vector<std::string> levels; };
struct CachePruneReport { std::vector<CacheFileEntry> files; uint64_t staleBytes = 0, unownedBytes = 0; size_t removed = 0; };
CachePruneReport pruneCacheFiles(const std::vector<std::string>& dirs, bool apply);

}  // namespace bundle
}  // namespace engine

#endif
