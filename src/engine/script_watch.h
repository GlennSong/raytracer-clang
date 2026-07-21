#ifndef RAYTRACER_ENGINE_SCRIPT_WATCH_H
#define RAYTRACER_ENGINE_SCRIPT_WATCH_H

#include <string>
#include <vector>

namespace engine {

// Recipe hot-reload bookkeeping, shared by every host that can rebuild a scene.
//
// It lived inside ArenaState, so `watch_scripts` worked in the viewer's play
// mode and nowhere else — in particular NOT in the Qt editor, which is where
// scenes are actually looked at. Rather than copy the poll loop into a second
// state (this codebase has already paid for duplicated script-path resolvers
// three times over), the mechanism moves here and both hosts drive it.
//
// The list is seeded from LevelLoader::lastLoadedScriptFiles() — the files the
// load genuinely opened — so a module pulled in by `require` is watched even
// though nothing in the level JSON mentions it.
class ScriptWatch {
public:
    // Seed from the last load. Call AFTER LevelLoader::load. `levelFile` is
    // watched too, so edits to a recipe's `opts` reload. No-op, leaving the
    // watch empty, unless the level sets "watch_scripts": true.
    void collect(const std::string& levelFile);

    // True when any watched file's mtime moved since the last call (and adopts
    // the new stamps, so it reports a given edit once).
    bool changed();

    bool empty() const { return files_.empty(); }
    std::size_t size() const { return files_.size(); }

    // Frame-counting poll: true roughly twice a second at 60 fps. Hosts call
    // this instead of each keeping their own timer.
    bool tick(double framesPerCheck = 30.0);

private:
    struct Entry {
        std::string path;
        long long stamp = 0;
    };
    std::vector<Entry> files_;
    double timer_ = 0.0;
};

}  // namespace engine

#endif
