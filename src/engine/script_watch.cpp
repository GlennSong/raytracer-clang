#include "script_watch.h"

#include "level_loader.h"
#include "script_assets.h"
#include "../log.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <system_error>

namespace engine {

namespace {

long long fileStamp(const std::string& path) {
    std::error_code ec;
    const auto t = std::filesystem::last_write_time(path, ec);
    if (ec) return 0;
    return static_cast<long long>(t.time_since_epoch().count());
}

}  // namespace

void ScriptWatch::collect(const std::string& levelFile) {
    files_.clear();
#ifndef __EMSCRIPTEN__
    std::ifstream f(levelFile);
    if (!f) return;
    const nlohmann::json root = nlohmann::json::parse(f, nullptr, false);
    if (!root.is_object() || !root.value("watch_scripts", false)) return;

    auto add = [&](const std::string& path) {
        const long long s = fileStamp(path);
        if (s != 0) files_.push_back({path, s});
    };
    add(levelFile);   // level edits (opts/knobs) reload too

    // What the load ACTUALLY read, which is the only way a `require`d module
    // gets watched — the level JSON cannot know a recipe's dependencies.
    for (const std::string& path : LevelLoader::lastLoadedScriptFiles()) add(path);

    // Fall back to the JSON-derived list when the load recorded nothing, most
    // importantly when it FAILED — otherwise a recipe with a syntax error could
    // never be fixed without restarting the app.
    if (files_.size() <= 1) {
        const std::string levelDir =
            std::filesystem::path(levelFile).parent_path().string();
        for (const auto& ent : root.value("entities", nlohmann::json::array())) {
            if (ent.value("shape", std::string()) != "script") continue;
            const std::string file = ent.value("file", std::string());
            if (file.empty()) continue;
            const std::string path = resolveScriptPath(file, levelDir);
            if (!path.empty()) add(path);
        }
    }
    if (!files_.empty())
        LOG_INFO << "watch_scripts: reloading on changes to " << files_.size()
                 << " file(s)";
#else
    (void)levelFile;
#endif
}

bool ScriptWatch::changed() {
    bool dirty = false;
    for (Entry& e : files_) {
        const long long s = fileStamp(e.path);
        if (s != 0 && s != e.stamp) {
            e.stamp = s;
            dirty = true;
        }
    }
    return dirty;
}

bool ScriptWatch::tick(double framesPerCheck) {
    if (files_.empty()) return false;
    timer_ += 1.0;
    if (timer_ < framesPerCheck) return false;
    timer_ = 0.0;
    return changed();
}

}  // namespace engine
