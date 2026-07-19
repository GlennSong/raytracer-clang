#ifndef RAYTRACER_ENGINE_RECENT_SCENES_H
#define RAYTRACER_ENGINE_RECENT_SCENES_H

#include <string>
#include <vector>

namespace engine {

class Settings;

// A most-recently-used list of scene (level) file paths, so the editor can
// offer "reopen a scene you had open recently" instead of making you remember
// where the file lives. The list is persisted inside the app's Settings under
// one key ("recentScenes"), newest first, de-duplicated, and capped — Settings
// only stores flat strings, so the paths are joined with '\n' into that one
// value (a path never contains a newline).
constexpr int kMaxRecentScenes = 12;

// The recent scenes, newest first. Empty if none have been recorded.
std::vector<std::string> loadRecentScenes(const Settings& settings);

// Move `path` to the front of the recent list (adding it if new), drop any
// earlier copy so a path appears once, and trim the list to kMaxRecentScenes.
// A blank path is ignored. This only updates the in-memory Settings; the app
// writes settings.json to disk on exit (and some panels save mid-run).
void recordRecentScene(Settings& settings, const std::string& path);

}  // namespace engine

#endif
