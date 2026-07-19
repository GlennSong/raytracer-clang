#include "test_framework.h"

#include "../src/engine/recent_scenes.h"
#include "../src/renderer/settings.h"
#include <cstdio>

using namespace engine;

namespace {
const char* TMP_PATH = "test_recent_scenes_tmp.json";

struct TmpFile {  // removes the scratch file even when checks fail
    ~TmpFile() { std::remove(TMP_PATH); }
};
}  // namespace

TEST_CASE(recent_scenes_empty_by_default) {
    Settings s;
    CHECK(loadRecentScenes(s).empty());
}

TEST_CASE(recent_scenes_newest_first) {
    Settings s;
    recordRecentScene(s, "assets/levels/a.json");
    recordRecentScene(s, "assets/levels/b.json");
    recordRecentScene(s, "assets/levels/c.json");

    auto list = loadRecentScenes(s);
    CHECK(list.size() == 3);
    CHECK(list[0] == "assets/levels/c.json");
    CHECK(list[1] == "assets/levels/b.json");
    CHECK(list[2] == "assets/levels/a.json");
}

TEST_CASE(recent_scenes_reopening_moves_to_front_without_duplicating) {
    Settings s;
    recordRecentScene(s, "assets/levels/a.json");
    recordRecentScene(s, "assets/levels/b.json");
    recordRecentScene(s, "assets/levels/a.json");   // reopen the older one

    auto list = loadRecentScenes(s);
    CHECK(list.size() == 2);   // no duplicate 'a'
    CHECK(list[0] == "assets/levels/a.json");
    CHECK(list[1] == "assets/levels/b.json");
}

TEST_CASE(recent_scenes_caps_the_list) {
    Settings s;
    for (int i = 0; i < kMaxRecentScenes + 5; i++)
        recordRecentScene(s, "assets/levels/scene" + std::to_string(i) + ".json");

    auto list = loadRecentScenes(s);
    CHECK(static_cast<int>(list.size()) == kMaxRecentScenes);
    // The newest kMaxRecentScenes survive; the oldest fell off the end.
    CHECK(list.front() == "assets/levels/scene" +
                              std::to_string(kMaxRecentScenes + 4) + ".json");
    CHECK(list.back() == "assets/levels/scene5.json");
}

TEST_CASE(recent_scenes_ignores_blank) {
    Settings s;
    recordRecentScene(s, "");
    CHECK(loadRecentScenes(s).empty());
}

TEST_CASE(recent_scenes_clear_empties_the_list) {
    Settings s;
    recordRecentScene(s, "assets/levels/a.json");
    recordRecentScene(s, "assets/levels/b.json");
    CHECK(loadRecentScenes(s).size() == 2);

    clearRecentScenes(s);
    CHECK(loadRecentScenes(s).empty());
    // Clearing must not wedge the list — recording still works afterwards.
    recordRecentScene(s, "assets/levels/c.json");
    auto list = loadRecentScenes(s);
    CHECK(list.size() == 1);
    CHECK(list[0] == "assets/levels/c.json");
}

TEST_CASE(recent_scenes_survive_settings_round_trip) {
    TmpFile cleanup;
    {
        Settings s;
        recordRecentScene(s, "assets/levels/metropolis.json");
        recordRecentScene(s, "assets/levels/hillcity.json");
        s.save(TMP_PATH);
    }
    Settings reloaded;
    reloaded.load(TMP_PATH);
    auto list = loadRecentScenes(reloaded);
    CHECK(list.size() == 2);
    CHECK(list[0] == "assets/levels/hillcity.json");
    CHECK(list[1] == "assets/levels/metropolis.json");
}
