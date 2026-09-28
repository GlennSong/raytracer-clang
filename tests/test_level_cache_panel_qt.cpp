// Level cache panel (ADR-0084 bundles in the editor): what it says and which actions it offers for each state
// of the open level's bake. Runs in editor_qt_tests' offscreen QApplication; returns its failure count.
#include "../src/editor_app/level_cache_panel.h"

#include <QLabel>
#include <QPushButton>
#include <cstdio>

namespace {
int lcFailures = 0;
#define LC_REQUIRE(cond) do { if (!(cond)) { std::printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++lcFailures; } } while (0)
}  // namespace

int runLevelCachePanelQtTests() {
    const QDateTime now = QDateTime::fromString("2026-09-28T12:00:00Z", Qt::ISODate);
    LC_REQUIRE(humanBytes(512) == "512 B" && humanBytes(8482560000ull) == "7.9 GB" && humanBytes(1536) == "1.5 KB");
    LC_REQUIRE(bakedAgo("2026-09-28T09:00:00Z", now) == "3 h ago" && bakedAgo("2026-09-25T12:00:00Z", now) == "3 days ago");
    LC_REQUIRE(bakedAgo("2026-09-28T11:59:30Z", now) == "just now" && bakedAgo("garbage", now).isEmpty());

    LevelCachePanel p;
    int builds = 0, rebuilds = 0, shows = 0;
    p.onBuild = [&]() { ++builds; }; p.onRebuild = [&]() { ++rebuilds; }; p.onShowFolder = [&]() { ++shows; };

    LevelCacheView v; v.level = "island.json"; v.applies = true; v.root = "/c/levels"; v.currentDir = "/c/levels/aaaa";
    v.cacheBundles = 3; v.cacheBytes = 3000; v.cacheStale = 0;
    p.setView(v, now);   // not baked
    LC_REQUIRE(p.status->text().contains("Not baked"));
    LC_REQUIRE(p.buildButton->isEnabled() && !p.rebuildButton->isEnabled() && !p.deleteAllButton->isEnabled() && !p.pruneButton->isEnabled());
    LC_REQUIRE(p.folderToShow() == "/c/levels");
    p.buildButton->click(); LC_REQUIRE(builds == 1);

    v.current = true; v.currentBytes = 8482560000ull; v.currentCreated = "2026-09-28T09:00:00Z";
    p.setView(v, now);   // current
    LC_REQUIRE(p.status->text().contains("Current") && p.detail->text().contains("7.9 GB") && p.detail->text().contains("3 h ago"));
    LC_REQUIRE(!p.buildButton->isEnabled() && p.rebuildButton->isEnabled() && p.deleteAllButton->isEnabled() && p.deleteOlderButton->isHidden());
    LC_REQUIRE(p.folderToShow() == "/c/levels/aaaa");
    p.rebuildButton->click(); p.showButton->click(); LC_REQUIRE(rebuilds == 1 && shows == 1);

    v.current = false; v.olderCount = 2; v.olderBytes = 2048; v.newestOlderDir = "/c/levels/bbbb"; v.newestOlderCreated = "2026-09-25T12:00:00Z";
    v.cacheStale = 1; v.cacheStaleBytes = 1024;
    p.setView(v, now);   // out of date
    LC_REQUIRE(p.status->text().contains("Out of date") && p.detail->text().contains("3 days ago"));
    LC_REQUIRE(p.buildButton->isEnabled() && !p.deleteOlderButton->isHidden() && p.deleteOlderButton->isEnabled() && p.pruneButton->isEnabled());
    LC_REQUIRE(p.olderLine->text().contains("2 out-of-date bakes") && p.olderLine->text().contains("2.0 KB"));
    LC_REQUIRE(p.folderToShow() == "/c/levels/bbbb");

    v.baking = true;
    p.setView(v, now);   // a bake is running: nothing else may start
    LC_REQUIRE(!p.buildButton->isEnabled() && !p.deleteAllButton->isEnabled() && !p.deleteOlderButton->isEnabled() && !p.pruneButton->isEnabled());

    LevelCacheView none; none.level = "arena.json"; none.applies = false;
    p.setView(none, now);
    LC_REQUIRE(p.status->text().contains("Nothing to bake") && !p.buildButton->isEnabled() && !p.rebuildButton->isEnabled());
    std::printf("[level cache panel] %s\n", lcFailures == 0 ? "ok" : "FAILED");
    return lcFailures;
}
