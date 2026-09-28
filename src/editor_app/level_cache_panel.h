#ifndef RAYTRACER_EDITOR_APP_LEVEL_CACHE_PANEL_H
#define RAYTRACER_EDITOR_APP_LEVEL_CACHE_PANEL_H

// The open level's baked cache (ADR-0084 bundles), as a dock panel (Glenn: "the editor should show if there is
// a bake for a level, how current it is, the size and have options to build it, rebuild it, delete it, go to
// file viewer to view it"). Pure Qt and Q_OBJECT-free like the rest of the editor shell: editor_main fills a
// LevelCacheView from engine::bundle::levelCacheStatus and does the work in the callbacks, so this builds into
// editor_qt_tests without the engine.

#include <QDateTime>
#include <QString>
#include <QWidget>
#include <cstdint>
#include <functional>

class QLabel;
class QPushButton;

struct LevelCacheView {
    QString level;            // file name, for the heading
    QString error;            // the level could not be read
    bool applies = false;     // the level has something to bake (a city, lanes, ...)
    bool current = false;     // a bundle for the level as it is today exists
    bool baking = false;      // a bake is running: every action waits
    QString root;             // the bundle root
    QString currentDir;       // where today's bake lives (or would)
    QString currentCreated;   // ISO-8601 UTC, from its manifest
    uint64_t currentBytes = 0;
    int olderCount = 0;       // this level's out-of-date bundles
    uint64_t olderBytes = 0;
    QString newestOlderDir, newestOlderCreated;
    int cacheBundles = 0, cacheStale = 0;   // the whole root
    uint64_t cacheBytes = 0, cacheStaleBytes = 0;
};

QString humanBytes(uint64_t bytes);                               // "7.9 GB"
QString bakedAgo(const QString& isoCreated, const QDateTime& now);   // "3 h ago", "2 days ago"; empty if unparsable

class LevelCachePanel : public QWidget {
public:
    explicit LevelCachePanel(QWidget* parent = nullptr);
    void setView(const LevelCacheView& v, const QDateTime& now = QDateTime::currentDateTimeUtc());
    const LevelCacheView& view() const { return view_; }
    QString folderToShow() const;   // the current bake, else the newest older one, else the root

    std::function<void()> onRefresh, onBuild, onRebuild, onDeleteOlder, onDeleteAll, onShowFolder, onPruneAll;

    QLabel* heading = nullptr;
    QLabel* status = nullptr;       // "Current" / "Out of date" / "Not baked"
    QLabel* detail = nullptr;       // size, age, where
    QLabel* olderLine = nullptr;    // this level's out-of-date bakes
    QLabel* cacheLine = nullptr;    // the whole cache
    QPushButton* refreshButton = nullptr;
    QPushButton* buildButton = nullptr;
    QPushButton* rebuildButton = nullptr;
    QPushButton* deleteOlderButton = nullptr;
    QPushButton* deleteAllButton = nullptr;
    QPushButton* showButton = nullptr;
    QPushButton* pruneButton = nullptr;

private:
    LevelCacheView view_;
};

#endif
