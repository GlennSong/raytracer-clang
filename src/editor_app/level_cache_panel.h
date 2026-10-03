#ifndef RAYTRACER_EDITOR_APP_LEVEL_CACHE_PANEL_H
#define RAYTRACER_EDITOR_APP_LEVEL_CACHE_PANEL_H

// The open level's baked cache (ADR-0084 bundles), as a dock panel (Glenn: "the editor should show if there is
// a bake for a level, how current it is, the size and have options to build it, rebuild it, delete it, go to
// file viewer to view it"). Pure Qt and Q_OBJECT-free like the rest of the editor shell: editor_main fills a
// LevelCacheView from engine::bundle::levelCacheStatus and does the work in the callbacks, so this builds into
// editor_qt_tests without the engine.

#include <QDateTime>
#include <QDialog>
#include <QString>
#include <QWidget>
#include <cstdint>
#include <functional>
#include <vector>

class QLabel;
class QPushButton;
class QTreeWidget;

struct OlderBake { QString dir, created; uint64_t bytes = 0; };   // one out-of-date bundle of the level

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
    std::vector<OlderBake> older;   // this level's out-of-date bundles, newest first
    uint64_t olderBytes = 0;
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

    QStringList selectedOlder() const;   // the dirs of the rows picked in the older-bakes list

    std::function<void()> onRefresh, onBuild, onRebuild, onDeleteOlder, onDeleteAll, onPruneAll, onAllBakes;
    std::function<void(const QString&)> onShowDir;             // Show in folder, or a double-clicked older bake
    std::function<void(const QStringList&)> onDeleteDirs;      // Delete selected (older bakes)

    QLabel* heading = nullptr;
    QLabel* status = nullptr;       // "Current" / "Out of date" / "Not baked"
    QLabel* detail = nullptr;       // size, age, where
    QLabel* olderLine = nullptr;    // this level's out-of-date bakes
    QTreeWidget* olderList = nullptr;   // one row each: baked, age, size, folder
    QPushButton* showSelectedButton = nullptr;
    QPushButton* deleteSelectedButton = nullptr;
    QLabel* cacheLine = nullptr;    // the whole cache
    QPushButton* refreshButton = nullptr;
    QPushButton* buildButton = nullptr;
    QPushButton* rebuildButton = nullptr;
    QPushButton* deleteOlderButton = nullptr;
    QPushButton* deleteAllButton = nullptr;
    QPushButton* showButton = nullptr;
    QPushButton* pruneButton = nullptr;
    QPushButton* allBakesButton = nullptr;

private:
    LevelCacheView view_;
};

// EVERY BAKE in the cache (Glenn: "it would be nice to see all of the bakes somewhere"): one row per bundle --
// its level, whether it is that level's current bake, size, when it was baked, its folder -- sortable, with
// Show / Open level / Delete on the selection. editor_main fills it from engine::bundle::listBakes.
struct BakeRow {
    QString level;     // the level file's path ("" when the bundle has no manifest)
    QString dir;       // the bundle directory
    QString created;   // ISO-8601 UTC
    uint64_t bytes = 0;
    QString state;     // "current", "out of date", "level deleted", "incomplete", or "" (still checking)
};

class QTreeWidget;
class AllBakesWindow : public QDialog {
public:
    explicit AllBakesWindow(QWidget* parent = nullptr);
    // `checked` false: the states are still being worked out (shown as "checking..." until the next call)
    void setRows(const std::vector<BakeRow>& rows, bool checked, const QDateTime& now = QDateTime::currentDateTimeUtc());
    QStringList selectedDirs() const;
    QStringList selectedLevels() const;
    void selectOutOfDate();   // every bake that is not a level's current one

    std::function<void()> onRefresh;
    std::function<void(const QString&)> onShowDir, onOpenLevel;
    std::function<void(const QStringList&)> onDeleteDirs;

    QTreeWidget* table = nullptr;
    QLabel* summary = nullptr;
    QPushButton* refreshButton = nullptr;
    QPushButton* selectStaleButton = nullptr;
    QPushButton* showButton = nullptr;
    QPushButton* openButton = nullptr;
    QPushButton* deleteButton = nullptr;
};

#endif
