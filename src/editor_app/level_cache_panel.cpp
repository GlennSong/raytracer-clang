#include "level_cache_panel.h"

#include <QColor>
#include <QFileInfo>

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

QString humanBytes(uint64_t bytes) {
    const char* units[] = {"B", "KB", "MB", "GB", "TB"};
    double v = static_cast<double>(bytes); int u = 0;
    while (v >= 1024.0 && u < 4) { v /= 1024.0; ++u; }
    return u == 0 ? QString("%1 B").arg(bytes) : QString("%1 %2").arg(v, 0, 'f', v < 10.0 ? 1 : 0).arg(units[u]);
}

QString bakedAgo(const QString& isoCreated, const QDateTime& now) {
    QDateTime t = QDateTime::fromString(isoCreated, Qt::ISODate);
    if (!t.isValid()) return QString();
    const qint64 s = std::max<qint64>(0, t.secsTo(now));
    if (s < 90) return "just now";
    if (s < 90 * 60) return QString("%1 min ago").arg((s + 30) / 60);
    if (s < 36 * 3600) return QString("%1 h ago").arg((s + 1800) / 3600);
    return QString("%1 days ago").arg((s + 43200) / 86400);
}

LevelCachePanel::LevelCachePanel(QWidget* parent) : QWidget(parent) {
    auto* col = new QVBoxLayout(this);
    heading = new QLabel(this); heading->setTextFormat(Qt::RichText);
    status = new QLabel(this); status->setTextFormat(Qt::RichText);
    detail = new QLabel(this); detail->setWordWrap(true); detail->setTextInteractionFlags(Qt::TextSelectableByMouse);
    olderLine = new QLabel(this); olderLine->setWordWrap(true);
    cacheLine = new QLabel(this); cacheLine->setWordWrap(true);
    auto* top = new QHBoxLayout; top->addWidget(heading, 1);
    refreshButton = new QPushButton("Refresh", this); top->addWidget(refreshButton);
    col->addLayout(top); col->addWidget(status); col->addWidget(detail);

    auto* grid = new QGridLayout;
    buildButton = new QPushButton("Build", this); buildButton->setToolTip("Bake the level now (only when it is out of date or not baked)");
    rebuildButton = new QPushButton("Rebuild", this); rebuildButton->setToolTip("Bake again even though the cache is current");
    showButton = new QPushButton("Show in folder", this); showButton->setToolTip("Open the bake's folder in the file viewer");
    deleteAllButton = new QPushButton("Delete", this); deleteAllButton->setToolTip("Delete every bake of this level, current and out of date");
    grid->addWidget(buildButton, 0, 0); grid->addWidget(rebuildButton, 0, 1);
    grid->addWidget(showButton, 1, 0); grid->addWidget(deleteAllButton, 1, 1);
    col->addLayout(grid);
    col->addWidget(olderLine);
    olderList = new QTreeWidget(this);
    olderList->setColumnCount(4);
    olderList->setHeaderLabels({"Baked", "Age", "Size", "Folder"});
    olderList->setRootIsDecorated(false);
    olderList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    olderList->header()->setStretchLastSection(true);
    olderList->setToolTip("Double-click a bake to open its folder");
    col->addWidget(olderList, 1);
    auto* olderButtons = new QHBoxLayout;
    showSelectedButton = new QPushButton("Show selected", this);
    deleteSelectedButton = new QPushButton("Delete selected", this);
    deleteOlderButton = new QPushButton("Delete all out-of-date", this); deleteOlderButton->setToolTip("Delete this level's older bakes; keep the current one");
    olderButtons->addWidget(showSelectedButton); olderButtons->addWidget(deleteSelectedButton); olderButtons->addWidget(deleteOlderButton);
    col->addLayout(olderButtons);
    // ALL LEVELS, visibly apart from this one (#88: "I thought it meant the river town one was 51 GB")
    auto* rule = new QFrame(this); rule->setFrameShape(QFrame::HLine); rule->setFrameShadow(QFrame::Sunken);
    col->addSpacing(6);
    col->addWidget(rule);
    auto* allHeading = new QLabel("<b>All levels</b>", this);
    col->addWidget(allHeading);
    col->addWidget(cacheLine);
    pruneButton = new QPushButton("Prune stale bakes (all levels)", this); pruneButton->setToolTip("Delete every bake superseded by a newer bake of the same level (rt_bake --prune)");
    col->addWidget(pruneButton);
    allBakesButton = new QPushButton("All bakes...", this);
    allBakesButton->setToolTip("Every bake in the cache: which level it belongs to, whether it is current, its size");
    col->addWidget(allBakesButton);

    auto call = [](const std::function<void()>& f) { if (f) f(); };
    QObject::connect(refreshButton, &QPushButton::clicked, [this, call]() { call(onRefresh); });
    QObject::connect(buildButton, &QPushButton::clicked, [this, call]() { call(onBuild); });
    QObject::connect(rebuildButton, &QPushButton::clicked, [this, call]() { call(onRebuild); });
    QObject::connect(deleteOlderButton, &QPushButton::clicked, [this, call]() { call(onDeleteOlder); });
    QObject::connect(deleteAllButton, &QPushButton::clicked, [this, call]() { call(onDeleteAll); });
    QObject::connect(showButton, &QPushButton::clicked, [this]() { if (onShowDir && !folderToShow().isEmpty()) onShowDir(folderToShow()); });
    QObject::connect(showSelectedButton, &QPushButton::clicked, [this]() { if (onShowDir) for (const QString& d : selectedOlder()) onShowDir(d); });
    QObject::connect(deleteSelectedButton, &QPushButton::clicked, [this]() { const QStringList d = selectedOlder(); if (onDeleteDirs && !d.isEmpty()) onDeleteDirs(d); });
    QObject::connect(olderList, &QTreeWidget::itemDoubleClicked, [this](QTreeWidgetItem* it, int) { if (onShowDir && it) onShowDir(it->data(0, Qt::UserRole).toString()); });
    QObject::connect(olderList, &QTreeWidget::itemSelectionChanged, [this]() {
        const bool any = !selectedOlder().isEmpty();
        showSelectedButton->setEnabled(any);
        deleteSelectedButton->setEnabled(any && !view_.baking);
    });
    QObject::connect(pruneButton, &QPushButton::clicked, [this, call]() { call(onPruneAll); });
    QObject::connect(allBakesButton, &QPushButton::clicked, [this, call]() { call(onAllBakes); });
    setView(LevelCacheView{});
}

QString LevelCachePanel::folderToShow() const {
    if (view_.current) return view_.currentDir;
    if (!view_.older.empty()) return view_.older.front().dir;
    return view_.root;
}

QStringList LevelCachePanel::selectedOlder() const {
    QStringList dirs;
    for (QTreeWidgetItem* it : olderList->selectedItems()) dirs << it->data(0, Qt::UserRole).toString();
    return dirs;
}

void LevelCachePanel::setView(const LevelCacheView& v, const QDateTime& now) {
    view_ = v;
    // the level's name and what ITS bakes weigh, together, so the all-levels total below can't be read as it
    const uint64_t mine = (v.current ? v.currentBytes : 0) + v.olderBytes;
    heading->setText(v.level.isEmpty() ? QString("<b>Level cache</b>")
                                       : QString("<b>%1</b>%2").arg(v.level.toHtmlEscaped(),
                                                                    mine > 0 ? QString(" &mdash; %1 baked").arg(humanBytes(mine)) : QString()));
    if (!v.error.isEmpty()) {
        status->setText("<span style='color:#c44'>Can't read the level</span>");
        detail->setText(v.error);
    } else if (!v.applies) {
        status->setText("Nothing to bake");
        detail->setText("This level has no city, lanes or other baked content; it always loads from source.");
    } else if (v.current) {
        const QString ago = bakedAgo(v.currentCreated, now);
        status->setText("<span style='color:#3a3'>● Current</span> — loads from the bake");
        detail->setText(QString("%1, baked %2\n%3").arg(humanBytes(v.currentBytes), ago.isEmpty() ? v.currentCreated : ago, v.currentDir));
    } else if (!v.older.empty()) {
        const QString ago = bakedAgo(v.older.front().created, now);
        status->setText("<span style='color:#c80'>● Out of date</span> — the level, its inputs or the engine changed");
        detail->setText(QString("Last baked %1; the next load rebuilds from source (or Build now).").arg(ago.isEmpty() ? v.older.front().created : ago));
    } else {
        status->setText("<span style='color:#888'>● Not baked</span> — loads from source");
        detail->setText(QString("Build writes it to %1").arg(v.root));
    }
    const int n = static_cast<int>(v.older.size());
    olderLine->setText(n == 0 ? QString("No out-of-date bakes of this level")
                              : QString("%1 out-of-date bake%2 of this level: %3").arg(n).arg(n == 1 ? "" : "s").arg(humanBytes(v.olderBytes)));
    olderList->clear();
    for (const OlderBake& b : v.older) {
        const QDateTime t = QDateTime::fromString(b.created, Qt::ISODate);
        auto* it = new QTreeWidgetItem(olderList, {t.isValid() ? t.toLocalTime().toString("yyyy-MM-dd HH:mm") : b.created, bakedAgo(b.created, now), humanBytes(b.bytes), b.dir});
        it->setData(0, Qt::UserRole, b.dir);
        it->setTextAlignment(2, Qt::AlignRight | Qt::AlignVCenter);
    }
    for (int c = 0; c < 3; ++c) olderList->resizeColumnToContents(c);
    olderList->setVisible(n > 0);
    showSelectedButton->setVisible(n > 0); deleteSelectedButton->setVisible(n > 0);
    showSelectedButton->setEnabled(false); deleteSelectedButton->setEnabled(false);
    cacheLine->setText(QString("%1 bake%2 across all levels, %3 in total; %4 stale (%5)")
                           .arg(v.cacheBundles).arg(v.cacheBundles == 1 ? "" : "s").arg(humanBytes(v.cacheBytes))
                           .arg(v.cacheStale).arg(humanBytes(v.cacheStaleBytes)));

    const bool idle = !v.baking && v.error.isEmpty();
    buildButton->setEnabled(idle && v.applies && !v.current);
    rebuildButton->setEnabled(idle && v.applies && v.current);
    showButton->setEnabled(!folderToShow().isEmpty());
    deleteAllButton->setEnabled(idle && (v.current || !v.older.empty()));
    deleteOlderButton->setVisible(n > 0);
    deleteOlderButton->setEnabled(idle && n > 0);
    pruneButton->setEnabled(!v.baking && v.cacheStale > 0);
    refreshButton->setEnabled(!v.baking);
}

// ---- All bakes ---------------------------------------------------------------------------------------------
namespace {
// Sorts Size and Baked by value (bytes, time), not by their text ("9 GB" < "10 GB")
class BakeItem : public QTreeWidgetItem {
public:
    using QTreeWidgetItem::QTreeWidgetItem;
    bool operator<(const QTreeWidgetItem& o) const override {
        const int c = treeWidget() ? treeWidget()->sortColumn() : 0;
        const QVariant a = data(c, Qt::UserRole + 1), b = o.data(c, Qt::UserRole + 1);
        if (a.isValid() && b.isValid()) return a.toULongLong() < b.toULongLong();
        return text(c).localeAwareCompare(o.text(c)) < 0;
    }
};
enum { kColLevel = 0, kColState, kColSize, kColBaked, kColFolder };
}  // namespace

AllBakesWindow::AllBakesWindow(QWidget* parent) : QDialog(parent) {
    setWindowTitle("All bakes");
    resize(900, 480);
    auto* col = new QVBoxLayout(this);
    summary = new QLabel(this);
    col->addWidget(summary);
    table = new QTreeWidget(this);
    table->setColumnCount(5);
    table->setHeaderLabels({"Level", "Status", "Size", "Baked", "Folder"});
    table->setRootIsDecorated(false);
    table->setSelectionMode(QAbstractItemView::ExtendedSelection);
    table->setSortingEnabled(true);
    table->sortByColumn(kColLevel, Qt::AscendingOrder);
    table->header()->setStretchLastSection(true);
    table->setToolTip("Double-click a bake to open its folder");
    col->addWidget(table, 1);
    auto* row = new QHBoxLayout;
    refreshButton = new QPushButton("Refresh", this);
    selectStaleButton = new QPushButton("Select out of date", this);
    selectStaleButton->setToolTip("Select every bake that is not its level's current one (out of date, level deleted, incomplete)");
    showButton = new QPushButton("Show in folder", this);
    openButton = new QPushButton("Open level", this);
    deleteButton = new QPushButton("Delete selected", this);
    row->addWidget(refreshButton);
    row->addWidget(selectStaleButton);
    row->addStretch(1);
    row->addWidget(showButton);
    row->addWidget(openButton);
    row->addWidget(deleteButton);
    col->addLayout(row);

    auto updateButtons = [this]() {
        const bool any = !selectedDirs().isEmpty();
        showButton->setEnabled(any);
        deleteButton->setEnabled(any);
        const QStringList lv = selectedLevels();
        openButton->setEnabled(lv.size() == 1 && !lv.front().isEmpty());
    };
    QObject::connect(table, &QTreeWidget::itemSelectionChanged, updateButtons);
    QObject::connect(refreshButton, &QPushButton::clicked, [this]() { if (onRefresh) onRefresh(); });
    QObject::connect(selectStaleButton, &QPushButton::clicked, [this]() { selectOutOfDate(); });
    QObject::connect(showButton, &QPushButton::clicked, [this]() { if (onShowDir) for (const QString& d : selectedDirs()) onShowDir(d); });
    QObject::connect(openButton, &QPushButton::clicked, [this]() { const QStringList lv = selectedLevels(); if (onOpenLevel && lv.size() == 1) onOpenLevel(lv.front()); });
    QObject::connect(deleteButton, &QPushButton::clicked, [this]() { const QStringList d = selectedDirs(); if (onDeleteDirs && !d.isEmpty()) onDeleteDirs(d); });
    QObject::connect(table, &QTreeWidget::itemDoubleClicked, [this](QTreeWidgetItem* it, int) { if (onShowDir && it) onShowDir(it->data(kColFolder, Qt::UserRole).toString()); });
    updateButtons();
}

void AllBakesWindow::setRows(const std::vector<BakeRow>& rows, bool checked, const QDateTime& now) {
    const QStringList keep = selectedDirs();
    table->setSortingEnabled(false);
    table->clear();
    uint64_t total = 0, stale = 0;
    int nCurrent = 0, nStale = 0, nOrphan = 0;
    for (const BakeRow& r : rows) {
        auto* it = new BakeItem(table);
        const QString name = r.level.isEmpty() ? QString("(unknown)") : QFileInfo(r.level).fileName();
        it->setText(kColLevel, name);
        it->setToolTip(kColLevel, r.level);
        it->setData(kColLevel, Qt::UserRole, r.level);
        const QString st = checked ? r.state : QString("checking...");
        it->setText(kColState, st);
        if (st == "current") it->setForeground(kColState, QColor(60, 170, 60));
        else if (checked) it->setForeground(kColState, QColor(210, 140, 0));
        it->setText(kColSize, humanBytes(r.bytes));
        it->setData(kColSize, Qt::UserRole + 1, QVariant::fromValue<qulonglong>(r.bytes));
        it->setTextAlignment(kColSize, Qt::AlignRight | Qt::AlignVCenter);
        const QDateTime t = QDateTime::fromString(r.created, Qt::ISODate);
        it->setText(kColBaked, t.isValid() ? t.toLocalTime().toString("yyyy-MM-dd HH:mm") + "  (" + bakedAgo(r.created, now) + ")" : r.created);
        it->setData(kColBaked, Qt::UserRole + 1, QVariant::fromValue<qulonglong>(t.isValid() ? static_cast<qulonglong>(t.toSecsSinceEpoch()) : 0));
        it->setText(kColFolder, r.dir);
        it->setData(kColFolder, Qt::UserRole, r.dir);
        if (keep.contains(r.dir)) it->setSelected(true);
        total += r.bytes;
        if (r.state == "current") ++nCurrent;
        else if (r.state == "level deleted") { ++nOrphan; stale += r.bytes; }
        else if (!r.state.isEmpty()) { ++nStale; stale += r.bytes; }
    }
    table->setSortingEnabled(true);
    for (int c = 0; c < 4; ++c) table->resizeColumnToContents(c);
    QString s = QString("%1 bake%2, %3 in total").arg(rows.size()).arg(rows.size() == 1 ? "" : "s").arg(humanBytes(total));
    if (checked)
        s += QString(" -- %1 current, %2 out of date, %3 of a deleted level (%4 reclaimable)").arg(nCurrent).arg(nStale).arg(nOrphan).arg(humanBytes(stale));
    else
        s += " -- checking which are current...";
    summary->setText(s);
    selectStaleButton->setEnabled(checked && nStale + nOrphan > 0);
}

QStringList AllBakesWindow::selectedDirs() const {
    QStringList d;
    for (QTreeWidgetItem* it : table->selectedItems()) d << it->data(kColFolder, Qt::UserRole).toString();
    return d;
}

QStringList AllBakesWindow::selectedLevels() const {
    QStringList l;
    for (QTreeWidgetItem* it : table->selectedItems()) l << it->data(kColLevel, Qt::UserRole).toString();
    return l;
}

void AllBakesWindow::selectOutOfDate() {
    table->clearSelection();
    for (int i = 0; i < table->topLevelItemCount(); ++i) {
        QTreeWidgetItem* it = table->topLevelItem(i);
        const QString st = it->text(kColState);
        if (st != "current" && st != "checking...") it->setSelected(true);
    }
}
