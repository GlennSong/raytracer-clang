#include "level_cache_panel.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
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
    deleteOlderButton = new QPushButton("Delete out-of-date bakes", this); deleteOlderButton->setToolTip("Delete this level's older bakes; keep the current one");
    col->addWidget(deleteOlderButton);
    col->addWidget(cacheLine);
    pruneButton = new QPushButton("Prune stale bakes (all levels)", this); pruneButton->setToolTip("Delete every bake superseded by a newer bake of the same level (rt_bake --prune)");
    col->addWidget(pruneButton);
    col->addStretch(1);

    auto call = [](const std::function<void()>& f) { if (f) f(); };
    QObject::connect(refreshButton, &QPushButton::clicked, [this, call]() { call(onRefresh); });
    QObject::connect(buildButton, &QPushButton::clicked, [this, call]() { call(onBuild); });
    QObject::connect(rebuildButton, &QPushButton::clicked, [this, call]() { call(onRebuild); });
    QObject::connect(deleteOlderButton, &QPushButton::clicked, [this, call]() { call(onDeleteOlder); });
    QObject::connect(deleteAllButton, &QPushButton::clicked, [this, call]() { call(onDeleteAll); });
    QObject::connect(showButton, &QPushButton::clicked, [this, call]() { call(onShowFolder); });
    QObject::connect(pruneButton, &QPushButton::clicked, [this, call]() { call(onPruneAll); });
    setView(LevelCacheView{});
}

QString LevelCachePanel::folderToShow() const {
    if (view_.current) return view_.currentDir;
    if (view_.olderCount > 0) return view_.newestOlderDir;
    return view_.root;
}

void LevelCachePanel::setView(const LevelCacheView& v, const QDateTime& now) {
    view_ = v;
    heading->setText(v.level.isEmpty() ? QString("<b>Level cache</b>") : QString("<b>%1</b>").arg(v.level.toHtmlEscaped()));
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
    } else if (v.olderCount > 0) {
        const QString ago = bakedAgo(v.newestOlderCreated, now);
        status->setText("<span style='color:#c80'>● Out of date</span> — the level, its inputs or the engine changed");
        detail->setText(QString("Last baked %1; the next load rebuilds from source (or Build now).").arg(ago.isEmpty() ? v.newestOlderCreated : ago));
    } else {
        status->setText("<span style='color:#888'>● Not baked</span> — loads from source");
        detail->setText(QString("Build writes it to %1").arg(v.root));
    }
    olderLine->setVisible(v.olderCount > 0);
    olderLine->setText(QString("%1 out-of-date bake%2 of this level: %3").arg(v.olderCount).arg(v.olderCount == 1 ? "" : "s").arg(humanBytes(v.olderBytes)));
    cacheLine->setText(QString("Whole cache: %1 bake%2, %3; %4 stale (%5)")
                           .arg(v.cacheBundles).arg(v.cacheBundles == 1 ? "" : "s").arg(humanBytes(v.cacheBytes))
                           .arg(v.cacheStale).arg(humanBytes(v.cacheStaleBytes)));

    const bool idle = !v.baking && v.error.isEmpty();
    buildButton->setEnabled(idle && v.applies && !v.current);
    rebuildButton->setEnabled(idle && v.applies && v.current);
    showButton->setEnabled(!folderToShow().isEmpty());
    deleteAllButton->setEnabled(idle && (v.current || v.olderCount > 0));
    deleteOlderButton->setVisible(v.olderCount > 0);
    deleteOlderButton->setEnabled(idle && v.olderCount > 0);
    pruneButton->setEnabled(!v.baking && v.cacheStale > 0);
    refreshButton->setEnabled(!v.baking);
}
