// The editor application shell (editor-app plan, Phase A2+A3): a Qt window
// hosting the engine viewport through the HostedWindow seam, with native
// panels — hierarchy, inspector, asset browser, toolbar — talking to the
// engine through the EditorBridge. The engine inside is the exact
// EditorState/ArenaState stack the standalone viewer runs, so the viewport
// behaves 1:1 with the game.
//
// Panels poll the bridge on a UI timer and rewrite widgets only when content
// changed; while playing (bridge detached) they gray out. Deliberately
// Q_OBJECT-free (no moc): event overrides + lambda connects only.

#include "../engine/application.h"
#include "../engine/components.h"
#include "../engine/editor_bridge.h"
#include "../engine/model_importer.h"
#include "../engine/recent_scenes.h"
#include "../engine/states/editor_state.h"
#include "../game/arena_state.h"
#include "../renderer/hosted_window.h"
#include "../renderer/gamepad_gc.h"
#include "../log.h"
#include "property_inspector.h"

// Vulkan viewport surface seam (ADR-0057). Present only on non-Apple targets
// where the Vulkan backend is compiled in (RT_HAVE_VULKAN). Qt extracts the
// native handles here; vulkan_viewport.cpp owns the Vulkan/Xlib headers.
#if defined(RT_HAVE_VULKAN)
#include "vulkan_viewport.h"
#include <QGuiApplication>
#if !defined(_WIN32) && __has_include(<QtGui/qguiapplication_platform.h>)
#include <QtGui/qguiapplication_platform.h>
#define RT_EDITOR_VK_X11 1
#endif
#endif

#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QCursor>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QDropEvent>
#include <QFile>
#include <QFileDialog>
#include <QHeaderView>
#include <QFileInfo>
#include <QFileSystemModel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QKeyEvent>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QHeaderView>
#include <QMouseEvent>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStatusBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QTreeView>
#include <QWheelEvent>
#include <QWidget>

#include <algorithm>
#include <cstring>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using namespace engine;

KeyCode mapQtKey(int key) {
    if (key >= Qt::Key_A && key <= Qt::Key_Z)
        return static_cast<KeyCode>(static_cast<int>(KeyCode::A) + (key - Qt::Key_A));
    if (key >= Qt::Key_0 && key <= Qt::Key_9)
        return static_cast<KeyCode>(static_cast<int>(KeyCode::Num0) + (key - Qt::Key_0));
    switch (key) {
        case Qt::Key_Up:        return KeyCode::Up;
        case Qt::Key_Down:      return KeyCode::Down;
        case Qt::Key_Left:      return KeyCode::Left;
        case Qt::Key_Right:     return KeyCode::Right;
        case Qt::Key_Escape:    return KeyCode::Escape;
        case Qt::Key_Space:     return KeyCode::Space;
        case Qt::Key_Return:
        case Qt::Key_Enter:     return KeyCode::Enter;
        case Qt::Key_Tab:       return KeyCode::Tab;
        case Qt::Key_Backspace: return KeyCode::Backspace;
        case Qt::Key_Shift:     return KeyCode::LeftShift;
        case Qt::Key_Control:   return KeyCode::LeftControl;
        case Qt::Key_Alt:       return KeyCode::LeftAlt;
        case Qt::Key_Comma:     return KeyCode::Comma;
        case Qt::Key_Period:    return KeyCode::Period;
        case Qt::Key_Slash:     return KeyCode::Slash;
        case Qt::Key_Semicolon: return KeyCode::Semicolon;
        case Qt::Key_Minus:     return KeyCode::Minus;
        case Qt::Key_Equal:     return KeyCode::Equal;
        case Qt::Key_BracketLeft:  return KeyCode::LeftBracket;
        case Qt::Key_BracketRight: return KeyCode::RightBracket;
        case Qt::Key_QuoteLeft: return KeyCode::GraveAccent;
        default:                return KeyCode::Unknown;
    }
}

MouseButton mapQtButton(Qt::MouseButton b) {
    if (b == Qt::RightButton) return MouseButton::Right;
    if (b == Qt::MiddleButton) return MouseButton::Middle;
    return MouseButton::Left;
}

// The engine's viewport pane: a native widget whose window handle the renderer
// binds to (NSView* on macOS), forwarding Qt input through the HostedWindow
// seam. Qt must not paint over the Metal layer, hence the paint-engine
// opt-outs.
class EngineViewport : public QWidget {
public:
    explicit EngineViewport(HostedWindow& hosted) : hosted(hosted) {
        setAttribute(Qt::WA_NativeWindow);
        setAttribute(Qt::WA_PaintOnScreen);
        setAttribute(Qt::WA_OpaquePaintEvent);
        setAttribute(Qt::WA_NoSystemBackground);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);
        setMinimumSize(320, 240);
    }

    QPaintEngine* paintEngine() const override { return nullptr; }

    // First-person capture (engine CursorMode::Disabled, e.g. Play). The
    // toolkit half of pointer lock, done WITHOUT grabMouse or move events:
    // both are unreliable on macOS (grabs need the cursor already inside
    // the widget; warping suppresses the events it would produce). Instead
    // the host's frame loop calls pollCapturedMouse(): read the global
    // cursor, inject the offset from the viewport center as a delta, warp
    // back to center. A global override cursor keeps it hidden wherever
    // the (pinned) pointer happens to sit.
    void setCaptured(bool on) {
        if (captured == on) return;
        captured = on;
        if (on) {
            setFocus();
            QGuiApplication::setOverrideCursor(Qt::BlankCursor);
            resyncCapture = true;   // first poll centers without a spike
        } else {
            QGuiApplication::restoreOverrideCursor();
        }
    }

    // Once per host frame, before the engine consumes input.
    void pollCapturedMouse() {
        if (!captured || !isVisible() || width() <= 0 || height() <= 0)
            return;
        // Don't fight the OS cursor while another window/app is in front
        // (cmd-tab, the save dialog); re-sync when we come back.
        if (!window()->isActiveWindow()) {
            resyncCapture = true;
            return;
        }
        const QPoint center = mapToGlobal(QPoint(width() / 2, height() / 2));
        if (resyncCapture) {
            resyncCapture = false;
            QCursor::setPos(center);
            return;
        }
        const QPoint delta = QCursor::pos() - center;
        if (delta.x() != 0 || delta.y() != 0) {
            hosted.injectMouseDelta(delta.x(), delta.y());
            QCursor::setPos(center);
        }
    }

protected:
    void resizeEvent(QResizeEvent*) override {
        const qreal scale = devicePixelRatioF();
        hosted.setSizes(width(), height(),
                        static_cast<int>(width() * scale),
                        static_cast<int>(height() * scale));
    }
    void mouseMoveEvent(QMouseEvent* e) override {
        if (captured) return;   // relative motion comes from the poll pump
        hosted.injectMouseMove(e->position().x(), e->position().y());
    }
    void mousePressEvent(QMouseEvent* e) override {
        setFocus();
        hosted.injectMouseButton(mapQtButton(e->button()), true,
                                 e->position().x(), e->position().y());
    }
    void mouseReleaseEvent(QMouseEvent* e) override {
        hosted.injectMouseButton(mapQtButton(e->button()), false,
                                 e->position().x(), e->position().y());
    }
    void wheelEvent(QWheelEvent* e) override {
        hosted.injectScroll(e->angleDelta().y() / 120.0);
    }
    void keyPressEvent(QKeyEvent* e) override {
        KeyCode key = mapQtKey(e->key());
        if (key != KeyCode::Unknown)
            hosted.injectKey(key, true, e->isAutoRepeat());
        else
            QWidget::keyPressEvent(e);
    }
    void keyReleaseEvent(QKeyEvent* e) override {
        KeyCode key = mapQtKey(e->key());
        if (key != KeyCode::Unknown && !e->isAutoRepeat())
            hosted.injectKey(key, false);
        else
            QWidget::keyReleaseEvent(e);
    }

private:
    HostedWindow& hosted;
    bool captured = false;
    bool resyncCapture = false;
};

// Engine log -> console dock. The sink fires on whatever thread logged
// (physics jobs included), so lines land in a locked buffer; the UI timer
// drains them on the Qt thread. Bounded by the view's max block count.
struct LogConsole {
    QPlainTextEdit* view = nullptr;
    std::mutex mutex;
    std::vector<std::pair<engine::logging::Level, QString>> pending;

    QDockWidget* buildDock(QMainWindow* main) {
        auto* dock = new QDockWidget("Console", main);
        view = new QPlainTextEdit(dock);
        view->setReadOnly(true);
        view->setMaximumBlockCount(2000);
        dock->setWidget(view);

        engine::logging::setSink(
            [this](engine::logging::Level level, const std::string& line) {
                std::lock_guard<std::mutex> lock(mutex);
                pending.emplace_back(level, QString::fromStdString(line));
            });
        return dock;
    }

    ~LogConsole() { engine::logging::setSink(nullptr); }

    void drain() {
        std::vector<std::pair<engine::logging::Level, QString>> lines;
        {
            std::lock_guard<std::mutex> lock(mutex);
            lines.swap(pending);
        }
        for (const auto& [level, text] : lines) {
            const char* tag = level == engine::logging::Level::Error ? "[ERROR] "
                              : level == engine::logging::Level::Warn ? "[WARN] "
                                                                      : "";
            view->appendPlainText(tag + text);
        }
    }
};

// Main window with a save prompt on close when the document is dirty.
class EditorWindow : public QMainWindow {
public:
    std::function<bool()> isDirty;    // wired once the bridge exists
    std::function<void()> saveNow;

protected:
    void closeEvent(QCloseEvent* event) override {
        if (!isDirty || !isDirty()) {
            event->accept();
            return;
        }
        const auto choice = QMessageBox::warning(
            this, "Unsaved Changes", "The level has unsaved changes.",
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if (choice == QMessageBox::Cancel) {
            event->ignore();
            return;
        }
        if (choice == QMessageBox::Save && saveNow) saveNow();
        event->accept();
    }
};

// Hierarchy tree with drag-to-reparent. Dropping the selected item(s) ONTO
// another makes them its children; dropping into empty space moves them to
// the root. The engine applies the reparent and the tree rebuilds from the
// document on the next refresh, so Qt must NOT move or delete items itself:
// the drop is accepted as an IgnoreAction (otherwise Qt's InternalMove would
// remove the source rows after this handler returns — deleting the very
// QTreeWidgetItems still referenced by rowItems/lastList, a crash).
class HierarchyTree : public QTreeWidget {
public:
    // (childRows, parentRow); parentRow < 0 means root. Rows index the
    // bridge's last entity list (stored per item in Qt::UserRole).
    std::function<void(const std::vector<int>&, int)> onReparent;

    explicit HierarchyTree(QWidget* parent) : QTreeWidget(parent) {
        setHeaderHidden(true);
        setSelectionMode(QAbstractItemView::ExtendedSelection);
        setDragDropMode(QAbstractItemView::InternalMove);
        setDragEnabled(true);
        setAcceptDrops(true);
        // A bit of breathing room: clearer nesting, alternating rows, and
        // taller rows so the tree reads as structure, not a dense list.
        setIndentation(16);
        setAlternatingRowColors(true);
        setAnimated(true);
        setStyleSheet("QTreeWidget::item { padding: 2px 0px; }");
    }

protected:
    void dropEvent(QDropEvent* event) override {
        QTreeWidgetItem* target = itemAt(event->position().toPoint());
        int parentRow = target ? target->data(0, Qt::UserRole).toInt() : -1;
        // Snapshot every dragged row (the whole selection) up front, before
        // any model change, skipping the drop target itself.
        std::vector<int> childRows;
        for (QTreeWidgetItem* item : selectedItems()) {
            if (item == target) continue;
            childRows.push_back(item->data(0, Qt::UserRole).toInt());
        }
        // Accept without moving: the engine owns the hierarchy. IgnoreAction
        // stops Qt from removing/deleting the source rows.
        event->setDropAction(Qt::IgnoreAction);
        event->accept();
        if (onReparent && !childRows.empty()) onReparent(childRows, parentRow);
    }
};

// Hierarchy + inspector, refreshed by polling the bridge (cheap at editor
// entity counts; avoids engine-side notification plumbing).
struct Panels {
    EditorBridge& bridge;
    HierarchyTree* hierarchy = nullptr;
    PropertyInspector* inspector = nullptr;

    std::vector<EditorBridge::EntityInfo> lastList;
    bool applyingUi = false;   // guard: list writes vs refresh loop

    explicit Panels(EditorBridge& bridge) : bridge(bridge) {}

    QDockWidget* buildHierarchyDock(QMainWindow* main) {
        auto* dock = new QDockWidget("Hierarchy", main);
        hierarchy = new HierarchyTree(dock);
        dock->setWidget(hierarchy);
        // Selection (possibly multiple rows) flows to the engine; the current
        // item is the primary (gizmo anchor + inspector).
        QObject::connect(hierarchy, &QTreeWidget::itemSelectionChanged,
                         [this]() {
            if (applyingUi || !bridge.attached()) return;
            std::vector<Entity> sel;
            for (QTreeWidgetItem* item : hierarchy->selectedItems()) {
                int row = item->data(0, Qt::UserRole).toInt();
                if (row >= 0 && row < static_cast<int>(lastList.size()))
                    sel.push_back(lastList[row].entity);
            }
            Entity primary;
            if (QTreeWidgetItem* cur = hierarchy->currentItem()) {
                int row = cur->data(0, Qt::UserRole).toInt();
                if (row >= 0 && row < static_cast<int>(lastList.size()))
                    primary = lastList[row].entity;
            }
            bridge.setSelection(sel, primary);
        });
        hierarchy->onReparent = [this](const std::vector<int>& childRows,
                                       int parentRow) {
            if (!bridge.editable()) return;
            Entity parent = (parentRow >= 0 &&
                             parentRow < static_cast<int>(lastList.size()))
                                ? lastList[parentRow].entity
                                : Entity{};
            for (int childRow : childRows) {
                if (childRow < 0 || childRow >= static_cast<int>(lastList.size()))
                    continue;
                bridge.reparent(lastList[childRow].entity, parent);
            }
        };
        return dock;
    }

    QDockWidget* buildInspectorDock(QMainWindow* main) {
        auto* dock = new QDockWidget("Inspector", main);
        auto* body = new QWidget(dock);
        auto* column = new QVBoxLayout(body);

        // Every row below is generated from the engine's property
        // descriptions — Unity-style sections per component, zero
        // field-specific code on the Qt side. Delete lives as a small trash
        // button in the inspector's own header now.
        inspector = new PropertyInspector(bridge);
        column->addWidget(inspector);
        column->addStretch();

        dock->setWidget(body);
        return dock;
    }

    // Called on a UI timer: mirror engine state into the widgets. During a
    // playtest the bridge is attached read-only (observer mode): the
    // hierarchy stays navigable and the inspector keeps syncing live values,
    // but its widgets gray out — watch, don't touch.
    void refresh() {
        const bool live = bridge.attached();
        hierarchy->setEnabled(live);
        inspector->setEnabled(bridge.editable());
        inspector->refresh();
        if (!live) return;

        applyingUi = true;
        auto list = bridge.listEntities();
        // Rebuild when anything structural changed: membership, label, OR the
        // parent graph (a reparent keeps the same entities but moves them).
        bool sameList = list.size() == lastList.size();
        for (size_t i = 0; sameList && i < list.size(); i++)
            sameList = list[i].entity == lastList[i].entity &&
                       list[i].label == lastList[i].label &&
                       list[i].parentId == lastList[i].parentId;
        if (!sameList) {
            lastList = list;
            rebuildTree();
        }

        // Mirror the engine's selection set into the tree. Block signals so
        // this programmatic update doesn't echo back as a user selection
        // (which also avoids the first-paint selection churn).
        const QSignalBlocker blocker(hierarchy);
        std::vector<Entity> engineSel = bridge.selectionList();
        Entity primary = bridge.selected();
        QTreeWidgetItem* primaryItem = nullptr;
        for (size_t i = 0; i < lastList.size(); i++) {
            QTreeWidgetItem* item = itemForRow(static_cast<int>(i));
            if (!item) continue;
            bool want = false;
            for (Entity e : engineSel)
                if (e == lastList[i].entity) { want = true; break; }
            if (item->isSelected() != want) item->setSelected(want);
            if (lastList[i].entity == primary) primaryItem = item;
        }
        // NoUpdate: set the current item without disturbing the multi-row
        // selection we just applied above.
        if (hierarchy->currentItem() != primaryItem)
            hierarchy->setCurrentItem(primaryItem, 0,
                                      QItemSelectionModel::NoUpdate);
        applyingUi = false;
    }

    // Build the tree from the document's parent graph (parentId references an
    // id; cameras/player carry id 0 and sit at the root). Parent-before-child
    // ordering is not assumed — items are created first, then attached.
    void rebuildTree() {
        hierarchy->clear();
        rowItems.assign(lastList.size(), nullptr);
        std::unordered_map<uint32_t, QTreeWidgetItem*> byId;

        QStyle* st = hierarchy->style();
        for (size_t i = 0; i < lastList.size(); i++) {
            const auto& info = lastList[i];
            auto* item = new QTreeWidgetItem;
            item->setText(0, QString::fromStdString(info.label));
            // A type icon instead of text tags: folder for groups, a desktop
            // glyph for cameras, a file glyph for plain objects.
            QStyle::StandardPixmap icon =
                info.isGroup ? QStyle::SP_DirIcon
                : info.isCamera ? QStyle::SP_ComputerIcon
                                : QStyle::SP_FileIcon;
            item->setIcon(0, st->standardIcon(icon));
            item->setData(0, Qt::UserRole, static_cast<int>(i));
            rowItems[i] = item;
            if (info.id != 0) byId[info.id] = item;
        }
        for (size_t i = 0; i < lastList.size(); i++) {
            uint32_t pid = lastList[i].parentId;
            auto it = (pid != 0) ? byId.find(pid) : byId.end();
            if (it != byId.end() && it->second != rowItems[i])
                it->second->addChild(rowItems[i]);
            else
                hierarchy->addTopLevelItem(rowItems[i]);
        }
        hierarchy->expandAll();
    }

    QTreeWidgetItem* itemForRow(int row) {
        return (row >= 0 && row < static_cast<int>(rowItems.size()))
                   ? rowItems[row]
                   : nullptr;
    }

    std::vector<QTreeWidgetItem*> rowItems;
};

}  // namespace

int main(int argc, char** argv) {
    QApplication qtApp(argc, argv);

    std::string levelPath = "assets/levels/arena.json";
    for (int i = 1; i < argc; i++)
        if (argv[i][0] != '-') levelPath = argv[i];

    auto hostedOwned = std::make_unique<engine::HostedWindow>();
    engine::HostedWindow* hosted = hostedOwned.get();
    engine::EditorBridge bridge;

    EditorWindow mainWindow;
    QString baseTitle = "Raytracer Editor";
    mainWindow.setWindowTitle(baseTitle);
    auto* viewport = new EngineViewport(*hosted);
    mainWindow.setCentralWidget(viewport);
    mainWindow.resize(1560, 960);

    mainWindow.isDirty = [&bridge]() { return bridge.documentDirty(); };
    mainWindow.saveNow = [&bridge]() {
        if (bridge.attached()) bridge.saveDocument();
    };
    // Engine cursor modes drive the viewport's pointer capture (play mode
    // locks the pointer for first-person look; the editor releases it).
    hosted->setCursorModeCallback([viewport](engine::CursorMode mode) {
        viewport->setCaptured(mode == engine::CursorMode::Disabled);
    });

    Panels panels(bridge);
    QDockWidget* hierarchyDock = panels.buildHierarchyDock(&mainWindow);
    QDockWidget* inspectorDock = panels.buildInspectorDock(&mainWindow);
    mainWindow.addDockWidget(Qt::LeftDockWidgetArea, hierarchyDock);
    mainWindow.addDockWidget(Qt::RightDockWidgetArea, inspectorDock);

    // Asset browser: a filesystem view of assets/; double-clicking a level
    // opens it (Play/Stop and open both go through Application::requestState).
    auto* assetsDock = new QDockWidget("Assets", &mainWindow);
    auto* assetsModel = new QFileSystemModel(assetsDock);
    assetsModel->setRootPath("assets");
    auto* assetsView = new QTreeView(assetsDock);
    assetsView->setModel(assetsModel);
    assetsView->setRootIndex(assetsModel->index("assets"));
    assetsView->setColumnHidden(1, true);   // size — hide so the Name column gets the room
    assetsView->setColumnHidden(2, true);   // type
    assetsView->setColumnHidden(3, true);   // date
    // Name was cramped to a few characters; let it stretch to the dock width and start wide.
    assetsView->header()->setStretchLastSection(true);
    assetsView->setColumnWidth(0, 320);
    assetsDock->setWidget(assetsView);
    mainWindow.addDockWidget(Qt::BottomDockWidgetArea, assetsDock);

    // Engine log console, tabbed with the asset browser.
    LogConsole console;
    QDockWidget* consoleDock = console.buildDock(&mainWindow);
    mainWindow.addDockWidget(Qt::BottomDockWidgetArea, consoleDock);
    mainWindow.tabifyDockWidget(assetsDock, consoleDock);
    assetsDock->raise();

    auto* fileMenu = mainWindow.menuBar()->addMenu("&File");
    // Import = validate, then copy into the project's asset tree (asset
    // cooking grows behind this action — editor-app plan A4). The assets dock
    // refreshes itself (QFileSystemModel watches the directory).
    fileMenu->addAction("&Import Asset...", [&mainWindow]() {
        QString src = QFileDialog::getOpenFileName(
            &mainWindow, "Import Asset", QString(),
            "Assets (*.gltf *.glb *.hdr)");
        if (src.isEmpty()) return;

        QFileInfo info(src);
        QString kind = info.suffix().toLower();
        if (kind == "gltf" || kind == "glb") {
            std::string error;
            if (!engine::ModelImporter::validate(src.toStdString(), error)) {
                mainWindow.statusBar()->showMessage(
                    QString("Import failed: %1").arg(error.c_str()), 6000);
                return;
            }
        } else if (kind == "hdr") {
            if (!engine::EnvironmentLoader::loadHdr(src.toStdString()).valid()) {
                mainWindow.statusBar()->showMessage(
                    "Import failed: unreadable HDR", 6000);
                return;
            }
        }

        QString destDir = (kind == "hdr") ? "assets/env" : "assets/models";
        QDir().mkpath(destDir);
        QString dest = destDir + "/" + info.fileName();
        if (QFile::exists(dest)) QFile::remove(dest);
        bool copied = QFile::copy(src, dest);
        mainWindow.statusBar()->showMessage(
            copied ? QString("Imported %1").arg(dest)
                   : QString("Import failed: cannot copy to %1").arg(dest),
            6000);
    });
    // "Recently Opened" is inserted here (before this separator) once the
    // state factories exist — it needs to open a level, which requestState does.
    auto* fileTailSeparator = fileMenu->addSeparator();
    fileMenu->addAction("&Quit", &qtApp, &QApplication::quit);

    // Edit menu: the engine-side command log, reachable with the native
    // chords. Enabled state mirrors the bridge on the panel timer below.
    auto* editMenu = mainWindow.menuBar()->addMenu("&Edit");
    auto* undoAction = editMenu->addAction("&Undo", [&]() { bridge.undo(); });
    undoAction->setShortcut(QKeySequence::Undo);
    auto* redoAction = editMenu->addAction("&Redo", [&]() { bridge.redo(); });
    redoAction->setShortcut(QKeySequence::Redo);
    editMenu->addSeparator();
    auto* duplicateAction = editMenu->addAction("&Duplicate", [&]() {
        if (bridge.attached()) bridge.duplicateSelected();
    });
    duplicateAction->setShortcut(QKeySequence("Ctrl+D"));
    auto* deleteAction = editMenu->addAction("De&lete", [&]() {
        if (bridge.attached()) bridge.deleteSelection();
    });
    deleteAction->setShortcut(QKeySequence::Delete);
    mainWindow.statusBar()->showMessage(
        "Click selects | 1/2/3 move/rotate/scale | Shift-drag snaps | "
        "F frames selection");
    // Mode indicator, pinned right: EDITING / PLAYING / PAUSED.
    auto* modeLabel = new QLabel("EDITING");
    mainWindow.statusBar()->addPermanentWidget(modeLabel);

    // Realize the native view BEFORE the renderer binds to it.
    mainWindow.show();
    // Side panels start narrow; the viewport is the star. (resizeDocks only
    // takes effect once the window is realized.)
    mainWindow.resizeDocks({hierarchyDock, inspectorDock}, {200, 330},
                           Qt::Horizontal);
    hosted->setNativeHandle(reinterpret_cast<void*>(viewport->winId()));
    {
        const qreal scale = viewport->devicePixelRatioF();
        hosted->setSizes(viewport->width(), viewport->height(),
                         static_cast<int>(viewport->width() * scale),
                         static_cast<int>(viewport->height() * scale));
    }

    // Vulkan backend (PC/Linux): hand the renderer a surface for the Qt
    // viewport. Resolve the toolkit-native handles from the Qt platform plugin
    // and defer the actual VkSurfaceKHR creation to the platform helper. The
    // callbacks fire during app.initialize() below, by which point the native
    // window is realized (mainWindow.show() above).
#if defined(RT_HAVE_VULKAN)
    {
        const QString plat = QGuiApplication::platformName();
        hosted->setVulkanSurfaceProvider(
            [plat]() -> std::vector<std::string> {
                if (plat == "windows")
                    return editor::vulkanInstanceExtensions(
                        editor::VkPlatform::Win32);
                if (plat == "xcb")
                    return editor::vulkanInstanceExtensions(
                        editor::VkPlatform::Xlib);
                return {};
            },
            [plat, viewport](void* instance, uint64_t* outSurface) -> bool {
                const unsigned long long winId =
                    static_cast<unsigned long long>(viewport->winId());
                if (plat == "windows")
                    return editor::createVulkanSurface(
                        instance, editor::VkPlatform::Win32, nullptr, winId,
                        outSurface);
#if defined(RT_EDITOR_VK_X11)
                if (plat == "xcb") {
                    void* display = nullptr;
                    if (auto* x11 = qApp->nativeInterface<
                                        QNativeInterface::QX11Application>())
                        display = x11->display();
                    return editor::createVulkanSurface(
                        instance, editor::VkPlatform::Xlib, display, winId,
                        outSurface);
                }
#endif
                LOG_ERROR << "[editor] no Vulkan surface path for Qt platform "
                          << plat.toStdString();
                return false;
            });
    }
#endif

    engine::Application app;
    if (!app.initialize({1280, 720, "Editor", "settings.json"},
                        std::move(hostedOwned))) {
        LOG_ERROR << "Engine failed to initialize inside the editor shell";
        return 1;
    }

    // Same factory wiring as the standalone viewer; the bridge rides along so
    // panels attach whenever an editor state is active.
    std::function<std::unique_ptr<engine::AppState>()> makePlay;
    std::function<std::unique_ptr<engine::AppState>()> makeEditor;
    makeEditor = [&]() -> std::unique_ptr<engine::AppState> {
        return std::make_unique<engine::EditorState>(
            app.windowRef(), app.renderer(), levelPath, makePlay, &bridge);
    };
    makePlay = [&]() -> std::unique_ptr<engine::AppState> {
        // The bridge rides into play too — observer mode: live panels,
        // editing off (ArenaState attaches it read-only).
        return std::make_unique<ArenaState>(app.windowRef(), app.renderer(),
                                            levelPath, makeEditor, &bridge);
    };

    // Normalize a level path to one relative to the working directory, so the
    // same scene opened from the asset browser (which yields an absolute path)
    // and from the command line (relative) dedupe to ONE recent entry.
    auto relativeScenePath = [](const std::string& raw) {
        const QString abs = QFileInfo(QString::fromStdString(raw)).absoluteFilePath();
        return QDir::current().relativeFilePath(abs).toStdString();
    };

    // Open a level by path: the one place a scene becomes the loaded scene.
    // Records it in the recent list, updates the title, and swaps the editor
    // state. Used by the asset-browser double-click and File > Recently Opened.
    auto openScene = [&](const std::string& rawPath) {
        const std::string path = relativeScenePath(rawPath);
        levelPath = path;
        engine::recordRecentScene(app.settings(), path);
        baseTitle = "Raytracer Editor — " + QString::fromStdString(path);
        mainWindow.setWindowTitle(baseTitle);
        app.requestState(makeEditor());
        viewport->setFocus();
    };

    // File > Recently Opened: the last few scenes you've opened, newest first,
    // persisted in settings.json. Rebuilt each time the menu opens so it always
    // reflects the current list. Inserted into the File menu above Quit.
    auto* recentMenu = new QMenu("&Recently Opened", &mainWindow);
    QObject::connect(recentMenu, &QMenu::aboutToShow, [&, recentMenu]() {
        recentMenu->clear();
        const std::vector<std::string> recent =
            engine::loadRecentScenes(app.settings());
        if (recent.empty()) {
            recentMenu->addAction("(none yet)")->setEnabled(false);
            return;
        }
        for (const std::string& path : recent) {
            // Show the bare scene name (no directory, no ".json"); open the
            // full path. Skip any file that has since been deleted.
            const QString qpath = QString::fromStdString(path);
            const QString name = QFileInfo(qpath).completeBaseName();
            QAction* act = recentMenu->addAction(name, [&, path]() {
                openScene(path);
            });
            act->setEnabled(QFileInfo::exists(qpath));
        }
        recentMenu->addSeparator();
        recentMenu->addAction("Clear List", [&]() {
            engine::clearRecentScenes(app.settings());
        });
    });
    fileMenu->insertMenu(fileTailSeparator, recentMenu);

    // Level menu: document-level properties (they belong to the level, not
    // to any entity in the hierarchy). The environment HDR is the first.
    // Built here, after the state factories exist (its action reloads).
    auto* levelMenu = mainWindow.menuBar()->addMenu("&Level");
    levelMenu->addAction("&Environment...", [&]() {
        if (!bridge.editable()) return;
        QStringList options{"(procedural sky + day/night)"};
        const QFileInfoList hdrs =
            QDir("assets/env").entryInfoList({"*.hdr"}, QDir::Files);
        for (const QFileInfo& f : hdrs) options << f.fileName();

        // Preselect whatever the document uses now.
        const QString current =
            QFileInfo(QString::fromStdString(bridge.environmentHdr()))
                .fileName();
        int currentIndex =
            std::max(0, static_cast<int>(options.indexOf(current)));

        bool ok = false;
        const QString pick = QInputDialog::getItem(
            &mainWindow, "Level Environment",
            "HDR environment (owns sun + sky while set; the first entry\n"
            "removes it so the procedural sky and day/night cycle drive):",
            options, currentIndex, false, &ok);
        if (!ok) return;
        const std::string rel =
            (pick == options[0]) ? "" : ("../env/" + pick).toStdString();
        if (bridge.setEnvironmentHdr(rel)) {
            app.requestState(makeEditor());   // reload re-cooks IBL + sun
            viewport->setFocus();
        }
    });

    // Toolbar, in groups: document | transport (icon buttons, Unity-style) |
    // gizmo modes | editing tools. Standard style icons keep it native.
    const QStyle* style = mainWindow.style();
    auto* toolbar = mainWindow.addToolBar("Main");
    toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);   // transport = icons
    // Force a toolbar action's button back to showing its text (for the
    // labelled tools after the icon-only transport cluster).
    auto asTextButton = [&](QAction* a) {
        if (auto* w = qobject_cast<QToolButton*>(toolbar->widgetForAction(a)))
            w->setToolButtonStyle(Qt::ToolButtonTextOnly);
    };

    auto* saveAction = toolbar->addAction(
        style->standardIcon(QStyle::SP_DialogSaveButton), "Save", [&]() {
        if (bridge.editable()) bridge.saveDocument();
    });
    saveAction->setToolTip("Save the level (Ctrl+S)");
    toolbar->addSeparator();

    // Play = compile + run. Click plays from the document spawn; press-and-
    // hold opens a menu to Play From Here (the editor camera) instead.
    auto startPlay = [&](bool fromHere) {
        if (!bridge.editable()) return;     // already playing
        bridge.saveDocument();
        if (fromHere) app.settings().setBool("playFromHere", true);
        app.simClock().setPaused(false);    // a fresh playtest always runs
        app.requestState(makePlay());
        viewport->setFocus();               // WASD goes to the game, not Qt
    };
    auto* playButton = new QToolButton(toolbar);
    playButton->setIcon(style->standardIcon(QStyle::SP_MediaPlay));
    playButton->setToolTip("Play  (press and hold for Play From Here)");
    playButton->setPopupMode(QToolButton::DelayedPopup);
    QObject::connect(playButton, &QToolButton::clicked,
                     [startPlay]() { startPlay(false); });
    auto* playMenu = new QMenu(playButton);
    playMenu->addAction(style->standardIcon(QStyle::SP_MediaPlay), "Play",
                        [startPlay]() { startPlay(false); });
    playMenu->addAction(style->standardIcon(QStyle::SP_MediaSkipForward),
                        "Play From Here", [startPlay]() { startPlay(true); });
    playButton->setMenu(playMenu);
    toolbar->addWidget(playButton);

    // Transport during play: the same pause switch Space toggles in-game,
    // plus fixed-step frame advance (Application::simClock is the hook).
    auto* pauseAction = toolbar->addAction(
        style->standardIcon(QStyle::SP_MediaPause), "Pause", [&]() {
        app.simClock().setPaused(!app.simClock().paused());
        viewport->setFocus();
    });
    pauseAction->setCheckable(true);
    pauseAction->setToolTip("Pause / resume the simulation (Space)");
    auto* stepAction = toolbar->addAction(
        style->standardIcon(QStyle::SP_MediaSeekForward), "Step", [&]() {
        app.simClock().requestStep();
    });
    stepAction->setToolTip("Advance one fixed step while paused");
    auto* stopAction = toolbar->addAction(
        style->standardIcon(QStyle::SP_MediaStop), "Stop", [&]() {
        if (bridge.editable()) return;      // already editing
        app.requestState(makeEditor());
        viewport->setFocus();
    });
    stopAction->setToolTip("Stop and return to edit mode (Esc)");
    auto* restartAction = toolbar->addAction(
        style->standardIcon(QStyle::SP_BrowserReload), "Restart", [&]() {
        if (bridge.editable() || !bridge.attached()) return;
        app.simClock().setPaused(false);
        app.requestState(makePlay());
        viewport->setFocus();
    });
    restartAction->setToolTip("Restart the playtest from the spawn point");
    toolbar->addSeparator();

    // Gizmo mode (mirrors the viewport's 1/2/3 keys). Exclusive, text labels.
    auto* gizmoGroup = new QActionGroup(toolbar);
    auto addGizmo = [&](const char* label, int op, const char* tip) {
        auto* a = toolbar->addAction(label, [&, op]() {
            bridge.setGizmoMode(op);
            viewport->setFocus();
        });
        a->setCheckable(true);
        a->setActionGroup(gizmoGroup);
        a->setToolTip(tip);
        asTextButton(a);
        return a;
    };
    auto* moveAction = addGizmo("Move", 0, "Move tool (1)");
    auto* rotateAction = addGizmo("Rotate", 1, "Rotate tool (2)");
    auto* scaleAction = addGizmo("Scale", 2, "Scale tool (3)");
    moveAction->setChecked(true);
    toolbar->addSeparator();

    // Add: place primitives / cameras / groups from native UI. Creation is
    // queued onto the editor — the spawn point comes from the live view.
    auto* addButton = new QToolButton(toolbar);
    addButton->setText("Add");
    addButton->setIcon(style->standardIcon(QStyle::SP_FileDialogNewFolder));
    addButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    addButton->setPopupMode(QToolButton::InstantPopup);
    addButton->setToolTip("Add an object to the level");
    auto* addShapeMenu = new QMenu(addButton);
    static const char* SHAPES[] = {"box", "sphere", "cylinder", "plane",
                                   "cone", "wedge", "torus", "capsule"};
    for (const char* shape : SHAPES)
        addShapeMenu->addAction(shape, [&bridge, shape]() {
            if (bridge.attached()) bridge.addPrimitive(shape);
        });
    addShapeMenu->addSeparator();
    addShapeMenu->addAction("camera", [&bridge]() {
        if (bridge.attached()) bridge.placeCamera();
    });
    addShapeMenu->addAction("empty group", [&bridge]() {
        if (bridge.attached()) bridge.addGroup();
    });
    addShapeMenu->addAction("player spawn", [&bridge]() {
        if (bridge.attached()) bridge.addPlayerSpawn();
    });
    addButton->setMenu(addShapeMenu);
    toolbar->addWidget(addButton);

    // Ground-grid toggle; EditorSystem reads the setting every frame.
    auto* gridAction = toolbar->addAction("Grid", [&](bool on) {
        app.settings().setBool("editorGrid", on);
    });
    gridAction->setCheckable(true);
    gridAction->setChecked(app.settings().getBool("editorGrid", true));
    gridAction->setToolTip("Toggle the ground grid");
    asTextButton(gridAction);

    // Physics as a level-design tool: while playing, overwrite the document
    // with the live world (settled stacks, pushed props). Stop then reloads
    // the baked result. Destructive, hence the confirmation.
    auto* bakeAction = toolbar->addAction("Bake", [&]() {
        if (bridge.editable() || !bridge.attached()) return;
        const auto choice = QMessageBox::question(
            &mainWindow, "Bake Play State",
            "Overwrite the level with the current play state?\n"
            "(Object positions as physics left them.)");
        if (choice != QMessageBox::Yes) return;
        bool ok = bridge.bakePlaytestToDocument();
        mainWindow.statusBar()->showMessage(
            ok ? "Baked play state into the document"
               : "Bake failed (see console)",
            4000);
    });
    bakeAction->setToolTip("Overwrite the level with the current play state");
    asTextButton(bakeAction);

    // Open a level from the asset browser.
    QObject::connect(assetsView, &QTreeView::doubleClicked, [&](const QModelIndex& idx) {
        QString path = assetsModel->filePath(idx);
        if (!path.endsWith(".json")) return;
        openScene(path.toStdString());
    });

    app.settings().setString("cameraMode", "fly");
    // The scene we boot with counts as recently opened (normalized like the rest).
    engine::recordRecentScene(app.settings(), relativeScenePath(levelPath));
    app.pushState(makeEditor());
    app.begin();

    // Window chrome that mirrors engine state: action enables, dirty title,
    // mode indicator. Run on the slow poll AND immediately when a bridge
    // notice arrives, so mode/selection flips don't wait out the timer.
    auto refreshChrome = [&]() {
        const bool editing = bridge.editable();
        playButton->setEnabled(editing);
        stopAction->setEnabled(!editing);
        addButton->setEnabled(editing);
        pauseAction->setEnabled(!editing);
        pauseAction->setChecked(!editing && app.simClock().paused());
        stepAction->setEnabled(!editing && app.simClock().paused());
        bakeAction->setEnabled(!editing && bridge.attached());
        restartAction->setEnabled(!editing && bridge.attached());
        undoAction->setEnabled(bridge.canUndo());
        redoAction->setEnabled(bridge.canRedo());
        duplicateAction->setEnabled(editing);
        deleteAction->setEnabled(editing);
        // Gizmo mode buttons mirror the engine (1/2/3 keys can change it);
        // block signals so reflecting the state doesn't fire setGizmoMode.
        moveAction->setEnabled(editing);
        rotateAction->setEnabled(editing);
        scaleAction->setEnabled(editing);
        if (editing) {
            QAction* modes[3] = {moveAction, rotateAction, scaleAction};
            int op = std::clamp(bridge.gizmoMode(), 0, 2);
            if (!modes[op]->isChecked()) {
                const QSignalBlocker b(gizmoGroup);
                modes[op]->setChecked(true);
            }
        }
        const QString title =
            bridge.documentDirty() ? baseTitle + " *" : baseTitle;
        if (mainWindow.windowTitle() != title) mainWindow.setWindowTitle(title);
        modeLabel->setText(editing            ? "EDITING"
                           : bridge.attached() ? (app.simClock().paused()
                                                      ? "PAUSED"
                                                      : "PLAYING")
                                               : "...");
    };

    // Qt owns the loop; the engine steps per timer tick, panels poll slower
    // — except when the engine notifies (mode/selection/save), which
    // refreshes on the very next frame.
    // Gamepads: the Qt host has no GLFW, so poll the GCController backend each
    // frame and push the snapshot into the hosted window (no-op off macOS).
    // The engine's player input + fly camera then respond in the editor.
    engine::GamepadSet editorGamepads;

    QTimer frameTimer;
    QObject::connect(&frameTimer, &QTimer::timeout, [&]() {
        if (!app.running()) {
            qtApp.quit();
            return;
        }
        engine::gcPollGamepads(editorGamepads);
        hosted->setGamepads(editorGamepads);
        viewport->pollCapturedMouse();   // relative look while playing
        app.runFrame();
        console.drain();
        if (!bridge.drainNotices().empty()) {
            panels.refresh();
            refreshChrome();
        }
    });
    frameTimer.start(16);

    QTimer panelTimer;
    QObject::connect(&panelTimer, &QTimer::timeout, [&]() {
        panels.refresh();
        refreshChrome();
    });
    panelTimer.start(150);

    int result = qtApp.exec();
    app.end();
    return result;
}
