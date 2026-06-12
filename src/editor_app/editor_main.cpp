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
#include "../engine/states/editor_state.h"
#include "../game/arena_state.h"
#include "../renderer/hosted_window.h"
#include "../log.h"
#include "property_inspector.h"

#include <QApplication>
#include <QCloseEvent>
#include <QCursor>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFileSystemModel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QTreeView>
#include <QWheelEvent>
#include <QWidget>

#include <cstring>

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

    // First-person capture (engine CursorMode::Disabled, e.g. Play): hide
    // the cursor and report relative motion, warping back to the viewport
    // center after each move — the toolkit-side half of pointer lock.
    void setCaptured(bool on) {
        if (captured == on) return;
        captured = on;
        if (on) {
            setFocus();
            setCursor(Qt::BlankCursor);
            grabMouse();   // motion must arrive even before the first warp
            firstCapturedMove = true;   // sync to center without a delta spike
        } else {
            releaseMouse();
            unsetCursor();
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
        if (captured) {
            if (!hasFocus()) return;   // alt-tabbed away: leave the cursor be
            QPoint center(width() / 2, height() / 2);
            double dx = e->position().x() - center.x();
            double dy = e->position().y() - center.y();
            if (firstCapturedMove) {
                firstCapturedMove = false;   // swallow the pre-warp position
            } else if (dx != 0.0 || dy != 0.0) {
                hosted.injectMouseDelta(dx, dy);
            }
            if (dx != 0.0 || dy != 0.0) QCursor::setPos(mapToGlobal(center));
            return;
        }
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
    bool firstCapturedMove = false;
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

// Hierarchy + inspector, refreshed by polling the bridge (cheap at editor
// entity counts; avoids engine-side notification plumbing).
struct Panels {
    EditorBridge& bridge;
    QListWidget* hierarchy = nullptr;
    PropertyInspector* inspector = nullptr;
    QPushButton* deleteButton = nullptr;

    std::vector<EditorBridge::EntityInfo> lastList;
    bool applyingUi = false;   // guard: list writes vs refresh loop

    explicit Panels(EditorBridge& bridge) : bridge(bridge) {}

    QDockWidget* buildHierarchyDock(QMainWindow* main) {
        auto* dock = new QDockWidget("Hierarchy", main);
        hierarchy = new QListWidget(dock);
        dock->setWidget(hierarchy);
        QObject::connect(hierarchy, &QListWidget::currentRowChanged, [this](int row) {
            if (applyingUi || !bridge.attached()) return;
            if (row >= 0 && row < static_cast<int>(lastList.size()))
                bridge.select(lastList[row].entity);
        });
        return dock;
    }

    QDockWidget* buildInspectorDock(QMainWindow* main) {
        auto* dock = new QDockWidget("Inspector", main);
        auto* body = new QWidget(dock);
        auto* column = new QVBoxLayout(body);

        // Every row below is generated from the engine's property
        // descriptions — Unity-style sections per component, zero
        // field-specific code on the Qt side.
        inspector = new PropertyInspector(bridge);
        column->addWidget(inspector);

        deleteButton = new QPushButton("Delete", body);
        QObject::connect(deleteButton, &QPushButton::clicked, [this]() {
            if (bridge.attached()) bridge.deleteEntity(bridge.selected());
        });
        column->addWidget(deleteButton);
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
        deleteButton->setEnabled(bridge.editable());
        inspector->refresh();
        if (!live) return;

        applyingUi = true;
        auto list = bridge.listEntities();
        bool sameList = list.size() == lastList.size();
        for (size_t i = 0; sameList && i < list.size(); i++)
            sameList = list[i].entity == lastList[i].entity &&
                       list[i].label == lastList[i].label;
        if (!sameList) {
            lastList = list;
            hierarchy->clear();
            for (const auto& info : lastList)
                hierarchy->addItem(QString::fromStdString(
                    (info.isCamera ? "[cam] " : "") + info.label));
        }

        Entity selected = bridge.selected();
        int row = -1;
        for (size_t i = 0; i < lastList.size(); i++)
            if (lastList[i].entity == selected) row = static_cast<int>(i);
        if (hierarchy->currentRow() != row) hierarchy->setCurrentRow(row);
        applyingUi = false;
    }
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
    mainWindow.addDockWidget(Qt::LeftDockWidgetArea,
                             panels.buildHierarchyDock(&mainWindow));
    mainWindow.addDockWidget(Qt::RightDockWidgetArea,
                             panels.buildInspectorDock(&mainWindow));

    // Asset browser: a filesystem view of assets/; double-clicking a level
    // opens it (Play/Stop and open both go through Application::requestState).
    auto* assetsDock = new QDockWidget("Assets", &mainWindow);
    auto* assetsModel = new QFileSystemModel(assetsDock);
    assetsModel->setRootPath("assets");
    auto* assetsView = new QTreeView(assetsDock);
    assetsView->setModel(assetsModel);
    assetsView->setRootIndex(assetsModel->index("assets"));
    assetsView->setColumnHidden(2, true);   // type
    assetsView->setColumnHidden(3, true);   // date
    assetsDock->setWidget(assetsView);
    mainWindow.addDockWidget(Qt::BottomDockWidgetArea, assetsDock);

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
    fileMenu->addSeparator();
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
        if (bridge.attached()) bridge.deleteEntity(bridge.selected());
    });
    deleteAction->setShortcut(QKeySequence::Delete);
    mainWindow.statusBar()->showMessage(
        "Click selects | 1/2/3 move/rotate/scale | F free-fly | C place camera");

    // Realize the native view BEFORE the renderer binds to it.
    mainWindow.show();
    hosted->setNativeHandle(reinterpret_cast<void*>(viewport->winId()));
    {
        const qreal scale = viewport->devicePixelRatioF();
        hosted->setSizes(viewport->width(), viewport->height(),
                         static_cast<int>(viewport->width() * scale),
                         static_cast<int>(viewport->height() * scale));
    }

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

    // Toolbar: the document loop (save / play / stop) from native UI.
    auto* toolbar = mainWindow.addToolBar("Main");
    toolbar->addAction("Save", [&]() {
        if (bridge.editable()) bridge.saveDocument();
    });
    auto* playAction = toolbar->addAction("Play", [&]() {
        if (!bridge.editable()) return;     // already playing
        bridge.saveDocument();              // Play = compile + run
        app.simClock().setPaused(false);    // a fresh playtest always runs
        app.requestState(makePlay());
        viewport->setFocus();               // WASD goes to the game, not Qt
    });
    auto* playHereAction = toolbar->addAction("Play Here", [&]() {
        if (!bridge.editable()) return;
        bridge.saveDocument();
        app.settings().setBool("playFromHere", true);   // one-shot override
        app.simClock().setPaused(false);
        app.requestState(makePlay());
        viewport->setFocus();
    });
    auto* stopAction = toolbar->addAction("Stop", [&]() {
        if (bridge.editable()) return;      // already editing
        app.requestState(makeEditor());
        viewport->setFocus();
    });

    // Transport controls during play: the same pause switch Space toggles
    // in-game, plus fixed-step frame advance — editor-shell hooks into the
    // engine's clock (Application::simClock).
    auto* pauseAction = toolbar->addAction("Pause", [&]() {
        app.simClock().setPaused(!app.simClock().paused());
        viewport->setFocus();
    });
    pauseAction->setCheckable(true);
    auto* stepAction = toolbar->addAction("Step", [&]() {
        app.simClock().requestStep();
    });

    // Add: place primitives / cameras from native UI (the in-viewport ImGui
    // Add panel is suppressed while the shell hosts the engine). Creation is
    // queued onto the editor — the spawn point comes from the live view.
    auto* addButton = new QToolButton(toolbar);
    addButton->setText("Add");
    addButton->setPopupMode(QToolButton::InstantPopup);
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
    addButton->setMenu(addShapeMenu);
    toolbar->addWidget(addButton);

    // Open a level from the asset browser.
    QObject::connect(assetsView, &QTreeView::doubleClicked, [&](const QModelIndex& idx) {
        QString path = assetsModel->filePath(idx);
        if (!path.endsWith(".json")) return;
        levelPath = path.toStdString();
        baseTitle = QString("Raytracer Editor — %1").arg(path);
        mainWindow.setWindowTitle(baseTitle);
        app.requestState(makeEditor());
        viewport->setFocus();
    });

    app.settings().setString("cameraMode", "fly");
    app.pushState(makeEditor());
    app.begin();

    // Qt owns the loop; the engine steps per timer tick, panels poll slower.
    QTimer frameTimer;
    QObject::connect(&frameTimer, &QTimer::timeout, [&]() {
        if (!app.running()) {
            qtApp.quit();
            return;
        }
        app.runFrame();
    });
    frameTimer.start(16);

    QTimer panelTimer;
    QObject::connect(&panelTimer, &QTimer::timeout, [&]() {
        panels.refresh();
        const bool editing = bridge.editable();
        playAction->setEnabled(editing);
        playHereAction->setEnabled(editing);
        stopAction->setEnabled(!editing);
        addButton->setEnabled(editing);
        pauseAction->setEnabled(!editing);
        pauseAction->setChecked(!editing && app.simClock().paused());
        stepAction->setEnabled(!editing && app.simClock().paused());
        undoAction->setEnabled(bridge.canUndo());
        redoAction->setEnabled(bridge.canRedo());
        duplicateAction->setEnabled(editing);
        deleteAction->setEnabled(editing);
        const QString title =
            bridge.documentDirty() ? baseTitle + " *" : baseTitle;
        if (mainWindow.windowTitle() != title) mainWindow.setWindowTitle(title);
    });
    panelTimer.start(150);

    int result = qtApp.exec();
    app.end();
    return result;
}
