// The editor application shell (editor-app plan, Phase A2 skeleton): a Qt
// window whose central widget hosts the engine through the HostedWindow seam.
// The engine inside is the exact EditorState/ArenaState stack the standalone
// viewer runs — picking, gizmos, the in-viewport ImGui tools, and the
// Play/Esc loop all work unchanged; Qt owns the OS window and the event loop
// and forwards input. Native panels (hierarchy, inspector, asset browser)
// arrive with Phase A3's EditorBridge.
//
// Deliberately Q_OBJECT-free (no moc): event handling overrides + lambda
// connects only.

#include "../engine/application.h"
#include "../engine/states/editor_state.h"
#include "../game/arena_state.h"
#include "../renderer/hosted_window.h"
#include "../log.h"

#include <QApplication>
#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QTimer>
#include <QWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>

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

// The engine's viewport pane: a native widget whose window handle the
// renderer binds to (NSView* on macOS), forwarding Qt input through the
// HostedWindow seam. Qt must not paint over the Metal layer, hence the
// paint-engine opt-outs.
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

protected:
    void resizeEvent(QResizeEvent*) override {
        const qreal scale = devicePixelRatioF();
        hosted.setSizes(width(), height(),
                        static_cast<int>(width() * scale),
                        static_cast<int>(height() * scale));
    }
    void mouseMoveEvent(QMouseEvent* e) override {
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
};

}  // namespace

int main(int argc, char** argv) {
    QApplication qtApp(argc, argv);

    std::string levelPath = "assets/levels/arena.json";
    for (int i = 1; i < argc; i++)
        if (argv[i][0] != '-') levelPath = argv[i];

    // The hosted window outlives Application (which owns it); keep a borrowed
    // pointer for event forwarding.
    auto hostedOwned = std::make_unique<engine::HostedWindow>();
    engine::HostedWindow* hosted = hostedOwned.get();

    QMainWindow mainWindow;
    mainWindow.setWindowTitle("Raytracer Editor");
    auto* viewport = new EngineViewport(*hosted);
    mainWindow.setCentralWidget(viewport);
    mainWindow.resize(1440, 900);

    auto* fileMenu = mainWindow.menuBar()->addMenu("&File");
    fileMenu->addAction("&Quit", &qtApp, &QApplication::quit);
    mainWindow.statusBar()->showMessage(
        "Click selects | 1/2/3 move/rotate/scale | in-viewport panels: Save/Play");

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

    // Same factory wiring as the standalone viewer: editor <-> play states
    // swap inside the viewport.
    std::function<std::unique_ptr<engine::AppState>()> makePlay;
    std::function<std::unique_ptr<engine::AppState>()> makeEditor;
    makeEditor = [&app, levelPath, &makePlay]() -> std::unique_ptr<engine::AppState> {
        return std::make_unique<engine::EditorState>(app.windowRef(), app.renderer(),
                                                     levelPath, makePlay);
    };
    makePlay = [&app, levelPath, &makeEditor]() -> std::unique_ptr<engine::AppState> {
        return std::make_unique<ArenaState>(app.windowRef(), app.renderer(),
                                            levelPath, makeEditor);
    };
    app.settings().setString("cameraMode", "fly");
    app.pushState(makeEditor());
    app.begin();

    // Qt owns the loop; the engine steps one frame per timer tick.
    QTimer frameTimer;
    QObject::connect(&frameTimer, &QTimer::timeout, [&]() {
        if (!app.running()) {
            qtApp.quit();
            return;
        }
        app.runFrame();
    });
    frameTimer.start(16);

    int result = qtApp.exec();
    app.end();
    return result;
}
