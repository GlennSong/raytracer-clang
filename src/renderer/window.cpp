#define GLFW_INCLUDE_NONE
// Include Vulkan before glfw3.h (only where the build has it) so GLFW declares
// its Vulkan interop — glfwCreateWindowSurface / glfwGetRequiredInstanceExtensions
// — without us defining GLFW_INCLUDE_VULKAN (which would force vulkan.h on macOS).
#if defined(RT_HAVE_VULKAN)
#include <vulkan/vulkan.h>
#endif
#include "window.h"
#include "gamepad_gc.h"
#include "../log.h"
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#include <cmath>
#endif

#ifdef RT_ENABLE_IMGUI
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#endif

// Native window-handle access is the one genuinely per-platform piece of the
// window layer. It is confined here so the backend (and engine) never touch a
// windowing-library symbol. Add a branch per platform as backends land.
#if defined(__APPLE__)
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3native.h>
#elif defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

namespace engine {

// The GLFW implementation of the Window seam. File-local: hosts reach it only
// through createPlatformWindow().
class GlfwWindow final : public Window {
public:
    GlfwWindow();
    ~GlfwWindow() override;

    bool initialize(int width, int height, const std::string& title) override;
    void shutdown() override;
    bool shouldClose() const override;
    void pollEvents() override;
    void getSize(int& width, int& height) const override;
    bool setSize(int width, int height) override;
    void getFramebufferSize(int& width, int& height) const override;
    void* nativeWindowHandle() const override;
    bool createVulkanSurface(void* instance, uint64_t* outSurface) const override;
    std::vector<std::string> requiredVulkanInstanceExtensions() const override;
    const InputState& getInput() const override;
    const std::vector<Event>& getEvents() const override;
    const GamepadSet& getGamepads() const override;
    double getDeltaTime() const override;
    void setDrawCallback(std::function<void()> callback) override;
    void setCursorMode(CursorMode mode) override;
    void resetMouseDelta() override;
    void initDebugUi() override;
    void newDebugUiFrame() override;
    void shutdownDebugUi() override;

    // Public so the file-local GLFW callbacks below can name the type.
    struct Impl;

private:
    std::unique_ptr<Impl> impl;
};

std::unique_ptr<Window> createPlatformWindow() {
    return std::make_unique<GlfwWindow>();
}

// All GLFW-typed state lives here, out of the public header. The GLFW user
// pointer is set to this Impl, so the file-local callbacks below operate on it
// directly without going back through Window.
struct GlfwWindow::Impl {
    GLFWwindow* window = nullptr;
    InputState input;
    std::vector<Event> events;
    GamepadSet gamepads;
    std::array<bool, MAX_GAMEPADS> prevConnected{};
    std::function<void()> drawCallback;
    double lastMouseX = 0, lastMouseY = 0;
    double lastFrameTime = 0;
    double deltaTime = 0;
    bool firstMouse = true;

#ifdef __EMSCRIPTEN__
    // Touch state (ADR-0058). Gestures map onto the mouse InputState the camera
    // already reads. Deltas accumulate in the touch callbacks (which fire between
    // frames) and are drained in pollEvents.
    double touchAccumDX = 0, touchAccumDY = 0, touchAccumScroll = 0;
    double lastTouchX = 0, lastTouchY = 0, lastPinchDist = 0;
    bool touchLeftDown = false, touchRightDown = false, touchPinch = false;
    // Keyboard fed from JS via rt_web_key (W,S,A,D,E,Q,Shift,Num1,Num2);
    // pollEvents turns edges into the same KeyPressed/Released the keyboard emits.
    bool vkey[9] = {};
    bool vkeyPrev[9] = {};
    // Virtual gamepad driven by the page's on-screen sticks (left = move, right
    // = look) — fed to the engine as gamepad 0, the analog path the camera reads.
    GamepadState virtualPad;
#endif
};

#ifdef __EMSCRIPTEN__
// The live window's Impl, so the exported rt_web_key (called from the page's
// on-screen D-pad) can reach the input state. One window per web app lifetime.
static GlfwWindow::Impl* g_activeImpl = nullptr;
#endif

#ifdef __EMSCRIPTEN__
// Touch -> mouse mapping for the web. iOS/Android have no GLFW mouse, so map
// gestures onto the same InputState the camera reads: one-finger drag = left-drag
// (orbit), two-finger drag = right-drag (pan) + pinch = scroll (zoom). Returns
// EM_TRUE to consume the event (the canvas has touch-action: none).
static double rtTouchDist(const EmscriptenTouchPoint& a, const EmscriptenTouchPoint& b) {
    double dx = a.targetX - b.targetX, dy = a.targetY - b.targetY;
    return std::sqrt(dx * dx + dy * dy);
}
static EM_BOOL onTouchStart(int, const EmscriptenTouchEvent* e, void* user) {
    auto* impl = static_cast<GlfwWindow::Impl*>(user);
    if (e->numTouches >= 2) {
        impl->touchPinch = true; impl->touchLeftDown = false; impl->touchRightDown = true;
        impl->lastPinchDist = rtTouchDist(e->touches[0], e->touches[1]);
        impl->lastTouchX = (e->touches[0].targetX + e->touches[1].targetX) * 0.5;
        impl->lastTouchY = (e->touches[0].targetY + e->touches[1].targetY) * 0.5;
    } else if (e->numTouches == 1) {
        impl->touchPinch = false; impl->touchLeftDown = true; impl->touchRightDown = false;
        impl->lastTouchX = e->touches[0].targetX; impl->lastTouchY = e->touches[0].targetY;
        impl->input.mouseX = e->touches[0].targetX; impl->input.mouseY = e->touches[0].targetY;
    }
    return EM_TRUE;
}
static EM_BOOL onTouchMove(int, const EmscriptenTouchEvent* e, void* user) {
    auto* impl = static_cast<GlfwWindow::Impl*>(user);
    if (e->numTouches >= 2) {
        double d = rtTouchDist(e->touches[0], e->touches[1]);
        impl->touchAccumScroll += (d - impl->lastPinchDist) * 0.01;  // px -> scroll units
        impl->lastPinchDist = d;
        double cx = (e->touches[0].targetX + e->touches[1].targetX) * 0.5;
        double cy = (e->touches[0].targetY + e->touches[1].targetY) * 0.5;
        impl->touchAccumDX += cx - impl->lastTouchX;
        impl->touchAccumDY += cy - impl->lastTouchY;
        impl->lastTouchX = cx; impl->lastTouchY = cy;
    } else if (e->numTouches == 1) {
        double x = e->touches[0].targetX, y = e->touches[0].targetY;
        impl->touchAccumDX += x - impl->lastTouchX;
        impl->touchAccumDY += y - impl->lastTouchY;
        impl->lastTouchX = x; impl->lastTouchY = y;
        impl->input.mouseX = x; impl->input.mouseY = y;
    }
    return EM_TRUE;
}
static EM_BOOL onTouchEnd(int, const EmscriptenTouchEvent*, void* user) {
    auto* impl = static_cast<GlfwWindow::Impl*>(user);
    impl->touchLeftDown = false; impl->touchRightDown = false; impl->touchPinch = false;
    return EM_TRUE;
}

// Called from the page's on-screen movement buttons. idx: 0=W 1=S 2=A 3=D 4=E
// 5=Q 6=Shift 7=Num1 8=Num2; down = press/release. pollEvents converts the
// change into the keyboard events the InputMap binds to camera/gameplay actions
// (Num1/Num2 are the weapon-slot keys play mode binds to slot_1/slot_2).
extern "C" EMSCRIPTEN_KEEPALIVE void rt_web_key(int idx, int down) {
    if (g_activeImpl && idx >= 0 && idx < 9) g_activeImpl->vkey[idx] = (down != 0);
}

// On-screen analog sticks. stick 0 = left (move) -> LeftX/LeftY; stick 1 = right
// (look) -> RightX/RightY. Components in [-1, 1] (x right+, y down+).
extern "C" EMSCRIPTEN_KEEPALIVE void rt_web_axis(int stick, float x, float y) {
    if (!g_activeImpl) return;
    auto& a = g_activeImpl->virtualPad.axes;
    if (stick == 0) {
        a[static_cast<size_t>(GamepadAxis::LeftX)] = x;
        a[static_cast<size_t>(GamepadAxis::LeftY)] = y;
    } else if (stick == 1) {
        a[static_cast<size_t>(GamepadAxis::RightX)] = x;
        a[static_cast<size_t>(GamepadAxis::RightY)] = y;
    }
}

// On-screen gamepad buttons: idx 0 = boost (LeftBumper), 1 = camera toggle
// (Back, flips fly/orbit via cam_toggle), 2 = fire (RightBumper, the gameplay
// fire binding play mode reads).
extern "C" EMSCRIPTEN_KEEPALIVE void rt_web_button(int idx, int down) {
    if (!g_activeImpl) return;
    GamepadButton b;
    switch (idx) {
        case 1:  b = GamepadButton::Back; break;
        case 2:  b = GamepadButton::RightBumper; break;
        default: b = GamepadButton::LeftBumper; break;
    }
    g_activeImpl->virtualPad.buttons[static_cast<size_t>(b)] = (down != 0);
}

// Desktop mouse-look: feed a look delta (px) into the same accumulator touch uses,
// which pollEvents drains into the camera's mouse delta. Bypasses Emscripten
// GLFW's mouse path (which doesn't reliably deliver events in this build).
extern "C" EMSCRIPTEN_KEEPALIVE void rt_web_look(float dx, float dy) {
    if (!g_activeImpl) return;
    g_activeImpl->touchAccumDX += dx;
    g_activeImpl->touchAccumDY += dy;
}
#endif  // __EMSCRIPTEN__

// Maps backend (GLFW) key codes to the backend-independent KeyCode the rest of
// the engine sees. This table is the only place that knows GLFW key values.
static KeyCode translateKey(int glfwKey) {
    if (glfwKey >= GLFW_KEY_A && glfwKey <= GLFW_KEY_Z)
        return static_cast<KeyCode>(static_cast<int>(KeyCode::A) + (glfwKey - GLFW_KEY_A));
    if (glfwKey >= GLFW_KEY_0 && glfwKey <= GLFW_KEY_9)
        return static_cast<KeyCode>(static_cast<int>(KeyCode::Num0) + (glfwKey - GLFW_KEY_0));

    switch (glfwKey) {
        case GLFW_KEY_UP:            return KeyCode::Up;
        case GLFW_KEY_DOWN:          return KeyCode::Down;
        case GLFW_KEY_LEFT:          return KeyCode::Left;
        case GLFW_KEY_RIGHT:         return KeyCode::Right;
        case GLFW_KEY_ESCAPE:        return KeyCode::Escape;
        case GLFW_KEY_SPACE:         return KeyCode::Space;
        case GLFW_KEY_ENTER:         return KeyCode::Enter;
        case GLFW_KEY_TAB:           return KeyCode::Tab;
        case GLFW_KEY_BACKSPACE:     return KeyCode::Backspace;
        case GLFW_KEY_LEFT_SHIFT:    return KeyCode::LeftShift;
        case GLFW_KEY_RIGHT_SHIFT:   return KeyCode::RightShift;
        case GLFW_KEY_LEFT_CONTROL:  return KeyCode::LeftControl;
        case GLFW_KEY_RIGHT_CONTROL: return KeyCode::RightControl;
        case GLFW_KEY_LEFT_ALT:      return KeyCode::LeftAlt;
        case GLFW_KEY_RIGHT_ALT:     return KeyCode::RightAlt;
        case GLFW_KEY_COMMA:         return KeyCode::Comma;
        case GLFW_KEY_PERIOD:        return KeyCode::Period;
        case GLFW_KEY_SLASH:         return KeyCode::Slash;
        case GLFW_KEY_SEMICOLON:     return KeyCode::Semicolon;
        case GLFW_KEY_MINUS:         return KeyCode::Minus;
        case GLFW_KEY_EQUAL:         return KeyCode::Equal;
        case GLFW_KEY_LEFT_BRACKET:  return KeyCode::LeftBracket;
        case GLFW_KEY_RIGHT_BRACKET: return KeyCode::RightBracket;
        case GLFW_KEY_GRAVE_ACCENT:  return KeyCode::GraveAccent;
        default:                     return KeyCode::Unknown;
    }
}

static MouseButton translateButton(int glfwButton) {
    switch (glfwButton) {
        case GLFW_MOUSE_BUTTON_RIGHT:  return MouseButton::Right;
        case GLFW_MOUSE_BUTTON_MIDDLE: return MouseButton::Middle;
        default:                       return MouseButton::Left;
    }
}

static GlfwWindow::Impl* implOf(GLFWwindow* window) {
    return static_cast<GlfwWindow::Impl*>(glfwGetWindowUserPointer(window));
}

// Translate GLFW's gamepad snapshot into our backend-neutral one. Our
// GamepadButton/GamepadAxis enums mirror GLFW's standard layout order, but we
// map explicitly (and normalize triggers from GLFW's [-1, 1] to [0, 1]) so the
// neutral types stay decoupled from GLFW values.
#ifndef __EMSCRIPTEN__  // unused on the web (no glfwGetGamepadState there)
static void fillGamepadState(GamepadState& out, const GLFWgamepadstate& in) {
    out.connected = true;
    for (std::size_t i = 0; i < GAMEPAD_BUTTON_COUNT; i++)
        out.buttons[i] = (in.buttons[i] == GLFW_PRESS);

    out.axes[static_cast<std::size_t>(GamepadAxis::LeftX)] =
        in.axes[GLFW_GAMEPAD_AXIS_LEFT_X];
    out.axes[static_cast<std::size_t>(GamepadAxis::LeftY)] =
        in.axes[GLFW_GAMEPAD_AXIS_LEFT_Y];
    out.axes[static_cast<std::size_t>(GamepadAxis::RightX)] =
        in.axes[GLFW_GAMEPAD_AXIS_RIGHT_X];
    out.axes[static_cast<std::size_t>(GamepadAxis::RightY)] =
        in.axes[GLFW_GAMEPAD_AXIS_RIGHT_Y];
    out.axes[static_cast<std::size_t>(GamepadAxis::LeftTrigger)] =
        (in.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER] + 1.0f) * 0.5f;
    out.axes[static_cast<std::size_t>(GamepadAxis::RightTrigger)] =
        (in.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] + 1.0f) * 0.5f;
}
#endif

static void onCursorPos(GLFWwindow* window, double xpos, double ypos) {
    auto* impl = implOf(window);
    if (impl->firstMouse) {
        impl->lastMouseX = xpos;
        impl->lastMouseY = ypos;
        impl->firstMouse = false;
    }
    impl->input.mouseDeltaX = xpos - impl->lastMouseX;
    impl->input.mouseDeltaY = ypos - impl->lastMouseY;
    impl->lastMouseX = xpos;
    impl->lastMouseY = ypos;
    impl->input.mouseX = xpos;
    impl->input.mouseY = ypos;

    Event event(EventType::MouseMoved);
    event.x = xpos;
    event.y = ypos;
    impl->events.push_back(event);
}

static void onScroll(GLFWwindow* window, double xoffset, double yoffset) {
    auto* impl = implOf(window);
    impl->input.scrollDelta = yoffset;

    Event event(EventType::MouseScrolled);
    event.x = xoffset;
    event.y = yoffset;
    impl->events.push_back(event);
}

static void onKey(GLFWwindow* window, int key, int, int action, int) {
    auto* impl = implOf(window);
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        Event event(EventType::KeyPressed);
        event.key = translateKey(key);
        event.repeat = (action == GLFW_REPEAT);
        impl->events.push_back(event);
    } else if (action == GLFW_RELEASE) {
        Event event(EventType::KeyReleased);
        event.key = translateKey(key);
        impl->events.push_back(event);
    }
}

static void onMouseButton(GLFWwindow* window, int button, int action, int) {
    auto* impl = implOf(window);
    EventType type = (action == GLFW_PRESS) ? EventType::MouseButtonPressed
                                            : EventType::MouseButtonReleased;
    Event event(type);
    event.button = translateButton(button);
    event.x = impl->input.mouseX;
    event.y = impl->input.mouseY;
    impl->events.push_back(event);
}

static void onFramebufferSize(GLFWwindow* window, int width, int height) {
    auto* impl = implOf(window);
    Event event(EventType::FramebufferResized);
    event.width = width;
    event.height = height;
    impl->events.push_back(event);
}

static void onWindowSize(GLFWwindow* window, int width, int height) {
    auto* impl = implOf(window);
    Event event(EventType::WindowResized);
    event.width = width;
    event.height = height;
    impl->events.push_back(event);
}

static void onWindowFocus(GLFWwindow* window, int focused) {
    auto* impl = implOf(window);
    impl->events.push_back(Event(focused ? EventType::WindowFocused
                                         : EventType::WindowUnfocused));
}

static void onWindowIconify(GLFWwindow* window, int iconified) {
    auto* impl = implOf(window);
    impl->events.push_back(Event(iconified ? EventType::WindowMinimized
                                           : EventType::WindowRestored));
}

static void onWindowClose(GLFWwindow* window) {
    auto* impl = implOf(window);
    impl->events.push_back(Event(EventType::WindowCloseRequested));
}

static void onWindowRefresh(GLFWwindow* window) {
    auto* impl = implOf(window);
    if (impl->drawCallback) impl->drawCallback();
}

GlfwWindow::GlfwWindow() : impl(std::make_unique<Impl>()) {}

GlfwWindow::~GlfwWindow() {
    shutdown();
}

bool GlfwWindow::initialize(int width, int height, const std::string& title) {
    if (!glfwInit()) return false;
#ifdef GLFW_PLATFORM_WAYLAND
    // Which backend GLFW picked matters for the clipboard: under XWayland an
    // X11 app's copy reaches Wayland apps only while it holds focus (flaky);
    // native Wayland copies reliably. Say which one this window is.
    {
        const int platform = glfwGetPlatform();
        LOG_INFO << "GLFW " << glfwGetVersionString() << " platform: "
                 << (platform == GLFW_PLATFORM_WAYLAND ? "wayland"
                     : platform == GLFW_PLATFORM_X11   ? "x11 (XWayland if the session is Wayland)"
                     : platform == GLFW_PLATFORM_COCOA ? "cocoa"
                     : platform == GLFW_PLATFORM_WIN32 ? "win32" : "other");
    }
#endif

    // Emscripten's GLFW shim doesn't implement the gamepad-mapping API; the web
    // build skips loading the SDL controller DB (browser gamepads come mapped).
#ifndef __EMSCRIPTEN__
    {
        std::ifstream file("gamecontrollerdb.txt");
        if (file) {
            std::ostringstream buf;
            buf << file.rdbuf();
            std::string mappings = buf.str();
            if (glfwUpdateGamepadMappings(mappings.c_str()))
                LOG_INFO << "Loaded gamepad mappings from gamecontrollerdb.txt";
            else
                LOG_WARN << "gamecontrollerdb.txt found but glfwUpdateGamepadMappings failed";
        }
    }
#endif

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    impl->window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!impl->window) {
        glfwTerminate();
        return false;
    }

    glfwSetWindowUserPointer(impl->window, impl.get());
    glfwSetCursorPosCallback(impl->window, onCursorPos);
    glfwSetScrollCallback(impl->window, onScroll);
    glfwSetKeyCallback(impl->window, onKey);
    glfwSetMouseButtonCallback(impl->window, onMouseButton);
    glfwSetFramebufferSizeCallback(impl->window, onFramebufferSize);
    glfwSetWindowSizeCallback(impl->window, onWindowSize);
    glfwSetWindowFocusCallback(impl->window, onWindowFocus);
    glfwSetWindowIconifyCallback(impl->window, onWindowIconify);
    glfwSetWindowCloseCallback(impl->window, onWindowClose);
    glfwSetWindowRefreshCallback(impl->window, onWindowRefresh);

#ifdef __EMSCRIPTEN__
    g_activeImpl = impl.get();
    impl->virtualPad.connected = true;  // on-screen sticks act as gamepad 0
    // Touch input on the canvas (the web has no GLFW mouse). useCapture=true.
    emscripten_set_touchstart_callback("#canvas", impl.get(), EM_TRUE, onTouchStart);
    emscripten_set_touchmove_callback("#canvas", impl.get(), EM_TRUE, onTouchMove);
    emscripten_set_touchend_callback("#canvas", impl.get(), EM_TRUE, onTouchEnd);
    emscripten_set_touchcancel_callback("#canvas", impl.get(), EM_TRUE, onTouchEnd);
#endif

    impl->lastFrameTime = glfwGetTime();
    return true;
}

void GlfwWindow::shutdown() {
    if (impl->window) {
        glfwDestroyWindow(impl->window);
        impl->window = nullptr;
    }
    glfwTerminate();
}

void GlfwWindow::setCursorMode(CursorMode mode) {
#ifdef __EMSCRIPTEN__
    // No-op on the web. iOS Safari has no Pointer Lock API at all, so *both*
    // requestPointerLock (CURSOR_DISABLED) and exitPointerLock (CURSOR_NORMAL)
    // are undefined and throw — and the throw during startup aborts main()
    // before the render loop registers. So touch nothing here.
    (void)mode;
#else
    int glfwMode = (mode == CursorMode::Disabled) ? GLFW_CURSOR_DISABLED
                                                  : GLFW_CURSOR_NORMAL;
    glfwSetInputMode(impl->window, GLFW_CURSOR, glfwMode);
#endif
}

void GlfwWindow::resetMouseDelta() {
    impl->firstMouse = true;
    impl->input.mouseDeltaX = 0;
    impl->input.mouseDeltaY = 0;
}

bool GlfwWindow::shouldClose() const {
    return glfwWindowShouldClose(impl->window);
}

void GlfwWindow::pollEvents() {
    impl->input.mouseDeltaX = 0;
    impl->input.mouseDeltaY = 0;
    impl->input.scrollDelta = 0;
    impl->events.clear();

    glfwPollEvents();

    GLFWwindow* window = impl->window;
#ifdef __EMSCRIPTEN__
    // Desktop browser: real mouse comes through GLFW (onCursorPos/onScroll set the
    // deltas during glfwPollEvents). Add the touch accumulators on top, and OR the
    // touch button state with the real mouse buttons — so keyboard+mouse work on
    // desktop and touch works on mobile, from the same build.
    impl->input.mouseDeltaX += impl->touchAccumDX; impl->touchAccumDX = 0;
    impl->input.mouseDeltaY += impl->touchAccumDY; impl->touchAccumDY = 0;
    impl->input.scrollDelta += impl->touchAccumScroll; impl->touchAccumScroll = 0;
    impl->input.mouseLeftDown =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) || impl->touchLeftDown;
    impl->input.mouseRightDown =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) || impl->touchRightDown;
    // Turn on-screen button state changes into keyboard events (same path as a
    // real keyboard → InputMap → camera move/boost axes).
    static const KeyCode kVKeys[9] = {KeyCode::W, KeyCode::S, KeyCode::A, KeyCode::D,
                                      KeyCode::E, KeyCode::Q, KeyCode::LeftShift,
                                      KeyCode::Num1, KeyCode::Num2};
    for (int i = 0; i < 9; ++i) {
        if (impl->vkey[i] == impl->vkeyPrev[i]) continue;
        Event ev(impl->vkey[i] ? EventType::KeyPressed : EventType::KeyReleased);
        ev.key = kVKeys[i];
        impl->events.push_back(ev);
        impl->vkeyPrev[i] = impl->vkey[i];
    }
#else
    impl->input.mouseLeftDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    impl->input.mouseRightDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
#endif
    impl->input.keyW = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    impl->input.keyA = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    impl->input.keyS = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
    impl->input.keyD = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
    impl->input.keyQ = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
    impl->input.keyE = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
    impl->input.keyShift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    impl->input.keyUp = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    impl->input.keyDown = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;

    // Poll every gamepad slot. GLFW's IOKit path handles controllers it
    // recognizes; the GCController layer (macOS-only) overwrites slots where
    // Apple's Game Controller framework provides data — this is the only
    // reliable path for Xbox/PS controllers on macOS 13+ where Apple's
    // DriverKit driver intercepts the USB device (ADR-0010).
    for (int jid = 0; jid < MAX_GAMEPADS; jid++) {
        GamepadState& slot = impl->gamepads[jid];

        // Emscripten's GLFW shim lacks glfwGetGamepadState; the web build leaves
        // the slots cleared (browser gamepad support is a later phase).
#ifndef __EMSCRIPTEN__
        GLFWgamepadstate gs;
        if (glfwJoystickIsGamepad(jid) && glfwGetGamepadState(jid, &gs)) {
            fillGamepadState(slot, gs);
        } else {
            slot = GamepadState{};
        }
#else
        slot = (jid == 0) ? impl->virtualPad : GamepadState{};  // on-screen sticks
#endif
    }

    // GCController overlay: fills slots that GLFW left empty (or overwrites
    // with better data). Does nothing on non-Apple platforms.
    gcPollGamepads(impl->gamepads);

    // Surface connect/disconnect transitions as events. Run after both
    // backends have had their say.
    for (int jid = 0; jid < MAX_GAMEPADS; jid++) {
        GamepadState& slot = impl->gamepads[jid];
        bool wasConnected = impl->prevConnected[jid];
        impl->prevConnected[jid] = slot.connected;

        if (slot.connected && !wasConnected) {
            const char* name = glfwJoystickPresent(jid)
                ? glfwGetJoystickName(jid) : nullptr;
            LOG_INFO << "Gamepad " << jid << " connected"
                     << (name ? std::string(": \"") + name + "\"" : "");
            Event event(EventType::GamepadConnected);
            event.gamepad = jid;
            impl->events.push_back(event);
        } else if (!slot.connected && wasConnected) {
            LOG_INFO << "Gamepad " << jid << " disconnected";
            Event event(EventType::GamepadDisconnected);
            event.gamepad = jid;
            impl->events.push_back(event);
        }
    }

    double now = glfwGetTime();
    impl->deltaTime = now - impl->lastFrameTime;
    impl->lastFrameTime = now;

    newDebugUiFrame();   // ImGui GLFW new-frame (no-op without RT_ENABLE_IMGUI)

#ifdef RT_ENABLE_IMGUI
    // After the UI new-frame so hover/drag state is current for this frame.
    // A DISABLED cursor can never hover UI: once mouse-look recaptures the
    // pointer, ImGui stops receiving cursor positions, so WantCaptureMouse
    // can stay latched on whatever was hovered when the overlay closed —
    // which silently killed mouse-look until the overlay was reopened
    // (device: "the mouse loses the ability to look around after I open and
    // then close the imgui window").
    impl->input.uiWantsMouse =
        ImGui::GetIO().WantCaptureMouse &&
        glfwGetInputMode(impl->window, GLFW_CURSOR) != GLFW_CURSOR_DISABLED;
#endif
}

void GlfwWindow::getSize(int& width, int& height) const {
    glfwGetWindowSize(impl->window, &width, &height);
}

bool GlfwWindow::setSize(int width, int height) {
    glfwSetWindowSize(impl->window, width, height);
    return true;
}

void GlfwWindow::getFramebufferSize(int& width, int& height) const {
    glfwGetFramebufferSize(impl->window, &width, &height);
}

void* GlfwWindow::nativeWindowHandle() const {
#if defined(__APPLE__)
    return glfwGetCocoaWindow(impl->window);
#elif defined(_WIN32)
    return glfwGetWin32Window(impl->window);
#else
    return nullptr;
#endif
}

bool GlfwWindow::createVulkanSurface(void* instance, uint64_t* outSurface) const {
#if defined(RT_HAVE_VULKAN)
    if (!instance || !outSurface) return false;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkResult result = glfwCreateWindowSurface(
        reinterpret_cast<VkInstance>(instance), impl->window, nullptr, &surface);
    if (result != VK_SUCCESS) {
        LOG_ERROR("glfwCreateWindowSurface failed (VkResult %d)", static_cast<int>(result));
        return false;
    }
    // VkSurfaceKHR is a non-dispatchable handle (always 64-bit); carry it across
    // the seam as a uint64_t so window.h stays free of Vulkan headers.
    *outSurface = reinterpret_cast<uint64_t>(surface);
    return true;
#else
    (void)instance;
    (void)outSurface;
    return false;
#endif
}

std::vector<std::string> GlfwWindow::requiredVulkanInstanceExtensions() const {
#if defined(RT_HAVE_VULKAN)
    uint32_t count = 0;
    const char** names = glfwGetRequiredInstanceExtensions(&count);
    std::vector<std::string> extensions;
    extensions.reserve(count);
    for (uint32_t i = 0; i < count; ++i) extensions.emplace_back(names[i]);
    return extensions;
#else
    return {};
#endif
}

const InputState& GlfwWindow::getInput() const {
    return impl->input;
}

const std::vector<Event>& GlfwWindow::getEvents() const {
    return impl->events;
}

const GamepadSet& GlfwWindow::getGamepads() const {
    return impl->gamepads;
}

double GlfwWindow::getDeltaTime() const {
    return impl->deltaTime;
}

void GlfwWindow::setDrawCallback(std::function<void()> callback) {
    impl->drawCallback = std::move(callback);
}

// Debug-UI GLFW backend (ADR-0011). The ImGui platform backend lives here, the
// one file that owns the GLFWwindow*. No-op without ImGui.
void GlfwWindow::initDebugUi() {
#ifdef RT_ENABLE_IMGUI
    // The ImGui context already exists (Renderer::initDebugUi ran first).
    // InitForOther is the right entry point for a non-GL/Vulkan (Metal) backend;
    // true installs ImGui's GLFW input callbacks, chained to ours.
    ImGui_ImplGlfw_InitForOther(impl->window, true);
#endif
}

void GlfwWindow::newDebugUiFrame() {
#ifdef RT_ENABLE_IMGUI
    ImGui_ImplGlfw_NewFrame();
#endif
}

void GlfwWindow::shutdownDebugUi() {
#ifdef RT_ENABLE_IMGUI
    ImGui_ImplGlfw_Shutdown();
#endif
}

}  // namespace engine
