#define GLFW_INCLUDE_NONE
#include "window.h"
#include <GLFW/glfw3.h>

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

// All GLFW-typed state lives here, out of the public header. The GLFW user
// pointer is set to this Impl, so the file-local callbacks below operate on it
// directly without going back through Window.
struct Window::Impl {
    GLFWwindow* window = nullptr;
    InputState input;
    std::vector<Event> events;
    GamepadSet gamepads;
    std::function<void()> drawCallback;
    double lastMouseX = 0, lastMouseY = 0;
    double lastFrameTime = 0;
    double deltaTime = 0;
    bool firstMouse = true;
};

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

static Window::Impl* implOf(GLFWwindow* window) {
    return static_cast<Window::Impl*>(glfwGetWindowUserPointer(window));
}

// Translate GLFW's gamepad snapshot into our backend-neutral one. Our
// GamepadButton/GamepadAxis enums mirror GLFW's standard layout order, but we
// map explicitly (and normalize triggers from GLFW's [-1, 1] to [0, 1]) so the
// neutral types stay decoupled from GLFW values.
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

Window::Window() : impl(std::make_unique<Impl>()) {}

Window::~Window() {
    shutdown();
}

bool Window::initialize(int width, int height, const std::string& title) {
    if (!glfwInit()) return false;

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

    impl->lastFrameTime = glfwGetTime();
    return true;
}

void Window::shutdown() {
    if (impl->window) {
        glfwDestroyWindow(impl->window);
        impl->window = nullptr;
    }
    glfwTerminate();
}

bool Window::shouldClose() const {
    return glfwWindowShouldClose(impl->window);
}

void Window::pollEvents() {
    impl->input.mouseDeltaX = 0;
    impl->input.mouseDeltaY = 0;
    impl->input.scrollDelta = 0;
    impl->events.clear();

    glfwPollEvents();

    GLFWwindow* window = impl->window;
    impl->input.mouseLeftDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    impl->input.mouseRightDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    impl->input.keyW = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    impl->input.keyA = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    impl->input.keyS = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
    impl->input.keyD = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
    impl->input.keyQ = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
    impl->input.keyE = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
    impl->input.keyShift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    impl->input.keyUp = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    impl->input.keyDown = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;

    // Poll every gamepad slot; surface connect/disconnect transitions as events.
    for (int jid = 0; jid < MAX_GAMEPADS; jid++) {
        GamepadState& slot = impl->gamepads[jid];
        bool wasConnected = slot.connected;

        GLFWgamepadstate gs;
        if (glfwJoystickIsGamepad(jid) && glfwGetGamepadState(jid, &gs)) {
            fillGamepadState(slot, gs);
        } else {
            slot = GamepadState{};
        }

        if (slot.connected && !wasConnected) {
            Event event(EventType::GamepadConnected);
            event.gamepad = jid;
            impl->events.push_back(event);
        } else if (!slot.connected && wasConnected) {
            Event event(EventType::GamepadDisconnected);
            event.gamepad = jid;
            impl->events.push_back(event);
        }
    }

    double now = glfwGetTime();
    impl->deltaTime = now - impl->lastFrameTime;
    impl->lastFrameTime = now;

    newDebugUiFrame();   // ImGui GLFW new-frame (no-op without RT_ENABLE_IMGUI)
}

void Window::getSize(int& width, int& height) const {
    glfwGetWindowSize(impl->window, &width, &height);
}

void Window::getFramebufferSize(int& width, int& height) const {
    glfwGetFramebufferSize(impl->window, &width, &height);
}

void* Window::nativeWindowHandle() const {
#if defined(__APPLE__)
    return glfwGetCocoaWindow(impl->window);
#elif defined(_WIN32)
    return glfwGetWin32Window(impl->window);
#else
    return nullptr;
#endif
}

const InputState& Window::getInput() const {
    return impl->input;
}

const std::vector<Event>& Window::getEvents() const {
    return impl->events;
}

const GamepadSet& Window::getGamepads() const {
    return impl->gamepads;
}

double Window::getDeltaTime() const {
    return impl->deltaTime;
}

void Window::setDrawCallback(std::function<void()> callback) {
    impl->drawCallback = std::move(callback);
}

// Debug-UI GLFW backend (ADR-0011). The ImGui platform backend lives here, the
// one file that owns the GLFWwindow*. No-op without ImGui.
void Window::initDebugUi() {
#ifdef RT_ENABLE_IMGUI
    // The ImGui context already exists (Renderer::initDebugUi ran first).
    // InitForOther is the right entry point for a non-GL/Vulkan (Metal) backend;
    // true installs ImGui's GLFW input callbacks, chained to ours.
    ImGui_ImplGlfw_InitForOther(impl->window, true);
#endif
}

void Window::newDebugUiFrame() {
#ifdef RT_ENABLE_IMGUI
    ImGui_ImplGlfw_NewFrame();
#endif
}

void Window::shutdownDebugUi() {
#ifdef RT_ENABLE_IMGUI
    ImGui_ImplGlfw_Shutdown();
#endif
}
