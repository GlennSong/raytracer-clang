#define GLFW_INCLUDE_NONE
#include "window.h"
#include <GLFW/glfw3.h>

Window::Window()
    : window(nullptr), lastMouseX(0), lastMouseY(0),
      lastFrameTime(0), deltaTime(0), firstMouse(true) {}

Window::~Window() {
    shutdown();
}

bool Window::initialize(int width, int height, const std::string& title) {
    if (!glfwInit()) return false;

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return false;
    }

    glfwSetWindowUserPointer(window, this);
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetWindowSizeCallback(window, windowSizeCallback);
    glfwSetWindowFocusCallback(window, windowFocusCallback);
    glfwSetWindowIconifyCallback(window, windowIconifyCallback);
    glfwSetWindowCloseCallback(window, windowCloseCallback);

    lastFrameTime = glfwGetTime();
    return true;
}

void Window::shutdown() {
    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
}

bool Window::shouldClose() const {
    return glfwWindowShouldClose(window);
}

void Window::pollEvents() {
    input.mouseDeltaX = 0;
    input.mouseDeltaY = 0;
    input.scrollDelta = 0;
    events.clear();

    glfwPollEvents();

    input.mouseLeftDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    input.mouseRightDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    input.keyW = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    input.keyA = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    input.keyS = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
    input.keyD = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
    input.keyQ = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
    input.keyE = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
    input.keyShift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    input.keyUp = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    input.keyDown = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;

    double now = glfwGetTime();
    deltaTime = now - lastFrameTime;
    lastFrameTime = now;
}

void Window::getSize(int& width, int& height) const {
    glfwGetWindowSize(window, &width, &height);
}

void Window::getFramebufferSize(int& width, int& height) const {
    glfwGetFramebufferSize(window, &width, &height);
}

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

void Window::mouseCallback(GLFWwindow* glfwWindow, double xpos, double ypos) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    if (self->firstMouse) {
        self->lastMouseX = xpos;
        self->lastMouseY = ypos;
        self->firstMouse = false;
    }
    self->input.mouseDeltaX = xpos - self->lastMouseX;
    self->input.mouseDeltaY = ypos - self->lastMouseY;
    self->lastMouseX = xpos;
    self->lastMouseY = ypos;
    self->input.mouseX = xpos;
    self->input.mouseY = ypos;

    Event event(EventType::MouseMoved);
    event.x = xpos;
    event.y = ypos;
    self->events.push_back(event);
}

void Window::scrollCallback(GLFWwindow* glfwWindow, double xoffset, double yoffset) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    self->input.scrollDelta = yoffset;

    Event event(EventType::MouseScrolled);
    event.x = xoffset;
    event.y = yoffset;
    self->events.push_back(event);
}

void Window::keyCallback(GLFWwindow* glfwWindow, int key, int, int action, int) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        Event event(EventType::KeyPressed);
        event.key = translateKey(key);
        event.repeat = (action == GLFW_REPEAT);
        self->events.push_back(event);
    } else if (action == GLFW_RELEASE) {
        Event event(EventType::KeyReleased);
        event.key = translateKey(key);
        self->events.push_back(event);
    }
}

void Window::mouseButtonCallback(GLFWwindow* glfwWindow, int button, int action, int) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    EventType type = (action == GLFW_PRESS) ? EventType::MouseButtonPressed
                                            : EventType::MouseButtonReleased;
    Event event(type);
    event.button = translateButton(button);
    event.x = self->input.mouseX;
    event.y = self->input.mouseY;
    self->events.push_back(event);
}

void Window::framebufferSizeCallback(GLFWwindow* glfwWindow, int width, int height) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    Event event(EventType::FramebufferResized);
    event.width = width;
    event.height = height;
    self->events.push_back(event);
}

void Window::windowSizeCallback(GLFWwindow* glfwWindow, int width, int height) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    Event event(EventType::WindowResized);
    event.width = width;
    event.height = height;
    self->events.push_back(event);
}

void Window::windowFocusCallback(GLFWwindow* glfwWindow, int focused) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    self->events.push_back(Event(focused ? EventType::WindowFocused
                                         : EventType::WindowUnfocused));
}

void Window::windowIconifyCallback(GLFWwindow* glfwWindow, int iconified) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    self->events.push_back(Event(iconified ? EventType::WindowMinimized
                                           : EventType::WindowRestored));
}

void Window::windowCloseCallback(GLFWwindow* glfwWindow) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    self->events.push_back(Event(EventType::WindowCloseRequested));
}
