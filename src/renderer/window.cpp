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
    input.keyEscape = glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
    input.keySpace = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    input.keyComma = glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS;
    input.keyPeriod = glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS;
    input.keyNum0 = glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS;

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
}

void Window::scrollCallback(GLFWwindow* glfwWindow, double, double yoffset) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    self->input.scrollDelta = yoffset;
}
