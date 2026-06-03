#ifndef RAYTRACER_WINDOW_H
#define RAYTRACER_WINDOW_H

#include "event.h"
#include <string>
#include <vector>
#include <functional>

struct GLFWwindow;

struct InputState {
    double mouseX, mouseY;
    double mouseDeltaX, mouseDeltaY;
    double scrollDelta;
    bool mouseLeftDown;
    bool mouseRightDown;
    bool keyW, keyA, keyS, keyD, keyQ, keyE;
    bool keyShift;
    bool keyUp, keyDown;

    InputState()
        : mouseX(0), mouseY(0), mouseDeltaX(0), mouseDeltaY(0),
          scrollDelta(0), mouseLeftDown(false), mouseRightDown(false),
          keyW(false), keyA(false), keyS(false), keyD(false),
          keyQ(false), keyE(false), keyShift(false),
          keyUp(false), keyDown(false) {}
};

class Window {
public:
    Window();
    ~Window();

    bool initialize(int width, int height, const std::string& title);
    void shutdown();
    bool shouldClose() const;
    void pollEvents();
    void getSize(int& width, int& height) const;
    void getFramebufferSize(int& width, int& height) const;

    // Opaque native OS window pointer (NSWindow* on macOS, HWND on Windows...)
    // for the renderer to bind its surface to. The only seam through which a
    // platform handle crosses into the renderer; keeps GLFW out of the backend.
    void* nativeWindowHandle() const;

    const InputState& getInput() const { return input; }
    const std::vector<Event>& getEvents() const { return events; }
    double getDeltaTime() const { return deltaTime; }

    // Invoked to redraw a frame. The platform calls this both from the normal
    // loop and during a modal resize (when pollEvents blocks), so the window
    // keeps painting instead of freezing while the user drags its edge.
    void setDrawCallback(std::function<void()> callback);

private:
    GLFWwindow* window;
    InputState input;
    std::vector<Event> events;
    std::function<void()> drawCallback;
    double lastMouseX, lastMouseY;
    double lastFrameTime;
    double deltaTime;
    bool firstMouse;

    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void windowSizeCallback(GLFWwindow* window, int width, int height);
    static void windowFocusCallback(GLFWwindow* window, int focused);
    static void windowIconifyCallback(GLFWwindow* window, int iconified);
    static void windowCloseCallback(GLFWwindow* window);
    static void windowRefreshCallback(GLFWwindow* window);
};

#endif
