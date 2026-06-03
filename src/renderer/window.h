#ifndef RAYTRACER_WINDOW_H
#define RAYTRACER_WINDOW_H

#include <string>
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
    bool keyUp, keyDown, keyEscape;
    bool keySpace, keyComma, keyPeriod, keyNum0;

    InputState()
        : mouseX(0), mouseY(0), mouseDeltaX(0), mouseDeltaY(0),
          scrollDelta(0), mouseLeftDown(false), mouseRightDown(false),
          keyW(false), keyA(false), keyS(false), keyD(false),
          keyQ(false), keyE(false), keyShift(false),
          keyUp(false), keyDown(false), keyEscape(false),
          keySpace(false), keyComma(false), keyPeriod(false), keyNum0(false) {}
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
    GLFWwindow* getHandle() const { return window; }
    const InputState& getInput() const { return input; }
    double getDeltaTime() const { return deltaTime; }

private:
    GLFWwindow* window;
    InputState input;
    double lastMouseX, lastMouseY;
    double lastFrameTime;
    double deltaTime;
    bool firstMouse;

    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
};

#endif
