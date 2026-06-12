#include "test_framework.h"

#include "../src/renderer/hosted_window.h"

using namespace engine;  // namespace migration (ADR-0015)

// The hosted-window seam (editor-app plan A1): a host application injects its
// toolkit's events and the engine sees the same InputState/Event semantics
// the GLFW window produces. Headless-testable by construction.

TEST_CASE(hosted_window_publishes_injected_events_once) {
    HostedWindow win;
    CHECK(win.initialize(800, 600, "test"));

    win.injectKey(KeyCode::W, true);
    win.injectMouseButton(MouseButton::Left, true, 10, 20);

    win.pollEvents();
    CHECK(win.getEvents().size() == 2);
    CHECK(win.getEvents()[0].type == EventType::KeyPressed);
    CHECK(win.getEvents()[0].key == KeyCode::W);
    CHECK(win.getEvents()[1].type == EventType::MouseButtonPressed);
    CHECK_APPROX(win.getEvents()[1].x, 10.0, 1e-9);

    win.pollEvents();   // next frame: queue drained, nothing re-delivered
    CHECK(win.getEvents().empty());
}

TEST_CASE(hosted_window_mouse_deltas_per_poll) {
    HostedWindow win;
    win.initialize(800, 600, "test");

    win.injectMouseMove(100, 100);
    win.pollEvents();
    // First sample establishes position without a spike.
    CHECK_APPROX(win.getInput().mouseDeltaX, 0.0, 1e-9);

    win.injectMouseMove(110, 95);
    win.pollEvents();
    CHECK_APPROX(win.getInput().mouseDeltaX, 10.0, 1e-9);
    CHECK_APPROX(win.getInput().mouseDeltaY, -5.0, 1e-9);
    CHECK_APPROX(win.getInput().mouseX, 110.0, 1e-9);

    // No movement -> zero delta (not a stale repeat).
    win.pollEvents();
    CHECK_APPROX(win.getInput().mouseDeltaX, 0.0, 1e-9);

    // resetMouseDelta suppresses the next sample (mode switches).
    win.resetMouseDelta();
    win.injectMouseMove(300, 300);
    win.pollEvents();
    CHECK_APPROX(win.getInput().mouseDeltaX, 0.0, 1e-9);
}

TEST_CASE(hosted_window_scroll_accumulates_then_clears) {
    HostedWindow win;
    win.initialize(800, 600, "test");

    win.injectScroll(1.0);
    win.injectScroll(0.5);
    win.pollEvents();
    CHECK_APPROX(win.getInput().scrollDelta, 1.5, 1e-9);

    win.pollEvents();
    CHECK_APPROX(win.getInput().scrollDelta, 0.0, 1e-9);
}

TEST_CASE(hosted_window_state_and_geometry) {
    HostedWindow win;
    win.initialize(800, 600, "test");

    // Raw key snapshot mirrors the platform window's.
    win.injectKey(KeyCode::LeftShift, true);
    CHECK(win.getInput().keyShift);
    win.injectKey(KeyCode::LeftShift, false);
    CHECK(!win.getInput().keyShift);

    // Retina-style geometry: logical window vs pixel framebuffer + resize event.
    win.setSizes(800, 600, 1600, 1200);
    int w = 0, h = 0;
    win.getSize(w, h);
    CHECK(w == 800);
    win.getFramebufferSize(w, h);
    CHECK(w == 1600);
    win.pollEvents();
    bool sawResize = false;
    for (const Event& e : win.getEvents())
        sawResize |= (e.type == EventType::FramebufferResized && e.width == 1600);
    CHECK(sawResize);

    CHECK(!win.shouldClose());
    win.requestClose();
    CHECK(win.shouldClose());
}
