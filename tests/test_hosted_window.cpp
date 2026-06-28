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

TEST_CASE(hosted_window_relative_mouse_mode_for_capture) {
    HostedWindow win;
    win.initialize(800, 600, "test");

    // The host learns about engine cursor-mode changes through its callback
    // (it hides/warps the toolkit cursor in response).
    CursorMode seen = CursorMode::Normal;
    win.setCursorModeCallback([&](CursorMode mode) { seen = mode; });

    // Establish an absolute position first (editor mode).
    win.injectMouseMove(400, 300);
    win.pollEvents();

    win.setCursorMode(CursorMode::Disabled);   // play: first-person capture
    CHECK(seen == CursorMode::Disabled);
    CHECK(win.relativeMouseMode());

    // While captured, motion arrives as deltas and accumulates per poll;
    // absolute positions carry no motion (the host pins its cursor).
    win.injectMouseDelta(4.0, -2.0);
    win.injectMouseDelta(1.0, 0.5);
    win.pollEvents();
    CHECK_APPROX(win.getInput().mouseDeltaX, 5.0, 1e-9);
    CHECK_APPROX(win.getInput().mouseDeltaY, -1.5, 1e-9);

    // Deltas are per-frame, not sticky.
    win.pollEvents();
    CHECK_APPROX(win.getInput().mouseDeltaX, 0.0, 1e-9);

    // Release (back to the editor): no spike from the absolute position the
    // cursor was parked at, then absolute deltas resume.
    win.setCursorMode(CursorMode::Normal);
    CHECK(seen == CursorMode::Normal);
    CHECK(!win.relativeMouseMode());
    win.injectMouseMove(100, 100);
    win.pollEvents();
    CHECK_APPROX(win.getInput().mouseDeltaX, 0.0, 1e-9);
    win.injectMouseMove(105, 104);
    win.pollEvents();
    CHECK_APPROX(win.getInput().mouseDeltaX, 5.0, 1e-9);
    CHECK_APPROX(win.getInput().mouseDeltaY, 4.0, 1e-9);
}

TEST_CASE(hosted_window_publishes_injected_gamepads) {
    // The Qt editor has no GLFW; it polls the GCController backend and pushes
    // the snapshot via setGamepads. getGamepads() must reflect it so the engine
    // (player input, fly camera) sees the sticks in the editor.
    HostedWindow win;
    win.initialize(800, 600, "test");
    CHECK(!win.getGamepads()[0].connected);   // none by default in hosted mode

    GamepadSet pads;
    pads[0].connected = true;
    pads[0].axes[static_cast<size_t>(GamepadAxis::LeftX)] = 0.5f;
    win.setGamepads(pads);

    CHECK(win.getGamepads()[0].connected);
    CHECK_APPROX(win.getGamepads()[0].axes[static_cast<size_t>(GamepadAxis::LeftX)],
                 0.5, 1e-6);
}

TEST_CASE(hosted_window_vulkan_surface_provider) {
    // The Vulkan surface seam (ADR-0057): the host owns the toolkit, so it
    // supplies surface creation + the required instance extensions. Unset, the
    // hosted window reports no Vulkan support (matching the base Window), which
    // keeps the NullRenderer/Metal paths and headless tests unaffected.
    HostedWindow win;
    win.initialize(800, 600, "test");

    uint64_t surface = 0;
    CHECK(win.requiredVulkanInstanceExtensions().empty());
    CHECK(!win.createVulkanSurface(reinterpret_cast<void*>(0x1), &surface));

    // Once the host installs a provider, both seam methods delegate to it.
    void* seenInstance = nullptr;
    win.setVulkanSurfaceProvider(
        []() -> std::vector<std::string> {
            return {"VK_KHR_surface", "VK_KHR_test_surface"};
        },
        [&seenInstance](void* instance, uint64_t* out) {
            seenInstance = instance;
            *out = 0xABCD;
            return true;
        });

    auto exts = win.requiredVulkanInstanceExtensions();
    CHECK(exts.size() == 2);
    CHECK(exts[0] == "VK_KHR_surface");

    CHECK(win.createVulkanSurface(reinterpret_cast<void*>(0x42), &surface));
    CHECK(surface == 0xABCD);
    CHECK(seenInstance == reinterpret_cast<void*>(0x42));
}
