// GCController-based gamepad polling for macOS.
//
// On macOS 13+ Apple's DriverKit driver claims Xbox/PS controllers at the USB
// level and re-presents them with a vendor-specific HID descriptor that
// IOKit-based libraries (GLFW 3.4) cannot parse. GCController speaks the
// vendor protocol and is the only reliable path for modern controllers.
//
// GCController delivers input updates asynchronously through the Cocoa run
// loop. GLFW's glfwPollEvents() does not fully service the run loop for
// GCController's dispatch, so we use a callback-based approach: a value-changed
// handler caches the latest input snapshot, and gcPollGamepads() copies that
// cached state into the engine's GamepadSet each frame.

#include "gamepad_gc.h"
#include "../log.h"

#import <GameController/GameController.h>
#import <Foundation/Foundation.h>

// Per-controller cached state, updated from GCController callbacks.
struct GCCachedPad {
    GamepadState state;
    bool active = false;
};

static GCCachedPad gcCache[MAX_GAMEPADS];
static bool gcInitialized = false;

// Read the full state from the extended gamepad profile into a GamepadState.
static void snapshotExtendedGamepad(GCExtendedGamepad* gp, GamepadState& out) {
    out.connected = true;

    out.buttons[static_cast<std::size_t>(GamepadButton::A)] = gp.buttonA.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::B)] = gp.buttonB.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::X)] = gp.buttonX.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::Y)] = gp.buttonY.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::LeftBumper)] = gp.leftShoulder.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::RightBumper)] = gp.rightShoulder.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::LeftThumb)] =
        gp.leftThumbstickButton ? gp.leftThumbstickButton.pressed : false;
    out.buttons[static_cast<std::size_t>(GamepadButton::RightThumb)] =
        gp.rightThumbstickButton ? gp.rightThumbstickButton.pressed : false;
    out.buttons[static_cast<std::size_t>(GamepadButton::DpadUp)] = gp.dpad.up.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::DpadDown)] = gp.dpad.down.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::DpadLeft)] = gp.dpad.left.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::DpadRight)] = gp.dpad.right.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::Start)] = gp.buttonMenu.pressed;
    out.buttons[static_cast<std::size_t>(GamepadButton::Back)] =
        gp.buttonOptions ? gp.buttonOptions.pressed : false;
    out.buttons[static_cast<std::size_t>(GamepadButton::Guide)] =
        gp.buttonHome ? gp.buttonHome.pressed : false;

    // Sticks — negate Y to match GLFW convention (stick-up = negative).
    out.axes[static_cast<std::size_t>(GamepadAxis::LeftX)]  =  gp.leftThumbstick.xAxis.value;
    out.axes[static_cast<std::size_t>(GamepadAxis::LeftY)]  = -gp.leftThumbstick.yAxis.value;
    out.axes[static_cast<std::size_t>(GamepadAxis::RightX)] =  gp.rightThumbstick.xAxis.value;
    out.axes[static_cast<std::size_t>(GamepadAxis::RightY)] = -gp.rightThumbstick.yAxis.value;

    // Triggers — GCController returns [0, 1] which matches our convention.
    out.axes[static_cast<std::size_t>(GamepadAxis::LeftTrigger)]  = gp.leftTrigger.value;
    out.axes[static_cast<std::size_t>(GamepadAxis::RightTrigger)] = gp.rightTrigger.value;
}

// Assign a cache slot for a controller. Returns the slot index, or -1 if full.
static int assignSlot(GCController* controller) {
    // Check if already assigned (by pointer tag via playerIndex).
    if (controller.playerIndex >= 0 &&
        controller.playerIndex < (GCControllerPlayerIndex)(-1) &&
        (int)controller.playerIndex < MAX_GAMEPADS) {
        return (int)controller.playerIndex;
    }
    // Find first free slot.
    for (int i = 0; i < MAX_GAMEPADS; i++) {
        if (!gcCache[i].active) {
            controller.playerIndex = (GCControllerPlayerIndex)i;
            return i;
        }
    }
    return -1;
}

static void setupController(GCController* controller) {
    int slot = assignSlot(controller);
    if (slot < 0) return;

    gcCache[slot].active = true;
    gcCache[slot].state = GamepadState{};
    gcCache[slot].state.connected = true;

    GCExtendedGamepad* gp = [controller extendedGamepad];
    if (!gp) return;

    // Take an initial snapshot.
    snapshotExtendedGamepad(gp, gcCache[slot].state);

    // Install a callback that fires on any input change. GCController calls
    // this on its internal dispatch queue; we write to the cache which
    // gcPollGamepads reads from the main thread. GamepadState fields are
    // small and the main thread only reads between frames, so a torn read
    // is harmless (one frame of slightly stale data).
    gp.valueChangedHandler = ^(GCExtendedGamepad* gamepad,
                               GCControllerElement* __unused element) {
        if (slot >= 0 && slot < MAX_GAMEPADS) {
            snapshotExtendedGamepad(gamepad, gcCache[slot].state);
        }
    };

    LOG_INFO << "GCController: set up slot " << slot
             << " (\"" << (controller.vendorName
                           ? [controller.vendorName UTF8String] : "?") << "\")";
}

static void teardownController(GCController* controller) {
    int slot = (int)controller.playerIndex;
    if (slot >= 0 && slot < MAX_GAMEPADS) {
        gcCache[slot] = GCCachedPad{};
        LOG_INFO << "GCController: released slot " << slot;
    }
    controller.playerIndex = (GCControllerPlayerIndex)(-1);
}

static void gcInit() {
    if (gcInitialized) return;
    gcInitialized = true;

    // Register for connect/disconnect notifications.
    [[NSNotificationCenter defaultCenter]
        addObserverForName:GCControllerDidConnectNotification
                    object:nil
                     queue:[NSOperationQueue mainQueue]
                usingBlock:^(NSNotification* note) {
        GCController* c = note.object;
        setupController(c);
    }];

    [[NSNotificationCenter defaultCenter]
        addObserverForName:GCControllerDidDisconnectNotification
                    object:nil
                     queue:[NSOperationQueue mainQueue]
                usingBlock:^(NSNotification* note) {
        GCController* c = note.object;
        teardownController(c);
    }];

    // Pump the run loop so any pending connect notifications for controllers
    // already plugged in get delivered before we enumerate.
    @autoreleasepool {
        [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                 beforeDate:[NSDate distantPast]];
    }

    // Set up any controllers already connected at launch.
    for (GCController* c in [GCController controllers]) {
        setupController(c);
    }

    LOG_INFO << "GCController: initialized, "
             << [[GCController controllers] count] << " controller(s) present";
}

void gcPollGamepads(GamepadSet& pads) {
    gcInit();

    // Service the run loop briefly so GCController's internal dispatch
    // delivers any pending input events and connect/disconnect notifications.
    // Without this, GLFW's glfwPollEvents() alone is not enough.
    @autoreleasepool {
        [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                 beforeDate:[NSDate distantPast]];
    }

    for (int i = 0; i < MAX_GAMEPADS; i++) {
        if (!gcCache[i].active) continue;
        pads[i] = gcCache[i].state;
    }
}
