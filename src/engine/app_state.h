#ifndef RAYTRACER_ENGINE_APP_STATE_H
#define RAYTRACER_ENGINE_APP_STATE_H

#include "system.h"

namespace engine {

class AppState {
public:
    virtual ~AppState() = default;

    virtual bool isTransparent() const { return false; }
    virtual bool blocksInput() const { return true; }

    virtual void onEnter(FrameContext&) {}
    virtual void onExit(FrameContext&) {}
    virtual void onSuspend(FrameContext&) {}
    virtual void onResume(FrameContext&) {}

    virtual void onEvent(const Event&, FrameContext&) {}
    virtual void update(FrameContext&) {}
    virtual void fixedUpdate(FrameContext&) {}
    virtual void render(FrameContext&) {}
};

}  // namespace engine

#endif
