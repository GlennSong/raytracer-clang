#ifndef RAYTRACER_ENGINE_SYSTEMS_INTERACT_BROKER_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_INTERACT_BROKER_SYSTEM_H

#include "../interact_broker.h"
#include "../system.h"
#include <string>

namespace engine {

class PhysicsSystem;

// THE INTERACTION BROKER's system (engine/interact_broker.h): owns E. Registered AFTER every provider, so a frame's
// offers are all in when it ranks them. Keeps one FOCUS -- held while it stays a candidate and nothing is clearly
// better (no flicker between two chairs), moved by Tab or (first person) the mouse wheel -- draws its marker and
// prompt, and turns E into a command for the focused offer's provider: a tap, or a hold past HOLD_S when the offer
// has a hold verb.
class InteractBrokerSystem : public System {
public:
    explicit InteractBrokerSystem(PhysicsSystem* physics = nullptr) : physics_(physics) {}
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

    static constexpr double HOLD_S = 0.4;

private:
    PhysicsSystem* physics_;
    // the focus, by identity (offers are rebuilt every frame)
    bool haveFocus_ = false;
    std::string fProvider_;
    Entity fEntity_;
    uint64_t fKey_ = 0;
    bool cycled_ = false;   // chosen with Tab/the wheel: kept while it is a candidate
    bool holding_ = false, holdFired_ = false;
    double holdT_ = 0;
    // what render() draws
    bool show_ = false, marker_ = false;
    Vec3 anchor_{0, 0, 0};
    std::string prompt_, name_;
    int others_ = 0;
    double pulse_ = 0;
};

}  // namespace engine

#endif
