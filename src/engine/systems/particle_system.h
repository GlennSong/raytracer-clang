#ifndef RAYTRACER_ENGINE_PARTICLE_SYSTEM_H
#define RAYTRACER_ENGINE_PARTICLE_SYSTEM_H

#include "../system.h"
#include "../particles.h"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace engine {

// THE PARTICLES, spent and drawn (engine/particles.h has the effects, the simulation and how to ask for them). Each
// frame: the queued requests (requestParticles) become bursts, every ParticleEmitter emits its rate, the pool steps,
// and what is alive is drawn. It decides nothing about WHEN: the systems where things happen ask (a board's kick in
// PlayerSystem, a sliding tyre in VehicleSystem, a level's placed "emitters").
//
// DRAWN as camera-facing quads with a soft texture whose alpha shapes them (RenderMaterial::FLAG_ALPHA_FROM_MAP, the
// beacon sprites' way). An instance group has one opacity, so each effect draws in kOpacitySteps groups (made the
// first time the effect shows), every particle in the step nearest its fade; within a group they go back to front.
// An effect with a stretch is drawn as a line along its speed across the view, relative to its `ref`.
class ParticleSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    static constexpr int kOpacitySteps = 4;
    static constexpr float kOpacity[kOpacitySteps] = {0.18f, 0.38f, 0.62f, 0.9f};

    const ParticlePool& pool() const { return pool_; }

private:
    void spend(FrameContext& ctx, Real dt);
    void draw(FrameContext& ctx);
    std::array<Entity, kOpacitySteps>& groupsFor(FrameContext& ctx, int effect);

    ParticlePool pool_;
    MeshHandle quad_;
    bool haveQuad_ = false;
    std::array<TextureHandle, 3> tex_{};   // by ParticleShape
    std::vector<std::array<Entity, kOpacitySteps>> groups_;   // by effect; invalid until it first shows
    std::vector<std::pair<Real, std::size_t>> order_;           // (distance², particle) scratch
    std::vector<std::string> warned_;                           // unknown effect names already logged
};

}  // namespace engine

#endif
