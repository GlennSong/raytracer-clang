#ifndef RAYTRACER_ENGINE_SYSTEMS_PREFAB_USE_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_PREFAB_USE_SYSTEM_H

// STAMPED PREFABS as an interaction PROVIDER (#191, step 9; engine/prefab_use.h, interact_broker.h). Each frame, on
// foot, every use of a prefab copy whose marker is within reach is OFFERED to the broker, with the word it would
// do now ("open", or "close" once open). On the broker's command the use is done, or, for a seat, the player sits
// (the furniture's Seated, naming the prefab copy as its set). While seated on a prefab the only offer is "stand
// up", any move key stands them up too, and the eye is carried with the seat if its node moves (a rocking chair).
//
// The furniture kit's own pieces are still InteractionSystem's; this is the same broker and the same Seated, for
// things that are prefabs. When everything placed is a prefab the two become one (docs/model-record-design.md,
// "One catalog").

#include "../audio/audio_engine.h"
#include "../system.h"

#include <map>
#include <string>
#include <utility>

namespace engine {

class PrefabUseSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;   // the lamps that are on, to the light board (lights/light_board.h)

    static constexpr double REACH = 1.8;

private:
    // WHAT A USE SOUNDS LIKE (assets/scripts/prefab_sounds.lua): a clip for each thing that can happen to it. A
    // clip is one of the engine's procedural sounds by name (audio/sfx.h), made once.
    struct Sound {
        std::string done, undone, arrived, shut, moving, running;
        float volume = 1.0f, pitch = 1.0f;
        Real range = 12.0;
    };
    void sounds(FrameContext& ctx);
    AudioClipHandle clip(FrameContext& ctx, const std::string& name);
    std::map<std::string, Sound> sounds_;
    std::map<std::string, AudioClipHandle> clips_;
    std::map<std::pair<uint64_t, std::size_t>, AudioVoiceHandle> loops_;   // (copy, use) -> its looping voice
    bool soundsRead_ = false;
};

}  // namespace engine

#endif
