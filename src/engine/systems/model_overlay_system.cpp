#include "model_overlay_system.h"

#include "../model_inspect.h"
#include "../model_overlay.h"

namespace engine {

void ModelOverlaySystem::render(FrameContext& ctx) {
    const ModelOverlay& what = modelOverlay();
    if (!what.any()) return;
    for (const Entity instance : modelInstances(ctx.world)) drawModelOverlay(ctx.world, instance, what, ctx.debug);
}

}  // namespace engine
