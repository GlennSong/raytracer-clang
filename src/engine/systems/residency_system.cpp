#include "residency_system.h"
#include "../components.h"
#include "../residency.h"
#include "../../log.h"

#include <cstdlib>

namespace engine {

namespace {
// The per-frame loading budget: a building cell is a few milliseconds to read, drape and queue
// for upload, and a frame at 60 Hz is 16. The nearest wanted item loads regardless.
constexpr double kResidencyBudgetMs = 4.0;
}  // namespace

void ResidencySystem::render(FrameContext& ctx) {
    ctx.world.each<ResidencyService>([&](Entity, ResidencyService& rs) {
        if (!rs.service) return;
        rs.service->update(ctx.view.camera.position, kResidencyBudgetMs, &ctx.jobs);
        static const bool log = [] { const char* e = std::getenv("RT_RESIDENCY_LOG"); return e && e[0] == '1'; }();
        logClock_ += ctx.frameDelta;
        if (log && logClock_ > 5.0) {
            logClock_ = 0.0;
            for (const auto& [client, s] : rs.service->stats())
                LOG_INFO << "[residency] " << client << ": " << s.resident << " of " << s.items << " resident, "
                         << s.bytes / 1048576 << " MB, " << s.waiting << " waiting, " << s.loads << " loads, "
                         << s.unloads << " unloads; " << s.commits << " prepared off-thread, committed in "
                         << (s.commits ? s.commitMs / s.commits : 0.0) << " ms (slowest " << s.maxCommitMs
                         << "); " << (s.loads - s.commits) << " loaded in place (jumps), slowest " << s.maxLoadMs << " ms";
        }
    });
}

}  // namespace engine
