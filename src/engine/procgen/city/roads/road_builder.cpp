#include "road_builder.h"

#include "../../../../log.h"

#include <mutex>
#include <set>

namespace engine::roads {

namespace {

std::vector<std::unique_ptr<RoadBuilder>>& registry() {
    static std::vector<std::unique_ptr<RoadBuilder>> r;
    return r;
}

}  // namespace

// Defined by each builder's own .cpp. Called explicitly (not by a file-static
// registrar) because engine_core is a STATIC library: a translation unit nothing
// references is dropped at link time, and a self-registering builder would
// simply vanish from a linked binary.
void registerLatticeRoadBuilder();
#ifdef RT_ROADS_LANES
void registerLanesRoadBuilder();
#endif

namespace {

void ensureBuiltins() {
    static std::once_flag once;
    std::call_once(once, [] {
        registerLatticeRoadBuilder();
#ifdef RT_ROADS_LANES
        registerLanesRoadBuilder();
#endif
    });
}

}  // namespace

void registerRoadBuilder(std::unique_ptr<RoadBuilder> builder) {
    if (!builder) return;
    for (const std::unique_ptr<RoadBuilder>& b : registry())
        if (std::string_view(b->name()) == builder->name()) return;   // first wins
    registry().push_back(std::move(builder));
}

RoadBuilder* roadBuilder(std::string_view name) {
    ensureBuiltins();
    for (const std::unique_ptr<RoadBuilder>& b : registry())
        if (std::string_view(b->name()) == name) return b.get();
    static std::set<std::string> warned;
    if (warned.insert(std::string(name)).second) {
        std::string have;
        for (const std::string& n : roadBuilderNames()) have += (have.empty() ? "" : ", ") + n;
        LOG_WARN << "[roads] no road builder named \"" << name << "\" in this build (have: "
                 << have << ")";
    }
    return nullptr;
}

std::vector<std::string> roadBuilderNames() {
    ensureBuiltins();
    std::vector<std::string> out;
    for (const std::unique_ptr<RoadBuilder>& b : registry()) out.emplace_back(b->name());
    return out;
}

std::string roadBuilderName(const nlohmann::json& roadBlock) {
    if (roadBlock.is_object() && roadBlock.contains("builder") && roadBlock["builder"].is_string())
        return roadBlock["builder"].get<std::string>();
    return kDefaultRoadBuilder;
}

RoadBuilder& roadBuilderFor(const nlohmann::json& roadBlock) {
    const std::string want = roadBuilderName(roadBlock);
    if (RoadBuilder* b = roadBuilder(want)) return *b;
    RoadBuilder* fallback = roadBuilder(kDefaultRoadBuilder);
    // The default builder is compiled in unconditionally; if it is missing the
    // build is broken in a way no level can work around.
    return *fallback;
}

}  // namespace engine::roads
