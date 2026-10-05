#include "activities.h"

namespace citysim {

const char* spotKindName(SpotKind k) {
    switch (k) {
        case SpotKind::Sit: return "sit";
        case SpotKind::Lie: return "lie";
        case SpotKind::Stand: return "stand";
        case SpotKind::Jog: return "jog";
        case SpotKind::Play: return "play";
        case SpotKind::Watch: return "watch";
        default: return "?";
    }
}

bool spotKindFromName(const std::string& name, SpotKind& out) {
    for (int i = 0; i < static_cast<int>(SpotKind::Count); ++i)
        if (name == spotKindName(static_cast<SpotKind>(i))) { out = static_cast<SpotKind>(i); return true; }
    return false;
}

uint32_t spotTagFromName(const std::string& name) {
    if (name == "campus") return spot_tag::kCampus;
    if (name == "park") return spot_tag::kPark;
    if (name == "sports") return spot_tag::kSports;
    return 0;
}

}  // namespace citysim
