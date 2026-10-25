#ifndef RAYTRACER_ENGINE_PLAYER_TOOLS_H
#define RAYTRACER_ENGINE_PLAYER_TOOLS_H

#include <cstdint>
#include <string>

namespace engine {

// THE TOOLS IN HAND, one per slot (the number keys, LB / RB on a pad: ToolCycleSystem). The light is not one of
// them: it is a switch of its own that goes with any of them (FlashlightSystem, ADR-0249).
enum class Tool : uint8_t { Hands = 1, Gun, Map, Parachute, Wingsuit, Grapple, Skateboard, Scooter };
constexpr int kToolSlots = 8;

inline int slotOf(Tool t) { return static_cast<int>(t); }
inline std::string slotAction(int slot) { return "slot_" + std::to_string(slot); }
inline std::string slotAction(Tool t) { return slotAction(slotOf(t)); }
inline const char* toolName(Tool t) {
    switch (t) {
        case Tool::Hands: return "hands";
        case Tool::Gun: return "gun";
        case Tool::Map: return "map";
        case Tool::Parachute: return "parachute";
        case Tool::Wingsuit: return "wingsuit";
        case Tool::Grapple: return "grapple";
        case Tool::Skateboard: return "skateboard";
        case Tool::Scooter: return "scooter";
    }
    return "?";
}

}  // namespace engine

#endif
