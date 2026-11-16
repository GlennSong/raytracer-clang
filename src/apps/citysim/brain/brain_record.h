#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_RECORD_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_RECORD_H

// ONE PERSON'S BRAIN, AS STORED (docs/agent-brain-design.md sections 2 to 5; ADR-0440): what they want (intent), how
// they will do it (plan), what they are doing this moment (doing), and what they have. Plain data of a fixed size: no
// heap, no strings, no virtual functions, so hundreds of thousands fit in one array, are copied byte for byte and are
// hashed byte for byte.
//
// Words: a LEG is one part of a trip made one way (walk to the car; drive; walk to the door). A WAY is a row of
// travel.lua. A STATE is a row of brain.lua. A PLACE is a handle the world hands out and reads back (brain_skills.h
// World): a door, a kerb parking space. The brain never looks inside one.

#include <cstdint>
#include <type_traits>

namespace citysim {
namespace brain {

// TIME: whole milliseconds since the simulation began. Whole numbers so two runs agree to the bit and a far person's
// "due at" is exact. 32 bits hold a little over 49 days of simulated time (a known limit; ADR-0440).
using BrainTime = uint32_t;
constexpr BrainTime kNever = 0xFFFFFFFFu;        // no time known: a state with no planned end
constexpr BrainTime kMillisPerSecond = 1000;

using AgentId = uint32_t;                        // a person's place in the machine's array
constexpr AgentId kNoAgent = 0xFFFFFFFFu;
using StateId = uint8_t;                         // a row of brain.lua `states`
using EventId = uint8_t;                         // a row of brain.lua `events`
using IntentKind = uint8_t;                      // a row of brain.lua `intents`
using WayId = uint8_t;                           // a row of travel.lua `modes`
constexpr uint8_t kNoId = 0xFF;                  // no state, no event, no way

// A PLACE, as the world names it. The brain stores it, compares it for sameness and hands it back; nothing else.
struct PlaceRef {
    static constexpr uint32_t kNone = 0xFFFFFFFFu;
    uint32_t id = kNone;
    bool valid() const { return id != kNone; }
    bool operator==(const PlaceRef& o) const { return id == o.id; }
    bool operator!=(const PlaceRef& o) const { return id != o.id; }
};
// A VEHICLE OR A TRANSIT LINE, as the world names it. 24 bits are stored per leg (kMaxHandle).
struct CarRef {
    static constexpr uint32_t kNone = 0xFFFFFFFFu;
    uint32_t id = kNone;
    bool valid() const { return id != kNone; }
    bool operator==(const CarRef& o) const { return id == o.id; }
    bool operator!=(const CarRef& o) const { return id != o.id; }
};
constexpr uint32_t kNoHandle = 0xFFFFFFu;        // a leg that uses no vehicle and no line
constexpr uint32_t kMaxHandle = kNoHandle - 1;

// HOW CLOSELY A PERSON IS SIMULATED (design section 8). Near: skills move the body. Far: no skill runs, and a state
// with a known end is advanced by the clock alone. Middle is not built: it is treated as Near (ADR-0440).
enum class Detail : uint8_t { Far = 0, Middle = 1, Near = 2 };

constexpr int kMaxLegs = 8;                      // the same number as the planner's kMaxPlanLegs (checked in brain_planner.cpp)

// THE FEW BYTES THAT BELONG TO THE CURRENT STATE and that only it reads.
union StatePayload {
    uint32_t word;
    // In a state that HOLDS (possessed, knocked down, stuck, crashed): how many milliseconds the interrupted state had
    // still to run when it was interrupted; kNever when it had no planned end.
    uint32_t remaining;
};

struct AgentBrain {
    // ---- PLAN: up to eight legs, and no route (a route is found when its leg begins and belongs to the skill) -------
    uint32_t legEnd[kMaxLegs];       // where each leg ends (a PlaceRef's id)
    uint32_t legUse[kMaxLegs];       // low 8 bits: the way; high 24: the vehicle or line it uses (kNoHandle: none)
    uint16_t legSeconds[kMaxLegs];   // each leg's planned whole seconds, so a far person needs no route to know when
                                     // a leg ends
    // ---- INTENT: what the person wants ---------------------------------------------------------------------------
    uint32_t intentTarget;           // the place it is about (a PlaceRef's id)
    BrainTime intentBegan;
    // ---- DOING: the one thing they are doing this moment ------------------------------------------------------------
    BrainTime stateEntered;
    BrainTime stateDue;              // when it is due to end; kNever when that is not known
    StatePayload payload;
    // ---- WHERE THE PLAN BEGAN, and where the person is while not travelling: plan x and z in whole centimetres. The
    // first leg has no leg before it to say where it starts, and a far person part-way along it is placed from here.
    int32_t originX, originZ;
    // ---- HAS -------------------------------------------------------------------------------------------------------
    uint32_t car;                    // the household's car (a CarRef's id; kNone: none). Shared: whoever reaches it
                                     // first takes it
    uint16_t ruledOut;               // ways a leg of which failed while following this intent: bit i for way i
    IntentKind intentKind;
    StateId state;
    uint8_t legCount : 4;            // legs in the plan (0 to 8)
    uint8_t legNow : 4;              // the leg under way; equal to legCount when the plan is done
    uint8_t traits;                  // bit i: has travel.lua traits[i] (a board, a scooter)
    StateId heldState;               // in a state that holds: the state that was interrupted (kNoId: none)
    uint8_t detail;                  // a Detail

    WayId legWay(int leg) const { return static_cast<WayId>(legUse[leg] & 0xFFu); }
    uint32_t legHandle(int leg) const { return legUse[leg] >> 8; }
    bool travelling() const { return legNow < legCount; }
};

constexpr uint32_t packLegUse(WayId way, uint32_t handle) { return static_cast<uint32_t>(way) | (handle << 8); }

// The size is part of the design: 238,000 people at 120 bytes are 28.6 MB. Every byte is a field (the sizes above add
// up to it), so the record is hashed as bytes.
static_assert(std::is_trivially_copyable<AgentBrain>::value, "a brain record is copied and hashed as bytes");
static_assert(std::is_standard_layout<AgentBrain>::value, "a brain record is plain data");
static_assert(sizeof(AgentBrain) == 120, "the brain record's size is stated in ADR-0440; change both together");
static_assert(sizeof(AgentBrain) == 8 * 4 + 8 * 4 + 8 * 2 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 2 + 6,
              "the brain record has no padding: every byte is a field");
static_assert(alignof(AgentBrain) == 4, "the brain record packs on 4 bytes");

// ONE EVENT: a report from a skill ("arrived", "parked") or an interruption ("vehicle gone", "possessed") for one
// person. `order` is the machine's count when it was posted, so events for one person are taken in the order they came.
struct BrainEvent {
    AgentId agent = kNoAgent;
    uint32_t payload = 0;
    uint32_t order = 0;
    EventId kind = kNoId;
    uint8_t unused[3] = {0, 0, 0};
};
static_assert(sizeof(BrainEvent) == 16, "an event is 16 bytes");

}  // namespace brain
}  // namespace citysim

#endif
