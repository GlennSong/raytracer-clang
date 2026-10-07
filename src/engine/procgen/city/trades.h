#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_TRADES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_TRADES_H

// THE TRADES (~/.claude/plans/storefronts-identity.md, stage 1; Glenn: "all those places need to look unique from one
// another and like the actual places"). A ground-floor shop unit IS a business of one trade, and everything about it
// reads this one table: the facade grammar draws a unit's trade from it (how common each is, how many bays it
// needs), the interior is furnished by it (the unit room's style IS the trade id), the fascia wears its colour, the
// level loader makes the unit a citysim place of its type and hours, and the lot pass lays a terrace for the trades
// that sit out. The citysim and the street agree on what every shop is.
//
// Ids are the shop room's style: 0-6 as they always were, 13+ for the trades added since (7-12 are the big-box
// store's rooms, room_plan bigBoxRoomPlan).

#include "../../../rt_math.h"

#include <cstdint>

namespace engine {

struct TradeInfo {
    uint8_t id;
    const char* name;        // "cafe", "restaurant", ...
    const char* placeType;   // the citysim place it is: cafe restaurant bar club shop supermarket
    float openHour, closeHour;   // when customers come (wraps midnight: a bar opens 16, closes 2)
    int weight;              // how common, among the trades
    int minBays;             // the fewest bays a unit of it needs (a club wants a wide front)
    Vec3 fascia;             // the sign band's lit colour
    bool terrace;            // sets tables out on the pavement
};

int tradeCount();
const TradeInfo& tradeAt(int index);          // by table position
const TradeInfo* tradeById(uint8_t id);       // by id (a room's style); nullptr if none
// The trade for a unit of `bays` bays from 32 bits of its own hash: weighted, among the trades that fit.
uint8_t pickTrade(uint32_t bits, int bays);

}  // namespace engine

#endif
