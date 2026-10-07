#include "trades.h"

namespace engine {

namespace {
// The mix: a street of shops downtown is mostly somewhere to eat and drink, then the everyday trades. (A city-wide
// mix by district is the next step: plan stage 1b.)
const TradeInfo kTrades[] = {
    {0, "cafe", "cafe", 6.5f, 19.0f, 14, 2, Vec3(0.95, 0.70, 0.35), true},
    {1, "grocery", "supermarket", 7.0f, 22.0f, 8, 2, Vec3(0.35, 0.85, 0.40), false},
    {2, "boutique", "shop", 10.0f, 19.0f, 9, 2, Vec3(0.95, 0.45, 0.65), false},
    {3, "bookshop", "shop", 9.0f, 20.0f, 5, 2, Vec3(0.40, 0.65, 1.00), false},
    {4, "electronics", "shop", 10.0f, 20.0f, 5, 2, Vec3(0.30, 0.85, 1.00), false},
    {5, "pharmacy", "shop", 8.0f, 21.0f, 5, 2, Vec3(0.35, 1.00, 0.55), false},
    {6, "bakery", "cafe", 6.0f, 17.0f, 6, 2, Vec3(1.00, 0.85, 0.50), true},
    {13, "restaurant", "restaurant", 11.0f, 23.5f, 18, 2, Vec3(1.00, 0.40, 0.30), true},
    {14, "bar", "bar", 16.0f, 2.0f, 11, 2, Vec3(0.75, 0.40, 1.00), false},
    {15, "club", "club", 21.0f, 3.0f, 3, 3, Vec3(1.00, 0.25, 0.85), false},
};
constexpr int kCount = static_cast<int>(sizeof(kTrades) / sizeof(kTrades[0]));

// The rooms. Colours are albedo (the room lights do the rest): a bar is oxblood, bottle green or ink over dark
// panelling, a club black, a restaurant warm plaster over timber, a bakery cream over a black-and-white checker.
const TradeInterior kPlainShop = {{{0.88, 0.87, 0.84}, {0.84, 0.86, 0.88}, {0.90, 0.88, 0.82}}, 0.0, {}, {0.55, 0.55, 0.56},
                                  {}, 0.0, {0.92, 0.92, 0.90}, 0};
struct TradeRoom { uint8_t id; TradeInterior in; };
const TradeRoom kRooms[] = {
    {0, {{{0.80, 0.62, 0.45}, {0.55, 0.62, 0.50}, {0.86, 0.80, 0.68}}, 1.1, {0.92, 0.92, 0.88},   // cafe: white tile dado
         {0.42, 0.30, 0.20}, {}, 0.0, {0.90, 0.88, 0.84}, 1}},
    {1, {{{0.92, 0.92, 0.90}, {0.90, 0.92, 0.92}, {0.92, 0.91, 0.88}}, 0.0, {}, {0.70, 0.70, 0.68},
         {}, 0.0, {0.94, 0.94, 0.94}, 0}},
    {2, {{{0.90, 0.84, 0.82}, {0.80, 0.82, 0.84}, {0.94, 0.92, 0.88}}, 0.0, {}, {0.78, 0.74, 0.68},
         {}, 0.0, {0.95, 0.95, 0.94}, 1}},
    {3, {{{0.20, 0.32, 0.25}, {0.18, 0.22, 0.34}, {0.42, 0.18, 0.15}}, 0.9, {0.30, 0.20, 0.13},   // bookshop: library
         {0.35, 0.24, 0.15}, {}, 0.0, {0.86, 0.84, 0.78}, 1}},
    {4, {{{0.90, 0.91, 0.93}, {0.25, 0.27, 0.30}, {0.88, 0.88, 0.88}}, 0.0, {}, {0.45, 0.46, 0.48},
         {}, 0.0, {0.94, 0.94, 0.95}, 0}},
    {5, {{{0.92, 0.94, 0.93}, {0.88, 0.93, 0.92}, {0.93, 0.93, 0.93}}, 0.0, {}, {0.78, 0.84, 0.82},
         {}, 0.0, {0.95, 0.95, 0.95}, 0}},
    {6, {{{0.95, 0.88, 0.70}, {0.92, 0.80, 0.78}, {0.80, 0.86, 0.80}}, 1.2, {0.95, 0.95, 0.92},   // bakery: tile + checker
         {0.08, 0.08, 0.08}, {0.92, 0.92, 0.90}, 0.4, {0.95, 0.94, 0.90}, 1}},
    {13, {{{0.62, 0.30, 0.20}, {0.78, 0.62, 0.42}, {0.35, 0.40, 0.30}}, 1.0, {0.26, 0.16, 0.10},   // restaurant
          {0.25, 0.16, 0.10}, {}, 0.0, {0.55, 0.45, 0.36}, 1}},
    {14, {{{0.30, 0.07, 0.08}, {0.10, 0.20, 0.14}, {0.08, 0.10, 0.18}}, 1.3, {0.16, 0.09, 0.05},   // bar
          {0.13, 0.08, 0.05}, {}, 0.0, {0.07, 0.06, 0.06}, 2}},
    {15, {{{0.04, 0.04, 0.05}, {0.08, 0.03, 0.10}, {0.03, 0.04, 0.08}}, 0.0, {},                   // club
          {0.03, 0.03, 0.035}, {}, 0.0, {0.02, 0.02, 0.025}, 2}},
};
}  // namespace

const TradeInterior& tradeInterior(uint8_t id) {
    for (const TradeRoom& r : kRooms) if (r.id == id) return r.in;
    return kPlainShop;
}

int tradeCount() { return kCount; }
const TradeInfo& tradeAt(int index) { return kTrades[index < 0 ? 0 : index >= kCount ? kCount - 1 : index]; }

const TradeInfo* tradeById(uint8_t id) {
    for (const TradeInfo& t : kTrades) if (t.id == id) return &t;
    return nullptr;
}

uint8_t pickTrade(uint32_t bits, int bays) {
    int total = 0;
    for (const TradeInfo& t : kTrades) if (bays >= t.minBays) total += t.weight;
    if (total <= 0) return 0;
    int r = static_cast<int>(bits % static_cast<uint32_t>(total));
    for (const TradeInfo& t : kTrades) {
        if (bays < t.minBays) continue;
        if (r < t.weight) return t.id;
        r -= t.weight;
    }
    return 0;
}

}  // namespace engine
