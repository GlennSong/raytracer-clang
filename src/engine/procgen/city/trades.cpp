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
}  // namespace

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
