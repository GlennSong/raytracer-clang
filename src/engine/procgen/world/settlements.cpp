#include "settlements.h"

#include <set>
#include <sstream>

namespace engine {

const SettlementKind* SettlementCatalog::find(const std::string& name) const {
    for (const SettlementKind& k : kinds)
        if (k.name == name) return &k;
    return nullptr;
}

const SettlementKind& SettlementCatalog::of(const std::string& name) const {
    const SettlementKind* k = find(name);
    return k ? *k : kinds.front();
}

const SettlementKind* SettlementCatalog::forHub(const std::string& hub) const {
    for (const SettlementKind& k : kinds)
        if (k.hub == hub) return &k;
    return nullptr;
}

std::string SettlementCatalog::check() const {
    if (kinds.empty()) return "no kinds";
    std::set<std::string> seen;
    for (const SettlementKind& k : kinds) {
        if (k.name.empty()) return "a kind with no name";
        if (!seen.insert(k.name).second) return "kind `" + k.name + "` twice";
        if (k.radiusScale <= 0 || k.radiusMin <= 0 || k.radiusMax < k.radiusMin) return k.name + ": radius = { scale > 0, 0 < min <= max }";
        if (k.areaScale <= 0 || k.sizeMin <= 0 || k.spokeSpacing <= 0 || k.ringSpacing <= 0 || k.spokes < 0)
            return k.name + ": the brief's sizes must be positive";
        if (k.coreBlock[0] <= 0 || k.coreBlock[1] <= 0 || k.midtownBlock[0] <= 0 || k.midtownBlock[1] <= 0)
            return k.name + ": blocks must be positive";
        if (k.naming != "city" && k.naming != "town" && k.naming != "mountain") return k.name + ": naming is city, town or mountain";
        if (k.strip && (k.stripLength < 200.0 || k.stripRows < 1 || k.stripRow < 40.0 || k.stripCross < 40.0))
            return k.name + ".strip: length >= 200, rows >= 1, row and cross >= 40 (m)";
    }
    return {};
}

std::string SettlementCatalog::describe() const {
    std::ostringstream o;
    for (const SettlementKind& k : kinds) {
        o << "kind " << k.name << " hub=" << k.hub << " radius=" << k.radiusScale << "/" << k.radiusMin << "/" << k.radiusMax
          << " coast=" << k.coastReach << " ring=" << k.onRing << " pass=" << k.passEnd << " mountain=" << k.mountainRoad
          << " interchanges=" << k.interchanges << " links=" << k.linkCandidates << "/" << k.linkSpacing << " buses=" << k.busRoutes << "/"
          << k.busStops << " signed=" << k.signedFar << " limit=" << k.limitSign << " naming=" << k.naming << " brief=" << k.briefPrefix
          << " " << k.areaScale << "/" << k.sizeMin << "/" << k.spokeSpacing << " core=" << k.coreBlock[0] << "x" << k.coreBlock[1]
          << " midtown=" << k.midtownBlock[0] << "x" << k.midtownBlock[1] << " warp=" << k.warp << " rings=" << k.ringSpacing
          << " spokes=" << k.spokes << " strip=" << k.strip << "/" << k.stripLength << "/" << k.stripRows << "x" << k.stripRow << "/"
          << k.stripCross << "/" << k.stripSea << "/" << k.stripPromenade << " reach=" << k.hubReach << " mixes=";
        for (const auto& [d, es] : k.mixes) {
            o << d << "{";
            for (const auto& [r, wt] : es) o << r << ":" << wt << ",";
            o << "}";
        }
        o << " map=" << k.rgb[0] << "," << k.rgb[1] << "," << k.rgb[2] << "/" << k.dot << "/" << k.label << "\n";
    }
    return o.str();
}

SettlementCatalog defaultSettlements() {
    SettlementCatalog c;
    SettlementKind city;
    city.name = "city";
    city.hub = "financial";
    city.radiusScale = 1.6; city.radiusMin = 600.0; city.radiusMax = 2000.0;
    city.coastReach = 2500.0;
    city.passEnd = true;
    city.interchanges = 3;
    city.linkCandidates = 6; city.linkSpacing = 700.0;
    city.busRoutes = 4; city.busStops = 14;
    city.signedFar = true;
    city.limitSign = "CITY LIMIT";
    city.naming = "city";
    // a CITY's downtown blocks are big enough for towers with room round them (Glenn: "Saltwood downtown blocks
    // should be bigger"; 94 x 64 left a 72 x 42 m site; 128 x 96 leaves ~105 x 75)
    city.briefPrefix = "city_";
    city.areaScale = 1.15; city.sizeMin = 1400.0; city.spokeSpacing = 480.0;
    city.coreBlock[0] = 128.0; city.coreBlock[1] = 96.0;
    city.midtownBlock[0] = 165.0; city.midtownBlock[1] = 118.0;
    city.warp = 18.0; city.ringSpacing = 140.0; city.spokes = 10;
    city.rgb[0] = 0.85f; city.rgb[1] = 0.12f; city.rgb[2] = 0.12f;
    city.dot = 7.0; city.label = 260.0;
    c.kinds.push_back(city);

    SettlementKind town;   // the struct's defaults: a town's small walkable grid
    town.name = "town";
    c.kinds.push_back(town);

    SettlementKind mt = town;
    mt.name = "mountain town";
    mt.radiusScale = 1.4; mt.radiusMin = 150.0; mt.radiusMax = 450.0;
    mt.onRing = false;
    mt.mountainRoad = true;
    mt.linkCandidates = 0;   // its road is the mountain road
    mt.busRoutes = 0;
    mt.naming = "mountain";
    mt.briefPrefix = "mountain_town_";
    mt.rgb[0] = 0.62f; mt.rgb[1] = 0.25f; mt.rgb[2] = 0.75f;
    c.kinds.push_back(mt);

    // THE BEACH TOWN (Glenn, 2026-11-01: "A beach town with shops and nearby homes" / the coast road "where the beach
    // towns grow out of"): a strip along its stretch of the coast road, shops on the seafront, homes behind; off the
    // ring, reached by the coast road, its own bus loop
    SettlementKind bt = town;
    bt.name = "beach town";
    bt.radiusScale = 1.0; bt.radiusMin = 300.0; bt.radiusMax = 600.0;
    bt.onRing = false;
    bt.linkCandidates = 0;   // the coast road is its road
    bt.busRoutes = 1; bt.busStops = 6;
    bt.briefPrefix = "beach_town_";
    bt.strip = true;
    bt.stripPromenade = 55.0;
    bt.hubReach = 60.0;   // the seafront's row of blocks
    // shops, cafes, flats over shops and a small hotel on the seafront; cottages, villas and garden flats behind
    bt.mixes["commercial"] = {{"mixed_use", 28}, {"brick_shop", 18}, {"oldtown_cafe", 12}, {"corner_shop", 8}, {"hotel", 10},
                              {"terrace_condo", 8}, {"walkup_homes", 6}, {"supermarket", 4}, {"plaza", 3}, {"pocket_park", 3}};
    bt.mixes["residential"] = {{"cottage", 14}, {"villa", 12}, {"modern_house", 12}, {"bungalow", 10}, {"duplex", 8},
                               {"yard_house", 8}, {"garden_condo", 12}, {"walkup_homes", 8}, {"terrace_condo", 6},
                               {"rowhouses", 6}, {"pocket_park", 4}};
    bt.rgb[0] = 0.10f; bt.rgb[1] = 0.62f; bt.rgb[2] = 0.68f;
    c.kinds.push_back(bt);
    return c;
}

}  // namespace engine
