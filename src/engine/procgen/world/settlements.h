#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_SETTLEMENTS_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_SETTLEMENTS_H

// THE KINDS OF PLACE ON AN ISLAND (~/.claude/plans/beach-town.md step 1; AGENTS.md "Kinds Are Data"): a city, a
// town, a mountain town -- and the beach town and the villages to come -- are DATA. Each kind says how big it grows
// from its flat ground, whether the freeway ring calls at it, whether the pass or a mountain road ends at it, how
// many ways onto the freeway and local buses it gets, how its streets are briefed, how the map draws it and what its
// limit sign says. Where a kind is FOUND on new ground (the deepest plain each side, the highest good ground) is the
// planner's (island_world.cpp planIsland); what a kind IS lives here.
//
// Authored in assets/scripts/settlements.lua (scripting/settlements_lua.h reads it); defaultSettlements is the same
// catalog in C++ (a test keeps the two describing identically).

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace engine {

// A LANDMARK a kind of place is given (architect.h LandmarkKind, by its name: "school", "church", "market_hall",
// "police", ...): `count` of them once the place has `minLots` lots (and `minOldTown` of them old town), on the best
// lots of their districts -- or, `any`, the best lot of any district when none of theirs can carry one.
struct SettlementLandmark {
    std::string name;
    int count = 1, minLots = 0, minOldTown = 0;
    bool any = false;
};

struct SettlementKind {
    std::string name;                     // "city", "town", "mountain town"
    std::string hub = "commercial";       // its centre's district in the level (citysim.districts.hubs[].kind)
    // its radius from how deep its flat ground runs (m): depth x scale, clamped
    double radiusScale = 1.3, radiusMin = 250.0, radiusMax = 700.0;
    double coastReach = 1600.0;           // coastal when the sea is this near (a kept place)
    bool onRing = true;                   // the island freeway's ring calls at it
    bool passEnd = false;                 // the pass over the range joins two of these
    bool mountainRoad = false;            // a mountain road climbs to it from the freeway (its only road)
    bool highRoad = false;                // a HIGH ROAD climbs from it over the range to the city across (a second pass;
                                          // ~/.claude/plans/mountain-towns.md): a foothill town
    int interchanges = 1;                 // the diamonds it is given first (more first)
    int linkCandidates = 4;               // link roads offered to the freeway...
    double linkSpacing = 300.0;           // ...their freeway ends this far apart
    int busRoutes = 1, busStops = 8;      // its local bus network (BusNetwork::build); 0 routes: none
    bool signedFar = false;               // named on the island's distance and control-city signs
    std::string limitSign = "TOWN LIMIT";
    std::string naming = "town";          // place_names.h: "city", "town", "mountain"
    // THE BRIEF (islandSiteBrief): the area it grows to (its radius' disc x areaScale), the plan's square at least
    // sizeMin across, spokes this far apart; the core's and midtown's blocks, midtown's warp; the outskirts' rings
    // and spokes
    std::string briefPrefix = "town_";
    double areaScale = 1.0, sizeMin = 500.0, spokeSpacing = 360.0;
    double coreBlock[2] = {94.0, 64.0}, midtownBlock[2] = {158.0, 108.0}, warp = 10.0;
    double ringSpacing = 120.0;
    int spokes = 6;
    // ITS LAYOUT (a place not a strip): "grown" -- grown over its land to its area and laid out by depth (the grid, the
    // rings, the spokes); "terraces" -- a hill town, a street along the back of each shelf its regrade cut (regradeTerrace
    // apart), ramps between them (city_plan.cpp world.terraces, ADR-0326)
    std::string layout = "grown";
    // A STRIP (a beach town; ~/.claude/plans/beach-town.md): laid along the road it grew from instead of grown over its
    // land -- its spine `stripLength` of the coast road, the seafront street; `stripRows` streets behind it every
    // `stripRow` m inland; cross streets every `stripCross`; its limits `stripSea` out over the spine's sea side.
    bool strip = false;
    double stripLength = 900.0, stripRow = 85.0, stripCross = 100.0, stripSea = 30.0;
    double stripPromenade = 0.0;   // a promenade this far out on the sand side: a beachfront row of blocks (0: none)
    int stripRows = 3;
    std::string stripAlong = "coast";   // the road it grows along: "coast" (the coast road) or "high" (a high road: a hamlet)
    bool stripBoth = false;             // its rows on both sides of the spine (a hamlet's street), not only inland
    std::string stripMain = "arterial";  // its spine's class: "arterial" (a seafront) or "local" (a hamlet's two-lane street)
    // ITS GROUND LEVELLED (procgen/regrade.h, ADR-0323): over its disc (or a strip's outline) and `regradeMargin` m more,
    // blurred over regradeBlur, capped at regradeMaxSlope, shelves regradeTerrace apart (0: none) with regradeBank,
    // eased back over regradeFeather; written into the terrain block when the place is added
    bool regrade = false;
    double regradeBlur = 100.0, regradeMaxSlope = 0.12, regradeTerrace = 0.0, regradeBank = 0.35, regradeFeather = 100.0, regradeMargin = 60.0;
    // ITS DISTRICTS (a strip town's hub is its seafront LINE): commercial within `hubReach` m of it, residential
    // behind; and its own MIXES, {district: [{recipe, weight}]}, in place of the district ladders (the level's hub
    // carries them: level_params.cpp)
    double hubReach = 0.0;
    std::map<std::string, std::vector<std::pair<std::string, double>>> mixes;
    // ITS LANDMARKS (the lot pass's landmark planner, per place): a `civic` kind has the city's full civic table (a
    // capitol, courthouse, hospital, university, schools by its quarters, ...); any other the list -- a town's school
    // and church, and a market hall in an old town
    bool civic = false;
    std::vector<SettlementLandmark> landmarks{{"school", 1, 10, 0, true}, {"church", 1, 8, 0, true}, {"market_hall", 1, 0, 6, false}};
    // THE MAP: its circle's colour, its dot (px) and its label (m)
    float rgb[3] = {0.95f, 0.45f, 0.05f};
    double dot = 5.0, label = 150.0;
};

struct SettlementCatalog {
    std::vector<SettlementKind> kinds;
    // The kind named `name`; the first kind when there is none (an unknown kind is planned as the first).
    const SettlementKind& of(const std::string& name) const;
    const SettlementKind* find(const std::string& name) const;
    // The kind a level's hub of district `hub` stands for, when the hub does not name one: the first kind with that
    // hub ("financial" -> city, "commercial" -> town).
    const SettlementKind* forHub(const std::string& hub) const;
    // What is wrong with it ("" when nothing): no kinds, a kind twice, a radius or a brief that cannot be.
    std::string check() const;
    std::string describe() const;
};

SettlementCatalog defaultSettlements();

}  // namespace engine

#endif
