// Roads-v2 band model (plan Part 1): a road is an ordered list of BANDS, not
// "class + width". These pin the presets' shapes, the legacy compat shim (old
// width/oneWay levels keep loading, with their exact carriageway width), and
// the JSON round-trip (preset names stay names; custom band lists survive).
#include "test_framework.h"

#include "../src/engine/procgen/city/road_spec.h"
#include "../src/engine/procgen/city/road_net.h"
#include <cmath>

using namespace engine;

TEST_CASE(spec_presets_have_the_right_shape) {
    const RoadSpec fwy = roadSpecPreset("freeway3");
    CHECK(fwy.laneCount() == 6);
    CHECK(!fwy.hasSidewalk(-1));
    CHECK(!fwy.hasSidewalk(+1));
    CHECK(!fwy.isOneWay());                    // three lanes each way
    CHECK(fwy.designSpeed > 0);                // engineering pass runs
    CHECK(fwy.bands.front().kind == BandKind::Barrier);
    CHECK(fwy.bands.back().kind == BandKind::Barrier);

    const RoadSpec alley = roadSpecPreset("alley");
    CHECK(alley.laneCount() == 1);
    CHECK(alley.isOneWay());                   // direction lives per lane
    CHECK(!alley.hasSidewalk(-1));

    const RoadSpec dirt = roadSpecPreset("dirt");
    for (const RoadBand& b : dirt.bands) CHECK(b.surface == BandSurface::Dirt);
    CHECK(!dirt.hasSidewalk(-1));              // country road: no curbs, no walks

    const RoadSpec local = roadSpecPreset("local");
    CHECK(local.hasSidewalk(-1));
    CHECK(local.hasSidewalk(+1));
    CHECK(local.laneCount() == 2);
    CHECK(!local.isOneWay());
}

TEST_CASE(spec_legacy_shim_preserves_width) {
    // The old model: carriageway width 8, two-way, sidewalk 3.5, curb 0.16.
    const RoadSpec s = roadSpecFromLegacy(8.0, false, 3.5, 0.16);
    CHECK(std::fabs(s.carriagewayWidth() - 8.0) < 1e-6);   // EXACT width kept
    CHECK(s.laneCount() >= 2);
    CHECK(!s.isOneWay());
    CHECK(s.hasSidewalk(-1));
    CHECK(s.hasSidewalk(+1));

    const RoadSpec one = roadSpecFromLegacy(4.0, true, 0.0, 0.0);
    CHECK(one.isOneWay());
    CHECK(!one.hasSidewalk(-1));               // no sidewalk on a bare legacy lane
    CHECK(std::fabs(one.carriagewayWidth() - 4.0) < 1e-6);
}

TEST_CASE(spec_json_round_trips) {
    // A pristine preset serializes back to its NAME (compact, upgradeable).
    const RoadSpec fwy = roadSpecPreset("freeway3");
    const nlohmann::json j = roadSpecToJson(fwy);
    CHECK(j.is_string());
    const RoadSpec back = roadSpecFromJson(j);
    CHECK(back.bands.size() == fwy.bands.size());
    CHECK(std::fabs(back.totalWidth() - fwy.totalWidth()) < 1e-6);

    // A custom band list (Venice-Blvd-ish) survives field-for-field.
    RoadSpec venice;
    venice.bands = {
        { BandKind::Sidewalk, 3.0f, 0, BandSurface::Concrete, kPaintNone },
        { BandKind::Curb, 0.25f, 0, BandSurface::Concrete, kPaintRed },
        { BandKind::Parking, 2.4f, -1, BandSurface::Asphalt, kPaintNone },
        { BandKind::Travel, 3.6f, -1, BandSurface::Asphalt, kPaintNone },
        { BandKind::Travel, 3.6f, -1, BandSurface::Asphalt, kPaintNone },
        { BandKind::Turn, 3.6f, 0, BandSurface::Asphalt, kPaintNone },
        { BandKind::Travel, 3.6f, +1, BandSurface::Asphalt, kPaintNone },
        { BandKind::Travel, 3.6f, +1, BandSurface::Asphalt, kPaintNone },
        { BandKind::Parking, 2.4f, +1, BandSurface::Asphalt, kPaintNone },
        { BandKind::Curb, 0.25f, 0, BandSurface::Concrete, kPaintGreen },
        { BandKind::Sidewalk, 3.0f, 0, BandSurface::Concrete, kPaintNone },
    };
    const RoadSpec v2 = roadSpecFromJson(roadSpecToJson(venice));
    CHECK(v2.bands.size() == venice.bands.size());
    for (std::size_t i = 0; i < venice.bands.size(); ++i) {
        CHECK(v2.bands[i].kind == venice.bands[i].kind);
        CHECK(std::fabs(v2.bands[i].width - venice.bands[i].width) < 1e-6);
        CHECK(v2.bands[i].dir == venice.bands[i].dir);
        CHECK(v2.bands[i].surface == venice.bands[i].surface);
        CHECK(v2.bands[i].paint == venice.bands[i].paint);
    }
    CHECK(v2.laneCount() == 5);                // 4 travel + 1 turn
    CHECK(v2.bands[1].paint == kPaintRed);     // curb paint survives
}

TEST_CASE(spec_reaches_every_edge_via_the_net) {
    // A net with one spec'd edge and one legacy edge: both answer the band model.
    RoadEntity net;
    net.look.defaultWidth = 8.0;
    net.look.sidewalk = 3.5;
    net.look.curb = 0.16;
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(100, 0)},
                        RoadNode{Vec2(200, 0)} };
    net.graph.addEdge(0, 1, net.look.defaultWidth);
    net.graph.addEdge(1, 2, net.look.defaultWidth);
    net.graph.specs.push_back(roadSpecPreset("freeway3"));
    net.graph.edges[0].spec = 0;               // edge 0 spec'd, edge 1 legacy (-1)

    const RoadSpec e0 = roadNetEdgeSpec(net, 0);
    CHECK(e0.laneCount() == 6);
    CHECK(!e0.hasSidewalk(-1));

    const RoadSpec e1 = roadNetEdgeSpec(net, 1);
    CHECK(std::fabs(e1.carriagewayWidth() - 8.0) < 1e-6);  // legacy width kept
    CHECK(e1.hasSidewalk(-1));                              // net look -> sidewalk

    // JSON: a net block carrying specs parses them onto the net — the wire
    // format keeps the parallel edge_specs array, resolved onto the edges
    // (which must exist for the indices to land on).
    nlohmann::json j;
    j["width"] = 8.0;
    j["nodes"] = nlohmann::json::array(
        { { { "x", 0 }, { "z", 0 } }, { { "x", 100 }, { "z", 0 } },
          { { "x", 200 }, { "z", 0 } } });
    j["edges"] = nlohmann::json::array({ { 0, 1 }, { 1, 2 } });
    j["specs"] = nlohmann::json::array({ "freeway3", "local" });
    j["edge_specs"] = nlohmann::json::array({ 0, 1 });
    const RoadEntity parsed = roadNetFromJson(j);
    CHECK(parsed.graph.specs.size() == 2);
    CHECK(parsed.graph.edges.size() == 2);
    CHECK(parsed.graph.edges[0].spec == 0);
    CHECK(parsed.graph.edges[1].spec == 1);
    CHECK(parsed.graph.specs[0].laneCount() == 6);
    CHECK(parsed.graph.specs[1].hasSidewalk(-1));
}
