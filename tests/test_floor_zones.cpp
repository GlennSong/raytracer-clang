// THE FLOOR AS A BLOCK (floor_zones.h; Glenn, 2026-10-16): any office plate -- a rectangle, a chamfered or notched
// one, a rounded or stadium section, a drum -- is cut into lots that do not overlap, cover the floor, front its
// circulation and stay inside its walls; the tagged floor walks, and the round towers get the office floor their
// rectangles always had (reception, meeting rooms, offices, open plan) instead of a ring round an empty middle.
#include "test_framework.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/furniture.h"
#include "../src/engine/scripting/furniture_library_lua.h"
#include "../src/engine/procgen/city/floor_zones.h"
#include "../src/engine/procgen/city/roads/lanes/geom2d.h"
#include "../src/engine/procgen/city/room_plan.h"
#include "../src/engine/procgen/city/section.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace engine;

namespace {

struct Plate {
    std::string name;
    Poly2 plan;
};

std::vector<Plate> plates() {
    std::vector<Plate> v;
    auto box = [](Real w, Real d) { return Section::rect(w, d).tessellate(); };
    v.push_back({"rect 48x34", box(48, 34)});
    v.push_back({"rect 36x24", box(36, 24)});
    v.push_back({"rect 30x20", box(30, 20)});
    v.push_back({"chamfered 40x30", Section::rect(40, 30).chamfer(0, 5.0).tessellate()});
    v.push_back({"notched 98x64", {{-49, -32}, {49, -32}, {49, 32}, {21.71, 32}, {21.71, 26.16}, {-17.45, 26.16},
                                   {-17.45, 32}, {-49, 32}}});
    v.push_back({"rounded r6", Section::rect(56, 40).rounded(6.0).tessellate()});
    v.push_back({"rounded r12", Section::rect(56, 40).rounded(12.0).tessellate()});
    v.push_back({"stadium 64x30", Section::rect(64, 30).rounded(14.9).tessellate()});
    v.push_back({"drum r18", Section::ngon(16, 18.0).tessellate()});
    v.push_back({"drum r24", Section::ngon(24, 24.0).tessellate()});
    return v;
}

BuildingParams officeParams(uint32_t seed) {
    BuildingParams p;
    p.floors = 20;
    p.curtainWall = true;
    p.officeUse = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = seed;
    return p;
}

// RT_ZONE_SVG=<dir>: each plate's lots drawn (window blue, corner purple, interior green; circulation grey; a lot with
// no way in red), and with RT_ZONE_SVG_ROOMS the tagged floor's rooms and walls.
void dumpSvg(const std::string& name, const Poly2& plan, const FloorPartition& fp, const RoomPlan* rp) {
    const char* dir = std::getenv("RT_ZONE_SVG");
    if (!dir) return;
    Vec2 lo, hi;
    bounds(plan, lo, hi);
    const Real s = 12.0, pad = 2.0;
    std::string fn = std::string(dir) + "/" + name + (rp ? "_rooms" : "_lots") + ".svg";
    for (char& c : fn) if (c == ' ') c = '_';
    std::ofstream f(fn);
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << (hi.x - lo.x + 2 * pad) * s << "' height='" << (hi.y - lo.y + 2 * pad) * s
      << "' style='background:#fff'>\n";
    auto pt = [&](const Vec2& p) { return std::to_string((p.x - lo.x + pad) * s) + "," + std::to_string((hi.y - p.y + pad) * s); };
    auto poly = [&](const Poly2& r, const char* fill, const char* stroke, Real w) {
        f << "<polygon points='";
        for (const Vec2& p : r) f << pt(p) << " ";
        f << "' fill='" << fill << "' stroke='" << stroke << "' stroke-width='" << w << "'/>\n";
    };
    poly(plan, "#f4f1ea", "#333", 2);
    for (const std::vector<Poly2>& c : fp.circulation) {
        poly(c.front(), "#ddd", "none", 0);
        for (std::size_t h = 1; h < c.size(); ++h) poly(c[h], "#f4f1ea", "none", 0);
    }
    if (!rp) {
        for (const FloorZone& z : fp.zones) {
            const char* fill = z.accessLen < 1.0 ? "#e66" : z.site == ZoneSite::Window ? "#9cf" : z.site == ZoneSite::Corner ? "#c9f" : "#9d9";
            poly(z.poly, fill, "#246", 1);
            if (z.frame.size() == 4) poly(z.frame, "none", "#000", 0.5);
        }
    } else {
        for (const Room& r : rp->rooms) {
            const char* fill = r.kind == RoomKind::Office ? "#c9f" : r.kind == RoomKind::Meeting ? "#fc6" : r.kind == RoomKind::OpenPlan ? "#9cf"
                             : r.kind == RoomKind::Reception ? "#f66" : r.kind == RoomKind::Kitchenette ? "#6c6" : r.kind == RoomKind::Lounge ? "#fa8"
                             : r.kind == RoomKind::CopyRoom ? "#a98" : r.kind == RoomKind::PhoneBooths ? "#7bb"
                             : r.kind == RoomKind::WaterCooler ? "#6ae" : r.kind == RoomKind::Living ? "#bde3a8"
                             : r.kind == RoomKind::Bed ? "#f3d9a4" : r.kind == RoomKind::Bath ? "#a8d8e8"
                             : r.kind == RoomKind::Kitchen ? "#e8b8a8" : r.kind == RoomKind::Hall ? "#eee"
                             : r.kind == RoomKind::Closet ? "#ccb" : r.kind == RoomKind::Flat ? "#f9a" : "#bbb";
            poly(r.rect, fill, "#888", 0.5);
            f << "<text x='" << pt(centroid(r.rect)).substr(0, pt(centroid(r.rect)).find(',')) << "' y='"
              << pt(centroid(r.rect)).substr(pt(centroid(r.rect)).find(',') + 1) << "' font-size='9' text-anchor='middle'>"
              << cellUseName(r.kind) << "</text>\n";
        }
        for (const RoomWall& w : rp->walls) {
            f << "<line x1='" << pt(w.a).substr(0, pt(w.a).find(',')) << "' y1='" << pt(w.a).substr(pt(w.a).find(',') + 1) << "' x2='"
              << pt(w.b).substr(0, pt(w.b).find(',')) << "' y2='" << pt(w.b).substr(pt(w.b).find(',') + 1) << "' stroke='"
              << (w.glass ? "#29c" : "#222") << "' stroke-width='2.5'/>\n";
            if (w.doorAt >= 0) {
                const Vec2 d = w.a + (w.b - w.a) * w.doorAt;
                f << "<circle cx='" << pt(d).substr(0, pt(d).find(',')) << "' cy='" << pt(d).substr(pt(d).find(',') + 1)
                  << "' r='4' fill='#c00'/>\n";
            }
        }
    }
    f << "</svg>\n";
}

}  // namespace

TEST_CASE(every_office_plate_is_cut_into_lots_that_tile_its_floor) {
    namespace G = roads::lanes;
    int badPlates = 0;
    for (const Plate& pl : plates())
        for (uint32_t seed = 1; seed <= 3; ++seed) {
            const BuildingParams p = officeParams(seed);
            const CorePlan core = coreFor(pl.plan, p, entranceEdgeFor(pl.plan, p));
            if (!core.valid) { std::printf("    [lots] %s: no core\n", pl.name.c_str()); ++badPlates; continue; }
            const Real inset = interiorWallInset(p);
            const FloorPartition fp = partitionFloor(pl.plan, core, inset, p, FloorZoneParams{});
            Real covered = 0;
            int overlaps = 0, outside = 0, cutOff = 0, unframed = 0, windows = 0, corners = 0, interior = 0;
            for (std::size_t a = 0; a < fp.zones.size(); ++a) {
                const FloorZone& z = fp.zones[a];
                covered += z.area;
                windows += z.site == ZoneSite::Window;
                corners += z.site == ZoneSite::Corner;
                interior += z.site == ZoneSite::Interior;
                if (z.accessLen < 1.0) ++cutOff;
                if (z.frame.size() != 4) ++unframed;
                for (const Vec2& c : z.poly) if (!pointInPolygon(pl.plan, c + (centroid(z.poly) - c) * 0.01)) ++outside;
                for (std::size_t b = a + 1; b < fp.zones.size(); ++b)
                    if (G::setArea(G::intersectSets(G::fromRing(z.poly), G::fromRing(fp.zones[b].poly))) > 0.05) ++overlaps;
            }
            const Real share = fp.fieldArea > 0 ? covered / fp.fieldArea : 0;
            if (seed == 1) dumpSvg(pl.name, pl.plan, fp, nullptr);
            if (seed == 1)
                std::printf("    [lots] %-16s %3zu lots (%d window, %d corner, %d interior), %.0f%% of %.0f m2; %d overlap, %d outside, "
                            "%d cut off, %d unframed\n", pl.name.c_str(), fp.zones.size(), windows, corners, interior, share * 100,
                            fp.fieldArea, overlaps, outside, cutOff, unframed);
            const bool ok = !fp.zones.empty() && overlaps == 0 && outside == 0 && cutOff == 0 && share >= 0.60;
            if (!ok) {
                ++badPlates;
                if (seed != 1)
                    std::printf("    [lots] %s seed %u: %zu lots, %.0f%%, %d overlap, %d outside, %d cut off\n", pl.name.c_str(), seed,
                                fp.zones.size(), share * 100, overlaps, outside, cutOff);
            }
        }
    CHECK(badPlates == 0);
}

TEST_CASE(every_office_plate_gets_a_zoned_office_floor_that_walks) {
    int floors = 0, walked = 0, withMeeting = 0, withReception = 0, withOpen = 0, withOffices = 0;
    for (const Plate& pl : plates())
        for (uint32_t seed = 1; seed <= 3; ++seed) {
            const BuildingParams p = officeParams(seed);
            const CorePlan core = coreFor(pl.plan, p, entranceEdgeFor(pl.plan, p));
            if (!core.valid) continue;
            const RoomPlan rp = roomPlan(pl.plan, p, core, static_cast<std::size_t>(-1), interiorWallInset(p), 5);
            ++floors;
            if (seed == 1) dumpSvg(pl.name, pl.plan, partitionFloor(pl.plan, core, interiorWallInset(p), p, FloorZoneParams{}), &rp);
            int n[64] = {};
            for (const Room& r : rp.rooms) ++n[static_cast<int>(r.kind)];
            const bool walks = floorIsWalkable(rp, pl.plan, core.frame.toWorld({core.length * 0.5, -1.1}));
            walked += walks;
            withMeeting += n[static_cast<int>(RoomKind::Meeting)] > 0;
            withReception += n[static_cast<int>(RoomKind::Reception)] > 0;
            withOpen += n[static_cast<int>(RoomKind::OpenPlan)] + n[static_cast<int>(RoomKind::DeskRow)] > 0;
            withOffices += n[static_cast<int>(RoomKind::Office)] > 0;
            if (seed == 1 || !walks) {
                std::printf("    [floor] %-16s seed %u: %zu rooms, %zu walls%s:", pl.name.c_str(), seed, rp.rooms.size(), rp.walls.size(),
                            walks ? "" : " NOT WALKABLE");
                for (int k = 0; k < 64; ++k) if (n[k]) std::printf(" %d %s", n[k], cellUseName(static_cast<RoomKind>(k)));
                std::printf("\n");
            }
        }
    std::printf("    [floor] %d floors: %d walk, %d meeting, %d reception, %d open plan, %d offices\n", floors, walked, withMeeting,
                withReception, withOpen, withOffices);
    CHECK(floors >= 27);
    CHECK(walked == floors);
    CHECK(withMeeting == floors);
    CHECK(withReception == floors);
    CHECK(withOpen == floors);
}

// EVERY ZONE IS FURNISHED: the tag's program fills the lot's frame -- a corner office in a rounded corner, an office
// in a drum's wedge, the open plan along a stadium's end -- none left an empty room.
TEST_CASE(every_zone_of_a_zoned_floor_is_furnished) {
    auto read = [](const char* name) {
        std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
        std::stringstream ss;
        ss << in.rdbuf();
        return ss.str();
    };
    FurnitureLibrary saved = FurnitureLibrary::global(), lib;
    std::string err;
    CHECK(loadFurnitureLibrarySource(read("furniture_library.lua") + "\n" + read("furniture_rooms.lua"), lib, &err));
    FurnitureLibrary::global() = lib;
    int rooms = 0, empty = 0;
    // the floor's points hold what they are for: a copier, phone booths, a water cooler
    int points = 0, pointsWithTheirPiece = 0;
    for (const Plate& pl : plates())
        for (uint32_t seed = 1; seed <= 3; ++seed) {
            const BuildingParams p = officeParams(seed);
            const CorePlan core = coreFor(pl.plan, p, entranceEdgeFor(pl.plan, p));
            if (!core.valid) continue;
            const RoomPlan rp = roomPlan(pl.plan, p, core, static_cast<std::size_t>(-1), interiorWallInset(p), 5);
            std::vector<PlacedPiece> placed;
            emitFurniture(placed, nullptr, rp, 20.0, p.seed, 23.0);
            for (const Room& r : rp.rooms) {
                ++rooms;
                int in = 0;
                for (const PlacedPiece& pc : placed)
                    if (pointInPolygon(r.rect, Vec2(pc.xform.m[0][3], pc.xform.m[2][3]))) ++in;
                const Piece want = r.kind == RoomKind::CopyRoom ? Piece::Copier : r.kind == RoomKind::PhoneBooths ? Piece::PhoneBooth
                                 : r.kind == RoomKind::WaterCooler ? Piece::WaterCooler : Piece::Count;
                if (want != Piece::Count) {
                    ++points;
                    bool has = false;
                    for (const PlacedPiece& pc : placed)
                        has = has || (pc.piece == static_cast<int>(want) && pointInPolygon(r.rect, Vec2(pc.xform.m[0][3], pc.xform.m[2][3])));
                    if (has) ++pointsWithTheirPiece;
                    else std::printf("    [furnished] %s seed %u: a %s without its piece, %.1f m2\n", pl.name.c_str(), seed,
                                     cellUseName(r.kind), area(r.rect));
                }
                if (in > 0) continue;
                ++empty;
                const Poly2& f = r.frame;
                std::printf("    [furnished] %s seed %u: an empty %s, %zu corners, %.1f m2, frame %s\n", pl.name.c_str(), seed, cellUseName(r.kind),
                            r.rect.size(), area(r.rect),
                            f.size() == 4 ? (std::to_string((f[1] - f[0]).length()).substr(0, 4) + " x " + std::to_string((f[3] - f[0]).length()).substr(0, 4)).c_str() : "none");
            }
        }
    FurnitureLibrary::global() = saved;
    std::printf("    [furnished] %d rooms, %d empty; %d copy rooms, phone booths and water coolers, %d with their piece\n",
                rooms, empty, points, pointsWithTheirPiece);
    CHECK(rooms > 500);
    CHECK(empty == 0);
    CHECK(points > 20);
    CHECK(pointsWithTheirPiece == points);
}

// THE SHIPPED PROGRAM (assets/scripts/floor_programs.lua) loads, names an office program whose zones all build a known
// room, and lays floors that walk -- the same gate as the built-in program it ships alongside.
TEST_CASE(the_shipped_floor_program_loads_and_lays_walkable_floors) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/floor_programs.lua");
    std::stringstream ss;
    ss << in.rdbuf();
    FloorProgramBook book;
    std::string err;
    const bool ok = loadFloorProgramsSource(ss.str(), book, &err);
    if (!ok) std::printf("    [program] %s\n", err.c_str());
    CHECK(ok);
    const FloorProgram* office = book.find("office");
    CHECK(office != nullptr);
    if (!office) return;
    const FloorProgram builtin = builtinOfficeProgram();
    std::printf("    [program] office: %zu zones, %zu styles (built-in %zu, %zu)\n", office->rules.size(), office->styles.size(),
                builtin.rules.size(), builtin.styles.size());
    CHECK(office->rules.size() >= 4);
    // the built-in programs are what the file ships with: the same lots, the same zones in the same order
    for (const FloorProgram& bi : {builtin, builtinApartmentProgram()}) {
        const FloorProgram* fp = book.find(bi.name);
        CHECK(fp != nullptr);
        if (!fp) continue;
        CHECK(fp->residential == bi.residential && fp->fill == bi.fill && fp->shallow == bi.shallow);
        CHECK(std::fabs(fp->zones.windowDepth - bi.zones.windowDepth) < 1e-9 && std::fabs(fp->zones.lotWidth - bi.zones.lotWidth) < 1e-9 &&
              std::fabs(fp->zones.interiorLot - bi.zones.interiorLot) < 1e-9 && std::fabs(fp->zones.minAccess - bi.zones.minAccess) < 1e-9 &&
              std::fabs(fp->zones.minLotArea - bi.zones.minLotArea) < 1e-9);
        CHECK(fp->rules.size() == bi.rules.size());
        for (std::size_t i = 0; i < std::min(fp->rules.size(), bi.rules.size()); ++i) {
            const ZoneRule &a = fp->rules[i], &b = bi.rules[i];
            CHECK(a.tag == b.tag && a.kind == b.kind && a.sites == b.sites && a.count == b.count && a.enclosed == b.enclosed &&
                  a.walls == b.walls && a.near == b.near && std::fabs(a.minFloor - b.minFloor) < 1e-9 &&
                  std::fabs(a.perM2 - b.perM2) < 1e-9 && std::fabs(a.share - b.share) < 1e-9 &&
                  std::fabs(a.minArea - b.minArea) < 1e-9 && std::fabs(a.maxArea - b.maxArea) < 1e-9 &&
                  std::fabs(a.minDepth - b.minDepth) < 1e-9);
            if (a.tag != b.tag) std::printf("    [program] %s zone %zu: %s in the file, %s built in\n", bi.name.c_str(), i, a.tag.c_str(), b.tag.c_str());
        }
    }
    int floors = 0, walked = 0;
    for (const Plate& pl : plates()) {
        const BuildingParams p = officeParams(2);
        const CorePlan core = coreFor(pl.plan, p, entranceEdgeFor(pl.plan, p));
        if (!core.valid) continue;
        const Vec2 entry = core.frame.toWorld({core.length * 0.5, -1.1});
        const RoomPlan rp = zonedFloorPlan(pl.plan, p, core, interiorWallInset(p), 5, true, entry, Poly2{}, *office);
        ++floors;
        walked += !rp.rooms.empty() && floorIsWalkable(rp, pl.plan, entry);
    }
    CHECK(floors >= 9);
    CHECK(walked == floors);
}

// APARTMENT FLOORS ARE ZONED TOO (Glenn, 2026-10-16: "partition the open space of an office floor or even a
// residential building's floor"): every plate, glass or masonry, is cut into flats along its glass, each reaching the
// corridor through its own front door and laid out inside -- a living room, bedrooms, a bath, a kitchen -- where the
// old apartment floor took rectangles only and a drum's floor was studios round an empty middle. Every floor walks
// from the lifts, few flats fall back to studios, and the rooms are furnished.
TEST_CASE(every_plate_gets_an_apartment_floor_of_flats) {
    auto read = [](const char* name) {
        std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
        std::stringstream ss;
        ss << in.rdbuf();
        return ss.str();
    };
    FurnitureLibrary saved = FurnitureLibrary::global(), lib;
    std::string err;
    CHECK(loadFurnitureLibrarySource(read("furniture_library.lua") + "\n" + read("furniture_rooms.lua"), lib, &err));
    FurnitureLibrary::global() = lib;
    int floors = 0, walked = 0, flats = 0, studios = 0, fronts = 0, unfurnished = 0, laid = 0;
    for (const Plate& pl : plates())
        for (uint32_t seed = 1; seed <= 4; ++seed) {
            BuildingParams p = officeParams(seed);
            p.officeUse = false;
            p.residential = true;
            p.curtainWall = seed % 2 == 0;
            const CorePlan core = coreFor(pl.plan, p, entranceEdgeFor(pl.plan, p));
            if (!core.valid) continue;
            const RoomPlan rp = roomPlan(pl.plan, p, core, static_cast<std::size_t>(-1), interiorWallInset(p), 5);
            ++floors;
            if (seed == 1 && std::getenv("RT_ZONE_SVG")) {
                const FloorProgram& ap = floorProgram("apartments");
                dumpSvg("apartments " + pl.name, pl.plan, partitionFloor(pl.plan, core, interiorWallInset(p), p, ap.zones), &rp);
            }
            int n[64] = {};
            for (const Room& r : rp.rooms) ++n[static_cast<int>(r.kind)];
            const bool walks = floorIsWalkable(rp, pl.plan, core.frame.toWorld({core.length * 0.5, -1.1}));
            walked += walks;
            // a laid flat has one bath and one room to live in (a living room, or a studio's Flat room); a lot that
            // would not lay is a Flat room on its own, with no bath
            const int liv = n[static_cast<int>(RoomKind::Bath)];
            const int st = std::max(0, n[static_cast<int>(RoomKind::Flat)] + n[static_cast<int>(RoomKind::Living)] - liv);
            flats += liv;
            studios += st;
            laid += liv + st > 0;
            for (const RoomWall& w : rp.walls) fronts += w.doorAt >= 0;
            std::vector<PlacedPiece> placed;
            emitFurniture(placed, nullptr, rp, 20.0, p.seed, 23.0);
            if (std::getenv("RT_APT_PROBE") && seed == 1 && pl.name == std::getenv("RT_APT_PROBE")) {
                const FloorPartition fpp = partitionFloor(pl.plan, core, interiorWallInset(p), p, floorProgram("apartments").zones);
                for (const FloorZone& z : fpp.zones)
                    std::printf("    [probe] zone site %d area %.1f depth %.1f access %.1f at %.1f %.1f\n", static_cast<int>(z.site), z.area,
                                z.depth, z.accessLen, centroid(z.poly).x, centroid(z.poly).y);
                std::printf("    [probe] %s: %zu pieces\n", pl.name.c_str(), placed.size());
                for (const Room& r : rp.rooms)
                    std::printf("    [probe]   %s frame %zu (%.2f,%.2f)-(%.2f,%.2f)-(%.2f,%.2f) rect0 (%.2f,%.2f)\n", cellUseName(r.kind), r.frame.size(),
                                r.frame.size() == 4 ? r.frame[0].x : 0., r.frame.size() == 4 ? r.frame[0].y : 0.,
                                r.frame.size() == 4 ? r.frame[1].x : 0., r.frame.size() == 4 ? r.frame[1].y : 0.,
                                r.frame.size() == 4 ? r.frame[3].x : 0., r.frame.size() == 4 ? r.frame[3].y : 0., r.rect[0].x, r.rect[0].y);
                for (std::size_t k = 0; k < std::min<std::size_t>(placed.size(), 10); ++k)
                    std::printf("    [probe]   piece %s at %.2f %.2f\n", furniturePieceName(static_cast<Piece>(placed[k].piece)), placed[k].xform.m[0][3], placed[k].xform.m[2][3]);
            }
            for (const Room& r : rp.rooms) {
                if (r.kind != RoomKind::Living && r.kind != RoomKind::Bed && r.kind != RoomKind::Kitchen && r.kind != RoomKind::Bath) continue;
                int in = 0;
                for (const PlacedPiece& pc : placed)
                    if (pointInPolygon(r.rect, Vec2(pc.xform.m[0][3], pc.xform.m[2][3]))) ++in;
                if (in == 0) {
                    ++unfurnished;
                    std::printf("    [apts] %s seed %u: an empty %s, %.1f m2\n", pl.name.c_str(), seed, cellUseName(r.kind), area(r.rect));
                }
            }
            if (seed == 1)
                std::printf("    [apts] %-16s %2d flats %d studios, %d beds %d baths %d kitchens, %zu walls; walks %d\n",
                            pl.name.c_str(), liv, st, n[static_cast<int>(RoomKind::Bed)], n[static_cast<int>(RoomKind::Bath)],
                            n[static_cast<int>(RoomKind::Kitchen)], rp.walls.size(), walks ? 1 : 0);
        }
    FurnitureLibrary::global() = saved;
    std::printf("    [apts] %d floors, %d with flats, %d walk; %d flats, %d studios, %d doors, %d living rooms unfurnished\n",
                floors, laid, walked, flats, studios, fronts, unfurnished);
    CHECK(floors >= 30);
    CHECK(laid == floors);
    CHECK(walked == floors);
    CHECK(flats >= 8 * floors);
    CHECK(studios * 5 <= flats);
    CHECK(unfurnished == 0);
}
