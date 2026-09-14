#include "engine/procgen/city/lot_cache.h"

#include "engine/mesh_builder.h"
#include <set>

#include "engine/bundle/codecs.h"

namespace engine {
namespace lotcache {

using bundle::BinReader;
using bundle::BinWriter;

// If this trips, BuildingParams gained or changed a field: extend putBuildingParams/getBuildingParams, the
// every-field test, and bump kLotsFormatVersion. (304 on clang/Linux x86_64 with Real = double.)
static_assert(sizeof(BuildingParams) == 352, "BuildingParams layout changed: update lot_cache.cpp");

namespace {
constexpr uint32_t kParamsVersion = 2, kLotsVersion = 5, kPlanVersion = 1, kGradeVersion = 1, kPartVersion = 1;

void putVec2(BinWriter& w, const Vec2& v) { w.put<double>(v.x); w.put<double>(v.y); }
bool getVec2(BinReader& r, Vec2& v) { return r.get(v.x) && r.get(v.y); }
void putVec3(BinWriter& w, const Vec3& v) { w.put<double>(v.x); w.put<double>(v.y); w.put<double>(v.z); }
bool getVec3(BinReader& r, Vec3& v) { return r.get(v.x) && r.get(v.y) && r.get(v.z); }
void putPoly(BinWriter& w, const Poly2& p) { w.put<uint32_t>(static_cast<uint32_t>(p.size())); for (const Vec2& q : p) putVec2(w, q); }
bool getPoly(BinReader& r, Poly2& p) { uint32_t n = 0; if (!r.get(n)) return false; p.resize(n); for (Vec2& q : p) if (!getVec2(r, q)) return false; return true; }
void putPolys(BinWriter& w, const std::vector<Poly2>& ps) { w.put<uint32_t>(static_cast<uint32_t>(ps.size())); for (const Poly2& p : ps) putPoly(w, p); }
bool getPolys(BinReader& r, std::vector<Poly2>& ps) { uint32_t n = 0; if (!r.get(n)) return false; ps.resize(n); for (Poly2& p : ps) if (!getPoly(r, p)) return false; return true; }
void putBool(BinWriter& w, bool b) { w.put<uint8_t>(b ? 1 : 0); }
bool getBool(BinReader& r, bool& b) { uint8_t v = 0; if (!r.get(v)) return false; b = v != 0; return true; }
}  // namespace

// ---- building parameters (every field, in declaration order) -------------------------------------------

void putBuildingParams(BinWriter& w, const BuildingParams& p) {
    w.magic("BPRM", kParamsVersion);
    w.put<int32_t>(p.floors); putBool(w, p.groundRetail); w.put<double>(p.floorHeight); w.put<double>(p.groundHeight); w.put<double>(p.bayWidth); w.put<double>(p.windowInset);
    putBool(w, p.walkableGround); w.put<double>(p.wallThickness); putBool(w, p.openDoorway); w.put<double>(p.setbackEvery); w.put<int32_t>(p.setbackFloors); w.put<double>(p.parapet);
    putVec3(w, p.wallColor); putBool(w, p.curtainWall); putBool(w, p.solidFacade); w.put<uint8_t>(static_cast<uint8_t>(p.wallPart)); putVec3(w, p.faceDir); w.put<double>(p.entranceDropBelow);
    putBool(w, p.baseCourse); putBool(w, p.stringCourse); putBool(w, p.pilasters); putBool(w, p.awning);
    const OpeningStyle& o = p.window;
    w.put<uint8_t>(static_cast<uint8_t>(o.head)); w.put<double>(o.archRise); w.put<double>(o.frameWidth); w.put<int32_t>(o.lightsX); w.put<int32_t>(o.lightsY); putBool(w, o.sill); w.put<uint8_t>(static_cast<uint8_t>(o.hood)); putVec3(w, o.frameColor);
    putBool(w, p.quoins); w.put<int32_t>(p.groundBays); w.put<int32_t>(p.portico); putBool(w, p.entranceSteps); putBool(w, p.dome);
    w.put<uint8_t>(static_cast<uint8_t>(p.roofStyle)); w.put<double>(p.roofPitch); putBool(w, p.retailStreetOnly); putBool(w, p.balconies); putBool(w, p.porch); putBool(w, p.chimney); putBool(w, p.spire); putBool(w, p.steeple); putBool(w, p.parkingDecks);
    w.put<int32_t>(p.sideBays); putVec3(w, p.trimColor); w.put<uint8_t>(static_cast<uint8_t>(p.shape)); w.put<int32_t>(p.tiers); w.put<int32_t>(p.sides); w.put<uint32_t>(p.seed);
    w.put<uint8_t>(static_cast<uint8_t>(p.envelope)); w.put<int32_t>(p.baseFloors); w.put<double>(p.setback1); w.put<int32_t>(p.stepFloors); w.put<double>(p.stepDepth); w.put<double>(p.towerFrac); w.put<int32_t>(p.towerFloor);
}

bool getBuildingParams(BinReader& r, BuildingParams& p) {
    uint32_t v = 0; if (!r.magic("BPRM", &v) || v != kParamsVersion) return false;
    uint8_t wallPart = 0, head = 0, hood = 0, roof = 0, shape = 0;
    if (!r.get(p.floors) || !getBool(r, p.groundRetail) || !r.get(p.floorHeight) || !r.get(p.groundHeight) || !r.get(p.bayWidth) || !r.get(p.windowInset)) return false;
    if (!getBool(r, p.walkableGround) || !r.get(p.wallThickness) || !getBool(r, p.openDoorway) || !r.get(p.setbackEvery) || !r.get(p.setbackFloors) || !r.get(p.parapet)) return false;
    if (!getVec3(r, p.wallColor) || !getBool(r, p.curtainWall) || !getBool(r, p.solidFacade) || !r.get(wallPart) || !getVec3(r, p.faceDir) || !r.get(p.entranceDropBelow)) return false;
    if (!getBool(r, p.baseCourse) || !getBool(r, p.stringCourse) || !getBool(r, p.pilasters) || !getBool(r, p.awning)) return false;
    OpeningStyle& o = p.window;
    if (!r.get(head) || !r.get(o.archRise) || !r.get(o.frameWidth) || !r.get(o.lightsX) || !r.get(o.lightsY) || !getBool(r, o.sill) || !r.get(hood) || !getVec3(r, o.frameColor)) return false;
    if (!getBool(r, p.quoins) || !r.get(p.groundBays) || !r.get(p.portico) || !getBool(r, p.entranceSteps) || !getBool(r, p.dome)) return false;
    if (!r.get(roof) || !r.get(p.roofPitch) || !getBool(r, p.retailStreetOnly) || !getBool(r, p.balconies) || !getBool(r, p.porch) || !getBool(r, p.chimney) || !getBool(r, p.spire) || !getBool(r, p.steeple) || !getBool(r, p.parkingDecks)) return false;
    if (!r.get(p.sideBays) || !getVec3(r, p.trimColor) || !r.get(shape) || !r.get(p.tiers) || !r.get(p.sides) || !r.get(p.seed)) return false;
    uint8_t envelope = 0;
    if (!r.get(envelope) || !r.get(p.baseFloors) || !r.get(p.setback1) || !r.get(p.stepFloors) || !r.get(p.stepDepth) || !r.get(p.towerFrac) || !r.get(p.towerFloor)) return false;
    p.envelope = static_cast<BuildingParams::Envelope>(envelope);
    p.wallPart = static_cast<PartId>(wallPart); o.head = static_cast<OpeningStyle::Head>(head); o.hood = static_cast<OpeningStyle::Hood>(hood);
    p.roofStyle = static_cast<BuildingParams::RoofStyle>(roof); p.shape = static_cast<BuildingShape>(shape);
    return r.ok();
}

// ---- lots ----------------------------------------------------------------------------------------------

void putLots(BinWriter& w, const std::vector<LotBuilding>& lots) {
    w.magic("LOTS", kLotsVersion); w.put<uint32_t>(static_cast<uint32_t>(lots.size()));
    for (const LotBuilding& lb : lots) {
        putVec2(w, lb.site); w.put<double>(lb.width); w.put<double>(lb.depth); w.put<double>(lb.height); w.put<double>(lb.yaw);
        w.putStr(lb.type); w.put<int32_t>(lb.block); w.putStr(lb.district); w.putStr(lb.recipe); w.put<double>(lb.baseY); w.put<double>(lb.groundY);
        putPoly(w, lb.plan); putVec3(w, lb.color); putPoly(w, lb.pad);
        bundle::putPackedMesh(w, bundle::packMesh(lb.padMesh, "pad", lb.color, 0u, 1.0f, 0.0));
        w.put<uint32_t>(static_cast<uint32_t>(lb.treeSpots.size())); for (const Vec3& t : lb.treeSpots) putVec3(w, t);
        w.put<uint32_t>(static_cast<uint32_t>(lb.fenceSegs.size())); for (const auto& f : lb.fenceSegs) { putVec2(w, f.first); putVec2(w, f.second); }
        w.put<uint32_t>(static_cast<uint32_t>(lb.units.size()));
        for (const BuildingUnit& u : lb.units) {
            putPoly(w, u.plan); w.put<double>(u.baseY); putBuildingParams(w, u.params);
            w.put<uint32_t>(static_cast<uint32_t>(u.doors.size())); for (const DoorSpec& d : u.doors) { putVec2(w, d.foot); putVec2(w, d.normal); w.put<double>(d.width); w.put<double>(d.height); }
            putBool(w, u.enterable);
        }
        // Site plan (format 3): the lot's ground around the building, by kind.
        w.put<uint32_t>(static_cast<uint32_t>(lb.open.size()));
        for (const OpenSpace& o : lb.open) { w.put<uint8_t>(static_cast<uint8_t>(o.kind)); putPoly(w, o.poly); }
        putPoly(w, lb.pavedLot); w.put<double>(lb.paveY);
        putPoly(w, lb.lot);
        putPoly(w, lb.padBound);
    }
}

bool getLots(BinReader& r, std::vector<LotBuilding>& lots) {
    uint32_t v = 0; if (!r.magic("LOTS", &v) || v != kLotsVersion) return false;
    uint32_t n = 0; if (!r.get(n)) return false; lots.clear(); lots.resize(n);
    for (LotBuilding& lb : lots) {
        if (!getVec2(r, lb.site) || !r.get(lb.width) || !r.get(lb.depth) || !r.get(lb.height) || !r.get(lb.yaw)) return false;
        if (!r.getStr(lb.type) || !r.get(lb.block) || !r.getStr(lb.district) || !r.getStr(lb.recipe) || !r.get(lb.baseY) || !r.get(lb.groundY)) return false;
        if (!getPoly(r, lb.plan) || !getVec3(r, lb.color) || !getPoly(r, lb.pad)) return false;
        bundle::PackedMesh pm; if (!bundle::getPackedMesh(r, pm)) return false; lb.padMesh = bundle::unpackMesh(pm);
        uint32_t nt = 0; if (!r.get(nt)) return false; lb.treeSpots.resize(nt); for (Vec3& t : lb.treeSpots) if (!getVec3(r, t)) return false;
        uint32_t nf = 0; if (!r.get(nf)) return false; lb.fenceSegs.resize(nf); for (auto& f : lb.fenceSegs) if (!getVec2(r, f.first) || !getVec2(r, f.second)) return false;
        uint32_t nu = 0; if (!r.get(nu)) return false; lb.units.resize(nu);
        for (BuildingUnit& u : lb.units) {
            if (!getPoly(r, u.plan) || !r.get(u.baseY) || !getBuildingParams(r, u.params)) return false;
            uint32_t nd = 0; if (!r.get(nd)) return false; u.doors.resize(nd);
            for (DoorSpec& d : u.doors) if (!getVec2(r, d.foot) || !getVec2(r, d.normal) || !r.get(d.width) || !r.get(d.height)) return false;
            if (!getBool(r, u.enterable)) return false;
        }
        uint32_t no = 0; if (!r.get(no)) return false; lb.open.resize(no);
        for (OpenSpace& o : lb.open) { uint8_t k = 0; if (!r.get(k) || !getPoly(r, o.poly)) return false; o.kind = static_cast<OpenKind>(k); }
        if (!getPoly(r, lb.pavedLot) || !r.get(lb.paveY)) return false;
        if (!getPoly(r, lb.lot)) return false;
        if (!getPoly(r, lb.padBound)) return false;
    }
    return r.ok();
}

// ---- plan debug -----------------------------------------------------------------------------------------

void putLotPlan(BinWriter& w, const LotPlanDebug& p) {
    w.magic("LPLN", kPlanVersion); putPolys(w, p.blocks); putPolys(w, p.lots);
    w.put<uint32_t>(static_cast<uint32_t>(p.alleys.size())); for (const auto& a : p.alleys) { putVec2(w, a.first); putVec2(w, a.second); }
    const int32_t counters[] = {p.rejChance, p.rejSliver, p.rejAspect, p.rejFill, p.rejPlan, p.rejClear, p.rejBox, p.rejFrontage, p.bisectedBlocks,
                                p.pEdgeShort, p.pShallow, p.pMitered, p.pOverlap, p.pEscaped, p.pTiny, p.pThin, p.pPlaced,
                                p.pClips, p.pLeftOverlapping, p.pSameEdge, p.pAtInsert, p.pConcave, p.pClipFailed, p.rejRelief};
    w.put<uint32_t>(static_cast<uint32_t>(sizeof(counters) / sizeof(counters[0]))); for (int32_t c : counters) w.put<int32_t>(c);
}

bool getLotPlan(BinReader& r, LotPlanDebug& p) {
    uint32_t v = 0; if (!r.magic("LPLN", &v) || v != kPlanVersion) return false;
    if (!getPolys(r, p.blocks) || !getPolys(r, p.lots)) return false;
    uint32_t na = 0; if (!r.get(na)) return false; p.alleys.resize(na); for (auto& a : p.alleys) if (!getVec2(r, a.first) || !getVec2(r, a.second)) return false;
    uint32_t nc = 0; if (!r.get(nc) || nc != 24) return false;
    int32_t* slots[] = {&p.rejChance, &p.rejSliver, &p.rejAspect, &p.rejFill, &p.rejPlan, &p.rejClear, &p.rejBox, &p.rejFrontage, &p.bisectedBlocks,
                        &p.pEdgeShort, &p.pShallow, &p.pMitered, &p.pOverlap, &p.pEscaped, &p.pTiny, &p.pThin, &p.pPlaced,
                        &p.pClips, &p.pLeftOverlapping, &p.pSameEdge, &p.pAtInsert, &p.pConcave, &p.pClipFailed, &p.rejRelief};
    for (int32_t* s : slots) if (!r.get(*s)) return false;
    return r.ok();
}

// ---- terrain flattens ------------------------------------------------------------------------------------

void putFlattens(BinWriter& w, const std::vector<TerrainFlatten>& fs) {
    w.magic("GRAD", kGradeVersion); w.put<uint32_t>(static_cast<uint32_t>(fs.size()));
    for (const TerrainFlatten& f : fs) {
        w.put<uint32_t>(static_cast<uint32_t>(f.polygon.size())); for (const Vec3& v : f.polygon) putVec3(w, v);
        w.put<double>(f.c); w.put<double>(f.dx); w.put<double>(f.dz); w.put<double>(f.falloff); w.put<double>(f.minX); w.put<double>(f.minZ); w.put<double>(f.maxX); w.put<double>(f.maxZ);
        w.put<uint8_t>(static_cast<uint8_t>(f.falloffMode)); w.put<double>(f.cutBatter); w.put<double>(f.fillBatter); w.put<int32_t>(f.priority); w.put<int32_t>(f.owner);
    }
}

bool getFlattens(BinReader& r, std::vector<TerrainFlatten>& fs) {
    uint32_t v = 0; if (!r.magic("GRAD", &v) || v != kGradeVersion) return false;
    uint32_t n = 0; if (!r.get(n)) return false; fs.clear(); fs.resize(n);
    for (TerrainFlatten& f : fs) {
        uint32_t np = 0; if (!r.get(np)) return false; f.polygon.resize(np); for (Vec3& p : f.polygon) if (!getVec3(r, p)) return false;
        uint8_t mode = 0;
        if (!r.get(f.c) || !r.get(f.dx) || !r.get(f.dz) || !r.get(f.falloff) || !r.get(f.minX) || !r.get(f.minZ) || !r.get(f.maxX) || !r.get(f.maxZ)) return false;
        if (!r.get(mode) || !r.get(f.cutBatter) || !r.get(f.fillBatter) || !r.get(f.priority) || !r.get(f.owner)) return false;
        f.falloffMode = static_cast<TerrainFlatten::Falloff>(mode);
    }
    return r.ok();
}

// ---- the whole result as sections ------------------------------------------------------------------------

namespace {
void putPart(BinWriter& w, const RenderMesh& m) { w.magic("PART", kPartVersion); w.put<int32_t>(m.materialIndex); bundle::putPackedMesh(w, bundle::packMesh(m, "part", Vec3(1, 1, 1), 0u, 1.0f, 0.0)); }
bool getPart(BinReader& r, RenderMesh& m) {
    uint32_t v = 0; if (!r.magic("PART", &v) || v != kPartVersion) return false; int32_t mat = 0; if (!r.get(mat)) return false;
    bundle::PackedMesh pm; if (!bundle::getPackedMesh(r, pm)) return false; m = bundle::unpackMesh(pm); m.materialIndex = mat; return r.ok();
}
}  // namespace

std::string cellPrefix(const std::string& prefix, int cx, int cz) { return prefix + "cell/" + std::to_string(cx) + "_" + std::to_string(cz) + "/"; }

void writeLotResult(bundle::BundleWriter& w, const std::string& prefix, const NetLotResult& res, double cell) {
    size_t built = 0, units = 0; for (const LotBuilding& lb : res.lots) { built += lb.plan.size() >= 3; units += lb.units.size(); }
    { BinWriter b; putLots(b, res.lots); w.add(prefix + "lots", b.bytes); }
    { BinWriter b; putLotPlan(b, res.plan); w.add(prefix + "plan", b.bytes); }
    { BinWriter b; putFlattens(b, res.gradeFlatten); w.add(prefix + "grade", b.bytes); }
    std::set<std::pair<int, int>> cells;
    if (cell <= 0.0) {
        for (size_t i = 0; i < res.parts.size(); ++i) { if (res.parts[i].vertices.empty()) continue; BinWriter b; putPart(b, res.parts[i]); w.add(prefix + "parts/" + std::to_string(i), b.bytes); }
        for (size_t i = 0; i < res.flatParts.size(); ++i) { if (res.flatParts[i].vertices.empty()) continue; BinWriter b; putPart(b, res.flatParts[i]); w.add(prefix + "flat/" + std::to_string(i), b.bytes); }
    } else {
        // Per render cell: a bundle cell IS a render cell, so a chunk is what the loader would have made at load.
        auto writeParts = [&](const std::vector<RenderMesh>& parts, const char* kind) {
            for (size_t i = 0; i < parts.size(); ++i) {
                if (parts[i].vertices.empty()) continue;
                for (MeshBuilder::CellChunk& c : MeshBuilder::chunkByCell(parts[i], cell)) {
                    if (c.mesh.vertices.empty()) continue;
                    c.mesh.materialIndex = parts[i].materialIndex; cells.insert({c.cx, c.cz});
                    BinWriter b; putPart(b, c.mesh); w.add(cellPrefix(prefix, c.cx, c.cz) + kind + "/" + std::to_string(i), b.bytes);
                }
            }
        };
        writeParts(res.parts, "parts"); writeParts(res.flatParts, "flat");
    }
    nlohmann::json cellList = nlohmann::json::array(); for (const auto& c : cells) cellList.push_back({c.first, c.second});
    // meta last: the cell list is known only after the parts are split (readers find sections by name, not order).
    w.addJson(prefix + "meta", nlohmann::json{{"format", kLotsFormatVersion}, {"lots", res.lots.size()}, {"built", built}, {"units", units}, {"parts", res.parts.size()}, {"flatParts", res.flatParts.size()}, {"grades", res.gradeFlatten.size()}, {"planBlocks", res.plan.blocks.size()}, {"planLots", res.plan.lots.size()}, {"cell", cell > 0.0 ? cell : 0.0}, {"cells", cellList}});
}

double lotCellSize(const bundle::Bundle& b, const std::string& prefix) { const nlohmann::json meta = b.json(prefix + "meta"); return meta.is_null() ? 0.0 : meta.value("cell", 0.0); }

bool listLotCellParts(const bundle::Bundle& b, const std::string& prefix, std::vector<LotCellPart>& out, std::string* err) {
    out.clear();
    const nlohmann::json meta = b.json(prefix + "meta"); if (meta.is_null()) { if (err) *err = prefix + "meta: missing"; return false; }
    if (meta.value("cell", 0.0) <= 0.0) return true;
    const size_t nParts = meta.value("parts", size_t(0)), nFlat = meta.value("flatParts", size_t(0));
    for (const nlohmann::json& c : meta.value("cells", nlohmann::json::array())) {
        if (!c.is_array() || c.size() < 2) continue;
        const int cx = c[0].get<int>(), cz = c[1].get<int>(); const std::string cp = cellPrefix(prefix, cx, cz);
        for (size_t i = 0; i < nParts; ++i) { const std::string sec = cp + "parts/" + std::to_string(i); if (b.has(sec)) out.push_back({cx, cz, static_cast<int>(i), false, sec}); }
        for (size_t i = 0; i < nFlat; ++i) { const std::string sec = cp + "flat/" + std::to_string(i); if (b.has(sec)) out.push_back({cx, cz, static_cast<int>(i), true, sec}); }
    }
    return true;
}

bool readLotPart(const bundle::Bundle& b, const std::string& section, RenderMesh& m) {
    const bundle::Bundle::View v = b.section(section); if (v.empty()) return false;
    BinReader r(v.data, v.size); return getPart(r, m);
}

bool readLotResult(const bundle::Bundle& b, const std::string& prefix, NetLotResult& res, std::string* err, bool withParts) {
    auto fail = [&](const std::string& why) { if (err) *err = prefix + why; return false; };
    const nlohmann::json meta = b.json(prefix + "meta"); if (meta.is_null()) return fail("meta: missing");
    if (meta.value("format", 0) != kLotsFormatVersion) return fail("meta: lots format " + std::to_string(meta.value("format", 0)));
    bundle::Bundle::View v;
    v = b.section(prefix + "lots"); if (v.empty()) return fail("lots: missing"); { BinReader r(v.data, v.size); if (!getLots(r, res.lots)) return fail("lots: unreadable"); }
    v = b.section(prefix + "plan"); if (v.empty()) return fail("plan: missing"); { BinReader r(v.data, v.size); if (!getLotPlan(r, res.plan)) return fail("plan: unreadable"); }
    v = b.section(prefix + "grade"); if (v.empty()) return fail("grade: missing"); { BinReader r(v.data, v.size); if (!getFlattens(r, res.gradeFlatten)) return fail("grade: unreadable"); }
    res.parts.assign(meta.value("parts", size_t(0)), RenderMesh()); res.flatParts.assign(meta.value("flatParts", size_t(0)), RenderMesh());
    if (!withParts) return true;
    if (meta.value("cell", 0.0) <= 0.0) {
        for (size_t i = 0; i < res.parts.size(); ++i) { v = b.section(prefix + "parts/" + std::to_string(i)); if (v.empty()) continue; BinReader r(v.data, v.size); if (!getPart(r, res.parts[i])) return fail("parts/" + std::to_string(i) + ": unreadable"); }
        for (size_t i = 0; i < res.flatParts.size(); ++i) { v = b.section(prefix + "flat/" + std::to_string(i)); if (v.empty()) continue; BinReader r(v.data, v.size); if (!getPart(r, res.flatParts[i])) return fail("flat/" + std::to_string(i) + ": unreadable"); }
        return true;
    }
    std::vector<LotCellPart> cps; if (!listLotCellParts(b, prefix, cps, err)) return false;
    for (const LotCellPart& cp : cps) {   // reassemble: chunks appended in cell order (vertex order differs from the grown part; triangles do not)
        RenderMesh chunk; if (!readLotPart(b, cp.section, chunk)) return fail(cp.section.substr(prefix.size()) + ": unreadable");
        std::vector<RenderMesh>& vec = cp.flat ? res.flatParts : res.parts;
        if (cp.part < 0 || static_cast<size_t>(cp.part) >= vec.size()) return fail(cp.section.substr(prefix.size()) + ": part index out of range");
        RenderMesh& dst = vec[static_cast<size_t>(cp.part)]; const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
        dst.materialIndex = chunk.materialIndex;
        dst.vertices.insert(dst.vertices.end(), chunk.vertices.begin(), chunk.vertices.end());
        dst.indices.reserve(dst.indices.size() + chunk.indices.size()); for (uint32_t i : chunk.indices) dst.indices.push_back(base + i);
    }
    return true;
}

}  // namespace lotcache
}  // namespace engine
