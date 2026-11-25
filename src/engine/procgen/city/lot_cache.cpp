#include "engine/procgen/city/lot_cache.h"

#include "engine/mesh_builder.h"
#include "job_system.h"   // parallelRows: the cell chunks encoded on the shared pool (#317 step 5)
#include <set>
#include <stdexcept>
#include <cstdlib>
#include <cstdio>
#include <cmath>

#include "engine/bundle/codecs.h"

namespace engine {
namespace lotcache {

using bundle::BinReader;
using bundle::BinWriter;

constexpr std::size_t kBuildingParamsSize = 824;   // v24: stairPlace, liftBanks (in the uint8 run, no growth); v23: partyOpen0/1/K0/K1, mallOpenEnds; v22: links (linkN, linkC, linkStorey, linkLen, linkW); v21: skin, skinGap; v20: massMesh; v19: diagrid (in the uint8 run, no growth); v18: wave, waveLen; v17: lean; v16: four party walls, their tops and doors; v15: bulge; v13: storeFloors, dineFloors; v12: mallArms, mallCourt   // 432 before the party walls (params v8); 352 until the NYC-variety envelope + curtain fields (params v5)
// If this trips, BuildingParams gained or changed a field: extend putBuildingParams/getBuildingParams, the
// every-field test, and bump kLotsFormatVersion. (304 on clang/Linux x86_64 with Real = double.)
static_assert(sizeof(BuildingParams) == kBuildingParamsSize, "BuildingParams layout changed: update lot_cache.cpp");

namespace {
// kLotsVersion 16: a lot's paths as walkways (id, kind, width, line, heights, both ends), not lines (ADR-0476);
// 15: a building's and each door's frontage (street, station, kerb; walkway and along), ADR-0474
constexpr uint32_t kParamsVersion = 24, kLotsVersion = 16, kPlanVersion = 1, kGradeVersion = 1, kPartVersion = 2;

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
void putFrontage(BinWriter& w, const StatedFrontage& f) { w.putStr(f.street); w.put<double>(f.station); w.put<int32_t>(f.dir); w.putStr(f.walkway); w.put<double>(f.along); }
bool getFrontage(BinReader& r, StatedFrontage& f) { int32_t dir = 1; if (!r.getStr(f.street) || !r.get(f.station) || !r.get(dir) || !r.getStr(f.walkway) || !r.get(f.along)) return false; f.dir = dir; return true; }
void putWalkway(BinWriter& w, const ModelWalkway& k) {
    w.putStr(k.id); w.putStr(k.kind); w.put<double>(k.width);
    w.put<uint32_t>(static_cast<uint32_t>(k.line.size())); for (const Vec2& q : k.line) putVec2(w, q);
    w.put<uint32_t>(static_cast<uint32_t>(k.heights.size())); for (Real h : k.heights) w.put<double>(h);
    for (const WalkwayEnd& e : k.ends) { putFrontage(w, e.meets); putBool(w, e.deadEnd); }
}
bool getWalkway(BinReader& r, ModelWalkway& k) {
    uint32_t n = 0;
    if (!r.getStr(k.id) || !r.getStr(k.kind) || !r.get(k.width) || !r.get(n)) return false;
    k.line.resize(n); for (Vec2& q : k.line) if (!getVec2(r, q)) return false;
    if (!r.get(n)) return false;
    k.heights.resize(n); for (Real& h : k.heights) if (!r.get(h)) return false;
    for (WalkwayEnd& e : k.ends) if (!getFrontage(r, e.meets) || !getBool(r, e.deadEnd)) return false;
    return true;
}
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
    w.put<uint8_t>(static_cast<uint8_t>(p.envelope)); w.put<int32_t>(p.baseFloors); w.put<double>(p.setback1); w.put<int32_t>(p.stepFloors); w.put<double>(p.stepDepth); w.put<double>(p.towerFrac); w.put<int32_t>(p.towerFloor); w.put<uint8_t>(p.core); w.put<uint8_t>(p.stairPlace); w.put<uint8_t>(p.liftBanks); w.put<uint8_t>(p.crown); w.put<uint8_t>(p.signage); w.put<uint8_t>(p.uplights); w.put<uint8_t>(p.top); w.put<uint8_t>(p.helipad); w.put<uint8_t>(p.windowGroup); w.put<uint8_t>(p.verticals ? 1 : 0); w.put<uint8_t>(p.residential ? 1 : 0);
    w.put<double>(p.skyRatio); w.put<double>(p.taperTop); w.put<double>(p.chamferTop); w.put<double>(p.featherFrom); w.put<double>(p.twistDeg); w.put<double>(p.stackShift); w.put<double>(p.bulge); w.put<double>(p.lean); w.put<double>(p.wave); w.put<double>(p.waveLen); w.put<uint8_t>(p.massMesh); w.put<uint8_t>(p.skin); w.put<double>(p.skinGap);
    w.put<uint8_t>(p.glassTint); w.put<uint8_t>(p.mullionTone); w.put<uint8_t>(p.fins); w.put<uint8_t>(p.diagrid); w.put<double>(p.curtainBay); w.put<double>(p.spandrelFrac);
    w.put<uint8_t>(p.partyWalls); for (int k = 0; k < 4; ++k) { w.put<double>(p.partyN[k].x); w.put<double>(p.partyN[k].y); w.put<double>(p.partyAt[k]); w.put<int32_t>(p.partyTop[k]); } w.put<uint8_t>(p.partyDoors); for (int k = 0; k < 4; ++k) { w.put<double>(p.partyOpen0[k]); w.put<double>(p.partyOpen1[k]); w.put<int32_t>(p.partyOpenK0[k]); w.put<int32_t>(p.partyOpenK1[k]); } w.put<uint8_t>(p.links); for (int k = 0; k < 2; ++k) { w.put<double>(p.linkN[k].x); w.put<double>(p.linkN[k].y); w.put<double>(p.linkC[k].x); w.put<double>(p.linkC[k].y); w.put<int32_t>(p.linkStorey[k]); w.put<double>(p.linkLen[k]); w.put<double>(p.linkW[k]); } putBool(w, p.backDoor); putBool(w, p.fireEscape); w.put<uint8_t>(p.bigBox); w.put<uint8_t>(p.campus); w.put<uint8_t>(p.shopMix); w.put<uint8_t>(p.mallArms); w.put<uint8_t>(p.mallOpenEnds); w.put<double>(p.mallCourt.x); w.put<double>(p.mallCourt.y); w.put<uint8_t>(p.storeFloors); w.put<uint8_t>(p.dineFloors);
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
    if (!r.get(p.core) || !r.get(p.stairPlace) || !r.get(p.liftBanks) || !r.get(p.crown) || !r.get(p.signage) || !r.get(p.uplights) || !r.get(p.top) || !r.get(p.helipad)) return false;
    { uint8_t wg = 1, vt = 0; uint8_t rs = 0; if (!r.get(wg) || !r.get(vt) || !r.get(rs)) return false; p.windowGroup = wg; p.verticals = vt != 0; p.residential = rs != 0; }
    if (!r.get(p.skyRatio) || !r.get(p.taperTop) || !r.get(p.chamferTop) || !r.get(p.featherFrom) || !r.get(p.twistDeg) || !r.get(p.stackShift) || !r.get(p.bulge) || !r.get(p.lean) || !r.get(p.wave) || !r.get(p.waveLen) || !r.get(p.massMesh) || !r.get(p.skin) || !r.get(p.skinGap)) return false;
    if (!r.get(p.glassTint) || !r.get(p.mullionTone) || !r.get(p.fins) || !r.get(p.diagrid) || !r.get(p.curtainBay) || !r.get(p.spandrelFrac)) return false;
    if (!r.get(p.partyWalls)) return false;
    for (int k = 0; k < 4; ++k) { double nx = 0, nz = 0; int32_t top = 0; if (!r.get(nx) || !r.get(nz) || !r.get(p.partyAt[k]) || !r.get(top)) return false; p.partyN[k] = Vec2(nx, nz); p.partyTop[k] = static_cast<int16_t>(top); }
    if (!r.get(p.partyDoors)) return false;
    for (int k = 0; k < 4; ++k) {
        int32_t k0 = 0, k1 = -1;
        if (!r.get(p.partyOpen0[k]) || !r.get(p.partyOpen1[k]) || !r.get(k0) || !r.get(k1)) return false;
        p.partyOpenK0[k] = static_cast<int16_t>(k0); p.partyOpenK1[k] = static_cast<int16_t>(k1);
    }
    if (!r.get(p.links)) return false;
    for (int k = 0; k < 2; ++k) {
        double nx = 0, nz = 0, cx = 0, cz = 0; int32_t st = 0;
        if (!r.get(nx) || !r.get(nz) || !r.get(cx) || !r.get(cz) || !r.get(st) || !r.get(p.linkLen[k]) || !r.get(p.linkW[k])) return false;
        p.linkN[k] = Vec2(nx, nz); p.linkC[k] = Vec2(cx, cz); p.linkStorey[k] = static_cast<int16_t>(st);
    }
    if (!getBool(r, p.backDoor) || !getBool(r, p.fireEscape) || !r.get(p.bigBox) || !r.get(p.campus) || !r.get(p.shopMix) || !r.get(p.mallArms) || !r.get(p.mallOpenEnds) || !r.get(p.mallCourt.x) || !r.get(p.mallCourt.y) || !r.get(p.storeFloors) || !r.get(p.dineFloors)) return false;
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
        w.put<uint32_t>(static_cast<uint32_t>(lb.furniture.size())); for (const OutdoorPiece& f : lb.furniture) { w.put<uint8_t>(f.piece); w.put<uint32_t>(f.variant); putVec2(w, f.at); w.put<double>(f.yaw); w.put<double>(f.y); putBool(w, f.draped); }
        w.put<uint32_t>(static_cast<uint32_t>(lb.fenceSegs.size())); for (const auto& f : lb.fenceSegs) { putVec2(w, f.first); putVec2(w, f.second); }
        w.put<uint32_t>(static_cast<uint32_t>(lb.units.size()));
        for (const BuildingUnit& u : lb.units) {
            putPoly(w, u.plan); w.put<double>(u.baseY); putBuildingParams(w, u.params);
            w.put<uint32_t>(static_cast<uint32_t>(u.doors.size())); for (const DoorSpec& d : u.doors) { putVec2(w, d.foot); putVec2(w, d.normal); w.put<double>(d.width); w.put<double>(d.height); putBool(w, d.back); putBool(w, d.open); putFrontage(w, d.frontage); }
            putBool(w, u.enterable);
            w.put<uint32_t>(static_cast<uint32_t>(u.beacons.size())); for (const Vec3& b : u.beacons) putVec3(w, b);
            w.put<uint32_t>(static_cast<uint32_t>(u.roof.size()));   // (format 14: the roof's items)
            for (const RoofItem& ri : u.roof) { w.put<uint8_t>(static_cast<uint8_t>(ri.kind)); putPoly(w, ri.footprint); w.put<double>(ri.y0); w.put<double>(ri.h); }
        }
        // Site plan (format 3): the lot's ground around the building, by kind.
        w.put<uint32_t>(static_cast<uint32_t>(lb.open.size()));
        for (const OpenSpace& o : lb.open) { w.put<uint8_t>(static_cast<uint8_t>(o.kind)); putPoly(w, o.poly); }
        putPoly(w, lb.pavedLot); w.put<double>(lb.paveY);
        putPoly(w, lb.lot);
        putPoly(w, lb.padBound);
        putPolys(w, lb.sealed);   // lots v9: a park's paths and plazas (no grass on them)
        w.put<uint32_t>(static_cast<uint32_t>(lb.walkways.size()));   // lots v16: its paths as walkways (ADR-0476)
        for (const ModelWalkway& k : lb.walkways) putWalkway(w, k);
        putPolys(w, lb.loops);   // lots v11: a track to run
        w.put<uint32_t>(static_cast<uint32_t>(lb.areas.size()));   // lots v12: areas (a pitch)
        for (const LotBuilding::Area& ar : lb.areas) { w.putStr(ar.kind); putVec2(w, ar.center); putVec2(w, ar.axis); w.put<double>(ar.halfL); w.put<double>(ar.halfW); }
        w.put<uint32_t>(static_cast<uint32_t>(lb.sitSpots.size()));   // lots v13: places to sit (a fountain's rim)
        for (const LotBuilding::SitSpot& sp : lb.sitSpots) { putVec2(w, sp.at); putVec2(w, sp.face); w.put<double>(sp.hip); }
        putFrontage(w, lb.frontage);   // lots v15: where it fronts
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
        uint32_t nfu = 0; if (!r.get(nfu)) return false; lb.furniture.resize(nfu); for (OutdoorPiece& f : lb.furniture) if (!r.get(f.piece) || !r.get(f.variant) || !getVec2(r, f.at) || !r.get(f.yaw) || !r.get(f.y) || !getBool(r, f.draped)) return false;
        uint32_t nf = 0; if (!r.get(nf)) return false; lb.fenceSegs.resize(nf); for (auto& f : lb.fenceSegs) if (!getVec2(r, f.first) || !getVec2(r, f.second)) return false;
        uint32_t nu = 0; if (!r.get(nu)) return false; lb.units.resize(nu);
        for (BuildingUnit& u : lb.units) {
            if (!getPoly(r, u.plan) || !r.get(u.baseY) || !getBuildingParams(r, u.params)) return false;
            uint32_t nd = 0; if (!r.get(nd)) return false; u.doors.resize(nd);
            for (DoorSpec& d : u.doors) if (!getVec2(r, d.foot) || !getVec2(r, d.normal) || !r.get(d.width) || !r.get(d.height) || !getBool(r, d.back) || !getBool(r, d.open) || !getFrontage(r, d.frontage)) return false;
            if (!getBool(r, u.enterable)) return false;
            uint32_t nb = 0; if (!r.get(nb)) return false; u.beacons.resize(nb);
            for (Vec3& b : u.beacons) if (!getVec3(r, b)) return false;
            uint32_t nr = 0; if (!r.get(nr)) return false; u.roof.resize(nr);
            for (RoofItem& ri : u.roof) { uint8_t k = 0; if (!r.get(k) || !getPoly(r, ri.footprint) || !r.get(ri.y0) || !r.get(ri.h)) return false; ri.kind = static_cast<RoofItem::Kind>(k); }
        }
        uint32_t no = 0; if (!r.get(no)) return false; lb.open.resize(no);
        for (OpenSpace& o : lb.open) { uint8_t k = 0; if (!r.get(k) || !getPoly(r, o.poly)) return false; o.kind = static_cast<OpenKind>(k); }
        if (!getPoly(r, lb.pavedLot) || !r.get(lb.paveY)) return false;
        if (!getPoly(r, lb.lot)) return false;
        if (!getPoly(r, lb.padBound)) return false;
        if (!getPolys(r, lb.sealed)) return false;
        uint32_t nw = 0; if (!r.get(nw)) return false; lb.walkways.resize(nw);
        for (ModelWalkway& k : lb.walkways) if (!getWalkway(r, k)) return false;
        if (!getPolys(r, lb.loops)) return false;
        uint32_t na = 0; if (!r.get(na)) return false; lb.areas.resize(na);
        for (LotBuilding::Area& ar : lb.areas)
            if (!r.getStr(ar.kind) || !getVec2(r, ar.center) || !getVec2(r, ar.axis) || !r.get(ar.halfL) || !r.get(ar.halfW)) return false;
        uint32_t ns = 0; if (!r.get(ns)) return false; lb.sitSpots.resize(ns);
        for (LotBuilding::SitSpot& sp : lb.sitSpots) if (!getVec2(r, sp.at) || !getVec2(r, sp.face) || !r.get(sp.hip)) return false;
        if (!getFrontage(r, lb.frontage)) return false;
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
// v2: the compact mesh (ADR-0224); v1 (packed float planes) still reads
void putPart(BinWriter& w, const RenderMesh& m) { w.magic("PART", kPartVersion); w.put<int32_t>(m.materialIndex); bundle::putCompactMesh(w, m); }
bool getPart(BinReader& r, RenderMesh& m) {
    uint32_t v = 0; if (!r.magic("PART", &v) || (v != kPartVersion && v != 1)) return false; int32_t mat = 0; if (!r.get(mat)) return false;
    if (v >= 2) { if (!bundle::getCompactMesh(r, m)) return false; m.materialIndex = mat; return r.ok(); }
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
        // (a material's chunks made one at a time, #317: all of them at once were a second copy of it; and encoded a few
        // dozen at once on the shared pool, step 5, then added in the order they were made)
        const std::size_t batch = 2 * (static_cast<std::size_t>(sharedJobs().workerCount()) + 1);
        constexpr std::size_t kBatchBytes = std::size_t(256) << 20;   // ...or this many bytes of chunks, whichever is first
        std::vector<MeshBuilder::CellChunk> chunks;
        std::size_t chunkBytes = 0;
        std::vector<std::vector<uint8_t>> encoded;
        auto writeParts = [&](const std::vector<RenderMesh>& parts, const char* kind) {
            for (size_t i = 0; i < parts.size(); ++i) {
                if (parts[i].vertices.empty()) continue;
                auto flush = [&]() {
                    encoded.assign(chunks.size(), std::vector<uint8_t>());
                    parallelRows(static_cast<int>(chunks.size()), [&](int k) {
                        BinWriter b; putPart(b, chunks[static_cast<size_t>(k)].mesh); encoded[static_cast<size_t>(k)] = std::move(b.bytes);
                    }, 1);
                    for (size_t k = 0; k < chunks.size(); ++k)
                        w.add(cellPrefix(prefix, chunks[k].cx, chunks[k].cz) + kind + "/" + std::to_string(i), encoded[k]);
                    chunks.clear();
                    encoded.clear();
                    chunkBytes = 0;
                };
                MeshBuilder::forEachCellChunk(parts[i], cell, [&](MeshBuilder::CellChunk& c) {
                    if (c.mesh.vertices.empty()) return;
                    c.mesh.materialIndex = parts[i].materialIndex; cells.insert({c.cx, c.cz});
                    chunkBytes += c.mesh.vertices.size() * sizeof(Vertex) + c.mesh.indices.size() * sizeof(uint32_t);
                    chunks.push_back(std::move(c));
                    if (chunks.size() >= batch || chunkBytes >= kBatchBytes) flush();
                });
                flush();
            }
        };
        writeParts(res.parts, "parts"); writeParts(res.flatParts, "flat");
    }
    nlohmann::json cellList = nlohmann::json::array(); for (const auto& c : cells) cellList.push_back({c.first, c.second});
    // meta last: the cell list is known only after the parts are split (readers find sections by name, not order).
    w.addJson(prefix + "meta", nlohmann::json{{"format", kLotsFormatVersion}, {"lots", res.lots.size()}, {"built", built}, {"units", units}, {"parts", res.parts.size()}, {"flatParts", res.flatParts.size()}, {"grades", res.gradeFlatten.size()}, {"planBlocks", res.plan.blocks.size()}, {"planLots", res.plan.lots.size()}, {"cell", cell > 0.0 ? cell : 0.0}, {"cells", cellList}});
}

// ---- the lots written tile by tile (ADR-0504) -------------------------------------------------------------------

namespace {
std::string tileName(const LotTileId& t) { return std::to_string(t.tz) + "_" + std::to_string(t.tx); }
std::string hexKey(uint64_t k) { char b[24]; std::snprintf(b, sizeof b, "%016llx", static_cast<unsigned long long>(k)); return b; }
uint64_t keyOfHex(const std::string& s) { return std::strtoull(s.c_str(), nullptr, 16); }
}  // namespace

TileCellWriter::TileCellWriter(bundle::BundleWriter& w, std::string prefix, double cell, const bundle::Bundle* previous)
    : w_(w), prefix_(std::move(prefix)), cell_(cell), prev_(previous) {
    if (!prev_) return;
    const nlohmann::json was = prev_->json(prefix_ + "tiles");
    if (!was.is_array()) { prev_ = nullptr; return; }
    for (const nlohmann::json& t : was) {
        Was& wt = was_[LotTileId{t.value("tz", int64_t(0)), t.value("tx", int64_t(0))}];
        wt.key = keyOfHex(t.value("key", std::string()));
        wt.malls = t.value("malls", false);
        wt.counts = t.value("counts", std::vector<int>());
        for (const nlohmann::json& c : t.value("cells", nlohmann::json::array()))
            if (c.is_array() && c.size() == 2) wt.cells.push_back({c[0].get<int>(), c[1].get<int>()});
    }
}

// Kept: the same key as last time, and it read nothing other tiles made (the malls placed before it)
bool TileCellWriter::keep(const LotTilePlan& tile, uint64_t key, LotTile& kept) {
    if (!prev_) return false;
    const std::map<LotTileId, Was>::const_iterator wt = was_.find(tile.id);
    if (wt == was_.end() || wt->second.key != key || wt->second.malls) return false;
    const std::string pre = prefix_ + "tile/" + tileName(tile.id) + "/";
    bundle::Bundle::View v = prev_->section(pre + "lots");
    if (v.empty()) return false;
    { bundle::BinReader r(v.data, v.size); if (!getLots(r, kept.lots)) return false; }
    v = prev_->section(pre + "grade");
    if (v.empty()) return false;
    { bundle::BinReader r(v.data, v.size); if (!getFlattens(r, kept.grade)) return false; }
    if (!setLotPlanCounts(kept.counts, wt->second.counts)) return false;
    kept.malls = false;
    return true;
}

void TileCellWriter::done(LotTile& tile) {
    const std::string name = tileName(tile.id);
    const std::string pre = prefix_ + "tile/" + name + "/";
    { BinWriter b; putLots(b, tile.lots); w_.add(pre + "lots", b.bytes); }
    { BinWriter b; putFlattens(b, tile.grade); w_.add(pre + "grade", b.bytes); }
    std::set<std::pair<int, int>> cells;
    const std::map<LotTileId, Was>::const_iterator wt = was_.find(tile.id);
    if (tile.kept && prev_ && wt != was_.end()) {
        // KEPT: its cells' sections copied as they were (the writer clones their blocks)
        ++kept_;
        for (const std::pair<int, int>& c : wt->second.cells) {
            for (const std::string& s : prev_->sections(cellPrefix(prefix_, c.first, c.second) + name + "/")) {
                const bundle::Bundle::View v = prev_->section(s);
                w_.add(s, v.data, v.size);
            }
            cells.insert(c);
        }
    } else {
        // GROWN: its parts cut into cells, each cell's chunks encoded on the pool and written in order
        ++grown_;
        struct Piece { std::pair<int, int> cell; bool flat; std::size_t slot; RenderMesh mesh; std::vector<uint8_t> bytes; };
        std::vector<Piece> pieces;
        auto cut = [&](std::vector<RenderMesh>& parts, bool flat) {
            for (std::size_t i = 0; i < parts.size(); ++i) {
                if (parts[i].vertices.empty()) continue;
                MeshBuilder::forEachCellChunk(parts[i], cell_, [&](MeshBuilder::CellChunk& ch) {
                    if (ch.mesh.vertices.empty()) return;
                    ch.mesh.materialIndex = parts[i].materialIndex;
                    pieces.push_back({{ch.cx, ch.cz}, flat, i, std::move(ch.mesh), {}});
                });
                parts[i] = RenderMesh();   // (let go as soon as it is cut)
            }
        };
        cut(tile.parts, false);
        cut(tile.flatParts, true);
        std::stable_sort(pieces.begin(), pieces.end(), [](const Piece& a, const Piece& b) {
            return std::tie(a.cell, a.flat, a.slot) < std::tie(b.cell, b.flat, b.slot);
        });
        parallelRows(static_cast<int>(pieces.size()), [&](int k) {
            Piece& pc = pieces[static_cast<std::size_t>(k)];
            BinWriter b; putPart(b, pc.mesh); pc.bytes = std::move(b.bytes); pc.mesh = RenderMesh();
        }, 1);
        for (const Piece& pc : pieces) {
            w_.add(cellPrefix(prefix_, pc.cell.first, pc.cell.second) + name + (pc.flat ? "/flat/" : "/parts/") + std::to_string(pc.slot), pc.bytes);
            cells.insert(pc.cell);
        }
    }
    nlohmann::json cj = nlohmann::json::array();
    for (const std::pair<int, int>& c : cells) { cj.push_back({c.first, c.second}); written_.insert(c); }
    tiles_.push_back({{"tz", tile.id.tz}, {"tx", tile.id.tx}, {"key", hexKey(tile.key)}, {"malls", tile.malls}, {"cells", cj},
                      {"counts", lotPlanCounts(tile.counts)}});
}

void TileCellWriter::finish(const NetLotResult& res) {
    size_t built = 0, units = 0; for (const LotBuilding& lb : res.lots) { built += lb.plan.size() >= 3; units += lb.units.size(); }
    { BinWriter b; putLots(b, res.lots); w_.add(prefix_ + "lots", b.bytes); }
    { BinWriter b; putLotPlan(b, res.plan); w_.add(prefix_ + "plan", b.bytes); }
    { BinWriter b; putFlattens(b, res.gradeFlatten); w_.add(prefix_ + "grade", b.bytes); }
    w_.addJson(prefix_ + "tiles", tiles_);
    nlohmann::json cellList = nlohmann::json::array(); for (const std::pair<int, int>& c : written_) cellList.push_back({c.first, c.second});
    w_.addJson(prefix_ + "meta", nlohmann::json{{"format", kLotsFormatVersion}, {"lots", res.lots.size()}, {"built", built}, {"units", units}, {"parts", res.parts.size()}, {"flatParts", res.flatParts.size()}, {"grades", res.gradeFlatten.size()}, {"planBlocks", res.plan.blocks.size()}, {"planLots", res.plan.lots.size()}, {"cell", cell_}, {"cells", cellList}, {"tile", kLotTile}});
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
        // (written tile by tile, ADR-0504: each tile's chunks under the cell, `<tz>_<tx>/parts/<i>`; LOD0 before LOD1)
        std::vector<LotCellPart> flat;
        for (const std::string& sec : b.sections(cp)) {
            const std::size_t slash = sec.rfind('/');
            if (slash == std::string::npos || slash == 0) continue;
            const std::size_t kindAt = sec.rfind('/', slash - 1);
            const std::string kind = sec.substr(kindAt + 1, slash - kindAt - 1);
            const int part = std::atoi(sec.c_str() + slash + 1);
            if (kind == "parts" && part >= 0 && static_cast<size_t>(part) < nParts) out.push_back({cx, cz, part, false, sec});
            else if (kind == "flat" && part >= 0 && static_cast<size_t>(part) < nFlat) flat.push_back({cx, cz, part, true, sec});
        }
        out.insert(out.end(), flat.begin(), flat.end());
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
        RenderMesh& dst = vec[static_cast<size_t>(cp.part)];
        dst.materialIndex = chunk.materialIndex;
        MeshBuilder::append(dst, chunk);
    }
    return true;
}

}  // namespace lotcache
}  // namespace engine
