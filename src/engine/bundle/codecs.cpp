#include "engine/bundle/codecs.h"

#include "engine/procgen/city/road_spec.h"

#include <cmath>

namespace engine {
namespace bundle {

namespace {
constexpr uint32_t kMeshVersion = 1, kGridVersion = 1, kRoadGraphVersion = 1, kRoadEntityVersion = 1, kRingsVersion = 1;
constexpr uint32_t kDeckVersion = 1, kBandVersion = 1;
inline float f(double v) { return static_cast<float>(v); }
}  // namespace

// ---- meshes -------------------------------------------------------------------------------------------

PackedMesh packMesh(const RenderMesh& m, const std::string& name, const Vec3& albedo, uint32_t flags, float roughness, double friction) {
    PackedMesh p; p.name = name; p.roughness = roughness; p.friction = friction;
    p.albedo[0] = f(albedo.x); p.albedo[1] = f(albedo.y); p.albedo[2] = f(albedo.z);
    const size_t n = m.vertices.size();
    p.pos.resize(3 * n); p.nrm.resize(3 * n); p.tan.resize(3 * n); p.uv.resize(2 * n);
    bool uniform = true; const Vec3 c0 = n ? m.vertices[0].color : Vec3(1, 1, 1);
    for (size_t i = 0; i < n; ++i) {
        const Vertex& v = m.vertices[i];
        p.pos[3 * i] = f(v.position.x); p.pos[3 * i + 1] = f(v.position.y); p.pos[3 * i + 2] = f(v.position.z);
        p.nrm[3 * i] = f(v.normal.x); p.nrm[3 * i + 1] = f(v.normal.y); p.nrm[3 * i + 2] = f(v.normal.z);
        p.tan[3 * i] = f(v.tangent.x); p.tan[3 * i + 1] = f(v.tangent.y); p.tan[3 * i + 2] = f(v.tangent.z);
        p.uv[2 * i] = v.u; p.uv[2 * i + 1] = v.v;
        if (uniform && (v.color.x != c0.x || v.color.y != c0.y || v.color.z != c0.z)) uniform = false;
    }
    p.flags = (flags & ~static_cast<uint32_t>(PackedMesh::kColorUniform)) | (uniform ? PackedMesh::kColorUniform : 0u);
    if (uniform) { p.vertexColor[0] = f(c0.x); p.vertexColor[1] = f(c0.y); p.vertexColor[2] = f(c0.z); }
    else { p.col.resize(3 * n); for (size_t i = 0; i < n; ++i) { const Vec3& c = m.vertices[i].color; p.col[3 * i] = f(c.x); p.col[3 * i + 1] = f(c.y); p.col[3 * i + 2] = f(c.z); } }
    p.idx = m.indices;
    return p;
}

RenderMesh unpackMesh(const PackedMesh& p) {
    RenderMesh m; const size_t n = p.vertexCount(); m.vertices.resize(n);
    const Vec3 uc(p.vertexColor[0], p.vertexColor[1], p.vertexColor[2]); const bool uniform = p.colorUniform() || p.col.size() != 3 * n;
    for (size_t i = 0; i < n; ++i) {
        Vertex& v = m.vertices[i];
        v.position = Vec3(p.pos[3 * i], p.pos[3 * i + 1], p.pos[3 * i + 2]);
        v.normal = p.nrm.size() == 3 * n ? Vec3(p.nrm[3 * i], p.nrm[3 * i + 1], p.nrm[3 * i + 2]) : Vec3(0, 1, 0);
        v.tangent = p.tan.size() == 3 * n ? Vec3(p.tan[3 * i], p.tan[3 * i + 1], p.tan[3 * i + 2]) : Vec3(0, 0, 0);
        v.u = p.uv.size() == 2 * n ? p.uv[2 * i] : 0.0f; v.v = p.uv.size() == 2 * n ? p.uv[2 * i + 1] : 0.0f;
        v.color = uniform ? uc : Vec3(p.col[3 * i], p.col[3 * i + 1], p.col[3 * i + 2]);
    }
    m.indices = p.idx; return m;
}

void colliderFromPacked(const PackedMesh& p, MeshCollider& mc) {
    const size_t n = p.vertexCount(); mc.vertices.resize(n);
    for (size_t i = 0; i < n; ++i) mc.vertices[i] = Vec3(p.pos[3 * i], p.pos[3 * i + 1], p.pos[3 * i + 2]);
    mc.indices = p.idx; mc.friction = p.friction;
}

void putPackedMesh(BinWriter& w, const PackedMesh& p) {
    w.magic("MESH", kMeshVersion); w.putStr(p.name); w.put<uint32_t>(p.flags);
    for (float c : p.albedo) w.put<float>(c); for (float c : p.vertexColor) w.put<float>(c);
    w.put<float>(p.roughness); w.put<double>(p.friction);
    w.putVec(p.pos); w.putVec(p.nrm); w.putVec(p.tan); w.putVec(p.uv); w.putVec(p.col); w.putVec(p.idx);
}

bool getPackedMesh(BinReader& r, PackedMesh& p) {
    uint32_t v = 0; if (!r.magic("MESH", &v) || v != kMeshVersion) return false;
    if (!r.getStr(p.name) || !r.get(p.flags)) return false;
    for (float& c : p.albedo) if (!r.get(c)) return false; for (float& c : p.vertexColor) if (!r.get(c)) return false;
    if (!r.get(p.roughness) || !r.get(p.friction)) return false;
    if (!r.getVec(p.pos) || !r.getVec(p.nrm) || !r.getVec(p.tan) || !r.getVec(p.uv) || !r.getVec(p.col) || !r.getVec(p.idx)) return false;
    const size_t n = p.pos.size() / 3;
    if (p.pos.size() != 3 * n || p.nrm.size() != 3 * n || p.tan.size() != 3 * n || p.uv.size() != 2 * n || (!p.col.empty() && p.col.size() != 3 * n)) return false;
    for (uint32_t i : p.idx) if (i >= n) return false;
    return r.ok();
}

// ---- height grid --------------------------------------------------------------------------------------

void putHeightGrid(BinWriter& w, const HeightGridBlob& g) {
    w.magic("HGRD", kGridVersion); w.put<double>(g.x0); w.put<double>(g.y0); w.put<double>(g.res); w.put<int32_t>(g.nx); w.put<int32_t>(g.ny); w.putVec(g.z);
}
bool getHeightGrid(BinReader& r, HeightGridBlob& g) {
    uint32_t v = 0; if (!r.magic("HGRD", &v) || v != kGridVersion) return false;
    if (!r.get(g.x0) || !r.get(g.y0) || !r.get(g.res) || !r.get(g.nx) || !r.get(g.ny) || !r.getVec(g.z)) return false;
    if (g.nx < 0 || g.ny < 0 || g.z.size() != static_cast<size_t>(g.nx) * static_cast<size_t>(g.ny)) return false;
    return r.ok();
}

// ---- road graphs --------------------------------------------------------------------------------------

namespace {
void putGraphBody(BinWriter& w, const RoadGraph& g) {
    w.put<uint32_t>(static_cast<uint32_t>(g.nodes.size()));
    for (const RoadNode& n : g.nodes) {
        w.put<double>(n.pos.x); w.put<double>(n.pos.y); w.put<double>(n.elev); w.put<uint8_t>(n.elevAbsolute ? 1 : 0);
        w.put<uint8_t>(static_cast<uint8_t>(n.kind)); w.put<double>(n.tangent.x); w.put<double>(n.tangent.y);
    }
    w.put<uint32_t>(static_cast<uint32_t>(g.edges.size()));
    for (const RoadEdge& e : g.edges) {
        w.put<int32_t>(e.a); w.put<int32_t>(e.b); w.put<double>(e.width); w.put<uint8_t>(static_cast<uint8_t>(e.klass)); w.put<int32_t>(e.layer);
        w.put<uint8_t>(static_cast<uint8_t>(e.provenance)); w.put<uint8_t>(e.oneWay ? 1 : 0); w.put<int32_t>(e.spec); w.put<uint8_t>(e.walkable ? 1 : 0);
        w.put<uint8_t>(e.access); w.put<double>(e.parkOffset); w.put<double>(e.parkWidth); w.put<uint8_t>(e.baked ? 1 : 0);
    }
    w.put<uint32_t>(static_cast<uint32_t>(g.specs.size()));
    for (const RoadSpec& s : g.specs) w.putStr(roadSpecToJson(s).dump());
}
bool getGraphBody(BinReader& r, RoadGraph& g) {
    uint32_t nn = 0; if (!r.get(nn)) return false; g.nodes.resize(nn);
    for (RoadNode& n : g.nodes) {
        uint8_t abs8 = 0, kind8 = 0;
        if (!r.get(n.pos.x) || !r.get(n.pos.y) || !r.get(n.elev) || !r.get(abs8) || !r.get(kind8) || !r.get(n.tangent.x) || !r.get(n.tangent.y)) return false;
        n.elevAbsolute = abs8 != 0; n.kind = static_cast<JunctionKind>(kind8);
    }
    uint32_t ne = 0; if (!r.get(ne)) return false; g.edges.resize(ne);
    for (RoadEdge& e : g.edges) {
        uint8_t klass8 = 0, prov8 = 0, one8 = 0, walk8 = 0, baked8 = 0;
        if (!r.get(e.a) || !r.get(e.b) || !r.get(e.width) || !r.get(klass8) || !r.get(e.layer) || !r.get(prov8) || !r.get(one8) || !r.get(e.spec) || !r.get(walk8)
            || !r.get(e.access) || !r.get(e.parkOffset) || !r.get(e.parkWidth) || !r.get(baked8)) return false;
        e.klass = static_cast<RoadClass>(klass8); e.provenance = static_cast<RoadProvenance>(prov8); e.oneWay = one8 != 0; e.walkable = walk8 != 0; e.baked = baked8 != 0;
        if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(nn) || e.b >= static_cast<int>(nn)) return false;
    }
    uint32_t ns = 0; if (!r.get(ns)) return false; g.specs.resize(ns);
    for (RoadSpec& s : g.specs) { std::string j; if (!r.getStr(j)) return false; try { s = roadSpecFromJson(nlohmann::json::parse(j)); } catch (const std::exception&) { return false; } }
    return r.ok();
}
}  // namespace

void putRoadGraph(BinWriter& w, const RoadGraph& g) { w.magic("RGRF", kRoadGraphVersion); putGraphBody(w, g); }
bool getRoadGraph(BinReader& r, RoadGraph& g) { uint32_t v = 0; if (!r.magic("RGRF", &v) || v != kRoadGraphVersion) return false; return getGraphBody(r, g); }

void putRoadEntity(BinWriter& w, const RoadEntity& e) {
    w.magic("RENT", kRoadEntityVersion); putGraphBody(w, e.graph);
    const RoadLook& l = e.look;
    w.put<double>(l.defaultWidth); w.put<double>(l.sidewalk); w.put<double>(l.curb); w.put<double>(l.cornerRadius); w.put<double>(l.lift);
    w.put<uint8_t>(l.markings ? 1 : 0); w.put<uint8_t>(l.crosswalks ? 1 : 0); w.put<uint8_t>(l.autoRoundabout ? 1 : 0); w.put<uint8_t>(l.perClassGrade ? 1 : 0);
    w.put<double>(l.color.x); w.put<double>(l.color.y); w.put<double>(l.color.z);
    w.put<uint32_t>(static_cast<uint32_t>(e.plan.cityHubs.size()));
    for (const CityHub& h : e.plan.cityHubs) { w.put<double>(h.pos.x); w.put<double>(h.pos.y); w.put<int32_t>(h.kind); w.put<uint8_t>(h.radial ? 1 : 0); w.put<int32_t>(h.site); }
    w.put<uint32_t>(static_cast<uint32_t>(e.plan.freewayPlans.size()));
    for (const std::vector<Vec2>& pl : e.plan.freewayPlans) { w.put<uint32_t>(static_cast<uint32_t>(pl.size())); for (const Vec2& q : pl) { w.put<double>(q.x); w.put<double>(q.y); } }
}

bool getRoadEntity(BinReader& r, RoadEntity& e) {
    uint32_t v = 0; if (!r.magic("RENT", &v) || v != kRoadEntityVersion) return false;
    if (!getGraphBody(r, e.graph)) return false;
    RoadLook& l = e.look; uint8_t m8 = 0, c8 = 0, a8 = 0, p8 = 0;
    if (!r.get(l.defaultWidth) || !r.get(l.sidewalk) || !r.get(l.curb) || !r.get(l.cornerRadius) || !r.get(l.lift) || !r.get(m8) || !r.get(c8) || !r.get(a8) || !r.get(p8)
        || !r.get(l.color.x) || !r.get(l.color.y) || !r.get(l.color.z)) return false;
    l.markings = m8 != 0; l.crosswalks = c8 != 0; l.autoRoundabout = a8 != 0; l.perClassGrade = p8 != 0;
    uint32_t nh = 0; if (!r.get(nh)) return false; e.plan.cityHubs.resize(nh);
    for (CityHub& h : e.plan.cityHubs) { uint8_t rad8 = 0; if (!r.get(h.pos.x) || !r.get(h.pos.y) || !r.get(h.kind) || !r.get(rad8) || !r.get(h.site)) return false; h.radial = rad8 != 0; }
    uint32_t np = 0; if (!r.get(np)) return false; e.plan.freewayPlans.resize(np);
    for (std::vector<Vec2>& pl : e.plan.freewayPlans) { uint32_t n = 0; if (!r.get(n)) return false; pl.resize(n); for (Vec2& q : pl) if (!r.get(q.x) || !r.get(q.y)) return false; }
    return r.ok();
}

// ---- rings --------------------------------------------------------------------------------------------

void putRings(BinWriter& w, const std::vector<std::vector<Vec2>>& rings) {
    w.magic("RING", kRingsVersion); w.put<uint32_t>(static_cast<uint32_t>(rings.size()));
    for (const std::vector<Vec2>& ring : rings) { w.put<uint32_t>(static_cast<uint32_t>(ring.size())); for (const Vec2& q : ring) { w.put<double>(q.x); w.put<double>(q.y); } }
}
bool getRings(BinReader& r, std::vector<std::vector<Vec2>>& rings) {
    uint32_t v = 0; if (!r.magic("RING", &v) || v != kRingsVersion) return false;
    uint32_t n = 0; if (!r.get(n)) return false; rings.resize(n);
    for (std::vector<Vec2>& ring : rings) { uint32_t k = 0; if (!r.get(k)) return false; ring.resize(k); for (Vec2& q : ring) if (!r.get(q.x) || !r.get(q.y)) return false; }
    return r.ok();
}

// --- the deck and the kerb band -------------------------------------------------------------------
//
// The lattice hands its deck straight to the loader because it meshes at load. A city built through
// the bundle has to carry the same surface across, or everything the sim stands ON a road — moving
// traffic, parked cars, painted bays, signal poles, crosswalk decals — falls back to the terrain
// beside it and sinks into the asphalt (metro_lanes, measured: mean -0.067 m, worst -0.53 m).

namespace {
void putVec2s(BinWriter& w, const std::vector<Vec2>& v) {
    w.put<uint32_t>(static_cast<uint32_t>(v.size()));
    for (const Vec2& q : v) { w.put<double>(q.x); w.put<double>(q.y); }
}
bool getVec2s(BinReader& r, std::vector<Vec2>& v) {
    uint32_t n = 0; if (!r.get(n)) return false; v.resize(n);
    for (Vec2& q : v) if (!r.get(q.x) || !r.get(q.y)) return false;
    return true;
}
void putDoubles(BinWriter& w, const std::vector<double>& v) {
    w.put<uint32_t>(static_cast<uint32_t>(v.size()));
    for (double d : v) w.put<double>(d);
}
bool getDoubles(BinReader& r, std::vector<double>& v) {
    uint32_t n = 0; if (!r.get(n)) return false; v.resize(n);
    for (double& d : v) if (!r.get(d)) return false;
    return true;
}
}  // namespace

void putDeckField(BinWriter& w, const RoadDeckField& d) {
    w.magic("DECK", kDeckVersion);
    w.put<uint32_t>(static_cast<uint32_t>(d.spines.size()));
    for (const UnionSpine& s : d.spines) {
        putVec2s(w, s.points);
        putDoubles(w, s.yAbs);
        putDoubles(w, s.hw);
        putDoubles(w, s.crossSlope);
        w.put<double>(s.halfWidth);
        w.put<double>(s.travelEdgeFrac);
        w.put<uint8_t>(static_cast<uint8_t>(s.closed));
        w.put<uint8_t>(static_cast<uint8_t>(s.klass));
        w.put<uint8_t>(s.access);
        w.put<uint8_t>(s.accessBack);
        w.put<uint8_t>(static_cast<uint8_t>(s.authoredDeck));
        w.put<int32_t>(static_cast<int32_t>(s.layer));
    }
    w.put<uint32_t>(static_cast<uint32_t>(d.pads.size()));
    for (const RoadDeckField::Tri& t : d.pads) {
        const Vec3 v3[3] = {t.a, t.b, t.c};
        for (const Vec3& p : v3) { w.put<double>(p.x); w.put<double>(p.y); w.put<double>(p.z); }
    }
}

bool getDeckField(BinReader& r, RoadDeckField& d) {
    uint32_t v = 0; if (!r.magic("DECK", &v) || v != kDeckVersion) return false;
    uint32_t n = 0; if (!r.get(n)) return false; d.spines.assign(n, UnionSpine{});
    for (UnionSpine& s : d.spines) {
        if (!getVec2s(r, s.points) || !getDoubles(r, s.yAbs) || !getDoubles(r, s.hw) ||
            !getDoubles(r, s.crossSlope))
            return false;
        uint8_t closed = 0, klass = 0, access = 0, accessBack = 0, authored = 0;
        int32_t layer = 0;
        if (!r.get(s.halfWidth) || !r.get(s.travelEdgeFrac) || !r.get(closed) || !r.get(klass) ||
            !r.get(access) || !r.get(accessBack) || !r.get(authored) || !r.get(layer))
            return false;
        s.closed = closed != 0;
        s.klass = static_cast<RoadClass>(klass);
        s.access = access;
        s.accessBack = accessBack;
        s.authoredDeck = authored != 0;
        s.layer = layer;
    }
    if (!r.get(n)) return false; d.pads.assign(n, RoadDeckField::Tri{});
    for (RoadDeckField::Tri& t : d.pads) {
        Vec3* v3[3] = {&t.a, &t.b, &t.c};
        for (Vec3* p : v3) if (!r.get(p->x) || !r.get(p->y) || !r.get(p->z)) return false;
    }
    if (!r.ok()) return false;
    d.buildIndex();            // the field arrives ready to query, as the mesher's does
    return true;
}

void putCurbBands(BinWriter& w, const CurbBandAudit& b) {
    w.magic("KERB", kBandVersion);
    putRings(w, b.loops);
    w.put<uint32_t>(static_cast<uint32_t>(b.mouthGaps.size()));
    for (const auto& g : b.mouthGaps) {
        w.put<double>(g.first.x); w.put<double>(g.first.y);
        w.put<double>(g.second.x); w.put<double>(g.second.y);
    }
    putVec2s(w, b.junctions);
    w.put<uint32_t>(static_cast<uint32_t>(b.junctionDegree.size()));
    for (int deg : b.junctionDegree) w.put<int32_t>(static_cast<int32_t>(deg));
    putDoubles(w, b.junctionMinAngle);
    w.put<double>(b.sidewalkWidth);
    w.put<double>(b.curbHeight);
}

bool getCurbBands(BinReader& r, CurbBandAudit& b) {
    uint32_t v = 0; if (!r.magic("KERB", &v) || v != kBandVersion) return false;
    if (!getRings(r, b.loops)) return false;
    uint32_t n = 0; if (!r.get(n)) return false; b.mouthGaps.resize(n);
    for (auto& g : b.mouthGaps)
        if (!r.get(g.first.x) || !r.get(g.first.y) || !r.get(g.second.x) || !r.get(g.second.y))
            return false;
    if (!getVec2s(r, b.junctions)) return false;
    if (!r.get(n)) return false; b.junctionDegree.resize(n);
    for (int& deg : b.junctionDegree) { int32_t k = 0; if (!r.get(k)) return false; deg = k; }
    if (!getDoubles(r, b.junctionMinAngle)) return false;
    if (!r.get(b.sidewalkWidth) || !r.get(b.curbHeight)) return false;
    return r.ok();
}

}  // namespace bundle
}  // namespace engine
