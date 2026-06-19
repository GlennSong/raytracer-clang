#include "shape_grammar.h"

#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

// Deterministic per-building RNG (seeded; ADR-0002). Small xorshift — we only
// need cheap, reproducible jitter, not statistical quality.
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x9e3779b9u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    Real unit() { return (next() >> 8) * (1.0 / 16777216.0); }   // [0,1)
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
};

int materialIndexFor(PartId id) { return static_cast<int>(id); }

// Indexed access to a Vec3 component (avoids pointer arithmetic across members,
// which is UB under -Wpedantic).
Real axisComp(const Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }
void setAxisComp(Vec3& v, int i, Real val) {
    if (i == 0) v.x = val; else if (i == 1) v.y = val; else v.z = val;
}

// A rectangle on a building face: bottom-left corner + in-plane axes + outward
// normal. The facade subdivision works entirely in this 2D frame.
struct FaceRect {
    Vec3 bl;            // bottom-left corner (world)
    Vec3 h;             // unit horizontal axis
    Vec3 v;             // unit vertical axis (== scope up)
    Vec3 n;             // outward normal
    Real width = 0;
    Real height = 0;

    Vec3 at(Real x, Real y) const { return bl + h * x + v * y; }
};

// The four vertical faces of a storey scope, outward-facing.
FaceRect faceOf(const Scope& s, int side) {
    const Vec3 r = s.axis[0], u = s.axis[1], f = s.axis[2];
    const Real W = s.size.x, H = s.size.y, D = s.size.z;
    const Vec3 O = s.origin;
    FaceRect fr;
    fr.v = u; fr.height = H;
    switch (side) {
        case 0: fr.n = f;      fr.bl = O + f * D;          fr.h = r;       fr.width = W; break; // front
        case 1: fr.n = f * -1; fr.bl = O + r * W;          fr.h = r * -1;  fr.width = W; break; // back
        case 2: fr.n = r;      fr.bl = O + r * W + f * D;  fr.h = f * -1;  fr.width = D; break; // right
        default: fr.n = r * -1; fr.bl = O;                 fr.h = f;       fr.width = D; break; // left
    }
    return fr;
}

}  // namespace

RenderMaterial materialFor(PartId id, const Vec3& wallColor) {
    RenderMaterial m;
    switch (id) {
        case PartId::Glass:
            m.albedo = {0.18, 0.27, 0.34}; m.metallic = 0.9f; m.roughness = 0.08f; break;
        case PartId::Trim:
            m.albedo = wallColor * 0.55; m.metallic = 0.0f; m.roughness = 0.7f; break;
        case PartId::Roof:
            m.albedo = {0.18, 0.18, 0.20}; m.metallic = 0.0f; m.roughness = 0.85f; break;
        case PartId::Door:
            m.albedo = {0.12, 0.12, 0.13}; m.metallic = 0.3f; m.roughness = 0.4f; break;
        case PartId::Ground:
            m.albedo = {0.30, 0.30, 0.32}; m.metallic = 0.0f; m.roughness = 0.9f; break;
        case PartId::Detail:
            m.albedo = wallColor * 0.8; m.metallic = 0.1f; m.roughness = 0.6f; break;
        case PartId::Wall:
        default:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.75f; break;
    }
    return m;
}

void emitQuad(RenderMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
              const Vec3& d, const Vec3& normal, const Vec3& color) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 tan = normalize(b - a);
    auto vtx = [&](const Vec3& p, float u, float vv) {
        Vertex v(p, normal, tan, u, vv);
        v.color = color;
        return v;
    };
    mesh.vertices.push_back(vtx(a, 0, 0));
    mesh.vertices.push_back(vtx(b, 1, 0));
    mesh.vertices.push_back(vtx(c, 1, 1));
    mesh.vertices.push_back(vtx(d, 0, 1));
    // Orient the two triangles so their geometric normal agrees with `normal`.
    Vec3 geo = cross(b - a, c - a);
    if (dot(geo, normal) >= 0)
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    else
        mesh.indices.insert(mesh.indices.end(), {base, base + 2, base + 1, base, base + 3, base + 2});
}

// Append a part's geometry, creating the part lazily and keeping materialIndex.
// NOTE: the returned reference is invalidated by any later partMesh() that grows
// out.parts — use it immediately, never hold it across another partMesh() call.
static RenderMesh& partMesh(BuildingMesh& out, PartId id) {
    for (RenderMesh& p : out.parts)
        if (p.materialIndex == materialIndexFor(id)) return p;
    out.parts.emplace_back();
    RenderMesh& p = out.parts.back();
    p.materialIndex = materialIndexFor(id);
    return p;
}

// Append a whole mesh's geometry into a part (offsetting indices). Safe to call
// in sequence — the part reference is used only for this one append.
static void appendToPart(BuildingMesh& out, PartId id, const RenderMesh& src) {
    if (src.vertices.empty()) return;
    RenderMesh& dst = partMesh(out, id);
    uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
}

void emitBox(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color) {
    RenderMesh& mesh = partMesh(out, part);
    // Eight corners.
    Vec3 c000 = s.corner(0, 0, 0), c100 = s.corner(1, 0, 0);
    Vec3 c110 = s.corner(1, 1, 0), c010 = s.corner(0, 1, 0);
    Vec3 c001 = s.corner(0, 0, 1), c101 = s.corner(1, 0, 1);
    Vec3 c111 = s.corner(1, 1, 1), c011 = s.corner(0, 1, 1);
    const Vec3 r = s.axis[0], u = s.axis[1], f = s.axis[2];
    emitQuad(mesh, c000, c100, c110, c010, f * -1, color);   // back  (-f)
    emitQuad(mesh, c001, c101, c111, c011, f,      color);   // front (+f)
    emitQuad(mesh, c000, c001, c011, c010, r * -1, color);   // left  (-r)
    emitQuad(mesh, c100, c101, c111, c110, r,      color);   // right (+r)
    emitQuad(mesh, c000, c100, c101, c001, u * -1, color);   // bottom(-u)
    emitQuad(mesh, c010, c110, c111, c011, u,      color);   // top   (+u)
}

void emitShell(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color,
               bool floor, bool ceiling) {
    RenderMesh& mesh = partMesh(out, part);
    for (int side = 0; side < 4; ++side) {
        FaceRect fr = faceOf(s, side);
        emitQuad(mesh, fr.at(0, 0), fr.at(fr.width, 0), fr.at(fr.width, fr.height),
                 fr.at(0, fr.height), fr.n, color);
    }
    const Vec3 u = s.axis[1];
    if (floor)
        emitQuad(mesh, s.corner(0, 0, 0), s.corner(1, 0, 0), s.corner(1, 0, 1),
                 s.corner(0, 0, 1), u, color);
    if (ceiling)
        emitQuad(mesh, s.corner(0, 1, 0), s.corner(1, 1, 0), s.corner(1, 1, 1),
                 s.corner(0, 1, 1), u * -1, color);
}

std::vector<Scope> splitScope(const Scope& s, int axis, const std::vector<Real>& sizes) {
    std::vector<Scope> out;
    Real total = 0;
    for (Real sz : sizes) total += std::abs(sz);
    Real extent = axisComp(s.size, axis);
    Real cursor = 0;
    for (Real sz : sizes) {
        Real len = (total > 0) ? std::abs(sz) / total * extent : 0;
        Scope child = s;
        child.origin = s.origin + s.axis[axis] * cursor;
        setAxisComp(child.size, axis, len);
        out.push_back(child);
        cursor += len;
    }
    return out;
}

std::vector<Scope> repeatScope(const Scope& s, int axis, Real target) {
    Real extent = axisComp(s.size, axis);
    int n = std::max(1, static_cast<int>(std::lround(extent / std::max(target, Real(0.01)))));
    std::vector<Real> sizes(static_cast<std::size_t>(n), 1.0);
    return splitScope(s, axis, sizes);
}

Scope insetScope(const Scope& s, Real d) {
    Scope o = s;
    Real dx = std::min(d, s.size.x * 0.49);
    Real dz = std::min(d, s.size.z * 0.49);
    o.origin = s.origin + s.axis[0] * dx + s.axis[2] * dz;
    o.size.x = s.size.x - 2 * dx;
    o.size.z = s.size.z - 2 * dz;
    return o;
}

Scope scopeFromFootprint(const Poly2& footprint, Real baseY, Real height) {
    OBB2 obb = orientedBoundingBox(footprint);
    Scope s;
    // Long OBB axis -> forward (facades face the long sides); short -> right.
    int la = obb.longAxis(), sa = 1 - la;
    Vec2 fwd = obb.axis[la], rgt = obb.axis[sa];
    Real fwdHalf = obb.half[la], rgtHalf = obb.half[sa];
    s.axis[0] = normalize(Vec3(rgt.x, 0, rgt.y));
    s.axis[1] = Vec3(0, 1, 0);
    s.axis[2] = normalize(Vec3(fwd.x, 0, fwd.y));
    s.size = Vec3(rgtHalf * 2, height, fwdHalf * 2);
    Vec3 centerXZ(obb.center.x, baseY, obb.center.y);
    s.origin = centerXZ - s.axis[0] * rgtHalf - s.axis[2] * fwdHalf;
    return s;
}

// --- Facade subdivision -----------------------------------------------------

namespace {

enum class FacadeMode { Residential, Retail, Entrance };

// Subdivide one face into window bays. Wall margins around each window read as
// mullions/piers; the window is recessed by `inset` (Glass). In Entrance mode the
// centre bay is a real door-height opening (no fill) so the shell is enterable.
void emitFacade(BuildingMesh& out, const Scope& storey, int side, FacadeMode mode,
                const BuildingParams& p, const Vec3& wallColor) {
    FaceRect fr = faceOf(storey, side);
    // Accumulate into locals, then append once each — never hold a part reference
    // across a partMesh() that could reallocate out.parts.
    RenderMesh wall, glass, door;

    int bays = std::max(1, static_cast<int>(std::lround(fr.width / std::max(p.bayWidth, Real(0.5)))));
    Real bw = fr.width / bays;
    Real fh = fr.height;

    Real sill = (mode == FacadeMode::Retail) ? 0.4 : human::WINDOW_SILL;
    Real head = std::min(fh - 0.4, (mode == FacadeMode::Retail) ? fh - 0.4 : human::WINDOW_HEAD);
    Real margin = std::min(bw * 0.22, Real(0.6));
    if (head <= sill) { head = fh * 0.75; sill = fh * 0.2; }

    int centreBay = bays / 2;
    for (int b = 0; b < bays; ++b) {
        Real x0 = b * bw, x1 = (b + 1) * bw;
        bool entrance = (mode == FacadeMode::Entrance && b == centreBay);

        Real wx0 = x0 + margin, wx1 = x1 - margin;          // window/opening span
        Real openSill = entrance ? 0.0 : sill;
        Real openHead = entrance ? std::min(human::DOOR_HEIGHT, fh - 0.3) : head;
        if (entrance) {
            Real dw = std::min(human::DOOR_WIDTH, bw - 0.4);
            Real cx = (x0 + x1) * 0.5;
            wx0 = cx - dw * 0.5; wx1 = cx + dw * 0.5;
        }

        // Wall surround: bottom band, top band, left pier, right pier.
        auto wallQuad = [&](Real a0, Real a1, Real b0, Real b1) {
            if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
            emitQuad(wall, fr.at(a0, b0), fr.at(a1, b0), fr.at(a1, b1), fr.at(a0, b1),
                     fr.n, wallColor);
        };
        wallQuad(x0, x1, 0, openSill);                       // below opening
        wallQuad(x0, x1, openHead, fh);                      // above opening
        wallQuad(x0, wx0, openSill, openHead);               // left pier
        wallQuad(wx1, x1, openSill, openHead);               // right pier

        if (entrance) {
            // A real opening: a recessed dark threshold so it reads as a doorway
            // you can pass through (ADR-0038 §4).
            Vec3 in = fr.n * -0.15;
            emitQuad(door, fr.at(wx0, 0) + in, fr.at(wx1, 0) + in,
                     fr.at(wx1, openHead) + in, fr.at(wx0, openHead) + in, fr.n,
                     materialFor(PartId::Door, wallColor).albedo);
        } else {
            Vec3 in = fr.n * (-p.windowInset);
            emitQuad(glass, fr.at(wx0, openSill) + in, fr.at(wx1, openSill) + in,
                     fr.at(wx1, openHead) + in, fr.at(wx0, openHead) + in, fr.n,
                     materialFor(PartId::Glass, wallColor).albedo);
        }
    }

    appendToPart(out, PartId::Wall, wall);
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::Door, door);
}

}  // namespace

BuildingMesh growBuilding(const Scope& scope, const BuildingParams& params) {
    BuildingMesh out;
    Rng rng(params.seed);

    const Real baseY = scope.origin.y;
    const Vec3 wallColor =
        params.wallColor * (0.92 + 0.08 * rng.unit());      // slight per-building tint

    // Running footprint (origin XZ + extents along right/forward); shrinks at setbacks.
    Vec3 footOrigin = scope.origin;
    Real width = scope.size.x, depth = scope.size.z;
    const Vec3 r = scope.axis[0], f = scope.axis[2];

    auto storeyScope = [&](Real y, Real h) {
        Scope s;
        s.axis[0] = r; s.axis[1] = Vec3(0, 1, 0); s.axis[2] = f;
        s.origin = Vec3(footOrigin.x, y, footOrigin.z);
        s.size = Vec3(width, h, depth);
        return s;
    };

    Real y = baseY;

    // Ground floor: taller, glassy retail (or lobby), walkable shell with a real
    // entrance on the front face (ADR-0038 §4).
    Real gh = params.groundHeight;
    Scope ground = storeyScope(y, gh);
    FacadeMode groundMode = params.groundRetail ? FacadeMode::Retail : FacadeMode::Residential;
    for (int side = 0; side < 4; ++side) {
        FacadeMode mode = (side == 0 && params.walkableGround) ? FacadeMode::Entrance : groundMode;
        emitFacade(out, ground, side, mode, params, wallColor);
    }
    // Ground slab you can stand on.
    emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                       {r, Vec3(0, 1, 0), f}, Vec3(width, 0.1, depth)},
            PartId::Ground, materialFor(PartId::Ground, wallColor).albedo);
    {
        FaceRect ef = faceOf(ground, 0);
        out.attaches.push_back({ef.at(ef.width * 0.5, 0), ef.n, "entrance"});
    }
    y += gh;

    // Upper residential floors, with optional setbacks.
    for (int i = 0; i < params.floors; ++i) {
        if (params.setbackFloors > 0 && params.setbackEvery > 0 && i > 0 &&
            i % params.setbackFloors == 0) {
            Real d = params.setbackEvery;
            Real dx = std::min(d, width * 0.4), dz = std::min(d, depth * 0.4);
            footOrigin = footOrigin + r * dx + f * dz;
            width -= 2 * dx; depth -= 2 * dz;
        }
        Real fh = params.floorHeight;
        Scope storey = storeyScope(y, fh);
        for (int side = 0; side < 4; ++side)
            emitFacade(out, storey, side, FacadeMode::Residential, params, wallColor);
        if (i == params.floors / 2) {
            FaceRect ff = faceOf(storey, 0);
            out.attaches.push_back({ff.at(ff.width * 0.5, fh * 0.5), ff.n, "facade"});
        }
        y += fh;
    }

    // Roof: a flat slab + a parapet railing around the perimeter (ADR-0038 §4).
    emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                       {r, Vec3(0, 1, 0), f}, Vec3(width, 0.2, depth)},
            PartId::Roof, materialFor(PartId::Roof, wallColor).albedo);
    if (params.parapet > 0) {
        Scope para = storeyScope(y, params.parapet);
        emitShell(out, para, PartId::Trim, materialFor(PartId::Trim, wallColor).albedo,
                  false, false);
    }
    out.attaches.push_back({Vec3(footOrigin.x, y, footOrigin.z) +
                                r * (width * 0.5) + f * (depth * 0.5),
                            Vec3(0, 1, 0), "roof"});

    out.height = (y + params.parapet) - baseY;

    // Coarse proxy mass (single box, ground footprint to roof) for HLOD/impostor
    // baking (ADR-0038 §6). Built into a scratch BuildingMesh so it stays separate
    // from the detailed parts.
    {
        BuildingMesh scratch;
        emitBox(scratch, Scope{scope.origin, {scope.axis[0], Vec3(0, 1, 0), scope.axis[2]},
                               Vec3(scope.size.x, out.height, scope.size.z)},
                PartId::Wall, wallColor);
        if (!scratch.parts.empty()) out.proxy = scratch.parts.front();
    }

    return out;
}

RenderMesh BuildingMesh::merged() const {
    RenderMesh m;
    for (const RenderMesh& p : parts) MeshBuilder::append(m, p);
    return m;
}

}  // namespace engine
