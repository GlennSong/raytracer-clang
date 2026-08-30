#include "shape_grammar.h"

#include "road_mesh.h"            // triangulatePolygon (floorplan roof/slab fill)
#include "triangulate.h"          // triangulateWithHoles (interior ceilings, ADR-0080)
#include "../surface_maps.h"      // surfaceWorldTileSize (shingle slope UVs)
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

Vec3 facadeColor(FacadeStyle style, uint32_t seed) {
    Rng rng(seed ? seed : 1u);
    Real t = rng.unit();
    switch (style) {
        case FacadeStyle::Brick:
            // Warm reds/browns, some buff.
            return lerp(Vec3(0.50, 0.22, 0.16), Vec3(0.62, 0.40, 0.28), t);
        case FacadeStyle::Stucco:
            return lerp(Vec3(0.86, 0.82, 0.72), Vec3(0.80, 0.74, 0.60), t);
        case FacadeStyle::Painted:
            // Muted pastels (residential).
            return lerp(Vec3(0.74, 0.78, 0.78), Vec3(0.80, 0.72, 0.66), t) +
                   Vec3(rng.range(-0.04, 0.04), rng.range(-0.04, 0.04), rng.range(-0.04, 0.04));
        case FacadeStyle::GlassCurtain:
            return lerp(Vec3(0.58, 0.62, 0.66), Vec3(0.66, 0.68, 0.70), t);
        case FacadeStyle::Metal:
            // Cool steel / corrugated siding (industrial).
            return lerp(Vec3(0.46, 0.50, 0.54), Vec3(0.56, 0.58, 0.60), t);
        case FacadeStyle::Wood: {
            // Painted wood siding: a small swatch book of house paints —
            // whites, sages, blue-greys, butter yellows, barn reds.
            static const Vec3 kSwatch[] = {
                {0.88, 0.86, 0.80},   // farmhouse white
                {0.68, 0.74, 0.66},   // sage green
                {0.58, 0.66, 0.74},   // coastal blue-grey
                {0.84, 0.76, 0.52},   // butter yellow
                {0.60, 0.34, 0.28},   // barn red
                {0.74, 0.68, 0.58},   // driftwood tan
            };
            const Vec3 base = kSwatch[rng.next() % 6];
            return base + Vec3(rng.range(-0.03, 0.03), rng.range(-0.03, 0.03),
                               rng.range(-0.03, 0.03));
        }
        case FacadeStyle::DarkBrick:
            // Deep browns to charcoal reds (lofts, factories, dark towers).
            return lerp(Vec3(0.26, 0.14, 0.11), Vec3(0.38, 0.22, 0.18), t);
        case FacadeStyle::Sandstone:
            // Warm buff / honey ashlar (banks, museums, deco masonry).
            return lerp(Vec3(0.78, 0.66, 0.48), Vec3(0.86, 0.76, 0.58), t);
        case FacadeStyle::Concrete:
        default:
            return lerp(Vec3(0.62, 0.62, 0.60), Vec3(0.74, 0.73, 0.70), t);
    }
}

// Deterministic per-opening night-light coin flip (WS3): hash the opening's
// QUANTIZED world position, so the full, flat (LOD1) and curtain-wall emitters
// all agree on which windows glow — an LOD swap never flickers a window on or
// off — and rebuilds of the same city light the same homes. ~1/3 lit: enough
// that every block reads inhabited, sparse enough to stay night.
static uint32_t positionHash(const Vec3& worldPos) {
    const int32_t qx = static_cast<int32_t>(std::floor(worldPos.x * 2.0));
    const int32_t qy = static_cast<int32_t>(std::floor(worldPos.y * 2.0));
    const int32_t qz = static_cast<int32_t>(std::floor(worldPos.z * 2.0));
    uint32_t h = static_cast<uint32_t>(qx) * 0x8da6b343u ^
                 static_cast<uint32_t>(qy) * 0xd8163841u ^
                 static_cast<uint32_t>(qz) * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x9e3779b1u;
    h ^= h >> 16;
    return h;
}

bool litWindow(const Vec3& worldPos) {
    return (positionHash(worldPos) & 0xffu) < 85;   // ~exactly 1/3
}

Real litStoreyOccupancy(const Vec3& storeyAnchor) {
    // A different byte of the same hash than litWindow reads, so a storey's
    // occupancy and its first bay's coin are independent.
    const uint32_t h = (positionHash(storeyAnchor) >> 8) & 0xffffu;
    return 0.12 + 0.50 * (h / 65535.0);
}

bool litOfficeBay(const Vec3& bayAnchor, Real occupancy) {
    return (positionHash(bayAnchor) & 0xffu) / 255.0 < occupancy;
}

RenderMaterial materialFor(PartId id, const Vec3& wallColor) {
    RenderMaterial m;
    switch (id) {
        case PartId::Glass:
            m.albedo = {0.18, 0.27, 0.34}; m.metallic = 0.9f; m.roughness = 0.08f; break;
        case PartId::GlassLit:
            // Indistinguishable from Glass by DAY — the lit third of the
            // windows must not read as a checkerboard at noon. Night is the
            // loader's NightGlow tag raising emission, not this material.
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
        case PartId::Brick:
            // A wall, but shaded with a world-space procedural material from the
            // library. Albedo stays the wall colour (which rides in vertex colour
            // for the merged city mesh); the shader/tracer add the surface detail.
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.88f;
            m.setSurface(RenderMaterial::Surface::Brick); break;
        case PartId::Concrete:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.92f;
            m.setSurface(RenderMaterial::Surface::Concrete); break;
        case PartId::Interior:
            // Bare interior surfaces of enterable buildings (ADR-0080). The
            // shader is sun-only (no local lights, TECH_DEBT), so a roofed
            // room is sun-blind by design; a faint self-light -- emission =
            // albedo x 0.10 -- keeps it dim concrete instead of black, and
            // the loader tags interior chunks NightGlow for after dusk.
            m.albedo = {0.58, 0.57, 0.55}; m.metallic = 0.0f; m.roughness = 0.92f;
            m.setSurface(RenderMaterial::Surface::Concrete);
            m.emission = m.albedo * 0.10;
            break;
        case PartId::Stucco:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.85f;
            m.setSurface(RenderMaterial::Surface::Stucco); break;
        case PartId::Metal:
            m.albedo = wallColor; m.metallic = 0.55f; m.roughness = 0.45f;
            m.setSurface(RenderMaterial::Surface::CorrugatedMetal); break;
        case PartId::Wood:
            m.albedo = {0.52, 0.40, 0.27}; m.metallic = 0.0f; m.roughness = 0.85f;
            m.setSurface(RenderMaterial::Surface::WoodSiding); break;
        case PartId::Vent:
            // HVAC intake: baked maps carry the punched holes' albedo/rough/
            // metal split, so the base stays neutral white.
            m.albedo = {1, 1, 1}; m.metallic = 0.9f; m.roughness = 0.4f;
            m.setSurface(RenderMaterial::Surface::VentGrille); break;
        case PartId::Utility:
            m.albedo = {1, 1, 1}; m.metallic = 0.9f; m.roughness = 0.45f;
            m.setSurface(RenderMaterial::Surface::UtilityPanel); break;
        case PartId::Fan:
            m.albedo = {1, 1, 1}; m.metallic = 0.75f; m.roughness = 0.5f;
            m.setSurface(RenderMaterial::Surface::FanTop); break;
        case PartId::Shingle:
            // Pitched-roof slopes: the shingle bake carries the tone + course
            // relief (normal map); the per-building roof tint rides in vertex
            // colour, so the base stays neutral white. UVs are slope-fitted in
            // the grammar (u along the eave, v up the slope, world metres /
            // tile) — the loader must NOT re-UV this part world-planar.
            m.albedo = {1, 1, 1}; m.metallic = 0.0f; m.roughness = 0.92f;
            m.setSurface(RenderMaterial::Surface::RoofShingle); break;
        case PartId::Siding:
            // Painted siding: the paint colour rides in vertex colour (like
            // Brick/Stucco); the WoodSiding surface adds the board detail.
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.80f;
            m.setSurface(RenderMaterial::Surface::WoodSiding); break;
        case PartId::Path:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.95f;
            m.setSurface(RenderMaterial::Surface::Pavement); break;
        case PartId::Foliage:
            // Hedges/planters: colour rides in vertex colour, no surface.
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.95f; break;
        case PartId::Wall:
        default:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.75f; break;
    }
    return m;
}

// The shape grammar's quad emitter is the engine's winding-aware MeshBuilder
// helper (kept as a free function here so the grammar's many call sites read
// tersely). Centralising the winding rule means the grammar, terrain and roads
// can't drift apart on which way a front face points.
void emitQuad(RenderMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
              const Vec3& d, const Vec3& normal, const Vec3& color) {
    MeshBuilder::emitQuad(mesh, a, b, c, d, normal, color);
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

// A solid parapet: a closed ring of four thin boxes around a footprint
// perimeter, so the roof edge reads as a real wall with thickness from every
// angle (emitShell gives single-sided quads that vanish edge-on / from inside).
// `footOrigin` is the min corner, extents `width` (along r) × `depth` (along f),
// rising `height` from `y`, wall `thick` metres.
void emitParapet(BuildingMesh& out, const Vec3& footOrigin, Real width, Real depth,
                 const Vec3& r, const Vec3& f, Real y, Real height, Real thick,
                 PartId part, const Vec3& color) {
    const Vec3 up(0, 1, 0);
    Real t = std::min(thick, std::min(width, depth) * 0.45);
    Vec3 base(footOrigin.x, y, footOrigin.z);
    // Back (-f) and front (+f) run the full width; left/right fit between them.
    emitBox(out, Scope{base, {r, up, f}, Vec3(width, height, t)}, part, color);
    emitBox(out, Scope{base + f * (depth - t), {r, up, f}, Vec3(width, height, t)},
            part, color);
    emitBox(out, Scope{base + f * t, {r, up, f}, Vec3(t, height, depth - 2 * t)},
            part, color);
    emitBox(out, Scope{base + r * (width - t) + f * t, {r, up, f},
                       Vec3(t, height, depth - 2 * t)}, part, color);
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

Scope scopeFromFootprint(const Poly2& footprint, Real baseY, Real height,
                         const std::function<bool(const Vec2&)>& cornerOk) {
    OBB2 obb = orientedBoundingBox(footprint);
    // Fit the box INSIDE the footprint: a non-rectangular lot (wedge/trapezoid or
    // corner piece) has an OBB that bulges past the polygon, which would seat the
    // building proud of its lot. Shrink about an interior anchor until all four
    // corners sit inside, so rectangular lots keep full size and skew lots pull in.
    // `cornerOk` folds in the caller's extra constraint (road clearance).
    Vec2 anchor = pointInPolygon(footprint, obb.center) ? obb.center
                                                        : centroid(footprint);
    auto cornersInside = [&](Real s) {
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2) {
                Vec2 c = anchor + obb.axis[0] * (obb.half[0] * s * sx)
                                + obb.axis[1] * (obb.half[1] * s * sy);
                if (!pointInPolygon(footprint, c)) return false;
                if (cornerOk && !cornerOk(c)) return false;
            }
        return true;
    };
    Real fit = 1.0;
    if (!cornersInside(1.0)) {
        Real lo = 0.0, hi = 1.0;
        for (int it = 0; it < 16; ++it) {
            Real mid = (lo + hi) * 0.5;
            (cornersInside(mid) ? lo : hi) = mid;
        }
        fit = lo;
    }
    Scope s;
    // Long OBB axis -> forward (facades face the long sides); short -> right.
    int la = obb.longAxis(), sa = 1 - la;
    Vec2 fwd = obb.axis[la], rgt = obb.axis[sa];
    Real fwdHalf = obb.half[la] * fit, rgtHalf = obb.half[sa] * fit;
    s.axis[0] = normalize(Vec3(rgt.x, 0, rgt.y));
    s.axis[1] = Vec3(0, 1, 0);
    s.axis[2] = normalize(Vec3(fwd.x, 0, fwd.y));
    s.size = Vec3(rgtHalf * 2, height, fwdHalf * 2);
    Vec3 centerXZ(anchor.x, baseY, anchor.y);
    s.origin = centerXZ - s.axis[0] * rgtHalf - s.axis[2] * fwdHalf;
    return s;
}

// --- Facade subdivision -----------------------------------------------------

namespace {

enum class FacadeMode { Residential, Retail, Entrance, Solid };

// A curtain-wall storey (ADR-0040 Pass B): a continuous glass skin, not punched
// windows — an opaque spandrel band hiding the floor slab, vision glass above,
// and a proud steel mullion/transom grid. This is what a glass tower actually
// is (a skin hung off a frame), and it reads far better than flat panels.
void emitCurtainWallRect(BuildingMesh& out, const FaceRect& fr,
                         const Vec3& wallColor,
                         FacadeDetail detail = FacadeDetail::Full) {
    Real fh = fr.height, W = fr.width;
    if (W < 0.5 || fh < 0.5) return;
    RenderMesh glass, glassLit, mull;
    Vec3 glassCol = materialFor(PartId::Glass, wallColor).albedo;
    Vec3 spandrelCol = glassCol * 0.45;          // opaque shadow-box band
    Vec3 mullCol(0.34, 0.36, 0.40);              // steel mullions
    Real spandrelH = std::min(Real(0.9), fh * 0.30);

    // Glass sits INSET behind the frame plane; the mullion grid is SOLID
    // geometry — front face + side returns back to the glass — so up close it
    // reads as a frame the panes sit in, not a decal (device feedback).
    const Real glassIn = 0.10;                   // glass plane behind the grid
    Vec3 gin = fr.n * (-glassIn);
    emitQuad(glass, fr.at(0, 0) + gin, fr.at(W, 0) + gin,
             fr.at(W, spandrelH) + gin, fr.at(0, spandrelH) + gin,
             fr.n, spandrelCol);                 // spandrel (floor-slab band)
    // Vision glass, per BAY (device: "the way it's lit up row by row is
    // odd"). One pane per mullion bay — the same grid the lattice below
    // draws, so a lit cell sits inside a real frame — each lit or dark by the
    // position hash at its own centre, against a per-STOREY occupancy that
    // makes some floors busy and others nearly dark. The first cut lit the
    // whole storey-face from one hash at x = 0: every lit floor was a
    // full-width band (`curtain_wall_lights_vary_within_a_storey`). The
    // spandrel band stays dark. Flat (LOD1) runs this same loop, so the two
    // detail levels light the same offices.
    const int bays = std::max(1, static_cast<int>(std::lround(W / 1.6)));
    const Real occupancy = litStoreyOccupancy(fr.at(0, spandrelH));
    for (int b = 0; b < bays; ++b) {
        const Real x0 = W * b / bays, x1 = W * (b + 1) / bays;
        RenderMesh& vision =
            litOfficeBay(fr.at((x0 + x1) * 0.5, spandrelH), occupancy) ? glassLit : glass;
        emitQuad(vision, fr.at(x0, spandrelH) + gin, fr.at(x1, spandrelH) + gin,
                 fr.at(x1, fh) + gin, fr.at(x0, fh) + gin, fr.n, glassCol);
    }
    // FLAT (LOD1): the spandrel band + vision pane carry the curtain-wall read
    // at distance; the solid mullion lattice is the expensive half — skip it.
    if (detail == FacadeDetail::Flat) {
        appendToPart(out, PartId::Glass, glass);
        appendToPart(out, PartId::GlassLit, glassLit);
        return;
    }

    const Real proud = 0.06;                     // grid stands proud of the wall
    Vec3 outv = fr.n * proud;
    const Real mw = 0.09;
    // A solid BAR on the facade: front face + both side returns down to the
    // glass plane (vertical bars get left/right cheeks, horizontal get top/
    // bottom), so the lattice has real depth from any angle.
    auto bar = [&](Real a0, Real b0, Real a1, Real b1, bool vertical) {
        emitQuad(mull, fr.at(a0, b0) + outv, fr.at(a1, b0) + outv,
                 fr.at(a1, b1) + outv, fr.at(a0, b1) + outv, fr.n, mullCol);
        if (vertical) {
            emitQuad(mull, fr.at(a0, b0) + gin, fr.at(a0, b0) + outv,
                     fr.at(a0, b1) + outv, fr.at(a0, b1) + gin, fr.h * -1, mullCol);
            emitQuad(mull, fr.at(a1, b0) + gin, fr.at(a1, b0) + outv,
                     fr.at(a1, b1) + outv, fr.at(a1, b1) + gin, fr.h, mullCol);
        } else {
            emitQuad(mull, fr.at(a0, b1) + gin, fr.at(a1, b1) + gin,
                     fr.at(a1, b1) + outv, fr.at(a0, b1) + outv, fr.v, mullCol);
            emitQuad(mull, fr.at(a0, b0) + outv, fr.at(a1, b0) + outv,
                     fr.at(a1, b0) + gin, fr.at(a0, b0) + gin, fr.v * -1, mullCol);
        }
    };
    for (int b = 0; b <= bays; ++b) {            // vertical mullions (the pane grid)
        Real x = std::min(std::max(b * W / bays, mw * 0.5), W - mw * 0.5);
        bar(x - mw * 0.5, 0, x + mw * 0.5, fh, true);
    }
    for (Real ty : {spandrelH, fh - 0.04}) {     // transoms (spandrel line + head)
        Real t0 = std::max(Real(0), ty - mw * 0.5), t1 = std::min(fh, ty + mw * 0.5);
        bar(0, t0, W, t1, false);
    }
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::GlassLit, glassLit);
    appendToPart(out, PartId::Detail, mull);     // mullions read as metal detail
}
void emitCurtainWall(BuildingMesh& out, const Scope& storey, int side,
                     const Vec3& wallColor) {
    emitCurtainWallRect(out, faceOf(storey, side), wallColor);
}

// Subdivide one face into window bays. Wall margins around each window read as
// mullions/piers; the window is recessed by `inset` (Glass). In Entrance mode the
// centre bay is a real door-height opening (no fill) so the shell is enterable.
// Works on a bare FaceRect so BOTH massing paths share it: the box grammar
// (faceOf a storey scope) and the floorplan grammar (one rect per plan edge).
// The facade LAYOUT: the bay grid plus every opening's span and sill/head, as
// the splitter decides them — computed ONCE and consumed by BOTH the full
// emitter and the flat LOD1 emitter (city-render-perf R2), so two detail
// levels can never disagree about where a window or the door sits. This is the
// blueprint model's first in-engine step (lot-system-plan §15.2): an opening
// is a span along the wall plus a sill/head, decided before any geometry.
struct BayOpening {
    Real x0 = 0, x1 = 0;      // the bay's span along the face
    Real wx0 = 0, wx1 = 0;    // the opening's span
    Real sill = 0, head = 0;  // vertical extent (arches rise inside this box)
    bool entrance = false;    // this opening is the door
};
struct FacadeLayout {
    int bays = 1;
    Real bw = 0;
    bool retailish = false;
    std::vector<BayOpening> open;
};

static FacadeLayout facadeLayout(const FaceRect& fr, FacadeMode mode,
                                 const BuildingParams& p) {
    FacadeLayout L;
    L.bays = std::max(1, static_cast<int>(std::lround(fr.width / std::max(p.bayWidth, Real(0.5)))));
    L.bw = fr.width / L.bays;
    const Real bw = L.bw;
    const Real fh = fr.height;

    Real sill, head, margin;
    // The entrance face's non-door bays match the OTHER ground faces (retail
    // storefronts when groundRetail) — the front used to wear small residential
    // windows while the other three sides had tall shopfronts (device feedback).
    L.retailish = (mode == FacadeMode::Retail) ||
                  (mode == FacadeMode::Entrance && p.groundRetail);
    if (mode == FacadeMode::Solid) {           // warehouse: small high clerestory
        sill = fh * 0.66; head = fh * 0.84; margin = std::min(bw * 0.36, Real(1.4));
    } else {
        sill = L.retailish ? 0.4 : human::WINDOW_SILL;
        head = std::min(fh - 0.4, L.retailish ? fh - 0.4 : human::WINDOW_HEAD);
        // A CONSTANT window module across every face (the piers absorb the slack),
        // so a wide face and a narrow one show the same window size, not different
        // ones (ADR-0040). Width is the bay minus piers, clamped PORTRAIT: the
        // opening is 1.5 m tall (sill->head), so the width cap stays under it —
        // windows read taller than wide (device feedback). Arched heads go
        // narrower still: the classic French window is tall and slim, and the
        // slimmer span also steepens the arc so the arch reads.
        const bool arched = mode == FacadeMode::Residential &&
                            p.window.head != OpeningStyle::Head::Flat;
        Real winW = std::min(arched ? Real(1.05) : Real(1.25),
                             std::max(Real(0.8), bw - 0.8));
        margin = (bw - winW) * 0.5;
    }
    if (head <= sill) { head = fh * 0.75; sill = fh * 0.2; }

    const int centreBay = L.bays / 2;
    for (int b = 0; b < L.bays; ++b) {
        BayOpening o;
        o.x0 = b * bw; o.x1 = (b + 1) * bw;
        o.entrance = (mode == FacadeMode::Entrance && b == centreBay);
        o.wx0 = o.x0 + margin; o.wx1 = o.x1 - margin;       // window/opening span
        o.sill = o.entrance ? 0.0 : sill;
        o.head = o.entrance ? std::min(human::DOOR_HEIGHT, fh - 0.3) : head;
        if (o.entrance) {
            Real dw = std::min(human::DOOR_WIDTH, bw - 0.4);
            Real cx = (o.x0 + o.x1) * 0.5;
            o.wx0 = cx - dw * 0.5; o.wx1 = cx + dw * 0.5;
        }
        L.open.push_back(o);
    }
    return L;
}

// The FLAT emitter (LOD1, city-render-perf R2): the same layout, the cheapest
// honest drawing of it — one quad per wall, one flat pane per opening riding
// The INNER face of an exterior ground-storey wall (enterable buildings,
// ADR-0080): the SAME FacadeLayout as the outside, drawn at -thick facing the
// room, so from inside you see wall around every opening instead of the sky
// through a one-sided skin. Non-door openings also get an inward-facing pane
// (the same lit/dark hash as the outside pane) so windows read as glass, not
// holes; the door bay's aperture stays open -- its reveal is the passage.
void emitInnerWallRect(BuildingMesh& out, const FaceRect& fr,
                       const FacadeLayout& L, Real thick,
                       const Vec3& wallColor) {
    RenderMesh wall, glass, glassLit;
    const Vec3 in = fr.n * -thick;
    const Vec3 nIn = fr.n * -1.0;
    const Vec3 icol = materialFor(PartId::Interior, wallColor).albedo;
    auto q = [&](RenderMesh& m, Real a0, Real b0, Real a1, Real b1,
                 const Vec3& col, const Vec3& off) {
        if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
        emitQuad(m, fr.at(a0, b0) + off, fr.at(a1, b0) + off,
                 fr.at(a1, b1) + off, fr.at(a0, b1) + off, nIn, col);
    };
    for (const BayOpening& o : L.open) {
        q(wall, o.x0, 0, o.wx0, fr.height, icol, in);         // left pier
        q(wall, o.wx1, 0, o.x1, fr.height, icol, in);         // right pier
        q(wall, o.wx0, 0, o.wx1, o.sill, icol, in);           // apron
        q(wall, o.wx0, o.head, o.wx1, fr.height, icol, in);   // over the head
        if (!o.entrance) {
            const bool lit = litWindow(fr.at(o.wx0, o.sill));
            q(lit ? glassLit : glass, o.wx0, o.sill, o.wx1, o.head,
              materialFor(lit ? PartId::GlassLit : PartId::Glass, wallColor)
                  .albedo,
              in + fr.n * 0.02);
        }
    }
    appendToPart(out, PartId::Interior, wall);
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::GlassLit, glassLit);
}

// 2 cm proud (no reveal, no z-fight), the door as a dark quad. No surrounds,
// frames, muntins, sills, hoods, pilasters. ~2 triangles per opening instead
// of ~40; the wall is 2 instead of ~10 per bay.
static void emitFlatFacadeRect(BuildingMesh& out, const FaceRect& fr, FacadeMode mode,
                               const BuildingParams& p, const Vec3& wallColor) {
    RenderMesh wall, glass, glassLit, door;
    emitQuad(wall, fr.at(0, 0), fr.at(fr.width, 0),
             fr.at(fr.width, fr.height), fr.at(0, fr.height), fr.n, wallColor);
    const Vec3 proud = fr.n * 0.02;
    const Vec3 gcol = materialFor(PartId::Glass, wallColor).albedo;
    const Vec3 dcol = materialFor(PartId::Door, wallColor).albedo;
    for (const BayOpening& o : facadeLayout(fr, mode, p).open) {
        // Same anchor as the full emitter's pane (fr.at(wx0, sill)), so a
        // window keeps its lit/dark choice across the LOD swap.
        RenderMesh& dst =
            o.entrance ? door
                       : (litWindow(fr.at(o.wx0, o.sill)) ? glassLit : glass);
        emitQuad(dst, fr.at(o.wx0, o.sill) + proud, fr.at(o.wx1, o.sill) + proud,
                 fr.at(o.wx1, o.head) + proud, fr.at(o.wx0, o.head) + proud,
                 fr.n, o.entrance ? dcol : gcol);
    }
    appendToPart(out, p.wallPart, wall);
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::GlassLit, glassLit);
    appendToPart(out, PartId::Door, door);
}

void emitFacadeRect(BuildingMesh& out, const FaceRect& fr, FacadeMode mode,
                    const BuildingParams& p, const Vec3& wallColor) {
    // Accumulate into locals, then append once each — never hold a part reference
    // across a partMesh() that could reallocate out.parts.
    // surround = sill/hood trim courses; frame = window frames + muntin lights.
    RenderMesh wall, glass, glassLit, door, surround, frame;

    // The splitter's decisions come from the SHARED layout (see facadeLayout):
    // this function only decides how much detail to draw them with.
    const FacadeLayout L = facadeLayout(fr, mode, p);
    const int bays = L.bays;
    const Real bw = L.bw;
    const Real fh = fr.height;
    const bool retailish = L.retailish;

    for (const BayOpening& bay : L.open) {
        const Real x0 = bay.x0, x1 = bay.x1;
        const bool entrance = bay.entrance;
        const Real wx0 = bay.wx0, wx1 = bay.wx1;
        const Real openSill = bay.sill;
        const Real openHead = bay.head;

        // Wall surround: bottom band, top band, left pier, right pier.
        auto wallQuad = [&](Real a0, Real a1, Real b0, Real b1) {
            if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
            emitQuad(wall, fr.at(a0, b0), fr.at(a1, b0), fr.at(a1, b1), fr.at(a0, b1),
                     fr.n, wallColor);
        };
        // The opening ELEMENT (building-grammar-plan.md P2): its head may be an
        // ARCH — a real arc cut into the wall, not a square hole. Arches apply
        // to punched residential windows only (storefronts, clerestories and
        // doors stay flat) and only when the opening can carry the rise.
        OpeningStyle st = p.window;
        if (entrance || mode != FacadeMode::Residential)
            st.head = OpeningStyle::Head::Flat;
        const Real span = wx1 - wx0;
        Real rise = 0;
        if (st.head == OpeningStyle::Head::Round) rise = span * 0.5;
        else if (st.head == OpeningStyle::Head::Segmental)
            rise = span * std::min(Real(0.5), std::max(Real(0.12), st.archRise));
        if (rise > 0 && openHead - rise < openSill + 0.35) rise = 0;   // too squat
        const Real ysp = openHead - rise;                    // springline
        const Real cx = (wx0 + wx1) * 0.5;
        // Arc samples, left springer -> right springer (face space).
        const int NARC = 8;
        Real R = 0, Cy = 0;
        Vec2 arc[NARC + 1];
        if (rise > 0) {
            R = (rise * rise + span * span * 0.25) / (2 * rise);
            Cy = openHead - R;
            const Real thL = std::atan2(ysp - Cy, wx0 - cx);
            const Real thR = std::atan2(ysp - Cy, wx1 - cx);
            for (int k = 0; k <= NARC; ++k) {
                Real th = thL + (thR - thL) * (Real(k) / NARC);
                arc[k] = Vec2(cx + R * std::cos(th), Cy + R * std::sin(th));
            }
        }

        // Wall surround: below, piers to the springline, above the apex — and
        // for an arch, the SPANDRELS between the arc and the apex line.
        wallQuad(x0, x1, 0, openSill);                       // below opening
        wallQuad(x0, x1, openHead, fh);                      // above apex
        wallQuad(x0, wx0, openSill, ysp);                    // left pier
        wallQuad(wx1, x1, openSill, ysp);                    // right pier
        if (rise > 0) {
            wallQuad(x0, wx0, ysp, openHead);                // pier strips beside the arch
            wallQuad(wx1, x1, ysp, openHead);
            for (int k = 0; k < NARC; ++k) {                 // spandrel fill over the arc
                // At the APEX one arc point touches the openHead line — a quad
                // there has a degenerate first triangle, which breaks emitQuad's
                // winding pick (the zero-area cross can't vote). Emit the
                // surviving piece as a triangle instead.
                const Real h0 = openHead - arc[k].y, h1 = openHead - arc[k + 1].y;
                Vec3 A = fr.at(arc[k].x, arc[k].y), B = fr.at(arc[k + 1].x, arc[k + 1].y);
                Vec3 TB = fr.at(arc[k + 1].x, openHead), TA = fr.at(arc[k].x, openHead);
                if (h0 < 1e-4 && h1 < 1e-4) continue;
                if (h1 < 1e-4)      MeshBuilder::emitTri(wall, A, B, TA, fr.n, wallColor);
                else if (h0 < 1e-4) MeshBuilder::emitTri(wall, A, B, TB, fr.n, wallColor);
                else                emitQuad(wall, A, B, TB, TA, fr.n, wallColor);
            }
        }

        if (entrance) {
            // The DOOR element: a recessed doorway. Closed like the windows —
            // jambs + lintel + threshold connect the wall opening back to the
            // door leaf, so you can't see through the gap into the hollow
            // shell. With openDoorway (enterable buildings, ADR-0080) the
            // leaf and frame are DROPPED and the reveal deepens to
            // wallThickness: the aperture is a real hole through a real wall
            // section, and the interior shell behind it closes the views.
            const Real revealDepth = p.openDoorway ? p.wallThickness : 0.18;
            Vec3 in = fr.n * -revealDepth;
            Vec3 oBL = fr.at(wx0, 0), oBR = fr.at(wx1, 0);
            Vec3 oTL = fr.at(wx0, openHead), oTR = fr.at(wx1, openHead);
            Vec3 dBL = oBL + in, dBR = oBR + in, dTL = oTL + in, dTR = oTR + in;
            Vec3 rev = wallColor * 0.7;
            emitQuad(wall, oTL, oTR, dTR, dTL, fr.v * -1, rev);   // lintel (faces down)
            emitQuad(wall, oBL, oBR, dBR, dBL, fr.v, rev);        // threshold (faces up)
            emitQuad(wall, oBL, oTL, dTL, dBL, fr.h, rev);        // left jamb
            emitQuad(wall, oBR, oTR, dTR, dBR, fr.h * -1, rev);   // right jamb
            if (!p.openDoorway)
                emitQuad(door, dBL, dBR, dTR, dTL, fr.n,
                         materialFor(PartId::Door, wallColor).albedo);  // leaf
            // DOORFRAME (device feedback): painted stiles + head rail seated in
            // the recess around the leaf — the same joinery the windows wear.
            if (!p.openDoorway) {
                const Vec3 fp = fr.n * -0.09;
                const Real dfw = 0.10;
                auto dfQuad = [&](Real a0, Real b0, Real a1, Real b1) {
                    if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
                    emitQuad(frame, fr.at(a0, b0) + fp, fr.at(a1, b0) + fp,
                             fr.at(a1, b1) + fp, fr.at(a0, b1) + fp,
                             fr.n, p.window.frameColor);
                };
                dfQuad(wx0, 0, wx0 + dfw, openHead);              // left stile
                dfQuad(wx1 - dfw, 0, wx1, openHead);              // right stile
                dfQuad(wx0 + dfw, openHead - dfw, wx1 - dfw, openHead);   // head rail
            }
            // ARCHITRAVE: a proud trim surround on the wall face around the
            // opening — jamb casings + a head band.
            {
                RenderMesh& srd = surround;
                auto caseQuad = [&](Real a0, Real b0, Real a1, Real b1) {
                    Vec3 ov = fr.n * 0.05;
                    emitQuad(srd, fr.at(a0, b0) + ov, fr.at(a1, b0) + ov,
                             fr.at(a1, b1) + ov, fr.at(a0, b1) + ov, fr.n, p.trimColor);
                };
                const Real cw = 0.12;
                caseQuad(wx0 - cw, 0, wx0, openHead + cw);        // left casing
                caseQuad(wx1, 0, wx1 + cw, openHead + cw);        // right casing
                caseQuad(wx0, openHead, wx1, openHead + cw);      // head band
            }
            // AWNING over the DOOR (device: it was centred on the face, not the
            // door — it belongs to the door grammar): a projecting ledge just
            // above the opening, spanning a little wider than the leaf.
            if (p.awning) {
                const Real aw = std::min((wx1 - wx0) + 1.2, fr.width - 0.4);
                const Real ac = (wx0 + wx1) * 0.5;
                Vec3 c0 = fr.at(ac - aw * 0.5, openHead + 0.22);
                Vec3 across = normalize(fr.h);
                Scope a;
                a.axis[0] = across; a.axis[1] = Vec3(0, 1, 0); a.axis[2] = fr.n;
                a.size = Vec3(aw, 0.16, 1.25);
                a.origin = c0;
                emitBox(out, a, PartId::Detail, p.trimColor);
            }
            // The entrance attach point sits at the DOOR's foot (not the face
            // centre — off by half a bay on even bay counts), and carries the
            // aperture: the lot layer turns it into a DoorSpec for colliders,
            // records and the leaf.
            out.attaches.push_back({fr.at((wx0 + wx1) * 0.5, 0), fr.n,
                                    "entrance", wx1 - wx0, openHead});
        } else {
            const Vec3 in = fr.n * (-p.windowInset);
            const Vec3 rev = wallColor * 0.82;
            const Vec3 gcol = materialFor(PartId::Glass, wallColor).albedo;
            // Reveals: close the recess between the wall opening and the inset
            // glass — sill, jambs to the springline, then a flat lintel or the
            // arc SOFFIT (per-segment quads whose normals point at the arc
            // centre, so the underside of the arch shades correctly).
            Vec3 oBL = fr.at(wx0, openSill), oBR = fr.at(wx1, openSill);
            Vec3 oTL = fr.at(wx0, ysp), oTR = fr.at(wx1, ysp);
            emitQuad(wall, oBL, oBR, oBR + in, oBL + in, fr.v, rev);      // sill reveal
            emitQuad(wall, oBL, oTL, oTL + in, oBL + in, fr.h, rev);      // left jamb
            emitQuad(wall, oBR, oTR, oTR + in, oBR + in, fr.h * -1, rev); // right jamb
            if (rise <= 0) {
                emitQuad(wall, oTL, oTR, oTR + in, oTL + in, fr.v * -1, rev);   // lintel
            } else {
                for (int k = 0; k < NARC; ++k) {
                    Vec3 A = fr.at(arc[k].x, arc[k].y), B = fr.at(arc[k + 1].x, arc[k + 1].y);
                    Real mx = (arc[k].x + arc[k + 1].x) * 0.5;
                    Real my = (arc[k].y + arc[k + 1].y) * 0.5;
                    Vec3 nrm = normalize(fr.h * (cx - mx) + fr.v * (Cy - my));
                    emitQuad(wall, A, B, B + in, A + in, nrm, rev);       // arc soffit
                }
            }

            // FRAME: a painted border seated partway into the reveal — the glass
            // sits INSIDE it (device: "a frame around it and then in that frame
            // sits the actual window"). Rails + stiles, and on an arch a curved
            // head rail following the arc. Muntins split the frame into lights.
            const Vec3 fp = fr.n * (-p.windowInset * 0.45);
            const Real fw = std::max(Real(0.04), st.frameWidth);
            auto frameQuad = [&](Real a0, Real b0, Real a1, Real b1) {
                if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
                emitQuad(frame, fr.at(a0, b0) + fp, fr.at(a1, b0) + fp,
                         fr.at(a1, b1) + fp, fr.at(a0, b1) + fp, fr.n, st.frameColor);
            };
            frameQuad(wx0, openSill, wx1, openSill + fw);                 // bottom rail
            frameQuad(wx0, openSill + fw, wx0 + fw, ysp);                 // left stile
            frameQuad(wx1 - fw, openSill + fw, wx1, ysp);                 // right stile
            if (rise <= 0) {
                frameQuad(wx0 + fw, ysp - fw, wx1 - fw, ysp);             // top rail
            } else {
                const Real innerK = (R - fw) / R;
                for (int k = 0; k < NARC; ++k) {                          // curved head rail
                    Vec2 iA(cx + (arc[k].x - cx) * innerK, Cy + (arc[k].y - Cy) * innerK);
                    Vec2 iB(cx + (arc[k + 1].x - cx) * innerK, Cy + (arc[k + 1].y - Cy) * innerK);
                    emitQuad(frame, fr.at(iA.x, iA.y) + fp, fr.at(iB.x, iB.y) + fp,
                             fr.at(arc[k + 1].x, arc[k + 1].y) + fp,
                             fr.at(arc[k].x, arc[k].y) + fp, fr.n, st.frameColor);
                }
            }
            // Muntins: the pane grid. On an arch, a TRANSOM bar crosses at the
            // springline (sash below, lunette above — the classic layout) and
            // the vertical muntins CONTINUE into the lunette up to the arc
            // itself (device: "the vertical doesn't reach the top of the arch").
            const Real mw = 0.032;
            const Real pz0 = openSill + fw;
            const Real pz1 = (rise > 0) ? ysp : ysp - fw;
            if (rise > 0) frameQuad(wx0, ysp - 0.045, wx1, ysp + 0.02);   // transom
            for (int k = 1; k < st.lightsX; ++k) {
                Real xk = wx0 + span * (Real(k) / st.lightsX);
                frameQuad(xk - mw, pz0, xk + mw, pz1);
                if (rise > 0) {
                    // Up into the lunette: stop at the arc above this x.
                    Real dx = xk - cx;
                    Real yArc = Cy + std::sqrt(std::max(Real(0), R * R - dx * dx));
                    frameQuad(xk - mw, ysp, xk + mw, yArc - 0.01);
                }
            }
            for (int k = 1; k < st.lightsY; ++k) {
                Real yk = pz0 + (pz1 - pz0) * (Real(k) / st.lightsY);
                frameQuad(wx0 + fw, yk - mw, wx1 - fw, yk + mw);
            }

            // GLASS: the full opening behind the frame — a rectangle to the
            // springline plus (for an arch) a lunette fan to the arc. A third
            // of the panes route to GlassLit (litWindow — same anchor the flat
            // emitter hashes, so LOD swaps keep the same homes lit).
            RenderMesh& pane =
                litWindow(fr.at(wx0, openSill)) ? glassLit : glass;
            emitQuad(pane, oBL + in, oBR + in, oTR + in, oTL + in, fr.n, gcol);
            if (rise > 0) {
                Vec3 S = fr.at(cx, ysp) + in;
                for (int k = 0; k < NARC; ++k)
                    MeshBuilder::emitTri(pane, S, fr.at(arc[k].x, arc[k].y) + in,
                                         fr.at(arc[k + 1].x, arc[k + 1].y) + in,
                                         fr.n, gcol);
            }

            // Surrounds (Trim): the projecting SILL course, and a HOOD — a flat
            // header band, or a voussoir band that FOLLOWS the arch (device:
            // "window arches ... should follow the arch to look natural").
            if (!p.curtainWall && mode != FacadeMode::Solid) {
                RenderMesh& srd = surround;
                auto ledge = [&](Real a0, Real b0, Real a1, Real b1, Real proud) {
                    Vec3 ov = fr.n * proud;
                    Vec3 BL = fr.at(a0, b0), BR = fr.at(a1, b0);
                    Vec3 TL = fr.at(a0, b1), TR = fr.at(a1, b1);
                    emitQuad(srd, BL + ov, BR + ov, TR + ov, TL + ov, fr.n, p.trimColor);
                    emitQuad(srd, TL, TR, TR + ov, TL + ov, fr.v, p.trimColor);        // top ledge
                    emitQuad(srd, BL + ov, BR + ov, BR, BL, fr.v * -1, p.trimColor);   // underside
                    emitQuad(srd, BL, BL + ov, TL + ov, TL, fr.h * -1, p.trimColor);   // left end
                    emitQuad(srd, BR + ov, BR, TR, TR + ov, fr.h, p.trimColor);        // right end
                };
                const Real over = 0.14;                        // oversail past the jambs
                if (st.sill)
                    ledge(std::max(x0, wx0 - over), openSill - 0.13,
                          std::min(x1, wx1 + over), openSill, 0.16);      // sill course
                if (st.hood == OpeningStyle::Hood::Band && rise <= 0)
                    ledge(std::max(x0, wx0 - over), openHead,
                          std::min(x1, wx1 + over), openHead + 0.12, 0.06);   // header
                if (st.hood == OpeningStyle::Hood::Arch && rise > 0) {
                    // Voussoir band: a proud arc course from the extrados out.
                    const Real bw = 0.15, proud = 0.07;
                    const Vec3 hp = fr.n * proud;
                    const Real outerK = (R + bw) / R;
                    auto outerAt = [&](int k) {
                        return Vec2(cx + (arc[k].x - cx) * outerK,
                                    Cy + (arc[k].y - Cy) * outerK);
                    };
                    for (int k = 0; k < NARC; ++k) {
                        Vec3 A = fr.at(arc[k].x, arc[k].y);
                        Vec3 B = fr.at(arc[k + 1].x, arc[k + 1].y);
                        Vec2 oA = outerAt(k), oB = outerAt(k + 1);
                        Vec3 OA = fr.at(oA.x, oA.y), OB = fr.at(oB.x, oB.y);
                        emitQuad(srd, A + hp, B + hp, OB + hp, OA + hp, fr.n, p.trimColor);
                        Real mx = (arc[k].x + arc[k + 1].x) * 0.5;
                        Real my = (arc[k].y + arc[k + 1].y) * 0.5;
                        Vec3 dn = normalize(fr.h * (cx - mx) + fr.v * (Cy - my));
                        emitQuad(srd, A, B, B + hp, A + hp, dn, p.trimColor);      // intrados lip
                        emitQuad(srd, OA + hp, OB + hp, OB, OA, dn * -1, p.trimColor); // extrados
                    }
                    // End CAPS at the springers (device: "the arches ... are
                    // missing the end caps"): close the band's cut face where it
                    // dies onto the wall, on both sides.
                    for (int e : {0, NARC}) {
                        Vec3 A = fr.at(arc[e].x, arc[e].y);
                        Vec2 oE = outerAt(e);
                        Vec3 OE = fr.at(oE.x, oE.y);
                        emitQuad(srd, A, OE, OE + hp, A + hp, fr.v * -1, p.trimColor);
                    }
                }
            }
        }
    }

    // Pilasters: thin vertical piers proud of the wall at each bay boundary. Per
    // storey they stack into continuous full-height pillars. A box projecting
    // outward by `proud`, under Trim.
    if (p.pilasters && mode != FacadeMode::Entrance) {
        RenderMesh trim;
        Real proud = 0.18, pw = 0.5;
        Vec3 up = fr.v;
        for (int b = 0; b <= bays; ++b) {
            Real x = std::min(std::max(b * bw, pw * 0.5), fr.width - pw * 0.5);
            Vec3 c0 = fr.at(x, 0), c1 = fr.at(x, fr.height);
            Vec3 along = normalize(fr.h) * (pw * 0.5);
            Vec3 outv = fr.n * proud;
            // Front quad + two returns of the pier (a shallow box face).
            emitQuad(trim, c0 - along + outv, c0 + along + outv,
                     c1 + along + outv, c1 - along + outv, fr.n, p.trimColor);
            emitQuad(trim, c0 + along, c0 + along + outv, c1 + along + outv,
                     c1 + along, fr.h, p.trimColor);
            emitQuad(trim, c0 - along + outv, c0 - along, c1 - along,
                     c1 - along + outv, fr.h * -1, p.trimColor);
            (void)up;
        }
        appendToPart(out, PartId::Trim, trim);
    }

    // The wall surface goes to the building's chosen facade part (procedural
    // brick/concrete/stucco/metal, or the flat Wall).
    appendToPart(out, p.wallPart, wall);
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::GlassLit, glassLit);
    appendToPart(out, PartId::Door, door);
    appendToPart(out, PartId::Trim, surround);
    appendToPart(out, PartId::Detail, frame);   // frames/muntins read as joinery
}

// The box-grammar wrapper: pick the storey scope's face, then share the
// element machinery above with the floorplan path.
void emitFacade(BuildingMesh& out, const Scope& storey, int side, FacadeMode mode,
                const BuildingParams& p, const Vec3& wallColor,
                FacadeDetail detail = FacadeDetail::Full) {
    if (detail == FacadeDetail::Full)
        emitFacadeRect(out, faceOf(storey, side), mode, p, wallColor);
    else
        emitFlatFacadeRect(out, faceOf(storey, side), mode, p, wallColor);
}

}  // namespace

// --- Curved (cylindrical) tower -------------------------------------------

// A vertical tube (cylinder wall) y0..y1 at `radius`, `sides` facets, outward
// normals, into part `pid` with vertex colour `col`.
static void emitTube(BuildingMesh& out, const Vec3& cXZ, Real radius, Real y0,
                     Real y1, int sides, PartId pid, const Vec3& col) {
    RenderMesh m;
    for (int i = 0; i < sides; ++i) {
        Real a0 = 2 * PI * i / sides, a1 = 2 * PI * (i + 1) / sides;
        Vec3 d0(std::cos(a0), 0, std::sin(a0)), d1(std::cos(a1), 0, std::sin(a1));
        Vec3 n = normalize(d0 + d1);
        emitQuad(m, cXZ + d0 * radius + Vec3(0, y0, 0),
                    cXZ + d1 * radius + Vec3(0, y0, 0),
                    cXZ + d1 * radius + Vec3(0, y1, 0),
                    cXZ + d0 * radius + Vec3(0, y1, 0), n, col);
    }
    appendToPart(out, pid, m);
}

// A horizontal disc cap at height y (normal up or down).
static void emitDisc(BuildingMesh& out, const Vec3& cXZ, Real radius, Real y,
                     int sides, PartId pid, const Vec3& col, bool up) {
    RenderMesh m;
    Vec3 c = cXZ + Vec3(0, y, 0);
    Vec3 n(0, up ? 1 : -1, 0);
    for (int i = 0; i < sides; ++i) {
        Real a0 = 2 * PI * i / sides, a1 = 2 * PI * (i + 1) / sides;
        Vec3 p0 = c + Vec3(std::cos(a0), 0, std::sin(a0)) * radius;
        Vec3 p1 = c + Vec3(std::cos(a1), 0, std::sin(a1)) * radius;
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        auto v = [&](const Vec3& p) { Vertex vt(p, n, Vec3(1, 0, 0), 0, 0); vt.color = col; return vt; };
        m.vertices.push_back(v(c));
        m.vertices.push_back(v(up ? p0 : p1));
        m.vertices.push_back(v(up ? p1 : p0));
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
    }
    appendToPart(out, pid, m);
}

static BuildingMesh growCylinder(const Scope& scope, const BuildingParams& p) {
    BuildingMesh out;
    Vec3 cXZ = scope.corner(0.5, 0, 0.5); cXZ.y = 0;
    Real baseY = scope.origin.y;
    Real R = std::min(scope.size.x, scope.size.z) * 0.5 * 0.96;
    int sides = std::max(20, p.sides);
    Vec3 wall = p.wallColor;
    Vec3 glass = materialFor(PartId::Glass, wall).albedo;
    Real y = baseY;
    Real gh = p.groundHeight;

    // Round towers are stout, not needle-thin (Marina City / Torre Agbar read at
    // roughly height ≈ 5x diameter). Cap the storey count to that slenderness so a
    // small round lot doesn't become a pencil (ADR-0040).
    Real diameter = 2 * R;
    int maxFloors = std::max(4, static_cast<int>((5.0 * diameter - gh) / p.floorHeight));
    int floors = std::min(p.floors, maxFloors);

    // All bands share one radius — a continuous shell, so there are no radial
    // gaps between the spandrel and window rings to see through (the old inset
    // left open slots and lit the hollow interior).
    emitTube(out, cXZ, R, y, y + 0.5, sides, PartId::Trim, p.trimColor);          // base ring
    emitTube(out, cXZ, R, y + 0.5, y + gh - 0.3, sides, PartId::Glass, glass);    // lobby glass
    emitTube(out, cXZ, R, y + gh - 0.3, y + gh, sides, PartId::Trim, p.trimColor); // cornice ring
    y += gh;
    for (int i = 0; i < floors; ++i) {
        Real fh = p.floorHeight;
        emitTube(out, cXZ, R, y, y + 0.9, sides, PartId::Wall, wall);             // spandrel band
        emitTube(out, cXZ, R, y + 0.9, y + fh, sides, PartId::Glass, glass);      // window band
        y += fh;
    }
    emitDisc(out, cXZ, R, y, sides, PartId::Roof, materialFor(PartId::Roof, wall).albedo, true);
    emitTube(out, cXZ, R, y, y + p.parapet * 0.7, sides, PartId::Trim, p.trimColor);  // parapet
    out.height = (y + p.parapet * 0.7) - baseY;

    BuildingMesh sc;
    emitBox(sc, Scope{scope.origin, {scope.axis[0], Vec3(0, 1, 0), scope.axis[2]},
                      Vec3(scope.size.x, out.height, scope.size.z)}, PartId::Wall, wall);
    if (!sc.parts.empty()) out.proxy = sc.parts.front();
    out.attaches.push_back({cXZ + Vec3(0, y, 0), Vec3(0, 1, 0), "roof"});
    return out;
}

// --- Pagoda (tiered, flared upturned roofs) --------------------------------

// A flared hip roof over a square of half-width `halfW` centred at cXZ, eave at
// `eaveY`: deep eaves (overhang), a concave sweep up to the apex, and the corners
// lifted (`cornerLift`) for the iconic upturned-corner silhouette.
static void emitFlaredRoof(BuildingMesh& out, const Vec3& cXZ, Real eaveY, Real halfW,
                           Real overhang, Real rise, Real cornerLift, const Vec3& tile) {
    RenderMesh m;
    Real e = halfW + overhang;
    auto ring = [&](int k, Real radial, Real frac) {       // k: 0..7 around the square
        Real ang = PI * 0.25 * k;
        Real cx = std::cos(ang), cz = std::sin(ang);
        Real s = 1.0 / std::max(std::fabs(cx), std::fabs(cz));   // project dir onto square
        Vec3 p = cXZ + Vec3(cx, 0, cz) * (e * radial * s);
        bool corner = (k % 2 == 1);
        p.y = eaveY + (radial >= 0.99 ? (corner ? cornerLift : 0.0) : rise * frac);
        return p;
    };
    Vec3 apex = cXZ + Vec3(0, eaveY + rise, 0);
    for (int k = 0; k < 8; ++k) {
        Vec3 e0 = ring(k, 1.0, 0), e1 = ring((k + 1) % 8, 1.0, 0);
        Vec3 m0 = ring(k, 0.42, 0.62), m1 = ring((k + 1) % 8, 0.42, 0.62);
        Vec3 n = normalize(cross(e1 - e0, m0 - e0)); if (n.y < 0) n = n * -1;
        emitQuad(m, e0, e1, m1, m0, n, tile);              // eave -> mid (concave skirt)
        Vec3 tn = normalize(cross(m1 - m0, apex - m0)); if (tn.y < 0) tn = tn * -1;
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        auto v = [&](const Vec3& p) { Vertex vt(p, tn, Vec3(1, 0, 0), 0, 0); vt.color = tile; return vt; };
        m.vertices.push_back(v(m0)); m.vertices.push_back(v(m1)); m.vertices.push_back(v(apex));
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2});   // mid -> apex
    }
    appendToPart(out, PartId::Roof, m);
}

static BuildingMesh growPagoda(const Scope& scope, const BuildingParams& p) {
    BuildingMesh out;
    Rng rng(p.seed);
    Vec3 cXZ = scope.corner(0.5, 0, 0.5); cXZ.y = 0;
    Real baseY = scope.origin.y;
    const Vec3 r(1, 0, 0), f(0, 0, 1);             // world-aligned (grid city)
    Real w0 = std::min(scope.size.x, scope.size.z) * 0.5 * 0.78;

    Vec3 wallCol(0.62, 0.13, 0.11);                // vermillion
    Vec3 colCol(0.42, 0.08, 0.07);                 // darker red columns/trim
    Vec3 tile = (rng.unit() < 0.4) ? Vec3(0.16, 0.28, 0.46)   // imperial blue
                                   : Vec3(0.13, 0.40, 0.25);  // jade green
    Vec3 gold(0.80, 0.64, 0.20);

    auto boxAt = [&](Real cy, Real h, Real hw, PartId pid, const Vec3& col) {
        emitBox(out, Scope{cXZ + Vec3(0, cy, 0) - r * hw - f * hw,
                           {r, Vec3(0, 1, 0), f}, Vec3(hw * 2, h, hw * 2)}, pid, col);
    };

    int tiers = std::max(2, p.tiers);
    Real shrink = 0.80, tierH = std::max(Real(2.8), p.floorHeight);
    Real y = baseY;

    boxAt(y, 0.7, w0 + 0.6, PartId::Ground, Vec3(0.52, 0.50, 0.46));   // stone podium
    y += 0.7;

    Real w = w0;
    for (int t = 0; t < tiers; ++t) {
        boxAt(y, tierH, w, PartId::Wall, wallCol);                     // red tier body
        // Corner columns (the temple posts) + a dark lattice screen per face.
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                emitBox(out, Scope{cXZ + Vec3(sx * w - 0.16, y, sz * w - 0.16),
                                   {r, Vec3(0, 1, 0), f}, Vec3(0.32, tierH, 0.32)},
                        PartId::Trim, colCol);
        boxAt(y + tierH * 0.18, tierH * 0.6, w * 0.82, PartId::Glass, Vec3(0.18, 0.10, 0.06));
        y += tierH;
        // Flared roof at the top of this tier (deep eaves, upturned corners).
        emitFlaredRoof(out, cXZ, y, w, w * 0.55, w * 0.62, w * 0.16, tile);
        w *= shrink;
    }
    // Gold finial: a stacked post + bead spire.
    boxAt(y, 1.6, 0.16, PartId::Detail, gold);
    emitBox(out, Scope{cXZ + Vec3(-0.35, y + 1.6, -0.35), {r, Vec3(0, 1, 0), f},
                       Vec3(0.7, 0.7, 0.7)}, PartId::Detail, gold);
    out.height = (y + 2.3) - baseY;

    BuildingMesh sc;
    emitBox(sc, Scope{Vec3(cXZ.x - w0, baseY, cXZ.z - w0), {r, Vec3(0, 1, 0), f},
                      Vec3(w0 * 2, out.height, w0 * 2)}, PartId::Wall, wallCol);
    if (!sc.parts.empty()) out.proxy = sc.parts.front();
    out.attaches.push_back({cXZ + Vec3(0, y, 0), Vec3(0, 1, 0), "roof"});
    return out;
}

// Rooftop crown (ADR-0040 Pass B): a mechanical penthouse/bulkhead set back on
// the roof, plus the iconic timber water tank on a leg frame — what makes a top
// read as a real building, not an extruded box. `roofY` is the roof slab level;
// footOrigin/width/depth/r/f describe the (possibly set-back) roof footprint.
static void emitCrown(BuildingMesh& out, const Vec3& footOrigin, Real width,
                      Real depth, const Vec3& r, const Vec3& f, Real roofY,
                      const BuildingParams& p, Rng& rng,
                      const Poly2* topPlan = nullptr) {
    const Vec3 up(0, 1, 0);
    // The roof may be an L / U / courtyard PLAN, not its oriented box — a crown
    // element seated by box coordinates alone can hang over the notch (device:
    // "a giant block that doesn't fit properly with the rooftop"). When the
    // caller passes the top plan, every element proves its corners are ON it.
    auto onRoof = [&](const Vec3& o, Real w2, Real d2) {
        if (!topPlan) return true;
        for (int cx = 0; cx <= 1; ++cx)
            for (int cz = 0; cz <= 1; ++cz) {
                Vec3 q = o + r * (w2 * cx) + f * (d2 * cz);
                if (!pointInPolygon(*topPlan, Vec2(q.x, q.z))) return false;
            }
        return true;
    };
    // ROOF PLANT subgrammar (device: "what is that gray block supposed to
    // represent? ... it would help if it had some definition"). The old mute
    // penthouse box is two readable pieces of equipment now:
    //  - a roof-access BULKHEAD — the stair/elevator overrun: human-scaled,
    //    a capped lid slab, and a dark door facing the open roof;
    //  - a louvred HVAC unit — metal casing, intake bands, fan discs on top.
    // Every element proves its seat is ON the roof polygon before emitting.
    const Vec3 darkPanel(0.16, 0.17, 0.18);

    // ---- ROOF PLAN (device: "partition the rooftop space so the water tower,
    // HVAC unit(s) and roof access aren't sitting on top of one another").
    // The roof is planned like a tiny city block: recursive longest-axis
    // bisection (the same technique the street grid uses to cut blocks into
    // lots) carves the parapet-inset rect into cells, and every piece of
    // equipment gets its OWN cell — roof access exactly 1, water tanks 1-2,
    // HVAC units 1-4 by roof size. Items centre in their cell with jitter.
    struct RoofItem { int kind; Real w, d, minW, minD; };   // 0 access, 1 tank, 2 hvac
    struct RoofCell { Real u, v, w, d; };
    const Real inset = 0.7;                                 // parapet clearance
    const Real uw = width - 2 * inset, ud = depth - 2 * inset;
    if (uw < 3.5 || ud < 3.5) return;
    const Real roofArea = uw * ud;

    std::vector<RoofItem> items;
    if (p.floors >= 4 && width > 5 && depth > 5) {          // roof access: always 1
        const Real bw = std::min(std::max(width * 0.28, Real(3.0)), Real(5.0));
        const Real bd = std::min(std::max(depth * 0.24, Real(2.4)), Real(3.8));
        items.push_back({0, bw, bd, Real(2.4), Real(2.0)});
    }
    if (p.floors >= 3 && p.floors <= 20 && !p.curtainWall && width > 4 &&
        depth > 4 && rng.unit() < 0.65) {                   // water tanks: 1-2
        const int nTank = (roofArea > 170.0 && rng.unit() < 0.35) ? 2 : 1;
        for (int i = 0; i < nTank; ++i) {
            const Real tr = std::min(rng.range(1.2, 1.7),
                                     std::min(width, depth) * 0.22);
            items.push_back({1, 2 * tr + 1.0, 2 * tr + 1.0, Real(2.4), Real(2.4)});
        }
    }
    if (p.floors >= 4 && width > 6 && depth > 5 && rng.unit() < 0.85) {
        // HVAC units: 1-4 based on the size of the roof
        const int nHvac =
            std::max(1, std::min(4, static_cast<int>(roofArea / 70.0)));
        for (int i = 0; i < nHvac; ++i) {
            const Real aw = std::min(std::max(width * 0.26, Real(2.6)), Real(5.2));
            const Real ad = std::min(std::max(depth * 0.20, Real(2.0)), Real(3.2));
            items.push_back({2, aw, ad, Real(2.2), Real(1.8)});
        }
    }
    if (items.empty()) return;

    std::vector<RoofCell> cells{{inset, inset, uw, ud}};
    while (cells.size() < items.size()) {
        std::size_t bi = 0;                                 // split the biggest cell
        for (std::size_t i = 1; i < cells.size(); ++i)
            if (cells[i].w * cells[i].d > cells[bi].w * cells[bi].d) bi = i;
        RoofCell c = cells[bi];
        if (std::max(c.w, c.d) < 4.5) break;                // roof full: extras skipped
        const Real t = rng.range(0.4, 0.6);
        RoofCell a = c, b = c;
        if (c.w >= c.d) { a.w = c.w * t; b.u = c.u + a.w; b.w = c.w - a.w; }
        else            { a.d = c.d * t; b.v = c.v + a.d; b.d = c.d - a.d; }
        cells[bi] = a;
        cells.push_back(b);
    }
    // Biggest item takes the biggest cell.
    std::sort(items.begin(), items.end(), [](const RoofItem& a, const RoofItem& b) {
        return a.w * a.d > b.w * b.d;
    });
    std::sort(cells.begin(), cells.end(), [](const RoofCell& a, const RoofCell& b) {
        return a.w * a.d > b.w * b.d;
    });

    // --- emitters (verbatim geometry from the old placement code, seated at a
    // planned cell instead of a rejection-sampled spot) ---
    auto emitAccess = [&](Real u, Real v, Real bw, Real bd) {
        const Real bh = rng.range(2.8, 3.3);
        Vec3 po = footOrigin + r * u + f * v;
        emitBox(out, Scope{Vec3(po.x, roofY, po.z), {r, up, f}, Vec3(bw, bh, bd)},
                PartId::Trim, p.trimColor * 0.85);
        emitBox(out, Scope{Vec3(po.x, roofY + bh, po.z) - r * 0.15 - f * 0.15,
                           {r, up, f}, Vec3(bw + 0.3, 0.14, bd + 0.3)},
                PartId::Trim, p.trimColor * 0.6);
        Vec3 bc = po + r * (bw * 0.5) + f * (bd * 0.5);
        Vec3 rc = footOrigin + r * (width * 0.5) + f * (depth * 0.5);
        Vec3 to = rc - bc;
        const Real dr = dot(to, r), df = dot(to, f);
        const bool alongR = std::fabs(dr) >= std::fabs(df);
        Vec3 n2 = alongR ? r * (dr > 0 ? 1.0 : -1.0) : f * (df > 0 ? 1.0 : -1.0);
        Vec3 tang = alongR ? f : r;
        Vec3 fc = bc + n2 * ((alongR ? bw : bd) * 0.5);
        Vec3 doorO = fc - tang * 0.5;
        emitBox(out, Scope{Vec3(doorO.x, roofY, doorO.z), {tang, up, n2},
                           Vec3(1.0, 2.1, 0.06)},
                PartId::Detail, darkPanel);
    };
    auto emitHvac = [&](Real u, Real v, Real aw, Real ad) {
        const Real ah = rng.range(1.5, 2.1);
        const Vec3 casing = materialFor(PartId::Metal, p.wallColor).albedo;
        const Vec3 louvre(0.30, 0.31, 0.33);
        Vec3 po = footOrigin + r * u + f * v;
        emitBox(out, Scope{Vec3(po.x, roofY, po.z), {r, up, f}, Vec3(aw, ah, ad)},
                PartId::Utility, casing);
        for (int side = 0; side < 2; ++side) {
            Vec3 lo = po + r * 0.25 + f * (side ? ad : Real(-0.04));
            const Real pw = aw - 0.5, ph = ah * 0.6, py = roofY + ah * 0.18;
            const std::size_t vv0 = partMesh(out, PartId::Vent).vertices.size();
            emitBox(out, Scope{Vec3(lo.x, py, lo.z), {r, up, f}, Vec3(pw, ph, 0.04)},
                    PartId::Vent, louvre);
            RenderMesh& vm = partMesh(out, PartId::Vent);
            for (std::size_t vi = vv0; vi < vm.vertices.size(); ++vi) {
                Vertex& vt = vm.vertices[vi];
                const Vec3 rel = vt.position - Vec3(lo.x, py, lo.z);
                vt.u = static_cast<float>(dot(rel, r) / pw);
                vt.v = static_cast<float>(rel.y / ph);
            }
        }
        const int nf = aw > 4.2 ? 2 : 1;
        for (int k2 = 0; k2 < nf; ++k2) {
            const Real fu = aw * (nf == 1 ? 0.5 : (k2 == 0 ? 0.3 : 0.7));
            Vec3 fcen = po + r * fu + f * (ad * 0.5);
            fcen.y = 0;
            const Real frad = std::min(Real(0.6), std::min(aw, ad) * 0.22);
            emitTube(out, fcen, frad, roofY + ah, roofY + ah + 0.22, 12,
                     PartId::Utility, casing * 0.9);
            const std::size_t v0 = partMesh(out, PartId::Fan).vertices.size();
            emitDisc(out, fcen, frad, roofY + ah + 0.22, 12, PartId::Fan,
                     darkPanel, true);
            RenderMesh& fanMesh = partMesh(out, PartId::Fan);
            for (std::size_t vi = v0; vi < fanMesh.vertices.size(); ++vi) {
                Vertex& vv = fanMesh.vertices[vi];
                vv.u = static_cast<float>(0.5 + (vv.position.x - fcen.x) /
                                                    (2.0 * frad) * 0.92);
                vv.v = static_cast<float>(0.5 + (vv.position.z - fcen.z) /
                                                    (2.0 * frad) * 0.92);
            }
        }
    };
    auto emitTank = [&](Real u, Real v, Real w2, Real d2) {
        const Real tr = std::max(Real(0.9), (std::min(w2, d2) - 1.0) * 0.5);
        const Real th = rng.range(2.6, 3.4), legH = rng.range(1.6, 2.4);
        Vec3 tc = footOrigin + r * (u + w2 * 0.5) + f * (v + d2 * 0.5);
        tc.y = 0;
        const Real tankBase = roofY + legH;
        Vec3 woodCol = materialFor(PartId::Wood, p.wallColor).albedo;
        for (int lx = -1; lx <= 1; lx += 2)
            for (int lz = -1; lz <= 1; lz += 2) {
                Vec3 lc = tc + r * (lx * tr * 0.7) + f * (lz * tr * 0.7);
                emitBox(out, Scope{Vec3(lc.x, roofY, lc.z) - r * 0.08 - f * 0.08,
                                   {r, up, f}, Vec3(0.16, legH, 0.16)},
                        PartId::Wood, woodCol * 0.85);
            }
        emitTube(out, tc, tr, tankBase, tankBase + th, 14, PartId::Wood, woodCol);
        emitDisc(out, tc, tr, tankBase + th, 14, PartId::Wood, woodCol * 1.05, true);
    };

    const std::size_t nPlace = std::min(items.size(), cells.size());
    for (std::size_t i = 0; i < nPlace; ++i) {
        const RoofItem& it = items[i];
        const RoofCell& c = cells[i];
        const Real clear = 0.5;
        const Real w2 = std::min(it.w, c.w - clear);
        const Real d2 = std::min(it.d, c.d - clear);
        if (w2 < it.minW || d2 < it.minD) continue;         // cell too small: skip
        bool placed = false;
        for (int attempt = 0; attempt < 4 && !placed; ++attempt) {
            const Real u = c.u + (c.w - w2) * rng.range(0.2, 0.8);
            const Real v = c.v + (c.d - d2) * rng.range(0.2, 0.8);
            Vec3 po = footOrigin + r * u + f * v;
            if (!onRoof(po, w2, d2)) continue;              // L-plan notch: jitter again
            if (it.kind == 0) emitAccess(u, v, w2, d2);
            else if (it.kind == 1) emitTank(u, v, w2, d2);
            else emitHvac(u, v, w2, d2);
            placed = true;
        }
    }
}

BuildingMesh growBuilding(const Scope& scope, const BuildingParams& params,
                          FacadeDetail detail) {
    // Cylinder/Pagoda emit Full at both levels for now (already lean; see the
    // FacadeDetail note in the header).
    if (params.shape == BuildingShape::Cylinder) return growCylinder(scope, params);
    if (params.shape == BuildingShape::Pagoda)   return growPagoda(scope, params);
    BuildingMesh out;
    const bool full = detail == FacadeDetail::Full;
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

    // Entrance side: the face whose outward normal points most toward the street
    // (params.faceDir), so the door faces the road, not an alley (ADR-0040). Side
    // normals follow faceOf: 0=+f, 1=-f, 2=+r, 3=-r.
    Vec3 sideNormal[4] = {f, f * -1, r, r * -1};
    int entranceSide = 0;
    Real bestDot = -1e30;
    for (int s = 0; s < 4; ++s) {
        Real dp = sideNormal[s].x * params.faceDir.x + sideNormal[s].z * params.faceDir.z;
        if (dp > bestDot) { bestDot = dp; entranceSide = s; }
    }

    // Ground floor: taller, glassy retail (or lobby), walkable shell with a real
    // entrance on the street-facing face (ADR-0038 §4).
    Real gh = params.groundHeight;
    Scope ground = storeyScope(y, gh);
    FacadeMode groundMode = params.solidFacade ? FacadeMode::Solid
                          : params.groundRetail ? FacadeMode::Retail
                                                : FacadeMode::Residential;
    for (int side = 0; side < 4; ++side) {
        FacadeMode mode = (side == entranceSide && params.walkableGround)
                              ? FacadeMode::Entrance : groundMode;
        emitFacade(out, ground, side, mode, params, wallColor, detail);
    }
    // Ground slab you can stand on.
    emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                       {r, Vec3(0, 1, 0), f}, Vec3(width, 0.1, depth)},
            PartId::Ground, materialFor(PartId::Ground, wallColor).albedo);

    // Base course / water-table: a low band the building rises from. Emitted
    // per face and skipped on the entrance side (ADR-0040) so it steps around
    // the doorway instead of clipping it ("the foundation eats the base").
    if (full && params.baseCourse) {
        const Real bh = std::min(Real(0.45), gh * 0.12);   // water-table height
        const Real proud = 0.1;
        Vec3 col = params.trimColor * 0.8;
        RenderMesh band;
        for (int side = 0; side < 4; ++side) {
            if (side == entranceSide && params.walkableGround) continue;   // entrance breaks it
            FaceRect fr = faceOf(ground, side);
            Vec3 out = fr.n * proud;
            Vec3 b0 = fr.at(0, 0), b1 = fr.at(fr.width, 0);
            Vec3 t0 = fr.at(0, bh), t1 = fr.at(fr.width, bh);
            emitQuad(band, b0 + out, b1 + out, t1 + out, t0 + out, fr.n, col);  // face
            emitQuad(band, t0 + out, t1 + out, t1, t0, fr.v, col);              // top ledge
        }
        appendToPart(out, PartId::Trim, band);
    }
    // (The entrance attach point + awning + doorframe are emitted by the DOOR
    // element inside emitFacade, so they sit on the actual door bay — a face-
    // centred awning was off by half a bay on even bay counts.)
    // BASE CORNICE: a real stepped profile capping the base (device: "I don't
    // see a cornice at the top of the building's base" — the old single band
    // read as nothing). Three courses stepping outward — bed mould, corona,
    // and a thin cap slab — wrapped around the whole perimeter.
    auto emitCornice = [&](Real yTop, Real scale) {
        struct Tier { Real grow, h; };
        const Tier tiers[3] = {{0.10, 0.16}, {0.24, 0.18}, {0.34, 0.08}};
        Real yb = yTop;
        for (const Tier& t : tiers) {
            Real g2 = t.grow * scale, h2 = t.h * scale;
            Scope sc{Vec3(footOrigin.x, yb, footOrigin.z) - r * g2 - f * g2,
                     {r, Vec3(0, 1, 0), f}, Vec3(width + 2 * g2, h2, depth + 2 * g2)};
            emitBox(out, sc, PartId::Trim, params.trimColor);
            yb += h2;
        }
    };
    if (full && params.stringCourse) emitCornice(y - 0.32, 1.0);
    y += gh;

    // Upper floors carry no pilasters — base piers belong to the base only
    // (ADR-0040), capped by the string course above.
    BuildingParams upper = params;
    upper.pilasters = false;

    // Upper residential floors, with optional setbacks.
    bool didSetback = false;
    for (int i = 0; i < params.floors; ++i) {
        if (params.setbackFloors > 0 && params.setbackEvery > 0 && i > 0 &&
            i % params.setbackFloors == 0 &&
            width - 2 * params.setbackEvery >= 8.0 &&
            depth - 2 * params.setbackEvery >= 8.0) {
            didSetback = true;
            Real d = params.setbackEvery;
            Real dx = std::min(d, width * 0.4), dz = std::min(d, depth * 0.4);
            // Cap the lower (wider) mass with a roof slab so the exposed setback
            // ledge is a real surface, not an open shelf (ADR-0040: stepped
            // towers terrace at each setback). Slab spans the old footprint; the
            // new mass rises from it and hides all but the ledge ring.
            emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                               {r, Vec3(0, 1, 0), f}, Vec3(width, 0.2, depth)},
                    PartId::Roof, materialFor(PartId::Roof, wallColor).albedo);
            footOrigin = footOrigin + r * dx + f * dz;
            width -= 2 * dx; depth -= 2 * dz;
            // A low terrace parapet around the stepped-back mass.
            emitParapet(out, footOrigin, width, depth, r, f, y, 0.6, 0.24,
                        PartId::Trim, materialFor(PartId::Trim, wallColor).albedo);
        }
        Real fh = params.floorHeight;
        Scope storey = storeyScope(y, fh);
        FacadeMode mode = params.solidFacade ? FacadeMode::Solid
                                             : FacadeMode::Residential;
        for (int side = 0; side < 4; ++side) {
            if (params.curtainWall)
                emitCurtainWallRect(out, faceOf(storey, side), wallColor, detail);
            else emitFacade(out, storey, side, mode, upper, wallColor, detail);
        }
        if (i == params.floors / 2) {
            FaceRect ff = faceOf(storey, 0);
            out.attaches.push_back({ff.at(ff.width * 0.5, fh * 0.5), ff.n, "facade"});
        }
        y += fh;
    }

    // QUOINS (element, P2): alternating cast-stone blocks up every corner
    // arris — the masonry corner treatment (and it hides the thin-texture edge
    // where two brick faces meet, device feedback). Skipped on stepped towers
    // (the corners move) and clean-skin facades.
    if (full && params.quoins && !params.curtainWall && !params.solidFacade &&
        !didSetback) {
        const Real qh = 0.42, gap = 0.03, proud = 0.045;
        const Real longL = 0.62, shortL = 0.30;
        const Vec3 qcol = params.trimColor;
        for (int cxi = 0; cxi <= 1; ++cxi)
            for (int czi = 0; czi <= 1; ++czi) {
                int k = 0;
                for (Real qy = baseY + 0.06; qy + qh <= y - 0.45; qy += qh, ++k) {
                    const bool alongR = ((k + cxi + czi) & 1) == 0;
                    const Real lr = alongR ? longL : shortL;    // extent along r
                    const Real lf = alongR ? shortL : longL;    // extent along f
                    const Real x0q = cxi ? width - lr : -proud;
                    const Real z0q = czi ? depth - lf : -proud;
                    Vec3 o = footOrigin + r * x0q + f * z0q;
                    o.y = qy;
                    emitBox(out, Scope{o, {r, Vec3(0, 1, 0), f},
                                       Vec3(lr + proud, qh - gap, lf + proud)},
                            PartId::Trim, qcol);
                }
            }
    }

    // Top cornice: the stepped crown capping the shaft (mirrors the base
    // cornice, a touch larger), so the shaft reads as framed between base and
    // crown — the tripartite base/shaft/capital line a masonry building always
    // has. Glass towers get a clean cap instead.
    if (full && params.stringCourse && !params.curtainWall) emitCornice(y - 0.45, 1.25);
    // Roof: a flat slab + a parapet railing around the perimeter (ADR-0038 §4).
    emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                       {r, Vec3(0, 1, 0), f}, Vec3(width, 0.2, depth)},
            PartId::Roof, materialFor(PartId::Roof, wallColor).albedo);
    if (params.parapet > 0) {
        emitParapet(out, footOrigin, width, depth, r, f, y, params.parapet, 0.28,
                    PartId::Trim, materialFor(PartId::Trim, wallColor).albedo);
    }
    // Crown: mechanical penthouse + rooftop water tank (ADR-0040 Pass B).
    // Flat keeps the parapet for the silhouette but skips the roof furniture.
    if (full) emitCrown(out, footOrigin, width, depth, r, f, y, params, rng);
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


// --- Floorplan buildings (building-grammar-plan.md P3) ----------------------

namespace {

// Offset a CCW plan polygon: d > 0 shrinks (inset), d < 0 grows (outset) — the
// swept-cornice / setback-tier primitive. Same line-intersection construction
// as polygon.h's inset, without its d > 0 guard (cornices need the outset).
Poly2 offsetPlan(const Poly2& poly, Real d) {
    const std::size_t n = poly.size();
    if (n < 3 || d == 0) return poly;
    Poly2 p = poly;
    ensureCCW(p);
    struct Line { Vec2 pt, dir; };
    std::vector<Line> lines(n);
    for (std::size_t i = 0; i < n; ++i) {
        Vec2 a = p[i], b = p[(i + 1) % n];
        Vec2 dir = normalize(b - a);
        Vec2 inward(-dir.y, dir.x);          // interior is LEFT of a CCW edge
        lines[i] = {a + inward * d, dir};
    }
    Poly2 out(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Line& l0 = lines[(i + n - 1) % n];
        const Line& l1 = lines[i];
        Real denom = cross(l0.dir, l1.dir);
        if (std::abs(denom) < 1e-9) { out[i] = l1.pt; continue; }
        Real t = cross(l1.pt - l0.pt, l1.dir) / denom;
        Vec2 q = l0.pt + l0.dir * t;
        // MITER LIMIT: at a near-parallel joint (an arc prow's chords) the
        // line intersection flies arbitrarily far and the ring grows spikes
        // (device: the haywire tower top). Fall back to a bevel-ish offset —
        // the vertex translated by its own edge normal.
        if ((q - p[i]).length() > std::fabs(d) * 4.0 + 0.5) q = l1.pt;
        out[i] = q;
    }
    return out;
}

// One plan edge as a facade rectangle: outward for CCW is the RIGHT of a->b.
FaceRect planEdgeRect(const Poly2& pl, std::size_t i, Real y, Real h) {
    Vec2 a = pl[i], b = pl[(i + 1) % pl.size()];
    Vec2 d = normalize(b - a);
    FaceRect fr;
    fr.bl = Vec3(a.x, y, a.y);
    fr.h = Vec3(d.x, 0, d.y);
    fr.v = Vec3(0, 1, 0);
    fr.n = Vec3(d.y, 0, -d.x);
    fr.width = (b - a).length();
    fr.height = h;
    return fr;
}

// A horizontal SLAB filling the plan: triangulated top + underside, plus a
// side band around the outline — roof decks, cornice tiers, ground pads.
void emitPlanSlab(BuildingMesh& out, const Poly2& pl, Real yTop, Real thick,
                  PartId part, const Vec3& col) {
    if (pl.size() < 3) return;
    RenderMesh m;
    for (const auto& t : triangulatePolygon(pl)) {
        Vec3 a(pl[t[0]].x, yTop, pl[t[0]].y);
        Vec3 b(pl[t[1]].x, yTop, pl[t[1]].y);
        Vec3 c(pl[t[2]].x, yTop, pl[t[2]].y);
        MeshBuilder::emitTri(m, a, b, c, Vec3(0, 1, 0), col);
        Vec3 a2 = a, b2 = b, c2 = c;
        a2.y = b2.y = c2.y = yTop - thick;
        MeshBuilder::emitTri(m, a2, b2, c2, Vec3(0, -1, 0), col);
    }
    for (std::size_t i = 0; i < pl.size(); ++i) {
        FaceRect fr = planEdgeRect(pl, i, yTop - thick, thick);
        emitQuad(m, fr.at(0, 0), fr.at(fr.width, 0), fr.at(fr.width, thick),
                 fr.at(0, thick), fr.n, col);
    }
    appendToPart(out, part, m);
}

// A real LIP around the roof (device: "the roof tops don't really have a lip
// around them"): an upstand RING following the plan outline — outer face on
// the plan line, 0.24 m thick, usually in the facade's own material — capped
// by a slightly oversailing darker coping. Built as MITERED rings (the same
// offset construction the road ribbons use — device: "reuse ... to miter the
// ends together") so the lip is one continuous piece of geometry that turns
// every corner cleanly, instead of overlapping per-edge boxes.
void emitPlanParapet(BuildingMesh& out, const Poly2& pl, Real roofY, Real h,
                     const Vec3& wallCol, PartId wallPart,
                     const Vec3& copingCol) {
    if (h <= 0 || pl.size() < 3) return;
    const Real th = 0.24;    // upstand thickness
    const Real lip = 0.05;   // coping oversail, in and out
    const Real ch = 0.09;    // coping height
    Poly2 O = pl;            // upstand outer ring: ON the plan line
    ensureCCW(O);
    const Poly2 I = offsetPlan(O, th);             // upstand inner ring
    const Poly2 Oc = offsetPlan(O, -lip);          // coping outer ring
    const Poly2 Ic = offsetPlan(O, th + lip);      // coping inner ring
    const std::size_t n = O.size();
    if (I.size() != n || Oc.size() != n || Ic.size() != n) return;
    RenderMesh wall, cop;
    auto ringBand = [](RenderMesh& m, const Poly2& ring, Real y0, Real y1,
                       bool outward, const Vec3& col) {
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const Vec2& a = ring[i];
            const Vec2& b = ring[(i + 1) % ring.size()];
            Vec2 d = b - a;
            if (d.length() < 1e-6) continue;
            d = normalize(d);
            Vec2 nrm = outward ? Vec2(d.y, -d.x) : Vec2(-d.y, d.x);
            emitQuad(m, Vec3(a.x, y0, a.y), Vec3(b.x, y0, b.y),
                     Vec3(b.x, y1, b.y), Vec3(a.x, y1, a.y),
                     Vec3(nrm.x, 0, nrm.y), col);
        }
    };
    auto ringCap = [&](RenderMesh& m, const Poly2& oRing, const Poly2& iRing,
                       Real y, bool up2, const Vec3& col) {
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            emitQuad(m, Vec3(oRing[i].x, y, oRing[i].y),
                     Vec3(oRing[j].x, y, oRing[j].y),
                     Vec3(iRing[j].x, y, iRing[j].y),
                     Vec3(iRing[i].x, y, iRing[i].y),
                     Vec3(0, up2 ? 1 : -1, 0), col);
        }
    };
    // Upstand: outer + inner faces (the coping's underside closes the top).
    ringBand(wall, O, roofY, roofY + h, true, wallCol);
    ringBand(wall, I, roofY, roofY + h, false, wallCol);
    // Coping: oversailing ring with its own faces, top and underside.
    ringBand(cop, Oc, roofY + h, roofY + h + ch, true, copingCol);
    ringBand(cop, Ic, roofY + h, roofY + h + ch, false, copingCol);
    ringCap(cop, Oc, Ic, roofY + h + ch, true, copingCol);
    ringCap(cop, Oc, Ic, roofY + h, false, copingCol);
    appendToPart(out, wallPart, wall);
    appendToPart(out, PartId::Trim, cop);
}

}  // namespace

// ---- MESH OPS (the small op library: lathe / steps / array-composed) -------
// The classical vocabulary is lathe-and-array shaped: a column is a lathe
// profile, a colonnade is an array of columns, a dome is a lathe, a rotunda
// is a drum + a radial array + a dome. These ops live here (and mesh.lathe in
// Lua) so recipes COMPOSE them instead of hand-emitting quads.

RenderMesh latheMesh(const Vec3& c, const std::vector<Vec2>& prof, int segs,
                     const Vec3& col) {
    RenderMesh m;
    if (prof.size() < 2 || segs < 3) return m;
    const Real tau = 6.283185307179586;
    for (std::size_t i = 0; i + 1 < prof.size(); ++i) {
        const Real r0 = prof[i].x, y0 = c.y + prof[i].y;
        const Real r1 = prof[i + 1].x, y1 = c.y + prof[i + 1].y;
        if (r0 < 1e-5 && r1 < 1e-5) continue;
        for (int k = 0; k < segs; ++k) {
            const Real a0 = tau * k / segs, a1 = tau * (k + 1) / segs;
            const Vec2 d0(std::cos(a0), std::sin(a0)), d1(std::cos(a1), std::sin(a1));
            Vec3 p00(c.x + d0.x * r0, y0, c.z + d0.y * r0);
            Vec3 p10(c.x + d1.x * r0, y0, c.z + d1.y * r0);
            Vec3 p01(c.x + d0.x * r1, y1, c.z + d0.y * r1);
            Vec3 p11(c.x + d1.x * r1, y1, c.z + d1.y * r1);
            // Outward facet normal: radial mid-direction tilted by the
            // profile slope (dr/dy), with pure caps falling back to +/-Y.
            Vec2 mid = normalize(d0 + d1);
            Vec3 n(mid.x * (y1 - y0), r0 - r1, mid.y * (y1 - y0));
            if (n.length() < 1e-9) n = Vec3(mid.x, 0, mid.y);
            n = normalize(n);
            if (r0 < 1e-5)      MeshBuilder::emitTri(m, p00, p11, p01, n, col);
            else if (r1 < 1e-5) MeshBuilder::emitTri(m, p00, p10, p01, n, col);
            else                emitQuad(m, p00, p10, p11, p01, n, col);
        }
    }
    return m;
}

namespace {

static void emitLathe(BuildingMesh& out, PartId part, const Vec3& c,
                      const std::vector<Vec2>& prof, int segs, const Vec3& col) {
    appendToPart(out, part, latheMesh(c, prof, segs, col));
}

// A CLASSICAL COLUMN: square plinth, lathe-turned shaft with base rings,
// entasis taper and a flared capital, square abacus. `r3/f3` orient the
// square caps with the facade so rotated buildings stay coherent.
static void emitColumn(BuildingMesh& out, const Vec3& base, Real h, Real r,
                       const Vec3& r3, const Vec3& f3, PartId part,
                       const Vec3& col) {
    if (h < 1.2) return;
    const Vec3 up(0, 1, 0);
    emitBox(out, Scope{base - r3 * (r * 1.35) - f3 * (r * 1.35), {r3, up, f3},
                       Vec3(r * 2.7, 0.16, r * 2.7)}, part, col);
    std::vector<Vec2> prof = {
        {r * 1.20, 0.16},          {r * 1.20, 0.30},
        {r * 1.00, 0.42},          {r * 0.98, h * 0.55},
        {r * 0.84, h - 0.34},      {r * 1.10, h - 0.22},
        {r * 1.14, h - 0.12}};
    emitLathe(out, part, base, prof, 10, col);
    emitBox(out, Scope{base + Vec3(0, h - 0.12, 0) - r3 * (r * 1.3) - f3 * (r * 1.3),
                       {r3, up, f3}, Vec3(r * 2.6, 0.12, r * 2.6)}, part, col);
}

// ENTRANCE STEPS: a porch platform under the door with descending steps —
// centred on `cx` along the face, growing outward from the wall plane.
static void emitEntranceSteps(BuildingMesh& out, const FaceRect& fr, Real cx,
                              Real w, Real platformH, Real platformD,
                              PartId part, const Vec3& col,
                              Real dropBelow = 0) {
    const Vec3 up(0, 1, 0);
    // `dropBelow` extends the run below the storey base to the REAL ground
    // at the entrance (lot layer's sample): the whole stoop grows taller and
    // gains steps, instead of the old fixed platform hovering when the
    // ground fell away (device: "floating steps that aren't reachable").
    const Real total = platformH + std::max(Real(0), dropBelow);
    const int n = std::max(1, static_cast<int>(total / 0.16));
    const Real rise = total / n, run = 0.34;
    Vec3 o = fr.at(cx - w * 0.5, 0) - up * std::max(Real(0), dropBelow);
    // The platform itself (its top hides the door's lowest strip — the
    // threshold reads at platform height).
    emitBox(out, Scope{o, {fr.h, up, fr.n}, Vec3(w, total, platformD)},
            part, col);
    // Steps descend outward: box i tops out one rise lower than the last.
    for (int i = 0; i + 1 < n; ++i) {
        Vec3 so = o + fr.n * (platformD + i * run);
        emitBox(out, Scope{so, {fr.h, up, fr.n},
                           Vec3(w, total - rise * (i + 1), run)},
                part, col);
    }
}

// PORTICO: the columned porch — steps + platform, a colonnade, an
// entablature beam and a triangular pediment. The classical civic front.
static void emitPortico(BuildingMesh& out, const FaceRect& fr,
                        const BuildingParams& bp, int nCols, const Vec3& col) {
    const Vec3 up(0, 1, 0);
    nCols = std::max(2, nCols);
    const Real depth = std::min(Real(3.4), std::max(Real(2.2), fr.width * 0.16));
    Real span = std::min(fr.width * 0.72, (nCols - 1) * 3.4 + 1.0);
    const Real x0 = (fr.width - span) * 0.5;
    const Real platformH = 0.45;
    const Real colH = std::min(fr.height - 1.2, fr.height * 0.82) - platformH;
    if (colH < 2.0) return;
    // Porch platform + steps across the whole colonnade span.
    emitEntranceSteps(out, fr, fr.width * 0.5, span + 1.2, platformH,
                      depth + 0.4, PartId::Concrete, col * 0.92,
                      bp.entranceDropBelow);
    // The colonnade (a linear ARRAY of lathe columns).
    const Real r = std::min(Real(0.30), 0.02 * colH + 0.16);
    for (int i = 0; i < nCols; ++i) {
        const Real x = x0 + span * (Real(i) / (nCols - 1));
        Vec3 cb = fr.at(x, platformH) + fr.n * (depth - r * 1.6);
        emitColumn(out, cb, colH, r, fr.h, fr.n, PartId::Trim, col);
    }
    // Entablature: the beam the columns carry, back to the wall.
    const Real eb = platformH + colH;
    Vec3 eo = fr.at(x0 - 0.5, eb);
    emitBox(out, Scope{eo, {fr.h, up, fr.n}, Vec3(span + 1.0, 0.5, depth + 0.15)},
            PartId::Trim, col);
    // Pediment: a triangular prism — front/back tympanum triangles + two
    // raking roof slopes over the entablature.
    const Real pw = span + 1.0, ph = pw * 0.16, pd = depth + 0.15;
    Vec3 A = fr.at(x0 - 0.5, eb + 0.5), B = A + fr.h * pw;
    Vec3 Af = A + fr.n * pd, Bf = B + fr.n * pd;
    Vec3 apex = A + fr.h * (pw * 0.5) + up * ph;
    Vec3 apexF = apex + fr.n * pd;
    RenderMesh ped;
    MeshBuilder::emitTri(ped, Af, Bf, apexF, fr.n, col);            // front face
    MeshBuilder::emitTri(ped, B, A, apex, fr.n * -1, col);          // back face
    Vec3 nL = normalize(cross(fr.n * -1, apex - A));
    Vec3 nR = normalize(cross(Bf - B, apex - B));
    emitQuad(ped, A, Af, apexF, apex, nL, col * 0.96);              // left slope
    emitQuad(ped, Bf, B, apex, apexF, nR, col * 0.96);              // right slope
    appendToPart(out, PartId::Trim, ped);
}

// ROTUNDA: drum + radial colonnade + entablature ring + dome + cupola — the
// capitol crown, all lathe-and-array.
static void emitRotunda(BuildingMesh& out, const Vec3& c, Real R, Real roofY,
                        const Vec3& r3, const Vec3& f3, const Vec3& wallCol,
                        const Vec3& trimCol) {
    const Real drumH = std::max(Real(2.6), R * 0.85);
    Vec3 cc(c.x, 0, c.z);
    // Solid inner drum.
    emitTube(out, cc, R * 0.82, roofY, roofY + drumH, 20, PartId::Stucco, wallCol);
    emitDisc(out, cc, R * 0.82, roofY + drumH, 20, PartId::Stucco, wallCol, true);
    // The colonnade ring (a RADIAL array of columns).
    const int nCols = std::max(8, static_cast<int>(R * 4));
    const Real tau = 6.283185307179586;
    for (int i = 0; i < nCols; ++i) {
        const Real a = tau * i / nCols;
        Vec3 cb(c.x + std::cos(a) * R, roofY, c.z + std::sin(a) * R);
        emitColumn(out, cb, drumH - 0.5, std::min(Real(0.24), R * 0.09),
                   r3, f3, PartId::Trim, trimCol);
    }
    // Entablature ring over the columns, then the dome + cupola + finial.
    emitTube(out, cc, R + 0.35, roofY + drumH - 0.5, roofY + drumH + 0.1, 20,
             PartId::Trim, trimCol);
    emitDisc(out, cc, R + 0.35, roofY + drumH + 0.1, 20, PartId::Trim, trimCol, true);
    std::vector<Vec2> dome;
    const int DN = 6;
    for (int i = 0; i <= DN; ++i) {
        const Real t = Real(i) / DN * 1.5707963;
        dome.push_back(Vec2(R * 0.86 * std::cos(t),
                            drumH + 0.1 + R * 0.62 * std::sin(t)));
    }
    emitLathe(out, PartId::Roof, Vec3(c.x, roofY, c.z), dome, 20,
              trimCol * 0.9);
    std::vector<Vec2> cupola = {{R * 0.14, drumH + 0.1 + R * 0.60},
                                {R * 0.14, drumH + 0.1 + R * 0.62 + 0.9},
                                {R * 0.02, drumH + 0.1 + R * 0.62 + 1.4},
                                {0.0, drumH + 0.1 + R * 0.62 + 1.6}};
    emitLathe(out, PartId::Trim, Vec3(c.x, roofY, c.z), cupola, 10, trimCol);
}

// BALCONIES: one slab + railing per facade bay, hung at floor level over a
// street-facing edge (the condo / modern-flat vocabulary). Same bay math as
// the facade splitter, so balconies land under their windows.
static void emitBalconyRun(BuildingMesh& out, const FaceRect& fr,
                           const BuildingParams& p) {
    const Vec3 up(0, 1, 0);
    const int bays = std::max(
        1, static_cast<int>(std::lround(fr.width / std::max(p.bayWidth, Real(0.5)))));
    const Real bw = fr.width / bays;
    const Real w = std::min(bw - 0.9, Real(3.2));
    if (w < 1.2) return;
    const Real depth = 1.25, railH = 0.95;
    const Vec3 railCol(0.22, 0.23, 0.25);
    for (int b = 0; b < bays; ++b) {
        const Real cx = (b + 0.5) * bw;
        emitBox(out, Scope{fr.at(cx - w * 0.5, -0.07), {fr.h, up, fr.n},
                           Vec3(w, 0.14, depth)},
                PartId::Concrete, p.trimColor);
        emitBox(out, Scope{fr.at(cx - w * 0.5, 0.07) + fr.n * (depth - 0.06),
                           {fr.h, up, fr.n}, Vec3(w, railH, 0.06)},
                PartId::Metal, railCol);
        emitBox(out, Scope{fr.at(cx - w * 0.5, 0.07), {fr.h, up, fr.n},
                           Vec3(0.06, railH, depth)},
                PartId::Metal, railCol);
        emitBox(out, Scope{fr.at(cx + w * 0.5 - 0.06, 0.07), {fr.h, up, fr.n},
                           Vec3(0.06, railH, depth)},
                PartId::Metal, railCol);
    }
}

// PORCH: the covered timber entrance — platform + steps, corner posts, and a
// flat canopy with a fascia board — centred on the door (bungalow/craftsman).
static void emitPorch(BuildingMesh& out, const FaceRect& fr,
                      const BuildingParams& p) {
    const Vec3 up(0, 1, 0);
    const Real w = std::min(fr.width - 0.8, Real(4.6));
    if (w < 2.2) return;
    const Real depth = 1.9, platH = 0.28;
    const Real roofY = std::min(fr.height - 0.3, Real(2.75));
    const Real cx = fr.width * 0.5;
    const Vec3 wood = materialFor(PartId::Wood, p.wallColor).albedo;
    emitEntranceSteps(out, fr, cx, w, platH, depth, PartId::Concrete,
                      p.trimColor * 0.9, p.entranceDropBelow);
    const int posts = w > 3.6 ? 3 : 2;
    for (int i = 0; i < posts; ++i) {
        const Real x = cx - w * 0.5 + 0.12 + (w - 0.38) * (Real(i) / (posts - 1));
        emitBox(out, Scope{fr.at(x, platH) + fr.n * (depth - 0.24),
                           {fr.h, up, fr.n}, Vec3(0.14, roofY - platH, 0.14)},
                PartId::Wood, wood * 0.9);
    }
    emitBox(out, Scope{fr.at(cx - w * 0.5 - 0.25, roofY), {fr.h, up, fr.n},
                       Vec3(w + 0.5, 0.10, depth + 0.35)},
            PartId::Roof, materialFor(PartId::Roof, p.wallColor).albedo);
    emitBox(out, Scope{fr.at(cx - w * 0.5 - 0.25, roofY - 0.16) +
                           fr.n * (depth + 0.29),
                       {fr.h, up, fr.n}, Vec3(w + 0.5, 0.18, 0.06)},
            PartId::Wood, wood);
}

// OPEN PARKING DECK storey: a solid spandrel band below, an open air gap, a
// thin top edge band, and slim piers per bay — plus a guard rail across the
// opening. The caller lays a deck slab per storey so the openings read as
// floors, not holes.
static void emitParkingDeckRect(BuildingMesh& out, const FaceRect& fr,
                                const BuildingParams& p, const Vec3& wallColor) {
    const Vec3 up(0, 1, 0);
    RenderMesh wall;
    const Real spandrel = std::min(Real(1.05), fr.height * 0.35);
    const Real band = 0.30;
    emitQuad(wall, fr.at(0, 0), fr.at(fr.width, 0), fr.at(fr.width, spandrel),
             fr.at(0, spandrel), fr.n, wallColor);
    emitQuad(wall, fr.at(0, fr.height - band), fr.at(fr.width, fr.height - band),
             fr.at(fr.width, fr.height), fr.at(0, fr.height), fr.n, wallColor);
    appendToPart(out, p.wallPart, wall);
    const int bays = std::max(
        1, static_cast<int>(std::lround(fr.width / std::max(p.bayWidth, Real(0.5)))));
    const Real bw = fr.width / bays;
    for (int b = 0; b <= bays; ++b) {
        const Real x = std::min(std::max(b * bw - 0.14, Real(0)), fr.width - 0.28);
        emitBox(out, Scope{fr.at(x, 0) - fr.n * 0.05, {fr.h, up, fr.n},
                           Vec3(0.28, fr.height, 0.30)},
                PartId::Concrete, wallColor * 0.94);
    }
    emitBox(out, Scope{fr.at(0, spandrel + 0.32) - fr.n * 0.02,
                       {fr.h, up, fr.n}, Vec3(fr.width, 0.05, 0.05)},
            PartId::Metal, Vec3(0.25, 0.26, 0.28));
}

// ART-DECO SPIRE CROWN: stepped setback blocks over the top tier and a
// lathe-turned mast — the skyline finial (replaces the mechanical penthouse).
// Returns the crown's rise above roofY.
static Real emitSpireCrown(BuildingMesh& out, const OBB2& topObb, Real roofY,
                           const BuildingParams& p, const Vec3& wallColor) {
    const Vec3 up(0, 1, 0);
    Vec3 r3(topObb.axis[0].x, 0, topObb.axis[0].y);
    Vec3 f3(topObb.axis[1].x, 0, topObb.axis[1].y);
    const Vec3 c(topObb.center.x, 0, topObb.center.y);
    const Real hw = topObb.half[0], hd = topObb.half[1];
    Real y = roofY;
    for (Real s : {Real(0.62), Real(0.38)}) {
        const Real w2 = hw * s, d2 = hd * s;
        const Real h = std::max(Real(1.2), std::min(hw, hd) * 0.45);
        emitBox(out, Scope{Vec3(c.x, y, c.z) - r3 * w2 - f3 * d2, {r3, up, f3},
                           Vec3(w2 * 2, h, d2 * 2)},
                p.wallPart, wallColor);
        emitBox(out, Scope{Vec3(c.x, y + h, c.z) - r3 * (w2 + 0.12) -
                               f3 * (d2 + 0.12),
                           {r3, up, f3}, Vec3(w2 * 2 + 0.24, 0.14, d2 * 2 + 0.24)},
                PartId::Trim, p.trimColor);
        y += h + 0.14;
    }
    const Real mastH =
        std::min(Real(9.0), std::max(Real(3.0), std::min(hw, hd) * 1.6));
    std::vector<Vec2> prof = {{0.50, 0.0},
                              {0.34, mastH * 0.25},
                              {0.18, mastH * 0.60},
                              {0.06, mastH * 0.90},
                              {0.0, mastH}};
    emitLathe(out, PartId::Metal, Vec3(c.x, y, c.z), prof, 10,
              Vec3(0.55, 0.56, 0.60));
    return (y + mastH) - roofY;
}

// SAWTOOTH ROOF: north-light factory teeth — a slope up, a vertical
// clerestory glass drop, repeated along the top plan's long axis, with
// wall-material end caps. Returns the teeth's rise above y.
static Real emitSawtoothRoof(BuildingMesh& out, const Poly2& topPlan, Real y,
                             const BuildingParams& p, const Vec3& wallColor) {
    OBB2 obb = orientedBoundingBox(topPlan);
    const int la = obb.longAxis(), sa = 1 - la;
    Vec3 r3(obb.axis[la].x, 0, obb.axis[la].y);
    Vec3 f3(obb.axis[sa].x, 0, obb.axis[sa].y);
    Real hw = obb.half[la], hd = obb.half[sa];
    // INSCRIBE the teeth in the actual plan, not its bounding box: a rect-ish
    // plan can still fall 15% short of its OBB (notches, trapezoid ends), and
    // teeth spanning the full box hang past the walls there (device: "the roof
    // extends outwards ... not adhering to the shape of the floorplan").
    // Shrink both extents until all four corners sit inside the plan.
    {
        auto inside = [&](Real s) {
            for (int cu = -1; cu <= 1; cu += 2)
                for (int cv = -1; cv <= 1; cv += 2) {
                    Vec2 q = obb.center + obb.axis[la] * (cu * hw * s) +
                             obb.axis[sa] * (cv * hd * s);
                    if (!pointInPolygon(topPlan, q)) return false;
                }
            return true;
        };
        Real s = 1.0;
        while (s > 0.55 && !inside(s)) s -= 0.05;
        hw *= s; hd *= s;
    }
    const int teeth = std::max(2, static_cast<int>(std::lround(2 * hw / 4.2)));
    const Real tw = 2 * hw / teeth;
    const Real rise = std::min(Real(1.7), std::max(Real(0.9), tw * 0.38));
    const Vec3 C(obb.center.x, 0, obb.center.y);
    const Vec3 up(0, 1, 0);
    const Vec3 roofCol = materialFor(PartId::Roof, wallColor).albedo;
    const Vec3 glassCol = materialFor(PartId::Glass, wallColor).albedo;
    RenderMesh roof, glass, wallM;
    const Real y0 = y + 0.03;
    for (int k = 0; k < teeth; ++k) {
        const Real u0 = -hw + k * tw, u1 = u0 + tw;
        Vec3 A0 = C + r3 * u0 - f3 * hd + up * y0;         // low eave, near
        Vec3 A1 = C + r3 * u0 + f3 * hd + up * y0;         // low eave, far
        Vec3 B0 = C + r3 * u1 - f3 * hd + up * (y0 + rise);
        Vec3 B1 = C + r3 * u1 + f3 * hd + up * (y0 + rise);
        emitQuad(roof, A0, A1, B1, B0, normalize(up * tw - r3 * rise), roofCol);
        Vec3 D0 = C + r3 * u1 - f3 * hd + up * y0;
        Vec3 D1 = C + r3 * u1 + f3 * hd + up * y0;
        emitQuad(glass, D0, D1, B1, B0, r3, glassCol);     // the clerestory
        MeshBuilder::emitTri(wallM, A0, D0, B0, f3 * -1, wallColor);
        MeshBuilder::emitTri(wallM, D1, A1, B1, f3, wallColor);
    }
    appendToPart(out, PartId::Roof, roof);
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, p.wallPart, wallM);
    return rise + 0.03;
}

// STEEPLE: a square bell tower rising through the pitched roof at the
// entrance end of the ridge — belfry openings on all four faces, a cornice
// cap, a pyramidal spire and a finial. Returns the tower top (world y).
static Real emitSteeple(BuildingMesh& out, const Vec3& cXZ, const Vec3& r3,
                        const Vec3& f3, Real baseYv, Real ridgeY,
                        const BuildingParams& p, const Vec3& wallColor) {
    const Vec3 up(0, 1, 0);
    const Real tw = 1.5;                       // tower half-width
    const Real bodyTop = ridgeY + 2.6;
    emitBox(out, Scope{Vec3(cXZ.x, baseYv, cXZ.z) - r3 * tw - f3 * tw,
                       {r3, up, f3}, Vec3(tw * 2, bodyTop - baseYv, tw * 2)},
            p.wallPart, wallColor);
    // Belfry openings: a dark louvred panel proud of each face near the top.
    const Vec3 dark(0.14, 0.15, 0.16);
    const Real oy = bodyTop - 2.0;
    emitBox(out, Scope{Vec3(cXZ.x, oy, cXZ.z) + r3 * (tw - 0.02) - f3 * 0.45,
                       {f3, up, r3}, Vec3(0.9, 1.3, 0.04)}, PartId::Detail, dark);
    emitBox(out, Scope{Vec3(cXZ.x, oy, cXZ.z) - r3 * (tw + 0.02) - f3 * 0.45,
                       {f3, up, r3}, Vec3(0.9, 1.3, 0.04)}, PartId::Detail, dark);
    emitBox(out, Scope{Vec3(cXZ.x, oy, cXZ.z) + f3 * (tw - 0.02) - r3 * 0.45,
                       {r3, up, f3}, Vec3(0.9, 1.3, 0.04)}, PartId::Detail, dark);
    emitBox(out, Scope{Vec3(cXZ.x, oy, cXZ.z) - f3 * (tw + 0.06) - r3 * 0.45,
                       {r3, up, f3}, Vec3(0.9, 1.3, 0.04)}, PartId::Detail, dark);
    // Cornice cap, then the pyramidal spire + finial.
    emitBox(out, Scope{Vec3(cXZ.x, bodyTop, cXZ.z) - r3 * (tw + 0.15) -
                           f3 * (tw + 0.15),
                       {r3, up, f3}, Vec3(tw * 2 + 0.3, 0.18, tw * 2 + 0.3)},
            PartId::Trim, p.trimColor);
    const Real spireH = 3.2, sy = bodyTop + 0.18;
    const Vec3 apex(cXZ.x, sy + spireH, cXZ.z);
    Vec3 c0 = Vec3(cXZ.x, sy, cXZ.z) - r3 * tw - f3 * tw;
    Vec3 c1 = Vec3(cXZ.x, sy, cXZ.z) + r3 * tw - f3 * tw;
    Vec3 c2 = Vec3(cXZ.x, sy, cXZ.z) + r3 * tw + f3 * tw;
    Vec3 c3 = Vec3(cXZ.x, sy, cXZ.z) - r3 * tw + f3 * tw;
    RenderMesh spire;
    const Vec3 roofCol = materialFor(PartId::Roof, wallColor).albedo;
    MeshBuilder::emitTri(spire, c0, c1, apex, f3 * -1, roofCol);
    MeshBuilder::emitTri(spire, c1, c2, apex, r3, roofCol);
    MeshBuilder::emitTri(spire, c2, c3, apex, f3, roofCol);
    MeshBuilder::emitTri(spire, c3, c0, apex, r3 * -1, roofCol);
    appendToPart(out, PartId::Roof, spire);
    std::vector<Vec2> finial = {{0.06, 0.0}, {0.04, 0.5}, {0.0, 0.85}};
    emitLathe(out, PartId::Trim, apex, finial, 8, p.trimColor);
    return apex.y + 0.85;
}

// The VEHICLE BAY front (fire stations, loading docks, parking entries): the
// entrance edge's ground floor as a row of wide segmented roller doors. Each
// bay is a real recessed opening — jamb + head reveals connect the wall plane
// back to the door plane — and the door panel carries horizontal SLATS so it
// reads as a roller door, with a lintel band across the whole front.
void emitBayFront(BuildingMesh& out, const FaceRect& fr,
                  const BuildingParams& p, const Vec3& wallColor) {
    RenderMesh wall, detail, trim;
    const Real W = fr.width, H = fr.height;
    int n = std::max(1, p.groundBays);
    Real margin = 1.0;
    const Real gap = 0.8;
    Real bayW = (W - 2 * margin - (n - 1) * gap) / n;
    while (n > 1 && bayW < 3.2) {   // cramped front: fewer, proper-width bays
        --n;
        bayW = (W - 2 * margin - (n - 1) * gap) / n;
    }
    bayW = std::min(bayW, Real(4.8));
    if (bayW < 2.6) {   // no room for even one bay: plain wall face
        emitQuad(wall, fr.at(0, 0), fr.at(W, 0), fr.at(W, H), fr.at(0, H),
                 fr.n, wallColor);
        appendToPart(out, p.wallPart, wall);
        return;
    }
    margin = (W - (n * bayW + (n - 1) * gap)) * 0.5;
    const Real bayH = std::min(H - 0.8, Real(3.8));
    auto wallQuad = [&](Real a0, Real a1, Real b0, Real b1) {
        if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
        emitQuad(wall, fr.at(a0, b0), fr.at(a1, b0), fr.at(a1, b1),
                 fr.at(a0, b1), fr.n, wallColor);
    };
    wallQuad(0, margin, 0, H);                 // end piers
    wallQuad(W - margin, W, 0, H);
    wallQuad(margin, W - margin, bayH, H);     // over the doors
    const Vec3 in = fr.n * -0.35;              // door plane, well recessed
    const Vec3 panel(0.36, 0.37, 0.39), slat(0.28, 0.29, 0.31);
    for (int b = 0; b < n; ++b) {
        const Real x0 = margin + b * (bayW + gap), x1 = x0 + bayW;
        if (b + 1 < n) wallQuad(x1, x1 + gap, 0, bayH);   // pier between bays
        Vec3 tL = fr.at(x0, bayH), tR = fr.at(x1, bayH);
        Vec3 bL = fr.at(x0, 0), bR = fr.at(x1, 0);
        emitQuad(wall, bL, bL + in, tL + in, tL, fr.h, wallColor);        // jamb
        emitQuad(wall, bR + in, bR, tR, tR + in, fr.h * -1, wallColor);   // jamb
        emitQuad(wall, tL, tL + in, tR + in, tR, fr.v * -1, wallColor);   // head
        emitQuad(detail, bL + in, bR + in, tR + in, tL + in, fr.n, panel);
        const Vec3 proud = fr.n * 0.03;
        for (Real sy = 0.5; sy < bayH - 0.3; sy += 0.55) {
            Vec3 sL = fr.at(x0 + 0.12, sy) + in + proud;
            Vec3 sR = fr.at(x1 - 0.12, sy) + in + proud;
            emitQuad(detail, sL, sR, sR + fr.v * 0.16, sL + fr.v * 0.16,
                     fr.n, slat);
        }
    }
    // Lintel band across the whole front above the doors.
    const Vec3 ov = fr.n * 0.10;
    const Real ly0 = bayH + 0.05, ly1 = std::min(H - 0.1, bayH + 0.5);
    if (ly1 > ly0)
        emitQuad(trim, fr.at(margin * 0.4, ly0) + ov,
                 fr.at(W - margin * 0.4, ly0) + ov,
                 fr.at(W - margin * 0.4, ly1) + ov,
                 fr.at(margin * 0.4, ly1) + ov, fr.n, p.trimColor);
    appendToPart(out, p.wallPart, wall);
    appendToPart(out, PartId::Detail, detail);
    appendToPart(out, PartId::Trim, trim);
}

}  // namespace

std::size_t entranceEdgeFor(const Poly2& plan, const BuildingParams& params) {
    // The longest edge whose outward normal points most toward faceDir.
    std::size_t entranceEdge = 0;
    Real bestScore = -1e30;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        Vec2 a = plan[i], b = plan[(i + 1) % plan.size()];
        Vec2 d = b - a;
        Real len = d.length();
        if (len < human::DOOR_WIDTH + 1.2) continue;
        Vec2 nrm(d.y / len, -d.x / len);
        Real score = nrm.x * params.faceDir.x + nrm.y * params.faceDir.z + len * 0.01;
        if (score > bestScore) { bestScore = score; entranceEdge = i; }
    }
    return entranceEdge;
}

std::vector<StoreyPlan> storeyPlans(const Poly2& planIn,
                                    const BuildingParams& params) {
    std::vector<StoreyPlan> out;
    Poly2 plan = planIn;
    if (plan.size() < 3) return out;
    ensureCCW(plan);
    out.push_back({plan, 0, params.groundHeight, 0});
    // A tier inset that EXPLODES must not become the next tier (see the
    // exterior loop's history: "one of the triangle skyscrapers went haywire
    // when building the top"): valid only if it truly shrank, every vertex
    // stayed inside the tier below, and no edge flipped direction.
    auto insetOk = [](const Poly2& outer, const Poly2& inner) {
        if (inner.size() != outer.size()) return false;
        const Real ai = area(inner);
        if (ai < 60.0 || ai >= area(outer)) return false;
        for (std::size_t k = 0; k < inner.size(); ++k) {
            if (!pointInPolygon(outer, inner[k])) return false;
            Vec2 d0 = outer[(k + 1) % outer.size()] - outer[k];
            Vec2 d1 = inner[(k + 1) % inner.size()] - inner[k];
            if (dot(d0, d1) <= 0) return false;   // edge flipped
        }
        return true;
    };
    Poly2 cur = plan;
    Real y = params.groundHeight;
    int tier = 0;
    for (int i = 0; i < params.floors; ++i) {
        if (params.setbackFloors > 0 && params.setbackEvery > 0 && i > 0 &&
            i % params.setbackFloors == 0) {
            Poly2 next = offsetPlan(cur, params.setbackEvery);
            if (insetOk(cur, next)) {
                cur = next;
                ++tier;
            }
        }
        out.push_back({cur, y, params.floorHeight, tier});
        y += params.floorHeight;
    }
    return out;
}

InteriorLayout interiorLayout(const Poly2& planIn, const BuildingParams& params,
                              std::size_t entranceEdge) {
    InteriorLayout il;
    Poly2 plan = planIn;
    if (plan.size() < 3) return il;
    ensureCCW(plan);
    // One full-storey flight, sized by the TALLEST storey it must serve (the
    // ground storey): riser <= 0.18 (comfortable, and well under the
    // character's 0.55 stepHeight), run 0.28.
    const int risers =
        std::max(3, static_cast<int>(std::ceil(params.groundHeight / 0.18)));
    il.run = (risers - 1) * 0.28;
    // Candidate edges, longest first, never the entrance edge (the lobby's
    // clear path from the door stays clear).
    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < plan.size(); ++i)
        if (i != entranceEdge % plan.size()) order.push_back(i);
    auto elen = [&](std::size_t e) {
        return (plan[(e + 1) % plan.size()] - plan[e]).length();
    };
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) { return elen(a) > elen(b); });
    for (std::size_t e : order) {
        const Vec2 a = plan[e], b = plan[(e + 1) % plan.size()];
        const Real len = elen(e);
        if (len < 4.0) continue;
        const Vec2 u = (b - a) * (1.0 / len);
        const Vec2 nIn(-u.y, u.x);   // CCW plan: inside is left of the edge
        // STRAIGHT flights only (the dog-leg variant is owed: its arrival
        // lands under the slab above unless the hole grows case-by-case).
        // The stair needs its run + a 1.0 m arrival strip along the edge.
        const Real wellLen = il.run + 1.0;
        const Real wellWid = il.width;
        if (len < wellLen + 1.2) continue;
        const Real inset = std::max(params.wallThickness + 0.15, Real(0.45));
        const Vec2 s = a + u * ((len - wellLen) * 0.5) + nIn * inset;
        // The FULL stair footprint must sit inside the plan...
        Poly2 fit{s, s + u * wellLen, s + u * wellLen + nIn * wellWid,
                  s + nIn * wellWid};
        bool fits = true;
        for (const Vec2& c : fit)
            if (!pointInPolygon(plan, c)) { fits = false; break; }
        if (!fits) continue;
        il.hasStair = true;
        il.stairDir = u;
        il.stairFoot = s + nIn * (il.width * 0.5);
        // ...but the HOLE cut from ceilings/slabs is only the ascent's upper
        // reach plus a 5 cm lip past the top tread. Floor stays solid beside
        // and beyond it, so walking around the well to the next flight's
        // foot is walking on floor, not falling back down the stairwell.
        // EVERY flight uses this same run (growInterior gives upper storeys
        // the same riser COUNT with shallower risers), so one rect serves
        // every level; 0.25 run clears a 2.2 m capsule's head on a 3.2 m
        // storey with margin (the walk gate measured 0.3 run blocking it).
        const Vec2 h0 = s + u * (il.run * 0.25) - nIn * 0.05;
        const Vec2 h1 = s + u * (il.run + 0.05) - nIn * 0.05;
        const Vec2 wid = nIn * (wellWid + 0.10);
        il.well = Poly2{h0, h1, h1 + wid, h0 + wid};
        return il;
    }
    return il;
}

BuildingMesh growInterior(const Poly2& planIn, const BuildingParams& params,
                          Real baseY, RenderMesh* colliderOut) {
    BuildingMesh out;
    Poly2 plan = planIn;
    if (plan.size() < 3) return out;
    ensureCCW(plan);
    const std::size_t entranceEdge = entranceEdgeFor(plan, params);
    const InteriorLayout il = interiorLayout(plan, params, entranceEdge);
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, params);
    if (storeys.size() < 2) return out;   // no storeys above ground
    const Vec3 icol = materialFor(PartId::Interior, params.wallColor).albedo;
    RenderMesh mesh;   // slabs + stairs, folded into PartId::Interior at the end

    // How high the stair reaches: the first shrunken tier the well no longer
    // fits inside ends it (the slab there keeps its hole shut).
    int stairTop = 0;
    if (il.hasStair) {
        stairTop = static_cast<int>(storeys.size()) - 1;
        for (std::size_t k = 1; k < storeys.size(); ++k) {
            bool fits = true;
            for (const Vec2& c : il.well)
                if (!pointInPolygon(storeys[k].plan, c)) { fits = false; break; }
            if (!fits) { stairTop = static_cast<int>(k) - 1; break; }
        }
    }

    // --- floor slabs (top + underside; the underside IS the ceiling of the
    // storey below; the top storey's ceiling is the roof slab's underside,
    // which the exterior grow always emits) ------------------------------
    for (std::size_t k = 1; k < storeys.size(); ++k) {
        std::vector<Poly2> holes;
        if (il.hasStair && static_cast<int>(k) <= stairTop)
            holes.push_back(il.well);
        const Real yTop = baseY + storeys[k].y0 + 0.05;
        for (const auto& t : triangulateWithHoles(storeys[k].plan, holes)) {
            MeshBuilder::emitTri(mesh, Vec3(t[0].x, yTop, t[0].y),
                                 Vec3(t[1].x, yTop, t[1].y),
                                 Vec3(t[2].x, yTop, t[2].y), Vec3(0, 1, 0),
                                 icol);
            const Real yb = yTop - 0.25;
            MeshBuilder::emitTri(mesh, Vec3(t[0].x, yb, t[0].y),
                                 Vec3(t[2].x, yb, t[2].y),
                                 Vec3(t[1].x, yb, t[1].y), Vec3(0, -1, 0),
                                 icol);
            if (colliderOut)
                MeshBuilder::emitTri(*colliderOut, Vec3(t[0].x, yTop, t[0].y),
                                     Vec3(t[1].x, yTop, t[1].y),
                                     Vec3(t[2].x, yTop, t[2].y),
                                     Vec3(0, 1, 0), icol);
        }
    }

    // --- inner walls per storey (same layout truth as the facade) --------
    const FacadeMode upMode =
        params.solidFacade ? FacadeMode::Solid : FacadeMode::Residential;
    for (std::size_t k = 1; k < storeys.size(); ++k) {
        const StoreyPlan& spk = storeys[k];
        for (std::size_t e = 0; e < spk.plan.size(); ++e) {
            const FaceRect fr =
                planEdgeRect(spk.plan, e, baseY + spk.y0, spk.h);
            if (params.curtainWall) {
                RenderMesh skin;
                const Vec3 off = fr.n * -params.wallThickness;
                emitQuad(skin, fr.at(0, 0) + off, fr.at(fr.width, 0) + off,
                         fr.at(fr.width, fr.height) + off,
                         fr.at(0, fr.height) + off, fr.n * -1.0, icol);
                appendToPart(out, PartId::Interior, skin);
            } else {
                emitInnerWallRect(out, fr, facadeLayout(fr, upMode, params),
                                  params.wallThickness, params.wallColor);
            }
        }
    }

    // --- the stair (riser <= 0.18, run 0.28, inside the well) ------------
    const Vec2 u = il.stairDir;
    auto flight = [&](const Vec2& foot, const Vec2& dir, Real y0f, Real rise,
                      int nR) {
        const Vec2 perp(-dir.y, dir.x);
        const Real riser = rise / nR;
        const Real half = il.width * 0.5;
        for (int j = 0; j < nR; ++j) {
            const Real yT = y0f + riser * (j + 1);
            const Vec2 t0 = foot + dir * (0.28 * j);
            const Vec2 t1 = foot + dir * (0.28 * (j + 1));
            const Vec3 A(t0.x - perp.x * half, yT, t0.y - perp.y * half);
            const Vec3 B(t0.x + perp.x * half, yT, t0.y + perp.y * half);
            const Vec3 C(t1.x + perp.x * half, yT, t1.y + perp.y * half);
            const Vec3 D(t1.x - perp.x * half, yT, t1.y - perp.y * half);
            const Vec3 A0(A.x, yT - riser, A.z), B0(B.x, yT - riser, B.z);
            emitQuad(mesh, A, B, C, D, Vec3(0, 1, 0), icol);           // tread
            emitQuad(mesh, A0, B0, B, A, Vec3(-dir.x, 0, -dir.y), icol);
            if (colliderOut) {
                emitQuad(*colliderOut, A, B, C, D, Vec3(0, 1, 0), icol);
                emitQuad(*colliderOut, A0, B0, B, A,
                         Vec3(-dir.x, 0, -dir.y), icol);
            }
        }
        // Railing band on the open (+perp) side, 0.1..1.1 above the slope.
        const Vec2 r0 = foot + perp * half;
        const Vec2 r1 = foot + dir * (0.28 * nR) + perp * half;
        const Vec3 RA(r0.x, y0f + 0.1, r0.y), RB(r1.x, y0f + rise + 0.1, r1.y);
        const Vec3 RC(r1.x, y0f + rise + 1.1, r1.y), RD(r0.x, y0f + 1.1, r0.y);
        emitQuad(mesh, RA, RB, RC, RD, Vec3(perp.x, 0, perp.y), icol);
        emitQuad(mesh, RB, RA, RD, RC, Vec3(-perp.x, 0, -perp.y), icol);
        if (colliderOut)
            emitQuad(*colliderOut, RA, RB, RC, RD, Vec3(perp.x, 0, perp.y),
                     icol);
    };
    // UNIFORM flights: every storey's flight uses the ground flight's riser
    // count and run — upper risers just get shallower — so all flights share
    // one XZ footprint, one arrival x, and the single well hole. Mixed runs
    // measured as a trap: the shorter flight's head hit the slab before the
    // shared hole began, and its top opened onto the hole's mid-air.
    const int nR =
        std::max(3, static_cast<int>(std::lround(il.run / 0.28)) + 1);
    for (int k = 0; il.hasStair && k < stairTop; ++k) {
        const Real yk = baseY + storeys[static_cast<std::size_t>(k)].y0 + 0.05;
        const Real rise = storeys[static_cast<std::size_t>(k) + 1].y0 -
                          storeys[static_cast<std::size_t>(k)].y0;
        flight(il.stairFoot, u, yk, rise, nR);
    }

    appendToPart(out, PartId::Interior, mesh);
    out.height = storeys.back().y0 + storeys.back().h;
    return out;
}

BuildingMesh growPlanBuilding(const Poly2& planIn, const BuildingParams& params,
                              Real baseY, FacadeDetail detail) {
    BuildingMesh out;
    // LOD1 (city-render-perf R2): the SAME plan, layout and massing decisions,
    // drawn flat — ornament elements are skipped, silhouette elements (roof
    // planes, parapet, steeple/spire/dome) are kept. Every `full &&` gate
    // below is this switch.
    const bool full = detail == FacadeDetail::Full;
    Poly2 plan = planIn;
    if (plan.size() < 3) return out;
    ensureCCW(plan);
    Rng rng(params.seed);
    const Vec3 wallColor = params.wallColor * (0.92 + 0.08 * rng.unit());

    // Street-facing edge: the longest edge whose outward normal points most
    // toward faceDir — the door (and its awning/architrave) lands there.
    // Shared with growInterior (ADR-0080) so both agree on the front door.
    const std::size_t entranceEdge = entranceEdgeFor(plan, params);

    // Swept cornice: three stepped courses following the CURRENT plan outline.
    auto sweptCornice = [&](const Poly2& pl, Real yTop, Real scale) {
        struct Tier { Real grow, h; };
        const Tier tiers[3] = {{0.10, 0.16}, {0.24, 0.18}, {0.34, 0.08}};
        Real yb = yTop;
        for (const Tier& t : tiers) {
            emitPlanSlab(out, offsetPlan(pl, -t.grow * scale), yb + t.h * scale,
                         t.h * scale, PartId::Trim, params.trimColor);
            yb += t.h * scale;
        }
    };
    // Corner POSTS at every plan vertex: a square pier hiding the wall miter
    // (the floorplan counterpart of the box path's quoined arris). With quoins
    // on, the post alternates block heights like a quoin stack.
    auto cornerPosts = [&](const Poly2& pl, Real y0, Real h) {
        if (params.solidFacade) return;
        for (std::size_t i = 0; i < pl.size(); ++i) {
            Vec2 P = pl[i];
            Vec2 pPrev = pl[(i + pl.size() - 1) % pl.size()];
            Vec2 pNext = pl[(i + 1) % pl.size()];
            Vec2 d0 = normalize(P - pPrev), d1 = normalize(pNext - P);
            Vec2 n0(d0.y, -d0.x), n1(d1.y, -d1.x);
            Vec2 bis = n0 + n1;
            Real bl = bis.length();
            if (bl < 1e-6) continue;                       // straight-through vertex
            // Only REAL corners get a post: chord joints of a tessellated
            // CURVED plan edge (turn < ~33 deg) stay smooth, so a round tower
            // reads as a curve, not a ribbed drum.
            if (std::fabs(cross(d0, d1)) < (params.curtainWall ? 0.45 : 0.55))
                continue;
            bis = bis * (1.0 / bl);
            Vec2 side(-bis.y, bis.x);
            const Real half = 0.20, proud = 0.05;
            Vec3 r3(side.x, 0, side.y), f3(bis.x, 0, bis.y);
            if (params.curtainWall) {
                // A CORNER MULLION: the glass planes are inset 0.10 from each
                // face, so at a kinked vertex they don't meet — an open slit
                // into the hollow shell (device: "cutting holes into the
                // sides"). A metal post centred on the vertex closes the gap
                // and is what real curtain walls put there anyway.
                Scope s{Vec3(P.x, y0, P.y) - r3 * half - f3 * (half + 0.06),
                        {r3, Vec3(0, 1, 0), f3}, Vec3(half * 2, h, half * 2)};
                emitBox(out, s, PartId::Metal, Vec3(0.30, 0.31, 0.33));
                continue;
            }
            if (params.quoins) {
                const Real qh = 0.42, gap = 0.03;
                int k = 0;
                for (Real qy = y0; qy + qh <= y0 + h; qy += qh, ++k) {
                    Real w = (k & 1) ? half * 1.5 : half * 2.3;
                    Scope s{Vec3(P.x, qy, P.y) - r3 * (w * 0.5) - f3 * (half - proud),
                            {r3, Vec3(0, 1, 0), f3}, Vec3(w, qh - gap, half * 2)};
                    emitBox(out, s, PartId::Trim, params.trimColor);
                }
            } else {
                Scope s{Vec3(P.x, y0, P.y) - r3 * half - f3 * (half - proud),
                        {r3, Vec3(0, 1, 0), f3}, Vec3(half * 2, h, half * 2)};
                emitBox(out, s, PartId::Trim, params.trimColor * 0.92);
            }
        }
    };

    Real y = baseY;
    const Real gh = params.groundHeight;
    FacadeMode groundMode = params.solidFacade ? FacadeMode::Solid
                          : params.groundRetail ? FacadeMode::Retail
                                                : FacadeMode::Residential;
    // SIDE vehicle bays (attached garage / loading side): the widest non-
    // entrance edge roughly perpendicular to the street face carries them.
    std::size_t sideEdge = plan.size();   // invalid = none
    if (params.sideBays > 0) {
        Real bestLen = 3.4 * params.sideBays;
        for (std::size_t i = 0; i < plan.size(); ++i) {
            if (i == entranceEdge) continue;
            Vec2 a = plan[i], b = plan[(i + 1) % plan.size()];
            Vec2 d = b - a;
            const Real len = d.length();
            if (len < 1e-6 || len < bestLen) continue;
            Vec2 nrm(d.y / len, -d.x / len);
            if (std::fabs(nrm.x * params.faceDir.x + nrm.y * params.faceDir.z) >
                0.5)
                continue;   // faces the street or the rear, not a side
            bestLen = len;
            sideEdge = i;
        }
    }
    // Ground storey: one facade rect per plan edge; the door on the street edge.
    // retailStreetOnly (P3.c): storefronts only where the edge FACES the street
    // (normal within ~70 deg of faceDir); side/rear edges wear plain walls.
    for (std::size_t i = 0; i < plan.size(); ++i) {
        // VEHICLE BAYS claim the street edge outright (fire station, depot).
        if (i == entranceEdge && params.groundBays > 0) {
            emitBayFront(out, planEdgeRect(plan, i, y, gh), params, wallColor);
            continue;
        }
        if (i == sideEdge) {
            BuildingParams sp = params;
            sp.groundBays = params.sideBays;
            emitBayFront(out, planEdgeRect(plan, i, y, gh), sp, wallColor);
            continue;
        }
        FacadeMode mode = (i == entranceEdge && params.walkableGround)
                              ? FacadeMode::Entrance : groundMode;
        if (mode == FacadeMode::Retail && params.retailStreetOnly) {
            Vec2 a = plan[i], b = plan[(i + 1) % plan.size()];
            Vec2 d = normalize(b - a);
            Vec2 nrm(d.y, -d.x);
            if (nrm.x * params.faceDir.x + nrm.y * params.faceDir.z < 0.35)
                mode = FacadeMode::Residential;
        }
        if (params.curtainWall && mode != FacadeMode::Entrance)
            emitCurtainWallRect(out, planEdgeRect(plan, i, y, gh), wallColor, detail);
        else if (full)
            emitFacadeRect(out, planEdgeRect(plan, i, y, gh), mode, params, wallColor);
        else
            emitFlatFacadeRect(out, planEdgeRect(plan, i, y, gh), mode, params, wallColor);
        // Enterable buildings (ADR-0080): back the one-sided exterior skin
        // with an inner face at -wallThickness so the room reads as a room,
        // not as a view through to the sky.
        if (full && params.openDoorway) {
            const FaceRect ifr = planEdgeRect(plan, i, y, gh);
            if (params.curtainWall && mode != FacadeMode::Entrance) {
                RenderMesh skin;
                const Vec3 off = ifr.n * -params.wallThickness;
                emitQuad(skin, ifr.at(0, 0) + off, ifr.at(ifr.width, 0) + off,
                         ifr.at(ifr.width, ifr.height) + off,
                         ifr.at(0, ifr.height) + off, ifr.n * -1.0,
                         materialFor(PartId::Interior, wallColor).albedo);
                appendToPart(out, PartId::Interior, skin);
            } else {
                emitInnerWallRect(out, ifr, facadeLayout(ifr, mode, params),
                                  params.wallThickness, wallColor);
            }
        }
    }
    // The covered timber PORCH (bungalow/craftsman) — brings its own platform
    // and steps, so it replaces the classical entrance elements.
    if (full && params.porch) {
        emitPorch(out, planEdgeRect(plan, entranceEdge, y, gh), params);
    } else
    // CLASSICAL entrance elements on the street face: a portico (colonnade +
    // entablature + pediment over porch steps) or bare entrance steps.
    if (full && (params.portico > 0 || params.entranceSteps)) {
        FaceRect efr = planEdgeRect(plan, entranceEdge, y, gh);
        if (params.portico > 0 && efr.width > 7.0)
            emitPortico(out, efr, params, params.portico, params.trimColor);
        else
            emitEntranceSteps(out, efr, efr.width * 0.5,
                              std::min(efr.width * 0.5, Real(5.0)), 0.4, 1.4,
                              PartId::Concrete, params.trimColor * 0.92,
                              params.entranceDropBelow);
    }
    emitPlanSlab(out, plan, y + 0.05, 0.1, PartId::Ground,
                 materialFor(PartId::Ground, wallColor).albedo);
    // Enterable ground storey (ADR-0080): close the room from above. A
    // ceiling underside at the storey head, with the stair well punched
    // through when one fits -- the hole comes from the same interiorLayout
    // the streamed interior will build its stair from, so they agree by
    // construction.
    if (full && params.openDoorway) {
        const InteriorLayout il = interiorLayout(plan, params, entranceEdge);
        std::vector<Poly2> holes;
        if (il.hasStair) holes.push_back(il.well);
        const Real cy = y + gh - 0.25;
        const Vec3 icol = materialFor(PartId::Interior, wallColor).albedo;
        RenderMesh ceil;
        for (const auto& t : triangulateWithHoles(plan, holes))
            MeshBuilder::emitTri(ceil, Vec3(t[0].x, cy, t[0].y),
                                 Vec3(t[2].x, cy, t[2].y),
                                 Vec3(t[1].x, cy, t[1].y), Vec3(0, -1, 0),
                                 icol);
        appendToPart(out, PartId::Interior, ceil);
    }
    // Base course wraps the plan (skipping the door edge).
    if (full && params.baseCourse) {
        const Real bh = std::min(Real(0.45), gh * 0.12);
        RenderMesh band;
        for (std::size_t i = 0; i < plan.size(); ++i) {
            if (i == entranceEdge && params.walkableGround) continue;
            FaceRect fr = planEdgeRect(plan, i, y, bh);
            Vec3 ov = fr.n * 0.1;
            emitQuad(band, fr.at(0, 0) + ov, fr.at(fr.width, 0) + ov,
                     fr.at(fr.width, bh) + ov, fr.at(0, bh) + ov, fr.n,
                     params.trimColor * 0.8);
            emitQuad(band, fr.at(0, bh) + ov, fr.at(fr.width, bh) + ov,
                     fr.at(fr.width, bh), fr.at(0, bh), fr.v, params.trimColor * 0.8);
        }
        appendToPart(out, PartId::Trim, band);
    }
    if (full && params.stringCourse) sweptCornice(plan, y + gh - 0.32, 1.0);
    if (full) cornerPosts(plan, y, gh);
    y += gh;

    // Upper floors; setbacks shrink the plan per tier (base/shaft/capital),
    // each transition capped by a roof slab + a swept cornice. The storey
    // stack itself comes from storeyPlans (ADR-0080) so the streamed
    // interior can never disagree with the exterior about where a floor is;
    // this loop draws the SAME sequence it always drew (the mesh-hash census
    // in test_building_lod is the byte-identity witness).
    BuildingParams upper = params;
    upper.pilasters = false;
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, params);
    Poly2 cur = plan;
    Real tierY0 = y;
    for (int i = 0; i < params.floors; ++i) {
        const StoreyPlan& sp = storeys[static_cast<std::size_t>(i) + 1];
        if (i > 0 && sp.tier != storeys[static_cast<std::size_t>(i)].tier) {
            // A setback landed at this floor: cap the tier below.
            emitPlanSlab(out, cur, y - 0.05, 0.2, PartId::Roof,
                         materialFor(PartId::Roof, wallColor).albedo);
            if (full && params.stringCourse && !params.curtainWall)
                sweptCornice(cur, y - 0.4, 1.0);
            if (full) cornerPosts(cur, tierY0, y - tierY0);
            emitPlanParapet(out, offsetPlan(cur, 0.02), y, 0.55,
                            materialFor(PartId::Trim, wallColor).albedo,
                            PartId::Trim,
                            materialFor(PartId::Trim, wallColor).albedo * 0.9);
            cur = sp.plan;
            tierY0 = y;
        }
        const Real fh = params.floorHeight;
        for (std::size_t e = 0; e < cur.size(); ++e) {
            if (full && params.parkingDecks && !params.curtainWall) {
                emitParkingDeckRect(out, planEdgeRect(cur, e, y, fh), upper,
                                    wallColor);
                continue;
            }
            if (params.curtainWall)
                emitCurtainWallRect(out, planEdgeRect(cur, e, y, fh), wallColor,
                                    detail);
            else if (full)
                emitFacadeRect(out, planEdgeRect(cur, e, y, fh),
                               params.solidFacade ? FacadeMode::Solid
                                                  : FacadeMode::Residential,
                               upper, wallColor);
            else
                emitFlatFacadeRect(out, planEdgeRect(cur, e, y, fh),
                                   params.solidFacade ? FacadeMode::Solid
                                                      : FacadeMode::Residential,
                                   upper, wallColor);
            // BALCONIES on street-facing edges, second storey and up.
            if (full && params.balconies && !params.curtainWall &&
                !params.solidFacade && i >= 1) {
                Vec2 a = cur[e], b2 = cur[(e + 1) % cur.size()];
                Vec2 d = b2 - a;
                const Real len = d.length();
                if (len > 2.4) {
                    Vec2 nrm(d.y / len, -d.x / len);
                    if (nrm.x * params.faceDir.x + nrm.y * params.faceDir.z >
                        0.3)
                        emitBalconyRun(out, planEdgeRect(cur, e, y, fh), params);
                }
            }
        }
        // Parking storeys read as DECKS, not holes: a slab per level.
        if (params.parkingDecks && !params.curtainWall)
            emitPlanSlab(out, cur, y + 0.02, 0.12, PartId::Concrete,
                         wallColor * 0.92);
        if (i == params.floors / 2) {
            FaceRect ff = planEdgeRect(cur, entranceEdge % cur.size(), y, fh);
            out.attaches.push_back({ff.at(ff.width * 0.5, fh * 0.5), ff.n, "facade"});
        }
        y += fh;
    }
    if (full) cornerPosts(cur, tierY0, y - tierY0);

    // ROOF (P3.c): a Gable/Hip pitched roof over a rect-ish top plan — the
    // residential silhouette — else the flat deck + parapet + crown.
    OBB2 topObb = orientedBoundingBox(cur);
    const bool rectish =
        area(cur) > 0.85 * (4 * topObb.half[0] * topObb.half[1]);
    const bool sawtooth =
        params.roofStyle == BuildingParams::RoofStyle::Sawtooth && rectish;
    const bool pitched =
        !sawtooth && params.roofStyle != BuildingParams::RoofStyle::Flat &&
        params.roofStyle != BuildingParams::RoofStyle::Sawtooth && rectish;
    Real roofRise = 0;
    if (sawtooth) {
        // The factory roof: a ceiling deck, then the north-light teeth.
        if (full && params.stringCourse && !params.curtainWall)
            sweptCornice(cur, y - 0.30, 0.7);
        emitPlanSlab(out, cur, y + 0.02, 0.15, PartId::Roof,
                     materialFor(PartId::Roof, wallColor).albedo);
        roofRise = emitSawtoothRoof(out, cur, y, params, wallColor);
    } else if (pitched) {
        const bool hip = params.roofStyle == BuildingParams::RoofStyle::Hip;
        const int la = topObb.longAxis(), sa = 1 - la;
        Vec3 r3(topObb.axis[la].x, 0, topObb.axis[la].y);
        Vec3 f3(topObb.axis[sa].x, 0, topObb.axis[sa].y);
        const Real ov = 0.45;                            // eaves overhang
        const Real rk = hip ? Real(0) : Real(0.40);      // gable RAKE overhang
        const Real hwW = topObb.half[la];                // wall plane (gable ends)
        const Real hw = hwW + (hip ? ov : rk);           // slope extent along ridge
        const Real hd = topObb.half[sa] + ov;
        const Real rise = std::max(Real(0.8), params.roofPitch * hd);
        roofRise = rise + 0.03;
        // A modest eaves cornice band, then a thin ceiling deck under the roof.
        if (full && params.stringCourse && !params.curtainWall) sweptCornice(cur, y - 0.30, 0.7);
        emitPlanSlab(out, cur, y + 0.03, 0.15, PartId::Roof,
                     materialFor(PartId::Roof, wallColor).albedo);
        Vec3 C(topObb.center.x, y + 0.03, topObb.center.y);
        const Real rh = hip ? std::max(Real(0.6), hw - hd) : hw;   // ridge half-length
        Vec3 A0 = C - r3 * hw - f3 * hd, A1 = C + r3 * hw - f3 * hd;
        Vec3 B0 = C - r3 * hw + f3 * hd, B1 = C + r3 * hw + f3 * hd;
        Vec3 up(0, 1, 0);
        Vec3 Rg0 = C - r3 * rh + up * rise, Rg1 = C + r3 * rh + up * rise;
        RenderMesh roof, gableW;
        // SHINGLES (device: top faces want shingle relief): slopes go in their
        // own part with the RoofShingle bake; the per-building tint rides in
        // vertex colour (the bake carries the tone + course steps -> normal
        // map). Tint picked deterministically from the wall colour so the
        // house palette stays coherent without another RNG draw.
        static const Vec3 kShingleTint[] = {
            {0.78, 0.78, 0.84},   // slate grey
            {0.94, 0.78, 0.62},   // warm cedar
            {0.72, 0.80, 0.74},   // mossy grey-green
            {0.90, 0.62, 0.52},   // faded terracotta
        };
        const Vec3 roofCol = kShingleTint[
            (static_cast<int>(wallColor.x * 255) * 3 +
             static_cast<int>(wallColor.y * 255) * 5 +
             static_cast<int>(wallColor.z * 255) * 7) & 3];
        // Slope-fitted UVs in world metres / tile: u marches along the eave,
        // v climbs the slope, so the courses always run parallel to the eave
        // whatever the building's yaw. The loader skips its world-planar re-UV
        // for this part (it would break exactly this).
        const Real tile = surfaceWorldTileSize(RenderMaterial::Surface::RoofShingle);
        const float sv = static_cast<float>(std::sqrt(hd * hd + rise * rise) / tile);
        auto su = [&](const Vec3& p) {
            return static_cast<float>(dot(p - A0, r3) / tile);
        };
        Vec3 nNear = normalize(f3 * (-rise) + up * hd);
        Vec3 nFar = normalize(f3 * rise + up * hd);
        MeshBuilder::emitQuadUV(roof, A0, A1, Rg1, Rg0, nNear, roofCol,   // near slope
                                su(A0), 0, su(A1), 0, su(Rg1), sv, su(Rg0), sv);
        MeshBuilder::emitQuadUV(roof, B1, B0, Rg0, Rg1, nFar, roofCol,    // far slope
                                su(B1), 0, su(B0), 0, su(Rg0), sv, su(Rg1), sv);
        // ROOF SLAB ANATOMY (device: "the roof should have some thickness ...
        // edges not shingled but a solid color"): the roof is a SLAB, not a
        // film. Shingles live on the top surfaces only; every edge and every
        // underside is solid trim, named as built: the UNDERSIDE (open-eave
        // soffit) is the slope plane dropped by the slab thickness, the EAVE
        // FASCIA is the vertical band closing the slab along the horizontal
        // eaves, and on gables the RAKE FASCIA is the sloped band closing the
        // slab cross-section at the end overhangs.
        const Vec3 dnT = up * -0.18;                     // slab thickness
        const Vec3 tc = materialFor(PartId::Trim, wallColor).albedo;
        RenderMesh under, fascia;
        emitQuad(under, A0 + dnT, A1 + dnT, Rg1 + dnT, Rg0 + dnT, nNear * -1, tc);
        emitQuad(under, B1 + dnT, B0 + dnT, Rg0 + dnT, Rg1 + dnT, nFar * -1, tc);
        emitQuad(fascia, A0 + dnT, A1 + dnT, A1, A0, f3 * -1, tc);   // near eave
        emitQuad(fascia, B1 + dnT, B0 + dnT, B0, B1, f3, tc);        // far eave
        if (hip) {
            const Real run = std::max(Real(0.2), hw - rh);
            const float hv = static_cast<float>(std::sqrt(run * run + rise * rise) / tile);
            const float du = static_cast<float>(2.0 * hd / tile);
            MeshBuilder::emitTriUV(roof, A0, B0, Rg0,
                                   normalize(r3 * (-rise) + up * run), roofCol,
                                   0, 0, du, 0, du * 0.5f, hv);
            MeshBuilder::emitTriUV(roof, A1, B1, Rg1,
                                   normalize(r3 * rise + up * run), roofCol,
                                   0, 0, du, 0, du * 0.5f, hv);
            // hip end slopes: underside copies + the end eave fascia bands
            MeshBuilder::emitTri(under, A0 + dnT, B0 + dnT, Rg0 + dnT,
                                 normalize(r3 * rise - up * run), tc);
            MeshBuilder::emitTri(under, A1 + dnT, B1 + dnT, Rg1 + dnT,
                                 normalize(r3 * (-rise) - up * run), tc);
            emitQuad(fascia, A0 + dnT, B0 + dnT, B0, A0, r3 * -1, tc);
            emitQuad(fascia, B1 + dnT, A1 + dnT, A1, B1, r3, tc);
        } else {
            // Gable END walls stay on the WALL plane and rise to the ridge in
            // the wall material; the slopes overhang them by the rake, and the
            // RAKE FASCIA (one sloped band per slope per end) closes the slab
            // cross-section so nothing is see-through from any angle.
            for (int sgn = -1; sgn <= 1; sgn += 2) {
                const Vec3 e3 = r3 * static_cast<Real>(sgn);
                const Vec3 We = C + e3 * hwW;            // wall plane
                const Vec3 eN = (sgn < 0 ? A0 : A1);     // near-eave corner
                const Vec3 eF = (sgn < 0 ? B0 : B1);     // far-eave corner
                const Vec3 rg = (sgn < 0 ? Rg0 : Rg1);   // ridge end
                MeshBuilder::emitTri(gableW, We - f3 * hd, We + f3 * hd,
                                     We + up * rise, e3, wallColor);
                emitQuad(fascia, eN + dnT, rg + dnT, rg, eN, e3, tc);
                emitQuad(fascia, eF + dnT, rg + dnT, rg, eF, e3, tc);
            }
        }
        appendToPart(out, PartId::Trim, under);
        appendToPart(out, PartId::Trim, fascia);
        appendToPart(out, PartId::Shingle, roof);
        appendToPart(out, params.wallPart, gableW);
        // CHIMNEY: a masonry stack through the slope near the ridge, offset
        // along the ridge so it reads against the sky.
        if (params.chimney) {
            const Real cu = rh * 0.55;
            Vec3 cc = C + r3 * cu;
            const Vec3 brick(0.42, 0.24, 0.18);
            const Real chTop = y + rise + 0.85;
            emitBox(out, Scope{Vec3(cc.x, y - 0.6, cc.z) - r3 * 0.42 - f3 * 0.42,
                               {r3, up, f3}, Vec3(0.84, chTop - (y - 0.6), 0.84)},
                    PartId::Brick, brick);
            emitBox(out, Scope{Vec3(cc.x, chTop, cc.z) - r3 * 0.52 - f3 * 0.52,
                               {r3, up, f3}, Vec3(1.04, 0.16, 1.04)},
                    PartId::Trim, params.trimColor * 0.8);
        }
        // STEEPLE (churches): the bell tower rises through the roof at the
        // ridge end nearest the entrance.
        if (params.steeple) {
            Vec2 em = (plan[entranceEdge] +
                       plan[(entranceEdge + 1) % plan.size()]) * 0.5;
            const Real side =
                dot(em - topObb.center, topObb.axis[la]) >= 0 ? 1.0 : -1.0;
            Vec3 cS = C + r3 * (side * rh * 0.7);
            const Real towerTop =
                emitSteeple(out, cS, r3, f3, y - 1.2, y + 0.03 + rise, params,
                            wallColor);
            roofRise = std::max(roofRise, towerTop - y);
        }
    } else {
        if (full && params.stringCourse && !params.curtainWall) sweptCornice(cur, y - 0.45, 1.25);
        emitPlanSlab(out, cur, y + 0.05, 0.2, PartId::Roof,
                     materialFor(PartId::Roof, wallColor).albedo);
        if (params.parapet > 0) {
            // The lip continues the FACADE material (a brick building has a
            // brick parapet) with a trim coping; glass/solid facades keep the
            // whole lip in trim so the ring doesn't read as floating cladding.
            const bool plainLip = params.curtainWall || params.solidFacade;
            emitPlanParapet(out, cur, y + 0.05, params.parapet,
                            plainLip ? materialFor(PartId::Trim, wallColor).albedo
                                     : wallColor,
                            plainLip ? PartId::Trim : params.wallPart,
                            materialFor(PartId::Trim, wallColor).albedo * 0.9);
        }
        // Crown seated on the top tier's oriented frame: a DOME rotunda for
        // capitols/town halls, else the mechanical penthouse + tank.
        Vec3 r3(topObb.axis[0].x, 0, topObb.axis[0].y);
        Vec3 f3(topObb.axis[1].x, 0, topObb.axis[1].y);
        Vec3 fo = Vec3(topObb.center.x, 0, topObb.center.y) -
                  r3 * topObb.half[0] - f3 * topObb.half[1];
        if (params.spire) {
            // The art-deco stepped crown + mast instead of the penthouse.
            roofRise = std::max(
                roofRise, emitSpireCrown(out, topObb, y + 0.05, params,
                                         wallColor));
        } else if (params.dome) {
            const Real R = std::min(
                Real(6.0), std::max(Real(2.6),
                                    std::min(topObb.half[0], topObb.half[1]) *
                                        0.55));
            emitRotunda(out, Vec3(topObb.center.x, 0, topObb.center.y), R,
                        y + 0.05, r3, f3, wallColor, params.trimColor);
            roofRise = R * 0.62 + std::max(Real(2.6), R * 0.85) + 1.7;
        } else if (full) {
            // Flat (LOD1) keeps spire/dome/steeple — they are the skyline —
            // but skips the penthouse + roof-furniture pack.
            emitCrown(out, fo, topObb.half[0] * 2, topObb.half[1] * 2, r3, f3,
                      y + 0.05, params, rng, &cur);
        }
    }
    out.attaches.push_back({Vec3(centroid(cur).x, y + roofRise, centroid(cur).y),
                            Vec3(0, 1, 0), "roof"});
    // roofRise carries the pitched ridge OR the dome rotunda's height.
    out.height = (y + (pitched ? roofRise : std::max(params.parapet, roofRise))) -
                 baseY;

    // Coarse HLOD proxy: the plan's oriented box, ground to roof.
    {
        OBB2 obb = orientedBoundingBox(plan);
        Vec3 r3(obb.axis[0].x, 0, obb.axis[0].y), f3(obb.axis[1].x, 0, obb.axis[1].y);
        BuildingMesh scratch;
        emitBox(scratch, Scope{Vec3(obb.center.x, baseY, obb.center.y) -
                                   r3 * obb.half[0] - f3 * obb.half[1],
                               {r3, Vec3(0, 1, 0), f3},
                               Vec3(obb.half[0] * 2, out.height, obb.half[1] * 2)},
                PartId::Wall, wallColor);
        out.proxy = scratch.merged();
    }
    return out;
}

}  // namespace engine
