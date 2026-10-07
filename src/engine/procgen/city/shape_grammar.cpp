#include "shape_grammar.h"
#include "trades.h"   // a shop unit's trade: one table for the facade, its interior and its place
#include "core_plan.h"
#include "room_plan.h"
#include "furniture.h"   // the core: shafts, stairwells, the ground ceiling's holes (M5)

#include "road_mesh.h"            // triangulatePolygon (floorplan roof/slab fill)
#include "triangulate.h"          // triangulateWithHoles (interior ceilings, ADR-0080)
#include "../surface_maps.h"      // surfaceWorldTileSize (shingle slope UVs)
#include "../../mesh_builder.h"
#include "../noise.h"             // emitSoftBox's lumps
#include <algorithm>
#include <array>
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
        case FacadeStyle::Brick: {
            // Warm reds/browns for most; then the brickyard's other runs (Glenn, 2026-09-30: "more varied
            // building materials ... it's all very samey"): buff, cream, chocolate, orange, iron-spot grey.
            const Real k = rng.unit();
            if (k < 0.52) return lerp(Vec3(0.50, 0.22, 0.16), Vec3(0.62, 0.40, 0.28), t);
            static const Vec3 kBrick[] = {
                {0.74, 0.62, 0.46},   // buff / tan (the Upper East Side apartment tower)
                {0.82, 0.76, 0.64},   // cream
                {0.36, 0.23, 0.17},   // chocolate
                {0.66, 0.34, 0.20},   // orange-red
                {0.50, 0.47, 0.45},   // iron-spot grey
            };
            return kBrick[static_cast<int>((k - 0.52) / 0.48 * 5) % 5] * (0.94 + 0.12 * t);
        }
        case FacadeStyle::Stucco:
            return lerp(Vec3(0.86, 0.82, 0.72), Vec3(0.80, 0.74, 0.60), t);
        case FacadeStyle::Painted:
            // Muted pastels (residential).
            return lerp(Vec3(0.74, 0.78, 0.78), Vec3(0.80, 0.72, 0.66), t) +
                   Vec3(rng.range(-0.04, 0.04), rng.range(-0.04, 0.04), rng.range(-0.04, 0.04));
        case FacadeStyle::GlassCurtain: {
            // The frame and trim of a glass building: aluminium, or dark bronze / black / white metal.
            const Real k = rng.unit();
            if (k < 0.55) return lerp(Vec3(0.58, 0.62, 0.66), Vec3(0.66, 0.68, 0.70), t);
            if (k < 0.72) return Vec3(0.20, 0.18, 0.16);   // dark bronze
            if (k < 0.87) return Vec3(0.12, 0.12, 0.13);   // black
            return Vec3(0.84, 0.84, 0.82);                 // white
        }
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
        case FacadeStyle::Sandstone: {
            // Warm buff / honey ashlar (banks, museums, deco masonry) -- and the other dressed stones and
            // terracottas of a midtown avenue.
            const Real k = rng.unit();
            if (k < 0.40) return lerp(Vec3(0.78, 0.66, 0.48), Vec3(0.86, 0.76, 0.58), t);
            static const Vec3 kStone[] = {
                {0.80, 0.79, 0.74},   // Indiana limestone
                {0.90, 0.88, 0.82},   // white glazed terracotta (the Woolworth)
                {0.62, 0.47, 0.43},   // pink granite
                {0.50, 0.50, 0.50},   // grey granite
                {0.19, 0.19, 0.20},   // black granite
                {0.60, 0.63, 0.56},   // celadon terracotta
            };
            return kStone[static_cast<int>((k - 0.40) / 0.60 * 6) % 6] * (0.95 + 0.10 * t);
        }
        case FacadeStyle::Concrete:
        default: {
            const Real k = rng.unit();
            if (k < 0.55) return lerp(Vec3(0.62, 0.62, 0.60), Vec3(0.74, 0.73, 0.70), t);
            static const Vec3 kConcrete[] = {
                {0.86, 0.85, 0.82},   // white precast (432 Park)
                {0.70, 0.65, 0.58},   // warm sand aggregate
                {0.56, 0.60, 0.62},   // cool blue-grey
                {0.40, 0.40, 0.40},   // dark charcoal panel
            };
            return kConcrete[static_cast<int>((k - 0.55) / 0.45 * 4) % 4] * (0.95 + 0.10 * t);
        }
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

int litTintIndex(const Vec3& worldPos, bool curtainWall) {
    // A second hash stream (offset anchor) so the tint is independent of the
    // lit/dark choice and the storey occupancy.
    const uint32_t h = positionHash(worldPos + Vec3(0.37, 0.11, 0.53));
    const Real u = static_cast<Real>(h & 0xffu) / 255.0;
    if (curtainWall) return u < 0.62 ? 1 : u < 0.90 ? 2 : 3;
    return u < 0.60 ? 4 : u < 0.85 ? 5 : u < 0.95 ? 6 : 7;
}

// The palette mesh.frag (and lighting_surface.metal, mesh.wgsl) decode the index with -- keep them in step.
Vec3 litTintOf(int index) {
    switch (index) {
        case 1: return Vec3(1.00, 0.96, 0.88);   // office white
        case 2: return Vec3(0.82, 0.90, 1.00);   // fluorescent blue-white
        case 3: return Vec3(1.00, 0.82, 0.58);   // a warm room
        case 4: return Vec3(1.00, 0.72, 0.42);   // incandescent
        case 5: return Vec3(1.00, 0.86, 0.64);   // cream
        case 6: return Vec3(0.80, 0.88, 1.00);   // a cool room
        case 7: return Vec3(0.72, 1.00, 0.78);   // the odd green-white
        default: return Vec3(1, 1, 1);
    }
}

Vec3 litTint(const Vec3& worldPos, bool curtainWall) { return litTintOf(litTintIndex(worldPos, curtainWall)); }

Vec3 glassGrey() { return Vec3(0.032, 0.073, 0.116); }

bool mechanicalStorey(const BuildingParams& p, int floor) {
    if (p.floors < 30 || p.parkingDecks || floor < 1 || floor >= p.floors) return false;
    if (floor == p.floors - 1 && p.floors >= 40) return true;   // the plant under the roof
    const int every = 15 + static_cast<int>((p.seed >> 7) % 11);
    return floor % every == 0 && floor + 3 < p.floors;
}

  // the old grey material (0.18 0.27 0.34) squared

Vec3 litPaneColour(const Vec3& glassCol, const Vec3& worldPos, bool curtainWall) {
    const int idx = litTintIndex(worldPos, curtainWall);
    auto chan = [](Real c, int bit) {
        int b = static_cast<int>(std::lround(std::clamp(c, Real(0), Real(1)) * 255.0)) & ~1;
        return static_cast<Real>(b | bit) / 255.0;
    };
    return Vec3(chan(glassCol.x, idx & 1), chan(glassCol.y, (idx >> 1) & 1), chan(glassCol.z, (idx >> 2) & 1));
}

RenderMaterial materialFor(PartId id, const Vec3& wallColor) {
    RenderMaterial m;
    switch (id) {
        case PartId::Glass:
            // Front face only: from inside a streamed interior the facade's
            // pane is not there, and the interior's own clear pane shows out.
            // White: the pane's vertex colour is its glass (glassGrey, a curtain wall's tint).
            m.albedo = {1.0, 1.0, 1.0}; m.metallic = 0.9f; m.roughness = 0.08f;
            m.flags |= RenderMaterial::FLAG_FRONT_ONLY; break;
        case PartId::GlassLit:
            // Indistinguishable from Glass by DAY — the lit third of the
            // windows must not read as a checkerboard at noon. Night is the
            // loader's NightGlow tag raising emission, not this material —
            // and the pane's vertex colour is its TINT (litTint), which the
            // FLAG_EMISSIVE_VERTEX_TINT shader path applies to the emission
            // only, so the day glass stays the one glass colour.
            // The vertex colour PACKS the glass and the lit tint (litPaneColour): the interior-mapped path in
            // mesh.frag unpacks both, so a lit pane is its building's glass by day. This albedo is the fallback
            // for a renderer that does not unpack (the default grey).
            m.albedo = glassGrey(); m.metallic = 0.9f; m.roughness = 0.08f;
            m.flags |= RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT | RenderMaterial::FLAG_FRONT_ONLY; break;
        case PartId::Beacon:
            // The lamp's LENS: red glass — a dark red by day with a tight
            // specular so it catches the sky; the loader's NightGlow +
            // BeaconBlink drive its emission, tinted per vertex.
            m.albedo = {0.42, 0.05, 0.03}; m.metallic = 0.0f; m.roughness = 0.16f;
            m.flags |= RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT; break;
        case PartId::BeaconGlow:
            // The lamp's bulb: a translucent sphere (the transparent pass),
            // near-invisible by day (no albedo to speak of, no emission), a
            // red glow at night — the "emissive quality" the painted box
            // alone lacked (Glenn, 2026-09-14).
            m.albedo = {0.02, 0.02, 0.02}; m.metallic = 0.0f; m.roughness = 1.0f;
            m.opacity = 0.55f;
            m.flags |= RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT; break;
        case PartId::LitBand:
            // Lit dressing (crown bands, signage, podium uplights): the lit
            // glass look by day, emission tinted per vertex at night, no room.
            m.albedo = {0.16, 0.20, 0.26}; m.metallic = 0.6f; m.roughness = 0.2f;
            m.flags |= RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT; break;
        case PartId::Furniture:
            // Neutral satin: the piece's colour rides the vertex (a dark monitor stays dark, a white carcass stays
            // white), with a little of drywall's self-light so a room without a staged light is not black.
            m.albedo = {1.0, 1.0, 1.0}; m.metallic = 0.0f; m.roughness = 0.55f;
            m.emission = {0.03, 0.03, 0.03};
            break;
        case PartId::FurnitureWood:
            m.albedo = {1.0, 1.0, 1.0}; m.metallic = 0.0f; m.roughness = 0.5f;
            m.setSurface(RenderMaterial::Surface::WoodGrain);
            m.emission = {0.02, 0.02, 0.02};
            break;
        case PartId::FurnitureFabric:
            m.albedo = {1.0, 1.0, 1.0}; m.metallic = 0.0f; m.roughness = 0.92f;
            m.setSurface(RenderMaterial::Surface::Fabric);
            m.emission = {0.02, 0.02, 0.02};
            break;
        case PartId::FurnitureMetal:
            m.albedo = {1.0, 1.0, 1.0}; m.metallic = 0.85f; m.roughness = 0.28f;
            break;
        case PartId::FurnitureCeramic:
            m.albedo = {1.0, 1.0, 1.0}; m.metallic = 0.0f; m.roughness = 0.12f;
            m.emission = {0.02, 0.02, 0.02};
            break;
        case PartId::GlassClear:
            // Clear glass: a faint blue, a sharp fresnel, most of what is
            // behind it coming through (the transparent pass), both faces.
            // REFLECTIVE (Glenn, 2026-10-02): a little metal and a mirror-smooth face, so a pane catches the room
            // and the sky instead of vanishing -- still mostly see-through.
            m.albedo = {0.55, 0.66, 0.74}; m.metallic = 0.35f; m.roughness = 0.03f;
            m.opacity = 0.24f; break;
        case PartId::BeaconHaze:
            // The bulb's corona: larger, fainter, the soft red halo.
            m.albedo = {0.02, 0.02, 0.02}; m.metallic = 0.0f; m.roughness = 1.0f;
            m.opacity = 0.22f;
            m.flags |= RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT; break;
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
            // Interior DRYWALL (ADR-0080; device: "an interior dry wall
            // material for the ceiling and walls"): smooth warm-white
            // painted board -- deliberately NO procedural surface, drywall
            // is flat. The faint self-light stays so a room with no staged
            // light never goes black.
            m.albedo = {0.86, 0.84, 0.80}; m.metallic = 0.0f; m.roughness = 0.85f;
            // 0.22, not 0.08: an omni room light grazes a ceiling (no GI
            // bounce in the renderer), so drywall carries the bounce
            // itself -- night-test measured, walls bright / ceiling dark.
            m.emission = m.albedo * 0.22;
            break;
        case PartId::InteriorFloor:
            // Interior flooring BASE recipe (wood). The finish VARIES per
            // building via floorFinishFor (dark walnut / light oak /
            // stone tile -- device: "all of the flooring in all of the
            // buildings are the same").
            m.albedo = {0.47, 0.34, 0.22}; m.metallic = 0.0f; m.roughness = 0.55f;
            m.setSurface(RenderMaterial::Surface::WoodSiding);
            m.emission = m.albedo * 0.06;
            break;
        case PartId::InteriorFloorTile:
            // Stone tile: the Concrete bake, pale and polished (one of the
            // floorFinishFor finishes; its own part -- see the header).
            m.albedo = {0.62, 0.61, 0.58}; m.metallic = 0.0f; m.roughness = 0.35f;
            m.setSurface(RenderMaterial::Surface::Concrete);
            m.emission = m.albedo * 0.05;
            break;
        case PartId::InteriorFloorMarble:
            // Veined polished stone (its own bake).
            m.albedo = {0.80, 0.78, 0.76}; m.metallic = 0.0f; m.roughness = 0.25f;
            m.setSurface(RenderMaterial::Surface::Marble);
            m.emission = m.albedo * 0.05;
            break;
        case PartId::InteriorFloorCarpet:
            // Soft cut-pile: warm greige, dead matte.
            m.albedo = {0.62, 0.58, 0.52}; m.metallic = 0.0f; m.roughness = 0.95f;
            m.setSurface(RenderMaterial::Surface::Carpet);
            m.emission = m.albedo * 0.05;
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

// ROOM UVs on a pane (the interior mapping; Glenn's walk 2026-09-14: "the
// rooms are stretched on the ground floor because the windows are tall"):
// u counts 3 m rooms along the face, v is the fraction of the STOREY, so a
// floor-to-ceiling pane shows a storey-high room instead of one stretched
// to the pane. The storey height rides in the tangent's length (the shader
// reads it back to put the room box in metres). Rewrites the vertices from
// `v0` on — call right after the pane's quad (and its lunette) went in.
static void roomUV(RenderMesh& m, std::size_t v0, const FaceRect& fr) {
    const Real sh = std::max(Real(1.0), fr.height);
    for (std::size_t i = v0; i < m.vertices.size(); ++i) {
        Vertex& vt = m.vertices[i];
        const Vec3 rel = vt.position - fr.bl;
        vt.u = static_cast<float>(dot(rel, fr.h) / 3.0);
        vt.v = static_cast<float>(dot(rel, fr.v) / sh);
        vt.tangent = fr.h * sh;
    }
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

void emitSoftBox(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color, uint32_t seed) {
    RenderMesh& mesh = partMesh(out, part);
    const Vec3 h = s.size * 0.5;
    const Vec3 c = s.center();
    const Real rad = std::min<Real>(0.2, 0.45 * std::min({h.x, h.y, h.z}));
    auto lin = [](Real v) { return std::pow(std::max<Real>(v, 0), Real(2.2)); };
    const Vec3 light(lin(color.x) * 1.1, lin(color.y) * 1.1, lin(color.z) * 1.1), dark = light * 0.3;
    // seed 0: from where it stands, so a lot's other draws are not disturbed
    if (seed == 0)
        seed = static_cast<uint32_t>(static_cast<int64_t>(std::floor(c.x * 8)) * 73856093LL ^
                                     static_cast<int64_t>(std::floor(c.z * 8)) * 19349663LL) | 1u;
    const Noise noise(seed);
    // about 30 cm cells, so the lumps have vertices to show on
    auto cells = [&](const Vec3& axis) {
        const Real extent = 2.0 * (std::fabs(axis.x) * h.x + std::fabs(axis.y) * h.y + std::fabs(axis.z) * h.z);
        return std::clamp(static_cast<int>(std::ceil(extent / 0.3)), 2, 8);
    };
    // local coords: x along axis[0], y up, z along axis[2]; each face a grid over [-1, 1]^2
    struct Face { Vec3 n, a, b; };
    const Face faces[5] = {{{0, 1, 0}, {1, 0, 0}, {0, 0, 1}}, {{1, 0, 0}, {0, 0, 1}, {0, 1, 0}}, {{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                           {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}}, {{0, 0, -1}, {1, 0, 0}, {0, 1, 0}}};
    for (const Face& f : faces) {
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        const std::size_t firstIndex = mesh.indices.size();
        const int NU = cells(f.a), NV = cells(f.b);
        for (int j = 0; j <= NV; ++j)
            for (int i = 0; i <= NU; ++i) {
                const Real u = -1 + 2.0 * i / NU, v = -1 + 2.0 * j / NV;
                const Vec3 p = f.n + f.a * u + f.b * v;                      // on the unit cube
                const Vec3 P(p.x * h.x, p.y * h.y, p.z * h.z);                  // on the box (local)
                const Vec3 inner(std::clamp(P.x, -h.x + rad, h.x - rad), std::clamp(P.y, -h.y + rad, h.y - rad),
                                 std::clamp(P.z, -h.z + rad, h.z - rad));
                Vec3 dir = P - inner;
                dir = dir.lengthSquared() > 1e-12 ? normalize(dir) : f.n;
                Vec3 L = inner + dir * rad;                                       // the rounded box
                const Vec3 W0 = c + s.axis[0] * L.x + s.axis[1] * L.y + s.axis[2] * L.z;
                const Real lump = 0.7 * noise.noise3(W0.x * 2.4, W0.y * 2.4, W0.z * 2.4) +
                                  0.3 * noise.noise3(W0.x * 5.9 + 5.1, W0.y * 5.9, W0.z * 5.9 - 2.7);   // leafy, two scales
                const Real amp = std::min<Real>(0.14, 0.3 * std::min({h.x, h.y, h.z}));
                if (L.y > -h.y + 1e-6) L = L + dir * (amp * lump * 1.6);           // lumps, not below the base
                const Vec3 soft = normalize(dir * 0.5 + normalize(Vec3(L.x / (h.x * h.x), L.y / (h.y * h.y), L.z / (h.z * h.z))) * 0.5);
                const Vec3 W = c + s.axis[0] * L.x + s.axis[1] * L.y + s.axis[2] * L.z;
                const Vec3 N3 = normalize(s.axis[0] * soft.x + s.axis[1] * soft.y + s.axis[2] * soft.z);
                const Real t = std::clamp((L.y + h.y) / (2 * h.y), Real(0), Real(1));
                Vertex vx(W, N3, s.axis[0], 0.0f, 0.0f);
                vx.color = (dark + (light - dark) * (0.15 + 0.85 * t)) * (1.0 + 0.35 * lump);   // mottled: lumps catch light, hollows shade
                mesh.vertices.push_back(vx);
            }
        for (int j = 0; j < NV; ++j)
            for (int i = 0; i < NU; ++i) {
                const uint32_t a = base + j * (NU + 1) + i, b = a + 1, cc = a + (NU + 1), d = cc + 1;
                const Vec3 out3 = mesh.vertices[a].normal;
                for (const auto& t3 : {std::array<uint32_t, 3>{a, b, d}, std::array<uint32_t, 3>{a, d, cc}}) {
                    const Vec3& A = mesh.vertices[t3[0]].position; const Vec3& B = mesh.vertices[t3[1]].position; const Vec3& C3 = mesh.vertices[t3[2]].position;
                    if (dot(cross(C3 - A, B - A), out3) >= 0) mesh.indices.insert(mesh.indices.end(), {t3[0], t3[1], t3[2]});
                    else mesh.indices.insert(mesh.indices.end(), {t3[0], t3[2], t3[1]});
                }
            }
        // Light the LUMPS: blend the smooth mass normal with the lumpy surface's own, so the
        // bumps catch the sun and shade the hollows (the soft normal alone hid them).
        const std::size_t v0 = base, v1 = mesh.vertices.size();
        std::vector<Vec3> geo(v1 - v0, Vec3(0, 0, 0));
        for (std::size_t k = firstIndex; k + 2 < mesh.indices.size(); k += 3) {
            const uint32_t a = mesh.indices[k], b = mesh.indices[k + 1], cc = mesh.indices[k + 2];
            const Vec3 n = cross(mesh.vertices[cc].position - mesh.vertices[a].position,
                                 mesh.vertices[b].position - mesh.vertices[a].position);   // front-facing, as wound above
            for (uint32_t q : {a, b, cc}) geo[q - v0] = geo[q - v0] + n;
        }
        for (std::size_t q = v0; q < v1; ++q) {
            const Vec3& g3 = geo[q - v0];
            if (g3.lengthSquared() < 1e-18) continue;
            Vec3 gn = normalize(g3);
            if (dot(gn, mesh.vertices[q].normal) < 0) gn = gn * -1.0;
            mesh.vertices[q].normal = normalize(gn * 0.6 + mesh.vertices[q].normal * 0.4);
        }
    }
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

// Rear: a residential ground face whose middle bay is a plain SERVICE door (attached buildings' back doors);
// facadeLayout and the emitters read it as Residential everywhere else.
// StairDoor: the wall a stair hugs (Solid: no panes behind the flights) with ONE door, just short of the flight's
// foot -- a campus hall's door onto its quad, whose rear wall is its stair's (stairWallDoorX).
enum class FacadeMode { Residential, Retail, Entrance, Solid, Rear, StairDoor };

// A curtain-wall storey (ADR-0040 Pass B): a continuous glass skin, not punched
// windows — an opaque spandrel band hiding the floor slab, vision glass above,
// and a proud steel mullion/transom grid. This is what a glass tower actually
// is (a skin hung off a frame), and it reads far better than flat panels.
// The panes of a facade land in Glass / GlassLit — or, for the ground storey
// of an enterable building, ALL in GlassClear (the lobby shows through).
static void appendGlassParts(BuildingMesh& out, RenderMesh& glass, RenderMesh& glassLit,
                             bool clearPanes) {
    if (clearPanes) {
        // Clear lobby glass is see-through whatever the tower's tint: its own faint colour (the old grey pane).
        for (RenderMesh* m : {&glass, &glassLit})
            for (Vertex& v : m->vertices) v.color = Vec3(0.18, 0.27, 0.34);
        appendToPart(out, PartId::GlassClear, glass);
        appendToPart(out, PartId::GlassClear, glassLit);
    } else {
        appendToPart(out, PartId::Glass, glass);
        appendToPart(out, PartId::GlassLit, glassLit);
    }
}

namespace {
// The glass by tint (vertex colour on the glass part): 0 keeps the wall-derived grey.
Vec3 curtainGlassColour(uint8_t tint, const Vec3& grey) {
    switch (tint) {
        // The REFLECTANCE (the glass material is white, metallic): what colour the sky comes back in.
        case 1: return Vec3(0.18, 0.32, 0.54);   // blue
        case 2: return Vec3(0.18, 0.42, 0.33);   // green (Lever House)
        case 3: return Vec3(0.42, 0.28, 0.15);   // bronze (Seagram)
        case 4: return Vec3(0.06, 0.07, 0.08);   // smoke, near black
        case 5: return Vec3(0.62, 0.66, 0.70);   // silver, a mirror
        case 6: return Vec3(0.28, 0.36, 0.38);   // clear, pale green-grey
        case 7: return Vec3(0.72, 0.54, 0.18);   // gold (the Toronto bank towers)
        case 8: return Vec3(0.10, 0.38, 0.42);   // teal
        case 9: return Vec3(0.54, 0.31, 0.23);   // copper / rose
        default: return grey;
    }
}
Vec3 curtainMullionColour(uint8_t tone) {
    switch (tone) {
        case 1: return Vec3(0.34, 0.25, 0.16);   // bronze
        case 2: return Vec3(0.06, 0.06, 0.07);   // black
        case 3: return Vec3(0.72, 0.74, 0.77);   // silver
        case 4: return Vec3(0.88, 0.88, 0.86);   // white
        default: return Vec3(0.34, 0.36, 0.40);  // steel
    }
}
}  // namespace

// How far a curtain wall's glass sits behind its mullion grid (the interior's skin is drawn just behind it).
constexpr Real kCurtainGlassIn = 0.10;

namespace {
// The LOUVRE BAND of a mechanical storey across one face: a dark recess with horizontal blades proud of it (the
// intake and exhaust of the plant room behind), a solid sill and head. `full` adds the blades; the far tier keeps
// the dark band and its three lines.
void emitLouvreBand(BuildingMesh& out, const FaceRect& fr, const Vec3& bladeCol, bool full) {
    const Real W = fr.width, H = fr.height;
    if (W < 0.5 || H < 0.5) return;
    RenderMesh m;
    const Real sill = std::min(Real(0.5), H * 0.12), head = std::min(Real(0.35), H * 0.08);
    const Vec3 in = fr.n * -0.18;
    const Vec3 dark(0.13, 0.13, 0.14);
    // The solid sill and head, flush with the face.
    MeshBuilder::emitQuad(m, fr.at(0, 0), fr.at(W, 0), fr.at(W, sill), fr.at(0, sill), fr.n, bladeCol * 0.7);
    MeshBuilder::emitQuad(m, fr.at(0, H - head), fr.at(W, H - head), fr.at(W, H), fr.at(0, H), fr.n, bladeCol * 0.7);
    // The recess and its reveals.
    MeshBuilder::emitQuad(m, fr.at(0, sill) + in, fr.at(W, sill) + in, fr.at(W, H - head) + in, fr.at(0, H - head) + in,
                          fr.n, dark);
    MeshBuilder::emitQuad(m, fr.at(0, sill), fr.at(W, sill), fr.at(W, sill) + in, fr.at(0, sill) + in, fr.v, dark);
    MeshBuilder::emitQuad(m, fr.at(0, H - head) + in, fr.at(W, H - head) + in, fr.at(W, H - head), fr.at(0, H - head),
                          fr.v * -1.0, dark);
    const Real span = H - sill - head;
    const int blades = full ? std::max(3, static_cast<int>(span / 0.32)) : 3;
    for (int k = 0; k < blades; ++k) {
        // Each blade leans out and down: a strip from the recess up to near the face.
        const Real y = sill + span * (k + 0.75) / blades;
        const Vec3 top0 = fr.at(0, y) + in * 0.15, top1 = fr.at(W, y) + in * 0.15;
        const Vec3 bot0 = fr.at(0, y - 0.16) + in * 0.9, bot1 = fr.at(W, y - 0.16) + in * 0.9;
        MeshBuilder::emitQuad(m, bot0, bot1, top1, top0, normalize(fr.n + fr.v * -0.8), bladeCol);
    }
    appendToPart(out, PartId::Detail, m);
}
}  // namespace 
void emitCurtainWallRect(BuildingMesh& out, const FaceRect& fr,
                         const Vec3& /*wallColor: the glass is its own colour now*/,
                         FacadeDetail detail = FacadeDetail::Full,
                         bool clearPanes = false, const CurtainStyle& cs = CurtainStyle{}) {
    Real fh = fr.height, W = fr.width;
    if (W < 0.5 || fh < 0.5) return;
    RenderMesh glass, glassLit, mull;
    Vec3 glassCol = curtainGlassColour(cs.glassTint, glassGrey());
    Vec3 spandrelCol = glassCol * 0.45;          // opaque shadow-box band
    Vec3 mullCol = curtainMullionColour(cs.mullionTone);
    Real spandrelH = cs.spandrelH(fh);

    // Glass sits INSET behind the frame plane; the mullion grid is SOLID
    // geometry — front face + side returns back to the glass — so up close it
    // reads as a frame the panes sit in, not a decal (device feedback).
    const Real glassIn = kCurtainGlassIn;        // glass plane behind the grid
    Vec3 gin = fr.n * (-glassIn);
    // A face too NARROW for a pane (a chamfer's first steps, a setback's sliver face): an opaque metal panel the
    // storey's full height, not a stick-thin window (Glenn, 2026-09-30: "some windows are super skinny").
    if (W < 1.0) {
        emitQuad(glass, fr.at(0, 0) + gin, fr.at(W, 0) + gin, fr.at(W, fh) + gin, fr.at(0, fh) + gin, fr.n, spandrelCol);
        appendGlassParts(out, glass, glassLit, clearPanes);
        return;
    }
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
    const int bays = cs.bays(W);
    const Real occupancy = litStoreyOccupancy(fr.at(0, spandrelH));
    for (int b = 0; b < bays; ++b) {
        const Real x0 = W * b / bays, x1 = W * (b + 1) / bays;
        const Vec3 bayAnchor = fr.at((x0 + x1) * 0.5, spandrelH);
        const bool litBay = litOfficeBay(bayAnchor, occupancy);
        RenderMesh& vision = litBay ? glassLit : glass;
        const std::size_t pv0 = vision.vertices.size();
        emitQuad(vision, fr.at(x0, spandrelH) + gin, fr.at(x1, spandrelH) + gin,
                 fr.at(x1, fh) + gin, fr.at(x0, fh) + gin, fr.n,
                 litBay ? litPaneColour(glassCol, bayAnchor, true) : glassCol);
        roomUV(vision, pv0, fr);
    }
    // FLAT (LOD1): the spandrel band + vision pane carry the curtain-wall read
    // at distance; the solid mullion lattice is the expensive half — skip it.
    if (detail == FacadeDetail::Flat) {
        appendGlassParts(out, glass, glassLit, clearPanes);
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
    // FINS: a deep vertical blade every `fins` bays (111 W 57th's ribs, a modern glass tower's shading fins),
    // standing well proud of the grid -- they catch the light and break a glass face into strips.
    if (cs.fins > 0) {
        const Real finOut = 0.42, finW = 0.12;
        const Vec3 fo = fr.n * finOut;
        for (int b = 0; b <= bays; b += cs.fins) {
            const Real x = std::min(std::max(b * W / bays, finW * 0.5), W - finW * 0.5);
            const Real a0 = x - finW * 0.5, a1 = x + finW * 0.5;
            emitQuad(mull, fr.at(a0, 0) + fo, fr.at(a1, 0) + fo, fr.at(a1, fh) + fo, fr.at(a0, fh) + fo, fr.n, mullCol);
            emitQuad(mull, fr.at(a0, 0) + outv, fr.at(a0, 0) + fo, fr.at(a0, fh) + fo, fr.at(a0, fh) + outv, fr.h * -1, mullCol);
            emitQuad(mull, fr.at(a1, 0) + outv, fr.at(a1, 0) + fo, fr.at(a1, fh) + fo, fr.at(a1, fh) + outv, fr.h, mullCol);
        }
    }
    appendGlassParts(out, glass, glassLit, clearPanes);
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
    Real rise = 0;            // arch rise; 0 = a flat head. `head` is the APEX,
                              // so the springline is head - rise and the
                              // sill..head box is the arch's bounding box.
    bool entrance = false;    // this opening is the door
    bool shopDoor = false;    // ...a SHOP's own street door (buildings: shops), not the building's entrance
    bool backDoor = false;    // ...the building's rear service door (FacadeMode::Rear)
};
// A SHOP on a storefront face (Glenn, 2026-10-01: "I'm still waiting to see these small shops"): bays b0..b1
// (inclusive), its door in bay `door`, its trade.
struct ShopUnit { int b0 = 0, b1 = 0, door = 0; uint8_t type = 0; };
constexpr Real kShopHead = 3.45;   // a storefront's glazing head; its fascia sign sits just above
struct FacadeLayout {
    std::vector<ShopUnit> shops;   // the storefront face's shops (empty elsewhere)
    int bays = 1;
    Real bw = 0;
    bool retailish = false;
    std::vector<BayOpening> open;
};

// How many windows make a group on this face (M3): the building's windowGroup on its punched residential faces
// with flat heads; 1 on storefronts, clerestories and arched windows.
static int windowGroupOf(const BuildingParams& p, FacadeMode mode, bool retailish) {
    if (mode != FacadeMode::Residential || retailish || p.window.head != OpeningStyle::Head::Flat) return 1;
    return std::clamp(static_cast<int>(p.windowGroup), 1, 3);
}

static FacadeLayout facadeLayout(const FaceRect& fr, FacadeMode mode,
                                 const BuildingParams& p) {
    FacadeLayout L;
    const bool rear = mode == FacadeMode::Rear;
    if (rear) mode = FacadeMode::Residential;
    const bool stairDoor = mode == FacadeMode::StairDoor;
    if (stairDoor) mode = FacadeMode::Solid;
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
        // An OFFICE storey's window (buildings M3): a building of office-height storeys (3.6 m and up, M1) glazes
        // its storey, not a flat's 1.5 m opening -- sill at desk height, head under the ceiling void.
        if (!L.retailish && p.floorHeight >= 3.6) {
            sill = 0.75;
            head = fh - std::max(Real(0.55), fh * 0.15);
        }
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
        // A STOREFRONT is glazed nearly bay to bay (slim piers), its head under the shop's fascia sign -- not a
        // flat's portrait window: on a tall lobby storey those read as slits (Glenn's first look at the shops).
        if (L.retailish) {
            winW = std::max(Real(0.8), bw - 0.3);
            head = std::min(fh - 0.4, kShopHead);
        }
        margin = (bw - winW) * 0.5;
    }
    if (head <= sill) { head = fh * 0.75; sill = fh * 0.2; }

    const int centreBay = L.bays / 2;
    const int group = windowGroupOf(p, mode, L.retailish);
    // THE SHOPS of a storefront face: its bays grouped into units of two or three, each with its own door, the
    // building's entrance bay and its neighbours kept for the lobby. Seeded by the face's own corner, so the
    // facade, its far tier and the interior read the same shops.
    if (L.retailish && p.groundRetail && !p.campus && p.walkableGround && L.bays >= 2 && mode != FacadeMode::Solid) {   // (a campus building: no shops)
        uint32_t h = positionHash(fr.bl + Vec3(0.13, 0.0, 0.29)) ^ static_cast<uint32_t>(p.seed);
        auto next = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return h; };
        const int lobby = mode == FacadeMode::Entrance ? centreBay : -100;
        auto inLobby = [&](int b) { return b >= lobby - 1 && b <= lobby + 1; };
        int b = 0;
        while (b < L.bays) {
            if (inLobby(b)) { ++b; continue; }
            int run = 0;
            while (b + run < L.bays && !inLobby(b + run)) ++run;
            // Cut this run into units of two or three bays (a single bay joins its neighbour).
            int s = b;
            while (s < b + run) {
                const int left = b + run - s;
                int g = left <= 3 ? left : 2 + static_cast<int>(next() % 2u);
                if (left - g == 1) g = (g == 2 ? 3 : 2);
                if (g < 2) {   // a lone bay: grow the previous unit over it, else leave it to the lobby
                    if (!L.shops.empty() && L.shops.back().b1 == s - 1) L.shops.back().b1 = s;
                    break;
                }
                ShopUnit u;
                u.b0 = s; u.b1 = s + g - 1;
                u.door = g == 3 ? s + 1 : s + static_cast<int>(next() % 2u);
                u.type = pickTrade(next(), g);   // (trades.h: weighted, among those its bays fit)
                L.shops.push_back(u);
                s += g;
            }
            b += run;
        }
    }
    auto isShopDoor = [&](int b) {
        for (const ShopUnit& u : L.shops) if (u.door == b) return true;
        return false;
    };
    for (int b = 0; b < L.bays; ++b) {
        BayOpening o;
        o.x0 = b * bw; o.x1 = (b + 1) * bw;
        o.entrance = (mode == FacadeMode::Entrance && b == centreBay);
        if (isShopDoor(b)) { o.entrance = true; o.shopDoor = true; }
        // the back door: the middle bay, or with a fire escape the END bay, so the stair has the rest of the wall
        // for a walkable pitch
        if (rear && b == (p.fireEscape && L.bays >= 3 ? 0 : centreBay) && fr.width >= 1.6) {
            o.entrance = true;
            o.backDoor = true;
        }
        // the stair wall's one door: the bay holding stairWallDoorX
        if (stairDoor && fr.width >= 1.6 &&
            b == std::clamp(static_cast<int>(stairWallDoorX(fr.width, p) / bw), 0, L.bays - 1)) {
            o.entrance = true;
            o.backDoor = true;
        }
        o.wx0 = o.x0 + margin; o.wx1 = o.x1 - margin;       // window/opening span
        if (group > 1) {
            // GROUPED windows (M3): a slim mullion inside the group, a broad pier at its ends.
            const bool first = b % group == 0, last = b % group == group - 1 || b == L.bays - 1;
            const Real outer = std::max(Real(0.3), margin * 1.25);
            o.wx0 = o.x0 + (first ? outer : 0.08);
            o.wx1 = o.x1 - (last ? outer : 0.08);
        }
        o.sill = o.entrance ? 0.0 : sill;
        o.head = o.entrance ? std::min(human::DOOR_HEIGHT, fh - 0.3) : head;
        if (o.entrance) {
            Real dw = std::min(human::DOOR_WIDTH, bw - 0.4);
            Real cx = (o.x0 + o.x1) * 0.5;
            o.wx0 = cx - dw * 0.5; o.wx1 = cx + dw * 0.5;
        }
        // TOO NARROW TO BE A WINDOW (Glenn: "some windows are super skinny"): a face narrower than a window
        // module, or an opening squeezed under 0.55 m, is BLANK -- solid wall. Encoded as a zero-height opening at
        // mid-storey, so every reader (the facade, its far tier, the inner wall) fills the bay with wall unchanged.
        // ...and a BIG BOX's walls are blank but for its doors (the sign and the entry glazing dress the front), as is
        // a campus hall's stair wall but for its quad door (no pane behind the flights)
        if (!o.entrance && (fr.width < 1.2 || o.wx1 - o.wx0 < 0.55 || p.bigBox || stairDoor)) {
            const Real mid = (o.x0 + o.x1) * 0.5;
            o.wx0 = o.wx1 = mid;
            o.sill = o.head = fh * 0.5;
            o.rise = 0;
            L.open.push_back(o);
            continue;
        }
        // THE ARCH, decided here and nowhere else. It used to be worked out
        // inside the full facade emitter, so the inner wall and the flat
        // tier cut a plain rectangle: an arched window was square from
        // inside and square from across the street (Glenn, 2026-09-15).
        if (!o.entrance && mode == FacadeMode::Residential &&
            p.window.head != OpeningStyle::Head::Flat) {
            const Real span = o.wx1 - o.wx0;
            Real r = p.window.head == OpeningStyle::Head::Round
                         ? span * 0.5
                         : span * std::min(Real(0.5), std::max(Real(0.12), p.window.archRise));
            if (o.head - r < o.sill + 0.35) r = 0;   // too squat to read as an arch
            o.rise = r;
        }
        L.open.push_back(o);
    }
    return L;
}

// THE GROUND STOREY'S MODE on edge `e` -- what the exterior draws there, and so where the shops and their doors
// are: the entrance edge is the Entrance, a vehicle-bay front or the stair's wall carries no openings to speak of
// (Solid), a storefront edge that does not face the street (retailStreetOnly) is a plain wall. One answer for the
// exterior, its far tier, the interior's colliders and its shops.
}  // namespace (the two below are declared in the header)

bool partyEdge(const Poly2& plan, const BuildingParams& params, std::size_t e) {
    const std::size_t n = plan.size();
    if (params.partyWalls == 0 || n < 3) return false;
    const Vec2 a = plan[e % n], b = plan[(e + 1) % n];
    const Vec2 d = b - a;
    const Real len = d.length();
    if (len < 1e-6) return false;
    const Vec2 nrm(d.y / len, -d.x / len);   // CCW plan: outward
    for (int k = 0; k < std::min<int>(params.partyWalls, 2); ++k) {
        const Vec2 N = params.partyN[k];
        if (dot(nrm, N) < 0.98) continue;
        if (std::fabs(dot(a, N) - params.partyAt[k]) < 0.35 && std::fabs(dot(b, N) - params.partyAt[k]) < 0.35)
            return true;
    }
    return false;
}

// Where along a stair's wall (from its first corner, CCW) the stair-wall door stands: a metre short of the flight's
// foot. interiorLayout centres the flight on the wall -- its run (from the ground storey's risers) plus a metre's
// arrival -- so this needs only the wall's length and the params.
Real stairWallDoorX(Real wallLen, const BuildingParams& params) {
    const int risers = std::max(3, static_cast<int>(std::ceil(params.groundHeight / 0.28)));
    const Real wellLen = (risers - 1) * Real(0.25) + 1.0;
    return std::max(Real(0.9), (wallLen - wellLen) * 0.5 - 1.0);
}

bool campusQuadDoor(const Poly2& planIn, const BuildingParams& params, std::size_t entranceEdge, Vec2& centre, Vec2& outward) {
    if (!params.campus || !params.backDoor || planIn.size() < 3) return false;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t re = rearEdgeOf(plan, params);
    if (re >= plan.size()) return false;
    const Vec2 a = plan[re], b = plan[(re + 1) % plan.size()];
    const Real W = (b - a).length();
    if (W < 1.6) return false;
    const Vec2 d = (b - a) * (1.0 / W);
    outward = Vec2(d.y, -d.x);
    const InteriorLayout il = interiorLayout(plan, params, entranceEdge);
    const int bays = std::max(1, static_cast<int>(std::lround(W / std::max(params.bayWidth, Real(0.5)))));
    const Real bw = W / bays;
    int bay = bays / 2;   // the rear face's middle bay (FacadeMode::Rear)
    if (il.hasStair && il.edge == re) bay = std::clamp(static_cast<int>(stairWallDoorX(W, params) / bw), 0, bays - 1);
    centre = a + d * ((bay + 0.5) * bw);
    return true;
}

std::size_t rearEdgeOf(const Poly2& plan, const BuildingParams& params) {
    std::size_t best = plan.size();
    Real bestLen = 0;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const Vec2 d = plan[(i + 1) % plan.size()] - plan[i];
        const Real len = d.length();
        if (len < 2.0 || len <= bestLen) continue;
        const Vec2 nrm(d.y / len, -d.x / len);
        if (nrm.x * params.faceDir.x + nrm.y * params.faceDir.z > -0.7) continue;
        if (partyEdge(plan, params, i)) continue;
        best = i;
        bestLen = len;
    }
    return best;
}

namespace {

static FacadeMode groundModeFor(const Poly2& plan, const BuildingParams& params, std::size_t e,
                                std::size_t entranceEdge) {
    if (e == entranceEdge && params.groundBays > 0) return FacadeMode::Solid;
    // A PARTY WALL has nothing in it: the neighbour stands against it.
    if (e != entranceEdge && partyEdge(plan, params, e)) return FacadeMode::Solid;
    const FacadeMode base = params.solidFacade ? FacadeMode::Solid
                          : params.groundRetail ? FacadeMode::Retail
                                                : FacadeMode::Residential;
    FacadeMode mode = (e == entranceEdge && params.walkableGround) ? FacadeMode::Entrance : base;
    if (params.openDoorway && e != entranceEdge) {
        const InteriorLayout il = interiorLayout(plan, params, entranceEdge);
        // a campus hall's stair hugs its rear wall -- the one facing the quad: blank still, but with its quad door
        if (il.hasStair && e == il.edge)
            return params.campus && params.backDoor && e == rearEdgeOf(plan, params) ? FacadeMode::StairDoor : FacadeMode::Solid;
    }
    if (mode == FacadeMode::Retail && params.retailStreetOnly) {
        const Vec2 a = plan[e], b = plan[(e + 1) % plan.size()];
        const Vec2 d = normalize(b - a);
        const Vec2 nrm(d.y, -d.x);
        if (nrm.x * params.faceDir.x + nrm.y * params.faceDir.z < 0.35) mode = FacadeMode::Residential;
    }
    // The BACK DOOR: the rear face's middle bay, out to the yard (attached buildings).
    if (params.backDoor && params.walkableGround && e != entranceEdge && mode != FacadeMode::Entrance &&
        e == rearEdgeOf(plan, params))
        mode = FacadeMode::Rear;
    return mode;
}

// The ARC of an arched head, left springer to right springer, in FACE space
// (x along the wall, y up). Shared by the full facade, the inner wall and
// the flat tier so all three build the SAME head. Returns the sample count
// (0 for a flat head) and, optionally, the arc's centre.
static int openingArc(const BayOpening& o, Vec2* out, int n, Vec2* centreOut = nullptr) {
    if (o.rise <= 0 || n < 1) return 0;
    const Real span = o.wx1 - o.wx0, cx = (o.wx0 + o.wx1) * 0.5;
    const Real R = (o.rise * o.rise + span * span * 0.25) / (2 * o.rise);
    const Real Cy = o.head - R, ysp = o.head - o.rise;
    const Real thL = std::atan2(ysp - Cy, o.wx0 - cx);
    const Real thR = std::atan2(ysp - Cy, o.wx1 - cx);
    for (int k = 0; k <= n; ++k) {
        const Real th = thL + (thR - thL) * (Real(k) / n);
        out[k] = Vec2(cx + R * std::cos(th), Cy + R * std::sin(th));
    }
    if (centreOut) *centreOut = Vec2(cx, Cy);
    return n + 1;
}

// The FLAT emitter (LOD1, city-render-perf R2): the same layout, the cheapest
// honest drawing of it — one quad per wall, one flat pane per opening riding
// Interior wall inset for enterable buildings: at least the wall build-up,
// and DEEPER than the capsule radius (0.3) + the camera near plane, so a
// player pressed against the collider cannot poke the near plane through
// the inner skin into the one-sided cavity (device: "you could clip
// through the walls to look outside").
Real interiorInset(const BuildingParams& p) {
    // 0.42 measured insufficient in play ("I can still clip through"): the
    // near plane's CORNER reach at a wide FOV is ~0.2, so radius 0.3 + 0.2
    // + margin.
    return std::max(p.wallThickness, Real(0.55));
}

// The INNER face of an exterior ground-storey wall (enterable buildings,
// ADR-0080): the SAME FacadeLayout as the outside, drawn at -thick facing the
// room, so from inside you see wall around every opening instead of the sky
// through a one-sided skin. Non-door openings also get an inward-facing pane
// (the same lit/dark hash as the outside pane) so windows read as glass, not
// holes; the door bay's aperture stays open -- its reveal is the passage.
// Quads are DOUBLE-sided (the back face closes the cavity if the camera ever
// does get in), and each wall extends past both ends by the inset so
// adjacent edges' planes overlap and the plan CORNERS close (device: "the
// interior didn't have corners where the walls met").
// The painted interior SKIN behind a curtain wall (no openings to cut): one
// quad per plan edge on the plan INSET by `thick` — consecutive inset edges
// share their corners (offsetPolygonEdges re-intersects the offset lines),
// so the room's corners close instead of leaving a `thick`-wide slot to see
// the sky through (Glenn: "the skyscraper interiors don't have corners").
static void emitInsetSkin(BuildingMesh& out, const Poly2& plan, std::size_t edge, Real y0, Real h,
                          Real thick, const Vec3& paint, bool bothSides,
                          PartId part = PartId::Interior) {
    if (plan.size() < 3) return;
    const Poly2 inner = offsetPolygonEdges(plan, std::vector<Real>(plan.size(), -thick));
    if (inner.size() != plan.size()) return;
    const std::size_t e = edge % plan.size();
    const Vec2 a = inner[e], b = inner[(e + 1) % inner.size()];
    const Vec2 d = b - a;
    const Real len = d.length();
    if (len < 1e-6) return;
    const Vec2 nOut(d.y / len, -d.x / len);   // CCW plan: outward is the right normal
    RenderMesh skin;
    emitQuad(skin, Vec3(a.x, y0, a.y), Vec3(b.x, y0, b.y), Vec3(b.x, y0 + h, b.y),
             Vec3(a.x, y0 + h, a.y), Vec3(-nOut.x, 0, -nOut.y), paint);
    if (bothSides)
        emitQuad(skin, Vec3(a.x, y0, a.y), Vec3(b.x, y0, b.y), Vec3(b.x, y0 + h, b.y),
                 Vec3(a.x, y0 + h, a.y), Vec3(nOut.x, 0, nOut.y), paint);
    appendToPart(out, part, skin);
}

// The INSIDE of a curtain wall's mullion grid (Glenn's third walk,
// 2026-09-15: "the interior was not using the same windows as the
// exterior" — the inner face was one clear sheet, the outside a lattice of
// 1.6 m bays). The same bay count and transom lines as emitCurtainWallRect,
// as bars standing proud of the inner glass into the room, so a bay reads
// as the same bay from both sides.
static void emitInnerCurtainGrid(BuildingMesh& out, const FaceRect& fr, Real inset, Real spandrelH,
                                 const CurtainStyle& cs = CurtainStyle{}) {
    RenderMesh mull;
    const Vec3 mullCol = curtainMullionColour(cs.mullionTone);
    const Real W = fr.width, fh = fr.height, mw = 0.09, proud = 0.06;
    if (W < 2.0 * inset + 0.5 || fh < 0.5) return;
    const Vec3 gin = fr.n * -inset;               // the inner glass plane
    const Vec3 inv = fr.n * -(inset + proud);     // the bar's room face
    // Flat bars (one quad each, no cheeks): a streamed window of five storeys
    // carries four faces of them, and the per-window census is a gate.
    (void)gin;
    auto bar = [&](Real a0, Real b0, Real a1, Real b1, bool vertical) {
        (void)vertical;
        if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
        emitQuad(mull, fr.at(a0, b0) + inv, fr.at(a1, b0) + inv, fr.at(a1, b1) + inv, fr.at(a0, b1) + inv,
                 fr.n * -1.0, mullCol);
    };
    const int bays = cs.bays(W);   // emitCurtainWallRect's rule
    for (int b = 0; b <= bays; ++b) {
        const Real x = std::min(std::max(b * W / bays, inset + mw * 0.5), W - inset - mw * 0.5);
        bar(x - mw * 0.5, 0, x + mw * 0.5, fh, true);
    }
    for (Real ty : {spandrelH, fh - 0.04}) {
        bar(inset, std::max(Real(0), ty - mw * 0.5), W - inset, std::min(fh, ty + mw * 0.5), false);
    }
    appendToPart(out, PartId::Detail, mull);
}

void emitInnerWallRect(BuildingMesh& out, const FaceRect& fr,
                       const FacadeLayout& L, Real thick,
                       const Vec3& /*wallColor*/, const Poly2& plan,
                       const Vec3& paint, bool curtainWall, bool clearPanes = false,
                       Real revealFrom = 0.0) {
    RenderMesh wall, glass, glassLit;
    const Vec3 in = fr.n * -thick;
    const Vec3 nIn = fr.n * -1.0;
    const Vec3 icol = paint;   // per-building interior paint (walls only)
    // The MITRE ends (Glenn: "the interiors don't have corners"): the wall's
    // end pieces run to the INSET polygon's corners, where the neighbouring
    // edge's inner wall ends too. The old closers extended past the edge
    // only where that stayed inside the plan — which at a convex corner it
    // never does — leaving a `thick`-wide slot at every room corner.
    Vec3 ia = fr.at(0, 0) + in, ib = fr.at(fr.width, 0) + in;
    {
        Poly2 ccw = plan;
        ensureCCW(ccw);
        const Poly2 inner = offsetPolygonEdges(ccw, std::vector<Real>(ccw.size(), -thick));
        for (std::size_t e = 0; e < ccw.size() && inner.size() == ccw.size(); ++e) {
            if (std::fabs(ccw[e].x - fr.bl.x) > 1e-4 || std::fabs(ccw[e].y - fr.bl.z) > 1e-4) continue;
            const Vec2 a2 = inner[e], b2 = inner[(e + 1) % inner.size()];
            ia = Vec3(a2.x, fr.bl.y, a2.y);
            ib = Vec3(b2.x, fr.bl.y, b2.y);
            break;
        }
    }
    auto q = [&](RenderMesh& m, Real a0, Real b0, Real a1, Real b1,
                 const Vec3& col, const Vec3& off) {
        if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
        const bool atIn = (off - in).lengthSquared() < 1e-12;
        Vec3 A = fr.at(a0, b0) + off, B = fr.at(a1, b0) + off;
        if (atIn && a0 <= 1e-9) A = Vec3(ia.x, A.y, ia.z);
        if (atIn && a1 >= fr.width - 1e-9) B = Vec3(ib.x, B.y, ib.z);
        const Vec3 up(0, b1 - b0, 0);
        emitQuad(m, A, B, B + up, A + up, nIn, col);
        emitQuad(m, A, B, B + up, A + up, fr.n, col);
    };
    for (const BayOpening& o : L.open) {
        q(wall, o.x0, 0, o.wx0, fr.height, icol, in);         // left pier
        q(wall, o.wx1, 0, o.x1, fr.height, icol, in);         // right pier
        q(wall, o.wx0, 0, o.wx1, o.sill, icol, in);           // apron
        q(wall, o.wx0, o.head, o.wx1, fr.height, icol, in);   // over the apex
        // THE SAME HEAD AS THE FACADE. With an arch the wall fills the
        // spandrels between the arc and the apex line, the pane stops at the
        // springline and fans to the arc, and the head reveal follows it.
        Vec2 arc[9];
        Vec2 arcC;
        const int narc = openingArc(o, arc, 8, &arcC);
        const Real ysp = o.head - o.rise;
        for (int k = 0; k + 1 < narc; ++k) {
            const Vec3 A = fr.at(arc[k].x, arc[k].y) + in, B = fr.at(arc[k + 1].x, arc[k + 1].y) + in;
            const Vec3 A2 = fr.at(arc[k].x, o.head) + in, B2 = fr.at(arc[k + 1].x, o.head) + in;
            // TRIANGLES, not a quad. At the apex the arc sample sits exactly
            // at the head, so the spandrel collapses to a sliver: emitQuad
            // picks one winding for both halves of a quad, and the degenerate
            // half then faces the wrong way (lot_building_parts_wind_to_the_
            // engine_convention, 2026-09-15). emitTri orients each on its own.
            for (const Vec3& nrm : {nIn, fr.n}) {
                MeshBuilder::emitTri(wall, A, B, B2, nrm, icol);
                MeshBuilder::emitTri(wall, A, B2, A2, nrm, icol);
            }
        }
        if (!o.entrance) {
            const bool lit = litWindow(fr.at(o.wx0, o.sill));
            RenderMesh& pane = lit ? glassLit : glass;
            const std::size_t pv0 = pane.vertices.size();
            const Vec3 pcol = lit ? litPaneColour(glassGrey(), fr.at(o.wx0, o.sill), curtainWall)
                                  : glassGrey();
            q(pane, o.wx0, o.sill, o.wx1, narc > 0 ? ysp : o.head, pcol, in + fr.n * 0.02);
            if (narc > 0) {
                const Vec3 off = in + fr.n * 0.02;
                const Vec3 S = fr.at(arcC.x, ysp) + off;
                for (int k = 0; k + 1 < narc; ++k) {
                    const Vec3 A = fr.at(arc[k].x, arc[k].y) + off;
                    const Vec3 B = fr.at(arc[k + 1].x, arc[k + 1].y) + off;
                    MeshBuilder::emitTri(pane, S, A, B, nIn, pcol);
                    MeshBuilder::emitTri(pane, S, B, A, fr.n, pcol);
                }
            }
            roomUV(pane, pv0, fr);
            // REVEALS (Glenn's walk, 2026-09-14: "a gap between the exterior
            // and interior — no geo there"): the two jambs, the sill top and
            // the head underside between the facade sheet and this skin, so
            // the wall has a thickness at the window instead of an open slot
            // into the cavity.
            // They START where the exterior's own reveals stop (the pane
            // inset, `revealFrom`): the full facade emitter already closes
            // the recess from the wall face to its glass, and doubling that
            // depth z-fought at every jamb (Glenn's third walk, 2026-09-15).
            if (revealFrom < thick - 0.01) {
                const Vec3 from = fr.n * -revealFrom;
                const Real jambTop = narc > 0 ? ysp : o.head;
                const Vec3 P00 = fr.at(o.wx0, o.sill) + from, P10 = fr.at(o.wx1, o.sill) + from;
                const Vec3 P01 = fr.at(o.wx0, jambTop) + from, P11 = fr.at(o.wx1, jambTop) + from;
                const Vec3 to = fr.n * -(thick - revealFrom);
                emitQuad(wall, P00, P00 + to, P01 + to, P01, fr.h, icol);              // left jamb
                emitQuad(wall, P10, P10 + to, P11 + to, P11, fr.h * -1.0, icol);       // right jamb
                emitQuad(wall, P00, P10, P10 + to, P00 + to, Vec3(0, 1, 0), icol);     // sill
                if (narc > 0) {
                    for (int k = 0; k + 1 < narc; ++k) {   // the arch SOFFIT, segment by segment
                        const Vec3 A = fr.at(arc[k].x, arc[k].y) + from;
                        const Vec3 B = fr.at(arc[k + 1].x, arc[k + 1].y) + from;
                        const Real mx = (arc[k].x + arc[k + 1].x) * 0.5;
                        const Real my = (arc[k].y + arc[k + 1].y) * 0.5;
                        const Vec3 nrm = normalize(fr.h * (arcC.x - mx) + fr.v * (arcC.y - my));
                        emitQuad(wall, A, B, B + to, A + to, nrm, icol);
                    }
                } else {
                    emitQuad(wall, P01, P11, P11 + to, P01 + to, Vec3(0, -1, 0), icol);   // flat head
                }
            }
        }
    }
    appendToPart(out, PartId::Interior, wall);
    appendGlassParts(out, glass, glassLit, clearPanes);
}

// 2 cm proud (no reveal, no z-fight), the door as a dark quad. No surrounds,
// frames, muntins, sills, hoods, pilasters. ~2 triangles per opening instead
// of ~40; the wall is 2 instead of ~10 per bay.
static void emitFlatFacadeRect(BuildingMesh& out, const FaceRect& fr, FacadeMode mode,
                               const BuildingParams& p, const Vec3& wallColor) {
    RenderMesh wall, glass, glassLit, door;
    const FacadeMode layoutMode = mode;   // Rear lays its back door out, and is Residential otherwise
    if (mode == FacadeMode::Rear) mode = FacadeMode::Residential;
    if (mode == FacadeMode::StairDoor) mode = FacadeMode::Solid;
    emitQuad(wall, fr.at(0, 0), fr.at(fr.width, 0),
             fr.at(fr.width, fr.height), fr.at(0, fr.height), fr.n, wallColor);
    const Vec3 proud = fr.n * 0.02;
    const Vec3 gcol = glassGrey();
    const Vec3 dcol = materialFor(PartId::Door, wallColor).albedo;
    for (const BayOpening& o : facadeLayout(fr, layoutMode, p).open) {
        // Same anchor as the full emitter's pane (fr.at(wx0, sill)), so a
        // window keeps its lit/dark choice across the LOD swap.
        if (!o.entrance && o.head - o.sill < 1e-3) continue;   // a BLANK bay (facadeLayout): the wall quad has it
        const bool litPane = !o.entrance && litWindow(fr.at(o.wx0, o.sill));
        RenderMesh& dst = o.entrance ? door : (litPane ? glassLit : glass);
        const std::size_t pv0 = dst.vertices.size();
        const Vec3 fcol = o.entrance ? dcol
                                     : (litPane ? litPaneColour(gcol, fr.at(o.wx0, o.sill), p.curtainWall) : gcol);
        Vec2 farc[9];
        Vec2 farcC;
        const int fn = openingArc(o, farc, 8, &farcC);       // the middle tier arches too
        const Real fysp = o.head - o.rise;
        emitQuad(dst, fr.at(o.wx0, o.sill) + proud, fr.at(o.wx1, o.sill) + proud,
                 fr.at(o.wx1, fn > 0 ? fysp : o.head) + proud,
                 fr.at(o.wx0, fn > 0 ? fysp : o.head) + proud, fr.n, fcol);
        for (int k = 0; k + 1 < fn; ++k)
            MeshBuilder::emitTri(dst, fr.at(farcC.x, fysp) + proud,
                                 fr.at(farc[k].x, farc[k].y) + proud,
                                 fr.at(farc[k + 1].x, farc[k + 1].y) + proud, fr.n, fcol);
        if (!o.entrance) roomUV(dst, pv0, fr);
    }
    // VERTICALS on the middle tier (M3): the dark spandrel bands and the pier fronts -- the stripe that carries
    // the tower's look across the street.
    const FacadeLayout FL = facadeLayout(fr, mode, p);
    if (p.verticals && mode == FacadeMode::Residential && !FL.retailish) {
        const int g = windowGroupOf(p, mode, FL.retailish);
        const Vec3 sp = fr.n * 0.01, sc = wallColor * 0.62;
        for (std::size_t b = 0; b < FL.open.size(); ++b) {
            const BayOpening& o = FL.open[b];
            if (o.entrance || o.rise > 0) continue;
            const bool first = g <= 1 || b % g == 0, last = g <= 1 || static_cast<int>(b % g) == g - 1 || b + 1 == FL.open.size();
            const Real sx0 = first ? o.wx0 : o.x0, sx1 = last ? o.wx1 : o.x1;
            emitQuad(wall, fr.at(sx0, 0) + sp, fr.at(sx1, 0) + sp, fr.at(sx1, o.sill) + sp, fr.at(sx0, o.sill) + sp, fr.n, sc);
            emitQuad(wall, fr.at(sx0, o.head) + sp, fr.at(sx1, o.head) + sp, fr.at(sx1, fr.height) + sp,
                     fr.at(sx0, fr.height) + sp, fr.n, sc);
        }
        for (int b = 0; b <= FL.bays; b += g) {
            const Real pw = 0.6, x = std::min(std::max(std::min(b, FL.bays) * FL.bw, pw * 0.5), fr.width - pw * 0.5);
            const Vec3 o3 = fr.n * 0.24, al = fr.h * (pw * 0.5);
            emitQuad(wall, fr.at(x, 0) - al + o3, fr.at(x, 0) + al + o3, fr.at(x, fr.height) + al + o3,
                     fr.at(x, fr.height) - al + o3, fr.n, wallColor * 1.05);
            if (b == FL.bays) break;
        }
    }
    appendToPart(out, p.wallPart, wall);
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::GlassLit, glassLit);
    appendToPart(out, PartId::Door, door);
}

void emitFacadeRect(BuildingMesh& out, const FaceRect& fr, FacadeMode mode,
                    const BuildingParams& p, const Vec3& wallColor, bool clearPanes = false) {
    // Accumulate into locals, then append once each — never hold a part reference
    // across a partMesh() that could reallocate out.parts.
    // surround = sill/hood trim courses; frame = window frames + muntin lights.
    RenderMesh wall, glass, glassLit, door, surround, frame;

    // The splitter's decisions come from the SHARED layout (see facadeLayout):
    // this function only decides how much detail to draw them with.
    const FacadeLayout L = facadeLayout(fr, mode, p);
    if (mode == FacadeMode::Rear) mode = FacadeMode::Residential;   // the layout has its back door
    if (mode == FacadeMode::StairDoor) mode = FacadeMode::Solid;    // ...as does the stair wall's
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
        const Real rise = bay.rise;   // decided once, in facadeLayout
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
        if (p.verticals && !entrance && !retailish && rise <= 0 && mode == FacadeMode::Residential) {
            // VERTICALS (M3): the spandrel under and over the window set back and darker, running across the
            // group's inner mullions; the broad piers stay on the wall plane (and wear the proud pier below).
            const int g = windowGroupOf(p, mode, retailish);
            const int bi = static_cast<int>(std::lround(x0 / std::max(bw, Real(1e-6))));
            const bool first = g <= 1 || bi % g == 0, last = g <= 1 || bi % g == g - 1 || bi == bays - 1;
            const Real sx0 = first ? wx0 : x0, sx1 = last ? wx1 : x1;
            const Real sd = 0.10;
            const Vec3 in = fr.n * -sd, sc = wallColor * 0.62;
            wallQuad(x0, sx0, 0, openSill); wallQuad(sx1, x1, 0, openSill);
            wallQuad(x0, sx0, openHead, fh); wallQuad(sx1, x1, openHead, fh);
            for (const auto& [b0, b1] : {std::pair<Real, Real>{0, openSill}, std::pair<Real, Real>{openHead, fh}}) {
                if (b1 - b0 < 1e-3) continue;
                emitQuad(wall, fr.at(sx0, b0) + in, fr.at(sx1, b0) + in, fr.at(sx1, b1) + in, fr.at(sx0, b1) + in,
                         fr.n, sc);
                if (first) emitQuad(wall, fr.at(sx0, b0), fr.at(sx0, b0) + in, fr.at(sx0, b1) + in, fr.at(sx0, b1),
                                    fr.h, wallColor * 0.8);
                if (last) emitQuad(wall, fr.at(sx1, b0) + in, fr.at(sx1, b0), fr.at(sx1, b1), fr.at(sx1, b1) + in,
                                   fr.h * -1, wallColor * 0.8);
            }
        } else {
            wallQuad(x0, x1, 0, openSill);                   // below opening
            wallQuad(x0, x1, openHead, fh);                  // above apex
        }
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

        if (!entrance && openHead - openSill < 1e-3) continue;   // a BLANK bay: all wall (facadeLayout)
        if (entrance) {
            // The DOOR element: a recessed doorway. Closed like the windows —
            // jambs + lintel + threshold connect the wall opening back to the
            // door leaf, so you can't see through the gap into the hollow
            // shell. With openDoorway (enterable buildings, ADR-0080) the
            // leaf and frame are DROPPED and the reveal deepens to
            // wallThickness: the aperture is a real hole through a real wall
            // section, and the interior shell behind it closes the views.
            const Real revealDepth = p.openDoorway ? interiorInset(p) : 0.18;
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
            if (p.awning && !bay.backDoor) {
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
                                    bay.shopDoor ? "shopdoor" : bay.backDoor ? "backdoor" : "entrance", wx1 - wx0, openHead});
        } else {
            const Vec3 in = fr.n * (-p.windowInset);
            const Vec3 rev = wallColor * 0.82;
            const Vec3 gcol = glassGrey();
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
            const bool litPane = litWindow(fr.at(wx0, openSill));
            RenderMesh& pane = litPane ? glassLit : glass;
            const Vec3 pcol = litPane ? litPaneColour(gcol, fr.at(wx0, openSill), p.curtainWall) : gcol;
            const std::size_t pv0 = pane.vertices.size();
            emitQuad(pane, oBL + in, oBR + in, oTR + in, oTL + in, fr.n, pcol);
            if (rise > 0) {
                Vec3 S = fr.at(cx, ysp) + in;
                for (int k = 0; k < NARC; ++k)
                    MeshBuilder::emitTri(pane, S, fr.at(arc[k].x, arc[k].y) + in,
                                         fr.at(arc[k + 1].x, arc[k + 1].y) + in,
                                         fr.n, pcol);
            }
            roomUV(pane, pv0, fr);

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

    // VERTICAL PIERS (M3): a pier proud of the wall at every window group's edge, the storey's full height -- the
    // storeys stack them into one unbroken line up the tower.
    if (p.verticals && mode == FacadeMode::Residential && !retailish) {
        const int g = windowGroupOf(p, mode, retailish);
        const Real proud = 0.24;
        for (int b = 0; b <= bays; b += g) {
            const int bb = std::min(b, bays);
            const Real outer = bb < bays ? (L.open[static_cast<std::size_t>(bb)].wx0 - L.open[static_cast<std::size_t>(bb)].x0)
                                         : (L.open.back().x1 - L.open.back().wx1);
            const Real pw = std::clamp(outer * 1.5, Real(0.35), Real(0.9));
            const Real x = std::min(std::max(bb * bw, pw * 0.5), fr.width - pw * 0.5);
            const Vec3 c0 = fr.at(x, 0), c1 = fr.at(x, fh);
            const Vec3 along = fr.h * (pw * 0.5), outv = fr.n * proud;
            const Vec3 pc = wallColor * 1.05;
            emitQuad(wall, c0 - along + outv, c0 + along + outv, c1 + along + outv, c1 - along + outv, fr.n, pc);
            emitQuad(wall, c0 + along, c0 + along + outv, c1 + along + outv, c1 + along, fr.h, pc * 0.85);
            emitQuad(wall, c0 - along + outv, c0 - along, c1 - along, c1 - along + outv, fr.h * -1, pc * 0.85);
            if (b == bays) break;
        }
    }
    // The wall surface goes to the building's chosen facade part (procedural
    // brick/concrete/stucco/metal, or the flat Wall).
    appendToPart(out, p.wallPart, wall);
    appendGlassParts(out, glass, glassLit, clearPanes);
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
    Vec3 glass = glassGrey();
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
        boxAt(y + tierH * 0.18, tierH * 0.6, w * 0.82, PartId::Glass, Vec3(0.032, 0.027, 0.020));
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
                emitCurtainWallRect(out, faceOf(storey, side), wallColor, detail, false, curtainStyleOf(params));
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
void emitPlanRing(BuildingMesh& out, const Poly2& outer, const Poly2& hole,
                  Real yTop, Real thick, PartId part, const Vec3& col) {
    if (outer.size() < 3) return;
    RenderMesh m;
    for (const auto& t : triangulateWithHoles(outer, {hole})) {
        const Vec3 a(t[0].x, yTop, t[0].y), b(t[1].x, yTop, t[1].y),
            c(t[2].x, yTop, t[2].y);
        MeshBuilder::emitTri(m, a, b, c, Vec3(0, 1, 0), col);
        const Real yb = yTop - thick;
        MeshBuilder::emitTri(m, Vec3(t[0].x, yb, t[0].y),
                             Vec3(t[2].x, yb, t[2].y),
                             Vec3(t[1].x, yb, t[1].y), Vec3(0, -1, 0), col);
    }
    for (std::size_t i = 0; i < outer.size(); ++i) {
        const Vec2 a2 = outer[i], b2 = outer[(i + 1) % outer.size()];
        Vec2 en(-(b2 - a2).y, (b2 - a2).x);
        const Real el = en.length();
        if (el < 1e-9) continue;
        en = en * (-1.0 / el);   // outward for a CCW outer ring
        MeshBuilder::emitQuad(m, Vec3(a2.x, yTop, a2.y),
                              Vec3(b2.x, yTop, b2.y),
                              Vec3(b2.x, yTop - thick, b2.y),
                              Vec3(a2.x, yTop - thick, a2.y),
                              Vec3(en.x, 0, en.y), col);
    }
    appendToPart(out, part, m);
}

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
// THE TOP of a tower (buildings M2; Glenn, 2026-09-30: "tops ... the skyline is where variety shows most"): what
// stands on the roof against the sky, by BuildingParams::top --
//   2 SCREEN   the curtain wall run on past the roof, a storey and a half of glass hiding the plant;
//   3 SLOPED   a single-pitch glass roof across the short side (Citigroup Center);
//   4 FACETED  a truncated glass pyramid and a mast (Bank of America Plaza), glowing at night on half of them;
//   5 LANTERN  a lit glass box set back on the roof, capped -- the skyline's lamp after dark;
//   6 FRAME    an open frame of posts and beams, a storey and a half tall (the open crowns of the new towers);
//   7 ANTENNAS two masts on the long axis (Sears);
//   8 MAST     one mast on a plinth (One World Trade Center).
// The plan forms (screen, faceted, lantern, frame) take any CONVEX top plan, octagon and drum included; sloped needs a
// rect-ish top. A top that cannot stand on this plan returns 0 and the caller keeps the penthouse. Returns the rise
// above roofY. `bodyH` is the building's height to the roof (the masts scale with it).
static Real emitTowerTop(BuildingMesh& out, const Poly2& topIn, Real roofY, Real bodyH, const BuildingParams& p,
                         const Vec3& /*wallColor*/, bool full, bool rectish) {
    if (topIn.size() < 3 || p.top < 2) return 0;
    Poly2 top = topIn;
    ensureCCW(top);
    bool convex = true;
    for (std::size_t i = 0; i < top.size(); ++i) {
        const Vec2 a = top[i], b = top[(i + 1) % top.size()], c = top[(i + 2) % top.size()];
        if ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x) < -1e-6) { convex = false; break; }
    }
    const OBB2 obb = orientedBoundingBox(top);
    const Real shortW = 2 * std::min(obb.half[0], obb.half[1]);
    const Real fh = std::max(Real(3.0), p.floorHeight);
    const Vec2 c2 = centroid(top);
    const Vec3 up(0, 1, 0);
    const Vec3 glass = p.curtainWall ? curtainGlassColour(p.glassTint, glassGrey()) : glassGrey();
    const Vec3 metal = curtainMullionColour(p.mullionTone);
    const uint32_t h = positionHash(Vec3(c2.x, roofY, c2.y) + Vec3(0.17, 0.41, 0.83));
    RenderMesh glassM, litM, metalM, roofM;
    auto Y = [](const Vec2& v, Real y) { return Vec3(v.x, y, v.y); };
    auto quad = [](RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& cc, const Vec3& d, const Vec3& col) {
        const Vec3 n = normalize(cross(b - a, d - a));
        emitQuad(m, a, b, cc, d, n, col);
    };
    // A vertical box post or beam (metal), centred on a segment's foot.
    auto post = [&](const Vec2& at, Real y0, Real hgt, Real w) {
        emitBox(out, Scope{Vec3(at.x - w * 0.5, y0, at.y - w * 0.5), {Vec3(1, 0, 0), up, Vec3(0, 0, 1)}, Vec3(w, hgt, w)},
                PartId::Detail, metal * 1.1);
    };
    auto beacon = [&](const Vec3& at) {
        emitBox(out, Scope{at - Vec3(0.2, 0, 0.2), {Vec3(1, 0, 0), up, Vec3(0, 0, 1)}, Vec3(0.4, 0.5, 0.4)},
                PartId::Beacon, Vec3(1.0, 0.04, 0.02));
        out.attaches.push_back({at + Vec3(0, 0.25, 0), up, "beacon"});
    };
    // A mast: three stacked, narrowing boxes and a beacon at the tip.
    auto mast = [&](const Vec2& at, Real y0, Real hgt, Real w, const Vec3& col) {
        Real y = y0;
        const Real seg[3] = {0.45, 0.35, 0.20}, wid[3] = {1.0, 0.7, 0.4};
        for (int k = 0; k < 3; ++k) {
            const Real ww = w * wid[k], hh = hgt * seg[k];
            emitBox(out, Scope{Vec3(at.x - ww * 0.5, y, at.y - ww * 0.5), {Vec3(1, 0, 0), up, Vec3(0, 0, 1)}, Vec3(ww, hh, ww)},
                    PartId::Detail, col);
            y += hh;
        }
        beacon(Vec3(at.x, y, at.y));
        return y - y0;
    };
    // The glass screen around a plan, `H` tall: glass outside, a dark back inside, a mullion every bay, a cap.
    auto screen = [&](const Poly2& pl, Real y0, Real H) {
        for (std::size_t e = 0; e < pl.size(); ++e) {
            const Vec2 a = pl[e], b = pl[(e + 1) % pl.size()];
            const Real W = (b - a).length();
            if (W < 0.3) continue;
            quad(glassM, Y(a, y0), Y(b, y0), Y(b, y0 + H), Y(a, y0 + H), glass);
            quad(metalM, Y(b, y0), Y(a, y0), Y(a, y0 + H), Y(b, y0 + H), Vec3(0.10, 0.10, 0.11));
            const int bays = std::max(1, static_cast<int>(std::lround(W / 3.0)));
            if (full)
                for (int k = 0; k <= bays; ++k) {
                    const Vec2 at = a + (b - a) * (static_cast<Real>(k) / bays);
                    const Vec2 n = normalize(Vec2(b.y - a.y, a.x - b.x));   // outward (CCW)
                    const Vec2 o = at + n * 0.08;
                    quad(metalM, Y(o - normalize(b - a) * 0.06, y0), Y(o + normalize(b - a) * 0.06, y0),
                         Y(o + normalize(b - a) * 0.06, y0 + H), Y(o - normalize(b - a) * 0.06, y0 + H), metal);
                }
        }
        emitPlanParapet(out, pl, y0 + H - 0.05, 0.05, metal, PartId::Detail, metal);
    };
    Real rise = 0;
    switch (p.top) {
        case 2: {   // SCREEN
            const Real H = std::clamp(fh * 1.6, Real(5.0), Real(9.0));
            screen(top, roofY, H);
            rise = H;
            break;
        }
        case 3: {   // SLOPED
            if (!rectish) return 0;
            const int la = obb.longAxis(), sa = 1 - la;
            const Vec2 L = obb.axis[la] * obb.half[la], S = obb.axis[sa] * obb.half[sa];
            const Vec2 o = obb.center;
            const Real H = shortW * (0.55 + 0.35 * ((h & 0xffu) / 255.0));
            // Low edge on -S, high edge on +S.
            const Vec3 A = Y(o - L - S, roofY), B = Y(o + L - S, roofY), C = Y(o + L + S, roofY), D = Y(o - L + S, roofY);
            const Vec3 Cu = C + up * H, Du = D + up * H;
            quad(glassM, A, B, Cu, Du, glass);                 // the slope
            quad(glassM, C, D, Du, Cu, glass);                 // the tall back face
            MeshBuilder::emitTri(glassM, B, C, Cu, normalize(cross(C - B, Cu - B)), glass);
            MeshBuilder::emitTri(glassM, D, A, Du, normalize(cross(A - D, Du - D)), glass);
            if (full) {   // mullion lines down the slope
                const int n = std::max(2, static_cast<int>((obb.half[la] * 2) / 3.0));
                for (int k = 1; k < n; ++k) {
                    const Real t = static_cast<Real>(k) / n;
                    const Vec3 lo = A + (B - A) * t, hi = Du + (Cu - Du) * t;
                    const Vec3 side = normalize(B - A) * 0.07, lift = normalize(cross(B - A, Du - A)) * 0.05;
                    quad(metalM, lo - side + lift, lo + side + lift, hi + side + lift, hi - side + lift, metal);
                }
            }
            rise = H;
            break;
        }
        case 4: {   // FACETED
            if (!convex) return 0;
            const Real H = shortW * (0.55 + 0.25 * ((h & 0xffu) / 255.0));
            Poly2 lo = offsetPlan(top, 0.4), hi = top;
            for (Vec2& v : hi) v = c2 + (v - c2) * 0.14;
            const bool glow = ((h >> 8) & 1u) != 0;
            // A masonry tower's pyramid is METAL -- verdigris copper, bright copper or slate -- not glass.
            static const Vec3 kRoofMetal[3] = {{0.45, 0.72, 0.60}, {0.80, 0.45, 0.28}, {0.30, 0.32, 0.36}};
            RenderMesh& faces = glow ? litM : (p.curtainWall ? glassM : metalM);
            const Vec3 faceCol = glow ? Vec3(0.95, 0.90, 0.80) : p.curtainWall ? glass : kRoofMetal[(h >> 12) % 3];
            for (std::size_t e = 0; e < lo.size() && e < hi.size(); ++e) {
                const std::size_t f = (e + 1) % lo.size();
                quad(faces, Y(lo[e], roofY), Y(lo[f], roofY), Y(hi[f], roofY + H), Y(hi[e], roofY + H), faceCol);
                if (full) {   // a metal rib up each arris
                    const Vec3 a = Y(lo[e], roofY), b = Y(hi[e], roofY + H);
                    const Vec3 side = normalize(Y(lo[f], roofY) - a) * 0.09;
                    quad(metalM, a - side, a + side, b + side * 0.4, b - side * 0.4, metal);
                }
            }
            emitPlanSlab(out, hi, roofY + H + 0.3, 0.3, PartId::Roof, metal);
            rise = H + 0.3 + mast(c2, roofY + H + 0.3, std::clamp(H * 0.6, Real(6), Real(30)), 0.9, metal * 1.2);
            break;
        }
        case 5: {   // LANTERN
            if (!convex) return 0;
            const Poly2 lan = offsetPlan(top, std::max(Real(2.0), shortW * 0.18));
            if (lan.size() < 3 || std::fabs(area(lan)) < 20) return 0;
            const Real H = fh * 2.0;
            for (std::size_t e = 0; e < lan.size(); ++e) {
                const Vec2 a = lan[e], b = lan[(e + 1) % lan.size()];
                quad(litM, Y(a, roofY), Y(b, roofY), Y(b, roofY + H), Y(a, roofY + H), Vec3(0.95, 0.92, 0.85));
            }
            if (full)
                for (const Vec2& v : lan) post(v, roofY, H, 0.35);
            emitPlanSlab(out, offsetPlan(lan, -0.5), roofY + H + 0.6, 0.6, PartId::Roof, metal);
            rise = H + 0.6;
            break;
        }
        case 6: {   // FRAME
            if (!convex) return 0;
            const Real H = fh * 1.6, w = 0.55;
            const Poly2 ring = offsetPlan(top, 0.3);
            for (std::size_t e = 0; e < ring.size(); ++e) {
                const Vec2 a = ring[e], b = ring[(e + 1) % ring.size()];
                const Real W = (b - a).length();
                const int n = std::max(1, static_cast<int>(std::lround(W / 4.5)));
                for (int k = 0; k < n; ++k) post(a + (b - a) * (static_cast<Real>(k) / n), roofY, H, w);
                // The ring beam along the edge.
                const Vec2 d = normalize(b - a), nn(-d.y, d.x);
                const Vec2 a0 = a + nn * (w * 0.5), b0 = b + nn * (w * 0.5), a1 = a - nn * (w * 0.5), b1 = b - nn * (w * 0.5);
                quad(metalM, Y(a1, roofY + H), Y(b1, roofY + H), Y(b0, roofY + H), Y(a0, roofY + H), metal * 1.1);
                quad(metalM, Y(a1, roofY + H - w), Y(b1, roofY + H - w), Y(b1, roofY + H), Y(a1, roofY + H), metal * 1.1);
                quad(metalM, Y(b0, roofY + H - w), Y(a0, roofY + H - w), Y(a0, roofY + H), Y(b0, roofY + H), metal * 1.1);
            }
            rise = H;
            break;
        }
        case 7: {   // ANTENNAS
            const int la = obb.longAxis();
            const Vec2 L = obb.axis[la] * (obb.half[la] * 0.45);
            const Real Hm = std::clamp(bodyH * 0.18, Real(20), Real(90));
            const Vec3 white(0.82, 0.82, 0.80);
            const Real r1 = mast(obb.center - L, roofY, Hm, 1.6, white);
            const Real r2 = mast(obb.center + L, roofY, Hm * (0.88 + 0.12 * ((h & 0xffu) / 255.0)), 1.6, white);
            rise = std::max(r1, r2);
            break;
        }
        case 8: {   // MAST
            if (!convex) return 0;
            const Poly2 plinth = offsetPlan(top, shortW * 0.3);
            if (plinth.size() >= 3) screen(plinth, roofY, fh);
            const Real Hm = std::clamp(bodyH * 0.25, Real(25), Real(120));
            rise = fh + mast(c2, roofY + fh, Hm, 2.2, Vec3(0.78, 0.79, 0.80));
            break;
        }
        default: return 0;
    }
    appendToPart(out, PartId::Glass, glassM);
    appendToPart(out, PartId::LitBand, litM);
    appendToPart(out, PartId::Detail, metalM);
    appendToPart(out, PartId::Roof, roofM);
    return rise;
}


// THE EXPOSED EDGES of a lower tier (buildings M10): the stretches of its outline that are roof edge, not wall --
// what is left of each edge once the stretches an upper tier's edges run along (collinear, within 5 cm) are taken
// out. A setback tier sits wholly inside the one below and covers nothing: then `covered` is false and the caller
// keeps the closed parapet ring exactly as before. A FLUSH face (a bundled tube that drops out, a feathered step)
// covers part of the outline: only the rest gets a parapet, so no ledge wraps round the tower at the drop-off.
struct ExposedRun { Vec2 a, b; };
static std::vector<ExposedRun> exposedRuns(const Poly2& lowerIn, const Poly2& upperIn, bool& covered) {
    Poly2 lower = lowerIn, upper = upperIn;
    ensureCCW(lower); ensureCCW(upper);
    std::vector<ExposedRun> runs;
    covered = false;
    for (std::size_t i = 0; i < lower.size(); ++i) {
        const Vec2 a = lower[i], b = lower[(i + 1) % lower.size()];
        const Real L = (b - a).length();
        if (L < 1e-6) continue;
        const Vec2 d = (b - a) * (1.0 / L);
        std::vector<std::pair<Real, Real>> cov;
        for (std::size_t j = 0; j < upper.size(); ++j) {
            const Vec2 c = upper[j], e = upper[(j + 1) % upper.size()];
            auto off = [&](const Vec2& q) { return std::fabs(cross(d, q - a)); };
            if (off(c) > 0.05 || off(e) > 0.05) continue;
            Real t0 = dot(c - a, d), t1 = dot(e - a, d);
            if (t0 > t1) std::swap(t0, t1);
            t0 = std::max(t0, Real(0)); t1 = std::min(t1, L);
            if (t1 - t0 > 0.05) cov.push_back({t0, t1});
        }
        if (cov.empty()) { runs.push_back({a, b}); continue; }
        covered = true;
        std::sort(cov.begin(), cov.end());
        Real t = 0;
        for (const auto& [c0, c1] : cov) {
            if (c0 > t + 0.05) runs.push_back({a + d * t, a + d * c0});
            t = std::max(t, c1);
        }
        if (L > t + 0.05) runs.push_back({a + d * t, b});
    }
    return runs;
}

// One straight PARAPET RUN: an upstand `h` tall on the edge a->b of a CCW plan (the interior on its left), its
// coping on top. The open-run sibling of emitPlanParapet's closed ring.
static void emitParapetRun(BuildingMesh& out, const Vec2& a, const Vec2& b, Real roofY, Real h, const Vec3& wallCol,
                           PartId wallPart, const Vec3& copingCol) {
    const Real L = (b - a).length();
    if (L < 0.05 || h <= 0) return;
    const Vec2 d = (b - a) * (1.0 / L), in(-d.y, d.x);
    const Real th = 0.24;
    const Vec3 X(d.x, 0, d.y), Z(in.x, 0, in.y);
    emitBox(out, Scope{Vec3(a.x, roofY, a.y), {X, Vec3(0, 1, 0), Z}, Vec3(L, h, th)}, wallPart, wallCol);
    emitBox(out, Scope{Vec3(a.x, roofY + h, a.y) - Z * 0.05, {X, Vec3(0, 1, 0), Z}, Vec3(L, 0.09, th + 0.10)},
            PartId::Trim, copingCol);
}

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
    const Vec3 glassCol = glassGrey();
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

// Wood-grain frame (ADR-0080; device: "proper wood grain uv direction"):
// planks run along the plan's LONGEST edge — u follows the boards, v crosses
// them, in world metres over the WoodSiding tile so seams read at real size.
// Deterministic; shared by the lobby overlay and every storey slab so the
// grain is continuous up the building.
static void plankFrame(const Poly2& plan, Vec2& d, Vec2& perp) {
    d = Vec2(1, 0);
    Real bestLen = -1;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const Vec2 e = plan[(i + 1) % plan.size()] - plan[i];
        const Real len = e.length();
        if (len > bestLen) { bestLen = len; d = e * (1.0 / len); }
    }
    perp = Vec2(-d.y, d.x);
}

}  // namespace

RenderMaterial floorFinishFor(const BuildingParams& params) {
    // Five finishes, seed-picked per building (device ask for variety,
    // then for MORE materials): two woods, stone tile, marble, carpet.
    RenderMaterial m = materialFor(PartId::InteriorFloor, params.wallColor);
    // Bits 6-8 ONLY (& 7 before % 5): without the mask, %5 consumes every
    // upper bit and the stair/paint picks LEAK into the floor pick (the
    // stair-axis census caught the correlation). 8 values % 5 biases the
    // woods 2:1 over marble/carpet -- accents, not defaults, by design.
    switch (((params.seed >> 6) & 7u) % 5u) {
        case 0:   // dark walnut (the base recipe)
            break;
        case 1:   // light oak
            m.albedo = {0.66, 0.52, 0.36};
            m.roughness = 0.5f;
            m.emission = m.albedo * 0.06;
            break;
        case 2:   // stone tile (single source: the part material)
            m = materialFor(PartId::InteriorFloorTile, params.wallColor);
            break;
        case 3:   // marble
            m = materialFor(PartId::InteriorFloorMarble, params.wallColor);
            break;
        default:  // carpet
            m = materialFor(PartId::InteriorFloorCarpet, params.wallColor);
            break;
    }
    return m;
}

PartId floorFinishPartFor(const BuildingParams& params) {
    // Bits 6-8 ONLY (& 7 before % 5): without the mask, %5 consumes every
    // upper bit and the stair/paint picks LEAK into the floor pick (the
    // stair-axis census caught the correlation). 8 values % 5 biases the
    // woods 2:1 over marble/carpet -- accents, not defaults, by design.
    switch (((params.seed >> 6) & 7u) % 5u) {
        case 2:  return PartId::InteriorFloorTile;
        case 3:  return PartId::InteriorFloorMarble;
        case 4:  return PartId::InteriorFloorCarpet;
        default: return PartId::InteriorFloor;
    }
}

Vec3 interiorPaintFor(const BuildingParams& params) {
    // THE STYLE'S OWN FIELD COLOUR, when the facade implies one (room_plan.h
    // interiorFinishFor: concrete is brutalist, stucco with round heads is
    // spanish, and so on). Only a painted facade, which implies nothing,
    // falls through to the free palette below — so a building's rooms are
    // the colour its outside says they are (Glenn, 2026-09-15).
    if (const WallFinish f = interiorFinishFor(params); f.kind != WallFinishKind::Accent)
        return f.base;
    // Interior paints, round 7: the first cut had FOUR white-family
    // washes and gentle tints -- the device read "most buildings ...
    // still white". Now exactly ONE white and seven colours saturated
    // enough to read under the warm room light. Channels stay >= 0.45
    // (census-pinned): the ceiling keeps the drywall bounce, and the
    // walls' own emission rides the part material, so a mid-tone wall
    // cannot cave-darken a room.
    static const Vec3 kPaints[8] = {
        {0.87, 0.85, 0.80},   // warm white (the drywall; the ONE white)
        {0.62, 0.76, 0.58},   // sage
        {0.55, 0.68, 0.84},   // cornflower blue
        {0.88, 0.64, 0.58},   // blush terracotta
        {0.82, 0.70, 0.48},   // ochre tan
        {0.90, 0.83, 0.48},   // butter yellow
        {0.66, 0.72, 0.74},   // slate grey-blue
        {0.78, 0.62, 0.74},   // mauve
    };
    return kPaints[(params.seed >> 11) & 7u];
}

RenderMaterial stairFinishFor(const BuildingParams& params) {
    switch ((params.seed >> 9) & 3u) {   // bits 9-10 only, disjoint from the floor/paint picks
        case 0:   // matched set: the stair follows the floor
            return floorFinishFor(params);
        case 1: { // light oak stair
            RenderMaterial m =
                materialFor(PartId::InteriorFloor, params.wallColor);
            m.albedo = {0.66, 0.52, 0.36};
            m.roughness = 0.5f;
            m.emission = m.albedo * 0.06;
            return m;
        }
        case 2:   // dark walnut stair
            return materialFor(PartId::InteriorFloor, params.wallColor);
        default:  // stone stair
            return materialFor(PartId::InteriorFloorTile, params.wallColor);
    }
}

PartId stairFinishPartFor(const BuildingParams& params) {
    switch ((params.seed >> 9) & 3u) {   // bits 9-10 only, disjoint from the floor/paint picks
        case 0:  return floorFinishPartFor(params);
        case 3:  return PartId::InteriorFloorTile;
        default: return PartId::InteriorFloor;
    }
}

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

namespace {
// A tier inset that EXPLODES must not become the next tier (see the exterior
// loop's history: "one of the triangle skyscrapers went haywire when building
// the top"): valid only if it truly shrank, every vertex stayed inside the
// tier below, and no edge flipped direction.
bool tierInsetOk(const Poly2& outer, const Poly2& inner) {
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
}

// ---- the NYC-variety envelopes' plan vocabulary (skyscrapers NYC variety M1) ----------------------------
// The narrowest a tier may get: the core (a hoistway bank, two stairs and a corridor round them, ~12.3 x
// 10.5 m) is placed in the TOP tier and must fit every tier, or the building loses its door.
constexpr Real kCoreTierMin = 14.0;
Real tierShortSide(const Poly2& p) { const OBB2 b = orientedBoundingBox(p); return 2 * std::min(b.half[0], b.half[1]); }
// `inner` stands on `outer`: every vertex inside it (nudged 2% to its centroid, so a shared edge counts),
// smaller, and big enough to be a floor at all.
bool tierNestedIn(const Poly2& outer, const Poly2& inner) {
    if (inner.size() < 3) return false;
    const Real ai = area(inner);
    if (ai < 60.0 || ai >= area(outer) * 0.999) return false;
    const Vec2 c = centroid(inner);
    for (const Vec2& q : inner) if (!pointInPolygon(outer, q + (c - q) * 0.02)) return false;
    return true;
}
// A box on `ob`'s axes: half extents hw (axis 0) x hd (axis 1), its centre moved `shift` along the axes, its
// corners chamfered by `cut` (an octagon; < 0.3 m none).
Poly2 tierBox(const OBB2& ob, Real hw, Real hd, Real cut = 0, Vec2 shift = Vec2(0, 0)) {
    const Vec2 c = ob.center + ob.axis[0] * shift.x + ob.axis[1] * shift.y, a0 = ob.axis[0], a1 = ob.axis[1];
    Poly2 p;
    if (cut < 0.3) {
        p = {c - a0 * hw - a1 * hd, c + a0 * hw - a1 * hd, c + a0 * hw + a1 * hd, c - a0 * hw + a1 * hd};
    } else {
        cut = std::min(cut, std::min(hw, hd) * 0.9);
        p = {c - a0 * (hw - cut) - a1 * hd, c + a0 * (hw - cut) - a1 * hd, c + a0 * hw - a1 * (hd - cut), c + a0 * hw + a1 * (hd - cut),
             c + a0 * (hw - cut) + a1 * hd, c - a0 * (hw - cut) + a1 * hd, c - a0 * hw + a1 * (hd - cut), c - a0 * hw - a1 * (hd - cut)};
    }
    ensureCCW(p);
    return p;
}
// The biggest box on the plan's own axes that stands inside it (the plan's OBB, shrunk until it fits).
bool tierBoxInside(const Poly2& plan, const OBB2& ob, Real& hw, Real& hd) {
    hw = ob.half[0]; hd = ob.half[1];
    for (int k = 0; k < 14; ++k) {
        const Poly2 b = tierBox(ob, hw, hd);
        bool in = true;
        for (const Vec2& q : b) if (!pointInPolygon(plan, q + (ob.center - q) * 0.02)) { in = false; break; }
        if (in) return true;
        hw *= 0.93; hd *= 0.93;
    }
    return false;
}
}  // namespace

std::vector<MassTier> massStack(const Poly2& planIn, const BuildingParams& params) {
    std::vector<MassTier> out;
    Poly2 plan = planIn;
    if (plan.size() < 3) return out;
    ensureCCW(plan);
    out.push_back({plan, 0});
    Poly2 cur = plan;
    if (params.envelope == BuildingParams::Envelope::StreetWallSetback &&
        params.floors > params.baseFloors + 1 && params.baseFloors > 0) {
        // The street wall, then the first (big) setback on every side.
        int f = params.baseFloors;
        {
            Poly2 next = offsetPlan(cur, params.setback1);
            if (tierInsetOk(cur, next)) { out.push_back({next, f}); cur = next; }
        }
        // Later steps, every stepFloors, until the shaft takes over.
        if (params.stepFloors > 0 && params.stepDepth > 0)
            for (int g = f + params.stepFloors; g < params.floors; g += params.stepFloors) {
                if (params.towerFloor > 0 && params.towerFrac > 0 && g >= params.towerFloor) break;
                Poly2 next = offsetPlan(cur, params.stepDepth);
                if (!tierInsetOk(cur, next)) break;
                out.push_back({next, g});
                cur = next;
            }
        // The SHAFT: a rectangle on the current tier's box axes covering towerFrac
        // of the base plan, centred, shrunk until it sits inside the tier below.
        if (params.towerFrac > 0 && params.towerFloor > 0 && params.towerFloor < params.floors) {
            const Real want = area(plan) * params.towerFrac;
            if (want < area(cur) * 0.95) {
                const OBB2 ob = orientedBoundingBox(cur);
                const Real box = std::max(Real(1), 4 * ob.half[0] * ob.half[1]);
                Real hw = ob.half[0] * std::sqrt(want / box), hd = ob.half[1] * std::sqrt(want / box);
                const int floor0 = std::max(params.towerFloor, out.back().floor0 + 1);
                for (int attempt = 0; attempt < 5 && std::min(hw, hd) >= 4.0; ++attempt) {
                    Poly2 shaft{ob.center - ob.axis[0] * hw - ob.axis[1] * hd,
                                ob.center + ob.axis[0] * hw - ob.axis[1] * hd,
                                ob.center + ob.axis[0] * hw + ob.axis[1] * hd,
                                ob.center - ob.axis[0] * hw + ob.axis[1] * hd};
                    ensureCCW(shaft);
                    bool inside = true;
                    for (const Vec2& q : shaft)
                        if (!pointInPolygon(cur, q + (ob.center - q) * 0.02)) { inside = false; break; }
                    if (inside && floor0 < params.floors) { out.push_back({shaft, floor0}); cur = shaft; break; }
                    hw *= 0.9;
                    hd *= 0.9;
                }
            }
        }
        // Above the shaft, the uniform steps (setbackFloors/setbackEvery)
        // keep a tall tower tiered — the podium tower's shaft kept them —
        // but only while the tier stays wide enough for a core: the core
        // (one hoistway, two stairs, a corridor round it) needs ~12.3 x
        // 10.5 m and must fit EVERY tier, so a tower stepped down to a
        // 10 m cap would have no core and no door at all (the lab metro's
        // podium towers lost theirs the first time). 14 m keeps a margin.
        if (out.size() > 1 && params.setbackFloors > 0 && params.setbackEvery > 0) {
            constexpr Real kMinCoreTier = 14.0;
            for (int i = out.back().floor0 + params.setbackFloors; i < params.floors; i += params.setbackFloors) {
                Poly2 next = offsetPlan(cur, params.setbackEvery);
                if (!tierInsetOk(cur, next)) break;
                const OBB2 nb = orientedBoundingBox(next);
                if (2.0 * std::min(nb.half[0], nb.half[1]) < kMinCoreTier) break;
                cur = next;
                out.push_back({cur, i});
            }
        }
        return out;
    }
    using Env = BuildingParams::Envelope;
    // THE SKY EXPOSURE PLANE: the street wall, then a step every `stepFloors` along the plane (skyRatio up per
    // metre back), the first at least setback1, until the mass covers towerFrac of the lot; the tower above
    // rises straight.
    if (params.envelope == Env::SkyExposure && params.baseFloors > 0 && params.floors > params.baseFloors + 1) {
        const int sf = std::max(1, params.stepFloors);
        const Real run = sf * params.floorHeight / std::max(Real(0.5), params.skyRatio);
        const Real towerArea = area(plan) * std::clamp(params.towerFrac, Real(0.15), Real(0.6));
        bool first = true;
        for (int g = params.baseFloors; g < params.floors - 1; g += sf) {
            if (area(cur) <= towerArea) break;
            const Poly2 next = offsetPlan(cur, first ? std::max(run, params.setback1) : run);
            if (!tierInsetOk(cur, next) || tierShortSide(next) < kCoreTierMin) break;
            out.push_back({next, g});
            cur = next;
            first = false;
        }
        return out;
    }
    // THE TOWER ON ITS LOT'S RECTANGLE: Taper (narrowing, chamfering), Slab (a thin slab on the long axis) and
    // Feathered (a pencil stepping back one face at a time near the top), from `baseFloors` (the floors below
    // are the lot plan -- a base when the rectangle is smaller than the lot).
    if (params.envelope == Env::Taper || params.envelope == Env::Slab || params.envelope == Env::Feathered) {
        const OBB2 ob = orientedBoundingBox(plan);
        Real hw = 0, hd = 0;
        if (!tierBoxInside(plan, ob, hw, hd)) return out;
        const int f0 = std::max(1, params.baseFloors);
        if (f0 >= params.floors) return out;
        auto stand = [&](const Poly2& next, int floor0) {   // a tier on the one below, or (equal to it) nothing new
            if (area(next) >= area(cur) * 0.999) return true;
            if (!tierNestedIn(cur, next) || tierShortSide(next) < kCoreTierMin) return false;
            out.push_back({next, floor0});
            cur = next;
            return true;
        };
        if (params.envelope == Env::Taper) {
            if (!stand(tierBox(ob, hw, hd), f0)) return out;
            const int span = params.floors - f0;
            const int sf = std::max(1, span / 16);   // ~16 steps over the height: a taper, not a staircase
            for (int g = f0 + sf; g < params.floors; g += sf) {
                const Real t = Real(g - f0) / span;
                const Real sc = 1 - (1 - std::clamp(params.taperTop, Real(0.3), Real(1))) * t;
                const Real cut = std::clamp(params.chamferTop, Real(0), Real(0.45)) * t * 2 * std::min(hw, hd) * sc;
                if (!stand(tierBox(ob, hw * sc, hd * sc, cut), g)) break;
            }
            return out;
        }
        if (params.envelope == Env::Slab) {
            const bool longX = hw >= hd;
            const Real shortHalf = longX ? hd : hw, longHalf = longX ? hw : hd;
            const Real sw = std::max(kCoreTierMin * 0.5 + 0.25, shortHalf * std::clamp(params.towerFrac, Real(0.3), Real(1)));
            const Real sl = longHalf * 0.97;
            if (sw >= shortHalf) { stand(tierBox(ob, hw, hd), f0); return out; }
            stand(longX ? tierBox(ob, sl, sw) : tierBox(ob, sw, sl), f0);
            return out;
        }
        // Feathered: the rectangle, then from featherFrom every two floors the BACK face (axis 1, minus side)
        // steps in stepDepth, and from the third step the SIDE face (axis 0, plus side) alternates with it.
        if (!stand(tierBox(ob, hw, hd), f0)) return out;
        Real x0 = -hw, x1 = hw, z0 = -hd, z1 = hd;
        const Real d = std::clamp(params.stepDepth, Real(1.0), Real(4.0));
        const int ff = std::max(f0 + 1, static_cast<int>(params.floors * std::clamp(params.featherFrom, Real(0.3), Real(0.95))));
        int k = 0;
        for (int g = ff; g < params.floors - 1; g += 2, ++k) {
            if (k < 2 || k % 2 == 0) z0 += d; else x1 -= d;
            if (x1 - x0 < kCoreTierMin || z1 - z0 < kCoreTierMin) break;
            if (!stand(tierBox(ob, (x1 - x0) * 0.5, (z1 - z0) * 0.5, 0, Vec2((x0 + x1) * 0.5, (z0 + z1) * 0.5)), g)) break;
        }
        return out;
    }
    // Envelope::None: the uniform offsets, exactly as the storey stack always did.
    for (int i = 1; i < params.floors; ++i) {
        if (params.setbackFloors > 0 && params.setbackEvery > 0 && i % params.setbackFloors == 0) {
            Poly2 next = offsetPlan(cur, params.setbackEvery);
            if (tierInsetOk(cur, next)) { cur = next; out.push_back({cur, i}); }
        }
    }
    return out;
}

std::vector<StoreyPlan> storeyPlans(const Poly2& planIn,
                                    const BuildingParams& params) {
    std::vector<StoreyPlan> out;
    Poly2 plan = planIn;
    if (plan.size() < 3) return out;
    ensureCCW(plan);
    out.push_back({plan, 0, params.groundHeight, 0});
    const std::vector<MassTier> tiers = massStack(plan, params);
    Poly2 cur = plan;
    Real y = params.groundHeight;
    int tier = 0;
    std::size_t next = 1;
    for (int i = 0; i < params.floors; ++i) {
        while (next < tiers.size() && tiers[next].floor0 <= i) {
            cur = tiers[next].plan;
            ++tier;
            ++next;
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
    // A building with a CORE climbs by its stairwells: no straight stair, no
    // well (the core's shafts are the holes — coreSlabHoles).
    if (coreFor(plan, params, entranceEdge).valid) return il;
    // One full-storey flight, sized by the TALLEST storey it must serve
    // (the ground storey). Riser 0.28 / tread 0.25 (~48 degrees, inside the
    // 50-degree slope limit and 0.55 stepHeight): the 0.18/0.28 first cut
    // read as too long in play -- and round 3's pitch change was silently
    // LOST to a mid-script abort, so this is where it actually lands.
    const int risers =
        std::max(3, static_cast<int>(std::ceil(params.groundHeight / 0.28)));
    il.run = (risers - 1) * il.tread;
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
        // Clear of the inner wall skin (interiorInset 0.55) with margin.
        const Real inset = std::max(params.wallThickness + 0.15, Real(0.70));
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
        il.edge = e;
        // ...but the HOLE cut from ceilings/slabs is only the ascent's upper
        // reach plus a 5 cm lip past the top tread. Floor stays solid beside
        // and beyond it, so walking around the well to the next flight's
        // foot is walking on floor, not falling back down the stairwell.
        // EVERY flight uses this same run (growInterior gives upper storeys
        // the same riser COUNT with shallower risers), so one rect serves
        // every level. The hole must OPEN before the climb runs out of
        // headroom: while it is still closed overhead, the capsule's head
        // plus the NEXT riser must clear the arriving slab (2.2 m rig,
        // r 0.3; the upper storeys' shallow riser is the binding flight,
        // and Jolt's step-up cast is what actually hits first -- the walk
        // gate wedged at EXACTLY the hole edge). "0.25 run" was tuned for
        // the 0.18 pitch and died with it; the start is now DERIVED from
        // the constraint instead of tuned.
        const Real upRiser = human::FLOOR_HEIGHT / risers;
        const Real upSlope = upRiser / il.tread;
        Real holeStart =
            (human::FLOOR_HEIGHT - 2.2 - upRiser - 0.1) / upSlope - 0.3;
        holeStart = std::min(holeStart, il.run * 0.25);
        holeStart = std::max(holeStart, Real(0.3));
        const Vec2 h0 = s + u * holeStart - nIn * 0.05;
        const Vec2 h1 = s + u * (il.run + 0.05) - nIn * 0.05;
        const Vec2 wid = nIn * (wellWid + 0.10);
        il.well = Poly2{h0, h1, h1 + wid, h0 + wid};
        return il;
    }
    return il;
}

Poly2 LobbyPiece::footprint() const {
    Poly2 cs = {c - u * (w * 0.5) - v * (d * 0.5), c + u * (w * 0.5) - v * (d * 0.5),
                c + u * (w * 0.5) + v * (d * 0.5), c - u * (w * 0.5) + v * (d * 0.5)};
    // INSIDE-OUT DESKS (Glenn, 2026-09-17): the ring's winding follows (u, v)'s handedness; force CCW so
    // each side's (dy, -dx) normal points out.
    Real area = 0;
    for (std::size_t i = 0; i < 4; ++i) area += cs[i].x * cs[(i + 1) % 4].y - cs[(i + 1) % 4].x * cs[i].y;
    if (area < 0) std::swap(cs[1], cs[3]);
    return cs;
}

namespace {
// Do two convex polygons overlap (separating axis)?
bool convexOverlap(const Poly2& A, const Poly2& B) {
    for (const Poly2* P : {&A, &B})
        for (std::size_t i = 0; i < P->size(); ++i) {
            const Vec2 e = (*P)[(i + 1) % P->size()] - (*P)[i];
            const Vec2 ax(-e.y, e.x);
            Real a0 = 1e300, a1 = -1e300, b0 = 1e300, b1 = -1e300;
            for (const Vec2& q : A) { const Real t = dot(q, ax); a0 = std::min(a0, t); a1 = std::max(a1, t); }
            for (const Vec2& q : B) { const Real t = dot(q, ax); b0 = std::min(b0, t); b1 = std::max(b1, t); }
            if (a1 < b0 || b1 < a0) return false;
        }
    return true;
}
}  // namespace

std::vector<LobbyPiece> lobbyDressing(const Poly2& plan, std::size_t entranceEdge, const CorePlan& core) {
    std::vector<LobbyPiece> out;
    if (!core.valid || plan.size() < 3) return out;
    const std::size_t e = entranceEdge % plan.size();
    const Vec2 E = (plan[e] + plan[(e + 1) % plan.size()]) * 0.5;
    const Vec2 C = core.frame.toWorld({core.length * 0.5, 0.0});
    if ((E - C).length() <= 7.0) return out;
    // FACING THE DOOR (#60, Glenn: "a flattened reception desk ... clips into the stairs"): the desk's width
    // runs SQUARE to the way it faces. It ran along the core's own axis, and where the door lay along that
    // axis the width and the facing were parallel and the desk collapsed to a line.
    const Vec2 v = normalize(E - C);
    const Vec2 u(v.y, -v.x);
    // the core grown by a walkway: nothing of the lobby's dressing stands in front of a shaft door or a stair
    Poly2 coreRect = core.rect();
    {
        Vec2 m(0, 0);
        for (const Vec2& q : coreRect) m = m + q;
        m = m * (1.0 / static_cast<Real>(coreRect.size()));
        for (Vec2& q : coreRect) { const Vec2 d = q - m; const Real l = d.length(); if (l > 1e-9) q = q + d * (1.4 / l); }
    }
    const Vec3 wood(0.42, 0.30, 0.20), top(0.62, 0.60, 0.56), pot(0.30, 0.30, 0.32), leaf(0.20, 0.42, 0.22);
    auto group = [&](const Vec2& centre, bool planters) {
        std::vector<LobbyPiece> g;
        g.push_back({centre, u, v, 3.4, 0.9, 0.0, 1.05, wood, true});     // the desk
        g.push_back({centre, u, v, 3.6, 1.0, 1.05, 1.12, top, true});     // its counter top
        if (planters)
            for (Real sgn : {-1.0, 1.0}) {
                const Vec2 pc = centre + u * (sgn * 2.6);
                g.push_back({pc, u, v, 0.7, 0.7, 0.0, 0.62, pot, true});    // planter
                g.push_back({pc, u, v, 0.55, 0.55, 0.62, 1.35, leaf, false});  // its plant
            }
        return g;
    };
    auto fits = [&](const std::vector<LobbyPiece>& g) {
        for (const LobbyPiece& pc : g) {
            const Poly2 fp = pc.footprint();
            if (convexOverlap(fp, coreRect)) return false;
            for (const Vec2& q : fp)
                if (!pointInPolygon(plan, q)) return false;
        }
        return true;
    };
    for (const bool planters : {true, false})
        for (Real t = 0.45; t <= 0.80 + 1e-9; t += 0.05) {
            std::vector<LobbyPiece> g = group(C + (E - C) * t, planters);
            if (fits(g)) return g;
        }
    return out;
}

// THE SHOPS of the ground storey (Glenn, 2026-10-01: "I'm still waiting to see these small shops ... I'd also like
// to see that with smaller buildings"): a room behind every shop unit of every storefront edge (facadeLayout's own
// units, so the room sits behind its shopfront and door), from the facade back to a wall that stops short of the
// core or the stair, party walls between. The lobby is whatever is left.
// THE BIG BOX'S FLOOR (Glenn, 2026-10-01): one store. Inside the doors the CHECKOUT lanes (a 10 m band along the
// front), behind them the sales floor in the chain's own stock (furniture.cpp: pallet racks, gondolas, televisions,
// racks of clothes), and across the back the STOCKROOM behind a wall with two doors. Rooms carry the trade:
// Shop styles 7-10 the chain's floor (7 + chain - 1), 11 the checkouts, 12 the stockroom.
std::vector<ShopFront> shopFrontsOf(const Poly2& planIn, const BuildingParams& params) {
    std::vector<ShopFront> out;
    Poly2 plan = planIn;
    if (plan.size() < 3 || !params.groundRetail || !params.walkableGround) return out;
    ensureCCW(plan);
    const std::size_t entranceEdge = entranceEdgeFor(plan, params);
    for (std::size_t e = 0; e < plan.size(); ++e) {
        const FacadeMode mode = groundModeFor(plan, params, e, entranceEdge);
        if (mode != FacadeMode::Retail && mode != FacadeMode::Entrance) continue;
        const FaceRect fr = planEdgeRect(plan, e, 0.0, params.groundHeight);
        const FacadeLayout L = facadeLayout(fr, mode, params);
        const Vec2 a = plan[e], d = normalize(plan[(e + 1) % plan.size()] - a), n(d.y, -d.x);
        for (const ShopUnit& u : L.shops) {
            const BayOpening& o0 = L.open[static_cast<std::size_t>(u.b0)];
            const BayOpening& o1 = L.open[static_cast<std::size_t>(u.b1)];
            const BayOpening& od = L.open[static_cast<std::size_t>(u.door)];
            ShopFront f;
            f.a = a + d * o0.x0;
            f.b = a + d * o1.x1;
            f.n = n;
            f.door = a + d * ((od.wx0 + od.wx1) * 0.5);
            f.trade = u.type;
            // (the fascia the facade emits: just above the glazing's head, below the storey's top -- see the shops'
            // fascia in the facade's dressing)
            f.fasciaY0 = std::min(params.groundHeight - 1.0, kShopHead + 0.12);
            f.fasciaY1 = f.fasciaY0 < human::DOOR_HEIGHT + 0.05 ? f.fasciaY0 : f.fasciaY0 + 0.55;
            out.push_back(f);
        }
    }
    return out;
}

static RoomPlan bigBoxRoomPlan(const Poly2& planIn, const BuildingParams& params, std::size_t entranceEdge) {
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = false;
    rp.finish = interiorFinishFor(params);
    Poly2 plan = planIn;
    ensureCCW(plan);
    if (plan.size() != 4 || entranceEdge >= plan.size()) return rp;
    const Real inset = interiorInset(params);
    const Vec2 a = plan[entranceEdge], b = plan[(entranceEdge + 1) % 4];
    const Real W = (b - a).length();
    if (W < 20) return rp;
    const Vec2 d = (b - a) * (1.0 / W), nOut(d.y, -d.x);
    Real D = 0;
    for (const Vec2& v : plan) D = std::max(D, dot(a - v, nOut));
    if (D < 30) return rp;
    auto P = [&](Real x, Real v) { return a + d * x - nOut * v; };
    auto rect = [&](Real v0, Real v1) { return Poly2{P(inset, v0), P(W - inset, v0), P(W - inset, v1), P(inset, v1)}; };
    const Real vc = inset + 10.0, vs = D - inset - 12.0;
    Room checkout, floor, stock;
    checkout.kind = floor.kind = stock.kind = RoomKind::Shop;
    checkout.style = 11;
    floor.style = static_cast<uint8_t>(7 + (std::clamp<int>(params.bigBox, 1, 4) - 1));
    stock.style = 12;
    checkout.rect = rect(inset, vc);
    floor.rect = rect(vc, vs);
    stock.rect = rect(vs, D - inset);
    checkout.edge = floor.edge = stock.edge = entranceEdge;
    rp.rooms = {checkout, floor, stock};
    // the stockroom wall, in two runs, each with a door
    for (int k = 0; k < 2; ++k) {
        RoomWall w;
        w.a = P(k == 0 ? W - inset : W * 0.5, vs);
        w.b = P(k == 0 ? W * 0.5 : inset, vs);
        w.doorAt = 0.5;
        rp.walls.push_back(w);
    }
    return rp;
}

static RoomPlan shopRoomPlan(const Poly2& planIn, const BuildingParams& params, std::size_t entranceEdge, Real y0,
                             Real h, const CorePlan& core, const Poly2& well, const Vec2& stairFoot = Vec2(1e30, 1e30)) {
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = false;
    rp.finish = interiorFinishFor(params);
    if (!params.groundRetail || !params.walkableGround || planIn.size() < 3) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    const Real inset = interiorInset(params);
    const Poly2 coreR = core.valid ? core.rect() : well;
    std::vector<Poly2> taken;
    for (std::size_t e = 0; e < n; ++e) {
        const FacadeMode mode = groundModeFor(plan, params, e, entranceEdge);
        if (mode != FacadeMode::Retail && mode != FacadeMode::Entrance) continue;
        const FaceRect fr = planEdgeRect(plan, e, y0, h);
        const FacadeLayout L = facadeLayout(fr, mode, params);
        if (L.shops.empty()) continue;
        const Vec2 a = plan[e], dv = plan[(e + 1) % n] - a;
        const Real W = dv.length();
        if (W < 1e-6) continue;
        const Vec2 d = dv * (1.0 / W), nOut(d.y, -d.x);
        // How deep: to 1.5 m short of the core or stair (only where it stands in front), else 45% of the plate.
        Real across = 0;
        for (const Vec2& v : plan) across = std::max(across, dot(a - v, nOut));
        Real deep = std::min(Real(10.0), across * 0.45);
        if (coreR.size() >= 3) {
            Real t0 = 1e9, t1 = -1e9, dist = 1e9;
            for (const Vec2& c : coreR) {
                t0 = std::min(t0, dot(c - a, d)); t1 = std::max(t1, dot(c - a, d));
                dist = std::min(dist, dot(a - c, nOut));
            }
            (void)t0; (void)t1;
            deep = std::min(deep, dist - 1.5);
        }
        if (deep < inset + 3.0) continue;
        auto P = [&](Real x, Real v) { return a + d * x - nOut * v; };
        // The STAIR and its approach stay lobby: a shop that would cover them is cut short of them, or left out.
        Poly2 stairZone;
        if (well.size() >= 3) {
            const Vec2 wc = centroid(well);
            stairZone = well;
            for (Vec2& v : stairZone) v = v + normalize(v - wc) * 2.5;
        }
        for (const ShopUnit& u : L.shops) {
            const Real x0 = L.open[static_cast<std::size_t>(u.b0)].x0, x1 = L.open[static_cast<std::size_t>(u.b1)].x1;
            Real dpt = deep;
            if (stairZone.size() >= 3) {
                for (int tries = 0; tries < 8; ++tries) {
                    const Poly2 r0 = {P(x0, inset), P(x1, inset), P(x1, dpt), P(x0, dpt)};
                    bool hit = false;
                    for (const Vec2& v : stairZone) if (pointInPolygon(r0, v)) hit = true;
                    const Vec2 c0 = centroid(r0);
                    for (const Vec2& v : r0) if (pointInPolygon(stairZone, v + (c0 - v) * 0.02)) hit = true;
                    if (!hit) break;
                    dpt -= 1.0;
                }
                if (dpt < inset + 3.0) continue;
            }
            Poly2 r = {P(x0, inset), P(x1, inset), P(x1, dpt), P(x0, dpt)};
            // A corner: the shop on the other edge got there first.
            bool clash = false;
            for (const Poly2& t : taken) {
                const Vec2 c = centroid(r);
                for (const Vec2& v : r) if (pointInPolygon(t, v + (c - v) * 0.02)) clash = true;
                const Vec2 ct = centroid(t);
                for (const Vec2& v : t) if (pointInPolygon(r, v + (ct - v) * 0.02)) clash = true;
            }
            if (clash) continue;
            taken.push_back(r);
            Room rm;
            rm.edge = e;
            rm.kind = RoomKind::Shop;
            rm.style = u.type;
            rm.rect = r;
            rp.rooms.push_back(rm);
            // Its walls: the back, and a party wall at each side (not where the side is the plan's own end).
            RoomWall back; back.a = P(x1, dpt); back.b = P(x0, dpt);
            rp.walls.push_back(back);
            for (Real x : {x0, x1}) {
                if (x < inset + 0.05 || x > W - inset - 0.05) continue;
                RoomWall w; w.a = P(x, inset); w.b = P(x, dpt);
                rp.walls.push_back(w);
            }
        }
    }
    // The LOBBY must still reach the stair from the building's entrance: walk it (the shops' walls only, a probe
    // room at the stair's foot); a ground floor that would wall the stair off gets no shops.
    if (!rp.rooms.empty() && stairFoot.x < 1e29 && entranceEdge < n) {
        const Vec2 a = plan[entranceEdge], b = plan[(entranceEdge + 1) % n];
        const Vec2 d = normalize(b - a), nOut(d.y, -d.x);
        const Vec2 entry = (a + b) * 0.5 - nOut * (inset + 0.8);
        RoomPlan probe;
        probe.walls = rp.walls;
        Room foot;
        foot.rect = {stairFoot + Vec2(-0.3, -0.3), stairFoot + Vec2(0.3, -0.3), stairFoot + Vec2(0.3, 0.3),
                     stairFoot + Vec2(-0.3, 0.3)};
        probe.rooms.push_back(foot);
        if (!floorIsWalkable(probe, plan, entry, well)) return RoomPlan{};
    }
    return rp;
}

// THE FIRE ESCAPE (attached buildings; Glenn, 2026-10-01: "different ways up to the second floor"). A steel
// stair on the REAR face, at the end clear of the back door: an inner strip of grating along the wall at every
// floor, and in an outer strip a flight from each level to the next -- every flight rising the same way, so each
// stands a storey above the one below (no flight under another's treads). The first flight starts in the yard.
// `vis` gets the drawing (Metal), `col` the walkable treads, landings and rails (the streamed interior's
// collider); either may be null. Derived from the plan and params alone: exterior and interior agree.
static void emitFireEscape(BuildingMesh* vis, RenderMesh* col, const Poly2& planIn, const BuildingParams& params,
                           Real baseY, bool full) {
    Poly2 plan = planIn;
    if (plan.size() < 3) return;
    ensureCCW(plan);
    const std::size_t re = rearEdgeOf(plan, params);
    if (re >= plan.size()) return;
    const std::vector<StoreyPlan> st = storeyPlans(plan, params);
    if (st.size() < 2) return;
    const FaceRect fr = planEdgeRect(plan, re, baseY, params.groundHeight);
    const Real W = fr.width;
    Real doorX0 = W * 0.5, doorX1 = W * 0.5;
    if (params.backDoor)
        for (const BayOpening& o : facadeLayout(fr, FacadeMode::Rear, params).open)
            if (o.backDoor) { doorX0 = o.x0; doorX1 = o.x1; }
    // the side of the door with more room; built as if on the right, mirrored onto the left (x -> W - x)
    const bool flip = doorX0 > W - doorX1;
    const Real room = flip ? doorX0 : W - doorX1;
    const Real xe1 = W - 0.3;
    const Real span = std::min(Real(9.0), room - 0.5);
    if (span < 3.4) return;
    const Real xe0 = xe1 - span;
    auto mx = [&](Real x) { return flip ? W - x : x; };
    const Real zi0 = 0.05, zi1 = 0.85, zo1 = 1.55;   // inner strip (landing), outer strip (flights)
    const Real fx0 = xe0, fx1 = xe1 - 0.8;            // the flights' run; the arrival platform beyond it
    const Vec3 X = fr.h, U(0, 1, 0), N = fr.n;
    const Vec3 iron(0.11, 0.11, 0.12);
    auto boxQuads = [&](RenderMesh& m, const Scope& sc) {
        const Vec3 c000 = sc.corner(0, 0, 0), c100 = sc.corner(1, 0, 0), c110 = sc.corner(1, 1, 0),
                   c010 = sc.corner(0, 1, 0), c001 = sc.corner(0, 0, 1), c101 = sc.corner(1, 0, 1),
                   c111 = sc.corner(1, 1, 1), c011 = sc.corner(0, 1, 1);
        const Vec3 r = sc.axis[0], u = sc.axis[1], f = sc.axis[2];
        emitQuad(m, c000, c100, c110, c010, f * -1, iron);
        emitQuad(m, c001, c101, c111, c011, f, iron);
        emitQuad(m, c000, c001, c011, c010, r * -1, iron);
        emitQuad(m, c100, c101, c111, c110, r, iron);
        emitQuad(m, c000, c100, c101, c001, u * -1, iron);
        emitQuad(m, c010, c110, c111, c011, u, iron);
    };
    RenderMesh steel;
    // An axis-aligned box in face space (x along the wall, y up from baseY, z out from it).
    auto box = [&](Real x0, Real x1, Real y0, Real y1, Real z0, Real z1, bool collide) {
        const Real a = std::min(mx(x0), mx(x1)), b = std::max(mx(x0), mx(x1));
        const Scope sc{fr.at(a, 0) + U * y0 + N * z0, {X, U, N}, Vec3(b - a, y1 - y0, z1 - z0)};
        if (vis) boxQuads(steel, sc);
        if (col && collide) boxQuads(*col, sc);
    };
    // A box sloped up the flight (x0,y0) -> (x1,y1): `lift` above the pitch line, `t` thick, z0..z1.
    auto sloped = [&](Real x0, Real y0, Real x1, Real y1, Real lift, Real t, Real z0, Real z1, bool collide) {
        const Real dx = mx(x1) - mx(x0), dy = y1 - y0, len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6) return;
        const Real sx = dx < 0 ? -1.0 : 1.0;   // the up-normal of the pitch line stays up when mirrored
        const Vec3 a0 = (X * dx + U * dy) * (1.0 / len), a1 = (X * (-dy * sx) + U * (dx * sx)) * (1.0 / len);
        const Scope sc{fr.at(mx(x0), 0) + U * y0 + a1 * lift + N * z0, {a0, a1, N}, Vec3(len, t, z1 - z0)};
        if (vis) boxQuads(steel, sc);
        if (col && collide) boxQuads(*col, sc);
    };
    Real yPrev = 0;   // the yard
    for (std::size_t k = 1; k < st.size(); ++k) {
        if (st[k].plan.size() != plan.size()) break;   // a setback moved the rear wall: stop at the tier
        const Real Y = st[k].y0;
        const Real rise = Y - yPrev;
        // the flight: treads (solid risers in the collider), stringers, the outer handrail
        const int n = std::max(1, static_cast<int>(std::ceil(rise / 0.19)));
        const Real tread = (fx1 - fx0) / n;
        for (int i = 0; i < n; ++i) {
            const Real top = yPrev + rise * (i + 1) / n;
            const Real xa = fx0 + tread * i, xb = xa + tread;
            if (vis && full) box(xa, xb, top - 0.04, top, zi1 + 0.04, zo1 - 0.04, false);   // a tread
            if (col) box(xa, xb, top - rise / n, top, zi1, zo1, true);                       // its riser, solid
        }
        sloped(fx0, yPrev, fx1, Y, -0.22, 0.2, zi1, zi1 + 0.03, false);          // inner stringer
        sloped(fx0, yPrev, fx1, Y, -0.22, 0.2, zo1 - 0.03, zo1, false);          // outer stringer
        sloped(fx0, yPrev + 0.95, fx1, Y + 0.95, 0, 0.05, zo1 - 0.05, zo1, true);   // handrail
        // the landing: grating along the wall, the arrival platform, rails and corner posts
        box(xe0, xe1, Y - 0.06, Y, zi0, zi1, true);
        box(fx1, xe1, Y - 0.06, Y, zi1, zo1, true);
        box(fx1, xe1, Y + 0.95, Y + 1.0, zo1 - 0.05, zo1, true);                 // platform's outer rail
        box(xe1 - 0.05, xe1, Y, Y + 1.0, zi0, zo1, true);                        // end rail
        box(xe0, xe0 + 0.05, Y, Y + 1.0, zi0, zi1, true);                        // the other end
        if (vis && full) {
            for (Real px : {xe0, fx1, xe1 - 0.05})
                box(px, px + 0.05, Y - 0.3, Y + 1.0, zo1 - 0.05, zo1, false);
            for (int b = 1; b * 0.12 < xe1 - fx1; ++b)                            // balusters
                box(fx1 + b * 0.12, fx1 + b * 0.12 + 0.02, Y, Y + 0.95, zo1 - 0.04, zo1 - 0.02, false);
            box(xe0, xe1, Y - 0.22, Y - 0.06, zi0, zi0 + 0.05, false);           // the bracket band on the wall
        }
        yPrev = Y;
    }
    if (vis) appendToPart(*vis, PartId::Metal, steel);
}

BuildingMesh growInterior(const Poly2& planIn, const BuildingParams& params,
                          Real baseY, RenderMesh* colliderOut, int k0, int k1) {
    BuildingMesh out;
    // The fire escape's treads and landings are walkable while the interior is resident (its back door streams it).
    if (params.fireEscape && colliderOut) emitFireEscape(nullptr, colliderOut, planIn, params, baseY, true);
    Poly2 plan = planIn;
    if (plan.size() < 3) return out;
    ensureCCW(plan);
    const std::size_t entranceEdge = entranceEdgeFor(plan, params);
    const InteriorLayout il = interiorLayout(plan, params, entranceEdge);
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, params);
    // no storeys above ground -- but a BIG BOX is its one tall storey, and that is the store
    if (storeys.size() < 2 && !params.bigBox) return out;
    // The core (M5) and the storey window [kA, kB).
    const CorePlan core = coreFor(plan, params, entranceEdge);
    const int nS = static_cast<int>(storeys.size());
    const int kA = std::max(0, std::min(k0, nS));
    const int kB = k1 < 0 ? nS : std::max(kA, std::min(k1, nS));
    const Vec3 icol = materialFor(PartId::Interior, params.wallColor).albedo;
    const Real floorTone =
        0.85 + 0.45 * (((params.seed >> 4) & 0xffu) / 255.0);
    const Vec3 wcol = floorFinishFor(params).albedo * floorTone;
    RenderMesh mesh;   // DRYWALL: slab undersides (= ceilings)
    RenderMesh floorMesh;   // WOOD: slab tops, treads, risers, railings
    // One grain direction for the whole building (the ground plan's longest
    // edge), so the boards line up storey over storey.
    Vec2 grainD, grainP;
    plankFrame(plan, grainD, grainP);
    const Real grainTile =
        surfaceWorldTileSize(RenderMaterial::Surface::WoodSiding);
    const Vec2 grainOrigin = plan[0];
    auto grainUV = [&](Real x, Real z, float& u, float& v) {
        const Real lx = x - grainOrigin.x, lz = z - grainOrigin.y;
        u = static_cast<float>((lx * grainD.x + lz * grainD.y) / grainTile);
        v = static_cast<float>((lx * grainP.x + lz * grainP.y) / grainTile);
    };

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
    for (int ki = std::max(1, kA); ki < kB; ++ki) {
        const std::size_t k = static_cast<std::size_t>(ki);
        std::vector<Poly2> holes;
        if (core.valid) holes = coreSlabHoles(core);   // the shafts' own walls dress these rims
        else if (il.hasStair && static_cast<int>(k) <= stairTop)
            holes.push_back(il.well);
        const Real yTop = baseY + storeys[k].y0 + 0.05;
        // The hole's CUT EDGE: without a skirt the slab is two horizontal
        // faces with an open 0.25 m rim between them — from below, the top
        // face's backface shows through it (the facing view's residual red
        // line along the stairwell). Double-sided vertical quads close it.
        if (!core.valid) for (const Poly2& hole : holes)
            for (std::size_t e = 0; e < hole.size(); ++e) {
                const Vec2 a = hole[e], b = hole[(e + 1) % hole.size()];
                Vec2 en(-(b - a).y, (b - a).x);
                const Real el = en.length();
                if (el < 1e-9) continue;
                en = en * (1.0 / el);
                const Vec3 A(a.x, yTop, a.y), B(b.x, yTop, b.y);
                const Vec3 C(b.x, yTop - 0.25, b.y), D(a.x, yTop - 0.25, a.y);
                emitQuad(mesh, A, B, C, D, Vec3(en.x, 0, en.y), icol);
                emitQuad(mesh, A, B, C, D, Vec3(-en.x, 0, -en.y), icol);
            }
        for (const auto& t : triangulateWithHoles(storeys[k].plan, holes)) {
            float u0, v0, u1, v1, u2, v2;
            grainUV(t[0].x, t[0].y, u0, v0);
            grainUV(t[1].x, t[1].y, u1, v1);
            grainUV(t[2].x, t[2].y, u2, v2);
            MeshBuilder::emitTriUV(floorMesh, Vec3(t[0].x, yTop, t[0].y),
                                   Vec3(t[1].x, yTop, t[1].y),
                                   Vec3(t[2].x, yTop, t[2].y), Vec3(0, 1, 0),
                                   wcol, u0, v0, u1, v1, u2, v2);
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

    // The wood normal-map's groove direction follows TANGENTS, and emitTriUV
    // derives one from each triangle's first edge — arbitrary after ear
    // clipping (device: "the wood grain was oriented differently"). Pin every
    // slab tangent to the grain frame. (Stairs and railing are emitted after
    // this line and keep their own along-the-run tangents.)
    {
        const Vec3 grainT(grainD.x, 0, grainD.y);
        for (Vertex& v : floorMesh.vertices) v.tangent = grainT;
    }

    // --- top-floor ceiling: the top storey looked up at the roof slab's
    // dark underside (device report). A drywall ceiling closes it, with
    // the well hole only if the stair reaches this storey (it never does:
    // stairTop is the last storey's FLOOR -- so no hole).
    if (kB == nS) {
        const StoreyPlan& topSp = storeys.back();
        const Real cy2 = baseY + topSp.y0 + topSp.h - 0.25;
        for (const auto& t : triangulateWithHoles(topSp.plan, {})) {
            MeshBuilder::emitTri(mesh, Vec3(t[0].x, cy2, t[0].y),
                                 Vec3(t[2].x, cy2, t[2].y),
                                 Vec3(t[1].x, cy2, t[1].y), Vec3(0, -1, 0),
                                 icol);
        }
    }

    // --- inner-wall COLLIDERS (all storeys, ground included): planes at
    // the inset so a player can never stand inside the visible wall band
    // (device: "I can sometimes slip in between walls" -- the skins were
    // visual-only, so the capsule sank into them against the exterior
    // prism plane). The ground entrance edge keeps a gap at the door bay.
    if (colliderOut) {
        const Real inset = interiorInset(params);
        for (int ki = kA; ki < kB; ++ki) {
            const std::size_t k = static_cast<std::size_t>(ki);
            const StoreyPlan& spk = storeys[k];
            const Real wy0 = baseY + spk.y0;
            // The inset polygon: consecutive inset planes meet at its corners,
            // so the collider closes the corner slot the visible skins close.
            const Poly2 innerPlan = offsetPolygonEdges(spk.plan, std::vector<Real>(spk.plan.size(), -inset));
            for (std::size_t e = 0; e < spk.plan.size(); ++e) {
                const FaceRect fr = planEdgeRect(spk.plan, e, wy0, spk.h);
                // The ground storey keeps a gap at EVERY door on the edge: the entrance and the shops' doors.
                std::vector<std::pair<Real, Real>> gaps;
                if (k == 0 && params.walkableGround) {
                    const FacadeLayout L = facadeLayout(fr, groundModeFor(spk.plan, params, e, entranceEdge), params);
                    for (const BayOpening& o : L.open)
                        if (o.entrance) gaps.push_back({o.wx0, o.wx1});
                }
                const Vec3 off = fr.n * -inset;
                // Along the INSET edge (mitred ends), parametrised by the
                // facade's own x so the entrance gap maps unchanged.
                Vec3 ia = fr.at(0, 0) + off, ib = fr.at(fr.width, 0) + off;
                if (innerPlan.size() == spk.plan.size()) {
                    const Vec2 a2 = innerPlan[e], b2 = innerPlan[(e + 1) % innerPlan.size()];
                    ia = Vec3(a2.x, wy0, a2.y);
                    ib = Vec3(b2.x, wy0, b2.y);
                }
                const Vec3 ih = fr.width > 1e-6 ? (ib - ia) * (1.0 / fr.width) : Vec3(0, 0, 0);
                auto wallQuad = [&](Real a0, Real a1) {
                    if (a1 - a0 < 0.05) return;
                    // The ends run to the mitre; an interior span keeps its x.
                    const Vec3 A = a0 <= 0.0 ? ia : fr.at(a0, 0) + off;
                    const Vec3 B = a1 >= fr.width ? ib : fr.at(a1, 0) + off;
                    const Vec3 up(0, fr.height, 0);
                    emitQuad(*colliderOut, A, B, B + up, A + up, fr.n * -1.0, icol);
                    (void)ih;
                };
                Real at = 0;
                for (const auto& [g0, g1] : gaps) { wallQuad(at, g0); at = g1; }
                wallQuad(at, fr.width);
            }
        }
    }

    // --- inner walls per storey (same layout truth as the facade) --------
    // The exterior lays its upper storeys out with a copy of the params that
    // drops the pilasters (growPlanBuilding's `upper`): the layout must see
    // the SAME params or its bays shift against the facade's (Glenn's third
    // walk, 2026-09-15: "the interior was not using the same windows").
    const FacadeMode upMode =
        params.solidFacade ? FacadeMode::Solid : FacadeMode::Residential;
    BuildingParams upperP = params;
    upperP.pilasters = false;
    for (int ki = std::max(1, kA); ki < kB; ++ki) {
        const std::size_t k = static_cast<std::size_t>(ki);
        const StoreyPlan& spk = storeys[k];
        for (std::size_t e = 0; e < spk.plan.size(); ++e) {
            const FaceRect fr =
                planEdgeRect(spk.plan, e, baseY + spk.y0, spk.h);
            if (partyEdge(spk.plan, params, e)) {   // the party wall: painted plaster, no windows
                emitInsetSkin(out, spk.plan, e, baseY + spk.y0, spk.h,
                              interiorInset(params), interiorPaintFor(params), false);
                continue;
            }
            if (mechanicalStorey(params, ki)) {
                // The plant room: a closed painted wall behind the louvres.
                emitInsetSkin(out, spk.plan, e, baseY + spk.y0, spk.h,
                              interiorInset(params), interiorPaintFor(params) * 0.8, false);
                continue;
            }
            if (params.curtainWall) {
                // The inside of a curtain wall is GLASS above a spandrel band:
                // the pane part, which the interior system draws clear, so a
                // floor looks out over the city; the band is painted.
                // The band is the exterior's spandrel (emitCurtainWallRect:
                // min(0.9, 0.30 fh)), and the grid inside mirrors its bays.
                // DRAWN AGAINST THE OUTER GLASS (Glenn, 2026-09-30: "in the skyscrapers with the steel frames the
                // interior and exterior don't really match up, it feels like there's a gap between the two"): the
                // inner skin sat at the 0.55 m clipping inset, 0.45 m behind the curtain's glass (0.10 in), so a
                // cavity and a second, offset mullion grid showed between. It is drawn 2 cm behind the glass now;
                // the COLLIDER planes keep the 0.55 m inset, so the camera still cannot reach it.
                const Real skin = kCurtainGlassIn + 0.02;
                const CurtainStyle ics = curtainStyleOf(params);
                const Real band = ics.spandrelH(spk.h);
                emitInsetSkin(out, spk.plan, e, baseY + spk.y0, band,
                              skin, interiorPaintFor(params), false);
                emitInsetSkin(out, spk.plan, e, baseY + spk.y0 + band, spk.h - band,
                              skin, Vec3(1, 1, 1), false, PartId::Glass);
                emitInnerCurtainGrid(out, fr, skin, band, ics);
            } else {
                emitInnerWallRect(out, fr, facadeLayout(fr, upMode, upperP),
                                  interiorInset(params), params.wallColor,
                                  spk.plan, interiorPaintFor(params), params.curtainWall, false,
                                  params.windowInset);
            }
        }
    }

    // --- the stair (riser <= 0.18, run 0.28, inside the well) ------------
    const Vec2 u = il.stairDir;
    // The stair's own mesh and tint: stairFinishFor is an independent
    // axis (device ask), so treads/risers/soffit/stringers/railing land in
    // the STAIR finish's part and carry its albedo.
    RenderMesh stairMesh;
    const Vec3 scol = stairFinishFor(params).albedo * floorTone;
    auto flight = [&](const Vec2& foot, const Vec2& dir, Real y0f, Real rise,
                      int nR) {
        const Vec2 perp(-dir.y, dir.x);
        const Real riser = rise / nR;
        const Real half = il.width * 0.5;
        for (int j = 0; j < nR; ++j) {
            const Real yT = y0f + riser * (j + 1);
            const Vec2 t0 = foot + dir * (il.tread * j);
            const Vec2 t1 = foot + dir * (il.tread * (j + 1));
            const Vec3 A(t0.x - perp.x * half, yT, t0.y - perp.y * half);
            const Vec3 B(t0.x + perp.x * half, yT, t0.y + perp.y * half);
            const Vec3 C(t1.x + perp.x * half, yT, t1.y + perp.y * half);
            const Vec3 D(t1.x - perp.x * half, yT, t1.y - perp.y * half);
            const Vec3 A0(A.x, yT - riser, A.z), B0(B.x, yT - riser, B.z);
            const float uw = static_cast<float>(il.width / grainTile);
            const float vj0 = static_cast<float>((0.28 * j) / grainTile);
            const float vj1 = static_cast<float>((0.28 * (j + 1)) / grainTile);
            MeshBuilder::emitQuadUV(stairMesh, A, B, C, D, Vec3(0, 1, 0),
                                    scol, 0, vj0, uw, vj0, uw, vj1, 0, vj1);
            MeshBuilder::emitQuadUV(stairMesh, A0, B0, B, A,
                                    Vec3(-dir.x, 0, -dir.y), scol, 0, vj1, uw,
                                    vj1, uw, vj0, 0, vj0);
            if (colliderOut) {
                emitQuad(*colliderOut, A, B, C, D, Vec3(0, 1, 0), icol);
                emitQuad(*colliderOut, A0, B0, B, A,
                         Vec3(-dir.x, 0, -dir.y), icol);
            }
        }
        // Railing: a slim BOX, not a paper plane (device: "the stairwell
        // wall ... should have some thickness") — two faces 8 cm apart and
        // a top cap, 0.1..1.1 above the slope line, on the open (+perp)
        // side.
        const Vec2 oA = foot + perp * (half + 0.04);
        const Vec2 oB = foot + dir * (il.tread * nR) + perp * (half + 0.04);
        const Vec2 iA = foot + perp * (half - 0.04);
        const Vec2 iB = foot + dir * (il.tread * nR) + perp * (half - 0.04);
        const float ru =
            static_cast<float>((il.tread * nR) / grainTile);   // along the slope
        const float rv = static_cast<float>(1.0 / grainTile);
        // VERTICAL planks on the stairwell wall (device style call) — the
        // corner order makes the tangent vertical; u spans the height, v
        // runs along the slope.
        const float hu = static_cast<float>(1.0 / grainTile);
        auto railFace = [&](const Vec2& a2, const Vec2& b2, const Vec2& n2) {
            const Vec3 A(a2.x, y0f + 0.1, a2.y);
            const Vec3 B(b2.x, y0f + rise + 0.1, b2.y);
            const Vec3 C(b2.x, y0f + rise + 1.1, b2.y);
            const Vec3 D(a2.x, y0f + 1.1, a2.y);
            MeshBuilder::emitQuadUV(stairMesh, A, D, C, B,
                                    Vec3(n2.x, 0, n2.y), scol, 0, 0, hu, 0,
                                    hu, ru, 0, ru);
        };
        railFace(oA, oB, perp);
        railFace(iA, iB, perp * -1.0);
        {   // top cap ...
            const Real runL = 0.28 * nR;
            const Vec3 slopeUp = normalize(
                Vec3(-dir.x * rise, runL, -dir.y * rise));   // true cap normal
            const Vec3 A(iA.x, y0f + 1.1, iA.y), B(oA.x, y0f + 1.1, oA.y);
            const Vec3 C(oB.x, y0f + rise + 1.1, oB.y);
            const Vec3 D(iB.x, y0f + rise + 1.1, iB.y);
            MeshBuilder::emitQuadUV(stairMesh, A, B, C, D, slopeUp,
                                    scol, 0, 0, 0.08f, 0, 0.08f, ru, 0, ru);
            // ...and the BOTTOM cap: without it, from under the slope the
            // top cap's backface showed through the open underside — the
            // "normals are flipped on the stairwell wall" report (the
            // facing debug view drew a red line along the rail's top edge).
            const Vec3 A0(iA.x, y0f + 0.1, iA.y), B0(oA.x, y0f + 0.1, oA.y);
            const Vec3 C0(oB.x, y0f + rise + 0.1, oB.y);
            const Vec3 D0(iB.x, y0f + rise + 0.1, iB.y);
            MeshBuilder::emitQuadUV(stairMesh, A0, B0, C0, D0, slopeUp * -1.0,
                                    scol, 0, 0, 0.08f, 0, 0.08f, ru, 0, ru);
            // ...and the two END caps (the facing re-check still showed a
            // red streak at the head end: the top cap's backface through
            // the open end). Normals along ±dir.
            const Vec3 dn(dir.x, 0, dir.y);
            MeshBuilder::emitQuadUV(stairMesh, A0, B0, B, A, dn * -1.0, scol,
                                    0, 0, 0.08f, 0, 0.08f, 0.04f, 0, 0.04f);
            MeshBuilder::emitQuadUV(stairMesh, D0, C0, C, D, dn, scol, 0, 0,
                                    0.08f, 0, 0.08f, 0.04f, 0, 0.04f);
        }
        if (colliderOut) {
            const Vec3 A(oA.x, y0f + 0.1, oA.y);
            const Vec3 B(oB.x, y0f + rise + 0.1, oB.y);
            const Vec3 C(oB.x, y0f + rise + 1.1, oB.y);
            const Vec3 D(oA.x, y0f + 1.1, oA.y);
            emitQuad(*colliderOut, A, B, C, D, Vec3(perp.x, 0, perp.y), icol);
        }
    };
    // UNIFORM flights: every storey's flight uses the ground flight's riser
    // count and run — upper risers just get shallower — so all flights share
    // one XZ footprint, one arrival x, and the single well hole. Mixed runs
    // measured as a trap: the shorter flight's head hit the slab before the
    // shared hole began, and its top opened onto the hole's mid-air.
    const int nR =
        std::max(3, static_cast<int>(std::lround(il.run / il.tread)) + 1);
    for (int k = kA; il.hasStair && k < std::min(stairTop, kB); ++k) {
        const Real yk = baseY + storeys[static_cast<std::size_t>(k)].y0 + 0.05;
        const Real rise = storeys[static_cast<std::size_t>(k) + 1].y0 -
                          storeys[static_cast<std::size_t>(k)].y0;
        flight(il.stairFoot, u, yk, rise, nR);
        // SOFFIT: a sloped plane under the flight (foot base to the top
        // tread's underside), facing down — closes the sawtooth so tread
        // backfaces never show from beneath. Visual only; the treads stay
        // the collider truth. STRINGERS: sloped side bands from the soffit
        // line to just above the nosing line, closing the sawtooth's open
        // sides so the flight has real depth (device: "the floors also
        // don't have dimension").
        {
            const Vec2 perp(-u.y, u.x);
            const Real half = il.width * 0.5;
            const Real runLen = il.tread * nR;
            const Real riser = rise / nR;
            const Vec2 f0 = il.stairFoot;
            const Vec2 f1 = il.stairFoot + u * runLen;
            const float su =
                static_cast<float>(runLen / grainTile);
            const float sv = static_cast<float>(il.width / grainTile);
            const Vec3 A(f0.x - perp.x * half, yk, f0.y - perp.y * half);
            const Vec3 B(f0.x + perp.x * half, yk, f0.y + perp.y * half);
            const Vec3 C(f1.x + perp.x * half, yk + rise - riser,
                         f1.y + perp.y * half);
            const Vec3 D(f1.x - perp.x * half, yk + rise - riser,
                         f1.y - perp.y * half);
            const Vec3 soffitN = normalize(
                Vec3(u.x * (rise - riser), -runLen, u.y * (rise - riser)));
            MeshBuilder::emitQuadUV(stairMesh, A, B, C, D, soffitN,
                                    scol, 0, 0, sv, 0, sv, su, 0, su);
            const Real bandH = riser + 0.04;
            const float bu = static_cast<float>(bandH / grainTile);
            auto stringer = [&](const Vec2& side, const Vec2& n2) {
                const Vec3 SA(f0.x + side.x, yk, f0.y + side.y);
                const Vec3 SB(f1.x + side.x, yk + rise - riser,
                              f1.y + side.y);
                const Vec3 SC(f1.x + side.x, yk + rise - riser + bandH,
                              f1.y + side.y);
                const Vec3 SD(f0.x + side.x, yk + bandH, f0.y + side.y);
                MeshBuilder::emitQuadUV(stairMesh, SA, SD, SC, SB,
                                        Vec3(n2.x, 0, n2.y), scol, 0, 0, bu,
                                        0, bu, su, 0, su);
            };
            stringer(perp * half, perp);
            stringer(perp * -half, perp * -1.0);
        }
    }

    // --- ROOMS (M7): a ring of rooms along the outside walls of every
    // storey above the lobby — offices behind a curtain wall, apartments
    // behind masonry (room_plan.h) — with or without a core.
    // From kA, not from storey 1: a house's GROUND floor is living space,
    // and the ring pass returns nothing for a lobby anyway.
    for (int ki = kA; ki < kB; ++ki) {
        const StoreyPlan& spk = storeys[static_cast<std::size_t>(ki)];
        if (mechanicalStorey(params, ki)) continue;   // the plant room: no partitions
        // THE GROUND STOREY'S SHOPS (where the facade has them) take the ground floor; else the floor's plan.
        RoomPlan rp = ki == 0 && params.bigBox ? bigBoxRoomPlan(spk.plan, params, entranceEdge)
                    : ki == 0 && !params.campus ? shopRoomPlan(spk.plan, params, entranceEdge, baseY + spk.y0, spk.h, core,
                                             il.hasStair ? il.well : Poly2{},
                                             il.hasStair ? il.stairFoot : Vec2(1e30, 1e30))
                              : RoomPlan{};
        if (rp.rooms.empty())
            rp = roomPlan(spk.plan, params, core,
                          il.hasStair ? il.edge : static_cast<std::size_t>(-1),
                          interiorInset(params), ki,
                          il.hasStair ? il.well : Poly2{}, entranceEdge,
                          il.hasStair ? il.stairFoot : Vec2(1e30, 1e30));
        if (rp.walls.empty()) continue;
        RoomMeshes rm;
        emitRooms(rm, colliderOut, rp, baseY + spk.y0, spk.h, interiorPaintFor(params));
        appendToPart(out, PartId::Interior, rm.drywall);
        appendToPart(out, PartId::GlassClear, rm.glass);
        appendToPart(out, rp.finish.part, rm.accent);   // brick, concrete or timber
        // FURNITURE (buildings M4b): every named room gets the kit's pieces, placed for instanced drawing.
        // the ceiling's underside: the next storey's floor slab is 0.25 m deep
        emitFurniture(out.furniture, colliderOut, rp, baseY + spk.y0, params.seed, baseY + spk.y0 + spk.h - 0.25);
    }

    // --- the core (M5): shaft walls with doors, the dog-leg flights and
    // landings, on every storey in range — the ground's included.
    if (core.valid) {
        CoreMeshes cm;
        // The shaft walls themselves are the exterior mesh's (permanent, full
        // height — emitCoreShaftWalls in growPlanBuilding); here only their
        // colliders, for the storeys streamed.
        for (int ki = kA; ki < kB; ++ki)
            emitCoreStorey(cm, colliderOut, core, storeys[static_cast<std::size_t>(ki)], baseY,
                           params, ki + 1 < nS, ki >= 1, false);
        // LOBBY DRESSING (M5; #60): the desk, its top and two planters, placed by lobbyDressing.
        if (kA == 0) {
            const Real yF = baseY + 0.07;   // on the lobby overlay
            for (const LobbyPiece& pc : lobbyDressing(plan, entranceEdge, core)) {
                Poly2 cs = pc.footprint();   // CCW, so (dy, -dx) is each side's outward normal
                for (std::size_t i = 0; i < 4; ++i) {
                    const Vec2 a = cs[i], b = cs[(i + 1) % 4];
                    const Vec2 dd = b - a;
                    const Vec2 n = normalize(Vec2(dd.y, -dd.x));
                    const Vec3 A(a.x, yF + pc.h0, a.y), B(b.x, yF + pc.h0, b.y), Cc(b.x, yF + pc.h1, b.y), D(a.x, yF + pc.h1, a.y);
                    emitQuad(cm.drywall, A, B, Cc, D, Vec3(n.x, 0, n.y), pc.colour);
                    if (pc.collide && colliderOut) emitQuad(*colliderOut, A, B, Cc, D, Vec3(n.x, 0, n.y), pc.colour);
                }
                const Vec3 T0(cs[0].x, yF + pc.h1, cs[0].y), T1(cs[1].x, yF + pc.h1, cs[1].y),
                    T2(cs[2].x, yF + pc.h1, cs[2].y), T3(cs[3].x, yF + pc.h1, cs[3].y);
                emitQuad(cm.drywall, T0, T1, T2, T3, Vec3(0, 1, 0), pc.colour);
                if (pc.collide && colliderOut) emitQuad(*colliderOut, T0, T1, T2, T3, Vec3(0, 1, 0), pc.colour);
            }
        }
        appendToPart(out, PartId::Interior, cm.drywall);
        appendToPart(out, floorFinishPartFor(params), cm.floor);
        appendToPart(out, stairFinishPartFor(params), cm.stair);
    }

    appendToPart(out, PartId::Interior, mesh);
    appendToPart(out, floorFinishPartFor(params), floorMesh);
    appendToPart(out, stairFinishPartFor(params), stairMesh);
    out.height = storeys.back().y0 + storeys.back().h;
    return out;
}

// THE BIG BOX'S FRONT (Glenn, 2026-10-01: "Big box stores like Costco or Bestbuy"): a band in the chain's colour
// round the top of every wall; over the doors a deep canopy on two posts, the glazed entry either side of them,
// and above it the chain's lit sign with its name in letter blocks. The walls and the door are the ordinary
// facade's (one aperture, so the interior and its colliders agree); this only dresses them.
static void emitBigBoxDress(BuildingMesh& out, const Poly2& plan, std::size_t entranceEdge, Real y, Real gh,
                            const BuildingParams& params, bool full) {
    const Vec3 brand = params.trimColor;
    for (std::size_t i = 0; i < plan.size(); ++i) {   // the band
        const FaceRect fr = planEdgeRect(plan, i, y, gh);
        if (fr.width < 1.0) continue;
        emitBox(out, Scope{fr.at(0, gh - 1.7), {fr.h, Vec3(0, 1, 0), fr.n}, Vec3(fr.width, 1.0, 0.08)}, PartId::Trim, brand);
    }
    if (entranceEdge >= plan.size()) return;
    const FaceRect fr = planEdgeRect(plan, entranceEdge, y, gh);
    const Real cx = fr.width * 0.5;
    const Vec3 X = fr.h, U(0, 1, 0), N = fr.n;
    // the canopy and its posts
    const Real cw = std::min(fr.width - 4.0, Real(18.0)), cd = 4.0, cy = 4.4;
    if (cw > 6) {
        emitBox(out, Scope{fr.at(cx - cw * 0.5, cy), {X, U, N}, Vec3(cw, 0.55, cd)}, PartId::Trim, brand * 0.85);
        emitBox(out, Scope{fr.at(cx - cw * 0.5, cy - 0.06), {X, U, N}, Vec3(cw, 0.06, cd)}, PartId::Trim, Vec3(0.85, 0.85, 0.83));
        for (Real px : {cx - cw * 0.5 + 0.4, cx + cw * 0.5 - 0.8})
            emitBox(out, Scope{fr.at(px, 0) + N * (cd - 0.8), {X, U, N}, Vec3(0.4, cy, 0.4)}, PartId::Metal, Vec3(0.30, 0.31, 0.33));
    }
    // the glazed entry either side of the doors: dark glass panels, mullions
    if (full) {
        RenderMesh glass, mull;
        for (int side = 0; side < 2; ++side) {
            const Real g0 = side == 0 ? cx - 7.0 : cx + 1.2, g1 = side == 0 ? cx - 1.2 : cx + 7.0;
            if (g0 < 0.5 || g1 > fr.width - 0.5) continue;
            const Vec3 o = N * 0.03;
            emitQuad(glass, fr.at(g0, 0.1) + o, fr.at(g1, 0.1) + o, fr.at(g1, 3.4) + o, fr.at(g0, 3.4) + o, N, glassGrey());
            for (Real mx = g0; mx <= g1 + 1e-6; mx += (g1 - g0) / 4)
                emitBox(out, Scope{fr.at(mx - 0.05, 0.1), {X, U, N}, Vec3(0.1, 3.3, 0.08)}, PartId::Metal, Vec3(0.55, 0.57, 0.6));
            emitBox(out, Scope{fr.at(g0, 3.4), {X, U, N}, Vec3(g1 - g0, 0.12, 0.08)}, PartId::Metal, Vec3(0.55, 0.57, 0.6));
        }
        appendToPart(out, PartId::Glass, glass);
    }
    // the sign: a dark board, the lit face in the chain's colour, the name in pale letter blocks
    const Real sw = std::min(fr.width * 0.4, Real(24.0));
    // a 2.4 m board just above the canopy -- standing proud of the roofline on a low box, as real ones do
    const Real sy0 = std::max(cy + 0.9, gh - 4.6), sy1 = std::max(sy0 + 2.4, gh - 1.9);
    if (sw > 6 && sy1 - sy0 > 1.2) {
        emitBox(out, Scope{fr.at(cx - sw * 0.5 - 0.3, sy0 - 0.3), {X, U, N}, Vec3(sw + 0.6, sy1 - sy0 + 0.6, 0.25)},
                PartId::Trim, Vec3(0.10, 0.10, 0.11));
        // the board in the chain's colour by day; the NAME is what lights (LitBand, a warm white at night)
        const Vec3 o = N * 0.26;
        emitBox(out, Scope{fr.at(cx - sw * 0.5, sy0) + N * 0.25, {X, U, N}, Vec3(sw, sy1 - sy0, 0.02)}, PartId::Trim, brand);
        const int nL = 5 + static_cast<int>(params.seed % 4u);
        const Real lh = (sy1 - sy0) * 0.62, lw = std::min(lh * 0.75, sw * 0.8 / nL);
        const Real lx0 = cx - (nL * lw * 1.15) * 0.5;
        for (int k = 0; k < nL; ++k)
            emitBox(out, Scope{fr.at(lx0 + k * lw * 1.15, sy0 + (sy1 - sy0 - lh) * 0.5) + o, {X, U, N},
                               Vec3(lw, lh, full ? 0.12 : 0.02)}, PartId::LitBand, Vec3(1.0, 0.96, 0.88));
    }
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
        // The band is a RING: emitPlanSlab filled the whole footprint, and
        // the hidden cap of the ground string course landed 5 cm ABOVE the
        // enterable interior's wood floor, burying it under untextured Trim
        // (the "second floor texture is black" play report — found by the
        // release test: the grey floor survived the interior's release).
        const Poly2 hole = offsetPlan(pl, 0.20);
        const bool holeOk = hole.size() == pl.size() && area(hole) > 1.0 &&
                            area(hole) < area(pl);
        Real yb = yTop;
        for (const Tier& t : tiers) {
            const Poly2 ring = offsetPlan(pl, -t.grow * scale);
            if (holeOk)
                emitPlanRing(out, ring, hole, yb + t.h * scale, t.h * scale,
                             PartId::Trim, params.trimColor);
            else
                emitPlanSlab(out, ring, yb + t.h * scale, t.h * scale,
                             PartId::Trim, params.trimColor);
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
        // A BIG BOX's back is its LOADING DOCKS: roller doors along the rear wall.
        if (params.bigBox && i != entranceEdge && i == rearEdgeOf(plan, params)) {
            BuildingParams sp = params;
            const FaceRect dfr = planEdgeRect(plan, i, y, gh);
            sp.groundBays = std::clamp(static_cast<int>(dfr.width / 14.0), 2, 6);
            emitBayFront(out, dfr, sp, wallColor);
            if (full && params.openDoorway)
                emitInsetSkin(out, plan, i, y, gh, interiorInset(params), interiorPaintFor(params), true);
            continue;
        }
        if (i == sideEdge) {
            BuildingParams sp = params;
            sp.groundBays = params.sideBays;
            emitBayFront(out, planEdgeRect(plan, i, y, gh), sp, wallColor);
            // Enterable homes: the bay front is single-sided and its edge
            // skipped the inner shell -- from inside, a MISSING wall
            // (see-through to the world). A plain drywall pane seals it.
            if (full && params.openDoorway) {
                const FaceRect gfr = planEdgeRect(plan, i, y, gh);
                RenderMesh gi;
                const Vec3 goff = gfr.n * -interiorInset(params);
                const Vec3 gcol = interiorPaintFor(params);
                emitQuad(gi, gfr.at(0, 0) + goff,
                         gfr.at(gfr.width, 0) + goff,
                         gfr.at(gfr.width, gfr.height) + goff,
                         gfr.at(0, gfr.height) + goff, gfr.n * -1.0, gcol);
                emitQuad(gi, gfr.at(0, 0) + goff,
                         gfr.at(gfr.width, 0) + goff,
                         gfr.at(gfr.width, gfr.height) + goff,
                         gfr.at(0, gfr.height) + goff, gfr.n, gcol);
                appendToPart(out, PartId::Interior, gi);
            }
            continue;
        }
        FacadeMode mode = groundModeFor(plan, params, i, entranceEdge);
        (void)groundMode;
        // The stairwell hugs one wall; ANY window there reads wrong from
        // both sides -- Solid mode's clerestory strip included (device:
        // "the wall along which the stairwell was still had windows"). The
        // stair edge gets a TRUE blank wall: plain quads both sides, no
        // opening layout at all.
        bool stairEdge = false;
        if (full && params.openDoorway && i != entranceEdge) {
            const InteriorLayout ilw =
                interiorLayout(plan, params, entranceEdge);
            stairEdge = ilw.hasStair && i == ilw.edge;
        }
        // ...except a campus hall's, whose quad door is in it: the facade lays it out blank but for that door
        if (mode == FacadeMode::StairDoor) stairEdge = false;
        // A PARTY WALL is the same blank wall, at every detail level (attached buildings).
        const bool party = i != entranceEdge && partyEdge(plan, params, i);
        if (stairEdge || party) {
            const FaceRect bfr = planEdgeRect(plan, i, y, gh);
            RenderMesh bw;
            emitQuad(bw, bfr.at(0, 0), bfr.at(bfr.width, 0),
                     bfr.at(bfr.width, bfr.height), bfr.at(0, bfr.height),
                     bfr.n, wallColor);
            appendToPart(out, params.wallPart, bw);
            if (full && params.openDoorway)
                emitInsetSkin(out, plan, i, y, gh, interiorInset(params), interiorPaintFor(params), true);
            continue;
        }
        // An enterable building's ground storey has CLEAR panes at Full
        // detail (the lobby shows from the street, the street from the lobby).
        const bool clearLobby = full && params.openDoorway;
        // A glass tower's STOREFRONT edges are shopfronts (the shops' glazing and doors), its other ground edges
        // its curtain wall.
        if (params.curtainWall && mode != FacadeMode::Entrance && mode != FacadeMode::Retail)
            emitCurtainWallRect(out, planEdgeRect(plan, i, y, gh), wallColor, detail, clearLobby, curtainStyleOf(params));
        else if (full)
            emitFacadeRect(out, planEdgeRect(plan, i, y, gh), mode, params, wallColor, clearLobby);
        else
            emitFlatFacadeRect(out, planEdgeRect(plan, i, y, gh), mode, params, wallColor);
        // THE SHOP SIGNS: over every shop on a storefront edge, a fascia board -- a dark backing, a lit face in
        // the trade's colour (it glows at night), and a line of letter blocks up close.
        if (mode == FacadeMode::Retail || mode == FacadeMode::Entrance) {
            const FaceRect sfr = planEdgeRect(plan, i, y, gh);
            const FacadeLayout SL = facadeLayout(sfr, mode, params);
            auto fasciaOf = [](uint8_t trade) { const TradeInfo* t = tradeById(trade); return t ? t->fascia : Vec3(0.9, 0.9, 0.9); };
            RenderMesh lit, letters;
            for (const ShopUnit& u : SL.shops) {
                const Real x0 = SL.open[static_cast<std::size_t>(u.b0)].x0 + 0.15;
                const Real x1 = SL.open[static_cast<std::size_t>(u.b1)].x1 - 0.15;
                // The fascia just above the storefront's glazing (kShopHead), below the uplight band at the
                // storey's head on a tall lobby storey.
                const Real yb = std::min(gh - 1.0, kShopHead + 0.12), yt = yb + 0.55, proud = 0.16;
                if (x1 - x0 < 1.0 || yb < human::DOOR_HEIGHT + 0.05) continue;
                const Vec3 X = normalize(sfr.h);
                emitBox(out, Scope{sfr.at(x0, yb), {X, Vec3(0, 1, 0), sfr.n}, Vec3(x1 - x0, yt - yb, proud)},
                        PartId::Trim, Vec3(0.10, 0.10, 0.11));
                const Vec3 o = sfr.n * (proud + 0.005);
                emitQuad(lit, sfr.at(x0 + 0.05, yb + 0.05) + o, sfr.at(x1 - 0.05, yb + 0.05) + o,
                         sfr.at(x1 - 0.05, yt - 0.05) + o, sfr.at(x0 + 0.05, yt - 0.05) + o, sfr.n,
                         fasciaOf(u.type));
                if (full) {   // the name: letter blocks centred on the board
                    const int nLetters = 4 + static_cast<int>((u.type * 3 + u.b0) % 5);
                    const Real lw = 0.22, gap = 0.06, total = nLetters * lw + (nLetters - 1) * gap;
                    const Real lx0 = (x0 + x1) * 0.5 - total * 0.5;
                    const Vec3 lo = sfr.n * (proud + 0.02);
                    for (int c = 0; c < nLetters && total < x1 - x0 - 0.3; ++c) {
                        const Real lx = lx0 + c * (lw + gap);
                        const Real lh = (c * 7 + u.type) % 3 == 0 ? 0.22 : 0.30;
                        emitQuad(letters, sfr.at(lx, yb + 0.10) + lo, sfr.at(lx + lw, yb + 0.10) + lo,
                                 sfr.at(lx + lw, yb + 0.10 + lh) + lo, sfr.at(lx, yb + 0.10 + lh) + lo, sfr.n,
                                 Vec3(0.12, 0.10, 0.10));
                    }
                }
            }
            appendToPart(out, PartId::LitBand, lit);
            appendToPart(out, PartId::Detail, letters);
        }
        // Enterable buildings (ADR-0080): back the one-sided exterior skin
        // with an inner face at -wallThickness so the room reads as a room,
        // not as a view through to the sky.
        if (full && params.openDoorway) {
            const FaceRect ifr = planEdgeRect(plan, i, y, gh);
            if (params.curtainWall && mode != FacadeMode::Entrance && mode != FacadeMode::Retail) {
                emitInsetSkin(out, plan, i, y, gh, interiorInset(params),
                              Vec3(1, 1, 1), false, PartId::GlassClear);
            } else {
                emitInnerWallRect(out, ifr, facadeLayout(ifr, mode, params),
                                  interiorInset(params), wallColor, plan,
                                  interiorPaintFor(params), params.curtainWall, true,
                                  params.windowInset);
            }
        }
    }
    if (params.bigBox) emitBigBoxDress(out, plan, entranceEdge, y, gh, params, full);
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
        const CorePlan gcore = coreFor(plan, params, entranceEdge);
        std::vector<Poly2> holes;
        if (gcore.valid) holes = coreSlabHoles(gcore);   // the shafts (M5)
        else if (il.hasStair) holes.push_back(il.well);
        const Real cy = y + gh - 0.25;
        const Vec3 icol = materialFor(PartId::Interior, wallColor).albedo;
        RenderMesh ceil;
        for (const auto& t : triangulateWithHoles(plan, holes))
            MeshBuilder::emitTri(ceil, Vec3(t[0].x, cy, t[0].y),
                                 Vec3(t[2].x, cy, t[2].y),
                                 Vec3(t[1].x, cy, t[1].y), Vec3(0, -1, 0),
                                 icol);
        // Skirt the well rim (see growInterior): the ceiling's hole edge is
        // otherwise open between its underside and the slab above. Core
        // shafts get theirs 2 cm INSIDE the hole line: their walls stand on
        // the line once the interior streams in, and the skirt must neither
        // z-fight them then nor leave the holes open when seen from the
        // street before it does.
        if (gcore.valid) {
            std::vector<Poly2> inset;
            for (const Poly2& hole : holes) inset.push_back(offsetPolygonEdges(hole, std::vector<Real>(hole.size(), -0.02)));
            holes.swap(inset);
        }
        for (const Poly2& hole : holes)
            for (std::size_t e = 0; e < hole.size(); ++e) {
                const Vec2 a = hole[e], b = hole[(e + 1) % hole.size()];
                Vec2 en(-(b - a).y, (b - a).x);
                const Real el = en.length();
                if (el < 1e-9) continue;
                en = en * (1.0 / el);
                const Vec3 A(a.x, cy + 0.25, a.y), B(b.x, cy + 0.25, b.y);
                const Vec3 C(b.x, cy, b.y), D(a.x, cy, a.y);
                emitQuad(ceil, A, B, C, D, Vec3(en.x, 0, en.y), icol);
                emitQuad(ceil, A, B, C, D, Vec3(-en.x, 0, -en.y), icol);
            }
        appendToPart(out, PartId::Interior, ceil);
        RenderMesh woodFloor;
        // Floor tone varies per building (device: "different flooring
        // variety... a lighter wood floor"): deterministic from the seed,
        // 0.85 (dark walnut) .. 1.30 (light oak). Same derivation in
        // growInterior, so lobby and storeys match. Tile/carpet/marble
        // need their own surface bakes -- recorded in TECH_DEBT.
        const Real floorTone =
            0.85 + 0.45 * (((params.seed >> 4) & 0xffu) / 255.0);
        const Vec3 wcol = floorFinishFor(params).albedo * floorTone;
        Vec2 pd, pp;
        plankFrame(plan, pd, pp);
        const Real ptile =
            surfaceWorldTileSize(RenderMaterial::Surface::WoodSiding);
        const Vec2 uvOrigin = plan[0];
        auto puv = [&](const Vec2& v, float& u, float& w) {
            u = static_cast<float>(dot(v - uvOrigin, pd) / ptile);
            w = static_cast<float>(dot(v - uvOrigin, pp) / ptile);
        };
        // The HOISTWAYS are pits in this sheet (Glenn's third walk, 2026-09-15:
        // "the shaft and cab are still messed up" — the lobby's wood ran
        // straight into the hoistway and over the cab's floor, so an open
        // cab read as an empty shaft with the lobby floor inside it). The
        // stairwells keep the wood: their flights start from the lobby floor.
        std::vector<Poly2> pits;
        {
            const CorePlan ocore = coreFor(plan, params, entranceEdge);
            if (ocore.valid)
                for (const CoreShaft& hw : ocore.hoistways) pits.push_back(hw.rect());
        }
        auto woodTri = [&](const Vec2& a, const Vec2& b, const Vec2& c) {
            float u0, v0, u1, v1, u2, v2;
            puv(a, u0, v0);
            puv(b, u1, v1);
            puv(c, u2, v2);
            MeshBuilder::emitTriUV(woodFloor, Vec3(a.x, y + 0.07, a.y), Vec3(b.x, y + 0.07, b.y),
                                   Vec3(c.x, y + 0.07, c.y), Vec3(0, 1, 0), wcol, u0, v0, u1, v1, u2, v2);
        };
        if (pits.empty()) {
            for (const auto& t : triangulatePolygon(plan)) woodTri(plan[t[0]], plan[t[1]], plan[t[2]]);
        } else {
            for (const auto& t : triangulateWithHoles(plan, pits)) woodTri(t[0], t[1], t[2]);
        }
        // Edge band: the overlay's cut edge shows at the doorway; give the
        // wood sheet a visible thickness instead of a paper line.
        for (std::size_t e2 = 0; e2 < plan.size(); ++e2) {
            const Vec2 a2 = plan[e2], b2 = plan[(e2 + 1) % plan.size()];
            Vec2 en(-(b2 - a2).y, (b2 - a2).x);
            const Real el = en.length();
            if (el < 1e-9) continue;
            en = en * (1.0 / el);
            emitQuad(woodFloor, Vec3(a2.x, y + 0.07, a2.y),
                     Vec3(b2.x, y + 0.07, b2.y),
                     Vec3(b2.x, y + 0.045, b2.y),
                     Vec3(a2.x, y + 0.045, a2.y),
                     Vec3(-en.x, 0, -en.y), wcol);
        }
        for (Vertex& v : woodFloor.vertices)
            v.tangent = Vec3(pd.x, 0, pd.y);
        appendToPart(out, floorFinishPartFor(params), woodFloor);
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
    // THE CORE'S ENCLOSURE at full height, permanent (skyscrapers v2 M5,
    // Glenn's walk 2026-09-14): the shaft walls of every storey ride in the
    // exterior mesh, so the stairwells and hoistways are closed rooms from
    // the lobby, from a stairwell looking down and from a riding cab no
    // matter which storeys the interior has streamed. The streamed interior
    // adds their colliders, the flights, landings and doors.
    if (full && params.openDoorway && wantsCore(params)) {
        const CorePlan ecore = coreFor(plan, params, entranceEdge);
        if (ecore.valid) {
            CoreMeshes cm;
            // EVERY storey, the top one included: storeyPlans returns
            // floors + 1 entries (the ground storey is entry 0), and stopping
            // at `floors` left the topmost floor with no shaft walls at all —
            // collision but nothing drawn, so the cab and the stair showed
            // through (Glenn, 2026-09-15: "elevator housing... inside out or
            // missing").
            for (int i = 0; i < static_cast<int>(storeys.size()); ++i)
                emitCoreShaftWalls(cm, nullptr, ecore, storeys[static_cast<std::size_t>(i)], baseY, params);
            appendToPart(out, PartId::Interior, cm.drywall);
        }
    }
    Poly2 cur = plan;
    Real tierY0 = y;
    for (int i = 0; i < params.floors; ++i) {
        const StoreyPlan& sp = storeys[static_cast<std::size_t>(i) + 1];
        if (i > 0 && sp.tier != storeys[static_cast<std::size_t>(i)].tier) {
            // A setback landed at this floor: cap the tier below.
            emitPlanSlab(out, cur, y - 0.05, 0.2, PartId::Roof,
                         materialFor(PartId::Roof, wallColor).albedo);
            bool flushFace = false;
            (void)exposedRuns(cur, sp.plan, flushFace);
            if (full && params.stringCourse && !params.curtainWall && !flushFace)
                sweptCornice(cur, y - 0.4, 1.0);   // a cornice rings a setback; a flush face runs on unbroken
            if (full) cornerPosts(cur, tierY0, y - tierY0);
            bool flush = false;
            const std::vector<ExposedRun> runs = exposedRuns(cur, sp.plan, flush);
            if (!flush)
                emitPlanParapet(out, offsetPlan(cur, 0.02), y, 0.55,
                                materialFor(PartId::Trim, wallColor).albedo,
                                PartId::Trim,
                                materialFor(PartId::Trim, wallColor).albedo * 0.9);
            else
                for (const ExposedRun& r : runs)
                    emitParapetRun(out, r.a, r.b, y, 0.55, materialFor(PartId::Trim, wallColor).albedo,
                                   PartId::Trim, materialFor(PartId::Trim, wallColor).albedo * 0.9);
            cur = sp.plan;
            tierY0 = y;
        }
        const Real fh = params.floorHeight;
        const bool plant = mechanicalStorey(params, i + 1);
        for (std::size_t e = 0; e < cur.size(); ++e) {
            if (plant) {
                const Vec3 blade = params.curtainWall ? curtainMullionColour(params.mullionTone) * 1.2 : wallColor * 0.8;
                emitLouvreBand(out, planEdgeRect(cur, e, y, fh), blade, full);
                continue;
            }
            if (full && params.parkingDecks && !params.curtainWall) {
                emitParkingDeckRect(out, planEdgeRect(cur, e, y, fh), upper,
                                    wallColor);
                continue;
            }
            if (partyEdge(cur, params, e)) {   // a party wall: blank to the roof
                const FaceRect bfr = planEdgeRect(cur, e, y, fh);
                RenderMesh bw;
                emitQuad(bw, bfr.at(0, 0), bfr.at(bfr.width, 0), bfr.at(bfr.width, bfr.height),
                         bfr.at(0, bfr.height), bfr.n, wallColor);
                appendToPart(out, params.wallPart, bw);
                continue;
            }
            if (params.curtainWall)
                emitCurtainWallRect(out, planEdgeRect(cur, e, y, fh), wallColor,
                                    detail, false, curtainStyleOf(params));
            else if (full) {
                bool stairEdgeU = false;
                if (params.openDoorway && cur.size() == plan.size()) {
                    const InteriorLayout ilw =
                        interiorLayout(plan, params, entranceEdge);
                    stairEdgeU = ilw.hasStair && e == ilw.edge;
                }
                if (stairEdgeU) {
                    const FaceRect bfr = planEdgeRect(cur, e, y, fh);
                    RenderMesh bw;
                    emitQuad(bw, bfr.at(0, 0), bfr.at(bfr.width, 0),
                             bfr.at(bfr.width, bfr.height),
                             bfr.at(0, bfr.height), bfr.n, wallColor);
                    appendToPart(out, params.wallPart, bw);
                } else {
                    emitFacadeRect(out, planEdgeRect(cur, e, y, fh),
                                   params.solidFacade
                                       ? FacadeMode::Solid
                                       : FacadeMode::Residential,
                                   upper, wallColor);
                }
            }
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
    if (params.fireEscape) emitFireEscape(&out, nullptr, plan, params, baseY, full);

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
        // NIGHT DRESSING of a tall roof (skyscrapers v2 M4, step 2): the
        // things that give a real skyline its colours after dark. All of it
        // is lit glass with a per-building TINT in the vertex colour
        // (FLAG_EMISSIVE_VERTEX_TINT), deterministic from the roof's position,
        // and emitted for both LOD tiers — at night the skyline IS these.
        //  - a CROWN BAND under the coping on towers of 15+ floors (curtain
        //    walls from 12): white, amber, blue, red, green or purple, or none;
        //  - AVIATION BEACONS: red lamps at the roof corners of any building
        //    over 61 m (flashing — PartId::Beacon), and a steady second ring at
        //    mid-height past 120 m;
        //  - a SIGNAGE BOX high on one face of a curtain-wall tower.
        if (cur.size() >= 3 && params.floors >= 12) {
            const Vec2 rc = centroid(cur);
            const uint32_t nh = positionHash(Vec3(rc.x, y, rc.y) + Vec3(0.71, 0.29, 0.13));
            const Real u = static_cast<Real>(nh & 0xffu) / 255.0;
            RenderMesh lit;
            const Vec3 up(0, 1, 0);
            const bool tall = params.floors >= 15 || (params.curtainWall && params.floors >= 12);
            // The crown: the lighting spec's colour when authored, else the
            // hash's pick (none 30 %, white 25 %, amber 15 %, blue 10 %, red
            // 8 %, green 6 %, purple 6 %).
            static const Vec3 crownColours[7] = {{1.0, 0.97, 0.90}, {1.0, 0.97, 0.90}, {1.0, 0.75, 0.35},
                                                 {0.45, 0.60, 1.0}, {1.0, 0.25, 0.20}, {0.30, 1.0, 0.50},
                                                 {0.75, 0.40, 1.0}};
            int crownPick = 0;   // 0 none, 1..6 white..purple
            if (params.crown != 0) crownPick = params.crown == 1 ? 0 : std::min(6, static_cast<int>(params.crown) - 1);
            else if (u >= 0.94) crownPick = 6;
            else if (u >= 0.88) crownPick = 5;
            else if (u >= 0.80) crownPick = 4;
            else if (u >= 0.70) crownPick = 3;
            else if (u >= 0.55) crownPick = 2;
            else if (u >= 0.30) crownPick = 1;
            if (tall && crownPick > 0) {
                const Vec3 crown = crownColours[crownPick];
                const Real bandH = 0.5, bandY = y + 0.05 + std::max(Real(0.6), params.parapet) - 0.7;
                for (std::size_t e = 0; e < cur.size(); ++e) {
                    const FaceRect fr = planEdgeRect(cur, e, bandY, bandH);
                    const Vec3 o = fr.n * 0.03;
                    emitQuad(lit, fr.at(0, 0) + o, fr.at(fr.width, 0) + o,
                             fr.at(fr.width, bandH) + o, fr.at(0, bandH) + o, fr.n, crown);
                }
            }
            if (y >= 61.0) {
                // Pure red: with the glow's gain the tint's green and blue
                // would read as pink-white once tonemapped (Glenn, 2026-09-14).
                const Vec3 red(1.0, 0.04, 0.02);
                // Every lamp FLASHES (the Beacon part, gated by BeaconBlink at
                // runtime) and wears two translucent spheres — a bulb
                // (BeaconGlow) and a fainter, larger haze (BeaconHaze) — so the
                // light reads as a glow at any distance.
                RenderMesh bulb, haze;
                auto sphere = [&](RenderMesh& m, const Vec3& c, Real r) {
                    // An octahedron subdivided twice and pushed onto the sphere:
                    // 128 faces, smooth enough at arm's length on the roof.
                    const Vec3 ax[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
                    auto tri = [&](const Vec3& a, const Vec3& b, const Vec3& d) {
                        const Vec3 A = c + a * r, B = c + b * r, D = c + d * r;
                        MeshBuilder::emitTri(m, A, B, D, normalize(cross(B - A, D - A)), red);
                    };
                    auto quarter = [&](const Vec3& a, const Vec3& b, const Vec3& d) {
                        const Vec3 ab = normalize(a + b), bd = normalize(b + d), da = normalize(d + a);
                        tri(a, ab, da); tri(ab, b, bd); tri(da, bd, d); tri(ab, bd, da);
                    };
                    auto face = [&](const Vec3& a, const Vec3& b, const Vec3& d) {
                        const Vec3 ab = normalize(a + b), bd = normalize(b + d), da = normalize(d + a);
                        quarter(a, ab, da); quarter(ab, b, bd); quarter(da, bd, d); quarter(ab, bd, da);
                    };
                    const Vec3 &px = ax[0], &nx = ax[1], &py = ax[2], &ny = ax[3], &pz = ax[4], &nz = ax[5];
                    face(py, pz, px); face(py, px, nz); face(py, nz, nx); face(py, nx, pz);
                    face(ny, px, pz); face(ny, nz, px); face(ny, nx, nz); face(ny, pz, nx);
                };
                auto beacon = [&](const Vec2& v, const Vec2& toward, Real by) {
                    const Vec2 pin = v + normalize(toward - v) * 0.35;
                    const Vec3 o(pin.x - 0.2, by, pin.y - 0.2);
                    emitBox(out, Scope{o, {Vec3(1, 0, 0), up, Vec3(0, 0, 1)}, Vec3(0.4, 0.5, 0.4)},
                            PartId::Beacon, red);
                    const Vec3 c(pin.x, by + 0.25, pin.y);
                    // The runtime's handle on the lamp (BeaconLightSystem: the
                    // far sprite and the near point light).
                    out.attaches.push_back({c, up, "beacon"});
                    // The near-tier spheres ride the Full mesh only: the far
                    // tier is the sprite's.
                    if (full) {
                        sphere(bulb, c, 0.5);
                        sphere(haze, c, 1.1);
                    }
                };
                // At most SIX lamps per ring: a round plan's two dozen corners
                // would wear a crown of beacons (the skyline frame showed it).
                auto ring = [&](const Poly2& poly, const Vec2& toward, Real by) {
                    const std::size_t n = poly.size();
                    const std::size_t step = n <= 6 ? 1 : (n + 5) / 6;
                    for (std::size_t i = 0; i < n; i += step) beacon(poly[i], toward, by);
                };
                ring(cur, rc, y + 0.05 + params.parapet);
                if (y >= 120.0) {
                    // The mid ring sits on the TIER at half height, not the
                    // base plan: above a setback the base's corners hang in
                    // the air (Glenn: "some of the lights float too far away").
                    const int midFloor = static_cast<int>((y * 0.5 - params.groundHeight) / std::max(Real(1), params.floorHeight)) + 1;
                    const std::vector<MassTier> midTiers = massStack(plan, params);   // owns the plans midPlan points into
                    const Poly2* midPlan = &plan;
                    for (const MassTier& t : midTiers)
                        if (t.floor0 <= midFloor && t.plan.size() >= 3) midPlan = &t.plan;
                    ring(*midPlan, centroid(*midPlan), y * 0.5);
                }
                appendToPart(out, PartId::BeaconGlow, bulb);
                appendToPart(out, PartId::BeaconHaze, haze);
            }
            const bool signageOn = params.signage == 2 ||
                                   (params.signage == 0 && params.curtainWall && params.floors >= 20 &&
                                    ((nh >> 8) & 0xffu) < 100);
            if (signageOn && cur.size() >= 3) {
                const std::size_t e = static_cast<std::size_t>((nh >> 16) % cur.size());
                const FaceRect fr = planEdgeRect(cur, e, y - 2.6, 1.6);
                const Real w = std::min(Real(8.0), fr.width * 0.5);
                if (w >= 3.0) {
                    const Real x0 = (fr.width - w) * 0.5;
                    const Vec3 o = fr.n * 0.06;
                    static const Vec3 signs[4] = {{1.0, 0.98, 0.92}, {0.3, 0.95, 1.0}, {1.0, 0.2, 0.15}, {1.0, 0.7, 0.25}};
                    emitQuad(lit, fr.at(x0, 0) + o, fr.at(x0 + w, 0) + o, fr.at(x0 + w, 1.6) + o,
                             fr.at(x0, 1.6) + o, fr.n, signs[(nh >> 24) & 3u]);
                }
            }
            // PODIUM UPLIGHTS (M4, owed): on two towers in five, a warm band
            // at the ground storey's head — the wash a lobby's canopy lights
            // throw up the base of a tower — on every ground edge.
            const bool uplightsOn = params.uplights == 2 || (params.uplights == 0 && tall && ((nh >> 12) & 0xffu) < 102);
            if (uplightsOn) {
                const Vec3 warm(1.0, 0.80, 0.55);
                const Real by = params.groundHeight - 0.34, bh = 0.30;
                for (std::size_t e = 0; e < plan.size(); ++e) {
                    const FaceRect fr = planEdgeRect(plan, e, by, bh);
                    const Vec3 o = fr.n * 0.05;
                    emitQuad(lit, fr.at(0, 0) + o, fr.at(fr.width, 0) + o,
                             fr.at(fr.width, bh) + o, fr.at(0, bh) + o, fr.n, warm);
                }
            }
            appendToPart(out, PartId::LitBand, lit);
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
        } else {
            // THE TOP (buildings M2) -- both tiers: it is the skyline. The penthouse stays behind a screen and
            // wherever no top stands; a sloped, faceted, lantern, frame or mast top houses the plant itself.
            const Real topRise = emitTowerTop(out, cur, y + 0.05, y - baseY, params, wallColor, full, rectish);
            roofRise = std::max(roofRise, topRise);
            // Flat (LOD1) keeps spire/dome/steeple — they are the skyline —
            // but skips the penthouse + roof-furniture pack.
            if (full && (topRise <= 0 || params.top == 2 || params.top == 7))
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


CurtainStyle curtainStyleOf(const BuildingParams& p) {
    CurtainStyle cs;
    cs.glassTint = p.glassTint; cs.mullionTone = p.mullionTone; cs.fins = p.fins;
    cs.bay = p.curtainBay > 0.5 ? p.curtainBay : 1.6;
    cs.spandrelFrac = p.spandrelFrac;
    return cs;
}

}  // namespace engine
