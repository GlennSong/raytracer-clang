#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SHAPE_GRAMMAR_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SHAPE_GRAMMAR_H

#include "polygon.h"
#include "../../../renderer/renderer.h"   // RenderMesh, RenderMaterial
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace engine {

// The split/shape grammar (CityEngine CGA) of ADR-0038 §2 / city-generation-plan
// §4: a building is grown by rewriting a *scope* (an oriented box) with the ops
// split / repeat / comp / inset / extrude / roof / hollow / opening, emitting
// geometry tagged by a named part (wall / glass / roof / trim / ...). This is an
// L1 grammar sibling of the L-system (different interpreter, shared substrate);
// the ops here are the vocabulary the Lua surface and the C++ recipes both use.
//
// Coordinate convention: a scope is axis-aligned to its own frame. We only need
// upright buildings, so the frame is { right (XZ), up (+Y), forward (XZ) } with
// `size` the extent along each. The footprint sits on the ground plane and the
// building rises along +Y, all in real metres (ADR-0038 §5: human scale is the
// unit).

// Human-scale reference constants (metres). The grammar reads these so a door it
// punches is a door the player rig fits through (ADR-0038 §5). Loose elsewhere in
// the engine today; pinned here for the city.
namespace human {
constexpr Real FLOOR_HEIGHT      = 3.2;   // residential floor-to-floor
constexpr Real GROUND_HEIGHT     = 4.5;   // taller ground-floor (retail/lobby)
constexpr Real DOOR_HEIGHT       = 2.7;   // clear opening — reads as an ENTRANCE
                                          // against 4.5 m retail ground floors
                                          // (device: "front doors are very short")
constexpr Real DOOR_WIDTH        = 2.0;   // double-leaf entrance
constexpr Real INNER_DOOR_WIDTH  = 1.2;   // single-leaf interior opening: a
                                          // 0.6 m player capsule walks through
                                          // it instead of squeezing
constexpr Real GLASS_THICKNESS   = 0.06;  // a pane reads as glazing, not a plane
constexpr Real WINDOW_SILL       = 0.9;
constexpr Real WINDOW_HEAD       = 2.4;   // top of window above its floor
constexpr Real PARAPET           = 1.1;   // roof-edge railing height
}

// A named material class a part is emitted under, so the multi-part output keeps
// distinct PBR materials (ADR-0032). Maps to a RenderMaterial in materialFor().
enum class PartId : uint8_t {
    Wall, Glass, Trim, Roof, Door, Ground, Detail,
    // Wall surfaces shaded with a procedural material from the library
    // (materialFor packs the Surface id into the RenderMaterial flags).
    Brick, Concrete, Stucco, Metal,
    Wood,    // rooftop water tanks, timber details
    Siding,  // painted wood-siding facades (WoodSiding surface, colour in verts)
    Path,    // walking paths / front walks (Pavement surface)
    Foliage, // hedges, planter greenery (lot landscaping)
    Vent,    // HVAC intake grille (VentGrille surface, long faces)
    Utility, // HVAC service panels (UtilityPanel surface, short faces)
    Fan,     // HVAC fan cowl top (FanTop surface, centred disc UVs)
    Shingle, // pitched roof slopes (RoofShingle surface, slope-fitted UVs)
    GlassLit,// the ~1/3 of window panes that light up at night (WS3): same
             // material as Glass by day; the loader tags these chunks with
             // NightGlow and the day/night cycle raises their emission after
             // dusk. Chosen per opening by a position hash (litWindow), so the
             // SAME windows glow at every LOD and across rebuilds.
    Interior,// interior DRYWALL of enterable buildings (inner walls,
             // ceilings, slab undersides): smooth warm-white painted board,
             // faintly self-lit so unlit rooms never go black (ADR-0080).
    InteriorFloor, // interior WOOD flooring (slab tops, the lobby floor
             // overlay, stair treads/risers/railing): plank boards via the
             // WoodSiding surface, satin roughness (device ask).
    InteriorFloorTile, // interior STONE-TILE flooring (the tile finish of
             // floorFinishFor): pale polished Concrete bake. Its own part
             // because the merged city mesh binds ONE surface per part --
             // tile lobbies must not inherit the wood bump.
    InteriorFloorMarble, // veined polished Marble bake (own part, same rule)
    InteriorFloorCarpet, // soft Carpet bake (own part, same rule)
    Beacon,  // FLASHING red aviation obstruction lamps at a tall roof's corners
             // (skyscrapers v2 M4): its own part so the loader can tag the
             // chunk BeaconBlink and the day/night pass can gate its emission
             // on the flash cycle (the FAA L-864 pattern, ~30 flashes/min).
             // The mid-height ring flashes with it (the FAA has every L-864
             // level on a structure flash together).
    BeaconGlow,   // the lamp's BULB: a translucent red sphere around each beacon
             // (opacity 0.55, emissive, tinted) so the light reads as a glow,
             // not a painted box — tagged and gated like Beacon, brighter.
    BeaconHaze,   // the bulb's outer HAZE: a larger, fainter sphere (opacity
             // 0.22) — the soft red corona around the lamp at any distance.
    LitBand,      // LIT DRESSING that is not a window: crown bands, signage
             // boxes, podium uplight bands — emissive with the vertex tint
             // like GlassLit, but no room behind it (GlassLit is
             // interior-mapped now) and its own night glow.
    GlassClear,   // CLEAR glass (skyscrapers v2, the real tier): the ground
             // storey's panes of an enterable building, outer and inner, so
             // the lobby that ships with the exterior shows from the street
             // and the street shows from the lobby. Transparent, both faces,
             // no room behind it.
    Furniture,    // FURNITURE (buildings M4): painted / plastic pieces -- a neutral satin material whose colour
             // is the vertex's, with a faint self-light (furniture.h). Its siblings carry the other finishes of
             // the furniture kit (procgen/furniture_kit.h), one per FurnMat:
    FurnitureWood,     // wood grain (Surface::WoodGrain), the species in the vertex colour
    FurnitureFabric,   // woven upholstery (Surface::Fabric), the colour on the vertex
    FurnitureMetal,    // brushed steel / chrome / black metal: metallic, the tint on the vertex
    FurnitureCeramic,  // glazed porcelain and stone tops: glossy
    Count    // KEEP LAST: materialIndexFor is the ordinal; arrays size by Count
};

// Deterministic per-opening night-light choice, shared by every facade
// emitter (full, flat, curtain wall) so LOD transitions never flicker a
// window on or off: quantized world position in, stable coin-flip out.
bool litWindow(const Vec3& worldPos);
// The TINT of a lit pane (skyscrapers v2 M4): a per-window pick from the
// building's palette — cool office whites with a share of fluorescent
// blue-white behind a curtain wall, warm incandescent with some cream and the
// odd cool room elsewhere — hashed from the same anchor as litWindow, so the
// choice survives the LOD swap. Carried in the pane's vertex colour and
// applied to the EMISSION only (RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT).
Vec3 litTint(const Vec3& worldPos, bool curtainWall);
// THE GLASS COLOUR (Glenn, 2026-09-30: "more varied glass colour, it's all very samey"). The Glass and GlassLit
// materials are white; a pane's VERTEX COLOUR is its glass -- the reflectance the sky is mirrored in -- so every
// building carries its own. glassGrey() is the default pane (what every glass looked like before).
Vec3 glassGrey();
// A LIT pane's vertex colour carries BOTH its glass (by day) and its lit tint (by night): the glass colour in the
// top 7 bits of each 8-bit channel, the lit tint's palette index (1..7, litTintOf) in the low bits -- r bit 0, g
// bit 1, b bit 2. mesh.frag decodes it on the interior-mapped lit part; index 0 = an unpacked, legacy pane.
int litTintIndex(const Vec3& worldPos, bool curtainWall);
Vec3 litTintOf(int index);
Vec3 litPaneColour(const Vec3& glassCol, const Vec3& worldPos, bool curtainWall);
// Curtain-wall towers light per BAY, not per storey-face (device: "the way
// it's lit up row by row is odd"): each mullion bay flips its own coin
// against a per-STOREY occupancy — some floors busy, some nearly dark, the
// way an office tower reads at night. Both hash quantized world positions,
// so every LOD agrees and rebuilds light the same offices.
Real  litStoreyOccupancy(const Vec3& storeyAnchor);   // 0.12 .. 0.62
bool  litOfficeBay(const Vec3& bayAnchor, Real occupancy);

// Facade material style (the "different facades like brick or concrete" axis).
// At the merged-model scale, style varies the wall *colour* per building (carried
// in vertex colour, so it works on one shared material); GlassCurtain also flips
// the building to an all-glass facade. A future layer can swap in tiled
// procedural brick/concrete *textures* with world-scaled UVs.
enum class FacadeStyle : uint8_t {
    Concrete, Brick, Stucco, Painted, GlassCurtain, Metal,
    Wood,       // painted wood siding (suburban timber homes)
    DarkBrick,  // deep browns / charcoal reds (lofts, factories, dark towers)
    Sandstone,  // warm buff ashlar (banks, museums, art-deco masonry)
    Count
};

// A seeded wall colour for a style (brick reds, concrete greys, stucco creams,
// painted pastels). Deterministic for the seed.
Vec3 facadeColor(FacadeStyle style, uint32_t seed);

// Building massing — not every building is a box (curved towers, tiered pagodas).
// The grammar dispatches on this in growBuilding.
enum class BuildingShape : uint8_t {
    Box,        // the orthogonal split-grammar building
    Cylinder,   // a round tower: cylindrical mass with wrapped glass banding
    Pagoda,     // East-Asian tiered mass with flared, upturned-corner tile roofs
};

// One scope: an oriented box. axis[] are unit and mutually perpendicular; size[i]
// is the full extent along axis[i]. origin is the corner where all three local
// coordinates are zero (the min corner of the box in its own frame).
struct Scope {
    Vec3 origin{0, 0, 0};
    Vec3 axis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};   // right, up, forward
    Vec3 size{1, 1, 1};

    Vec3 corner(Real u, Real v, Real w) const {         // u,v,w in [0,1]
        return origin + axis[0] * (size.x * u) + axis[1] * (size.y * v) +
               axis[2] * (size.z * w);
    }
    Vec3 center() const { return corner(0.5, 0.5, 0.5); }
};

// Build a ground-plane scope from a footprint polygon: its oriented bounding box
// becomes the box footprint, raised to `height` along +Y, sitting at ground
// elevation `baseY`. The forward axis is the OBB's long edge so facades face the
// long sides. This is `extrude` from a lot footprint (city-plan §3.4.2).
// The box is shrunk about an interior anchor until its corners sit inside the
// footprint; `cornerOk`, when set, is an EXTRA per-corner constraint folded into
// that fit (the Living City passes "far enough from every road centreline", so a
// building can never overhang the sidewalk — device feedback).
Scope scopeFromFootprint(const Poly2& footprint, Real baseY, Real height,
                         const std::function<bool(const Vec2&)>& cornerOk = {});

// The output of growing a building: geometry grouped by part (each part one
// RenderMesh with its own material), plus attach points the composition layer
// (ADR-0028 §2) can populate with props, and a coarse proxy for HLOD/impostors
// (ADR-0038 §6). All meshes are world-space.
struct AttachPoint {
    Vec3 position;
    Vec3 normal;        // outward facade normal (for oriented props/signage)
    std::string tag;    // "facade", "entrance", "roof", ...
    Real width = 0;     // opening extent along the facade (entrance: door span)
    Real height = 0;    // opening height (entrance: head above the foot)
};

// A FURNITURE piece placed in a building (buildings M4b): which piece of the kit (procgen/furniture_kit.h, Piece),
// in which variant, where -- drawn INSTANCED by the interior system, not merged into the parts.
struct PlacedPiece {
    uint8_t piece = 0;
    uint32_t variant = 0;
    Mat4 xform;   // piece space -> world
};

struct BuildingMesh {
    std::vector<RenderMesh> parts;     // one per non-empty PartId, materialIndex set
    std::vector<PlacedPiece> furniture;   // the interior's pieces (growInterior only)
    std::vector<AttachPoint> attaches;
    RenderMesh proxy;                  // coarse single-material mass (LOD/impostor bake)
    Real height = 0;

    // Merge all parts into one mesh (single-material preview / collision).
    RenderMesh merged() const;
};

struct BuildingParams;  // declared below (interiorLayout takes it by ref)

// Where the stair goes in an enterable building (ADR-0080): chosen from the
// plan alone -- deterministic, terrain-free -- so the ground ceiling's well
// hole (grown with the exterior at Full detail) and the streamed interior's
// stair (growInterior) are the SAME layout by construction.
struct InteriorLayout {
    bool hasStair = false;  // false: nothing fit -- no well is cut
    bool dogleg = false;    // two half flights + a landing (short edges)
    Vec2 stairFoot;         // XZ centre of the bottom riser
    Vec2 stairDir;          // unit XZ direction of climb
    Real width = 1.2;       // flight width
    Real run = 0;           // horizontal run of one full-storey flight
    Real tread = 0.25;      // horizontal depth per step (riser 0.28 pitch,
                            // ~48 degrees)
    std::size_t edge = 0;   // plan edge the stair hugs (the facade blanks
                            // its windows there -- a stair crossing panes
                            // read wrong; device feedback)
    Poly2 well;             // the well rectangle (cut from ceiling/slabs)
};
InteriorLayout interiorLayout(const Poly2& plan, const BuildingParams& params,
                              std::size_t entranceEdge);

// The storey stack (ADR-0080): every storey's plan, base height (relative to
// baseY) and height, with the setback/inset validation the exterior tier
// loop applies. Extracted so the EXTERIOR massing and the streamed INTERIOR
// (growInterior) agree on where every floor is by construction. `tier`
// increments at each accepted setback. Entry 0 is the ground storey.
struct StoreyPlan {
    Poly2 plan;
    Real y0 = 0;   // storey base, relative to baseY
    Real h = 0;    // storey height
    int tier = 0;
};
std::vector<StoreyPlan> storeyPlans(const Poly2& plan,
                                    const BuildingParams& params);

// The MASS STACK (ADR-0086 point 5): the tiers a building is built from — one
// plan per tier and the floor it starts at, the base first — derived from the
// plan and the params alone, so the exterior massing, the interior and any
// audit agree. Envelope::None reproduces the uniform setbackEvery/setbackFloors
// offsets exactly; storeyPlans consumes this.
struct MassTier {
    Poly2 plan;
    int floor0 = 0;   // the first floor (above the ground storey) this tier serves
};
std::vector<MassTier> massStack(const Poly2& plan, const BuildingParams& params);

// The street-facing edge index of a CCW plan — the edge the door lands on.
// Shared by growPlanBuilding, interiorLayout callers and growInterior so no
// two layers ever pick different front doors.
std::size_t entranceEdgeFor(const Poly2& plan, const BuildingParams& params);

// The building's interior floor finish (dark walnut / light oak / stone
// tile), seed-picked. The streamed-interior system binds the matching
// surface bake per part material.
RenderMaterial floorFinishFor(const BuildingParams& params);

// The part the LOBBY floor overlay lands in (merged city mesh): non-wood
// finishes use their own part so the loader binds the matching bake.
PartId floorFinishPartFor(const BuildingParams& params);

// Interior wall PAINT (walls only -- ceilings stay drywall white so the
// no-GI emission bounce keeps rooms readable): a soft seed-picked palette,
// bright by construction (device: "not all just white walls").
Vec3 interiorPaintFor(const BuildingParams& params);

// The stair's OWN finish, an axis independent of the floor (device:
// "different materials to the stairs for different looks"), plus the part
// its streamed mesh lands in so the matching bake binds.
RenderMaterial stairFinishFor(const BuildingParams& params);
PartId stairFinishPartFor(const BuildingParams& params);

// The streamed interior (ADR-0080 Phase 2): a floor slab, inner walls and
// the stairwell for every storey above ground, deterministic from the SAME
// (plan, params, baseY) regen key as the exterior — storeyPlans and
// interiorLayout are the shared truth, so the slabs land exactly where the
// facade says the floors are and the stair rises through the well the
// exterior's ground ceiling already cut. `colliderOut`, when given, receives
// only what needs physics: slab tops, treads, risers, landings, railings
// (the prism already blocks the walls). The ground storey's shell (inner
// walls + ceiling) ships with the exterior grow (openDoorway); this is
// everything above it — plus, when the building has a CORE (core_plan.h),
// the core's shaft walls, flights and landings on EVERY storey in range,
// the ground included (the lobby's core is streamed; its ceiling holes are
// the exterior's). `k0`/`k1` select the storeys [k0, k1) to grow (k1 < 0 =
// all): each storey's slab, inner walls and core, the top ceiling only when
// the range reaches the top — a tall building streams a window of floors
// around the player. No RNG — two calls are identical.
// THE LOBBY'S DRESSING (M5; #60): a reception desk facing the entrance, with its counter top and a planter
// at each end, as footprints -- a centre, the desk's width axis `u` (square to the way it faces) and its
// facing axis `v` (toward the door), and a height band. Placed between the core and the door and slid
// toward the door until the whole group clears the core (stair shafts included) by a walkway and stays
// inside the plan; without the room for the planters, the desk alone; without room for that, nothing.
struct LobbyPiece {
    Vec2 c, u, v;
    Real w = 0, d = 0, h0 = 0, h1 = 0;   // along u, along v, and the height band above the lobby floor
    Vec3 colour;
    bool collide = true;
    Poly2 footprint() const;
};
struct CorePlan;
std::vector<LobbyPiece> lobbyDressing(const Poly2& plan, std::size_t entranceEdge, const CorePlan& core);

BuildingMesh growInterior(const Poly2& plan, const BuildingParams& params,
                          Real baseY, RenderMesh* colliderOut = nullptr,
                          int k0 = 0, int k1 = -1);

// Parameters for the mid-rise mixed-use hero (ADR-0038 §7) and its variants. All
// lengths in metres. A skyscraper is just this with a big `floors` and setbacks.
// An OPENING design (building-grammar-plan.md P2) — the first ELEMENT: a
// parametric window/door assembly the facade splitter stamps once per bay.
// The opening's real shape is cut into the wall (an arched head is a
// tessellated arc, not a square hole), a FRAME sits in the reveal, and the
// glass sits inside the frame — with optional partitioned lights (muntins),
// a projecting sill, and a hood (flat header band, or a voussoir band that
// FOLLOWS the arch). Archetype tables pick styles; the geometry emitter is
// shared. More elements (cornices, quoins, balconies, storefronts) join the
// same pattern.
struct OpeningStyle {
    enum class Head : uint8_t { Flat, Segmental, Round };
    enum class Hood : uint8_t { None, Band, Arch };
    Head head = Head::Flat;
    Real archRise = 0.30;      // segmental arch rise as a fraction of the span
    Real frameWidth = 0.09;    // face width of the frame border (m)
    int  lightsX = 1;          // pane columns (muntin partitions)
    int  lightsY = 1;          // pane rows (below the springline on an arch)
    bool sill = true;          // projecting sill course under the opening
    Hood hood = Hood::Band;    // Band = flat header; Arch = follows the arc
    Vec3 frameColor{0.93, 0.91, 0.86};   // painted timber / metal frame
};

struct BuildingParams {
    int   floors = 5;            // residential floors above the ground floor
    bool  groundRetail = true;   // taller, glassier ground floor with an entrance
    Real  floorHeight = human::FLOOR_HEIGHT;
    Real  groundHeight = human::GROUND_HEIGHT;
    Real  bayWidth = 3.5;        // target facade bay (window module) width
    Real  windowInset = 0.12;    // window recess depth
    bool  walkableGround = true; // hollow the ground floor + punch a real entrance (ADR-0038 §4)
    Real  wallThickness = 0.3;
    // Enterable buildings (ADR-0080): emit the entrance as an OPEN aperture --
    // no leaf, no doorframe -- with the reveal deepened to wallThickness so
    // jambs/lintel/threshold read as a real wall section. The lot layer sets
    // this per unit (the spawn building for now); everything else is
    // byte-identical with it false.
    bool  openDoorway = false;
    Real  setbackEvery = 0;      // >0: step the mass back this much every N floors
    int   setbackFloors = 0;     // floors between setbacks (0 = none)
    Real  parapet = human::PARAPET;
    Vec3  wallColor{0.72, 0.70, 0.66};
    bool  curtainWall = false;   // all-glass facade (downtown towers)
    bool  solidFacade = false;   // mostly-solid walls + a high clerestory strip
                                 // (warehouses/industrial): few windows, no glass bays
    // Facade ornamentation (the "decorative bricks, pillars, awnings" axis).
    // Traditional styles (brick/stucco/concrete) switch these on; a glass curtain
    // wall leaves them off for a clean skin.
    // Which part the facade walls are emitted under — picks the wall's procedural
    // material (Brick/Concrete/Stucco/Metal from the library, or the flat Wall).
    PartId wallPart = PartId::Wall;
    // World XZ direction toward the street, so the entrance is placed on the face
    // that points at the road (not into an alley/courtyard). Default +Z.
    Vec3  faceDir{0, 0, 1};
    // How far BELOW the storey base the ground sits at the entrance (m) —
    // the lot layer samples it (the grammar stays terrain-free) and the
    // entrance steps extend down to meet it, so the stoop lands on real
    // ground instead of hovering at the plinth (floorplan-conformance
    // round; Glenn's foundation-block design). 0 = flat ground.
    Real  entranceDropBelow = 0;
    bool  baseCourse = true;     // a wider, darker plinth — the foundation/base
    bool  stringCourse = true;   // an oversailing cornice band atop the ground floor
    bool  pilasters = false;     // vertical piers framing each bay (run full height)
    bool  awning = true;         // a projecting ledge over the entrance
    // The upper-storey window ELEMENT (OpeningStyle above): head shape, frame,
    // lights, sill/hood. Ground retail keeps flat storefront openings.
    OpeningStyle window;
    // Quoins: alternating corner masonry blocks up every building arris —
    // traditional on brick/stucco, and they hide the thin-texture corner edge.
    bool  quoins = false;
    // VEHICLE BAYS on the street face (>0): the ground floor's entrance edge
    // becomes a bay-door front — wide segmented roller doors with reveals and
    // a lintel band. Fire stations, loading docks, parking entries.
    int   groundBays = 0;
    // CLASSICAL entrance elements (the mesh-op vocabulary: lathe/array/steps).
    // portico (>0): a colonnade of that many lathe-turned columns carrying an
    // entablature + pediment in front of the entrance; entranceSteps: a porch
    // platform with descending steps under the door; dome: a drum + colonnade
    // + dome ROTUNDA crowns the flat roof (capitols, town halls) instead of
    // the mechanical penthouse.
    int   portico = 0;
    bool  entranceSteps = false;
    bool  dome = false;
    // Roof form (P3.c): Flat keeps the parapet deck; Gable/Hip raise a pitched
    // roof over the top plan (rect-ish plans only — an odd plan falls back to
    // Flat until a straight-skeleton pass exists). Residential vocabulary.
    // Sawtooth is the factory roof: north-light teeth with clerestory glass.
    enum class RoofStyle : uint8_t { Flat, Gable, Hip, Sawtooth };
    RoofStyle roofStyle = RoofStyle::Flat;
    Real  roofPitch = 0.55;      // rise/run of the pitched roof
    // Street-aware facades (P3.c): when true, ground-floor RETAIL storefronts
    // appear only on edges whose outward normal faces the street (faceDir);
    // side/rear edges wear plain residential ground walls — a building has a
    // FRONT (living-city realism).
    bool  retailStreetOnly = false;
    // BALCONIES: a slab + railing stamped per bay on street-facing upper
    // floors (condos / modern flats). Skipped on curtain/solid facades.
    bool  balconies = false;
    // PORCH: a covered timber entrance porch — platform, posts, shed roof —
    // in front of the door (bungalows, craftsman houses).
    bool  porch = false;
    // CHIMNEY: a masonry stack through the pitched roof near the ridge.
    bool  chimney = false;
    // SPIRE: a stepped crown + lathe-turned finial mast on the flat roof
    // (art-deco towers) instead of the mechanical penthouse.
    bool  spire = false;
    // STEEPLE: a square bell tower over the entrance bay, capped by a
    // pyramidal spire (churches, chapels).
    bool  steeple = false;
    // PARKING DECKS: upper storeys become open decks — a solid spandrel band,
    // an open air gap, and slim piers per bay (parking garages).
    bool  parkingDecks = false;
    // SIDE vehicle bays (>0): roller doors on the edge NEXT to the street
    // face — the attached-garage / loading-side vocabulary.
    int   sideBays = 0;
    Vec3  trimColor{0.40, 0.38, 0.35};
    BuildingShape shape = BuildingShape::Box;
    int   tiers = 5;             // pagoda: number of stacked tiers (odd reads best)
    int   sides = 32;            // cylinder: facets around the round mass
    uint32_t seed = 0;
    // MASSING ENVELOPE (skyscrapers v2, ADR-0086 point 5): how the mass above
    // the street wall is shaped, as a few numbers so the regen key stays a
    // POD and exterior and interior derive the same tiers (massStack).
    //   None               today's uniform setbackEvery/setbackFloors offsets.
    //   StreetWallSetback  the 1916 New York rule: the base fills the plan for
    //                      `baseFloors`, steps back `setback1` on every side
    //                      above it, then `stepDepth` every `stepFloors`, and
    //                      from `towerFloor` a shaft covering `towerFrac` of
    //                      the base plan rises to the top (Empire State,
    //                      Chrysler). Floors count above the ground storey.
    //   SkyExposure        the SKY EXPOSURE PLANE (New York 1916/1961, Glenn 2026-09-30: "rules about building
    //                      step back to avoid having the whole city in shadow"): the street wall rises
    //                      `baseFloors`, then the mass steps back every `stepFloors` along a plane rising
    //                      `skyRatio` metres per metre back (1961: 2.7 on a narrow street, 5.6 on a wide one),
    //                      the first step at least `setback1`, until it covers `towerFrac` of the lot -- above
    //                      that a tower may rise straight (the wedding cake; Empire State, 1916).
    //   Taper              a tower on its lot's rectangle from `baseFloors`, narrowing to `taperTop` of its
    //                      width at the top, its corners chamfered up to `chamferTop` of the short side (One
    //                      World Trade Center), in steps of a few floors.
    //   Slab               a thin rectangular slab on the lot's long axis from `baseFloors` (UN Secretariat,
    //                      Lever House) -- its width `towerFrac` of the short side, never under a core's 14 m.
    //   Feathered          a pencil tower (432 Park, 111 W 57th): the lot's rectangle full height, stepping back
    //                      on ONE face every two floors from `featherFrom` of its height, then a second face.
    //   Twist, Stack       overhanging forms (milestone 3): each tier rotated `twistDeg` over the height /
    //                      shifted `stackShift` metres from the one below, with a soffit under the overhang.
    enum class Envelope : uint8_t { None, StreetWallSetback, SkyExposure, Taper, Slab, Feathered, Twist, Stack };
    Envelope envelope = Envelope::None;
    int   baseFloors = 5;
    Real  setback1 = 6.0;
    int   stepFloors = 10;       // 0 = no later steps
    Real  stepDepth = 3.0;
    Real  towerFrac = 0.35;      // 0 = no shaft
    int   towerFloor = 20;       // 0 = no shaft
    Real  skyRatio = 2.7;        // SkyExposure: metres up per metre back
    Real  taperTop = 0.7;        // Taper: width at the top, as a fraction of the base's
    Real  chamferTop = 0.0;      // Taper: corner cut at the top, fraction of the short side
    Real  featherFrom = 0.7;     // Feathered: where the one-sided steps begin, fraction of floors
    Real  twistDeg = 0.0;        // Twist: total rotation over the height (degrees)
    Real  stackShift = 0.0;      // Stack: each tier's offset from the one below (m)
    // THE CURTAIN STYLE (skyscrapers NYC variety M2): glassTint 0 auto grey, 1 blue, 2 green, 3 bronze, 4 smoke,
    // 5 silver (reflective), 6 clear; mullionTone 0 steel, 1 bronze, 2 black, 3 silver, 4 white; fins: a vertical
    // fin every `fins` bays (0 none); curtainBay the mullion spacing (m); spandrelFrac the opaque band's share of
    // the storey (0 = floor-to-ceiling glass, ~0.45 = ribbon windows).
    uint8_t glassTint = 0;
    uint8_t mullionTone = 0;
    uint8_t fins = 0;
    Real  curtainBay = 1.6;
    Real  spandrelFrac = 0.30;
    // THE CORE (skyscrapers v2 M5, core_plan.h): 0 = auto (four floors and
    // up get an elevator bank and two enclosed stairwells when one fits),
    // 1 = never (the straight stair of ADR-0080, or nothing), 2 = always.
    uint8_t core = 0;
    // THE LIGHTING SPEC (skyscrapers v2 M4, Lua-overridable; ADR-0086 point
    // 10): what a tall roof wears at night. 0 = auto (the position hash
    // decides, as before). crown: 1 none, 2 white, 3 amber, 4 blue, 5 red,
    // 6 green, 7 purple. signage / uplights: 1 off, 2 on. Beacons follow the
    // height rule (the FAA's, not ours). Lua: crown = "amber", signage =
    // false, uplights = true.
    uint8_t crown = 0;
    uint8_t signage = 0;
    uint8_t uplights = 0;
    // THE TOP (buildings M2): what stands on a flat roof against the sky -- 0/1 the mechanical penthouse, 2 screen,
    // 3 sloped, 4 faceted, 5 lantern, 6 frame, 7 twin antennas, 8 mast (emitTowerTop). spire/dome win over it.
    // Lua: top = "screen" | "sloped" | "faceted" | "lantern" | "frame" | "antennas" | "mast" | "penthouse".
    uint8_t top = 0;
    // MASONRY DEPTH (buildings M3): windowGroup 1-3 -- windows in pairs or triples, a slim mullion between them and
    // a broad pier between groups; verticals -- full-height piers proud of the wall at every group, the spandrel
    // panels between storeys recessed and darker (Rockefeller Center, the Empire State). Lua: window_group = 2,
    // verticals = true.
    uint8_t windowGroup = 1;
    bool  verticals = false;
    // WHAT IT IS FOR (buildings B): a residential building's typical floors are whole apartments off a corridor
    // (room_plan.h, PlateTopology::Apartments); anything else keeps its office ring. Set from the recipe's use.
    bool  residential = false;
    // ATTACHED BUILDINGS (Glenn, 2026-10-01: "in the dense part of town ... buildings right next to each other ...
    // windows aren't made on the sides and we have back doors and different ways up to the second floor"). Up to
    // two PARTY WALLS, each the world line it stands on: outward normal (x, z) and offset dot(normal, point). A
    // plan edge lying on one is a blank wall, ground to roof, inside and out. backDoor: the rear edge's middle bay
    // is a plain service door. fireEscape: a steel fire escape climbs the rear face -- a landing at every floor,
    // a flight between landings, a drop ladder over the yard. Set by the lot pass (city_lots), never by recipes.
    uint8_t partyWalls = 0;
    Vec2  partyN[2] = {Vec2(0, 0), Vec2(0, 0)};
    Real  partyAt[2] = {0, 0};
    bool  backDoor = false;
    bool  fireEscape = false;
    // A BIG-BOX STORE (architectBigBox): 0 none, else the chain -- 1 warehouse club, 2 electronics, 3 home
    // improvement, 4 discount store -- or 5, an INDOOR MALL (architectMall: a concourse of shop units, an anchor store,
    // a food court; mallLayout). The front edge wears the chain's sign and canopy, the rear its loading docks,
    // a band in the chain's colour (trimColor) rings the top; inside, one store floor (bigBoxRoomPlan).
    uint8_t bigBox = 0;
    // A UNIVERSITY building (the campus): 0 none, 1 a teaching hall (lecture halls, classrooms, labs), 2 the library
    // (a reading room, the stacks), 3 a residence hall (dorm rooms, a lounge, shared baths). A stair building
    // (core = 1): its floors are rooms either side of a corridor from the stair (campusPlan, room_plan.cpp).
    uint8_t campus = 0;
    // THE SHOP MIX of its ground floor (trades.h, ~/.claude/plans/nightlife-and-malls.md): 0 everyday (a residential
    // street's cafes, grocers, pharmacies), 1 high street, 2 nightlife (bars, clubs, restaurants), 3 a mall's. Set
    // by the lot pass from the district; the trades draw their weights from it.
    uint8_t shopMix = 1;
};

// THE SHOPFRONTS of a building's ground storey (buildings: shops; the facade's own ShopUnits): each shop's stretch
// of wall (world XZ, a -> b along the facade), the facade's outward normal, its door's centre and its trade (0 cafe,
// 1 grocery, 2 boutique, 3 bookshop, 4 electronics, 5 pharmacy, 6 bakery). The lot pass lays café terraces in front.
struct ShopFront {
    Vec2 a, b, n, door;
    uint8_t trade = 0;
    // its FASCIA (the sign band over the glazing), metres above the storey's base, standing `fasciaProud` proud of
    // the wall; fasciaY1 <= fasciaY0 when the storey is too low to carry one
    Real fasciaY0 = 0, fasciaY1 = 0, fasciaProud = 0.16;
    // INDOORS: a unit of an indoor mall (bigBox 5), its front on the concourse -- no terrace, and the sim's way in is
    // the mall's own street door (`entry`), not this one
    bool indoor = false;
    Vec2 entry;
};
std::vector<ShopFront> shopFrontsOf(const Poly2& plan, const BuildingParams& params);
// The SHOP ROOMS growInterior fits out behind those fronts (a unit too shallow, or squeezed out at a corner, has
// none): each room's floor rectangle and its trade.
struct ShopRoomRect { Poly2 rect; uint8_t trade = 0; };
std::vector<ShopRoomRect> shopRoomsOf(const Poly2& plan, const BuildingParams& params, Real baseY);
// An indoor mall's name board over its doors (emitBigBoxDress): its face's centre (XZ), outward normal, its band
// (y0..y1 above the base) and width. False for anything else.
bool mallSignOf(const Poly2& plan, const BuildingParams& params, Vec2& centre, Vec2& n, Real& y0, Real& y1, Real& width);
// THE PARKING GARAGE you can drive into (parkingDecks; buildGarage): its collider -- every deck, ramp and rail -- in
// world space (false: not a drivable garage, keep the prism); its stalls (centre, the way a parked car faces, height
// above the base); and its portal (the middle of the opening on the street, and the way out).
struct ParkingStall { Vec2 at, face; Real y = 0; };
// Offset a CCW plan polygon: d > 0 shrinks (inset), d < 0 grows (outset) -- the grammar's own offsetPlan.
Poly2 offsetPlanPublic(const Poly2& poly, Real d);
bool garageColliderOf(const Poly2& plan, const BuildingParams& params, Real baseY, std::vector<Vec3>& vertices,
                      std::vector<uint32_t>& indices);
std::vector<ParkingStall> garageStallsOf(const Poly2& plan, const BuildingParams& params);
bool garagePortalOf(const Poly2& plan, const BuildingParams& params, Vec2& at, Vec2& out);

// Is plan edge `e` a party wall (it lies on one of params' party lines, facing out across it)?
bool partyEdge(const Poly2& plan, const BuildingParams& params, std::size_t e);
// The REAR edge: the longest edge facing away from the street (normal . faceDir < -0.7) that is not a party wall;
// plan.size() when there is none.
std::size_t rearEdgeOf(const Poly2& plan, const BuildingParams& params);
// A campus hall's door onto its quad (params.backDoor): the rear face's middle bay, or -- when the stair hugs that wall,
// as it does in campus halls -- the bay a metre short of the flight's foot. Its centre on the wall and the wall's
// outward normal; false when there is none. The facade and the floor plan both place it by this.
bool campusQuadDoor(const Poly2& plan, const BuildingParams& params, std::size_t entranceEdge, Vec2& centre, Vec2& outward);
Real stairWallDoorX(Real wallLen, const BuildingParams& params);

// Facade DETAIL level (city-render-perf R2): the same grammar, two emissions.
// Full is today's facades — reveals, frames, muntins, sills, cornices, trim.
// Flat is the middle LOD: one quad per wall, one flat pane per opening, roof
// planes and parapet kept for the silhouette, every ornament element skipped.
// The two levels consume the SAME facade layout (the splitter decides bay and
// opening placement once), so they can never disagree about where a window or
// the door is — the first in-engine step of the blueprint model
// (lot-system-plan §15.2). Cylinder and Pagoda shapes currently emit Full at
// both levels (they are already lean; their flat pass is a later follow-up).
enum class FacadeDetail : uint8_t { Full, Flat };

// THE CURTAIN STYLE a glass facade is drawn in (skyscrapers NYC variety M2; the numbers live on BuildingParams:
// glassTint, mullionTone, fins, curtainBay, spandrelFrac). The defaults are the curtain wall as it always was:
// grey glass from the wall colour, steel mullions every 1.6 m, a 30% spandrel band (at most 0.9 m), no fins.
struct CurtainStyle {
    uint8_t glassTint = 0, mullionTone = 0, fins = 0;
    Real bay = 1.6, spandrelFrac = 0.30;
    Real spandrelH(Real storey) const {   // the opaque band: 0 = floor-to-ceiling glass (a slim slab edge stays)
        return spandrelFrac <= 0.02 ? std::min(Real(0.14), storey * 0.05) : std::min(Real(1.4), storey * spandrelFrac);
    }
    int bays(Real width) const { return std::max(1, static_cast<int>(std::lround(width / std::max(Real(0.8), bay)))); }
};
CurtainStyle curtainStyleOf(const BuildingParams& p);
// MECHANICAL FLOORS (buildings M1): a tall tower's plant storeys -- a louvre band in place of its windows, never lit,
// no rooms inside -- every 15-25 floors (by the seed) on a tower of 30+ floors, and the storey under the roof of a
// 40+ one. `floor` counts from 1 (the first storey above the ground storey). Derived from the params alone, so the
// exterior, its far tier and the interior agree.
bool mechanicalStorey(const BuildingParams& p, int floor);

// Grow a building into `scope` (ADR-0038 §2). Deterministic for `params.seed`.
BuildingMesh growBuilding(const Scope& scope, const BuildingParams& params,
                          FacadeDetail detail = FacadeDetail::Full);

// Grow a FLOORPLAN building (building-grammar-plan.md P3): the massing is a
// closed polygon — the lot's own shape, an L/T/U composition, a flatiron
// wedge — extruded storey by storey with the SAME facade/element machinery
// the box grammar uses (each plan edge becomes one facade rectangle: bays,
// windows, arches, frames, the door on the street-facing edge). Roof and
// ground slabs triangulate the plan; cornices are SWEPT around the plan
// outline; corner posts hide the wall miters at every vertex. Setbacks
// (setbackFloors/setbackEvery) shrink the plan by a uniform offset per tier —
// the base/shaft/capital stack — with a swept cornice at every transition.
// Winding is normalized internally; deterministic for `params.seed`.
BuildingMesh growPlanBuilding(const Poly2& plan, const BuildingParams& params,
                              Real baseY = 0.0,
                              FacadeDetail detail = FacadeDetail::Full);

// The default material for a part class (PBR; ADR-0017/0032). Recipes may override.
RenderMaterial materialFor(PartId id, const Vec3& wallColor);

// LATHE op (the mesh-op library's Lua face too, as mesh.lathe): revolve a 2D
// profile of (radius, height) rows around the +Y axis at `center` (center.y =
// the profile's y origin). Low-poly faceted normals; rows with radius ~0
// close into fans. Columns, domes, finials, balusters are all this one op.
RenderMesh latheMesh(const Vec3& center, const std::vector<Vec2>& profile,
                     int segments, const Vec3& color);

// --- The op vocabulary, exposed so Lua/C++ recipes compose buildings directly
// (city-plan §4.3). Each appends geometry to `out` under a part id. ----------

// Emit the six faces of a scope as a solid box under `part`.
void emitBox(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color);
// A SOFT BOX of foliage (the flora plan): a clipped hedge, a bush, a planted bed. The scope's
// box with rounded edges and a noise-lumped surface (~30 cm cells, five faces, no bottom),
// normals leaning toward the box's own centre so it lights as one soft mass, colour dark at the
// base and lighter on top. `color` is read as sRGB (the city's foliage colours were authored
// that way) and made linear. ~100-250 triangles where emitBox makes 12. `seed` 0 = from its position.
void emitSoftBox(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color, uint32_t seed);
// Emit only the four vertical walls of a scope (a hollow storey: floor+ceiling
// optional) under `part` — the walkable-shell primitive (ADR-0038 §4).
void emitShell(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color,
               bool floor, bool ceiling);
// Emit a solid parapet ring (four thin boxes with real thickness) around a
// footprint perimeter — a roof-edge wall that reads from every angle.
void emitParapet(BuildingMesh& out, const Vec3& footOrigin, Real width, Real depth,
                 const Vec3& r, const Vec3& f, Real y, Real height, Real thick,
                 PartId part, const Vec3& color);
// Emit a quad (one facade panel / window / sign) from four corners + a normal.
void emitQuad(RenderMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
              const Vec3& d, const Vec3& normal, const Vec3& color);
// Split a scope along a local axis (0=right,1=up,2=forward) into child scopes of
// the given sizes; a negative size means "repeat to fill" with |size| as target.
std::vector<Scope> splitScope(const Scope& s, int axis,
                              const std::vector<Real>& sizes);
// Repeat-divide a scope along an axis into N≈(extent/target) equal child scopes.
std::vector<Scope> repeatScope(const Scope& s, int axis, Real target);
// Inset a scope inward on its footprint (right/forward) by d, keeping height.
Scope insetScope(const Scope& s, Real d);

}  // namespace engine

#endif
