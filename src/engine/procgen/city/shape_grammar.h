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
    Count    // KEEP LAST: materialIndexFor is the ordinal; arrays size by Count
};

// Deterministic per-opening night-light choice, shared by every facade
// emitter (full, flat, curtain wall) so LOD transitions never flicker a
// window on or off: quantized world position in, stable coin-flip out.
bool litWindow(const Vec3& worldPos);
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

struct BuildingMesh {
    std::vector<RenderMesh> parts;     // one per non-empty PartId, materialIndex set
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
    Real tread = 0.26;      // horizontal depth per step (riser 0.24 pitch)
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

// The street-facing edge index of a CCW plan — the edge the door lands on.
// Shared by growPlanBuilding, interiorLayout callers and growInterior so no
// two layers ever pick different front doors.
std::size_t entranceEdgeFor(const Poly2& plan, const BuildingParams& params);

// The streamed interior (ADR-0080 Phase 2): a floor slab, inner walls and
// the stairwell for every storey above ground, deterministic from the SAME
// (plan, params, baseY) regen key as the exterior — storeyPlans and
// interiorLayout are the shared truth, so the slabs land exactly where the
// facade says the floors are and the stair rises through the well the
// exterior's ground ceiling already cut. `colliderOut`, when given, receives
// only what needs physics: slab tops, treads, risers, landings, railings
// (the prism already blocks the walls). The ground storey's shell (inner
// walls + ceiling) ships with the exterior grow (openDoorway); this is
// everything above it. No RNG — two calls are identical.
BuildingMesh growInterior(const Poly2& plan, const BuildingParams& params,
                          Real baseY, RenderMesh* colliderOut = nullptr);

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
};

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
