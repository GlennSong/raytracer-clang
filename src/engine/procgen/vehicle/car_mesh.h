#ifndef RAYTRACER_ENGINE_PROCGEN_VEHICLE_CAR_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_VEHICLE_CAR_MESH_H

#include "../../../renderer/renderer.h"
#include "../../../rt_math.h"

#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// LOW-POLY vehicle bodies, superseding the lofted buildCarShell (ADR-0062).
//
// The old cars were compositions of axis-aligned boxes and every one read as a
// cybertruck; the loft that replaced them regressed on device. This is neither:
// a chamfered profile polygon swept along stations placed at the SILHOUETTE
// BREAKS (nose, hood, A-pillar, C-pillar, deck, tail), emitted FLAT-SHADED
// through MeshBuilder::emitQuad.
//
// Three properties fall out of that choice, and they are the point:
//
//  * emitQuad takes an explicit normal and orients winding to match it, so the
//    normal is COMPUTED from each cell's own corners rather than guessed by an
//    outward heuristic. That was ADR-0062 defect #1.
//  * Per-cell vertices are duplicated, so there is no vertex-normal or
//    vertex-colour smearing across a crease. That was defect #2.
//  * Wheels and lamps are SEPARATE parts the caller places (physics wheels,
//    emissive lenses); the generator never bakes them into the body. That was
//    defect #3 — the doubled wheels and lamps.
//
// Windows and wheel arches are REAL HOLES: their cells are omitted from the
// body, and (from P1) an inner skin plus reveal quads close the cavity. Backface
// culling is unconditional in the Metal backend, so a hollow cabin only reads if
// the inward-facing geometry actually exists.
//
// Conventions match the rest of the engine's vehicles: the car faces +Z, +X is
// its right, origin at the BODY CENTRE, `size` = (width, height, length).

enum class CarLod : std::uint8_t { High, Mid, Low, Proxy };

// Parts are split by MATERIAL, not by anatomy: one RenderMesh per material the
// caller will bind, because Renderable/InstanceGroup each carry exactly one
// RenderMaterial. Body carries paint and trim together (they differ only in
// vertex colour); Glass must be separate because it is the only transparent one.
enum class CarPart : std::uint8_t { Body, Glass, Interior, Lamp, Wheel, Count };
constexpr int kCarPartCount = static_cast<int>(CarPart::Count);

// A key in the roofline / sill / plan curves: `z` is a normalised station along
// the car in [-0.5 tail .. +0.5 nose], `v` the value there. Evaluated with
// smoothstep between keys.
struct CarKey {
    Real z = 0;
    Real v = 0;
};

// A span along the car in the same z01 units, z0 <= z1 (z1 is the nose side).
struct CarBand {
    Real z0 = 0;
    Real z1 = 0;
};

// A named mount point on the finished body, in body-local space. Lamp tags match
// what citysim's syncCarLamps already parses ("headlight_l", "taillight_r", ...)
// so emissive lenses land exactly on the drawn housing.
struct CarAttach {
    Vec3 position{0, 0, 0};
    Vec3 normal{0, 0, 1};
    std::string tag;
};

struct CarMesh {
    RenderMesh parts[kCarPartCount];
    std::vector<CarAttach> attaches;
    Vec3 halfExtent{0, 0, 0};

    // Every part in one mesh — for collision proxies, tests, and the offline
    // path tracer, which is two-sided and single-material anyway.
    RenderMesh merged() const;
};

struct CarParams {
    // --- package (metres; recipes derive these from vehicle_classes.lua) ----
    Real width = 1.82;
    Real height = 1.45;
    Real length = 4.59;
    Real wheelDiameter = 0.62;
    Real frontOverhang = 0.90;
    Real rearOverhang = 1.00;
    Real groundClearance = 0.14;      // SAE H156

    // --- form --------------------------------------------------------------
    // Stations in z01, ordered NOSE FIRST (descending). These are silhouette
    // breaks, not uniform samples: a uniform sweep spends polygons on flat
    // panels and rounds off exactly the creases that make a low-poly car read.
    std::vector<Real> stations;

    std::vector<CarKey> roof;         // roofline, fraction of height
    std::vector<CarKey> sill;         // rocker line, fraction of height
    std::vector<CarKey> plan;         // plan-view half-width scale

    Real chamfer = 0.16;              // corner chamfer, fraction of half-extent
    int topPts = 6;                   // profile points across the top run
    int bottomPts = 3;                // ... across the floor
    Real tumblehome = 7.0;            // degrees the sides lean in above the belt

    // --- apertures ---------------------------------------------------------
    // The belt line is NOT authored: per Orsborn et al. it has no rules of its
    // own and emerges from the hood, side windows and trunk. We derive it as the
    // roofline height at the cowl — which is exactly the hood line carried aft.
    CarBand windshield{0.02, 0.24};
    CarBand backlight{-0.40, -0.22};
    std::vector<CarBand> sideWindows;
    std::vector<Real> pillars;        // z01 centres of body-colour pillars
    Real pillarHalfWidth = 0.030;

    // A windshield does not reach the car's edges and side glass does not reach
    // its roof — the leftover strips ARE the A-pillar and the roof rail, and
    // without them the greenhouse reads as one hole punched through the car.
    // `glassHalfU` bounds top-run glass laterally (fraction of half-width);
    // `railDrop` keeps side glass this far below the roofline (fraction of H).
    Real glassHalfU = 0.55;
    Real railDrop = 0.05;

    // THE BELTLINE RISES TOWARD THE REAR. This is the single thing that makes a
    // side window read as a car's rather than as a rectangle cut in a slab: the
    // daylight opening is a WEDGE that closes toward the back from both edges at
    // once — beltline climbing, roofline falling — and a good one "makes your eye
    // move quickly from front to back". A flat belt has no direction, so the
    // whole side reads static however good the roofline is.
    //
    // The front run (cowl to B-pillar) is essentially flat; the kick happens aft
    // of the B-pillar and does ~75-80% of the total rise there.
    Real beltKick = 0.10;         // fraction of height the belt climbs by the tail
    Real beltKickStart = -0.09;   // z01 where the kick begins (at/just aft of B)
    Real beltKickEnd = -0.40;     // ...and where it has finished climbing

    // Pillars are BLACKED OUT, not painted. Body-coloured pillars chop the
    // daylight opening into separate holes and make the flank look bulky; a dark
    // band is what lets two panes plus the pillar between them read as ONE
    // continuous graphic, which is how a real greenhouse is drawn.
    Vec3 pillarColor{0.06, 0.065, 0.075};

    // Side-glass edges lean back going up on a real car (A-pillar ~32 deg from
    // vertical, B ~8). TRIED AND REJECTED, kept as a knob because the code is
    // correct and a future finer grid could use it.
    //
    // A cell grid cannot draw a lean, only a staircase. With the sweep stations
    // that make it expressible at all, one step per pane edge reads as a NOTCH
    // out of the pillar rather than a slope, and three steps — the coarsest that
    // actually looks like a lean — took the car from 1058 to 3130 triangles.
    // That is 3x the budget to soften four edges, which is the wrong trade for a
    // deliberately faceted car. The vertical pillars stay.
    Real dloRake = 0.0;    // degrees from vertical, top leaning rearward
    Real archHalfSpan = 0.075;        // z01 half-width of a wheel opening
    Real archTop = 0.42;
    // Cells across each arch. The opening is a semi-ellipse and this is how many
    // steps draw it; 1 collapses back to the old rectangle.
    int archSegments = 4;

    // Lamp lenses. Drawn as their own part so the CALLER decides emission —
    // citysim overlays lit instances at the markers per state, so a baked glow
    // would double up and could never switch off.
    Real lampWidth = 0.30;
    Real lampHeight = 0.13;
    Real lampDepth = 0.10;

    // --- cavity ------------------------------------------------------------
    // Backface culling is unconditional, so a hole with nothing behind it shows
    // the sky through the car. `shellThickness` offsets an INNER skin inward
    // (the headliner and door cards — the surface you actually see through a
    // window); reveal quads bridge the two along every aperture edge, giving the
    // opening real thickness instead of a zero-width sliver.
    Real shellThickness = 0.055;      // outer skin -> inner skin, metres

    // Wheel arches are a RECESS, not a hole. A hole leaves the reveal's back
    // edge dangling — there is no inner skin out at the arches to close onto,
    // because the liner is deliberately glass-only — and a dangling edge is
    // invisible from one side, so at grazing angles you see straight through
    // the car. Re-emitting the cell at the inner offset gives the lip a back
    // wall: the wheel well. Dark, because a wheel well is in shadow.
    Vec3 archColor{0.05, 0.05, 0.055};
    Real glassInset = 0.022;          // glass sits this far inside the skin
    // Cabin floor as a fraction of height. 0.30 put the floor 0.43 m above the
    // ground on a 1.45 m car — real ones sit at 0.30-0.35 m — which left only
    // 0.94 m of headroom and pushed a seated occupant's head through the roof.
    Real cabinFloor = 0.22;

    // --- colours (vertex colours; materials stay white) --------------------
    Vec3 paint{0.72, 0.10, 0.10};
    Vec3 trimColor{0.07, 0.07, 0.08};
    Vec3 glassTint{0.16, 0.20, 0.24};
    Vec3 interiorColor{0.17, 0.17, 0.19};
    Vec3 seatColor{0.23, 0.21, 0.20};

    // --- cabin furniture (built by car_interior.cpp with the Scope grammar) --
    bool interior = true;
    int rows = 2;                     // seat rows; 0 = repeat-to-fill (a bus)
    Real seatPitch = 0.86;            // SAE L50 couple distance
    Real backAngle = 25.0;            // SAE A40, degrees from vertical
    Real steerDiameter = 0.37;        // SAE W9

    CarLod lod = CarLod::High;
};

// Build the body. Deterministic: same params in, byte-identical mesh out.
CarMesh buildCarMesh(const CarParams& p);

// A sedan with the class defaults from vehicle_classes.lua baked in — the
// starting point for recipes and the fixture the tests build against.
CarParams defaultSedanParams();

}  // namespace engine

#endif
