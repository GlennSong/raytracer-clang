#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_MASS_STACK_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_MASS_STACK_H

// THE STACK (architectural toolkit, issue #105, step 4; ADR-0174): a mass made of SECTIONS over STOREY RANGES, each
// under a keyed TRANSFORM -- rotate, scale, slide along its axes, keyed over the entry's height -- and standing in
// a RELATION to the entry below: nested inside it (a setback), flush with it, or SUPPORTED by it (an overhang: the
// storey above hangs past the one below within `maxOverhang`, resting on at least `minOverlap` of it). Today's
// tiers (massStack: the uniform setbacks, the street wall and its steps, the sky exposure plane, the taper, the
// slab, the feathered pencil) are the first preset -- stackOf -- with identity transforms, and the storey stack
// is the stack's slices: every building before this header is byte-identical through it. The Twist and Stack
// envelopes (rotated storeys; shifted modules) are the first transforms, live for the first time here.

#include "mass.h"
#include <cstdint>
#include <vector>

namespace engine {

// A value keyed over t in [0, 1] (the entry's storeys, its first 0 and its last 1), linear between keys.
struct Curve {
    std::vector<std::pair<Real, Real>> keys;   // (t, value), t ascending
    Real at(Real t, Real def) const;           // no keys: `def`
    bool flat(Real v) const;                   // every key is v (no keys: v is the default)
    static Curve constant(Real v);
    static Curve linear(Real v0, Real v1);
};

// How an entry's section moves up its storeys: scaled along its axes `u` / `v` about `pivot`, rotated about it by
// `rot` degrees, then slid along `u` / `v`. The axes are the entry's own (a rectangle's, the plan's box's).
struct Transform {
    Curve rot, scaleU, scaleV, slideU, slideV;
    Vec2 pivot{0, 0};
    Vec2 u{1, 0}, v{0, 1};
    bool identity() const;
    Section apply(const Section& s, Real t) const;
};

enum class Relation : uint8_t { Nest, Flush, Support };

struct StackEntry {
    Section section;
    int k0 = 0, k1 = 0;              // the storeys it covers, inclusive (0 = the ground storey)
    Relation rel = Relation::Nest;
    Transform xf;
    Real minOverlap = 0.6;           // Support: the fraction of a storey's area resting on the one below
    Real maxOverhang = 6.0;          // Support: how far a storey may hang past the one below (m)
    bool varies() const { return !xf.identity(); }
};

struct Cap {
    enum class Kind : uint8_t { Parapet, Flat, Spire, Dome, Vault, Sloped, Aperture, Flared };
    Kind kind = Kind::Parapet;
    Real rise = 0;
};

struct Stack : Mass {
    std::vector<StackEntry> entries;   // the base first; a storey belongs to the LAST entry whose k0 it reaches
    Cap cap;
    Real groundH = 4.0, floorH = 3.4;
    int floors = 0;                    // storeys above the ground
    // The entry a storey is in (the last whose k0 <= k), -1 when none.
    int entryOf(int k) const;
    // The storey's outline: its entry's section under the entry's transform at the storey's parameter.
    Section at(int k) const;
    // The tiers as massStack hands them out: one per entry while its transform is identity, one per storey where
    // it varies (a twisted entry's storeys are each their own tier for the core's fit), `floor0` the entry's first
    // FLOOR (storey - 1; the base 0).
    std::vector<MassTier> tiers() const;
    Slice slice(Real y) const override;
    Real height() const override { return groundH + floors * floorH; }
    MassBounds bounds() const override;
    RenderMesh shell(int lod, const Vec3& colour) const override;
};

// THE PRESET: today's tiers (envelopeTiers) as a stack with identity transforms; the Twist envelope as one entry
// over `twistDeg` about the plan's centre, the Stack envelope as modules of `stepFloors` storeys slid `stackShift`
// alternately along the plan's long axis -- both sized to stay inside the plan on every storey.
Stack stackOf(const Poly2& plan, const BuildingParams& params);

}  // namespace engine

#endif
