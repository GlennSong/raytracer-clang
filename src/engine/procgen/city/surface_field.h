#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SURFACE_FIELD_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SURFACE_FIELD_H

#include "../../../rt_math.h"       // Vec3
#include "../terrain.h"             // TerrainFlatten, FlattenGrid, buildFlattenGrid, applyFlatten
#include "../terrain_field.h"       // HeightField

#include <utility>
#include <vector>

namespace engine {

// SurfaceField — the authored-ground oracle (ADR-0075). ONE place that answers
// "how high is the ground at (x, z)", built from a natural base height plus a
// stack of cut/fill edits (road corridors, block grades, building pads) folded
// through applyFlatten with a spatial index. It is the mutable, indexed form of
// terrain_field's `conformField`: the READ side every system samples, and the
// WRITE side the corridor writer + block grader push edits into.
//
// The three historically interchangeable closures — HeightSampler
// (buildability.h), HeightField (terrain_field.h), and RoadEntity::heightAt — are
// this field's interchange form; `sampler()` hands one out for APIs that still
// take a bare closure. Ground is single-valued (2.5-D) by design: multi-valued
// structures (retaining walls, bridges, tunnels) are a separate StructureSet
// overlay, not part of this height field. Land CLASSIFICATION (water/beach/
// buildable) stays in buildability.h, layered on top of this height.
class SurfaceField {
public:
    SurfaceField() = default;
    explicit SurfaceField(HeightField base) : base_(std::move(base)) {}

    void setBase(HeightField base) { base_ = std::move(base); }
    bool hasBase() const { return static_cast<bool>(base_); }

    // Push a cut/fill footprint (road ramp, block grade, building pad). The
    // index goes stale until rebuildIndex(); until then queries use the linear
    // fold (same result, slower), so a forgotten rebuild is a perf bug, never a
    // correctness one.
    void addEdit(TerrainFlatten edit) {
        edits_.push_back(std::move(edit));
        dirty_ = true;
    }
    void setEdits(std::vector<TerrainFlatten> edits) {
        edits_ = std::move(edits);
        dirty_ = true;
    }
    void clearEdits() {
        edits_.clear();
        grid_ = FlattenGrid{};
        dirty_ = false;
    }
    void rebuildIndex() {
        grid_ = buildFlattenGrid(edits_);
        dirty_ = false;
    }

    const std::vector<TerrainFlatten>& edits() const { return edits_; }
    const FlattenGrid& index() const { return grid_; }

    // Authored ground height at (x, z). `dilate` grows every footprint for this
    // query (coarse CDLOD samplers pass ~half their cell so a triangle can't
    // interpolate natural ground across a narrow corridor — see applyFlatten).
    double height(double x, double z, double dilate = 0.0) const {
        double b = base_ ? base_(x, z) : 0.0;
        if (edits_.empty()) return b;
        if (!dirty_ && !grid_.empty())
            return applyFlatten(grid_, edits_, x, z, b, dilate);
        return applyFlatten(edits_, x, z, b, dilate);
    }

    // Surface normal from central differences on height(), at offset `eps` (m).
    Vec3 normal(double x, double z, double eps = 0.5) const;

    // A HeightField closure over this field, for APIs that take a bare sampler
    // (RoadEntity::heightAt, LotParams::ground, buildability). Captures `this`, so
    // the closure is valid only while this SurfaceField lives — the same
    // lifetime contract the engine's existing terrain closures already rely on
    // (they capture a stably-owned recipe). Rebuild the index before handing one
    // out if edits changed.
    HeightField sampler() const {
        const SurfaceField* self = this;
        return [self](double x, double z) { return self->height(x, z); };
    }

private:
    HeightField base_;
    std::vector<TerrainFlatten> edits_;
    FlattenGrid grid_;
    bool dirty_ = true;
};

}  // namespace engine

#endif
