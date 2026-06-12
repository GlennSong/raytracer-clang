#ifndef RAYTRACER_ENGINE_PROPERTIES_H
#define RAYTRACER_ENGINE_PROPERTIES_H

#include "../rt_math.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

struct Transform;
struct RenderMaterial;
struct LensParams;
struct SceneCamera;
struct SourceSpec;

// The property layer (editor-app plan): each component describes its editable
// fields ONCE — label, value reference, and editing semantics (range, scale,
// unit, choices) — and every consumer is a visitor: the Qt inspector, the
// in-viewport ImGui panels, JSON serialization, undo recording. The metadata
// describes what a field IS (an f-stop is bounded and log-scaled), never what
// widget shows it; each UI maps semantics to its own toolkit.
//
// Engine-only and UI-free by construction: visitors receive live references
// plus this metadata, nothing else crosses the seam.
struct FieldMeta {
    const char* label = "";
    Real minValue = 0.0;            // min == max == 0 -> unbounded
    Real maxValue = 0.0;
    Real step = 0.1;                // suggested increment for steppers
    bool logScale = false;          // perceptually logarithmic (focal, f-stop)
    bool color = false;             // a Vec3 that means a color
    bool readOnly = false;          // display, don't edit
    const char* unit = "";          // "mm", "m", ...
    std::vector<const char*> choices;   // string fields: closed option set

    FieldMeta(const char* label) : label(label) {}
    FieldMeta& range(Real lo, Real hi) { minValue = lo; maxValue = hi; return *this; }
    FieldMeta& increment(Real s) { step = s; return *this; }
    FieldMeta& log() { logScale = true; return *this; }
    FieldMeta& asColor() { color = true; return *this; }
    FieldMeta& locked() { readOnly = true; return *this; }
    FieldMeta& units(const char* u) { unit = u; return *this; }
    FieldMeta& options(std::vector<const char*> opts) {
        choices = std::move(opts);
        return *this;
    }
};

// Runtime visitor interface — the type-erasure point that lets a component
// registry hand "visit this entity's component" to UIs without templates
// crossing the seam. References are live into the component: valid for the
// duration of the visit only (sparse-set storage can move on add), so
// visitors either act immediately or re-visit to read/write later.
class PropertyVisitor {
public:
    virtual ~PropertyVisitor() = default;

    virtual void field(const FieldMeta& meta, Real& value) = 0;
    virtual void field(const FieldMeta& meta, float& value) = 0;
    virtual void field(const FieldMeta& meta, Vec3& value) = 0;
    virtual void field(const FieldMeta& meta, bool& value) = 0;
    virtual void field(const FieldMeta& meta, std::string& value) = 0;
    // A boolean view of one bit in a flags word (e.g. material checkerboard).
    virtual void bitFlag(const FieldMeta& meta, uint32_t& flags, uint32_t mask) = 0;
};

// One describe function per component — the single source of truth its
// inspectors and serializers derive from.
void describeProperties(Transform& t, PropertyVisitor& v);
void describeProperties(RenderMaterial& m, PropertyVisitor& v);
void describeProperties(LensParams& lens, PropertyVisitor& v);
void describeProperties(SceneCamera& cam, PropertyVisitor& v);
void describeProperties(SourceSpec& spec, PropertyVisitor& v);

}  // namespace engine

#endif
