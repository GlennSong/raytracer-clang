#ifndef RAYTRACER_ENGINE_SCRIPTING_STREET_PROPS_SPEC_H
#define RAYTRACER_ENGINE_SCRIPTING_STREET_PROPS_SPEC_H

// THE KERB PROPS THAT ARE DATA (assets/scripts/street_props.lua, ADR-0354), read for the level loader: the kinds the
// street planner places beside its own (their mesh, spacing, collider, breaking speed and spray), and the collider
// sizes of what the planner already places (its bins, racks and news boxes, the street trees' trunks, the lamp posts).

#include "../../rt_math.h"
#include "../../renderer/renderer.h"   // RenderMesh

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace engine {

struct StreetPropCollider {
    Vec3 half{0, 0, 0};       // a box standing on the pavement (half extents)
    Real breaksAt = 0;        // m/s a car knocks it loose at (0: it stops the car)
    Real mass = 30;           // kg, once loose
};

struct StreetPropKind {
    std::string name;
    Real every = 90, verge = 0.55, junction = 6;
    bool mainOnly = false;
    std::shared_ptr<RenderMesh> mesh;   // at the origin, standing on y = 0
    StreetPropCollider collider;
    std::string sprayEffect;            // empty: none
    Real sprayRate = 0, sprayUp = 0;
};

struct StreetPropsSpec {
    std::vector<StreetPropKind> kinds;
    std::map<std::string, StreetPropCollider> legacy;   // "bin", "bike_rack", "news_boxes"
    Real trunkRadius = 0, trunkHeight = 0;              // a street tree's (0: no collider)
    Real lampRadius = 0, lampHeight = 0;                // a lamp post's
    // how a piece knocked loose leaves (ADR-0355): this share of the knock's speed on along it, and up this much plus
    // a bit per m/s of the knock
    Real knockPush = 0.6, knockUp = 1.0, knockUpPerSpeed = 0.05;
};

// Run street_props.lua (resolved like any level script). False, with `err`, when it is missing or fails.
bool loadStreetProps(const std::string& levelDir, StreetPropsSpec& out, std::string* err = nullptr);

}  // namespace engine

#endif
