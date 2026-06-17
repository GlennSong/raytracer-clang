#ifndef RAYTRACER_ENGINE_SKELETON_H
#define RAYTRACER_ENGINE_SKELETON_H

#include "../../rt_math.h"   // Vec3
#include "lsystem.h"         // ModuleString
#include <cstdint>
#include <random>
#include <vector>

namespace engine {

// A branching skeleton — the agnostic output of interpreting an L-system module
// string with a turtle (ADR-0021 value type). Trees skin it as cylinders,
// mountain ranges use it as ridge axes, vegetation/roots/rivers consume it too;
// the skeleton itself carries no domain meaning, just the branching structure.
struct SkeletonNode {
    Vec3  pos;
    Vec3  heading{0, 1, 0};   // turtle forward at this node
    int   parent = -1;        // -1 = root
    int   childCount = 0;
    int   depth = 0;          // branch order (bracket nesting; 0 = main axis)
    float radius = 0.0f;      // filled by a consumer (e.g. the pipe model)
    float distFromRoot = 0.0f;
    bool  isTip = false;      // an unexpanded apex: leaf attach / ridge end / ...
};

struct Skeleton {
    std::vector<SkeletonNode> nodes;
    // Child index lists per node (parent-before-child order).
    std::vector<std::vector<int>> childLists() const;
};

// Interpret an expanded module string with a 3D turtle into a skeleton. Standard
// turtle alphabet: F = advance one segment (param = length), A/L = tip,
// + - = yaw (about Z), & ^ = pitch (about X), / \\ = roll (about Y),
// [ ] = push/pop a branch (depth + 1). `angleJitter` (degrees) perturbs every
// turn from the rng for natural variation.
Skeleton buildSkeleton(const ModuleString& modules, float angleJitter,
                       std::mt19937& rng);
Skeleton buildSkeleton(const ModuleString& modules, float angleJitter = 0.0f,
                       uint32_t seed = 0);

}  // namespace engine

#endif
