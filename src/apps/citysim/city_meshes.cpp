#include "city_meshes.h"

#include "../../engine/mesh_builder.h"

#include <cmath>

namespace citysim {

using engine::Vec3;
using engine::RenderMaterial;
using engine::MeshBuilder;

namespace {

// Append a coloured box (centred at `c`, dimensions `size`) into `out`.
void addBox(engine::RenderMesh& out, Vec3 size, Vec3 c, Vec3 color) {
    engine::RenderMesh b = MeshBuilder::box(size);
    uint32_t base = static_cast<uint32_t>(out.vertices.size());
    for (engine::Vertex v : b.vertices) {
        v.position = v.position + c;
        v.color = color;
        out.vertices.push_back(v);
    }
    for (uint32_t i : b.indices) out.indices.push_back(base + i);
}

// Append a coloured box PITCHED about an X-axis pivot (for limbs: the hip or
// shoulder line). `center` is the box centre before rotation; the whole box
// rotates by `angle` around the line y = pivotY (z = 0 plane of the body).
void addBoxPitched(engine::RenderMesh& out, Vec3 size, Vec3 c, Vec3 color,
                   Real pivotY, Real angle) {
    engine::RenderMesh b = MeshBuilder::box(size);
    Real ca = std::cos(angle), sa = std::sin(angle);
    uint32_t base = static_cast<uint32_t>(out.vertices.size());
    for (engine::Vertex v : b.vertices) {
        Vec3 p = v.position + c;
        Real ry = p.y - pivotY, rz = p.z;
        p.y = pivotY + ry * ca - rz * sa;
        p.z = ry * sa + rz * ca;
        v.position = p;
        Vec3 n = v.normal;
        v.normal = Vec3(n.x, n.y * ca - n.z * sa, n.y * sa + n.z * ca);
        v.color = color;
        out.vertices.push_back(v);
    }
    for (uint32_t i : b.indices) out.indices.push_back(base + i);
}

// Deterministic outfits for the person mesh: shirt / pants / skin triples.
struct PersonOutfit { Vec3 shirt, pants, skin; };
const PersonOutfit kOutfits[] = {
    {{0.62, 0.35, 0.25}, {0.22, 0.24, 0.30}, {0.87, 0.68, 0.55}},
    {{0.30, 0.42, 0.58}, {0.16, 0.16, 0.18}, {0.55, 0.38, 0.28}},
    {{0.38, 0.50, 0.32}, {0.30, 0.26, 0.22}, {0.76, 0.56, 0.42}},
    {{0.55, 0.48, 0.30}, {0.24, 0.28, 0.36}, {0.42, 0.29, 0.22}},
    {{0.45, 0.32, 0.48}, {0.20, 0.20, 0.24}, {0.87, 0.68, 0.55}},
    {{0.60, 0.58, 0.55}, {0.28, 0.22, 0.20}, {0.64, 0.46, 0.34}},
    {{0.72, 0.62, 0.28}, {0.18, 0.22, 0.28}, {0.55, 0.38, 0.28}},
    {{0.35, 0.55, 0.55}, {0.26, 0.24, 0.22}, {0.76, 0.56, 0.42}},
    {{0.66, 0.30, 0.34}, {0.22, 0.26, 0.24}, {0.87, 0.68, 0.55}},
    {{0.42, 0.38, 0.60}, {0.20, 0.18, 0.16}, {0.64, 0.46, 0.34}},
    {{0.52, 0.56, 0.62}, {0.30, 0.30, 0.34}, {0.42, 0.29, 0.22}},
    {{0.58, 0.44, 0.36}, {0.24, 0.20, 0.26}, {0.76, 0.56, 0.42}},
    // LAST SLOT: the PLAYER's outfit (playerOutfit()) — a vivid red jacket over
    // near-black trousers, deliberately absent from the muted walker palette
    // above so the player reads instantly in a crowd. personOutfitCount()
    // excludes it, so it is never dealt to a walker.
    {{0.88, 0.12, 0.10}, {0.10, 0.10, 0.12}, {0.87, 0.68, 0.55}},
};
constexpr int kNumOutfits = static_cast<int>(sizeof(kOutfits) / sizeof(kOutfits[0]));

// Append a flat white quad (two triangles, +Y normal) — the building block of
// the ground-projected debug meshes (strips, wedge edges/arc).
void addQuadXZ(engine::RenderMesh& m, Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    for (const Vec3& p : { a, b, c, d }) {
        engine::Vertex v; v.position = p; v.normal = Vec3(0, 1, 0); v.color = Vec3(1, 1, 1);
        m.vertices.push_back(v);
    }
    m.indices.push_back(base + 0); m.indices.push_back(base + 1); m.indices.push_back(base + 2);
    m.indices.push_back(base + 0); m.indices.push_back(base + 2); m.indices.push_back(base + 3);
}

}  // namespace

int personOutfitCount() { return kNumOutfits - 1; }   // last slot = the player's
int playerOutfit() { return kNumOutfits - 1; }

Real walkPoseSwing(int pose) {
    return kMaxSwing * std::sin(2.0 * engine::PI * pose / kWalkPoses);
}

int walkPoseIndex(Real& stridePhase, Real speed, Real dt) {
    if (speed <= kWalkAnimMinSpeed) return 0;
    stridePhase += speed * dt / kStrideLength;
    stridePhase -= std::floor(stridePhase);
    return static_cast<int>(stridePhase * kWalkPoses) % kWalkPoses;
}

engine::RenderMesh buildPersonMesh(Real swing, int outfit) {
    const PersonOutfit& o = kOutfits[((outfit % kNumOutfits) + kNumOutfits) % kNumOutfits];
    engine::RenderMesh m;
    // Proportions of a 1.8 m body, centred at mid-height (y in [-0.9, 0.9]) so it
    // drops in wherever the old walker box went. Facing +Z; hips at y = -0.05,
    // shoulders at y = 0.50. Legs swing opposed; arms counter-swing at ~70%.
    const Real hipY = -0.05, shoulderY = 0.50;
    addBoxPitched(m, Vec3(0.17, 0.85, 0.22), Vec3(-0.115, -0.475, 0), o.pants,
                  hipY, swing);                                        // left leg
    addBoxPitched(m, Vec3(0.17, 0.85, 0.22), Vec3(0.115, -0.475, 0), o.pants,
                  hipY, -swing);                                       // right leg
    addBox(m, Vec3(0.44, 0.60, 0.26), Vec3(0, 0.25, 0), o.shirt);      // torso
    addBoxPitched(m, Vec3(0.12, 0.62, 0.16), Vec3(-0.29, 0.19, 0), o.shirt,
                  shoulderY, -swing * 0.7);                            // left arm
    addBoxPitched(m, Vec3(0.12, 0.62, 0.16), Vec3(0.29, 0.19, 0), o.shirt,
                  shoulderY, swing * 0.7);                             // right arm
    addBox(m, Vec3(0.26, 0.26, 0.26), Vec3(0, 0.70, 0), o.skin);       // head
    return m;
}

engine::RenderMesh ringXZ(Real innerFrac, int segs) {
    engine::RenderMesh m;
    const Vec3 white(1, 1, 1);
    for (int i = 0; i < segs; ++i) {
        Real t0 = (Real(i) / segs) * 6.28318530718;
        Real t1 = (Real(i + 1) / segs) * 6.28318530718;
        Vec3 o0(std::cos(t0), 0, std::sin(t0)), o1(std::cos(t1), 0, std::sin(t1));
        Vec3 i0 = o0 * innerFrac, i1 = o1 * innerFrac;
        uint32_t b = static_cast<uint32_t>(m.vertices.size());
        for (const Vec3& p : { o0, o1, i1, i0 }) {
            engine::Vertex v; v.position = p; v.normal = Vec3(0, 1, 0); v.color = white;
            m.vertices.push_back(v);
        }
        m.indices.push_back(b + 0); m.indices.push_back(b + 1); m.indices.push_back(b + 2);
        m.indices.push_back(b + 0); m.indices.push_back(b + 2); m.indices.push_back(b + 3);
    }
    return m;
}

engine::RenderMesh arrowXZ() {
    engine::RenderMesh m;
    const Vec3 white(1, 1, 1);
    auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (const Vec3& p : { a, b, c, d }) {
            engine::Vertex v; v.position = p; v.normal = Vec3(0, 1, 0); v.color = white;
            m.vertices.push_back(v);
        }
        m.indices.push_back(base + 0); m.indices.push_back(base + 1); m.indices.push_back(base + 2);
        m.indices.push_back(base + 0); m.indices.push_back(base + 2); m.indices.push_back(base + 3);
    };
    quad(Vec3(-0.5, 0, 0), Vec3(0.5, 0, 0), Vec3(0.5, 0, 0.7), Vec3(-0.5, 0, 0.7));   // shaft
    // Arrow head (triangle) from z=0.6 to z=1.0.
    uint32_t b = static_cast<uint32_t>(m.vertices.size());
    for (const Vec3& p : { Vec3(-1.1, 0, 0.6), Vec3(1.1, 0, 0.6), Vec3(0, 0, 1.0) }) {
        engine::Vertex v; v.position = p; v.normal = Vec3(0, 1, 0); v.color = white;
        m.vertices.push_back(v);
    }
    m.indices.push_back(b + 0); m.indices.push_back(b + 1); m.indices.push_back(b + 2);
    return m;
}

engine::RenderMesh stripXZ() {
    engine::RenderMesh m;
    addQuadXZ(m, Vec3(-0.5, 0, 0), Vec3(0.5, 0, 0), Vec3(0.5, 0, 1), Vec3(-0.5, 0, 1));
    return m;
}

engine::RenderMesh wedgeXZ(Real halfAngleRad, int segs) {
    engine::RenderMesh m;
    const Real band = 0.04;   // thin outline band, like ringXZ's — reads as paint
    // The two straight edges, origin to rim, at +/- the half-angle off +Z.
    for (Real side : { Real(1), Real(-1) }) {
        Real a = side * halfAngleRad;
        Vec3 dir(std::sin(a), 0, std::cos(a));
        Vec3 off = Vec3(dir.z, 0, -dir.x) * (band * 0.5);   // in-plane perpendicular
        addQuadXZ(m, Vec3(0, 0, 0) - off, off, dir + off, dir - off);
    }
    // The arc between them: a ring band clipped to the wedge's angular span.
    for (int i = 0; i < segs; ++i) {
        Real t0 = -halfAngleRad + (2 * halfAngleRad) * i / segs;
        Real t1 = -halfAngleRad + (2 * halfAngleRad) * (i + 1) / segs;
        Vec3 o0(std::sin(t0), 0, std::cos(t0)), o1(std::sin(t1), 0, std::cos(t1));
        addQuadXZ(m, o0, o1, o1 * (1 - band), o0 * (1 - band));
    }
    return m;
}

RenderMaterial carMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);            // hue carried in the car mesh's vertex colour
    m.metallic = 0.5f;
    m.roughness = 0.4f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

RenderMaterial pedMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);   // hue carried in the person mesh's vertex colours
    m.metallic = 0.0f;
    m.roughness = 0.85f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

// An emissive lens: dark body, strong self-illumination in the signal colour so
// the active phase glows even before any scene lighting.
RenderMaterial signalMaterial(SignalState s) {
    RenderMaterial m;
    m.metallic = 0.0f;
    m.roughness = 0.4f;
    m.opacity = 1.0f;
    m.flags = 0;
    // Emission raised with the street lamps (head glow went 1.6 -> 4.5): these
    // lenses were tuned against an unlit city, and once the lamps read as real
    // fixtures a 1.6 signal beside a 4.5 lamp stopped carrying at night. A
    // signal must be the most legible light on the street, not the dimmest.
    switch (s) {
        case SignalState::Green:  m.albedo = Vec3(0.0, 0.2, 0.0); m.emission = Vec3(0.2, 4.0, 0.5); break;
        case SignalState::Yellow: m.albedo = Vec3(0.2, 0.18, 0.0); m.emission = Vec3(4.0, 3.2, 0.2); break;
        case SignalState::Red:    m.albedo = Vec3(0.2, 0.0, 0.0); m.emission = Vec3(4.0, 0.2, 0.2); break;
    }
    return m;
}

// The static signal assembly (pole/arm/head housing) carries its hue in vertex
// colour, like the rest of the city's street furniture.
RenderMaterial signalPostMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);
    m.metallic = 0.0f;
    m.roughness = 0.5f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

// Debug colour for a behaviour state — glowing ground paint drawn like road
// markings (regular depth), see the widget notes below.
RenderMaterial widgetMaterial(Vec3 color) {
    RenderMaterial m;
    // Bright glowing paint: strong emission (pops through bloom/tonemap), dark
    // base so lighting can't wash it toward a grey "shadow ring".
    // Emission tuned DOWN from 4x (device round 3): at 4x the bloom/tonemap
    // saturated every ring toward white — a green ring read as "white looking".
    // 1.7x still glows through daylight but keeps its hue legible.
    // FLAG_OVERLAY (device feedback): debug widgets/lines draw on TOP of world
    // geometry (the road solid was burying the block/lot outlines) and are
    // skipped by every backend's shadow pass, so gizmos cast no shadows.
    m.albedo = color * 0.15; m.metallic = 0.0f; m.roughness = 1.0f; m.opacity = 1.0f;
    m.emission = color * 1.7; m.flags = RenderMaterial::FLAG_OVERLAY;
    return m;
}

// A car lamp lens: dark body so the housing reads when unlit, strong emission in
// the lamp colour so it glows through daylight and bloom — the same recipe as
// signalMaterial (FLAG none, regular depth), just parameterised by colour.
RenderMaterial lampMaterial(Vec3 emission) {
    RenderMaterial m;
    m.albedo = emission * 0.12f;   // dark lens; the glow carries the hue
    m.metallic = 0.0f;
    m.roughness = 0.4f;
    m.opacity = 1.0f;
    m.emission = emission;
    m.flags = 0;
    return m;
}

Vec3 stateColor(Agent::State s) {
    switch (s) {
        // Pedestrian / shared
        case Agent::State::Walking:   return Vec3(0.15, 0.90, 0.25);   // green: go
        case Agent::State::Avoiding:  return Vec3(1.00, 0.55, 0.08);   // amber: stepping around
        case Agent::State::Waiting:   return Vec3(1.00, 0.10, 0.08);   // RED: held (signal/tether)
        // Driver FSM
        case Agent::State::Cruising:  return Vec3(0.15, 0.90, 0.25);   // green: go
        case Agent::State::Following: return Vec3(0.10, 0.65, 0.70);   // teal: pacing a leader
        case Agent::State::Yielding:  return Vec3(1.00, 0.55, 0.08);   // amber: braking for someone
        case Agent::State::Turning:   return Vec3(0.70, 0.35, 0.95);   // violet: arcing a node
        default:                      return Vec3(0.45, 0.45, 0.48);   // grey: resting/parked
    }
}

}  // namespace citysim
