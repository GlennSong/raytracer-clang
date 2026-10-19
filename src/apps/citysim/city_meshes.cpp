#include "city_meshes.h"
#include "../../engine/procgen/vehicle/occupant.h"   // the seated manikin (M5)

#include "../../engine/mesh_builder.h"
#include "../../engine/procgen/person.h"

#include <cmath>

namespace citysim {

using engine::Vec3;
using engine::RenderMaterial;
using engine::MeshBuilder;

namespace {

// The outfits and the standing body are the engine's (procgen/person.h): the walkers wear them, and so can anything
// else that stands a person (the scale lab).
using engine::PersonOutfit;
const int kNumOutfits = engine::personOutfitTotal();

// Append a flat white quad (two triangles, +Y normal) — the building block of
// the ground-projected debug meshes (strips, wedge edges/arc).
// A flat quad on the ground, facing up whichever way round its corners come (the wedge's arc runs them the other
// way): MeshBuilder::emitQuad winds it.
void addQuadXZ(engine::RenderMesh& m, Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
    engine::MeshBuilder::emitQuad(m, a, b, c, d, Vec3(0, 1, 0), Vec3(1, 1, 1));
}

}  // namespace

int personOutfitCount() { return kNumOutfits - 1; }   // last slot = the player's

engine::RenderMesh buildSeatedPersonMesh(int outfit) {
    const PersonOutfit& o = engine::personOutfit(outfit);
    engine::OccupantParams op;
    op.hipToHeel = 0.45;       // a bench or a chair
    op.sittingHeight = 0.86;
    op.backAngle = 8.0;        // sitting up
    op.armReach = 20.0;        // hands in the lap
    op.shirt = o.shirt;
    op.trousers = o.pants;
    op.skin = o.skin;
    return engine::buildSeatedOccupant(op);
}
// SITTING ON THE GRASS (a picnic): the seated body with its hip a hand off the ground, knees drawn up, feet flat in
// front, leaning back a touch, hands round the knees. Origin at the hip, like the bench sitter's.
engine::RenderMesh buildGroundSittingPersonMesh(int outfit) {
    const PersonOutfit& o = engine::personOutfit(outfit);
    engine::OccupantParams op;
    op.hipToHeel = kGroundSitHip - 0.02;
    op.thighRise = 38.0;
    op.sittingHeight = 0.86;
    op.backAngle = 14.0;
    op.armReach = 42.0;
    op.shirt = o.shirt;
    op.trousers = o.pants;
    op.skin = o.skin;
    return engine::buildSeatedOccupant(op);
}
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

engine::RenderMesh buildPersonMesh(Real swing, int outfit) { return engine::buildStandingPerson(swing, outfit); }

engine::RenderMesh ringXZ(Real innerFrac, int segs) {
    engine::RenderMesh m;
    const Vec3 white(1, 1, 1);
    for (int i = 0; i < segs; ++i) {
        Real t0 = (Real(i) / segs) * 6.28318530718;
        Real t1 = (Real(i + 1) / segs) * 6.28318530718;
        Vec3 o0(std::cos(t0), 0, std::sin(t0)), o1(std::cos(t1), 0, std::sin(t1));
        Vec3 i0 = o0 * innerFrac, i1 = o1 * innerFrac;
        addQuadXZ(m, o0, o1, i1, i0);
    }
    return m;
}

engine::RenderMesh arrowXZ() {
    engine::RenderMesh m;
    const Vec3 white(1, 1, 1);
    addQuadXZ(m, Vec3(-0.5, 0, 0), Vec3(0.5, 0, 0), Vec3(0.5, 0, 0.7), Vec3(-0.5, 0, 0.7));   // shaft
    // Arrow head (triangle) from z=0.6 to z=1.0.
    MeshBuilder::emitTri(m, Vec3(-1.1, 0, 0.6), Vec3(1.1, 0, 0.6), Vec3(0, 0, 1.0), Vec3(0, 1, 0), white);
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
