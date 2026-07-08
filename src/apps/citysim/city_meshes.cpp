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

// The car fleet's PAINT, one colour per fleet slot (mirrors city_sim kFleet slot
// for slot; body style + size come from the fleet body). Each slot is its own
// instance group (a group shares one mesh + material). Drivers spread across them
// by vehicle index, so a given car keeps its shape, size, and colour.
const Vec3 kCarColors[] = {
    Vec3(0.72, 0.10, 0.10), Vec3(0.10, 0.18, 0.52), Vec3(0.90, 0.90, 0.90),  // sedans
    Vec3(0.85, 0.78, 0.10), Vec3(0.10, 0.45, 0.30), Vec3(0.80, 0.40, 0.08),  // hatchbacks
    Vec3(0.09, 0.09, 0.11), Vec3(0.52, 0.53, 0.56), Vec3(0.30, 0.22, 0.14),  // SUVs
    Vec3(0.14, 0.30, 0.20),                                                  // pickup
    Vec3(0.62, 0.60, 0.42),                                                  // van (tan)
    Vec3(0.20, 0.42, 0.55),                                                  // box truck (teal)
};
constexpr int kNumCarVariants = static_cast<int>(sizeof(kCarColors) / sizeof(kCarColors[0]));

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

engine::RenderMesh buildCarMesh(int style, Vec3 color, Vec3 size, bool withWheels) {
    const Real w = size.x, h = size.y, l = size.z;
    const Real hw = w * 0.5, hl = l * 0.5;
    const Vec3 glass(0.05, 0.06, 0.09);    // dark glass (matches car_body)
    const Vec3 tyre(0.04, 0.04, 0.05);
    const Vec3 head(1.0, 0.97, 0.82), tail(0.85, 0.06, 0.05);
    engine::RenderMesh m;

    switch (style) {
        case 1:  // hatchback: hull + cabin carried back to the tail, glass fore & aft
            addBox(m, Vec3(w, h * 0.46, l * 0.92), Vec3(0, 0, 0), color);
            addBox(m, Vec3(w * 0.86, h * 0.44, l * 0.56), Vec3(0, h * 0.40, -l * 0.06), color);
            addBox(m, Vec3(w * 0.80, h * 0.30, l * 0.05), Vec3(0, h * 0.44, l * 0.12), glass);
            addBox(m, Vec3(w * 0.80, h * 0.30, l * 0.05), Vec3(0, h * 0.44, -l * 0.34), glass);
            break;
        case 2:  // SUV: tall hull, big greenhouse
            addBox(m, Vec3(w, h * 0.58, l), Vec3(0, 0, 0), color);
            addBox(m, Vec3(w * 0.90, h * 0.46, l * 0.62), Vec3(0, h * 0.44, -l * 0.02), color);
            addBox(m, Vec3(w * 0.84, h * 0.34, l * 0.05), Vec3(0, h * 0.46, l * 0.20), glass);
            addBox(m, Vec3(w * 0.84, h * 0.34, l * 0.05), Vec3(0, h * 0.46, -l * 0.26), glass);
            break;
        case 3:  // pickup: forward cab + open bed
            addBox(m, Vec3(w, h * 0.42, l), Vec3(0, -h * 0.04, 0), color);
            addBox(m, Vec3(w * 0.90, h * 0.40, l * 0.34), Vec3(0, h * 0.34, l * 0.22), color);
            addBox(m, Vec3(w * 0.80, h * 0.26, l * 0.05), Vec3(0, h * 0.40, l * 0.38), glass);
            addBox(m, Vec3(w * 0.92, h * 0.20, l * 0.42), Vec3(0, h * 0.10, -l * 0.26), color); // bed walls
            break;
        case 4:  // van: one tall slab body, raked windshield, low nose
            addBox(m, Vec3(w, h * 0.72, l * 0.86), Vec3(0, h * 0.06, -l * 0.06), color);
            addBox(m, Vec3(w, h * 0.34, l * 0.20), Vec3(0, -h * 0.10, l * 0.40), color);       // nose
            addBox(m, Vec3(w * 0.86, h * 0.30, l * 0.05), Vec3(0, h * 0.22, l * 0.30), glass);  // windshield
            addBox(m, Vec3(w * 0.86, h * 0.24, l * 0.05), Vec3(0, h * 0.24, -l * 0.48), glass); // rear glass
            break;
        case 5:  // box truck: a small cab up front + a tall square cargo box
            addBox(m, Vec3(w, h * 0.44, l * 0.30), Vec3(0, -h * 0.04, l * 0.33), color);        // cab lower
            addBox(m, Vec3(w * 0.94, h * 0.40, l * 0.24), Vec3(0, h * 0.30, l * 0.35), color);  // cab roof
            addBox(m, Vec3(w * 0.84, h * 0.30, l * 0.05), Vec3(0, h * 0.30, l * 0.47), glass);  // windshield
            addBox(m, Vec3(w, h * 0.86, l * 0.62), Vec3(0, h * 0.12, -l * 0.17), color);        // cargo box
            break;
        default: // sedan — matches the player's car_body proportions exactly
            addBox(m, Vec3(w, h * 0.46, l), Vec3(0, 0, 0), color);
            addBox(m, Vec3(w * 0.84, h * 0.42, l * 0.46), Vec3(0, h * 0.40, -l * 0.04), color);
            addBox(m, Vec3(w * 0.80, h * 0.30, l * 0.05), Vec3(0, h * 0.42, l * 0.15), glass);
            addBox(m, Vec3(w * 0.80, h * 0.26, l * 0.05), Vec3(0, h * 0.42, -l * 0.23), glass);
            break;
    }

    // Head/taillights at the corners (front +Z pale, rear -Z red) — like car_body.
    Real ly = -h * 0.08, lx = hw - 0.30;
    for (Real sx : { Real(1), Real(-1) }) {
        addBox(m, Vec3(0.34, 0.18, 0.10), Vec3(sx * lx, ly, hl - 0.05), head);
        addBox(m, Vec3(0.34, 0.18, 0.10), Vec3(sx * lx, ly, -hl + 0.05), tail);
    }

    // Four wheels (dark discs, thin along X), sat at the hull's lower edge —
    // skipped for a promoted physical car (its wheels are physics entities).
    if (withWheels) {
        Real wr = h * 0.26, axleY = -h * 0.5 + wr, fz = l * 0.32, wxo = hw - 0.03;
        for (Real wx : { wxo, -wxo })
            for (Real wz : { fz, -fz })
                addBox(m, Vec3(0.12, wr * 2, wr * 2), Vec3(wx, axleY, wz), tyre);
    } else {
        (void)tyre;
    }
    return m;
}

int styleForType(VehicleType t) {
    switch (t) {
        case VehicleType::Hatchback: return 1;
        case VehicleType::SUV:       return 2;
        case VehicleType::Pickup:    return 3;
        case VehicleType::Van:       return 4;
        case VehicleType::BoxTruck:  return 5;
        default:                     return 0;   // Sedan
    }
}

Vec3 fleetBodySize(int slot) {
    const VehicleBody& b = vehicleFleetBody(slot);
    return Vec3(b.width, b.height, b.length);
}

int carVariantCount() { return kNumCarVariants; }

engine::RenderMesh fleetCarMesh(int slot, bool withWheels) {
    int n = vehicleFleetSize();
    int s = ((slot % n) + n) % n;
    const VehicleBody& b = vehicleFleetBody(s);
    return buildCarMesh(styleForType(b.type), kCarColors[s % kNumCarVariants],
                        Vec3(b.width, b.height, b.length), withWheels);
}

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
    switch (s) {
        case SignalState::Green:  m.albedo = Vec3(0.0, 0.2, 0.0); m.emission = Vec3(0.1, 1.6, 0.2); break;
        case SignalState::Yellow: m.albedo = Vec3(0.2, 0.18, 0.0); m.emission = Vec3(1.6, 1.3, 0.1); break;
        case SignalState::Red:    m.albedo = Vec3(0.2, 0.0, 0.0); m.emission = Vec3(1.6, 0.1, 0.1); break;
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
