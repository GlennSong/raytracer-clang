#include "bus_stop_props.h"

#include <algorithm>
#include <unordered_map>

#include "../../engine/components.h"

#include <cmath>

namespace citysim {

using engine::AssetManager;
using engine::Entity;
using engine::MeshHandle;
using engine::NavGraph;
using engine::NavLink;
using engine::Real;
using engine::RenderMaterial;
using engine::Renderable;
using engine::Transform;
using engine::Vec2;
using engine::Vec3;
using engine::World;

// One colour per route, so the sign answers "which route is this" and not just
// "there is a bus stop here". Distinguishable rather than pretty, and it wraps
// for a network with more routes than colours.
Vec3 routeColour(int route) {
    static const Vec3 kPalette[8] = {
        {0.85, 0.18, 0.16},   // red
        {0.13, 0.42, 0.80},   // blue
        {0.95, 0.68, 0.10},   // amber
        {0.16, 0.60, 0.32},   // green
        {0.60, 0.24, 0.72},   // violet
        {0.05, 0.62, 0.66},   // teal
        {0.92, 0.42, 0.12},   // orange
        {0.45, 0.45, 0.48},   // grey
    };
    return kPalette[static_cast<std::size_t>(((route % 8) + 8) % 8)];
}

const char* routeColourName(int route) {
    static const char* kNames[8] = {"red", "blue", "amber", "green",
                                    "violet", "teal", "orange", "grey"};
    return kNames[static_cast<std::size_t>(((route % 8) + 8) % 8)];
}

namespace {

// The plaza benches in city_lots use exactly these dimensions; a bus stop bench
// that did not match them would read as a different kind of object.
// A bench somebody could actually sit on (Glenn: "that tiny ass bench is the
// bus bench?"). The plaza bench is a 1.6 m backless plank, which is fine in the
// middle of a plaza and mean as the only seat at a bus stop -- so this one is
// longer, deeper and has a BACK, which is most of what makes a bench read as
// furniture rather than a kerb.
constexpr Real kSeatL = 2.4, kSeatT = 0.09, kSeatW = 0.58, kSeatY = 0.44;
constexpr Real kBackH = 0.52, kBackT = 0.07;
constexpr Real kPoleH = 2.6, kPoleR = 0.05;
constexpr Real kSignW = 0.88, kSignH = 0.62, kSignT = 0.05, kSignY = 2.30;
constexpr Real kBandH = 0.34;   // one route's plate on a shared pole


Vec2 rightOf(Vec2 d) { return {d.y, -d.x}; }

Entity box(World& world, AssetManager& assets, Vec3 centre, Vec3 size,
           engine::Quat rot, Vec3 albedo, Real metallic, Real roughness,
           std::vector<Entity>* out) {
    MeshHandle mh = assets.acquirePrimitive("box", size);
    Entity e = world.create();
    Transform t;
    t.position = centre;
    t.orientation = rot;
    world.add<Transform>(e, t);
    world.add<engine::PrevTransform>(e, engine::PrevTransform{t});
    Renderable r;
    r.mesh = mh;
    RenderMaterial m;
    m.albedo = albedo;
    m.metallic = static_cast<float>(metallic);
    m.roughness = static_cast<float>(roughness);
    r.material = m;
    r.drawClass = engine::DrawClass::Structure;
    // Street furniture is small: past this it is a couple of pixels, and there
    // are one of these per stop times four routes.
    r.drawDistance = 260.0;
    world.add<Renderable>(e, r);
    if (out) out->push_back(e);
    return e;
}

}  // namespace

int buildBusStopProps(
    World& world, AssetManager& assets, const BusNetwork& net,
    const NavGraph& nav,
    const std::function<Real(Real, Real)>& groundAt,
    std::vector<Entity>* out, std::vector<Vec3>* outPositions) {
    if (net.empty() || nav.nodeCount() == 0 || !groundAt) return 0;
    int built = 0;

    // ONE STOP PER CORNER. Routes that share a node share its furniture: this
    // used to build a pole, sign and bench per ROUTE, all at the identical
    // spot, so a hub served by three routes was three benches inside each
    // other and one visible sign -- whichever colour won the z-fight. A real
    // shared stop is one pole carrying every route's plate.
    std::vector<int> nodeOrder;
    std::unordered_map<int, std::vector<int>> routesAt;
    for (int r = 0; r < net.routeCount(); ++r)
        for (const BusStop& stop : net.route(r).stops) {
            std::vector<int>& at = routesAt[stop.node];
            if (at.empty()) nodeOrder.push_back(stop.node);
            if (std::find(at.begin(), at.end(), r) == at.end()) at.push_back(r);
        }

    for (int node : nodeOrder) {
        const std::vector<int>& calling = routesAt[node];
        {
            const BusStop stop{node, node >= 0 && node < nav.nodeCount()
                                         ? nav.nodes[static_cast<std::size_t>(node)]
                                         : Vec2(0, 0)};
            if (stop.node < 0 || stop.node >= nav.nodeCount()) continue;
            const std::vector<int>& outs =
                nav.outLinks[static_cast<std::size_t>(stop.node)];

            // A STREET to stand beside: the LONGEST ordinary one leaving the
            // node. Freeway and ramp links have no kerb, and furniture in a
            // fast lane is worse than an unmarked stop, so a stop with no
            // kerbed street at all gets nothing.
            //
            // This first demanded 12 m and took the FIRST match, which
            // furnished 6 stops of 56 -- in a dense grid most
            // junction-to-junction links are shorter than that, and a short
            // street is no reason to have no bus stop.
            const NavLink* use = nullptr;
            for (int li : outs) {
                const NavLink& l = nav.links[static_cast<std::size_t>(li)];
                if (l.klass == engine::RoadClass::Freeway ||
                    l.klass == engine::RoadClass::Ramp)
                    continue;
                if (!use || l.length > use->length) use = &l;
            }
            if (!use) continue;

            const Vec2 a = nav.nodes[static_cast<std::size_t>(use->from)];
            const Vec2 b = nav.nodes[static_cast<std::size_t>(use->to)];
            Vec2 dir = b - a;
            const Real len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (len < 1e-3) continue;
            dir = Vec2(dir.x / len, dir.y / len);
            const Vec2 right = rightOf(dir);

            // BACK OFF from the node -- a nav node is usually an intersection
            // and a bench in the middle of a junction is worse than no bench --
            // then step out past the carriageway onto the pavement. The back-off
            // SCALES with the link: a fixed 7 m would put the bench in the next
            // junction on a 6 m street, which is what dropping the length filter
            // would otherwise have caused.
            Real along = use->length * 0.35;
            if (along > 7.0) along = 7.0;
            if (along < 1.2) along = 1.2;
            const Real offset = use->width * 0.5 + 1.5;
            const Vec2 p(a.x + dir.x * along + right.x * offset,
                         a.y + dir.y * along + right.y * offset);
            const Real gy = groundAt(p.x, p.y);

            // Everything faces the road, i.e. along -right. The facing
            // convention here is yaw = atan2(dir.x, dir.z), as in elevator_system.
            const engine::Quat rot = engine::Quat::fromAxisAngle(
                Vec3(0, 1, 0), std::atan2(-right.x, -right.y));

            // The pole, set slightly toward the kerb of the bench.
            const Vec2 polePos(p.x + right.x * 0.9, p.y + right.y * 0.9);
            box(world, assets, Vec3(polePos.x, gy + kPoleH * 0.5, polePos.y),
                Vec3(kPoleR * 2, kPoleH, kPoleR * 2), rot,
                Vec3(0.22, 0.23, 0.25), 0.6, 0.4, out);
            // The sign: the route's colour, and the thing that answers WHICH
            // route stops here.
            //
            // IT FACES ALONG THE STREET, not across it. Turned to face the
            // carriageway (as it first was) the plate is edge-on to anyone
            // coming down the pavement or the road -- 5 cm of it -- so it read
            // fine from 6 m and vanished by 22 m. A real bus flag is mounted
            // this way round for exactly that reason: you approach a stop ALONG
            // the street, and that is the direction it has to be legible from.
            const engine::Quat signRot = engine::Quat::fromAxisAngle(
                Vec3(0, 1, 0), std::atan2(dir.x, dir.y));
            // One plate per route calling here, stacked down from the top of
            // the sign: a lone route keeps the full plate, a hub gets a band
            // for each, so the colours on the pole ARE the list of routes.
            const int bands = static_cast<int>(calling.size());
            const Real bandH = bands == 1 ? kSignH : kBandH;
            const Real top = kSignY + kSignH * 0.5;
            for (int k = 0; k < bands; ++k)
                box(world, assets,
                    Vec3(polePos.x, gy + top - bandH * (k + 0.5), polePos.y),
                    Vec3(kSignW, bandH - (bands == 1 ? 0.0 : 0.02), kSignT), signRot,
                    routeColour(calling[static_cast<std::size_t>(k)]), 0.1, 0.45, out);

            // The bench, matching the plaza benches: a wooden seat on two metal
            // legs, facing the street.
            const Vec3 wood(0.45, 0.34, 0.22), steel(0.20, 0.21, 0.22);
            const Vec3 seatC(p.x, gy + kSeatY, p.y);
            box(world, assets, seatC, Vec3(kSeatL, kSeatT, kSeatW), rot, wood,
                0.0, 0.75, out);
            // The BACK, set at the kerb-side edge so you sit facing the road.
            const Vec2 backP(p.x + right.x * 0.24, p.y + right.y * 0.24);
            box(world, assets,
                Vec3(backP.x, gy + kSeatY + kBackH * 0.5, backP.y),
                Vec3(kSeatL, kBackH, kBackT), rot, wood, 0.0, 0.75, out);
            // Two end frames, out at the ends of a longer seat.
            for (int lg = 0; lg < 2; ++lg) {
                const Real off = lg ? 1.02 : -1.02;
                const Vec2 legP(p.x + dir.x * off, p.y + dir.y * off);
                box(world, assets, Vec3(legP.x, gy + kSeatY * 0.5, legP.y),
                    Vec3(0.10, kSeatY, kSeatW * 0.9), rot, steel, 0.7, 0.42, out);
            }
            if (outPositions) outPositions->push_back(Vec3(p.x, gy, p.y));
            built += bands;   // route-stops served, comparable to the network's count
        }
    }
    return built;
}

}  // namespace citysim
