#include "city_planner.h"

#include "asset_manager.h"
#include "components.h"
#include "mesh_builder.h"
#include "systems/camera_system.h"
#include "procgen/city/corridor_plan.h"   // rebakeNetCorridors
#include "procgen/city/road_net.h"
#include "../log.h"

#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <memory>

namespace engine {

namespace {

using nlohmann::json;

// The NATURAL terrain sampler for the planner's regen + overlays (roads no
// longer store one, unification step 3): derived from the level's CDLOD config
// with the road carve stripped (flatten = baseFlatten), the same "natural
// ground" the editor's conform pass measures against. Null when the level has
// no CDLOD terrain — the schematic overlays then sit at y=0, as before.
RoadGroundFn worldNaturalGround(World* world) {
    RoadGroundFn fn;
    if (!world) return fn;
    world->each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) {
        if (fn) return;
        auto tp = std::make_shared<TerrainParams>(c.params);
        tp->flatten = c.baseFlatten;
        rebuildFlattenIndex(*tp);
        auto noise = std::make_shared<Noise>(c.seed);
        fn = [tp, noise](double x, double z) {
            return terrainHeight(*tp, *noise, x, z);
        };
    });
    return fn;
}

// District-kind palette, matched to tools/diagnostics/road_map_svg.cpp
// (kDistrictTint): Financial / Commercial / Residential / Old Town /
// Industrial. sRGB hex / 255 — schematic overlay colour, not lit material.
const Vec3 kHubTint[5] = {
    {0.561, 0.659, 0.784},   // #8fa8c8 financial
    {0.784, 0.706, 0.561},   // #c8b48f commercial
    {0.624, 0.757, 0.604},   // #9fc19a residential
    {0.784, 0.624, 0.604},   // #c89f9a old town
    {0.690, 0.651, 0.769},   // #b0a6c4 industrial
};

// Road-class colours (road_map_svg): Arterial dark, Collector mid, Local light.
Vec3 classColor(RoadClass k) {
    switch (k) {
        case RoadClass::Freeway:
        case RoadClass::Arterial:  return {0.239, 0.275, 0.329};   // #3d4654
        case RoadClass::Collector: return {0.353, 0.392, 0.447};   // #5a6472
        default:                   return {0.545, 0.576, 0.631};   // #8b93a1
    }
}

double classHalfWidth(RoadClass k) {
    switch (k) {
        case RoadClass::Freeway:
        case RoadClass::Arterial:  return 4.5;
        case RoadClass::Collector: return 3.0;
        default:                   return 1.6;
    }
}

// A closed regular polygon around `c` — the circle both the disc fill and the
// ring outline are stroked from.
std::vector<Vec2> circlePts(const Vec2& c, double r, int n) {
    std::vector<Vec2> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double a = 2.0 * 3.14159265358979323846 * i / n;
        pts.push_back(Vec2(c.x + r * std::cos(a), c.y + r * std::sin(a)));
    }
    return pts;
}

// Filled disc: a closed circle stroked at half its radius with halfW = half
// its radius covers 0..r without a dedicated disc primitive.
void appendDisc(RenderMesh& into, const Vec2& c, double r, double y,
                const Vec3& color) {
    MeshBuilder::append(
        into, strokeRibbon(circlePts(c, r * 0.5, 24), {r * 0.5}, y, color,
                           /*closed=*/true));
}

void appendRing(RenderMesh& into, const Vec2& c, double r, double halfW,
                double y, const Vec3& color) {
    MeshBuilder::append(
        into, strokeRibbon(circlePts(c, r, 20), {halfW}, y, color,
                           /*closed=*/true));
}

// Junction-to-junction span count: walk chains through degree-2 curve nodes
// (the metric the metro span audits use — tests/test_metro_sites.cpp).
int junctionSpanCount(const RoadEntity& net) {
    const RoadGraph& g = net.graph;
    const int n = static_cast<int>(g.nodes.size());
    std::vector<int> deg(n, 0);
    std::vector<std::vector<std::pair<int, int>>> nbr(n);
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        int a = g.edges[ei].a, b = g.edges[ei].b;
        if (a < 0 || b < 0 || a >= n || b >= n) continue;
        ++deg[a];
        ++deg[b];
        nbr[a].push_back({b, ei});
        nbr[b].push_back({a, ei});
    }
    std::vector<char> walked(g.edges.size(), 0);
    int spans = 0;
    for (int start = 0; start < n; ++start) {
        if (deg[start] == 2) continue;   // chain interiors don't start spans
        for (auto [next, ei0] : nbr[start]) {
            if (walked[ei0]) continue;
            walked[ei0] = 1;
            int prev = start, cur = next;
            while (deg[cur] == 2) {
                auto [a0, ea] = nbr[cur][0];
                auto [a1, eb] = nbr[cur][1];
                int nn = (a0 == prev) ? a1 : a0;
                int ne = (a0 == prev) ? eb : ea;
                if (walked[ne]) break;
                walked[ne] = 1;
                prev = cur;
                cur = nn;
            }
            ++spans;
        }
    }
    return spans;
}

}  // namespace

void CityPlanner::attach(World* world, AssetManager* assets,
                         Renderer* renderer, CameraSystem* cameras) {
    world_ = world;
    assets_ = assets;
    renderer_ = renderer;
    cameras_ = cameras;
    // Fresh world: any prior overlay entities/meshes died with the old one
    // (assets.clear() on state teardown). Visibility flags survive.
    footprint_.group = hubs_.group = arterials_.group = nodes_.group = Entity{};
    footprint_.mesh = hubs_.mesh = arterials_.mesh = nodes_.mesh = MeshHandle{};
    bakedMesh_ = MeshHandle{};
}

void CityPlanner::detach() { attach(nullptr, nullptr, nullptr, nullptr); }

Entity CityPlanner::roadEntity() {
    Entity found;
    if (!world_) return found;
    world_->each<RoadEntity, SourceSpec>(
        [&](Entity e, RoadEntity&, SourceSpec& spec) {
            if (found.valid()) return;
            json recipe = json::parse(spec.recipe, nullptr, false);
            if (recipe.is_object() && recipe.contains("generate"))
                found = e;
        });
    return found;
}

std::string CityPlanner::recipe() {
    Entity e = roadEntity();
    if (!e.valid()) return "";
    SourceSpec* spec = world_->get<SourceSpec>(e);
    json recipe = json::parse(spec->recipe, nullptr, false);
    return recipe["generate"].dump();
}

CityPlannerStats CityPlanner::applyRecipe(const std::string& generateJson) {
    CityPlannerStats stats;
    Entity e = roadEntity();
    if (!e.valid()) return stats;
    RoadEntity* net = world_->get<RoadEntity>(e);
    SourceSpec* spec = world_->get<SourceSpec>(e);
    json gen = json::parse(generateJson, nullptr, false);
    if (!net || !spec || !gen.is_object()) return stats;

    // The panel hands back the WHOLE generate block; splice it into the
    // recipe so keys the panel doesn't display survive the round-trip.
    json recipe = json::parse(spec->recipe, nullptr, false);
    if (!recipe.is_object()) recipe = json::object();
    recipe["generate"] = gen;

    const auto t0 = std::chrono::steady_clock::now();
    const RoadGroundFn ground = worldNaturalGround(world_);
    // GRAPH-ONLY regen (never buildRoadNetMesh here — that is [Bake]'s job).
    applyGenerateRecipe(*net, gen, ground);
    // A metro that grew freeway plans gets them re-baked into the fresh graph
    // (same as the tuning panel's Regenerate) — otherwise the corridor and its
    // ramps silently vanish from the editable graph.
    if (const int baked = rebakeNetCorridors(
            *net, gen.value("interchange_spacing", 700.0), ground))
        LOG_INFO << "[planner] regenerate: " << baked
                 << " corridor(s) re-baked into the graph";
    stats.regenMs =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0)
            .count();

    // Round-trip: keep the generate block as the saved form (roadRecipeForSave
    // refreshes only the look from the net — never bakes the nodes).
    spec->recipe = roadRecipeForSave(recipe.dump(), *net).dump();

    stats.nodes = static_cast<int>(net->graph.nodes.size());
    stats.edges = static_cast<int>(net->graph.edges.size());
    stats.spans = junctionSpanCount(*net);
    stats.hubs = static_cast<int>(net->plan.cityHubs.size());
    stats.ok = true;

    // NOTE: the road's RENDERED carriageway (Renderable.mesh) is deliberately
    // left stale — the bones overlays are the live view; [Bake mesh] refreshes
    // the asphalt on demand.
    rebuildOverlays();
    return stats;
}

bool CityPlanner::bakeMesh() {
    Entity e = roadEntity();
    if (!e.valid() || !assets_) return false;
    RoadEntity* net = world_->get<RoadEntity>(e);
    Renderable* r = world_->get<Renderable>(e);
    if (!net || !r) return false;

    RenderMesh mesh = buildRoadNetMesh(*net, worldNaturalGround(world_));
    if (mesh.vertices.empty()) return false;
    // Route the bake through the AssetManager (not renderer.uploadMesh, whose
    // per-regenerate use leaks — editor_system.cpp regenerateRoad TECH_DEBT):
    // acquire the new mesh, then release the handle the road held. The loader
    // acquired the original through this same manager ("road:<n>"), so the
    // release frees it; an unknown handle (a leaked legacy upload) is ignored
    // — that single legacy mesh remains the pre-existing debt, but repeated
    // bakes here do NOT accumulate.
    MeshHandle nh =
        assets_->acquireMesh(mesh, "cityplan:bake:" + std::to_string(++rev_));
    if (!nh.valid()) return false;
    if (r->mesh.valid() && !(r->mesh == nh)) assets_->releaseMesh(r->mesh);
    r->mesh = nh;
    bakedMesh_ = nh;
    // The road's MeshCollider (if any) is load-time and stays stale until
    // Play reloads the document — same lifetime as terrain/lots on a regen.
    LOG_INFO << "[planner] baked road mesh: " << mesh.vertices.size()
             << " vertices";
    return true;
}

CityPlanner::Layer* CityPlanner::layerByName(const std::string& name) {
    if (name == "footprint") return &footprint_;
    if (name == "hubs") return &hubs_;
    if (name == "arterials") return &arterials_;
    if (name == "nodes") return &nodes_;
    return nullptr;
}

bool CityPlanner::clearRoads() {
    Entity e = roadEntity();
    if (!e.valid()) return false;
    RoadEntity* net = world_->get<RoadEntity>(e);
    if (!net) return false;
    // Same hygiene set a regenerate clears, plus the graph itself.
    net->graph.nodes.clear();
    net->graph.edges.clear();
    net->graph.specs.clear();
    net->plan.cityHubs.clear();
    net->plan.freewayPlans.clear();
    net->plan.siteFootprints.clear();
    // Release the baked carriageway (the release half of bakeMesh's swap) so
    // the terrain is genuinely bare — leak-free by the same path.
    if (Renderable* r = world_->get<Renderable>(e)) {
        if (assets_ && r->mesh.valid()) assets_->releaseMesh(r->mesh);
        r->mesh = MeshHandle{};
    }
    bakedMesh_ = MeshHandle{};
    rebuildOverlays();   // graph is empty: every layer publishes empty
    LOG_INFO << "[planner] cleared roads (recipe preserved)";
    return true;
}

void CityPlanner::setLayer(const std::string& name, bool on) {
    Layer* layer = layerByName(name);
    if (!layer) return;
    layer->visible = on;
    if (!world_ || !world_->alive(layer->group)) return;
    if (auto* g = world_->get<InstanceGroup>(layer->group))
        g->transforms = (on && layer->mesh.valid()) ? std::vector<Mat4>{Mat4()}
                                                    : std::vector<Mat4>{};
}

void CityPlanner::publishLayer(Layer& layer, const RenderMesh& mesh,
                               const char* name) {
    // Release the previous rebuild's mesh (we hold exactly one reference).
    if (layer.mesh.valid()) assets_->releaseMesh(layer.mesh);
    layer.mesh = MeshHandle{};

    if (!world_->alive(layer.group)) {
        layer.group = world_->create();
        InstanceGroup g;
        g.material.albedo = Vec3(1, 1, 1);   // colour rides Vertex::color
        g.material.roughness = 1.0f;
        // Overlay: on top of terrain/roads, unlit, never a shadow caster —
        // the debug-gizmo path (render on the group's single identity
        // instance goes through drawMesh, which routes FLAG_OVERLAY).
        g.material.flags = RenderMaterial::FLAG_OVERLAY;
        world_->add<InstanceGroup>(layer.group, g);
    }
    auto* g = world_->get<InstanceGroup>(layer.group);
    if (mesh.vertices.empty()) {
        g->mesh = MeshHandle{};
        g->transforms.clear();
        return;
    }
    layer.mesh = assets_->acquireMesh(
        mesh, "cityplan:" + std::string(name) + ":" + std::to_string(rev_));
    g->mesh = layer.mesh;
    g->boundsCenter = Vec3(0, 0, 0);
    g->boundsRadius = 1.0e6;   // city-wide merged mesh: never cull
    g->transforms = (layer.visible && layer.mesh.valid())
                        ? std::vector<Mat4>{Mat4()}
                        : std::vector<Mat4>{};
}

void CityPlanner::rebuildOverlays() {
    if (!world_ || !assets_) return;   // headless host: stats-only planner
    Entity e = roadEntity();
    if (!e.valid()) return;
    RoadEntity* net = world_->get<RoadEntity>(e);
    if (!net) return;
    ++rev_;   // fresh acquireMesh keys for this rebuild

    const RoadGroundFn groundFn = worldNaturalGround(world_);
    auto ground = [&](const Vec2& p) -> double {
        return groundFn ? groundFn(p.x, p.y) : 0.0;
    };

    // --- footprint: per-site polygon outline + rim gates --------------------
    // Warm sand ribbon so the land contract reads apart from the road tiers;
    // gates as small discs (connector gates darker + larger).
    RenderMesh fpMesh;
    for (const Footprint& sf : net->plan.siteFootprints) {
        if (sf.polygon.size() < 3) continue;
        std::vector<Vec2> ring = sf.polygon;
        double y = 0.0;
        for (const Vec2& v : ring) y = std::max(y, ground(v) + 2.4);
        MeshBuilder::append(
            fpMesh, strokeRibbon(ring, {3.5}, y,
                                 sf.degraded ? Vec3(0.75, 0.42, 0.34)
                                             : Vec3(0.82, 0.68, 0.38),
                                 /*closed=*/true));
        for (const FootprintGate& gate : sf.gates) {
            const bool conn = gate.toSite >= 0;
            appendDisc(fpMesh, gate.pos, conn ? 22.0 : 14.0,
                       ground(gate.pos) + 2.5,
                       conn ? Vec3(0.55, 0.35, 0.16) : Vec3(0.82, 0.68, 0.38));
        }
    }

    // --- hubs: kind-coloured discs + a dark outline ring --------------------
    RenderMesh hubMesh;
    for (const CityHub& h : net->plan.cityHubs) {
        const Vec3 tint = kHubTint[std::min(std::max(h.kind, 0), 4)];
        const double r = (h.site == 0) ? 48.0 : 34.0;
        const double y = ground(h.pos) + 2.2;
        appendDisc(hubMesh, h.pos, r, y, tint);
        appendRing(hubMesh, h.pos, r, 1.6, y + 0.05, Vec3(0.20, 0.23, 0.27));
    }

    // --- arterial graph: class-coloured schematic ribbons, per edge ---------
    // Per-edge flat strokes at the local ground height: strokeRibbon is flat
    // per call, and an edge (a colonization step / curve sample) is short
    // against the terrain, so per-edge draping reads clean in the plan views.
    RenderMesh roadMesh;
    const RoadGraph& g = net->graph;
    for (const RoadEdge& e2 : g.edges) {
        const int a = e2.a, b = e2.b;
        if (a < 0 || b < 0 || a >= static_cast<int>(g.nodes.size()) ||
            b >= static_cast<int>(g.nodes.size()))
            continue;
        const Vec2 pa = g.nodes[a].pos, pb = g.nodes[b].pos;
        const double y = std::max(ground(pa), ground(pb)) + 1.8;
        MeshBuilder::append(roadMesh,
                            strokeRibbon({pa, pb}, {classHalfWidth(e2.klass)}, y,
                                         classColor(e2.klass)));
    }

    // --- junction nodes: small rings, arterial-tier vs local-tier tint ------
    std::vector<int> deg(g.nodes.size(), 0);
    std::vector<int> minK(g.nodes.size(), 99);
    for (const RoadEdge& e2 : g.edges) {
        const int a = e2.a, b = e2.b;
        if (a < 0 || b < 0 || a >= static_cast<int>(g.nodes.size()) ||
            b >= static_cast<int>(g.nodes.size()))
            continue;
        const int k = static_cast<int>(e2.klass);
        ++deg[a];
        ++deg[b];
        minK[a] = std::min(minK[a], k);
        minK[b] = std::min(minK[b], k);
    }
    RenderMesh nodeMesh;
    for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        if (deg[i] < 3) continue;   // junctions, not curve samples
        const bool arterialTier =
            minK[i] <= static_cast<int>(RoadClass::Collector);
        const Vec3 tint = arterialTier ? Vec3(0.702, 0.271, 0.184)    // #b3452f
                                       : Vec3(0.184, 0.435, 0.702);   // #2f6fb3
        appendRing(nodeMesh, g.nodes[i].pos, arterialTier ? 7.0 : 5.0, 1.4,
                   ground(g.nodes[i].pos) + 2.0, tint);
    }

    publishLayer(footprint_, fpMesh, "footprint");
    publishLayer(hubs_, hubMesh, "hubs");
    publishLayer(arterials_, roadMesh, "arterials");
    publishLayer(nodes_, nodeMesh, "nodes");
}

void CityPlanner::cameraPreset(int preset) {
    if (!cameras_) return;
    if (preset == 2) {
        // Free: the perspective fly camera, wherever it last was.
        cameras_->flyController().setOrthographic(false);
        cameras_->setActiveController(/*flyOn=*/true);
        return;
    }

    // Plan framing: the primary city site from the recipe (sites[0], else the
    // top-level center/radius), else the graph bounds.
    Vec2 center(0, 0);
    double radius = 700.0;
    bool framed = false;
    Entity e = roadEntity();
    RoadEntity* net = e.valid() ? world_->get<RoadEntity>(e) : nullptr;
    if (e.valid()) {
        json gen = json::parse(recipe(), nullptr, false);
        if (gen.is_object()) {
            const json* site = &gen;
            if (gen.contains("sites") && gen["sites"].is_array() &&
                !gen["sites"].empty())
                site = &gen["sites"][0];
            if (site->contains("center")) {
                center = Vec2((*site)["center"].value("x", 0.0),
                              (*site)["center"].value("z", 0.0));
                radius = site->value("radius", radius);
                framed = true;
            }
        }
    }
    if (!framed && net && !net->graph.nodes.empty()) {
        Vec2 lo = net->graph.nodes[0].pos, hi = net->graph.nodes[0].pos;
        for (const RoadNode& nd : net->graph.nodes) {
            lo.x = std::min(lo.x, nd.pos.x); lo.y = std::min(lo.y, nd.pos.y);
            hi.x = std::max(hi.x, nd.pos.x); hi.y = std::max(hi.y, nd.pos.y);
        }
        center = (lo + hi) * 0.5;
        radius = std::max(1.0, 0.5 * (hi - lo).length());
    }

    const RoadGroundFn planGround = worldNaturalGround(world_);
    OrbitCameraController& orbit = cameras_->orbitController();
    orbit.target = Vec3(
        center.x, planGround ? planGround(center.x, center.y) : 0.0,
        center.y);
    orbit.distance = std::max(50.0, 2.0 * radius);
    // Orbit pitch convention here: POSITIVE pitch raises the eye above the
    // target (position() uses +sin(pitch)); "look straight down" is +89, not
    // the -89 a Y-down convention would use. 35.264 = atan(1/sqrt(2)), the
    // classic isometric elevation.
    if (preset == 1) {
        orbit.yaw = 45.0;
        orbit.pitch = 35.264;
    } else {
        orbit.yaw = 0.0;
        orbit.pitch = 89.0;
    }
    orbit.setOrthographic(true);   // ortho height tracks distance (plan view)
    cameras_->setActiveController(/*flyOn=*/false);
}

}  // namespace engine
