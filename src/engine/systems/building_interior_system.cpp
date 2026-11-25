#include "building_interior_system.h"

#include "physics_system.h"
#include "../components.h"
#include "../lights/light_kinds.h"   // lightKindId (ADR-0362)
#include "../../log.h"
#include "../../profile.h"
#include "../world.h"
#include "../procgen/city/shape_grammar.h"
#include "../procgen/city/building_uses.h"   // interiorParamsFor, buildingUsesGeneration
#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/furniture_library_lua.h"   // floorProgramsGeneration
#endif
#include "../procgen/surface_maps.h"   // surfaceMaps (bake, ADR-0080)
#include "../procgen/furniture_kit.h"
#include "../procgen/furniture_library.h"   // which pieces are interactive
#include "../interaction.h"
#include "../furniture_draw.h"   // the furniture kit, instanced (buildings M4b)
#include "../model_spawn.h"      // ModelDresser
#include "../prefab_batch.h"     // the prefabs in an interior: copies drawn in batches
#include "../prefab_stamp.h"     // a kit piece stood as a prefab (CityBuildings::prefabSwaps)
#include "../model_motion.h"     // rideMarker: what stood on a swapped piece rides the prefab
#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/model_recipes.h"   // readScriptList
#include "../scripting/prefab_list.h"     // findPrefab: a door prefab, to run its recipe at an opening's size
#endif
#include <nlohmann/json.hpp>
#include <map>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace engine {

namespace {
double msSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
}
}  // namespace

void BuildingInteriorSystem::fixedUpdate(FrameContext& ctx) {
    RT_PROFILE_ZONE_NAMED("buildingInteriors");
    if (!physics_) return;
    // The player (InVehicle redirect — the terrain window's rule: follow the
    // CAR, not the parked character).
    Vec3 player;
    bool found = false;
    ctx.world.each<Transform, ControlledBy>(
        [&](Entity e, Transform& t, ControlledBy&) {
            if (found) return;
            Entity tracked = e;
            if (const InVehicle* iv = ctx.world.get<InVehicle>(e))
                if (iv->vehicle.valid() && ctx.world.alive(iv->vehicle) &&
                    ctx.world.has<Transform>(iv->vehicle))
                    tracked = iv->vehicle;
            const Transform* tt =
                tracked == e ? &t : ctx.world.get<Transform>(tracked);
            player = tt->position;
            found = true;
        });
    if (!found) return;
    step(ctx.world, &physics_->physicsWorld(), ctx.assets, &ctx.renderer,
         player);

    // Room lights (ADR-0080, ADR-0365), published on the light board for LightSystem's rooms pool:
    // - every resident's ceiling fittings, each a light of its fitting's kind kept to its own room's box (the walls stop
    //   it: room-bounded, no shadow map);
    // - a dim fill per storey for the resident NEAREST the player, the 4 storeys around them: the light the rooms
    //   bounce about (corridors and halls have no fittings), until the light probes come.
    std::vector<LightCandidate>& rooms = ctx.view.lights.channel(LightChannel::Rooms);
    rooms.clear();
    ctx.view.lighting.viewerCeilingY = 0;
    {
        static const int fittingKind[3] = {lightKindId("room_panel"), lightKindId("room_round"), lightKindId("room_pendant")};
        for (const std::pair<const std::size_t, Resident>& kv : resident_)
            for (const RoomLight& rl : kv.second.lights) {
                LightCandidate c;
                c.kind = fittingKind[std::clamp(rl.kind, 0, 2)];
                c.position = rl.at;
                c.roomBox = true;
                c.boxCx = static_cast<float>(rl.centre.x);
                c.boxCz = static_cast<float>(rl.centre.y);
                c.boxUx = static_cast<float>(rl.u.x);
                c.boxUz = static_cast<float>(rl.u.y);
                c.boxHalfU = static_cast<float>(rl.halfU);
                c.boxHalfV = static_cast<float>(rl.halfV);
                c.boxFloorY = static_cast<float>(rl.floorY);
                rooms.push_back(c);
            }
    }
    if (!resident_.empty()) {
        const CityBuildings* cb = nullptr;
        ctx.world.each<CityBuildings>(
            [&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
        if (cb) {
            const Vec2 xz(player.x, player.z);
            std::size_t bestKey = static_cast<std::size_t>(-1);
            Real bestD = 1e30;
            for (const auto& kv : resident_) {
                const BuildingRecord& r = cb->records[kv.first];
                if (r.plan.size() < 3) continue;
                Vec2 c(0, 0);
                for (const Vec2& v : r.plan) c = c + v;
                c = c * (Real(1.0) / r.plan.size());
                const Real d = (c - xz).length();
                if (d < bestD) { bestD = d; bestKey = kv.first; }
            }
            if (bestKey != static_cast<std::size_t>(-1)) {
                const BuildingRecord& r = cb->records[bestKey];
                Vec2 c(0, 0);
                Real ext = 1;
                for (const Vec2& v : r.plan) c = c + v;
                c = c * (Real(1.0) / r.plan.size());
                for (const Vec2& v : r.plan)
                    ext = std::max(ext, (v - c).length());
                const std::vector<StoreyPlan> storeys =
                    storeyPlans(r.plan, r.params);
                // Four storeys around the player's (M5: a tower's lit floors
                // follow them up).
                const int nSt = static_cast<int>(storeys.size());
                const int f = storeyOf(r, player.y);
                // the ceiling over the player, while they are in this building (its windows mirror its panels)
                if (cb->recordAt(xz, player.y) == &r && f >= 0 && f < nSt) {
                    const StoreyPlan& sp = storeys[static_cast<std::size_t>(f)];
                    ctx.view.lighting.viewerCeilingY = static_cast<float>(r.baseY + sp.y0 + sp.h);
                }
                const int kFrom = std::max(0, std::min(f - WINDOW_BELOW, nSt - 4));
                const int lightCount = std::min(nSt, kFrom + 4);
                for (int k = kFrom; k < lightCount; ++k) {
                    const StoreyPlan& sp = storeys[static_cast<std::size_t>(k)];
                    static const int roomKind = lightKindId("room_fill");   // lights.lua (ADR-0362, 0365)
                    LightCandidate pl;
                    pl.kind = roomKind;
                    pl.position = Vec3(c.x, r.baseY + sp.y0 + sp.h - 0.5, c.y);
                    pl.range = static_cast<float>(ext * 2.0 + sp.h);
                    rooms.push_back(pl);
                }
            }
        }
    }
}

void BuildingInteriorSystem::step(World& world, PhysicsWorld* phys,
                                  AssetManager& assets, Renderer* renderer,
                                  const Vec3& player) {
    ++stepCount_;
    const CityBuildings* cb = nullptr;
    world.each<CityBuildings>(
        [&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
    if (cb != nullptr && cb->prefabSwaps != swapsFile_) readSwaps(cb->prefabSwaps);
    if (cb != nullptr && cb->prefabDoors != doorsFile_) readDoors(cb->prefabDoors);
    if (!cb || cb->records.empty()) {
        // No city: nothing may stay resident (a level change or a city
        // rebuild would strand bodies); still drain the free queue so onStop
        // leftovers clear.
        if (!resident_.empty()) {
            std::vector<std::size_t> keys;
            for (const auto& kv : resident_) keys.push_back(kv.first);
            for (std::size_t key : keys) release(world, phys, key);
        }
        if (!freeQueue_.empty() && stepCount_ - lastFree_ >= FREE_EVERY) {
            assets.releaseMesh(freeQueue_.back());
            freeQueue_.pop_back();
            lastFree_ = stepCount_;
        }
        return;
    }

    // HOT RELOAD (the building-uses plan): building_uses.lua or floor_programs.lua edited while the game runs -- the
    // interiors standing round the player are let go, and grown again below from the tables as they are now
    {
        long gen = static_cast<long>(buildingUsesGeneration()) * 100003L;
#ifdef RT_ENABLE_SCRIPTING
        gen += floorProgramsGeneration();
#endif
        if (tablesGeneration_ >= 0 && gen != tablesGeneration_ && !resident_.empty()) {
            LOG_INFO << "[interiors] floor uses or programs changed: growing " << resident_.size() << " interiors again";
            std::vector<std::size_t> keys;
            for (const auto& kv : resident_) keys.push_back(kv.first);
            for (std::size_t key : keys) release(world, phys, key);
        }
        tablesGeneration_ = gen;
    }

    const Vec2 xz(player.x, player.z);

    // Desired residents: enterable records the player is INSIDE or
    // APPROACHING (in front of a door, within APPROACH_M).
    std::vector<const BuildingRecord*> nearby;
    cb->near(xz, RELEASE_M, nearby);
    std::vector<std::size_t> desired;   // record indices
    std::vector<std::size_t> aloft;     // approached by a link door: grown round the walker's storey
    std::size_t insideKey = static_cast<std::size_t>(-1);
    for (const BuildingRecord* r : nearby) {
        if (!r->enterable) continue;
        const std::size_t key =
            static_cast<std::size_t>(r - cb->records.data());
        const bool inside =
            player.y > r->groundY - 0.5 &&
            player.y < r->baseY + r->height + 0.5 && r->plan.size() >= 3 &&
            pointInPolygon(r->plan, xz);
        bool approaching = false;
        for (const DoorSpec& d : r->doors) {
            const Vec2 toP = xz - d.foot;
            if (toP.length() < APPROACH_M && dot(toP, d.normal) > 0) {
                approaching = true;
                break;
            }
        }
        // ...or a LINK door at the walker's own storey (a sky bridge's, ADR-0179): approached from the bridge, the
        // tower is grown round that storey, not round its lobby (Glenn, 2026-10-21: "I'm standing in front of a
        // skybridge and I can't cross it!" -- the far tower's floor was not there to step onto)
        if (!inside && r->params.links > 0)
            for (const LinkDoor& ld : linkDoors(r->plan, r->params, -1, r->baseY)) {
                const Vec2 toP = xz - (ld.a + ld.b) * 0.5;
                const Real feet = player.y - 0.7;
                if (toP.length() < APPROACH_M + ld.len && dot(toP, ld.n) > 0 && feet > ld.y0 - 1.0 && feet < ld.y0 + ld.h) {
                    approaching = true;
                    aloft.push_back(key);
                    break;
                }
            }
        if (inside) insideKey = key;
        if (inside || approaching) desired.push_back(key);
    }

    // The storey WINDOW a record streams (M5): whole up to WHOLE_UP_TO
    // storeys; taller buildings grow around the player's storey when inside,
    // the lobby's first three storeys while they approach.
    auto windowFor = [&](const BuildingRecord& r, bool inside, int& k0, int& k1) {
        const int n = r.params.floors + 1;
        if (n <= WHOLE_UP_TO) { k0 = 0; k1 = -1; return; }
        if (!inside) { k0 = 0; k1 = std::min(n, 3); return; }   // (`inside`: or approached aloft, by a link door)
        const int f = storeyOf(r, player.y);
        k0 = std::max(0, f - WINDOW_BELOW);
        k1 = std::min(n, f + WINDOW_ABOVE);
    };
    // A resident window that no longer holds the player's storey with a
    // margin is regrown — unless they are RIDING (in a hoistway, moving
    // vertically): the cab's arrival is when the window catches up.
    const Real dy = havePlayerY_ ? player.y - lastPlayerY_ : Real(0);
    lastPlayerY_ = player.y;
    havePlayerY_ = true;
    if (insideKey != static_cast<std::size_t>(-1)) {
        auto it = resident_.find(insideKey);
        if (it != resident_.end() && it->second.k1 >= 0) {
            const BuildingRecord& r = cb->records[insideKey];
            const int n = r.params.floors + 1;
            const int f = storeyOf(r, player.y);
            Resident& res = it->second;
            const bool low = f < res.k0 + 1 && res.k0 > 0;
            const bool high = f > res.k1 - 2 && res.k1 < n;
            if (low || high) {
                if (!res.coreKnown) {
                    res.core = coreFor(r.plan, r.params, entranceEdgeFor(r.plan, r.params));
                    res.coreKnown = true;
                }
                bool riding = false;
                if (res.core.valid && std::fabs(dy) > 0.01)
                    for (const CoreShaft& h : res.core.hoistways)
                        if (pointInPolygon(h.rect(), xz)) riding = true;
                if (!riding) release(world, phys, insideKey);   // rebuilt below
            }
        }
    }

    // A tower approached by its link door but resident with a window that misses the walker's storey (its lobby,
    // grown when they passed its street door) is regrown round that storey.
    for (std::size_t key : aloft) {
        auto it = resident_.find(key);
        if (it == resident_.end() || it->second.k1 < 0) continue;
        const int f = storeyOf(cb->records[key], player.y);
        if (f < it->second.k0 || f >= it->second.k1) release(world, phys, key);
    }

    // ONE build per step; the record the player is inside builds regardless.
    bool builtThisStep = false;
    for (std::size_t key : desired) {
        if (resident_.count(key)) continue;
        if (resident_.size() >= MAX_RESIDENT && key != insideKey) continue;
        if (builtThisStep && key != insideKey) continue;
        int k0 = 0, k1 = -1;
        windowFor(cb->records[key], key == insideKey || std::find(aloft.begin(), aloft.end(), key) != aloft.end(), k0, k1);
        build(world, phys, assets, renderer, cb->records[key], key, k0, k1);
        builtThisStep = true;
    }

    // Release: resident, not desired, and beyond RELEASE_M from every door
    // (hysteresis — desired only reaches APPROACH_M).
    std::vector<std::size_t> toFree;
    for (const auto& kv : resident_) {
        if (std::find(desired.begin(), desired.end(), kv.first) !=
            desired.end())
            continue;
        const BuildingRecord& r = cb->records[kv.first];
        double nearest = 1e300;
        for (const DoorSpec& d : r.doors)
            nearest = std::min(nearest, (double)(xz - d.foot).length());
        if (r.doors.empty())
            nearest = (xz - Vec2(r.plan[0].x, r.plan[0].y)).length();
        if (nearest > RELEASE_M) toFree.push_back(kv.first);
    }
    for (std::size_t key : toFree) release(world, phys, key);

    // Rate-limited GPU frees: never on a build step (the device wait must not
    // share a step with a grow), at most one per FREE_EVERY steps.
    if (!builtThisStep && !freeQueue_.empty() &&
        stepCount_ - lastFree_ >= FREE_EVERY) {
        assets.releaseMesh(freeQueue_.back());
        freeQueue_.pop_back();
        lastFree_ = stepCount_;
    }
}

int BuildingInteriorSystem::storeyOf(const BuildingRecord& r, Real y, Real feetDrop) {
    const int n = r.params.floors + 1;
    // Feet sit `feetDrop` under the passed position (halfHeight + radius of
    // the player's own capsule; 0.7 for the lab's, 1.1 for metro_v2_test's).
    // 0.4 m of tolerance rounds a walker mid-flight up to the storey they
    // are climbing to. This was a 0.7 literal, so a taller player streamed
    // the storey above the one they stood on (Glenn, 2026-09-16).
    const Real h = y - r.baseY - feetDrop + 0.4;
    if (h < r.params.groundHeight) return 0;
    const Real fh = std::max(Real(1), r.params.floorHeight);
    const int k = 1 + static_cast<int>(std::floor((h - r.params.groundHeight) / fh));
    return std::max(0, std::min(n - 1, k));
}

void BuildingInteriorSystem::residentWindow(std::size_t key, int& k0, int& k1) const {
    auto it = resident_.find(key);
    if (it == resident_.end()) { k0 = 0; k1 = 0; return; }
    k0 = it->second.k0;
    k1 = it->second.k1;
}

// The doors list a level named (building_doors.lua): { default = {...}, types = { home = {...} }, recipes = {...} },
// each entry { front = name or { names }, back = ..., inside = ..., inside_glass = ... }.
void BuildingInteriorSystem::readDoors(const std::string& file) {
    doorsFile_ = file;
    doorDefault_ = DoorChoice();
    doorByType_.clear();
    doorByRecipe_.clear();
    doorModels_ = nlohmann::json();
    if (file.empty()) return;
#ifdef RT_ENABLE_SCRIPTING
    nlohmann::json list;
    std::string why;
    if (!readScriptList(file, std::string(), list, &why) || !readScriptList("models.lua", std::string(), doorModels_, &why)) {
        LOG_WARN << "[interior] doors '" << file << "': " << why << " (every doorway stays open)";
        return;
    }
    auto names = [](const nlohmann::json& row, const char* key, std::vector<std::string>& out) {
        if (!row.is_object() || !row.contains(key)) return;
        out.clear();
        if (row[key].is_string()) out.push_back(row[key].get<std::string>());
        else if (row[key].is_array()) for (const nlohmann::json& n : row[key]) if (n.is_string()) out.push_back(n.get<std::string>());
    };
    auto choice = [&](const nlohmann::json& row, DoorChoice base) {
        names(row, "front", base.front); names(row, "back", base.back);
        names(row, "inside", base.inside); names(row, "inside_glass", base.insideGlass);
        return base;
    };
    if (list.contains("default")) doorDefault_ = choice(list["default"], DoorChoice());
    if (list.contains("types") && list["types"].is_object())
        for (nlohmann::json::const_iterator it = list["types"].begin(); it != list["types"].end(); ++it) doorByType_[it.key()] = choice(it.value(), doorDefault_);
    // (a recipe's row is laid over its own type's, which only the building knows: kept as written, merged in doorsFor)
    if (list.contains("recipes") && list["recipes"].is_object())
        for (nlohmann::json::const_iterator it = list["recipes"].begin(); it != list["recipes"].end(); ++it) {
            DoorChoice c;
            // a name of "-" marks a key the row did not state
            c.front = c.back = c.inside = c.insideGlass = std::vector<std::string>{"-"};
            doorByRecipe_[it.key()] = choice(it.value(), c);
        }
    LOG_INFO << "[interior] doors '" << file << "': " << doorByType_.size() << " kinds of building, " << doorByRecipe_.size() << " recipes";
#endif
}

const std::vector<std::string>& BuildingInteriorSystem::doorsFor(const BuildingRecord& r, std::vector<std::string> DoorChoice::* which) const {
    const std::map<std::string, DoorChoice>::const_iterator byRecipe = doorByRecipe_.find(r.recipe);
    if (byRecipe != doorByRecipe_.end()) {
        const std::vector<std::string>& stated = byRecipe->second.*which;
        if (!(stated.size() == 1 && stated[0] == "-")) return stated;
    }
    const std::map<std::string, DoorChoice>::const_iterator byType = doorByType_.find(r.type);
    return (byType != doorByType_.end() ? byType->second : doorDefault_).*which;
}

bool BuildingInteriorSystem::standDoor(World& world, Entity holder, const std::string& name, const Vec3& at, const Vec2& out, Real width, Real height) {
#ifdef RT_ENABLE_SCRIPTING
    Prefab prefab;
    std::string why;
    if (!findPrefab(name, prefab, std::string(), &why)) {
        LOG_WARN << "[interior] door '" << name << "': " << why;
        return false;
    }
    // the prefab's own model, run again with the opening to fill (sizes to the centimetre, so like openings share
    // one model)
    if (!prefab.model.empty() && doorModels_.contains(prefab.model) && doorModels_[prefab.model].contains("recipe")) {
        const nlohmann::json& m = doorModels_[prefab.model];
        nlohmann::json args = m.contains("args") && m["args"].is_object() ? m["args"] : nlohmann::json::object();
        args["fill_width"] = std::round(width * 100.0) / 100.0;
        args["fill_height"] = std::round(height * 100.0) / 100.0;
        prefab.recipe = m["recipe"].get<std::string>();
        prefab.recipeArgs = args.dump();
        prefab.model.clear();
    }
    Transform t;
    t.position = at;
    t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(out.x, out.y));
    // (no body and nothing to press: the doorway is open to walk through and DoorSystem swings the leaves out of
    // a walker's way, as it did the box leaf this replaces -- ADR-0080's rule that a door can never trap anyone)
    prefab.body = Prefab::Body::None;
    prefab.uses.clear();
    // (a copy in the building's batches; it wakes into a door that swings while the player is near: prefab_batch.h)
    const uint32_t copy = batchPrefab(world, holder, dresser_.get(), prefab, t, std::string(), &why);
    if (copy == kNoBatchCopy) {
        LOG_WARN << "[interior] door '" << name << "': " << why;
        return false;
    }
    PrefabBatches::Copy& c = world.get<PrefabBatches>(holder)->copies[copy];
    c.lively = true;
    c.swings = true;
    c.swingFoot = Vec2(at.x, at.z);
    c.swingNormal = out;
    return true;
#else
    (void)world; (void)holder; (void)name; (void)at; (void)out; (void)width; (void)height;
    return false;
#endif
}

// The list a level named (prefab_swaps.lua): kit piece = { prefab = "...", back = metres }.
void BuildingInteriorSystem::readSwaps(const std::string& file) {
    swapsFile_ = file;
    swaps_.clear();
    if (file.empty()) return;
#ifdef RT_ENABLE_SCRIPTING
    nlohmann::json list;
    std::string why;
    if (!readScriptList(file, std::string(), list, &why)) {
        LOG_WARN << "[interior] prefab swaps '" << file << "': " << why << " (every piece stays the kit's)";
        return;
    }
    for (nlohmann::json::const_iterator it = list.begin(); it != list.end(); ++it) {
        Piece piece;
        if (!it.value().is_object() || !furniturePieceByName(it.key(), piece)) {
            LOG_WARN << "[interior] prefab swaps: '" << it.key() << "' is not a kit piece";
            continue;
        }
        Swap s;
        s.prefab = it.value().value("prefab", std::string());
        s.back = it.value().value("back", -1.0);
        s.surface = it.value().value("surface", std::string());
        s.turn = it.value().value("turn", 0.0);
        if (!s.prefab.empty()) swaps_[it.key()] = s;
    }
    LOG_INFO << "[interior] prefab swaps '" << file << "': " << swaps_.size() << " kit pieces stand as prefabs";
#endif
}

void BuildingInteriorSystem::build(World& world, PhysicsWorld* phys,
                                   AssetManager& assets, Renderer* renderer,
                                   const BuildingRecord& r, std::size_t key,
                                   int k0, int k1) {
    const auto t0 = std::chrono::steady_clock::now();
    RenderMesh collider;
    const BuildingParams params = interiorParamsFor(r);   // the runtime facts the lot cache does not hold
    BuildingMesh bm = growInterior(r.plan, params, r.baseY, &collider, k0, k1);
    const double growMs = msSince(t0);

    Resident res;
    res.k0 = k0;
    res.k1 = k1;
    res.lights = std::move(bm.roomLights);
    std::size_t tris = 0, bytes = 0;
    for (const RenderMesh& part : bm.parts) {
        if (part.indices.empty()) continue;
        tris += part.indices.size() / 3;
        bytes += part.vertices.size() * sizeof(Vertex) +
                 part.indices.size() * sizeof(uint32_t);
        const PartId pid = static_cast<PartId>(part.materialIndex);
        MeshHandle mh = assets.acquireMesh(part);
        Entity e = world.create();
        Transform t;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable rd;
        rd.mesh = mh;
        // Interior-floor-family parts: the FLOOR finish claims its part
        // first (floors dominate a shared part), the STAIR finish its own;
        // anything else keeps the part material.
        const bool floorFam = pid == PartId::InteriorFloor ||
                              pid == PartId::InteriorFloorTile ||
                              pid == PartId::InteriorFloorMarble ||
                              pid == PartId::InteriorFloorCarpet;
        if (floorFam && pid == floorFinishPartFor(r.params))
            rd.material = floorFinishFor(r.params);
        else if (floorFam && pid == stairFinishPartFor(r.params))
            rd.material = stairFinishFor(r.params);
        else if (pid == PartId::Glass || pid == PartId::GlassLit) {
            // The interior's own panes are CLEAR (looking out): a faint blue
            // tint, a sharp fresnel, most of the city coming through — and
            // both faces, since the facade's pane is front-only and gone from
            // in here.
            rd.material = materialFor(PartId::Glass, r.params.wallColor);
            rd.material.albedo = Vec3(0.55, 0.66, 0.74);
            rd.material.metallic = 0.0f;
            rd.material.roughness = 0.06f;
            rd.material.opacity = 0.18f;
            rd.material.flags &= ~RenderMaterial::FLAG_FRONT_ONLY;
            rd.material.flags &= ~RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT;
        } else
            rd.material = materialFor(pid, r.params.wallColor);
        // Bind the baked surface texture set, like the loader does for the
        // merged city parts — the surface FLAG alone leaves the shader on
        // its procedural fallback (world-planar, no normal map): the black/
        // diagonal upper floors.
        const RenderMaterial::Surface surf = rd.material.surface();
        if (renderer && surf != RenderMaterial::Surface::None) {
            const int sid = static_cast<int>(surf);
            auto it = surfTex_.find(sid);
            if (it == surfTex_.end()) {
                SurfaceMaps mp = surfaceMaps(surf, 256, 1337u);
                auto up = [&](const TextureData& td) {
                    return renderer->uploadTexture(td.width, td.height,
                                                   td.channels,
                                                   td.pixels.data());
                };
                it = surfTex_
                         .emplace(sid,
                                  std::array<TextureHandle, 4>{
                                      up(mp.albedo), up(mp.normal),
                                      up(mp.mr), up(mp.ao)})
                         .first;
            }
            rd.material.albedoMap = it->second[0];
            rd.material.normalMap = it->second[1];
            rd.material.metallicRoughnessMap = it->second[2];
            rd.material.aoMap = it->second[3];
        }
        rd.drawClass = DrawClass::Structure;
        rd.drawDistance = 300.0;
        world.add<Renderable>(e, rd);
        // Only the lit panes' inner faces glow at night (they are the windows
        // seen from inside); drywall keeps its own faint self-light — ramping
        // the walls lit every room up like a window (Glenn, 2026-09-14).
        // (No NightGlow on the interior's panes: they are clear now, and a
        // glowing sheet between you and the city was the old look.)
        res.entities.push_back(e);
        res.meshes.push_back(mh);
    }

    // PREFABS IN PLACE OF KIT PIECES (#191, "One catalog"; a level's citysim.prefabSwaps). The floor planner placed
    // a kit piece; where the level's list names a prefab for that piece, one copy of the prefab is stood there
    // and the kit piece is left out of everything below (its instance group, its seat). The prefab stands with its
    // back where the kit piece's was. One entity a copy, until many copies are drawn as a group (step 10).
    // (the copies are drawn in batches the resident holds: one entity for the lot, and a group for each thing drawn)
    res.holder = world.create();
    world.add<PrefabBatches>(res.holder, PrefabBatches{});
    struct OnTop {
        std::size_t piece;     // an index into bm.furniture as it is after the swap
        uint32_t owner;        // the copy it stands on
        std::string surface;
        Real drop = 0;         // how far it is lowered to rest on that surface
    };
    std::vector<OnTop> onTop;
    bool prefabPieces = false;   // (a room program asked for a prefab piece: stood as that prefab, list or no list)
    for (const PlacedPiece& pp : bm.furniture) prefabPieces = prefabPieces || pieceIsPrefab(static_cast<Piece>(pp.piece));
    if ((!swaps_.empty() || prefabPieces) && !bm.furniture.empty()) {
        if (dresser_ == nullptr && renderer != nullptr) dresser_ = std::make_shared<ModelDresser>(*renderer, assets);
        world.get<PrefabBatches>(res.holder)->dresser = dresser_;   // (what dresses a copy that wakes)
        // where a kit piece's prefab stands: its back where the piece's was, turned as the piece was
        // ...and at the height the kit piece's own shape begins: a wall cupboard hangs at 1.45 m in its piece's space,
        // and its prefab, which stands on y = 0, goes there, not on the floor.
        auto placeOf = [](const PlacedPiece& pp, Real back, Real turnDeg) {
            Real foot = 1e9;
            for (const RenderMesh& m : furniturePiece(static_cast<Piece>(pp.piece), pp.variant).mesh)
                for (const Vertex& v : m.vertices) foot = std::min(foot, static_cast<Real>(v.position.y));
            if (foot > 1e8) foot = 0;   // (a prefab piece: nothing of the kit's to measure)
            Transform at;
            at.position = pp.xform.transformPoint(Vec3(0, foot, back));
            const Vec3 out = pp.xform.transformPoint(Vec3(0, foot, back + 1)) - at.position;
            at.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(out.x, out.z) + turnDeg * Real(3.14159265358979323846 / 180.0));
            return at;
        };
        struct Stood {
            uint32_t copy;
            Mat4 toPiece;        // world -> the kit piece's own space
            Vec3 size;           // the kit piece's
            std::string surface;
            Real topY = 0;       // the height (world) of the kit piece's top, which what stood on it was placed at
        };
        // How far to lower what stood on a kit piece so it rests on the prefab's surface, whose top is seldom at
        // the kit piece's height (Glenn, 2026-11-21: "the objects are offset from the top of the desk's surface").
        auto dropOnto = [&](const Stood& o) {
            Vec3 at;
            Quat turn;
            return batchCopyMarker(world.get<PrefabBatches>(res.holder)->copies[o.copy], o.surface, at, turn) ? o.topY - at.y : Real(0);
        };
        std::vector<Stood> owners;
        std::vector<uint32_t> stoodCopies;
        std::vector<PlacedPiece> kept, rest;
        std::size_t stood = 0, riding = 0;
        for (const PlacedPiece& pp : bm.furniture) {
            // which prefab stands here: the piece's own (a prefab piece), or the one the level's list names for it
            const std::string kitName = furniturePieceName(static_cast<Piece>(pp.piece));
            Swap what;
            if (pieceIsPrefab(static_cast<Piece>(pp.piece))) what.prefab = kitName;
            else {
                const std::map<std::string, Swap>::const_iterator swap = swaps_.find(kitName);
                if (swap == swaps_.end()) { rest.push_back(pp); continue; }
                what = swap->second;
            }
            const Vec3 size = furniturePiece(static_cast<Piece>(pp.piece), pp.variant).size;
            std::string why;
            Prefab line;
            uint32_t e = kNoBatchCopy;
#ifdef RT_ENABLE_SCRIPTING
            if (findPrefab(what.prefab, line, std::string(), &why))
                e = batchPrefab(world, res.holder, dresser_.get(), line, placeOf(pp, what.back >= 0 ? what.back : size.z * Real(0.5), what.turn), std::string(), &why);
#endif
            if (e == kNoBatchCopy) {
                LOG_WARN << "[interior] the prefab for '" << kitName << "': " << why << (pieceIsPrefab(static_cast<Piece>(pp.piece)) ? " (nothing stands there)" : " (the kit piece stays)");
                if (!pieceIsPrefab(static_cast<Piece>(pp.piece))) rest.push_back(pp);
                continue;
            }
            stoodCopies.push_back(e);
            ++stood;
            if (!what.surface.empty()) {
                Real top = 0;
                for (const RenderMesh& m : furniturePiece(static_cast<Piece>(pp.piece), pp.variant).mesh)
                    for (const Vertex& v : m.vertices) top = std::max(top, static_cast<Real>(v.position.y));
                owners.push_back(Stood{e, pp.xform.inverse(), size, what.surface, pp.xform.transformPoint(Vec3(0, top, 0)).y});
            }
        }
        // WHAT STOOD ON A SWAPPED PIECE. A swapped piece that itself stood on another (a desk lamp on a desk) is
        // tied to the prefab's surface marker here. A kit piece that stood on one (a monitor, a mug) stays the
        // kit's and stays in its batch: it is tied below, once the batches are made and its place in one is known.
        auto ownerUnder = [&](const Vec3& at) -> const Stood* {
            for (const Stood& o : owners) {
                const Vec3 p = o.toPiece.transformPoint(at);
                if (std::fabs(p.x) <= o.size.x * Real(0.5) + Real(0.03) && p.z >= Real(-0.03) && p.z <= o.size.z + Real(0.03) &&
                    p.y > Real(0.3) && p.y < o.size.y + Real(0.6))
                    return &o;
            }
            return nullptr;
        };
        for (const uint32_t e : stoodCopies) {
            const Stood* under = ownerUnder(world.get<PrefabBatches>(res.holder)->copies[e].at.position);
            if (under == nullptr || under->copy == e) continue;
            const Real drop = dropOnto(*under);
            PrefabBatches* held = world.get<PrefabBatches>(res.holder);
            held->copies[e].at.position.y -= drop;
            poseBatchCopy(world, res.holder, e, held->copies[e].pose);   // (drawn where it now rests)
            held->copies[under->copy].riders.emplace_back(e, under->surface);   // (it wakes and moves with what it stands on)
            ++riding;
        }
        kept = rest;
        for (std::size_t i = 0; i < kept.size(); ++i)
            if (const Stood* under = owners.empty() ? nullptr : ownerUnder(kept[i].xform.transformPoint(Vec3(0, 0, 0))))
                onTop.push_back(OnTop{i, under->copy, under->surface, dropOnto(*under)});
        if (stood > 0) LOG_INFO << "[interior] " << stood << " kit pieces stand as prefabs (" << riding << " of them on another), "
                                << onTop.size() << " kit pieces stand on one, " << kept.size() << " stay the kit's";
        bm.furniture.swap(kept);
    }

    // DOORS IN THE DOORWAYS (#191; a level's citysim.prefabDoors): a door prefab in the street door, the back door
    // and every doorway between rooms, each fitted to its opening. Which door is the list's (building_doors.lua),
    // by the building's recipe and kind; one building's doors of a sort are all the same one.
    if (!doorsFile_.empty()) {
        if (dresser_ == nullptr && renderer != nullptr) dresser_ = std::make_shared<ModelDresser>(*renderer, assets);
        world.get<PrefabBatches>(res.holder)->dresser = dresser_;   // (what dresses a copy that wakes)
        const std::size_t pick = static_cast<std::size_t>(r.params.seed);   // (the building's own roll)
        auto one = [&](std::vector<std::string> DoorChoice::* which) {
            const std::vector<std::string>& names = doorsFor(r, which);
            return names.empty() ? std::string() : names[pick % names.size()];
        };
        std::size_t hung = 0;
        const Real inset = outsideWallDepth(r.params);
        for (const DoorSpec& d : r.doors) {
            if (d.open || d.width < 0.7) continue;
            const std::string name = one(d.back ? &DoorChoice::back : &DoorChoice::front);
            if (name.empty()) continue;
            const Vec2 foot = d.foot - d.normal * (inset * Real(0.5));   // in the middle of the wall's depth
            if (standDoor(world, res.holder, name, Vec3(foot.x, r.baseY + 0.05, foot.y), d.normal, d.width, d.height - Real(0.05))) ++hung;
        }
        for (const Doorway& w : bm.doorways) {
            const std::string name = one(w.glass ? &DoorChoice::insideGlass : &DoorChoice::inside);
            if (name.empty()) continue;
            if (standDoor(world, res.holder, name, w.at, Vec2(w.along.y, -w.along.x), w.width, w.height)) ++hung;
        }
        if (hung > 0) LOG_INFO << "[interior] " << hung << " doors hung in a " << r.recipe << " (" << r.type << "): street door '" << one(&DoorChoice::front)
                               << "', between rooms '" << one(&DoorChoice::inside) << "' in " << bm.doorways.size() << " doorways";
    }

    // FURNITURE (buildings M4b): the placed pieces, one INSTANCE GROUP per (piece, variant, finish) -- the kit's
    // mesh uploaded once for every building, this building's placements as its transforms.
    if (!bm.furniture.empty()) {
        std::vector<std::vector<FurnitureDrawn>> where;
        tris += spawnFurnitureGroups(world, assets, renderer, drawCache_, bm.furniture, r.params.wallColor, 80.0,
                                     res.entities, onTop.empty() ? nullptr : &where);
        // a kit piece that stands on a prefab: its one transform in each of its batches follows the prefab's surface
        for (const OnTop& t : onTop)
            for (const FurnitureDrawn& d : where[t.piece]) {
                if (InstanceGroup* g = world.get<InstanceGroup>(d.group))
                    if (d.index < g->transforms.size()) g->transforms[d.index].m[1][3] -= t.drop;
                // (tied to the copy it stands on when that copy wakes, so a desk that rises carries it)
                world.get<PrefabBatches>(res.holder)->copies[t.owner].groupRiders.push_back(PrefabBatches::Copy::GroupRider{d.group, d.index, t.surface});
            }
    }

    // INTERACTIONS (the furniture library, M1): the pieces someone can sit or lie on, as ONE set entity with the
    // building -- InteractionSystem asks it what is in reach; released with the rest of the resident.
    {
        const FurnitureLibrary& lib = FurnitureLibrary::global();
        Interactables set;
        for (const PlacedPiece& pp : bm.furniture)
            if (lib.interactive(static_cast<Piece>(pp.piece)))
                set.pieces.push_back({pp.piece, interactXform(lib.find(static_cast<Piece>(pp.piece)), pp.xform, pp.variant), 0, pp.unit, pp.variant});
        for (const FurnishedUnit& u : bm.units) set.units.push_back({u.rect, u.y0});
        if (!set.pieces.empty()) {
            set.building = static_cast<int>(key);   // whose people sit here (the citysim's indoor occupants)
            set.refreshBounds();
            Entity ie = world.create();
            world.add<Interactables>(ie, std::move(set));
            res.entities.push_back(ie);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    if (phys && !collider.indices.empty()) {
        std::vector<Vec3> verts;
        verts.reserve(collider.vertices.size());
        for (const Vertex& v : collider.vertices) verts.push_back(v.position);
        // Both-ways triangles: a stair must hold from above AND read from
        // below, and quad winding here follows render normals, not physics.
        std::vector<uint32_t> idx = collider.indices;
        const std::size_t oneSided = idx.size();
        idx.reserve(oneSided * 2);
        for (std::size_t i = 0; i + 2 < oneSided; i += 3) {
            idx.push_back(idx[i]);
            idx.push_back(idx[i + 2]);
            idx.push_back(idx[i + 1]);
        }
        res.body = phys->addMesh(verts, idx, Vec3(0, 0, 0), 0.85);
    }
    const double joltMs = msSince(t1);

    resident_[key] = std::move(res);
    LOG_INFO << "[interior] built " << r.recipe << " (" << r.type << ") storeys ["
             << k0 << ", " << (k1 < 0 ? r.params.floors + 1 : k1) << ") of "
             << r.params.floors + 1 << " tris="
             << tris << " grow=" << growMs << "ms jolt=" << joltMs
             << "ms bytes=" << bytes << " fin=f"
             << (((r.params.seed >> 6) & 7u) % 5u) << "s"
             << ((r.params.seed >> 9) & 3u) << "p"
             << ((r.params.seed >> 11) & 7u) << " resident="
             << resident_.size();
}

void BuildingInteriorSystem::release(World& world, PhysicsWorld* phys,
                                     std::size_t key) {
    auto it = resident_.find(key);
    if (it == resident_.end()) return;
    if (phys && it->second.body != INVALID_PHYSICS_BODY)
        phys->removeBody(it->second.body);
    for (Entity e : it->second.entities)
        if (world.alive(e)) world.destroy(e);
    if (it->second.holder.valid() && world.alive(it->second.holder)) {
        releaseBatches(world, it->second.holder, [&](PhysicsBodyId id) { if (phys) phys->removeBody(id); });
        world.destroy(it->second.holder);
    }
    for (MeshHandle mh : it->second.meshes) freeQueue_.push_back(mh);
    resident_.erase(it);
    LOG_INFO << "[interior] released record " << key
             << " resident=" << resident_.size()
             << " freeQueue=" << freeQueue_.size();
}

void BuildingInteriorSystem::onStop(FrameContext& ctx) {
    ctx.view.lights.channel(LightChannel::Rooms).clear();
    PhysicsWorld* phys = physics_ ? &physics_->physicsWorld() : nullptr;
    std::vector<std::size_t> keys;
    for (const auto& kv : resident_) keys.push_back(kv.first);
    for (std::size_t key : keys) release(ctx.world, phys, key);
    for (MeshHandle mh : freeQueue_) ctx.assets.releaseMesh(mh);
    freeQueue_.clear();
}

}  // namespace engine
