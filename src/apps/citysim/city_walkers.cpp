#include "../../log.h"
#include "city_walkers.h"
#include <cstdio>

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/world.h"
#include "city_sim.h"

#include <chrono>
#include <cmath>

namespace citysim {

using engine::Vec2;
using engine::Vec3;
using engine::Quat;
using engine::Real;
using engine::Entity;
using engine::World;
using engine::Transform;
using engine::PrevTransform;
using engine::Renderable;
using engine::CharacterController;
using engine::PhysicsWorld;

namespace {
constexpr Real kCapsuleRadius = 0.25;   // walker capsule (0.5 wide, 1.8 tall)
constexpr Real kCapsuleHalf = 0.65;     // cylinder half-height
constexpr Real kWalkGain = 1.2;         // metres behind the ghost -> m/s toward it
constexpr Real kWalkStandoff = 0.4;     // settle this short of the ghost's spot
constexpr Real kWalkMax = 2.4;          // catch-up ceiling (a brisk jog)
constexpr Real kTetherLead = 5.0;       // ghost may lead its walker by at most this
constexpr Real kTetherGiveUp = 4.0;     // ...for at most this long while stuck
constexpr Real kClosingRate = 0.20;     // closing slower than this is not closing
constexpr Real kKnockRadius = 1.5;      // a vehicle centre this close...
constexpr Real kKnockSpeed = 2.5;       // ...moving this fast -> knockdown
constexpr Real kFaceSpeed = 0.3;        // turn to face travel above this speed
constexpr Real kPersonalSpace = 1.3;    // 360-degree separation radius from people
constexpr Real kSeparationGain = 1.8;   // push strength inside that radius
constexpr Real kBackoffTime = 0.8;      // blocked -> stand still this long, re-think
}  // namespace

// The walk cycle (kWalkPoses / walkPoseSwing / walkPoseIndex, city_meshes.h) is
// SHARED with the third-person player body — one set of constants, one phase
// rule, so the player and the crowd stride identically.
void CityWalkerSystem::watchDrawn(Walker& w, Vec2 p, int path, Real dt, const CitySim& sim) {
    constexpr Real kFastest = 5.0;   // m/s: past a sprint (the kickabout's runners top out at 4)
    if (w.drawnValid && dt > 1e-6) {
        const Real d = (p - w.drawnLast).length();
        const Real sp = d / dt;
        if (sp > kFastest && d > 0.2) {
            SpeedEvent e;
            e.at = clockSec_;
            e.agent = w.agentId;
            e.from = w.drawnLast;
            e.to = p;
            e.speed = sp;
            e.pathFrom = w.drawnPath;
            e.pathTo = path;
            e.state = sim.describeAgent(w.agentId);
            ++speedEventCount_;
            if (std::getenv("RT_SPEED_LOG"))
                std::fprintf(stderr, "[speeder] %.1f s: %.1f m/s, %.2f m (%.1f %.1f -> %.1f %.1f) drawn %d -> %d | %s\n", e.at, sp, d,
                             e.from.x, e.from.y, e.to.x, e.to.y, e.pathFrom, e.pathTo, e.state.c_str());
            speedEvents_.push_back(std::move(e));
            if (speedEvents_.size() > 64) speedEvents_.erase(speedEvents_.begin());
        }
    }
    w.drawnLast = p;
    w.drawnValid = true;
    w.drawnPath = path;
}

engine::MeshHandle CityWalkerSystem::poseMesh(engine::AssetManager& assets,
                                              int outfit, int pose) {
    int key = outfit * kWalkPoses + pose;
    auto it = poseMeshes_.find(key);
    if (it != poseMeshes_.end()) return it->second;
    engine::MeshHandle h = assets.acquireMesh(
        buildPersonMesh(walkPoseSwing(pose), outfit),
        "citywalk:person" + std::to_string(outfit) + ":" + std::to_string(pose));
    poseMeshes_[key] = h;
    return h;
}

void CityWalkerSystem::spawnWalkers(engine::FrameContext& ctx) {
    // TIERED (8km-city plan P6, the 20fps fix): a physical walker body — a
    // Jolt capsule stepped every tick plus 360-degree separation against
    // every other body — exists ONLY for K-tier pedestrians (the bubble the
    // player can see; ~200 downtown). Piedmont's 1800 all-city capsules were
    // 10.1 ms of the 13 ms fixed step. V-tier peds keep full sim identity
    // and re-grow a body the moment the bubble reaches them. With tiering
    // off (small levels, tests) every ped is K and behavior is unchanged —
    // except that this reconcile now also runs per step, which is what lets
    // bodies come and go at all.
    if (!city_.built()) return;
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    PhysicsWorld& pw = physics_.physicsWorld();

    const auto& agents = sim.agents();
    // Drop walkers whose agents left the K tier (explicit character removal —
    // physics has no entity-destroy hook; see the onStop note).
    for (std::size_t wi = 0; wi < walkers_.size();) {
        Walker& w = walkers_[wi];
        // Gone INDOORS (home, work, a cafe), onto a bus, or out of the bubble:
        // no body on the pavement (CitySim::pedVisible). It gets a fresh one
        // when it steps out again.
        const bool stale =
            w.agentId < 0 || w.agentId >= static_cast<int>(agents.size()) ||
            !sim.pedVisible(w.agentId);
        if (!stale) { ++wi; continue; }
        if (w.agentId >= 0 && w.agentId < static_cast<int>(agents.size())) {
            const Agent& ga = agents[static_cast<std::size_t>(w.agentId)];
            const char* why = ga.mode != Agent::Mode::Pedestrian ? "got in a car"
                              : ga.far()                          ? "left the near tier"
                              : sim.riding(w.agentId)             ? "boarded a ride"
                              : ga.indoors                        ? "went indoors"
                                                                  : "stopped (not moving, not indoors?)";
            gone_[w.agentId] = Gone{std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                                    why, ga.pos};
        }
        if (world.alive(w.entity)) {
            if (CharacterController* cc = world.get<CharacterController>(w.entity))
                if (cc->characterId != engine::INVALID_CHARACTER)
                    pw.removeCharacter(cc->characterId);
            world.destroy(w.entity);
        }
        // THE LEASH GOES WITH THE BODY. setAgentTether is fed only by walkers
        // that HAVE one, so a reaped body leaves its ghost tethered to where it
        // last stood — and the next time that agent walks more than the lead
        // from that stale point, the stepper stops advancing it, for ever, with
        // nothing left in the world to explain why.
        if (w.agentId >= 0 && w.agentId < static_cast<int>(agents.size()))
            city_.simMutable().clearAgentTether(w.agentId);
        walkers_[wi] = walkers_.back();
        walkers_.pop_back();
    }
    // Membership of the CURRENT walker set, then grow the missing K peds.
    if (haveWalker_.size() != agents.size()) haveWalker_.assign(agents.size(), 0);
    else for (int id : haveSet_) haveWalker_[static_cast<std::size_t>(id)] = 0;   // (last step's marks only)
    haveSet_.clear();
    for (const Walker& w : walkers_) if (w.agentId >= 0 && w.agentId < static_cast<int>(agents.size())) haveSet_.push_back(w.agentId);
    for (const Walker& w : walkers_)
        if (w.agentId >= 0 && w.agentId < static_cast<int>(agents.size()))
            haveWalker_[w.agentId] = 1;
    // (a visible pedestrian is never far: the near list, in the same order -- not all 100k agents every step)
    for (int i : sim.nearAgents()) {
        const Agent& a = agents[static_cast<std::size_t>(i)];
        if (!sim.pedVisible(i)) continue;
        if (haveWalker_[i]) continue;
        if (auto g = gone_.find(i); g != gone_.end()) {
            const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if (now - g->second.at < 1.5) {
                ++flickers_;
                char b[600];
                std::snprintf(b, sizeof b, "agent %d gone %.2f s (%s) at %.1f %.1f, back at %.1f %.1f: %s", i,
                              now - g->second.at, g->second.why, g->second.pos.x, g->second.pos.y, a.pos.x, a.pos.y,
                              sim.describeAgent(i).substr(0, 300).c_str());
                flickerLog_.push_back(b);
                if (flickerLog_.size() > 12) flickerLog_.erase(flickerLog_.begin());
            }
            gone_.erase(g);
        }

        Entity e = world.create();
        // Spawn a little above the ghost's spot; the character settles under
        // gravity like the player does. Transform.position = capsule CENTRE.
        Transform t;
        t.position = Vec3(a.pos.x, kCapsuleHalf + kCapsuleRadius + 0.6, a.pos.y);
        t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0),
                                            std::atan2(a.heading.x, a.heading.y));
        t.scale = Vec3(1, 1, 1);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});

        // A simple articulated person (device ask): head, torso, swinging limbs,
        // outfit picked deterministically per walker. Hue lives in the mesh's
        // vertex colours, so the material stays white.
        int outfit = static_cast<int>((static_cast<uint32_t>(i) * 2654435761u >> 8) %
                                      static_cast<uint32_t>(personOutfitCount()));
        Renderable r;
        r.mesh = poseMesh(ctx.assets, outfit, 0);
        r.material.albedo = Vec3(1, 1, 1);
        r.material.metallic = 0.0f;
        r.material.roughness = 0.9f;
        r.material.opacity = 1.0f;
        world.add<Renderable>(e, r);

        // NO CharacterController yet — the body budget below decides who gets
        // one. An entity without it is drawn and animated but has no capsule.

        Walker w;
        w.entity = e;
        w.agentId = i;
        w.outfit = outfit;
        w.facing = a.heading;
        w.blocked.stallSeconds = 1.2;        // walker-scale stall watchdog
        w.blocked.minCommand = 0.5;
        w.blocked.minMotion = 0.15;
        walkers_.push_back(w);
    }

    // --- THE BODY BUDGET ------------------------------------------------
    // Every K pedestrian now has an entity and is drawn; this decides which of
    // them also get a Jolt capsule. Nearest-first up to the level's budget,
    // exactly
    // the shape CityPhysicsSystem::possessTier uses for its 12 physical cars.
    const int budget = city_.params().maxWalkerBodies;
    if (budget <= 0) {   // unbounded: every K ped keeps a capsule (historical)
        for (Walker& w : walkers_) {
            if (!world.alive(w.entity)) continue;
            if (world.get<CharacterController>(w.entity)) continue;
            CharacterController add;
            add.radius = kCapsuleRadius;
            add.halfHeight = kCapsuleHalf;
            add.stepHeight = 0.4;
            world.add<CharacterController>(w.entity, add);
        }
        return;
    }
    if (!sim.hasTierCenter()) return;   // no player fed: leave bodies as they are
    const Vec2 centre = sim.tierCenter();
    auto dist2 = [&](const Agent& a) {
        const Real dx = a.pos.x - centre.x, dy = a.pos.y - centre.y;
        return dx * dx + dy * dy;
    };

    // Rank every walker by distance, uid breaking ties so the choice cannot
    // depend on walker_ ordering (which swap-erase churns) — ADR-0002.
    bodyRank_.clear();
    bodyRank_.reserve(walkers_.size());
    for (std::size_t wi = 0; wi < walkers_.size(); ++wi) {
        const Walker& w = walkers_[wi];
        if (w.agentId < 0 || w.agentId >= static_cast<int>(agents.size())) continue;
        const Agent& a = agents[w.agentId];
        bodyRank_.push_back({dist2(a), a.uid, static_cast<int>(wi)});
    }
    std::sort(bodyRank_.begin(), bodyRank_.end(),
              [](const BodyRank& x, const BodyRank& y) {
                  if (x.d2 != y.d2) return x.d2 < y.d2;
                  return x.uid < y.uid;
              });

    const Real acquire2 = bodyRadius_ * bodyRadius_;
    const Real keep2 = (bodyRadius_ * 1.3) * (bodyRadius_ * 1.3);
    const int keepRank = static_cast<int>(budget * 1.3);
    for (int rank = 0; rank < static_cast<int>(bodyRank_.size()); ++rank) {
        Walker& w = walkers_[static_cast<std::size_t>(bodyRank_[rank].pos)];
        if (!world.alive(w.entity)) continue;
        CharacterController* cc = world.get<CharacterController>(w.entity);
        const Real d2 = bodyRank_[rank].d2;
        // Hysteresis in BOTH the rank and the radius: a walker keeps a body it
        // already has until it is clearly outside the set, so a body is never
        // created and destroyed on alternating steps.
        const bool wantBody = cc ? (rank < keepRank && d2 < keep2)
                                 : (rank < budget && d2 < acquire2);
        if (wantBody && !cc) {
            CharacterController add;
            add.radius = kCapsuleRadius;
            add.halfHeight = kCapsuleHalf;
            add.stepHeight = 0.4;            // kerbs and sidewalk lips
            world.add<CharacterController>(w.entity, add);   // PhysicsSystem makes it
        } else if (!wantBody && cc) {
            if (cc->characterId != engine::INVALID_CHARACTER)
                pw.removeCharacter(cc->characterId);   // no entity-destroy hook
            world.remove<CharacterController>(w.entity);
        }
    }
}

void CityWalkerSystem::driveWalkers(engine::FrameContext& ctx) {
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    PhysicsWorld& pw = physics_.physicsWorld();
    Real dt = ctx.clock.fixedStep();
    clockSec_ += dt;

    // Vehicle poses + speeds once for all walkers (the knockdown trigger): the
    // REAL vehicles (player's / promoted), plus the ambient planner cars — with
    // one motion authority their drawn pose IS the sim pose.
    std::vector<Vec2> carPos;
    std::vector<Real> carSpeed;
    world.each<Transform, engine::Vehicle>([&](Entity, Transform& t, engine::Vehicle& v) {
        carPos.push_back(Vec2(t.position.x, t.position.z));
        Real s = 0;
        if (v.vehicleId != PhysicsWorld::INVALID_VEHICLE) {
            Vec3 vel = pw.vehicleVelocity(v.vehicleId);
            s = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        }
        carSpeed.push_back(s);
    });
    for (int ni : sim.nearAgents()) {   // (the near list: a far car has no body -- see below)
        const Agent& a = sim.agents()[static_cast<std::size_t>(ni)];
        if (a.mode != Agent::Mode::Driver || a.released || !a.moving) continue;
        // FAR CARS CANNOT RUN ANYONE OVER. This was the only place in citysim
        // that iterated far-tier agents: it gathered every moving driver in the
        // whole 8 km city, and the knockdown test below is O(walkers x this
        // list). A car with no body, hundreds of metres away, was being
        // distance-tested against every walker on screen, every step.
        if (a.far()) continue;
        carPos.push_back(a.pos);
        carSpeed.push_back(a.speed);
    }
    // Every person (other walkers + the on-foot player) for 360° SEPARATION —
    // spatial awareness beyond the planner's forward cone, so a body squeezes
    // AROUND a neighbour instead of clumping against it or brushing the player.
    std::vector<Vec2> people;
    std::vector<Entity> peopleOwner;
    world.each<Transform, CharacterController>([&](Entity e, Transform& t, CharacterController&) {
        people.push_back(Vec2(t.position.x, t.position.z));
        peopleOwner.push_back(e);
    });

    std::vector<CityRenderSystem::ExternalAgentPose> widgetPoses;
    widgetPoses.reserve(walkers_.size());

    for (Walker& w : walkers_) {
        if (!world.alive(w.entity)) continue;
        CharacterController* cc = world.get<CharacterController>(w.entity);
        Transform* t = world.get<Transform>(w.entity);
        if (!t) continue;
        if (w.agentId < 0 || w.agentId >= static_cast<int>(sim.agents().size())) continue;
        const Agent& g = sim.agents()[w.agentId];

        // SITTING (the furniture library, M5): the walker is drawn seated on its bench or chair -- hip on the seat,
        // facing out -- and its capsule parks inside the seat, out of everyone's way, until it gets up.
        // ...and on the grass (a picnic), or lying on it (in the sun): the same, on the ground under it
        CitySim::RestPose rp;
        bool restNow = sim.restPose(w.agentId, rp);
        // A BODY sits down only once it has got there: its plan reaches the seat first, and drawing it on the seat
        // at once snapped it the last metres (the watchdog: body -> resting at 40-170 m/s). Until then it walks on.
        if (restNow && cc && cc->characterId != engine::INVALID_CHARACTER) {
            const Vec3 bp = pw.characterPosition(cc->characterId);
            if ((Vec2(bp.x, bp.z) - rp.pos).length() > 0.6 && w.drawnPath == 2) restNow = false;
        }
        if (restNow) {
            city_.simMutable().clearAgentTether(w.agentId);
            const Real gy = rp.kind == CitySim::RestPose::Kind::Seat ? rp.hip - 0.45 : city_.groundHeightAt(rp.pos.x, rp.pos.y);
            if (cc && cc->characterId != engine::INVALID_CHARACTER)
                pw.setCharacterPosition(cc->characterId, Vec3(rp.pos.x, gy + kCapsuleHalf + kCapsuleRadius, rp.pos.y));
            const Quat yaw = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(rp.face.x, rp.face.y));
            engine::MeshHandle mesh;
            if (rp.kind == CitySim::RestPose::Kind::Seat) {
                auto it = seatedMeshes_.find(w.outfit);
                if (it == seatedMeshes_.end())
                    it = seatedMeshes_.emplace(w.outfit, ctx.assets.acquireMesh(buildSeatedPersonMesh(w.outfit),
                                                                               "citywalk:seated" + std::to_string(w.outfit))).first;
                mesh = it->second;
                t->position = Vec3(rp.pos.x, rp.hip, rp.pos.y);
                t->orientation = yaw;
            } else if (rp.kind == CitySim::RestPose::Kind::SitGround) {
                auto it = groundSitMeshes_.find(w.outfit);
                if (it == groundSitMeshes_.end())
                    it = groundSitMeshes_.emplace(w.outfit, ctx.assets.acquireMesh(buildGroundSittingPersonMesh(w.outfit),
                                                                                  "citywalk:groundsit" + std::to_string(w.outfit))).first;
                mesh = it->second;
                t->position = Vec3(rp.pos.x, gy + kGroundSitHip, rp.pos.y);
                t->orientation = yaw;
            } else {   // lying on its back, feet the way it faces
                mesh = poseMesh(ctx.assets, w.outfit, 0);
                t->position = Vec3(rp.pos.x, gy + kLieHalfDepth, rp.pos.y);
                t->orientation = yaw * Quat::fromAxisAngle(Vec3(1, 0, 0), -engine::PI * 0.5);
            }
            if (Renderable* r = world.get<Renderable>(w.entity)) r->mesh = mesh;
            w.facing = rp.face;
            if (PrevTransform* pt = world.get<PrevTransform>(w.entity)) pt->value = *t;
            w.haveLast = false;
            watchDrawn(w, Vec2(t->position.x, t->position.z), 0, dt, sim);
            continue;
        }

        // BODYLESS (outside the physical budget): no capsule to step, nothing
        // to separate, nobody to knock down. Just wear the ghost's pose and
        // keep walking — visually identical, and the entire expensive tail of
        // this loop is skipped.
        if (!cc || cc->characterId == engine::INVALID_CHARACTER) {
            // NO BODY, NO LEASH. The tether is fed only from down in the
            // physical path below, so a walker that loses its capsule to the
            // body budget keeps the last anchor it was given — and the moment
            // its plan walks more than the lead from that dead point, the
            // stepper stops advancing it. It then stands at exactly `lead`
            // metres for the rest of the day, animating, with everyone behind
            // it queued at minGap. That is what "a bunch of guys stuck on a
            // fence" actually was: not the fence, the budget.
            city_.simMutable().clearAgentTether(w.agentId);
            Vec2 hd;
            Vec2 dp = city_.drawnAgentPos(w.agentId, &hd);   // through the tick, not hopping at it
            // A BODY JUST GIVEN UP (the nearest-bodies budget): it was a metre or two behind its plan; ease that away
            // over half a second instead of jumping to the plan (the watchdog: body -> posed at ~90 m/s)
            if (w.drawnPath == 2 && w.drawnValid) w.handoff = w.drawnLast - dp;
            if (w.drawnPath != 2 && w.drawnPath != 1) w.handoff = Vec2(0, 0);
            {
                const Real L = w.handoff.length(), ease = 3.0 * dt;
                w.handoff = L > ease ? w.handoff * ((L - ease) / L) : Vec2(0, 0);
                dp = dp + w.handoff;
            }
            t->position = Vec3(dp.x, city_.groundHeightAt(dp.x, dp.y) + kCapsuleHalf + kCapsuleRadius, dp.y);
            if (g.speed > 0.05) w.facing = hd;
            t->orientation = Quat::fromAxisAngle(
                Vec3(0, 1, 0), std::atan2(w.facing.x, w.facing.y));
            const int pose = walkPoseIndex(w.stride, g.speed, dt);
            if (Renderable* r = world.get<Renderable>(w.entity))
                r->mesh = poseMesh(ctx.assets, w.outfit, pose);
            watchDrawn(w, Vec2(t->position.x, t->position.z), 1, dt, sim);
            continue;
        }

        Vec3 pos = pw.characterPosition(cc->characterId);
        Vec2 posXZ(pos.x, pos.z);

        // Knockdown: a vehicle moving through the walker's body floors it.
        for (std::size_t c = 0; c < carPos.size(); ++c) {
            Vec2 d = carPos[c] - posXZ;
            if (d.length() < kKnockRadius && carSpeed[c] > kKnockSpeed) {
                w.knock.knock();
                break;
            }
        }
        bool down = w.knock.update(dt);

        // Body speed BEFORE this tick's move (the blocked watchdog's input).
        Vec3 velBefore = pw.characterVelocity(cc->characterId);
        Real hBefore = std::sqrt(velBefore.x * velBefore.x + velBefore.z * velBefore.z);

        // Walk toward the ghost's planned spot (station control, walker-simple):
        // speed grows with the distance behind the plan, capped at a brisk jog;
        // settles at a standoff. Plus 360° SEPARATION from nearby people — the
        // spatial awareness a forward cone lacks — so a walker squeezes around a
        // neighbour or the player instead of pressing into them.
        // Down (knocked): no intent. Backing off (blocked): stand still and let
        // the think cadence + tethered plan re-route before trying again.
        Vec3 desired;
        bool backingOff = w.backoff > 0;
        if (backingOff) w.backoff -= dt;
        if (!down && !backingOff) {
            Vec2 steer(0, 0);
            Vec2 to(g.pos.x - posXZ.x, g.pos.y - posXZ.y);
            Real d = to.length();
            if (d > kWalkStandoff) {
                Real speed = std::min(kWalkMax, kWalkGain * (d - kWalkStandoff));
                steer = Vec2(to.x / d * speed, to.y / d * speed);
            }
            for (std::size_t p = 0; p < people.size(); ++p) {
                if (peopleOwner[p] == w.entity) continue;
                Vec2 away = posXZ - people[p];
                Real ad = away.length();
                if (ad > 1e-4 && ad < kPersonalSpace) {
                    Real push = kSeparationGain * (kPersonalSpace - ad) / kPersonalSpace;
                    steer = steer + away * (push / ad);
                }
            }
            Real sl = steer.length();
            if (sl > kWalkMax) steer = steer * (kWalkMax / sl);
            desired = Vec3(steer.x, 0, steer.y);
            // Wants to walk but the body is pinned (crowd, parked car, wall):
            // stop trying for a beat — a standing person reads as "waiting", a
            // pinned person shoving reads as broken.
            Real commanded = std::sqrt(desired.x * desired.x + desired.z * desired.z);
            if (w.blocked.update(commanded, hBefore, dt)) {
                w.backoff = kBackoffTime;
                desired = Vec3();
            }
        }
        const Vec2 before(pos.x, pos.z);
        pw.moveCharacter(cc->characterId, desired, dt);   // zero intent still settles
        pos = pw.characterPosition(cc->characterId);
        posXZ = Vec2(pos.x, pos.z);
        // JITTER TELEMETRY (`walkers?`; Glenn, 2026-10-02: "npcs jittering about at superspeed"): the body's real
        // speed this step, and whether it reversed against the last one -- a twitching body flips every step.
        {
            const Vec2 v = (posXZ - before) * (1.0 / std::max(dt, Real(1e-6)));
            const Real sp = v.length();
            if (sp > tel_.maxSpeed) { tel_.maxSpeed = sp; tel_.maxAt = posXZ; tel_.maxAgent = w.agentId; }
            if (w.haveLast && sp > 0.8 && w.lastStep.length() > 0.8 && dot(v, w.lastStep) < 0) {
                ++tel_.reversals;
                tel_.revAt = posXZ;
            }
            w.lastStep = v;
            w.haveLast = true;
            ++tel_.steps;
        }

        // Face the actual travel direction once really moving.
        Vec3 vel = pw.characterVelocity(cc->characterId);
        Real hSpeed = std::sqrt(vel.x * vel.x + vel.z * vel.z);
        if (hSpeed > kFaceSpeed) {
            w.facing = Vec2(vel.x / hSpeed, vel.z / hSpeed);
        }

        // WALK CYCLE: the stride phase advances with the body's REAL speed, so
        // the legs move exactly as fast as the ground goes by — a held body
        // stands (pose 0), a knocked-down one keeps whatever it fell in.
        if (!down) {
            int pose = walkPoseIndex(w.stride, hSpeed, dt);
            if (Renderable* r = world.get<Renderable>(w.entity))
                r->mesh = poseMesh(ctx.assets, w.outfit, pose);
        }

        // Pose write-back. Standing: the box rides the capsule centre. Down: lay
        // the box flat at shin height, pitched forward over its facing (the
        // capsule itself stays standing — a v1 visual; ragdoll is future work).
        if (PrevTransform* pt = world.get<PrevTransform>(w.entity)) pt->value = *t;
        Real yaw = std::atan2(w.facing.x, w.facing.y);
        Quat yawQ = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
        if (down) {
            t->position = Vec3(pos.x, pos.y - kCapsuleHalf, pos.z);
            t->orientation = yawQ * Quat::fromAxisAngle(Vec3(1, 0, 0), engine::PI * 0.5);
        } else {
            t->position = pos;
            t->orientation = yawQ;
        }

        // THE LEASH NEEDS A WAY OUT. The tether holds the plan back until the
        // body catches up (ADR-0062), which is right until the body CANNOT
        // catch up: wedged on a guardrail, the two deadlock for ever — the
        // ghost is not stepped because it leads, the body cannot move because
        // it is stuck, and neither side ever yields. Measured on metro: the
        // same agent at the same coordinates held at anchorDist 5.0 against a
        // 5.0 m lead in two runs forty minutes apart, with a queue of walkers
        // stacked behind it, and Glenn watching three of them walk into a
        // fence. After kTetherGiveUp seconds of no progress the PLAN wins and
        // the body is moved to it. A visible step is less bad than a person who
        // stands in the road for the rest of the day.
        const Real lead = std::sqrt((g.pos.x - posXZ.x) * (g.pos.x - posXZ.x) +
                                    (g.pos.y - posXZ.y) * (g.pos.y - posXZ.y));
        // Held means NOT CLOSING, not "not moving". A body wedged against a
        // barrier keeps sliding along it at a few cm/s, so a speed test never
        // fires: measured zero unwedges across a whole session while walkers
        // stood at a fence in plain sight. What matters is whether the gap to
        // its own plan is coming down.
        if (!down && lead > kTetherLead &&
            (w.lastLead < 0 || lead > w.lastLead - kClosingRate * dt)) {
            w.heldFor += dt;
            if (w.heldFor >= kTetherGiveUp) {
                pw.setCharacterPosition(
                    cc->characterId,
                    Vec3(g.pos.x,
                         city_.groundHeightAt(g.pos.x, g.pos.y) + kCapsuleHalf +
                             kCapsuleRadius,
                         g.pos.y));
                pos = pw.characterPosition(cc->characterId);
                posXZ = Vec2(pos.x, pos.z);
                t->position = pos;
                w.heldFor = 0;
                w.backoff = 0;
                // Say so, thinly. Each one of these is a place where the
                // pedestrian network runs through something solid, so the
                // positions are the bug report; the thinning keeps a bad level
                // from drowning the log in them.
                if (unwedged_ < 10 || unwedged_ % 100 == 0)
                    LOG_INFO << "[walkers] unwedged agent " << w.agentId << " at "
                             << posXZ.x << "," << posXZ.y << " (total "
                             << (unwedged_ + 1) << ")";
                ++unwedged_;
            }
        } else {
            w.heldFor = 0;
        }
        w.lastLead = lead;
        watchDrawn(w, posXZ, 2, dt, sim);

        // The plan waits for the body (never outruns a blocked/downed walker),
        // and the debug widgets ring the REAL walker.
        city_.simMutable().setAgentTether(w.agentId, posXZ, kTetherLead);
        // Body truth for the debug ring: a downed or blocked walker shows RED
        // whatever its planner ghost believes.
        int stateOverride = (down || w.backoff > 0)
                                ? static_cast<int>(Agent::State::Waiting) : -1;
        widgetPoses.push_back(CityRenderSystem::ExternalAgentPose{
            w.agentId, posXZ, w.facing, g.pos, stateOverride});
    }
    city_.setExternalPedPoses(std::move(widgetPoses));
}

void CityWalkerSystem::fixedUpdate(engine::FrameContext& ctx) {
    spawnWalkers(ctx);
    driveWalkers(ctx);
    // `who? <id>`: one agent's whole state, answered every step while asked (poll `who?`); its drawn body too
    {
        const std::string q = ctx.settings.getString("who.query", "");
        if (!q.empty()) {
            const int id = std::atoi(q.c_str());
            std::string r = city_.sim().describeAgent(id);
            for (const Walker& w : walkers_)
                if (w.agentId == id) {
                    char b[160];
                    std::snprintf(b, sizeof b, " | drawn at %.1f %.1f as %s", w.drawnLast.x, w.drawnLast.y,
                                  w.drawnPath == 0 ? "resting" : w.drawnPath == 1 ? "posed from the plan" : w.drawnPath == 2 ? "a body" : "?");
                    r += b;
                }
            ctx.settings.setString("who.result", q + ": " + r);
        }
    }
    // Publish the jitter telemetry every 2 s of sim, then start a fresh window.
    tel_.window += ctx.clock.fixedStep();
    if (tel_.window >= 2.0) {
        // ...and who is sitting (M5): how many, and the one nearest the player's bubble centre
        const CitySim& sm = city_.sim();
        int seated = 0;
        Vec2 seatAt(0, 0);
        Real bestD = 1e30;
        for (int i = 0; i < static_cast<int>(sm.agents().size()); ++i)
            if (const CitySim::SeatSpot* st = sm.seatedOn(i)) {
                ++seated;
                const Real d = (st->pos - sm.tierCenter()).length();
                if (d < bestD) { bestD = d; seatAt = st->pos; }
            }
        // ...and how many are walking a park's or the quad's paths (footpaths), and students among them
        int onWalks = 0, studentsOnWalks = 0, joggers = 0;
        for (int i = 0; i < static_cast<int>(sm.agents().size()); ++i)
            if (const CitySim::ActivitySpot* sp = sm.usingSpot(i)) joggers += sp->kind == SpotKind::Jog ? 1 : 0;
        int playing = 0, games = 0, gathering = 0, signedUp = 0;   // a group's players on its area, and the games on
        for (const CitySim::Session& se : sm.sessions()) {
            if (se.state == CitySim::Session::State::Gathering) { ++gathering; signedUp += static_cast<int>(se.members.size()); }
            if (se.state != CitySim::Session::State::Running) continue;
            ++games;
            for (int m : se.members) playing += sm.agents()[static_cast<std::size_t>(m)].seatPhase == 2 ? 1 : 0;
        }
        int jogBooked = 0;   // runners on their way: jog spots held
        for (const CitySim::ActivitySpot& sp : sm.spots()) jogBooked += sp.kind == SpotKind::Jog && sp.occupant >= 0 ? 1 : 0;
        const engine::NavGraph& ng = sm.nav();
        for (const Agent& a : sm.agents()) {
            if (!a.moving || a.leg < 0 || a.leg >= static_cast<int>(a.route.links.size())) continue;
            const int li = a.route.links[static_cast<std::size_t>(a.leg)];
            if (li < 0 || li >= ng.linkCount() || !ng.links[static_cast<std::size_t>(li)].footpath) continue;
            ++onWalks;
            studentsOnWalks += a.role == Agent::Role::Student ? 1 : 0;
            if (onWalks == 1 && std::getenv("RT_WALK_DEBUG"))
                std::fprintf(stderr, "[walk debug] agent %d at %.1f %.1f link %d leg %d/%zu dist %.2f of %.2f speed %.2f far %d tier %d\n",
                             static_cast<int>(&a - sm.agents().data()), a.pos.x, a.pos.y, li, a.leg, a.route.links.size(), a.distOnLeg,
                             ng.links[static_cast<std::size_t>(li)].length, a.speed, a.far() ? 1 : 0, static_cast<int>(a.tier));
            if (onWalks == 1 && std::getenv("RT_WALK_DEBUG")) {
                const engine::NavLink& L0 = ng.links[static_cast<std::size_t>(li)];
                const int ai = static_cast<int>(&a - sm.agents().data());
                std::fprintf(stderr, "[walk debug]   gap %.2f minGap %.2f bodyLag %.2f tethered %d\n", sm.debugGap(ai), sm.debugMinGap(ai), a.bodyLag, a.tethered ? 1 : 0);
                std::fprintf(stderr, "[walk debug]   state %d hold %.1f riding %d awaiting %d seatPhase %d playerControlled %d | link %d: %.1f %.1f -> %.1f %.1f (nodes %d -> %d, street nodes < %d); next:",
                             static_cast<int>(a.state), a.holdTimer, sm.riding(static_cast<int>(&a - sm.agents().data())) ? 1 : 0,
                             sm.awaitingRide(static_cast<int>(&a - sm.agents().data())) ? 1 : 0, a.seatPhase, a.playerControlled ? 1 : 0,
                             li, L0.footA.x, L0.footA.y, L0.footB.x, L0.footB.y, L0.from, L0.to, ng.streetNodeCount());
                for (std::size_t q = 1; q < std::min<std::size_t>(5, a.route.links.size()); ++q) {
                    const engine::NavLink& Lq = ng.links[static_cast<std::size_t>(a.route.links[q])];
                    std::fprintf(stderr, " [%s %.1f %.1f]", Lq.footpath ? "walk" : "street", Lq.footB.x, Lq.footB.y);
                }
                std::fprintf(stderr, "\n");
            }
        }
        char b[480];
        std::snprintf(b, sizeof b, "walkers %zu max %.1f m/s (agent %d at %.0f %.0f) reversals %ld of %ld steps (last at %.0f %.0f) | seated %d (nearest at %.1f %.1f) | indoors drawn %d | students %d | on walks %d (students %d) | jogging %d (booked %d) | games %d (playing %d) gathering %d (%d coming)",
                      walkers_.size(), tel_.maxSpeed, tel_.maxAgent, tel_.maxAt.x, tel_.maxAt.y, tel_.reversals,
                      tel_.steps, tel_.revAt.x, tel_.revAt.y, seated, seatAt.x, seatAt.y, city_.indoorDrawn(), sm.studentCount(),
                      onWalks, studentsOnWalks, joggers, jogBooked, games, playing, gathering, signedUp);
        ctx.settings.setString("walkers.telemetry", b);
        {   // the flicker watch: the count, the latest few (newest last); old departures forgotten
            std::string f = std::to_string(flickers_) + " flickers (a body gone and back within 1.5 s)";
            for (const std::string& e : flickerLog_) f += " || " + e;
            ctx.settings.setString("walkers.flicker", f);
            const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            for (auto it = gone_.begin(); it != gone_.end();) it = now - it->second.at > 2.0 ? gone_.erase(it) : std::next(it);
        }
        // WHAT EVERYONE IS DOING (`activities?`): per activity in the catalog, how many are at it and how many on the
        // way; its groups going and the nearest one to the bubble's centre (where to go and look)
        {
            const ActivityCatalog& cat = sm.activityCatalog();
            std::vector<int> at(cat.defs.size(), 0), going(cat.defs.size(), 0), groups(cat.defs.size(), 0);
            std::vector<Real> nearD(cat.defs.size(), 1e30);
            std::vector<Vec2> nearAt(cat.defs.size());
            for (const Agent& a : sm.agents()) {
                if (a.moving) {
                    if (a.tripActivity >= 0 && a.tripActivity < static_cast<int>(cat.defs.size()))
                        ++going[static_cast<std::size_t>(a.tripActivity)];
                    continue;
                }
                if (a.atActivity < 0 || a.atActivity >= static_cast<int>(cat.defs.size())) continue;
                const std::size_t d = static_cast<std::size_t>(a.atActivity);
                ++at[d];
                const Real dd = (a.pos - sm.tierCenter()).lengthSquared();
                if (dd < nearD[d]) { nearD[d] = dd; nearAt[d] = a.pos; }
            }
            for (const CitySim::Session& se : sm.sessions())
                if (se.state != CitySim::Session::State::Dead && se.def >= 0 && se.def < static_cast<int>(cat.defs.size()))
                    ++groups[static_cast<std::size_t>(se.def)];
            std::string out;
            char line[200];
            for (std::size_t d = 0; d < cat.defs.size(); ++d) {
                if (!at[d] && !going[d] && !groups[d]) continue;
                std::snprintf(line, sizeof line, "%s%s %d (+%d coming)", out.empty() ? "" : " | ", cat.defs[d].name.c_str(), at[d], going[d]);
                out += line;
                if (groups[d]) { std::snprintf(line, sizeof line, " %d groups", groups[d]); out += line; }
                if (at[d]) { std::snprintf(line, sizeof line, " nearest %.0f %.0f", nearAt[d].x, nearAt[d].y); out += line; }
            }
            ctx.settings.setString("activities.telemetry", out.empty() ? "nobody out" : out);
        }
        // THE SPEEDERS (`speeders?`): how many drawn steps went past a sprint, and the latest few, each with the
        // agent's state at that moment and how it was drawn before and after (0 resting, 1 posed from the plan, 2 body)
        {
            std::string out;
            char line[200];
            std::snprintf(line, sizeof line, "%ld steps past 5 m/s since start; sim ticking at 1/%d of its rate (adaptive load)",
                          speedEventCount_, city_.loadMultiplier());
            out += line;
            const std::size_t n = speedEvents_.size();
            for (std::size_t k = n > 6 ? n - 6 : 0; k < n; ++k) {
                const SpeedEvent& e = speedEvents_[k];
                std::snprintf(line, sizeof line, " || %.1f s: %.1f m/s %.1f %.1f -> %.1f %.1f drawn %d->%d: ", e.at, e.speed, e.from.x,
                              e.from.y, e.to.x, e.to.y, e.pathFrom, e.pathTo);
                out += line;
                out += e.state;
            }
            ctx.settings.setString("speeders.telemetry", out);
        }
        tel_ = Telemetry{};
    }
}

void CityWalkerSystem::onStop(engine::FrameContext&) {
    // Tracking only; world/physics teardown reclaims entities + characters (the
    // same no-removal-hook note as the vehicle bridge — fine at level teardown).
    walkers_.clear();
    poseMeshes_.clear();
    seatedMeshes_.clear();
    groundSitMeshes_.clear();
}

}  // namespace citysim
