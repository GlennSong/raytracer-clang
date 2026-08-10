#include "city_physics.h"

#include "driver_control.h"

#include "../../engine/components.h"   // InstanceGroup
#include "../../engine/world.h"
#include "../../profile.h"

#include <cmath>

namespace citysim {

using engine::Vec3;
using engine::Quat;
using engine::Mat4;
using engine::Real;
using engine::Entity;
using engine::World;
using engine::InstanceGroup;
using engine::PhysicsWorld;
using engine::PhysicsBodyId;
using engine::BodyMotion;

void CityPhysicsSystem::releaseBodies() {
    PhysicsWorld& pw = physics_.physicsWorld();
    for (const auto& kv : carProxies_) pw.removeBody(kv.second.id);
    for (const auto& kv : pedProxies_) pw.removeBody(kv.second.id);
    for (PhysicsBodyId id : poleBodies_) pw.removeBody(id);
    carProxies_.clear();
    pedProxies_.clear();
    poleBodies_.clear();
    polesBuilt_ = false;
}

// Keep `proxies` matched to every drawn instance across `groups` as an
// incremental DIFF (P4.3): each instance keys its body by the baking agent's
// stable uid (scenery cars by their fixed bake ordinal), so a tier promotion
// adds ONE body, a demotion removes ONE, and everything else just moves —
// the old positional pool tore down and rebuilt every box whenever the drawn
// count changed, which tier churn would have turned into per-step O(n) churn.
void CityPhysicsSystem::syncKinematic(World& world, const std::vector<Entity>& groups,
                                      const std::vector<Vec3>& groupExtents,
                                      std::unordered_map<long long, ProxyBody>& proxies,
                                      Real dt,
                                      const std::vector<std::vector<int>>* agentIds) {
    RT_PROFILE_ZONE_NAMED("syncKinematic");
    PhysicsWorld& pw = physics_.physicsWorld();
    const std::vector<Agent>& agents = city_.sim().agents();
    const uint32_t stamp = ++proxyStamp_;
    long long sceneryKey = -1;   // scenery bake order is fixed for the run
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
        InstanceGroup* g = world.get<InstanceGroup>(groups[gi]);
        if (!g) continue;
        const Vec3& he = groupExtents[gi < groupExtents.size() ? gi : groupExtents.size() - 1];
        for (std::size_t ii = 0; ii < g->transforms.size(); ++ii) {
            int ai = -1;
            if (agentIds && gi < agentIds->size() && ii < (*agentIds)[gi].size())
                ai = (*agentIds)[gi][ii];
            // Proxy identity. An agent keys on its uid. A SCENERY car keys on
            // the id the bake hands over (-2 - bayIndex): stable across frames,
            // which matters now that scenery is distance-culled — bake ORDER
            // shifts as the player moves, so an order-derived key would shuffle
            // every parked car's proxy each step. -1 (unknown) keeps the old
            // sequential fallback.
            const long long key =
                (ai >= 0 && ai < static_cast<int>(agents.size()))
                    ? static_cast<long long>(agents[ai].uid)
                    : (ai <= -2 ? static_cast<long long>(ai) : sceneryKey--);
            bool isParked = false;
            if (ai >= 0)
                for (const Possessed& p : possessed_)
                    if (p.agent == ai) { isParked = true; break; }
            const Mat4& m = g->transforms[ii];
            Vec3 p(m.m[0][3], m.m[1][3], m.m[2][3]);
            if (isParked) p.y = -1000.0;   // possessed: box parks below the world
            Real yaw = std::atan2(m.m[0][2], m.m[2][2]);   // heading from local +Z basis
            const Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
            auto it = proxies.find(key);
            if (it == proxies.end()) {
                // Newly drawn (first frame, or a V agent promoted into the K
                // bubble): SPAWN the box at the drawn pose — like park/unpark
                // below, arriving is a discontinuous move, never a sweep.
                ProxyBody nb;
                nb.id = pw.addBox(he, p, rot, BodyMotion::Kinematic,
                                  /*restitution*/ 0.0, /*friction*/ 0.6);
                nb.parked = isParked ? 1 : 0;
                nb.stamp = stamp;
                proxies.emplace(key, nb);
                continue;
            }
            ProxyBody& b = it->second;
            b.stamp = stamp;
            const bool wasParked = b.parked != 0;
            // Park/unpark is a DISCONTINUOUS move: teleport, never sweep — a
            // MoveKinematic across the jump is a one-frame hypersonic sweep
            // straight through the freshly spawned chassis (measured: it batted
            // cars 1.7 km across the map).
            if (isParked != wasParked) pw.teleport(b.id, p, rot);
            else if (!isParked) pw.moveKinematic(b.id, p, rot, dt);
            b.parked = isParked ? 1 : 0;
        }
    }
    // Instances gone this pass (tier demotion, a released driver): drop their
    // bodies. Keys sorted so removal order reproduces run to run (ADR-0002).
    staleScratch_.clear();
    for (const auto& kv : proxies)
        if (kv.second.stamp != stamp) staleScratch_.push_back(kv.first);
    std::sort(staleScratch_.begin(), staleScratch_.end());
    for (long long k : staleScratch_) {
        auto it = proxies.find(k);
        pw.removeBody(it->second.id);
        proxies.erase(it);
    }
}

void CityPhysicsSystem::fixedUpdate(engine::FrameContext& ctx) {
    if (!city_.built()) return;
    step(ctx.world, ctx.clock.fixedStep());
}

// Re-stamp every possessed car's DRAWN transform from its body, in place, right
// after the tier has run (#26: "I can walk through cars"). The bake happens once
// per step, before this system runs, so a possessed car was always drawn one
// frame behind its chassis — and on the frame the tier's backstop SNAPS a lost
// body onto its ghost, that is a 10 m gap between where the car is drawn and
// where the only solid thing standing in for it actually is (a possessed car's
// kinematic proxy parks below the world, so the chassis is all there is).
// Writing the pose here closes the gap within the frame and leaves the proxy
// sync below reading fresh poses.
void CityPhysicsSystem::syncPossessedInstances(World& world) {
    if (possessed_.empty()) return;
    PhysicsWorld& pw = physics_.physicsWorld();
    const std::vector<engine::Entity>& groups = city_.carGroups();
    const std::vector<std::vector<int>>& ids = city_.carAgentIds();
    for (std::size_t gi = 0; gi < groups.size() && gi < ids.size(); ++gi) {
        InstanceGroup* g = world.get<InstanceGroup>(groups[gi]);
        if (!g) continue;
        for (std::size_t ii = 0; ii < ids[gi].size() && ii < g->transforms.size();
             ++ii) {
            const int aid = ids[gi][ii];
            if (aid < 0) continue;   // scenery parked car, no agent
            for (const Possessed& p : possessed_) {
                if (p.agent != aid) continue;
                const Vec3 bp = pw.vehiclePosition(p.vid);
                const Quat bq = pw.vehicleOrientation(p.vid);
                const Mat4 pose =
                    Mat4::translate(bp.x, bp.y, bp.z) * bq.toMat4();
                g->transforms[ii] = pose;
                city_.setAgentPhysPose(aid, pose);
                break;
            }
        }
    }
}

void CityPhysicsSystem::step(World& world, Real dt) {
    // Cars + pedestrians: kinematic bodies that track the drawn poses so the
    // player and the physics gun collide with them.
    possessTier(world, dt);   // R5: physical tier drives + parks proxies BEFORE the box sync
    syncPossessedInstances(world);   // drawn car == its body, this frame
    syncKinematic(world, city_.carGroups(), city_.carGroupHalfExtents(), carProxies_, dt,
                  &city_.carAgentIds());
    { std::vector<Entity> peds{ city_.pedGroup() };
      std::vector<Vec3> pedExtents{ city_.pedHalfExtent() };
      syncKinematic(world, peds, pedExtents, pedProxies_, dt, &city_.pedAgentIds()); }

    // Signal poles are static: one thin tall box per pole, built once (they never
    // move). The pole foot is the instance translation; the mast rises +Y.
    if (!polesBuilt_) {
        InstanceGroup* posts = world.get<InstanceGroup>(city_.signalPostGroup());
        if (posts) {
            PhysicsWorld& pw = physics_.physicsWorld();
            const Real poleH = 5.6;                       // = street_kit SignalParams.poleHeight
            const Vec3 he(0.16, poleH * 0.5, 0.16);
            for (const Mat4& m : posts->transforms) {
                Vec3 foot(m.m[0][3], m.m[1][3], m.m[2][3]);
                Vec3 c = foot + Vec3(0, poleH * 0.5, 0);
                poleBodies_.push_back(pw.addBox(he, c, Quat(), BodyMotion::Static));
            }
            polesBuilt_ = true;
        }
    }
}

void CityPhysicsSystem::possessTier(World& world, Real dt) {
    RT_PROFILE_ZONE_NAMED("possessTier");
    PhysicsWorld& pw = physics_.physicsWorld();
    // The tier centres on the PLAYER — found by ControlledBy, not by "the first
    // entity with a CharacterController". Every walker has one of those, so the
    // old scan could centre the possession ring on an arbitrary pedestrian
    // depending on component-pool order.
    Vec3 centre(0, 0, 0);
    bool haveCentre = false;
    world.each<engine::Transform, engine::ControlledBy>(
        [&](engine::Entity, engine::Transform& t, engine::ControlledBy&) {
            if (haveCentre) return;
            centre = t.position;
            haveCentre = true;
        });
    // Fallback for hosts that stand in a bare character for the player (the
    // headless tier tests do exactly this). In a real level the player carries
    // ControlledBy, so the walker crowd can never win this.
    if (!haveCentre)
        world.each<engine::CharacterController>(
            [&](engine::Entity e, engine::CharacterController&) {
                if (haveCentre) return;
                if (const engine::Transform* t = world.get<engine::Transform>(e)) {
                    centre = t->position;
                    haveCentre = true;
                }
            });
    const auto& agents = city_.sim().agents();
    if (!haveCentre || agents.empty()) return;

    auto dist2 = [&](const Agent& a) {
        const Real dx = a.pos.x - centre.x, dz = a.pos.y - centre.z;
        return dx * dx + dz * dz;
    };
    // Drop: gone, stopped being a driver, or left the hysteresis ring.
    const Real dropR2 = possessRadius_ * 1.3 * possessRadius_ * 1.3;
    for (std::size_t i = 0; i < possessed_.size();) {
        const int ai = possessed_[i].agent;
        const bool keep = ai >= 0 && ai < static_cast<int>(agents.size()) &&
                          agents[ai].mode == Agent::Mode::Driver &&
                          agents[ai].moving && !agents[ai].released &&
                          dist2(agents[ai]) < dropR2;
        if (!keep) {
            city_.clearAgentPhysPose(ai);
            pw.removeVehicle(possessed_[i].vid);
            possessed_[i] = possessed_.back();
            possessed_.pop_back();
        } else {
            ++i;
        }
    }
    // Acquire: nearest moving drivers inside the ring, up to the budget.
    if (static_cast<int>(possessed_.size()) < maxPhysical_) {
        const Real inR2 = possessRadius_ * possessRadius_;
        std::vector<std::pair<Real, int>> cand;
        // Grid candidates (P4.1) instead of the full agent scan — ascending,
        // and the exact radius/state filters below are unchanged, so the same
        // drivers are picked in the same order.
        city_.sim().queryAgentsNear(engine::Vec2(centre.x, centre.z),
                                    possessRadius_ + 8.0, nearScratch_);
        for (int aidx : nearScratch_) {
            const std::size_t ai = static_cast<std::size_t>(aidx);
            const Agent& a = agents[ai];
            if (a.mode != Agent::Mode::Driver || !a.moving || a.released)
                continue;
            if (a.far()) continue;   // far tier: nothing to possess
            bool already = false;
            for (const Possessed& p : possessed_)
                if (p.agent == static_cast<int>(ai)) { already = true; break; }
            if (already) continue;
            const Real d2 = dist2(a);
            if (d2 < inR2) cand.push_back({ d2, static_cast<int>(ai) });
        }
        std::sort(cand.begin(), cand.end());
        for (const auto& [d2, ai] : cand) {
            if (static_cast<int>(possessed_.size()) >= maxPhysical_) break;
            const Agent& a = agents[ai];
            PhysicsWorld::VehicleConfig cfg;   // ambient sedan
            cfg.chassisHalfExtent = Vec3(0.92, 0.45, 2.15);
            cfg.mass = 1400.0;
            cfg.comOffsetY = -0.42;
            cfg.engineTorque = 900.0;   // matches the lab sedan: the body must
                                        // out-accelerate the brains' 4 m/s^2
            auto wheel = [&](Real x, Real z, bool steered, bool driven) {
                PhysicsWorld::VehicleWheel w;
                w.position = Vec3(x, -0.35, z);
                w.radius = 0.34;
                w.width = 0.24;
                w.steered = steered;
                w.driven = driven;
                return w;
            };
            // AWD like the lab sedan: the launch is traction-limited and
            // rear drive alone cannot follow the brains' 4 m/s^2 plan.
            cfg.wheels = { wheel(-0.82, 1.45, true, true),
                           wheel(0.82, 1.45, true, true),
                           wheel(-0.82, -1.42, false, true),
                           wheel(0.82, -1.42, false, true) };
            const Real yaw = std::atan2(a.heading.x, a.heading.y);
            const Real gy = a.deckY > -1e29
                                ? a.deckY
                                : city_.groundHeightAt(a.pos.x, a.pos.y) +
                                      a.elevation;
            const PhysicsWorld::VehicleId vid = pw.addVehicle(
                cfg, Vec3(a.pos.x, gy + 0.8, a.pos.y),
                Quat::fromAxisAngle(Vec3(0, 1, 0), yaw));
            if (vid != PhysicsWorld::INVALID_VEHICLE) {
                // Velocity-match the ghost's moving frame: spawned at rest
                // under a cruising ghost, the body sags ~10 m before the
                // pedals can close it.
                pw.setLinearVelocity(
                    pw.vehicleBody(vid),
                    Vec3(a.heading.x * a.speed, 0, a.heading.y * a.speed));
                Possessed np;
                np.agent = ai;
                np.vid = vid;
                possessed_.push_back(np);
            }
        }
    }
    // Drive each body at the sim's own plan: chase the kinematic ghost's
    // lane frame and speed — the brains stay tier-invariant.
    for (Possessed& p : possessed_) {
        const Agent& a = agents[p.agent];
        const Vec3 bp = pw.vehiclePosition(p.vid);
        const Quat bq = pw.vehicleOrientation(p.vid);
        const Vec3 bv = pw.vehicleVelocity(p.vid);
        const Vec3 f3 = bq.rotate(Vec3(0, 0, 1));
        const Real fl = std::sqrt(f3.x * f3.x + f3.z * f3.z);
        const engine::Vec2 fwd =
            fl > 1e-6 ? engine::Vec2(f3.x / fl, f3.z / fl) : engine::Vec2(0, 1);
        const Real speed = bv.x * fwd.x + bv.z * fwd.y;
        const engine::Vec2 bpos(bp.x, bp.z);
        const engine::Vec2 toGhost = a.pos - bpos;
        const Real dist = toGhost.length();

        // BACKSTOP: a body that clearly lost its ghost snaps back onto it —
        // the LOD contract is "the drawn car is where the sim says a car
        // is", and past this range chasing looks worse than the cut. The
        // stuck clock catches the PINNED case early (wedged head-on against
        // a queued car's kinematic box in the opposing lane, a signal pole):
        // waiting for the distance fuse alone let the gap grow to 15 m in
        // plain view.
        // The stuck clock runs only when the body is PINNED: slow AND
        // making no progress toward its ghost. Speed alone missed the
        // GRINDER (wedged on a queued box's corner, creeping 1.5 m/s
        // sideways for 20+ s); receding alone false-fired on healthy
        // post-corner recoveries, teleporting cars in plain view.
        const Real bodySpeed = std::sqrt(bv.x * bv.x + bv.z * bv.z);
        const Real closing = (p.lastDist - dist) / std::max(dt, Real(1e-4));
        p.lastDist = dist;
        if (dist > 5.0 && bodySpeed < 2.0 && closing < 0.5)
            p.stuckTime += dt;
        else
            p.stuckTime = 0;
        if (dist > 12.0 || p.stuckTime > 1.5) {
            const Real yaw2 = std::atan2(a.heading.x, a.heading.y);
            const Real gy2 = a.deckY > -1e29
                                 ? a.deckY
                                 : city_.groundHeightAt(a.pos.x, a.pos.y) +
                                       a.elevation;
            pw.teleport(pw.vehicleBody(p.vid),
                        Vec3(a.pos.x, gy2 + 0.8, a.pos.y),
                        Quat::fromAxisAngle(Vec3(0, 1, 0), yaw2));
            pw.setLinearVelocity(
                pw.vehicleBody(p.vid),
                Vec3(a.heading.x * a.speed, 0, a.heading.y * a.speed));
            ++snapCount_;
            p.stuckTime = 0;
            // Move the RENDER pose with the body (#26). Skipping this left the
            // car drawn at the pre-snap spot until the next bake, while its
            // chassis — the only solid thing standing in for it — was already
            // metres away. syncPossessedInstances re-stamps the instance too.
            city_.setAgentPhysPose(
                p.agent, Mat4::translate(a.pos.x, gy2 + 0.8, a.pos.y) *
                             Quat::fromAxisAngle(Vec3(0, 1, 0), yaw2).toMat4());
            continue;
        }

        // PURE PURSUIT at the ghost's forward lead (the lab's arc-clean
        // law): laneA at the body zeroes Stanley's cross-track term, so
        // steering is pure heading error toward the lead point. Tracking
        // the ghost's own (rotating) lane frame instead left the body wide
        // out of every corner, where lock + slip ate its grip.
        // CLOSE + ALIGNED: Stanley on the ghost's lane frame — the cross-
        // track term is what holds sub-metre lane precision (opposing
        // streams pass 2 m apart; pursuit alone converges too lazily and
        // drifted bodies into the oncoming queue's boxes). Otherwise PURE
        // PURSUIT at the ghost, with the lead COLLAPSING when headings
        // disagree (mid-corner): projecting the ghost forward there points
        // ACROSS the junction and the body chords the box diagonally.
        const Real align = fwd.x * a.heading.x + fwd.y * a.heading.y;
        engine::Vec2 laneA, laneB;
        if (align > 0.75) {
            laneA = a.pos;
            laneB = a.pos + a.heading * 8.0;
        } else {
            const Real lead =
                std::clamp(Real(0.6) * std::fabs(speed), Real(2), Real(6)) *
                std::max(Real(0.15), align);
            laneA = bpos;
            laneB = a.pos + a.heading * lead;
        }
        // Rubber-band guard: the body may fall behind through a corner (its
        // real turning circle is wider than the ghost's arc) — give it real
        // catch-up authority; the engine has ~9 m/s^2 at low speed.
        const Real behind = toGhost.x * a.heading.x + toGhost.y * a.heading.y;
        Real target =
            std::max(Real(0), a.speed + std::clamp(behind * 0.5, Real(-3), Real(5)));
        // Follower envelope: never demand more speed than lets the body
        // ease onto a point 1.5 m short of the ghost at the ghost's own
        // speed (v^2 = vGhost^2 + 2a*room). At cruise it bounds the closing
        // rate; at a red it IS the stop-bar law. Pure speed-tracking overran
        // every light by ~4 m — the rendered car parked ON the crosswalk
        // for the whole red while the sim thought it behind the line.
        {
            const Real room = std::max(Real(0), behind - Real(1.5));
            target = std::min(
                target,
                std::sqrt(a.speed * a.speed + 2 * Real(2.6) * room));
        }
        // Corner governor, ANTICIPATORY: cap speed by the pure-pursuit
        // curvature toward the lead point (k = 2 sin(alpha) / L). As the
        // ghost begins an arc the lead swings sideways and the cap bites
        // BEFORE the body reaches the corner — a plain heading-error cap
        // reacted too late, and the catch-up bonus flew the body into
        // junctions at cruise while the ghost arced at 3 m/s (measured:
        // repeated overshoots + slow U-turns at the same corner).
        const engine::Vec2 toLead = laneB - bpos;
        const Real L = toLead.length();
        if (L > 0.5) {
            const Real sinA =
                std::fabs(fwd.x * toLead.y - fwd.y * toLead.x) / L;
            const Real cosA = (fwd.x * toLead.x + fwd.y * toLead.y) / L;
            const Real k = 2.0 * sinA / L;
            if (k > 1e-3)
                target = std::min(
                    target, std::max(Real(2.5), std::sqrt(Real(4.0) / k)));
            if (cosA < 0) target = std::min(target, Real(2.5));   // U-turn crawl
        }
        const DriverInput in =
            driveTowards(bpos, fwd, speed, target, laneA, laneB);
        pw.setVehicleInput(p.vid, in.throttle, in.steer, in.brake);
        city_.setAgentPhysPose(
            p.agent, Mat4::translate(bp.x, bp.y, bp.z) * bq.toMat4());
    }
}

void CityPhysicsSystem::onStop(engine::FrameContext& ctx) {
    for (const Possessed& p : possessed_) {
        city_.clearAgentPhysPose(p.agent);
        physics_.physicsWorld().removeVehicle(p.vid);
    }
    possessed_.clear();
    releaseBodies();
    (void)ctx;
}

}  // namespace citysim
