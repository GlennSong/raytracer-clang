#include "city_player_transit.h"

#include "bus_stop_props.h"   // routeColour / routeColourName
#include "city_render.h"
#include "city_sim.h"

#include "../../engine/camera/fly_camera_controller.h"
#include "../../engine/components.h"
#include "../../engine/input/input_map.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/procgen/city/building_records.h"
#include "../../engine/world.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace citysim {

using engine::Entity;
using engine::Real;
using engine::Transform;
using engine::Vec2;
using engine::Vec3;
using engine::World;

namespace {

// How close you must be to step aboard, and where you sit once you are. A bus
// is 11.4 m long, so "near the bus" is generous; the seat rides a little above
// its pose so the capsule is not buried in the floor.
constexpr Real kBoardRadius = 15.0;   // a bus now stands short of the corner
constexpr Real kSeatLift = 1.1;
constexpr Real kStepOffDistance = 3.4;
// Inside a bus. Reach: how near a door or a free seat must be for E to use it.
// The aisle's half width, walking pace in a moving saloon, and eye heights:
// standing above the floor, seated above the hip. kEyeAboveTransform is
// PlayerSystem's eyeHeight (the eye above the capsule centre it draws from).
constexpr Real kDoorReach = 1.6;
constexpr Real kSeatReach = 1.3;
constexpr Real kAisleHalf = 0.30;
constexpr Real kAisleWalk = 1.6;
constexpr Real kStandingEye = 1.62;
constexpr Real kSeatedEye = 0.74;
constexpr Real kEyeAboveTransform = 0.7;

// "ahead left", "behind" ... from the view direction to a target, both in XZ.
// Eight sectors: finer than that is noise for a walker turning to look.
const char* relativeDirection(Vec2 forward, Vec2 to) {
    const Real fl = std::sqrt(forward.x * forward.x + forward.y * forward.y);
    if (fl < 1e-6) return "";
    const Vec2 f(forward.x / fl, forward.y / fl);
    // right = forward x up = (-f.z, f.x) in XZ, for Y up.
    const Real side = -to.x * f.y + to.y * f.x;
    const Real ahead = to.x * f.x + to.y * f.y;
    const Real deg = std::atan2(side, ahead) * 57.29577951308232;
    static const char* kNames[8] = {"ahead", "ahead right", "right", "behind right",
                                    "behind", "behind left", "left", "ahead left"};
    int sector = static_cast<int>(std::floor((deg + 22.5) / 45.0));
    sector = ((sector % 8) + 8) % 8;
    return kNames[sector];
}

}  // namespace

void CityPlayerTransitSystem::onStart(engine::FrameContext& ctx) {
    // E IS THE INTERACT KEY, and boarding is an interaction. It only acts when
    // a bus or cab is genuinely within reach, so the elevator (which also reads
    // E) is unaffected: a hoistway is indoors and a bus is not, and the two are
    // never both in range. A separate key would have been safer and worse --
    // "press E at the thing" is the convention the level already teaches.
    ctx.actions.bindButton("transit_board", engine::KeyCode::E);
    ctx.actions.setActionContext("transit_board", engine::InputContext::OnFoot);
}

void CityPlayerTransitSystem::fixedUpdate(engine::FrameContext& ctx) {
    RideInput in;
    // `ride bus|taxi|any` boards, `ride off` gets off. Staged as a Setting the
    // way every other control verb reaches a system.
    in.request = ctx.settings.getDouble("player.rideRequest", -1.0);
    if (in.request >= 0.0) ctx.settings.setDouble("player.rideRequest", -1.0);
    // E. Latched in update(): `pressed` is a one-FRAME edge, and a frame can
    // run no fixed step at all, so reading it here dropped presses outright.
    // ElevatorSystem latches its E the same way.
    in.interact = boardEdge_;
    boardEdge_ = false;
    in.forward = ctx.actions.axis("cam_forward");
    in.right = ctx.actions.axis("cam_right");
    step(ctx.world, ctx.clock.fixedStep(), in);
}

void CityPlayerTransitSystem::step(World& world, Real dt, const RideInput& in) {
    const CitySim& sim = city_.sim();

    // The player, the same way every other citysim system finds it.
    Entity player;
    world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform&, engine::CharacterController&,
            engine::ControlledBy&) {
            if (!player.valid()) player = e;
        });
    if (!player.valid()) {
        riding_ = -1;
        return;
    }
    Transform* pt = world.get<Transform>(player);
    auto* cc = world.get<engine::CharacterController>(player);
    if (!pt || !cc) return;

    // THE RIDE YIELDS TO ANYTHING ELSE THAT MOVES YOU. This system writes the
    // player's position every step, which means it beats every OTHER way of
    // moving the player -- F's fast travel, `teleport`, T, a respawn. Glenn hit
    // exactly that: F put him at the new place and the next tick dragged him
    // back to the bus he did not know he was on.
    //
    // Rather than enumerate the ways a player can be moved (and miss one), the
    // rule is: if the player is no longer where WE put them, someone else moved
    // them, and that IS getting off.
    if (riding_ >= 0 && seated_) {
        const Real dx = pt->position.x - seat_.x;
        const Real dy = pt->position.y - seat_.y;
        const Real dz = pt->position.z - seat_.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 3.0) {
            LOG_INFO << "[transit] you left the vehicle (moved by something else)";
            riding_ = -1;
            seated_ = false;
            return;
        }
    }

    double req = in.request;
    // E on foot boards; in a cab it gets out; ON A BUS it is the saloon's own
    // verb -- sit, stand, or step off at a door -- handled in rideBus.
    bool busInteract = false;
    if (in.interact) {
        if (riding_ < 0) req = 3.0;
        else if (!sim.isBus(riding_)) req = 0.0;
        else busInteract = true;
    }

    const std::vector<Agent>& agents = sim.agents();

    // --- get off (a cab, the verb, or a vehicle that no longer exists) -----
    if (riding_ >= 0 && (req == 0.0 || riding_ >= static_cast<int>(agents.size()))) {
        const Vec3 here = pt->position;
        // Step off to the SIDE, not into the road the vehicle is standing in.
        const Real gy = city_.groundHeightAt(here.x, here.z);
        leave(world, player, Vec3(here.x + kStepOffDistance, gy + 1.2, here.z));
        return;
    }

    // --- get on --------------------------------------------------------
    // NOT FROM INDOORS. E is also the lift call, and a hoistway lobby can sit
    // within reach of a street: pressing E to call a lift while a bus passed
    // outside the wall would put you on the bus.
    bool indoors = false;
    world.each<engine::CityBuildings>([&](Entity, engine::CityBuildings& cb) {
        if (indoors) return;
        if (cb.recordAt(Vec2(pt->position.x, pt->position.z), pt->position.y))
            indoors = true;
    });
    if (indoors && req >= 1.0) {
        LOG_INFO << "[transit] not from indoors -- step outside to board";
        req = -1.0;
    }
    if (riding_ < 0 && req >= 1.0) {
        const bool wantBus = (req == 1.0 || req == 3.0);
        const bool wantTaxi = (req == 2.0 || req == 3.0);
        int best = -1;
        Real bestD = kBoardRadius;
        for (std::size_t i = 0; i < agents.size(); ++i) {
            const int ai = static_cast<int>(i);
            const bool isBus = sim.isBus(ai);
            const bool isTaxi = sim.isTaxi(ai);
            if (!((isBus && wantBus) || (isTaxi && wantTaxi))) continue;
            // You board at a STOP, not by touching a moving bus.
            if (agents[i].speed > 2.0) continue;
            const Real dx = agents[i].pos.x - pt->position.x;
            const Real dz = agents[i].pos.y - pt->position.z;
            const Real d = std::sqrt(dx * dx + dz * dz);
            if (d < bestD) { bestD = d; best = ai; }
        }
        if (best < 0) {
            LOG_INFO << "[transit] nothing to board within " << kBoardRadius
                     << " m (stand nearer a bus or a cab)";
        } else {
            riding_ = best;
            seatIdx_ = -1;
            haveBusYaw_ = false;
            // In through the MIDDLE DOOR, standing in the aisle beside it.
            const std::vector<Vec3>& doors = city_.busDoors();
            local_ = doors.size() >= 2 ? Vec3(0, city_.busFloorY(), doors[1].z)
                                       : Vec3(0, city_.busFloorY(), -0.8);
            if (!world.has<engine::Passenger>(player)) world.add<engine::Passenger>(player);
            LOG_INFO << "[transit] you boarded "
                     << (sim.isBus(best) ? "a bus" : "a cab") << " (agent "
                     << best << ") " << bestD << " m away";
        }
    }

    // --- ride ----------------------------------------------------------
    if (riding_ < 0) return;
    if (riding_ >= static_cast<int>(agents.size())) { riding_ = -1; return; }
    engine::Mat4 pose;
    if (sim.isBus(riding_) && city_.busFrame(riding_, &pose)) {
        rideBus(world, dt, in, player, *pt, *cc, pose, busInteract);
        return;
    }

    // A cab (no saloon to walk in): pinned over its pose, as a passenger.
    const Agent& veh = agents[static_cast<std::size_t>(riding_)];
    const Real gy = city_.groundHeightAt(veh.pos.x, veh.pos.y);
    pin(world, player, *pt, *cc, Vec3(veh.pos.x, gy + veh.elevation + kSeatLift, veh.pos.y));
}

// Put the player at `pos` for this step: transform, no interpolation streak,
// and the physics character with it (nothing else moves it while a Passenger).
void CityPlayerTransitSystem::pin(World& world, Entity player, Transform& t,
                                  engine::CharacterController& cc, const Vec3& pos) {
    t.position = pos;
    if (auto* prev = world.get<engine::PrevTransform>(player)) prev->value = t;
    if (cc.characterId != engine::INVALID_CHARACTER)
        physics_.physicsWorld().setCharacterPosition(cc.characterId, pos);
    seat_ = pos;
    seated_ = true;
}

// Off the vehicle, at `where`, back under PlayerSystem's control.
void CityPlayerTransitSystem::leave(World& world, Entity player, const Vec3& where) {
    if (Transform* t = world.get<Transform>(player)) {
        t->position = where;
        if (auto* prev = world.get<engine::PrevTransform>(player)) prev->value = *t;
    }
    if (auto* cc = world.get<engine::CharacterController>(player))
        if (cc->characterId != engine::INVALID_CHARACTER)
            physics_.physicsWorld().setCharacterPosition(cc->characterId, where);
    if (world.has<engine::Passenger>(player)) world.remove<engine::Passenger>(player);
    city_.setPlayerSeat(-1, -1);
    riding_ = -1;
    seated_ = false;
    seatIdx_ = -1;
    LOG_INFO << "[transit] you got off at (" << where.x << ", " << where.z << ")";
}

// ON THE BUS (Glenn, 2026-09-18: "when I'm in the bus there's a lot of crazy
// jittering... Should the player be able to move freely around the bus
// interior and find a seat?"). The player lives in the bus's OWN frame: a floor
// point in the saloon, moved by the ordinary walk keys, turned into a world
// position through the very matrix the bus body is drawn with this step. So
// the view cannot drift against the bus -- the jitter was the character being
// pinned from the sim's 30 Hz position while the body was drawn from its
// extrapolated one, and moved by physics in between.
void CityPlayerTransitSystem::rideBus(World& world, Real dt, const RideInput& in,
                                      Entity player, Transform& t,
                                      engine::CharacterController& cc,
                                      const engine::Mat4& pose, bool interact) {
    const CitySim& sim = city_.sim();
    const Agent& bus = sim.agents()[static_cast<std::size_t>(riding_)];
    const std::vector<Vec3>& seats = city_.busSeats();
    const std::vector<Vec3>& doors = city_.busDoors();
    const Real floorY = city_.busFloorY();

    auto flat = [](Vec3 v) {
        v.y = 0;
        const Real l = std::sqrt(v.x * v.x + v.z * v.z);
        return l > 1e-6 ? v * (1 / l) : Vec3(0, 0, 1);
    };
    const Vec3 fwdAxis = flat(Vec3(pose.m[0][2], 0, pose.m[2][2]));
    const Vec3 rightAxis = flat(Vec3(pose.m[0][0], 0, pose.m[2][0]));

    // The view turns WITH the bus: a corner swings the saloon, not the world.
    const Real busYaw = std::atan2(fwdAxis.x, -fwdAxis.z) * Real(57.29577951308232);
    if (haveBusYaw_) {
        Real d = busYaw - lastBusYaw_;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        fly_.yaw += d;
    }
    lastBusYaw_ = busYaw;
    haveBusYaw_ = true;

    auto dist2d = [](const Vec3& a, const Vec3& b) {
        const Real dx = a.x - b.x, dz = a.z - b.z;
        return std::sqrt(dx * dx + dz * dz);
    };
    int nearDoor = -1;
    Real doorD = kDoorReach;
    for (std::size_t i = 0; i < doors.size(); ++i) {
        const Real d = dist2d(local_, doors[i]);
        if (d < doorD) { doorD = d; nearDoor = static_cast<int>(i); }
    }
    int nearSeat = -1;
    Real seatD = kSeatReach;
    for (std::size_t i = 0; i < seats.size(); ++i) {
        if (city_.busSeatTaken(riding_, static_cast<int>(i))) continue;
        const Real d = dist2d(local_, seats[i]);
        if (d < seatD) { seatD = d; nearSeat = static_cast<int>(i); }
    }

    if (interact) {
        if (seatIdx_ >= 0) {                                   // stand up
            local_ = Vec3(0, floorY, seats[static_cast<std::size_t>(seatIdx_)].z);
            seatIdx_ = -1;
            city_.setPlayerSeat(-1, -1);
        } else if (nearDoor >= 0) {                            // step off
            if (bus.speed <= 2.0) {
                Vec3 out = pose.transformPoint(doors[static_cast<std::size_t>(nearDoor)]) +
                           rightAxis * Real(1.3);
                out.y = city_.groundHeightAt(out.x, out.z) + Real(1.2);
                leave(world, player, out);
                return;
            }
            LOG_INFO << "[transit] the doors open at the next stop";
        } else if (nearSeat >= 0) {                            // sit down
            seatIdx_ = nearSeat;
            city_.setPlayerSeat(riding_, nearSeat);
        }
    }

    // Walk the saloon: the ordinary move keys, taken in the bus's frame. The
    // aisle is the floor; a door's zone reaches out to the kerb-side wall.
    if (seatIdx_ < 0) {
        Vec3 move = flat(fly_.forward()) * in.forward + flat(fly_.right()) * in.right;
        const Real ml = std::sqrt(move.x * move.x + move.z * move.z);
        if (ml > 1) move = move * (1 / ml);
        local_.x += (move.x * rightAxis.x + move.z * rightAxis.z) * kAisleWalk * dt;
        local_.z += (move.x * fwdAxis.x + move.z * fwdAxis.z) * kAisleWalk * dt;
        Real zMin = 1e9, zMax = -1e9;
        for (const Vec3& s : seats) zMin = std::min(zMin, s.z - Real(0.2));
        for (const Vec3& d : doors) zMax = std::max(zMax, d.z + Real(0.3));
        if (zMin < zMax) local_.z = std::clamp(local_.z, zMin, zMax);
        Real xMax = kAisleHalf;
        for (const Vec3& d : doors)
            if (std::fabs(local_.z - d.z) < Real(0.7)) xMax = std::max(xMax, d.x);
        local_.x = std::clamp(local_.x, -kAisleHalf, xMax);
        local_.y = floorY;
    }

    const Vec3 at = seatIdx_ >= 0 ? seats[static_cast<std::size_t>(seatIdx_)] : local_;
    const Real eyeAbove = seatIdx_ >= 0 ? kSeatedEye : kStandingEye;
    const Vec3 w = pose.transformPoint(at);
    pin(world, player, t, cc, Vec3(w.x, w.y + eyeAbove - kEyeAboveTransform, w.z));

    // What E would do now, for the on-screen line.
    busPrompt_ = seatIdx_ >= 0 ? "E: stand up"
               : nearDoor >= 0 ? (bus.speed <= 2.0 ? "E: get off" : "doors open at the next stop")
               : nearSeat >= 0 ? "E: sit down"
                               : "";
}

void CityPlayerTransitSystem::update(engine::FrameContext& ctx) {
    if (ctx.actions.pressed("transit_board")) boardEdge_ = true;
    city_.setPlayerRidingAgent(riding_);
    hud_ = Hud{};
    const CitySim& sim = city_.sim();
    const BusNetwork& net = sim.buses();
    if (net.routeCount() == 0) return;              // no buses on this level
    // Driving a car of your own: the bus is not your problem.
    if (riding_ < 0 && ctx.actions.context() == engine::InputContext::InVehicle) return;

    World& world = ctx.world;
    Entity player;
    world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform&, engine::CharacterController&, engine::ControlledBy&) {
            if (!player.valid()) player = e;
        });
    if (!player.valid()) return;
    const Transform* pt = world.get<Transform>(player);
    if (!pt) return;
    const Vec2 me(pt->position.x, pt->position.z);
    const std::vector<Agent>& agents = sim.agents();

    if (riding_ >= 0 && riding_ < static_cast<int>(agents.size())) {
        hud_.mode = Hud::Mode::Riding;
        hud_.ridingRoute = sim.busRouteOf(riding_);
        const int si = sim.busNextStopOf(riding_);
        if (hud_.ridingRoute >= 0 && si >= 0) {
            const BusRoute& r = net.route(hud_.ridingRoute);
            if (si < static_cast<int>(r.stops.size())) {
                const Vec2 d = r.stops[static_cast<std::size_t>(si)].pos -
                               agents[static_cast<std::size_t>(riding_)].pos;
                hud_.nextStopDistance = std::sqrt(d.x * d.x + d.y * d.y);
            }
        }
        return;
    }

    // Not from indoors: the lift owns E in there, and "a bus stop 90 m
    // behind the wall" is not what someone in a lobby needs to read.
    bool indoors = false;
    world.each<engine::CityBuildings>([&](Entity, engine::CityBuildings& cb) {
        if (!indoors && cb.recordAt(me, pt->position.y)) indoors = true;
    });
    if (indoors) return;

    hud_.mode = Hud::Mode::OnFoot;
    hud_.inService = sim.busesInService();

    // The nearest stop, whichever route it belongs to.
    int bestR = -1, bestS = -1;
    Real bestD = 0;
    for (int r = 0; r < net.routeCount(); ++r) {
        const BusRoute& route = net.route(r);
        for (std::size_t s = 0; s < route.stops.size(); ++s) {
            const Vec2 d = route.stops[s].pos - me;
            const Real dd = std::sqrt(d.x * d.x + d.y * d.y);
            if (bestR < 0 || dd < bestD) { bestR = r; bestS = static_cast<int>(s); bestD = dd; }
        }
    }
    if (bestR < 0) { hud_.mode = Hud::Mode::None; return; }
    const BusStop& stop = net.route(bestR).stops[static_cast<std::size_t>(bestS)];
    hud_.stopDistance = bestD;
    const engine::Vec3 fwd = ctx.view.camera.target - ctx.view.camera.position;
    hud_.stopDirection = relativeDirection(Vec2(fwd.x, fwd.z), stop.pos - me);

    // Every route calling there (a hub is several), and how far off each one's
    // next bus is, counted in stops -- a time would be a guess about traffic.
    for (int r = 0; r < net.routeCount(); ++r) {
        const BusRoute& route = net.route(r);
        for (std::size_t s = 0; s < route.stops.size(); ++s) {
            if (route.stops[s].node != stop.node) continue;
            Hud::Serving sv;
            sv.route = r;
            const int n = static_cast<int>(route.stops.size());
            for (std::size_t i = 0; i < agents.size(); ++i) {
                const int ai = static_cast<int>(i);
                if (sim.busRouteOf(ai) != r) continue;
                const Vec2 d = agents[i].pos - stop.pos;
                const Real dd = std::sqrt(d.x * d.x + d.y * d.y);
                // STANDING HERE, doors open. Checked by position, not index: a
                // bus moves its index on the moment it arrives, so by its own
                // count the stop it is standing at is a whole lap away.
                if (dd < 25.0 && agents[i].speed <= 2.0) {
                    sv.atStop = true;
                    sv.stopsAway = 0;
                    sv.busDistance = dd;
                    break;
                }
                const int next = sim.busNextStopOf(ai);
                if (next < 0) continue;
                const int away = ((static_cast<int>(s) - next) % n + n) % n;
                if (sv.stopsAway < 0 || away < sv.stopsAway) {
                    sv.stopsAway = away;
                    sv.busDistance = dd;
                }
            }
            hud_.serving.push_back(sv);
        }
    }

    // What E would do right now: the same test the boarding code applies.
    Real nearest = kBoardRadius;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const int ai = static_cast<int>(i);
        const bool isBus = sim.isBus(ai), isTaxi = sim.isTaxi(ai);
        if (!isBus && !isTaxi) continue;
        if (agents[i].speed > 2.0) continue;
        const Vec2 d = agents[i].pos - me;
        const Real dd = std::sqrt(d.x * d.x + d.y * d.y);
        if (dd < nearest) {
            nearest = dd;
            hud_.boardable = isBus ? "bus" : "cab";
            hud_.boardableRoute = isBus ? sim.busRouteOf(ai) : -1;
        }
    }
}

void CityPlayerTransitSystem::render(engine::FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (hud_.mode == Hud::Mode::None) return;
    // Anchored by its BOTTOM-left corner: the line count varies (a hub lists
    // every route calling there), and a top-anchored panel ran off the screen.
    ImGui::SetNextWindowPos(ImVec2(24.0f, static_cast<float>(ctx.windowHeight) - 24.0f),
                            ImGuiCond_Always, ImVec2(0.0f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("Transit", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    auto routeText = [](int route) {
        // Lifted toward white: the sign colours are for paint in daylight, and
        // dark red or blue text on a dark panel was hard to read.
        const engine::Vec3 c = routeColour(route);
        auto lift = [](Real v) { return static_cast<float>(v + (1.0 - v) * 0.4); };
        ImGui::TextColored(ImVec4(lift(c.x), lift(c.y), lift(c.z), 1.0f),
                           "route %d (%s)", route, routeColourName(route));
    };
    if (hud_.mode == Hud::Mode::Riding) {
        if (hud_.ridingRoute >= 0) {
            ImGui::Text("ON THE BUS");
            ImGui::SameLine();
            routeText(hud_.ridingRoute);
            if (hud_.nextStopDistance >= 0)
                ImGui::Text("next stop in %.0f m", hud_.nextStopDistance);
            // What E does where you stand: sit, stand, or step off at a door.
            ImGui::Text("%s%s%s", seatIdx_ >= 0 ? "seated" : "walk: move keys",
                        busPrompt_[0] ? "      " : "", busPrompt_);
        } else {
            ImGui::Text("IN A CAB      E: get off");
        }
    } else {
        ImGui::Text("BUS STOP  %.0f m %s", hud_.stopDistance, hud_.stopDirection);
        if (!hud_.inService) {
            ImGui::Text("buses are off duty for the night");
        } else {
            for (const Hud::Serving& sv : hud_.serving) {
                routeText(sv.route);
                ImGui::SameLine();
                if (sv.stopsAway < 0) ImGui::Text("no bus running");
                else if (sv.atStop) ImGui::Text("bus at the stop now");
                else if (sv.stopsAway == 0)
                    ImGui::Text("next bus coming, %.0f m away", sv.busDistance);
                else ImGui::Text("next bus %d stop%s away", sv.stopsAway,
                                 sv.stopsAway == 1 ? "" : "s");
            }
        }
        if (hud_.boardable) {
            if (hud_.boardableRoute >= 0) {
                ImGui::Text("E: board the bus,");
                ImGui::SameLine();
                routeText(hud_.boardableRoute);
            } else {
                ImGui::Text("E: get in the %s", hud_.boardable);
            }
        }
    }
    ImGui::End();
#else
    (void)ctx;
#endif
}

}  // namespace citysim
