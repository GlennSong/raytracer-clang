#include "city_player_transit.h"

#include "bus_stop_props.h"   // routeColour / routeColourName
#include "city_render.h"
#include "city_sim.h"

#include "../../engine/components.h"
#include "../../engine/input/input_map.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/procgen/city/building_records.h"
#include "../../engine/world.h"
#include "../../log.h"

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
    World& world = ctx.world;
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

    // `ride bus|taxi|any` boards, `ride off` gets off. Staged as a Setting the
    // way every other control verb reaches a system.
    double req = ctx.settings.getDouble("player.rideRequest", -1.0);
    if (req >= 0.0) ctx.settings.setDouble("player.rideRequest", -1.0);
    // E: board whatever is in reach, or get off if already aboard. The press
    // was latched in update(): `pressed` is a one-FRAME edge, and a frame can
    // run no fixed step at all, so reading it here dropped presses outright
    // (a `tap E` beside a standing bus did nothing, not even "nothing to
    // board"). ElevatorSystem latches its E the same way.
    if (boardEdge_) {
        boardEdge_ = false;
        req = (riding_ >= 0) ? 0.0 : 3.0;
    }

    const std::vector<Agent>& agents = sim.agents();

    // --- get off -------------------------------------------------------
    if (riding_ >= 0 && (req == 0.0 || riding_ >= static_cast<int>(agents.size()))) {
        const Vec3 here = pt->position;
        riding_ = -1;
        seated_ = false;
        // Step off to the SIDE, not into the road the vehicle is standing in.
        const Real gy = city_.groundHeightAt(here.x, here.z);
        const Vec3 off(here.x + kStepOffDistance, gy + 1.2, here.z);
        pt->position = off;
        if (auto* prev = world.get<engine::PrevTransform>(player)) prev->value = *pt;
        if (cc->characterId != engine::INVALID_CHARACTER)
            physics_.physicsWorld().setCharacterPosition(cc->characterId, off);
        LOG_INFO << "[transit] you got off at (" << off.x << ", " << off.z << ")";
        wasAboard_ = false;
        return;
    }

    // --- get on --------------------------------------------------------
    // NOT FROM INDOORS. E is also the lift call, and a hoistway lobby can sit
    // within 9 m of a street: pressing E to call a lift while a bus passed
    // outside the wall would put you on the bus. If the player stands inside a
    // building plan, the interaction belongs to whatever is in there.
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
            // You board at a STOP, not by touching a moving bus. Without this,
            // pressing E for anything else while a bus drove past captured the
            // player silently -- and E is the lift key.
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
            LOG_INFO << "[transit] you boarded "
                     << (sim.isBus(best) ? "a bus" : "a cab") << " (agent "
                     << best << ") " << bestD << " m away";
        }
    }

    // --- ride ----------------------------------------------------------
    if (riding_ < 0) return;
    if (riding_ >= static_cast<int>(agents.size())) { riding_ = -1; return; }
    const Agent& veh = agents[static_cast<std::size_t>(riding_)];

    // Sit on the vehicle's pose. The ground under it is the reference, not the
    // agent's own elevation, so a bus on a bridge deck carries you at deck
    // height rather than dropping you through it.
    const Real gy = city_.groundHeightAt(veh.pos.x, veh.pos.y);
    const Vec3 seat(veh.pos.x, gy + veh.elevation + kSeatLift, veh.pos.y);
    pt->position = seat;
    if (auto* prev = world.get<engine::PrevTransform>(player)) prev->value = *pt;
    if (cc->characterId != engine::INVALID_CHARACTER)
        physics_.physicsWorld().setCharacterPosition(cc->characterId, seat);
    seat_ = seat;
    seated_ = true;
    if (!wasAboard_) wasAboard_ = true;
}

void CityPlayerTransitSystem::update(engine::FrameContext& ctx) {
    if (ctx.actions.pressed("transit_board")) boardEdge_ = true;
    city_.setPlayerRiding(riding_ >= 0);
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
                ImGui::Text("next stop in %.0f m      E: get off", hud_.nextStopDistance);
            else
                ImGui::Text("E: get off");
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
