#include "city_poses.h"

#include "city_sim.h"

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Entity;
using engine::Quat;
using engine::Vec2;
using engine::World;

// ---- The ground ------------------------------------------------------------------------------------------------------

Real CityGround::groundAt(Real x, Real z) const {
    // ON a road: the deck the mesher drew (RoadDeck). The terrain under a road
    // is carved 0.22 m below the profile on purpose; sampling it put every
    // car's wheels inside the asphalt (device: "below the road").
    for (const engine::RoadDeckField& f : decks) {
        double y;
        if (f.heightAt(x, z, sidewalk + 1.0, &y)) return static_cast<Real>(y);
    }
    return (terrain ? terrain(x, z) : 0.0) + roadLift;
}

Real CityGround::deckSurfaceNear(Real x, Real z, Real refY) const {
    // 3 m: past the lerp's worst error on a curved ramp (1.5 m measured) and well under a grade
    // separation (kLayerClearance 5.8 m)
    constexpr double kWindow = 3.0;
    for (const engine::RoadDeckField& f : decks) {
        double y;
        if (f.heightNear(x, z, 0.5, refY, kWindow, &y)) return static_cast<Real>(y);
    }
    return refY;
}

// ---- The models ------------------------------------------------------------------------------------------------------

int CityCarModels::variantOf(int slot, VehicleType type) const {
    const int n = count();
    int v = slot;
    if (v >= n) v %= n;
    if (type == VehicleType::Bus && bus >= 0 && bus < n) v = bus;                  // a bus parked in its depot
    if (type == VehicleType::FireEngine && fire >= 0 && fire < n) v = fire;        // in its bay
    if (type == VehicleType::Taxi && taxi >= 0 && taxi < n) v = taxi;              // a cab at the kerb
    return v;
}

// ---- The fit ---------------------------------------------------------------------------------------------------------

Vec3 CityPoses::smoothUp(Vec3& su, Vec3 up, Real dt) {
    const Real k = 1.0 - std::exp(-dt / 0.7);
    if (su.lengthSquared() < 1e-6)
        su = up;
    else
        su = normalize(su + (up - su) * k);
    return su;
}

// FOUR WHEELS ON THE ROAD (#35 follow-up, Glenn: "we'd have to make sure the 4 wheels are on the ground"): the drawn
// surface under each tyre's contact patch, a plane fitted through them for pitch and roll, then the height that puts
// the LOWEST tyre on the surface -- none sinks; on a crest or a twist one floats by the plane's residual, as a real
// car's would. The memory keeps the tilt low-pass and the last pose: A CAR THAT HAS NOT MOVED (at a light, in a queue,
// at the kerb) sits exactly where it sat, not four surface samples and a fit again every step.
bool CityPoses::bodyOnWheels(int slot, bool lineWheels, Real x, Real z, Vec2 heading, FitMemory* memory, Real dt,
                             const std::function<Real(Real, Real, Real)>& surface, Mat4& out) const {
    const std::size_t fewestWheels = lineWheels ? 2 : 3;
    const std::vector<std::vector<CarWheel>>& all = models.wheels;
    const std::vector<CarWheel>* wheels =
        slot >= 0 && slot < static_cast<int>(all.size()) && all[static_cast<std::size_t>(slot)].size() >= fewestWheels
            ? &all[static_cast<std::size_t>(slot)] : nullptr;
    const Real fl = heading.length();
    FitMemory::Pose* pc = memory ? &memory->pose : nullptr;
    if (pc && pc->valid && pc->x == x && pc->z == z && pc->hx == heading.x && pc->hz == heading.y && pc->slot == slot) {
        out = pc->m;
        return true;
    }
    if (!wheels || !(fl > 1e-6)) return false;
    const Vec2 f = heading * (1.0 / fl);
    const Vec2 r(f.y, -f.x);                  // +X, the car's right
    // least squares y = c + gx * lx + gz * lz over the contacts (lx right, lz forward)
    struct C { Real lx, lz, h; Vec3 local; };
    C cs[8];
    int n = 0;
    for (const CarWheel& w : *wheels) {
        if (n == 8) break;
        const Vec3 local(w.pos.x, w.pos.y - w.radius, w.pos.z);
        const Real wx = x + r.x * local.x + f.x * local.z, wz = z + r.y * local.x + f.y * local.z;
        cs[n++] = {local.x, local.z, surface(wx, wz, local.z), local};
    }
    Real sx = 0, sz = 0, sh = 0;
    for (int i = 0; i < n; ++i) { sx += cs[i].lx; sz += cs[i].lz; sh += cs[i].h; }
    sx /= n; sz /= n; sh /= n;
    Real xx = 0, zz = 0, xz = 0, xh = 0, zh = 0;
    for (int i = 0; i < n; ++i) {
        const Real dx = cs[i].lx - sx, dz = cs[i].lz - sz, dh = cs[i].h - sh;
        xx += dx * dx; zz += dz * dz; xz += dx * dz; xh += dx * dh; zh += dz * dh;
    }
    const Real det = xx * zz - xz * xz;
    Real gx = 0, gz = 0;   // roll and pitch slopes
    if (std::fabs(det) > 1e-9) { gx = (xh * zz - zh * xz) / det; gz = (zh * xx - xh * xz) / det; }
    else if (lineWheels && zz > 1e-9) gz = zh / zz;   // wheels in one line (a two-wheeler): pitch only
    Vec3 fw = normalize(Vec3(f.x, gz, f.y));
    Vec3 rt = normalize(Vec3(r.x, gx, r.y));
    Vec3 up = normalize(cross(fw, rt));
    if (up.y < 0) up = up * -1;
    if (memory) up = smoothUp(memory->smoothUp, up, dt);
    fw = normalize(Vec3(f.x, 0, f.y) - up * dot(Vec3(f.x, 0, f.y), up));
    rt = normalize(cross(up, fw));
    // the height at which the lowest tyre just touches
    Real cy = -1e30;
    for (int i = 0; i < n; ++i) {
        const Real dy = rt.y * cs[i].local.x + up.y * cs[i].local.y + fw.y * cs[i].local.z;
        cy = std::max(cy, cs[i].h - dy);
    }
    Mat4 m;   // columns: X = right, Y = up, Z = forward
    m.m[0][0] = rt.x; m.m[1][0] = rt.y; m.m[2][0] = rt.z;
    m.m[0][1] = up.x; m.m[1][1] = up.y; m.m[2][1] = up.z;
    m.m[0][2] = fw.x; m.m[1][2] = fw.y; m.m[2][2] = fw.z;
    m.m[0][3] = x; m.m[1][3] = cy; m.m[2][3] = z;
    if (pc) *pc = FitMemory::Pose{x, z, heading.x, heading.y, slot, true, m};
    out = m;
    return true;
}

Mat4 CityPoses::parkedCarPose(int variant, Vec2 pos, Vec2 heading, Real parkedY,
                              const std::vector<Vec3>& halfExtents) const {
    const Real y = (parkedY > -1e29 ? parkedY : ground.groundAt(pos.x, pos.y)) +
                   (variant >= 0 && variant < static_cast<int>(halfExtents.size()) ? halfExtents[static_cast<std::size_t>(variant)].y : 0.65);
    const Real yaw = std::atan2(heading.x, heading.y);
    return Mat4::trs(Vec3(pos.x, y, pos.y), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(1, 1, 1));
}

std::vector<Vec3> CityPoses::halfExtents(const CitySim& sim) const {
    std::vector<Vec3> out;
    out.reserve(static_cast<std::size_t>(models.groups));
    for (int v = 0; v < models.groups; ++v) {
        // The ADOPTED fleet, not the built-in table: these half-extents size the
        // kinematic collider proxies and the parked-car placement, so they have
        // to be the dimensions the drawn car was actually built at.
        const VehicleBody& b = sim.fleetBody(v);
        out.push_back(Vec3(b.width * 0.5, b.height * 0.5, b.length * 0.5));
    }
    return out;
}

// ---- The car entities ------------------------------------------------------------------------------------------------

Mat4 CityPoses::carEntityPose(const CitySim& sim, const CityCarOnLane* lane, const engine::Transform& t,
                              const engine::Transform& prev, int variant, const std::vector<Vec3>& he, FitMemory* memory,
                              Real dt, Real* roadRef) const {
    const Real period = sim.tickPeriod();
    const Real tickT = period > 0.0 ? std::clamp(sim.secondsSinceTick() / period, Real(0), Real(1)) : Real(1);
    Vec2 pos = carPosition(t), heading = carHeading(t);
    const Vec2 from = carPosition(prev), d = pos - from;
    if (d.lengthSquared() < 8.0 * 8.0) {   // (a placement snaps, never slides)
        const Real yawFrom = std::atan2(carHeading(prev).x, carHeading(prev).y);
        Real dy = std::atan2(heading.x, heading.y) - yawFrom;
        while (dy > engine::PI) dy -= 2 * engine::PI;
        while (dy < -engine::PI) dy += 2 * engine::PI;
        pos = from + d * tickT;
        heading = Vec2(std::sin(yawFrom + dy * tickT), std::cos(yawFrom + dy * tickT));
    }
    const Real ref = lane ? lane->roadYFrom + (lane->roadY - lane->roadYFrom) * tickT : Real(0);
    if (roadRef) *roadRef = ref;
    Mat4 m;
    if (!lane || !bodyOnWheels(variant, true, pos.x, pos.y, heading, memory, dt,
                               [&](Real px, Real pz, Real) { return ground.deckSurfaceNear(px, pz, ref); }, m))
        m = parkedCarPose(variant, pos, heading, -1e30, he);
    return m;
}

void CityPoses::stepCarPoses(World& world, const CitySim& sim, Real dt) {
    lastDt_ = dt;
    ++steps_;
    const std::vector<Vec3> he = halfExtents(sim);
    std::vector<Entity> cars;
    world.each<CityCarBody>([&](Entity e, CityCarBody&) { cars.push_back(e); });
    for (const Entity e : cars) poseCar(world, e, sim, he, dt);
    // (the memories of cars not posed this step go: see carMemory_)
    for (std::unordered_map<uint32_t, FitMemory>::iterator it = carMemory_.begin(); it != carMemory_.end();)
        it = it->second.step != steps_ ? carMemory_.erase(it) : std::next(it);
}

void CityPoses::poseCarNow(World& world, Entity car, const CitySim& sim) {
    poseCar(world, car, sim, halfExtents(sim), lastDt_);
}

void CityPoses::poseCar(World& world, Entity e, const CitySim& sim, const std::vector<Vec3>& he, Real dt) {
    const CityCarBody* body = world.get<CityCarBody>(e);
    const CityCarLink* link = world.get<CityCarLink>(e);
    const engine::Transform* t = world.get<engine::Transform>(e);
    const engine::PrevTransform* prev = world.get<engine::PrevTransform>(e);
    if (!body || !link || !t || !prev || link->away || world.has<engine::Vehicle>(e) || carNowhere(world, e)) {
        world.remove<CityCarPose>(e);
        return;
    }
    CityCarPose pose;
    FitMemory& memory = carMemory_[body->id];
    memory.step = steps_;
    pose.body = carEntityPose(sim, world.get<CityCarOnLane>(e), *t, prev->value, models.variantOf(body->slot, body->type),
                              he, &memory, dt, &pose.road);
    world.add<CityCarPose>(e, pose);
}

// ---- The walkers -----------------------------------------------------------------------------------------------------

Real CityPoses::rideDeck(Ride r) {
    return r == Ride::Skateboard ? Real(0.12) : r == Ride::Scooter ? Real(0.09) : Real(0);
}

Mat4 CityPoses::walkerPose(const CitySim& sim, const Agent& a, Real height) const {
    Real x = a.pos.x, z = a.pos.y;          // Vec2 maps to world XZ (.y = world z)
    Vec2 heading = a.heading;
    // BETWEEN SIM TICKS (localHz below the fixed rate): one tick behind, from where they were as this tick began to
    // where they are now (a placement snaps, never slides)
    const Real period = sim.tickPeriod();
    if (period > 0.0 && !a.released) {
        const Vec2 d = a.pos - a.tickFromPos;
        if (d.x * d.x + d.y * d.y < 8.0 * 8.0) {
            const Real t = std::clamp(sim.secondsSinceTick() / period, Real(0), Real(1));
            x = a.tickFromPos.x + d.x * t;
            z = a.tickFromPos.y + d.y * t;
            Real dy = std::atan2(a.heading.x, a.heading.y) - std::atan2(a.tickFromHeading.x, a.tickFromHeading.y);
            while (dy > engine::PI) dy -= 2 * engine::PI;
            while (dy < -engine::PI) dy += 2 * engine::PI;
            const Real yw = std::atan2(a.tickFromHeading.x, a.tickFromHeading.y) + dy * t;
            heading = Vec2(std::sin(yw), std::cos(yw));
        }
    }
    const Real halfH = height * 0.5;
    // Absolute deck (corridor): the deck Y IS the surface
    const Real y = (a.deckY > -1e29) ? ground.deckSurfaceNear(x, z, a.deckY) + halfH
                                     : ground.groundAt(x, z) + a.elevation + halfH;
    const Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(heading.x, heading.y));
    return Mat4::trs(Vec3(x, y, z), rot, Vec3(1, 1, 1));
}

Mat4 CityPoses::walkerStandingPose(const CitySim& sim, int ai, Real height, Real* feet) const {
    const Agent& a = sim.agents()[static_cast<std::size_t>(ai)];
    Mat4 pose = walkerPose(sim, a, height);
    if (feet) *feet = pose.m[1][3] - height * 0.5;
    pose.m[1][3] += rideDeck(sim.rideOf(ai));
    const Real s = sim.bodyScale(ai);
    if (s != 1.0) {
        pose.m[1][3] -= height * 0.5 * (1.0 - s);   // (walkerPose stands the body half its height up)
        pose = pose * Mat4::scale(s, s, s);
    }
    return pose;
}

void CityPoses::walkerBoxPoses(const CitySim& sim, Real height, std::vector<std::pair<int, Mat4>>& out) const {
    out.clear();
    const std::vector<Agent>& agents = sim.agents();
    for (const int ai : sim.nearAgents()) {
        const Agent& a = agents[static_cast<std::size_t>(ai)];
        if (a.far() || a.mode == Agent::Mode::Driver || !sim.pedVisible(ai)) continue;
        CitySim::RestPose rp;
        if (sim.restPose(ai, rp)) continue;   // (sitting or lying: drawn in their own groups, and given no box)
        out.push_back({ai, walkerStandingPose(sim, ai, height, nullptr)});
    }
}

}  // namespace citysim
