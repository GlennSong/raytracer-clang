#include "lot_fixtures.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;

Vec3 at3(const Vec2& p, Real y) { return Vec3(p.x, y, p.y); }

}  // namespace

const char* interactKindName(InteractKind k) {
    switch (k) {
        case InteractKind::Gate:   return "gate";
        case InteractKind::Door:   return "door";
        case InteractKind::Seat:   return "seat";
        case InteractKind::Switch: return "switch";
        case InteractKind::Bin:    return "bin";
    }
    return "?";
}

const char* fixtureKindName(FixtureKind k) {
    switch (k) {
        case FixtureKind::PorchLight: return "porch-light";
        case FixtureKind::PathLight:  return "path-light";
        case FixtureKind::GateLamp:   return "gate-lamp";
        case FixtureKind::WindowGlow: return "window-glow";
        case FixtureKind::SignFace:   return "sign-face";
        case FixtureKind::StreetLamp: return "street-lamp";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Instancing
// ---------------------------------------------------------------------------

std::vector<InstanceBatch> batchProps(const std::vector<PropInstance>& props) {
    std::vector<InstanceBatch> out;
    for (const PropInstance& p : props) {
        InstanceBatch* batch = nullptr;
        for (InstanceBatch& b : out)
            if (b.kind == p.kind) { batch = &b; break; }
        if (!batch) {
            out.push_back(InstanceBatch{p.kind, {}});
            batch = &out.back();
        }
        batch->instances.push_back(InstanceBatch::Xform{p.at, p.yaw, p.scale});
    }
    return out;
}

// ---------------------------------------------------------------------------
// Light budget
// ---------------------------------------------------------------------------

Real LightBudget::score(const LightSpec& l, const Vec3& viewer) {
    const Vec3 d = l.at - viewer;
    const Real dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
    // Falloff-weighted, so a bright distant lamp can still outrank a dim near
    // one — which is what stops a single porch bulb evicting a street lamp.
    return l.intensity / (1.0 + dist2);
}

std::vector<int> LightBudget::select(const std::vector<LightSpec>& lights,
                                     const Vec3& viewer) const {
    std::vector<std::pair<Real, int>> ranked;
    for (std::size_t i = 0; i < lights.size(); ++i) {
        // Emissive-only fixtures cost nothing and are always lit; they must
        // never occupy a slot. This is tier 1 of the ladder doing its job.
        if (lights[i].emissiveOnly) continue;
        ranked.push_back({score(lights[i], viewer), static_cast<int>(i)});
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const auto& a, const auto& b) {
                  if (a.first != b.first) return a.first > b.first;
                  return a.second < b.second;      // stable for equal scores
              });

    // Street lamps get a reserved share. Without it a dense lot's own fixtures
    // win every slot near the camera and the STREET goes dark — the exact
    // starvation the plan warns about, and it looks far worse than a lot with
    // one light fewer.
    const int total = std::max(0, slots);
    const int reserved =
        std::min(total, static_cast<int>(total * std::max(Real(0), reserveForStreet)));
    std::vector<int> out;
    std::vector<char> taken(lights.size(), 0);

    for (const auto& [s, i] : ranked) {
        if (static_cast<int>(out.size()) >= reserved) break;
        if (lights[i].kind != FixtureKind::StreetLamp) continue;
        out.push_back(i);
        taken[i] = 1;
    }
    for (const auto& [s, i] : ranked) {
        if (static_cast<int>(out.size()) >= total) break;
        if (taken[i]) continue;
        out.push_back(i);
        taken[i] = 1;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Assembly
// ---------------------------------------------------------------------------

LotFixtures fixturesFor(const SitePlan& site, const SiteFurnishing& furnishing,
                        const LotProgram& program, const BuiltBuilding& building,
                        Real groundY, std::uint32_t seed) {
    LotFixtures f;
    Rng rng(seed ? seed : 29u);

    f.batches = batchProps(furnishing.props);

    // --- gates: the object the brief insists must be real -------------------
    for (const GateSpec& g : furnishing.gates) {
        InteractableSpec it;
        it.kind = InteractKind::Gate;
        it.at = at3(g.centre, groundY);
        it.yaw = std::atan2(g.axis.y, g.axis.x);
        it.width = g.width;
        it.height = g.height;
        it.prompt = "Open gate";
        it.reach = 1.8;
        // The hinge post is one end of the leaf, not its centre — a gate that
        // pivots about its middle is a turnstile. Which end depends on the
        // side the leaf was hung.
        const Vec2 post = g.hingeLeft ? g.centre - g.axis * (g.width * 0.5)
                                      : g.centre + g.axis * (g.width * 0.5);
        it.hinge.anchor = at3(post, groundY);
        it.hinge.axis = Vec3(0, 1, 0);
        it.hinge.minAngle = 0;
        it.hinge.maxAngle = g.swing;
        it.hinge.friction = 0.35;
        it.hinge.motor = false;               // a garden gate swings free
        // Tier B needs a real hinge constraint, which the Jolt wrapper does not
        // expose yet. Recording the capability means the same spec upgrades
        // from the scripted swing without the recipes changing.
        it.tierBCapable = true;
        const int idx = static_cast<int>(f.interactables.size());
        f.interactables.push_back(it);

        SensorSpec s;
        s.centre = at3(g.centre, groundY + g.height * 0.5);
        s.halfExtent = Vec3(g.width * 0.9, g.height * 0.6, 1.4);
        s.yaw = it.yaw;
        s.interactableIndex = idx;
        f.sensors.push_back(s);

        // A gate lamp on the post, where a real one would be.
        LightSpec lamp;
        lamp.kind = FixtureKind::GateLamp;
        lamp.at = at3(post, groundY + g.height + 0.35);
        lamp.intensity = 0.55;
        lamp.radius = 5.0;
        f.lights.push_back(lamp);
    }

    // --- the front door -----------------------------------------------------
    if (site.hasEntrance) {
        InteractableSpec d;
        d.kind = InteractKind::Door;
        d.at = at3(site.entrance, groundY);
        d.width = 1.1;
        d.height = 2.15;
        d.prompt = "Enter";
        d.hinge.anchor = at3(site.entrance, groundY);
        d.hinge.maxAngle = 1.5;
        d.hinge.motor = program.frontage == FrontageKind::Direct;   // a shop door
        d.tierBCapable = true;
        const int idx = static_cast<int>(f.interactables.size());
        f.interactables.push_back(d);

        SensorSpec s;
        s.centre = at3(site.entrance, groundY + 1.1);
        s.halfExtent = Vec3(1.4, 1.2, 1.4);
        s.interactableIndex = idx;
        f.sensors.push_back(s);

        LightSpec porch;
        porch.kind = FixtureKind::PorchLight;
        porch.at = at3(site.entrance, groundY + 2.4);
        porch.intensity = 0.8;
        porch.radius = 7.0;
        f.lights.push_back(porch);
    }

    // --- seats --------------------------------------------------------------
    for (std::size_t i = 0; i < furnishing.props.size(); ++i) {
        const PropInstance& p = furnishing.props[i];
        if (p.kind != PropKind::Bench) continue;
        SeatSpec s;
        s.at = at3(p.at, groundY);
        s.yaw = p.yaw;
        s.propIndex = static_cast<int>(i);
        f.seats.push_back(s);

        InteractableSpec it;
        it.kind = InteractKind::Seat;
        it.at = s.at;
        it.yaw = p.yaw;
        it.prompt = "Sit";
        it.reach = 1.2;
        f.interactables.push_back(it);
    }
    for (std::size_t i = 0; i < furnishing.props.size(); ++i) {
        if (furnishing.props[i].kind != PropKind::Bin) continue;
        InteractableSpec it;
        it.kind = InteractKind::Bin;
        it.at = at3(furnishing.props[i].at, groundY);
        it.prompt = "Open";
        it.reach = 1.2;
        f.interactables.push_back(it);
    }

    // --- path lights --------------------------------------------------------
    if (const ZoneArea* circ = site.zoneOf(Zone::Circulation)) {
        int placed = 0;
        for (const Shape2& part : circ->parts) {
            const Poly2 ring = tessellate(part.outer, 0.4);
            for (std::size_t i = 0; i < ring.size() && placed < 4; i += 3) {
                LightSpec l;
                l.kind = FixtureKind::PathLight;
                l.at = at3(ring[i], groundY + 0.7);
                l.intensity = 0.28;
                l.radius = 3.5;
                // Path lights are DECORATION: emissive geometry and a ground
                // pool, never a real light. Thousands of them across a city is
                // precisely what the 32-slot wall cannot absorb, and tier 1 of
                // the ladder gets the look for free.
                l.emissiveOnly = true;
                f.lights.push_back(l);
                ++placed;
            }
        }
    }

    // --- window glow --------------------------------------------------------
    // Lit windows are what actually make a night city read, and they cost
    // nothing: emissive only, one per level of the building's street face, a
    // fraction of them lit.
    for (std::size_t li = 0; li < building.surfaces.levels.size(); ++li) {
        const Level& lv = building.surfaces.levels[li];
        for (const Shape2& plan : lv.plans) {
            for (std::size_t e = 0; e < plan.outer.size(); ++e) {
                if (plan.outer.edges[e].tag != EdgeTag::Street) continue;
                if (!rng.chance(0.45)) continue;
                const Vec2 m = (plan.outer.start(e) + plan.outer.end(e)) * 0.5;
                LightSpec l;
                l.kind = FixtureKind::WindowGlow;
                l.at = at3(m, (lv.y0 + lv.y1) * 0.5);
                l.color = Vec3(1.0, 0.93, 0.78);
                l.intensity = 0.22;
                l.radius = 2.5;
                l.emissiveOnly = true;
                f.lights.push_back(l);
            }
        }
    }

    // --- shop sign ----------------------------------------------------------
    if (program.frontage == FrontageKind::Direct && site.hasEntrance) {
        LightSpec s;
        s.kind = FixtureKind::SignFace;
        s.at = at3(site.entrance, groundY + 3.6);
        s.color = Vec3(0.9, 0.95, 1.0);
        s.intensity = 0.6;
        s.radius = 4.0;
        s.emissiveOnly = true;
        f.lights.push_back(s);
    }
    return f;
}

}  // namespace engine
