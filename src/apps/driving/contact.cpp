#include "apps/driving/contact.h"

#include "engine/procgen/city/roads/lane_graph.h"   // engine::bodyOnPlan

#include <algorithm>
#include <cmath>
#include <limits>

namespace driving {

PlanBody planBody(CarId car, const CarPose& pose, double length, double width) {
    // The lane graph's one body rule (engine::bodyOnPlan), so what is tested here is what the road builder swept.
    const engine::PlanRectangle r =
        engine::bodyOnPlan(engine::Vec2(pose.front.x, pose.front.z), engine::Vec2(pose.rear.x, pose.rear.z),
                           engine::Vec2(pose.travel.x, pose.travel.z), length, width);
    PlanBody b;
    b.car = car;
    b.x = r.middle.x;
    b.z = r.middle.y;
    b.height = (pose.front.y + pose.rear.y) * 0.5;
    b.ux = r.forward.x;
    b.uz = r.forward.y;
    b.halfLength = r.halfLength;
    b.halfWidth = r.halfWidth;
    return b;
}

double reachAlong(const PlanBody& b, double dx, double dz) {
    return b.halfLength * std::fabs(b.ux * dx + b.uz * dz) + b.halfWidth * std::fabs(-b.uz * dx + b.ux * dz);
}

double bodyOverlapDepth(const PlanBody& a, const PlanBody& b) {
    // The four axes of two rectangles: each one's length and width directions. The depth on an axis is how far the
    // two projections overlap; the shallowest is the push that parts them, and any axis with none parts them already.
    const double axes[4][2] = {{a.ux, a.uz}, {-a.uz, a.ux}, {b.ux, b.uz}, {-b.uz, b.ux}};
    const double cx = b.x - a.x, cz = b.z - a.z;
    double depth = std::numeric_limits<double>::infinity();
    for (const double* axis : axes) {
        const double apart = std::fabs(cx * axis[0] + cz * axis[1]);
        const double d = reachAlong(a, axis[0], axis[1]) + reachAlong(b, axis[0], axis[1]) - apart;
        if (!(d > 0)) return 0;
        depth = std::min(depth, d);
    }
    return depth;
}

void findContacts(const std::vector<PlanBody>& bodies, std::vector<std::size_t>& sweep, std::vector<BodyContact>& out) {
    out.clear();
    // THE BROAD PHASE: bodies in order of the lowest x their box on the plan reaches (then car id); sweeping along x,
    // each is tested only against those whose boxes it reaches into.
    sweep.resize(bodies.size());
    for (std::size_t i = 0; i < sweep.size(); ++i) sweep[i] = i;
    const auto lowX = [&](std::size_t i) { return bodies[i].x - reachAlong(bodies[i], 1, 0); };
    std::sort(sweep.begin(), sweep.end(), [&](std::size_t p, std::size_t q) {
        const double lp = lowX(p), lq = lowX(q);
        return lp != lq ? lp < lq : bodies[p].car < bodies[q].car;
    });
    for (std::size_t s = 0; s < sweep.size(); ++s) {
        const PlanBody& a = bodies[sweep[s]];
        const double highX = a.x + reachAlong(a, 1, 0);
        const double reachZ = reachAlong(a, 0, 1);
        for (std::size_t t = s + 1; t < sweep.size() && lowX(sweep[t]) <= highX; ++t) {
            const PlanBody& b = bodies[sweep[t]];
            if (std::fabs(b.z - a.z) > reachZ + reachAlong(b, 0, 1)) continue;
            if (std::fabs(b.height - a.height) > kTouchSameLevelMetres) continue;
            const double depth = bodyOverlapDepth(a, b);
            if (!(depth > kTouchToleranceMetres)) continue;
            BodyContact c;
            const bool aFirst = a.car < b.car;
            c.a = aFirst ? sweep[s] : sweep[t];
            c.b = aFirst ? sweep[t] : sweep[s];
            c.depth = depth;
            c.x = (a.x + b.x) * 0.5;
            c.z = (a.z + b.z) * 0.5;
            out.push_back(c);
        }
    }
    std::sort(out.begin(), out.end(), [&](const BodyContact& p, const BodyContact& q) {
        return bodies[p.a].car != bodies[q.a].car ? bodies[p.a].car < bodies[q.a].car : bodies[p.b].car < bodies[q.b].car;
    });
}

}  // namespace driving
