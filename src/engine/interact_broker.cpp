#include "interact_broker.h"
#include <algorithm>
#include <cmath>

namespace engine {

void offerInteraction(World& world, Entity player, InteractOffer offer) {
    if (!player.valid() || !world.alive(player)) return;
    InteractOffers* o = world.get<InteractOffers>(player);
    if (!o) {
        world.add<InteractOffers>(player, InteractOffers{});
        o = world.get<InteractOffers>(player);
    }
    o->list.push_back(std::move(offer));
}

bool takeInteractCommand(World& world, Entity player, const std::string& provider, InteractCommand& out) {
    if (!player.valid() || !world.alive(player)) return false;
    const InteractCommand* c = world.get<InteractCommand>(player);
    if (!c || c->provider != provider) return false;
    out = *c;
    world.remove<InteractCommand>(player);
    return true;
}

std::vector<RankedOffer> rankInteractions(const std::vector<InteractOffer>& offers, const Vec3& feet,
                                          const Vec3& eye, const Vec3& forward,
                                          bool (*blocked)(const Vec3&, const Vec3&, void*), void* blockedCtx) {
    std::vector<RankedOffer> out;
    bool anyExclusive = false;
    for (const InteractOffer& o : offers) anyExclusive = anyExclusive || o.exclusive;
    Vec3 f = forward;
    const Real fl = f.length();
    f = fl > 1e-6 ? f * (1.0 / fl) : Vec3(0, 0, -1);
    for (std::size_t i = 0; i < offers.size(); ++i) {
        const InteractOffer& o = offers[i];
        if (anyExclusive && !o.exclusive) continue;
        if (o.tap.empty() && o.hold.empty()) continue;
        if (o.exclusive) { out.push_back({i, -100.0}); continue; }
        const Vec3 toA = o.anchor - eye;
        const Real dEye = toA.length();
        const Real look = dEye > 1e-6 ? (toA.x * f.x + toA.y * f.y + toA.z * f.z) / dEye : 1.0;
        const Real dFeet = Vec3(o.anchor.x - feet.x, 0, o.anchor.z - feet.z).length();
        if (!o.inVolume) {
            if (dFeet > o.reach) continue;
            if (std::fabs(o.anchor.y - feet.y) > 2.2) continue;   // another floor
            // FACING it, side to side: a seat a metre away sits well below the eye, so looking a little down puts
            // even one BEHIND you within a 3D cone -- the turn is what says what you mean, the pitch only ranks.
            const Real hx = o.anchor.x - eye.x, hz = o.anchor.z - eye.z, hl = std::sqrt(hx * hx + hz * hz);
            const Real fh = std::sqrt(f.x * f.x + f.z * f.z);
            const Real facing = hl > 1e-6 && fh > 1e-6 ? (hx * f.x + hz * f.z) / (hl * fh) : 1.0;
            if (dFeet > 0.8 && facing < 0.5) continue;            // not what you are looking at
            if (o.needsSight && blocked && blocked(eye, o.anchor, blockedCtx)) continue;
        }
        // The LOOK counts most (the angle off the view's centre), then the distance; standing in a volume is a
        // strong claim.
        Real score = (1.0 - std::max(Real(-1), std::min(Real(1), look))) * 4.0 + dFeet * 0.6;
        if (o.inVolume) score -= 1.5;
        out.push_back({i, score});
    }
    std::sort(out.begin(), out.end(), [](const RankedOffer& a, const RankedOffer& b) { return a.score < b.score; });
    return out;
}

bool projectToScreen(const Vec3& p, const Vec3& pos, const Vec3& target, const Vec3& up, Real fovDeg, Real aspect,
                     Real width, Real height, Real& sx, Real& sy) {
    Vec3 fw = target - pos;
    const Real fl = fw.length();
    if (fl < 1e-9) return false;
    fw = fw * (1.0 / fl);
    Vec3 s = cross(fw, up);
    const Real sl = s.length();
    if (sl < 1e-9) return false;
    s = s * (1.0 / sl);
    const Vec3 u = cross(s, fw);
    const Vec3 d = p - pos;
    const Real z = d.x * fw.x + d.y * fw.y + d.z * fw.z;
    if (z < 0.05) return false;
    const Real x = d.x * s.x + d.y * s.y + d.z * s.z, y = d.x * u.x + d.y * u.y + d.z * u.z;
    const Real t = std::tan(fovDeg * 0.5 * 3.14159265358979323846 / 180.0);
    const Real nx = x / (z * t * aspect), ny = y / (z * t);
    sx = (nx + 1) * 0.5 * width;
    sy = (1 - ny) * 0.5 * height;
    return true;
}

}  // namespace engine
