#include "player_hands.h"
#include "player_body.h"

#include <algorithm>
#include <cmath>

namespace engine {

void PlayerHands::aimAndFire(PlayerBody& b, BodyStep& s) {
    if (holdingGrapple()) aim = findGrappleAnchor(s.pw, s.t.position + Vec3(0, b.eyeHeight, 0), s.in.look, b.tune);
    else aim.ok = false;
    if (s.in.fire) {
        s.in.fire = false;
        if (holdingGrapple() && !b.water.swimming && !b.is(b.grapple) && (hook == Hook::None || hook == Hook::Loose))
            fire(b, s);
    }
}

void PlayerHands::fire(PlayerBody& b, BodyStep& s) {
    // at what it was aimed at, or (nothing in reach) out to the end of the line, to come back loose
    const Vec3 hand = s.t.position + Vec3(0, s.cc.halfHeight + s.cc.radius * 0.5, 0);
    hookWillHold = aim.ok;
    hookTarget = aim.ok ? aim.point : s.t.position + Vec3(0, b.eyeHeight, 0) + s.in.look * b.tune.grappleRange;
    hookTip = hand;
    hook = Hook::Flying;
    rope.reset(hand);
    ++hookShots;
    b.event(aim.ok ? "hook out" : "nothing to hook");
}

// THE HOOK AND THE ROPE YOU SEE, every step whatever the body is doing (PlayerBody::publish runs it)
void PlayerHands::stepHook(PlayerBody& b, BodyStep& s) {
    constexpr Real kReelBack = 35.0;   // m/s a let-go rope comes back in
    const Real dt = s.dt;
    const Vec3 hand = s.t.position + Vec3(0, s.cc.halfHeight + s.cc.radius * 0.5, 0);
    if (hook == Hook::Flying) {
        const Vec3 to = hookTarget - hookTip;
        const Real d = to.length(), stepLen = b.tune.grappleHookSpeed * dt;
        if (!s.in.fireHeld) {
            hook = Hook::Loose;   // let go before it got there: it comes back
            looseLen = (hookTip - hand).length();
        } else if (d <= stepLen) {
            hookTip = hookTarget;
            if (hookWillHold) ++hookBites;
            if (hookWillHold && b.attachRope(s, hookTarget)) {
                hook = Hook::Hooked;
            } else {
                hook = Hook::Loose;
                looseLen = (hookTip - hand).length();
            }
        } else {
            hookTip = hookTip + to * (stepLen / d);
        }
    }
    if (hook == Hook::Hooked && !b.is(b.grapple)) {   // let go, pulled up over the edge, gone limp...
        hook = Hook::Loose;
        looseLen = std::max(b.grapple.ropeLen, (b.grapple.anchor - hand).length());
    }
    if (hook == Hook::Loose) {
        looseLen -= kReelBack * dt;
        if (looseLen <= 0.2) {
            hook = Hook::None;
            rope.clear();
        }
    }
    if (hook == Hook::None) return;
    const PhysicsWorld& pw = s.pw;
    const RopeLine::Sweep sweep = [&pw](const Vec3& from, const Vec3& to, Vec3& hit, Vec3& normal) {
        const Vec3 d = to - from;
        if (d.lengthSquared() < 1e-8) return false;
        PhysicsWorld::RayHit h;
        if (!pw.castRayHit(from, d, h)) return false;
        hit = h.point;
        normal = h.normal;
        return true;
    };
    if (hook == Hook::Flying) {
        // paying out behind the hook, a little slack in it so it snakes
        rope.step(hand, hookTip, false, (hookTip - hand).length() * 1.05 + 0.3, dt, sweep);
    } else if (hook == Hook::Hooked) {
        rope.step(hand, b.grapple.anchor, false, b.grapple.ropeLen, dt, sweep);
    } else {
        rope.step(hand, Vec3(0, 0, 0), true, looseLen, dt, sweep);
    }
}

}  // namespace engine
