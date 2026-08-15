#include "hand_interaction_system.h"

#include <cmath>

#include "../../log.h"
#include "../mesh_builder.h"
#include "../xr/xr_touch.h"
#include "physics_system.h"

namespace engine {

namespace {

// Grab reach, measured to the object's SURFACE (nearest point on its
// oriented shape): forgiving of centimetre fingertip wander, but the pick
// region has exactly the object's shape — a lying pillar grabs along its
// whole lying length.
constexpr Real kGrabSurfaceReach = 0.10;
// Affordance range: candidates inside this of the pinch point show the
// oriented highlight + fingertip projection cues.
constexpr Real kApproachRange = 0.35;
constexpr size_t kMaxObjects = 32;
constexpr Real kPalmRadius = 0.035;
constexpr Real kGripAnchorRadius = 0.015;
// Closure release: opening past XrGripMemory::kReleaseOpening must persist
// this long — a single noisy frame must not drop the object. Short, because
// during a toss every extra ms of hold lets the spring drag the object back
// down with the decelerating hand ("sticky"; device round two). A DECISIVE
// open (1.5x the threshold) releases the same frame.
constexpr Real kOpenDwell = 0.05;
// Anti-restick: after a release the object separates from the hand while
// the fingers are still near it for a few frames; closure must not re-hook
// a shape that is RECEDING from the grip faster than this (a toss leaving,
// not a regrip). Catches are unaffected — a caught object approaches.
constexpr Real kRecedeSpeed = 0.25;
// ...and a hand that just let go of a shape leaves it alone briefly — the
// frame after a throw the relative velocity is still ~0 (the spring had
// just been tracking), so the recede guard alone can't catch it. Same hand
// + same shape only: hand-to-hand tosses and throw-up-and-catch (the ball
// is gone and back ~0.5s later) never notice.
constexpr Real kRehookDelay = 0.12;
// Throw restore: hand-tracked releases fire LATE (finger extension lags
// real fingers at speed; fast swings drop tracking outright), so by
// release the arm — and the spring-held object — has decelerated. If the
// hand's PEAK recent speed beats the object's at release, the peak is what
// the throw actually was: restore it. Set-downs (peak below the floor) are
// untouched, so the placement assist still sees them.
constexpr Real kThrowRestoreFloor = 0.7;
// A tracking dropout while the grip moved faster than this is a throw
// losing its hand mid-swing: release NOW with the peak, instead of
// coasting the hold through the throw and dragging the object back down.
constexpr Real kDropoutThrowSpeed = 1.0;
// Fingers EXTENDING fast is release intent — fire early (at 60% of the
// opening threshold) instead of waiting for the full drift.
constexpr Real kOpeningVelRelease = 0.5;
// A release slower than this is a deliberate set-down: eligible for the
// placement-assist snap. Anything faster is a throw/drop — untouched.
constexpr Real kGentleRelease = 0.25;
// The other hand hooking the same object within this window logs as a toss.
constexpr Real kTossWindow = 0.75;
// Closure hook: fingertip JOINTS this close to the surface count as "on it"
// — just past the capsule radius (9-11mm), so the hook fires at first touch,
// BEFORE the kinematic squeeze that would eject the object.
constexpr Real kClosureReach = 0.014;
// Looser probe for seeding a pinch-hook's grip memory.
constexpr Real kPinchSeedReach = 0.03;

constexpr XrHandJointId kTipJoints[] = {
    XR_JOINT_THUMB_TIP, XR_JOINT_INDEX_TIP, XR_JOINT_MIDDLE_TIP,
    XR_JOINT_RING_TIP, XR_JOINT_LITTLE_TIP};

Real quatAngleBetween(const Quat& a, const Quat& b) {
    const Real d =
        std::min(Real(1), std::fabs(a.normalized().dot(b.normalized())));
    return 2 * std::acos(d);
}

XrPalette::Config paletteConfig(int itemCount) {
    XrPalette::Config c;
    c.itemCount = itemCount;
    return c;
}

}  // namespace

// The spawnable inventory. Every entry's collider matches its render mesh
// exactly, which keeps grab/rest behaviour honest. The chest and bolt are
// the articulated demos: two bodies + a joint, both independently hookable
// — hold the chest, flip its lid; pull the bolt to its stop and let the
// limit spring snap it back.
const HandInteractionSystem::ItemDef HandInteractionSystem::kItems[] = {
    {"crate", Vec3(0.075, 0.075, 0.075), Vec3(0.80, 0.52, 0.25), false,
     JointKind::None, Vec3(), Vec3(), Vec3(), Vec3(), Vec3(1, 0, 0), 0, 0, 0},
    {"ball", Vec3(0.075, 0.075, 0.075), Vec3(0.20, 0.55, 0.95), true,
     JointKind::None, Vec3(), Vec3(), Vec3(), Vec3(), Vec3(1, 0, 0), 0, 0, 0},
    {"slab", Vec3(0.15, 0.025, 0.10), Vec3(0.55, 0.55, 0.60), false,
     JointKind::None, Vec3(), Vec3(), Vec3(), Vec3(), Vec3(1, 0, 0), 0, 0, 0},
    {"pillar", Vec3(0.04, 0.18, 0.04), Vec3(0.85, 0.82, 0.70), false,
     JointKind::None, Vec3(), Vec3(), Vec3(), Vec3(), Vec3(1, 0, 0), 0, 0, 0},
    {"wedge", Vec3(0.10, 0.05, 0.075), Vec3(0.30, 0.75, 0.40), false,
     JointKind::None, Vec3(), Vec3(), Vec3(), Vec3(), Vec3(1, 0, 0), 0, 0, 0},
    // Chest: crate + lid hinged along the top back edge, 0..110 degrees,
    // soft stops. The 3mm gap keeps the resting lid off the chest's top
    // face so friction never fights the hinge.
    {"chest", Vec3(0.09, 0.05, 0.065), Vec3(0.55, 0.35, 0.18), false,
     JointKind::Hinge, Vec3(0.09, 0.012, 0.065), Vec3(0.70, 0.48, 0.26),
     Vec3(0, 0.065, 0), Vec3(0, 0.053, -0.065), Vec3(1, 0, 0),
     0.0, 1.92, 2.0},
    // Bolt: base + knob on a 6cm slider with soft detent ends — the
    // rack-the-slide interaction.
    {"bolt", Vec3(0.09, 0.025, 0.03), Vec3(0.45, 0.47, 0.52), false,
     JointKind::Slider, Vec3(0.028, 0.018, 0.03), Vec3(0.85, 0.30, 0.20),
     Vec3(-0.03, 0.046, 0), Vec3(-0.03, 0.046, 0), Vec3(1, 0, 0),
     0.0, 0.06, 6.0},
    // Crank: a post with a paddle on a FREE hinge (full-turn limits),
    // mounted beside the post so the sweep clears it — grab the paddle end
    // and spin it like a propeller.
    {"crank", Vec3(0.03, 0.10, 0.03), Vec3(0.40, 0.42, 0.48), false,
     JointKind::Hinge, Vec3(0.10, 0.015, 0.02), Vec3(0.90, 0.60, 0.15),
     Vec3(0, 0.10, 0.054), Vec3(0, 0.10, 0.054), Vec3(0, 0, 1),
     -PI, PI, 0.0},
    // Seesaw: a plank balanced on a small base, tilting +/-26 degrees —
    // press one end down (finger or a dropped crate) and the other rises.
    {"seesaw", Vec3(0.04, 0.03, 0.04), Vec3(0.35, 0.30, 0.28), false,
     JointKind::Hinge, Vec3(0.14, 0.012, 0.03), Vec3(0.75, 0.68, 0.35),
     Vec3(0, 0.045, 0), Vec3(0, 0.045, 0), Vec3(0, 0, 1),
     -0.45, 0.45, 0.0},
};
const int HandInteractionSystem::kItemCount =
    static_cast<int>(sizeof(HandInteractionSystem::kItems) /
                     sizeof(HandInteractionSystem::kItems[0]));

HandInteractionSystem::HandInteractionSystem(PhysicsSystem* physics)
    : physics_(physics), palette_(paletteConfig(kItemCount)) {}

Vec3 HandInteractionSystem::handWorld(FrameContext& ctx,
                                      const Vec3& originPoint) const {
    // Sandbox scale is locked to 1, so ORIGIN -> WORLD is the base
    // translation alone (xr_state.h spaces contract).
    return ctx.xr.originBase + originPoint;
}

Quat HandInteractionSystem::handOrientation(const XrHand& hand) const {
    return Quat::fromRotationMatrix(
        hand.joints[XR_JOINT_WRIST].originFromJoint);
}

bool HandInteractionSystem::isHeld(int objIndex) const {
    for (int h = 0; h < 2; h++)
        if (hold_[h].active() && hold_[h].pick.obj == objIndex) return true;
    return false;
}

PhysicsBodyId HandInteractionSystem::shapeBody(const Pick& pick) const {
    const SandboxObject& obj = objects_[pick.obj];
    return pick.sub ? obj.subBody : obj.body;
}

Vec3 HandInteractionSystem::shapeHalfExtent(const Pick& pick) const {
    const ItemDef& def = kItems[objects_[pick.obj].item];
    return pick.sub ? def.subHalfExtent : def.halfExtent;
}

bool HandInteractionSystem::shapeIsSphere(const Pick& pick) const {
    return !pick.sub && kItems[objects_[pick.obj].item].sphere;
}

Real HandInteractionSystem::shapeSurfaceDistance(const Pick& pick,
                                                 const Vec3& p) const {
    const PhysicsBodyId body = shapeBody(pick);
    const Vec3 c = physics_->physicsWorld().bodyPosition(body);
    if (shapeIsSphere(pick))
        return xrSphereSurfaceDistance(shapeHalfExtent(pick).x, c, p);
    return xrBoxSurfaceDistance(shapeHalfExtent(pick), c,
                                physics_->physicsWorld().bodyOrientation(body),
                                p);
}

Vec3 HandInteractionSystem::shapeNearestPoint(const Pick& pick,
                                              const Vec3& p) const {
    const PhysicsBodyId body = shapeBody(pick);
    const Vec3 c = physics_->physicsWorld().bodyPosition(body);
    if (shapeIsSphere(pick))
        return xrNearestPointOnSphere(shapeHalfExtent(pick).x, c, p);
    return xrNearestPointOnBox(shapeHalfExtent(pick), c,
                               physics_->physicsWorld().bodyOrientation(body),
                               p);
}

HandInteractionSystem::Pick HandInteractionSystem::nearestGrabbable(
    int h, const Vec3& p, Real maxSurfaceDistance) const {
    Pick best;
    Real bestD = maxSurfaceDistance;
    for (size_t i = 0; i < objects_.size(); i++) {
        const int idx = static_cast<int>(i);
        if (hold_[h].active() && hold_[h].pick.obj == idx)
            continue;   // this hand already holds part of it
        Pick candidates[2] = {{idx, false}, {idx, true}};
        const int n = objects_[i].subBody != INVALID_PHYSICS_BODY ? 2 : 1;
        for (int c = 0; c < n; c++) {
            const Real d = shapeSurfaceDistance(candidates[c], p);
            if (d < bestD) {
                bestD = d;
                best = candidates[c];
            }
        }
    }
    return best;
}

std::vector<XrGripPoint> HandInteractionSystem::probeShape(
    FrameContext& ctx, int h, const Pick& pick, Real reach) const {
    std::vector<std::pair<int, Vec3>> tips;
    liveFingertips(ctx, h, tips);
    const PhysicsBodyId body = shapeBody(pick);
    return xrProbeGrip(tips, shapeHalfExtent(pick), shapeIsSphere(pick),
                       physics_->physicsWorld().bodyPosition(body),
                       physics_->physicsWorld().bodyOrientation(body), reach);
}

void HandInteractionSystem::applyHeldLayer(const Pick& pick) {
    physics_->physicsWorld().setBodyLayer(shapeBody(pick),
                                          PhysicsWorld::BodyLayer::Held);
    for (size_t i = 0; i < layerClear_.size(); i++) {
        if (layerClear_[i].pick.obj == pick.obj &&
            layerClear_[i].pick.sub == pick.sub) {
            layerClear_.erase(layerClear_.begin() + i);
            break;
        }
    }
}

void HandInteractionSystem::releaseHeldLayer(int releasingHand,
                                             const Pick& pick) {
    // Still spring-held by the other hand? The layer stays.
    const int other = 1 - releasingHand;
    if (hold_[other].active() && hold_[other].pick.obj == pick.obj &&
        hold_[other].pick.sub == pick.sub)
        return;
    layerClear_.push_back({pick, static_cast<Real>(timeSeconds_)});
}

void HandInteractionSystem::settleHeldLayers(FrameContext& ctx) {
    // Restore hand collision on released shapes once every fingertip has
    // cleared them (untracked hands have parked capsules — they can't
    // squeeze anything, so they don't hold the restore up). The clearance
    // is just past the capsule radius — only actual OVERLAP ejects — and a
    // timeout backstops it: holding the chest keeps fingers permanently
    // near its released lid, and a lid the hand can never touch again is
    // worse than one brief nudge (device round two: "can't use the back of
    // my palm to close that lidded box").
    constexpr Real kClearance = 0.012;
    constexpr Real kClearTimeout = 1.0;
    for (size_t i = 0; i < layerClear_.size();) {
        const Pick& pick = layerClear_[i].pick;
        bool clear = timeSeconds_ - layerClear_[i].since > kClearTimeout;
        if (!clear) {
            clear = true;
            for (int h = 0; h < 2 && clear; h++) {
                if (!handActive_[h]) continue;
                std::vector<std::pair<int, Vec3>> tips;
                liveFingertips(ctx, h, tips);
                for (const auto& tip : tips) {
                    if (shapeSurfaceDistance(pick, tip.second) < kClearance) {
                        clear = false;
                        break;
                    }
                }
            }
        }
        if (clear) {
            physics_->physicsWorld().setBodyLayer(
                shapeBody(pick), PhysicsWorld::BodyLayer::Default);
            layerClear_.erase(layerClear_.begin() + i);
        } else {
            i++;
        }
    }
}

void HandInteractionSystem::liveFingertips(
    FrameContext& ctx, int h, std::vector<std::pair<int, Vec3>>& out) const {
    out.clear();
    const XrHand& hand = ctx.xr.hands[h];
    if (!hand.tracked) return;
    for (const XrHandJointId tip : kTipJoints) {
        if (!hand.joints[tip].tracked) continue;
        out.emplace_back(static_cast<int>(tip),
                         handWorld(ctx, xrJointPosition(hand, tip)));
    }
}

MeshHandle HandInteractionSystem::itemMesh(FrameContext& ctx, int item) {
    if (!itemMeshes_[item].valid()) {
        const ItemDef& def = kItems[item];
        itemMeshes_[item] = ctx.renderer.uploadMesh(
            def.sphere ? MeshBuilder::sphere(static_cast<float>(def.halfExtent.x))
                       : MeshBuilder::box(def.halfExtent * 2.0));
    }
    return itemMeshes_[item];
}

MeshHandle HandInteractionSystem::subMesh(FrameContext& ctx, int item) {
    if (!subMeshes_[item].valid()) {
        subMeshes_[item] = ctx.renderer.uploadMesh(
            MeshBuilder::box(kItems[item].subHalfExtent * 2.0));
    }
    return subMeshes_[item];
}

// Room for one more: recycle the oldest FREE object, never a held one.
bool HandInteractionSystem::makeRoom() {
    if (objects_.size() < kMaxObjects) return true;
    for (size_t i = 0; i < objects_.size(); i++) {
        if (isHeld(static_cast<int>(i))) continue;
        SandboxObject& obj = objects_[i];
        if (obj.jointId != INVALID_CONSTRAINT)
            physics_->physicsWorld().removeConstraint(obj.jointId);
        if (obj.subBody != INVALID_PHYSICS_BODY)
            physics_->physicsWorld().removeBody(obj.subBody);
        physics_->physicsWorld().removeBody(obj.body);
        objects_.erase(objects_.begin() + i);
        for (int h = 0; h < 2; h++)   // reindex held slots past the hole
            if (hold_[h].active() && hold_[h].pick.obj > static_cast<int>(i))
                hold_[h].pick.obj--;
        for (size_t k = 0; k < layerClear_.size();) {   // and pending clears
            if (layerClear_[k].pick.obj == static_cast<int>(i)) {
                layerClear_.erase(layerClear_.begin() + k);   // body removed
                continue;
            }
            if (layerClear_[k].pick.obj > static_cast<int>(i))
                layerClear_[k].pick.obj--;
            k++;
        }
        for (int h = 0; h < 2; h++) {
            if (recentUnhook_[h].obj == static_cast<int>(i))
                recentUnhook_[h] = Pick{};
            else if (recentUnhook_[h].obj > static_cast<int>(i))
                recentUnhook_[h].obj--;
        }
        lastUnhookObj_ = -1;
        return true;
    }
    return false;   // everything held + cap (unreachable with two hands)
}

void HandInteractionSystem::hook(FrameContext& ctx, int h, const Pick& pick,
                                 bool pinchHold,
                                 const std::vector<XrGripPoint>& contacts) {
    if (gripBody_[h] == INVALID_PHYSICS_BODY) return;
    const PhysicsBodyId body = shapeBody(pick);
    const ConstraintId spring =
        physics_->physicsWorld().addGripSpring(gripBody_[h], body);
    if (spring == INVALID_CONSTRAINT) return;

    Hold& hold = hold_[h];
    hold.pick = pick;
    hold.spring = spring;
    hold.pinchHold = pinchHold;
    hold.openSince = -1;
    applyHeldLayer(pick);

    // Grip memory: the touching fingertips, object-local. A pinch that
    // landed with no capsule contacts yet remembers the thumb and index
    // tips projected onto the surface instead (their press directions
    // oppose across the object — the same "each side" shape).
    const Vec3 objP = physics_->physicsWorld().bodyPosition(body);
    const Quat objQ = physics_->physicsWorld().bodyOrientation(body);
    std::vector<XrGripPoint> points = contacts;
    if (points.empty()) {
        const XrHand& hand = ctx.xr.hands[h];
        const XrHandJointId tips[2] = {XR_JOINT_THUMB_TIP, XR_JOINT_INDEX_TIP};
        for (const XrHandJointId tip : tips) {
            if (!hand.joints[tip].tracked) continue;
            const Vec3 tipWorld = handWorld(ctx, xrJointPosition(hand, tip));
            const Vec3 onSurface = shapeNearestPoint(pick, tipWorld);
            Vec3 press = onSurface - tipWorld;
            if (press.length() < 1e-6) press = objP - tipWorld;
            if (press.length() < 1e-6) continue;
            points.push_back({static_cast<int>(tip), onSurface,
                              press * (1.0 / press.length())});
        }
    }
    hold.grip.capture(points, objP, objQ);

    const char* name = kItems[objects_[pick.obj].item].name;
    if (lastUnhookObj_ == pick.obj && lastUnhookHand_ != h &&
        timeSeconds_ - lastUnhookTime_ < kTossWindow) {
        LOG_INFO("[xr] toss %s->%s %s", lastUnhookHand_ == 0 ? "L" : "R",
                 h == 0 ? "L" : "R", name);
    }
    LOG_INFO("[xr] hook %s %s%s (%s, %zu contacts)", h == 0 ? "L" : "R",
             name, pick.sub ? ":sub" : "", pinchHold ? "pinch" : "closure",
             hold.grip.contactCount());
}

void HandInteractionSystem::unhook(FrameContext& ctx, int h) {
    (void)ctx;
    Hold& hold = hold_[h];
    if (!hold.active()) return;
    const PhysicsBodyId body = shapeBody(hold.pick);
    physics_->physicsWorld().removeConstraint(hold.spring);

    // The object keeps its momentum — but a hand-tracked release fires
    // LATE, after the arm decelerated, so a throw's true speed is the
    // hand's recent PEAK, not the object's current velocity. Restore it
    // when it beats what the object has (set-downs stay below the floor
    // and keep their gentle velocity for the placement assist).
    Vec3 v = physics_->physicsWorld().getLinearVelocity(body);
    const Vec3 peak = handMemory_[h].peakVelocity(0.15);
    if (peak.length() > kThrowRestoreFloor && peak.length() > v.length()) {
        physics_->physicsWorld().setLinearVelocity(body, peak);
        v = peak;
    }
    bool placed = false;
    if (placeAssist_ && !hold.pick.sub && !shapeIsSphere(hold.pick) &&
        v.length() < kGentleRelease) {
        const XrAlignedPose target = placementTarget(hold.pick);
        if (target.valid) {
            physics_->physicsWorld().teleport(body, target.position,
                                              target.orientation);
            placed = true;
        }
    }

    const char* name = kItems[objects_[hold.pick.obj].item].name;
    LOG_INFO("[xr] unhook %s %s%s v=%.2fm/s%s", h == 0 ? "L" : "R", name,
             hold.pick.sub ? ":sub" : "", v.length(),
             placed ? " (placed aligned)" : "");

    lastUnhookObj_ = hold.pick.obj;
    lastUnhookHand_ = h;
    lastUnhookTime_ = timeSeconds_;
    recentUnhook_[h] = hold.pick;
    recentUnhookAt_[h] = timeSeconds_;
    releaseHeldLayer(h, hold.pick);
    hold = Hold{};
    consumePinchUntil_ = timeSeconds_ + 0.4;
}

XrAlignedPose HandInteractionSystem::placementTarget(const Pick& pick) const {
    // Supports, best first: another resting box whose orientation is
    // near-axis-aligned (the Jenga stack), else whatever a short downward
    // ray finds (table, floor — flush + axis-aligned, unbounded rect).
    const PhysicsBodyId body = shapeBody(pick);
    const Vec3 he = shapeHalfExtent(pick);
    const Vec3 pos = physics_->physicsWorld().bodyPosition(body);
    const Quat q = physics_->physicsWorld().bodyOrientation(body);

    XrAlignedPose best;
    Real bestDrop = 1e9;
    for (size_t i = 0; i < objects_.size(); i++) {
        const int idx = static_cast<int>(i);
        if (idx == pick.obj || isHeld(idx)) continue;
        const ItemDef& def = kItems[objects_[i].item];
        if (def.sphere) continue;
        const Quat oq =
            physics_->physicsWorld().bodyOrientation(objects_[i].body);
        if (quatAngleBetween(oq, xrNearestAxisAligned(oq)) > 0.1) continue;
        const Vec3 oc = physics_->physicsWorld().bodyPosition(objects_[i].body);
        const Mat4 m = oq.toMat4();
        auto ext = [&](int row) {
            return std::fabs(m.m[row][0]) * def.halfExtent.x +
                   std::fabs(m.m[row][1]) * def.halfExtent.y +
                   std::fabs(m.m[row][2]) * def.halfExtent.z;
        };
        const Real topY = oc.y + ext(1);
        if (topY > pos.y) continue;   // support must be below the centre
        const XrAlignedPose a = xrAlignToSupport(
            he, pos, q, topY, Vec3(oc.x - ext(0), 0, oc.z - ext(2)),
            Vec3(oc.x + ext(0), 0, oc.z + ext(2)));
        if (a.valid && pos.y - topY < bestDrop) {
            bestDrop = pos.y - topY;
            best = a;
        }
    }
    if (best.valid) return best;

    Vec3 hit;
    if (physics_->physicsWorld().castRay(pos, Vec3(0, -0.3, 0), hit)) {
        return xrAlignToSupport(he, pos, q, hit.y,
                                Vec3(pos.x - 10, 0, pos.z - 10),
                                Vec3(pos.x + 10, 0, pos.z + 10));
    }
    return XrAlignedPose{};
}

void HandInteractionSystem::spawnIntoHand(FrameContext& ctx, int item,
                                          int hand) {
    if (!makeRoom()) return;
    if (gripBody_[hand] == INVALID_PHYSICS_BODY) return;

    const ItemDef& def = kItems[item];
    const Vec3 at = handWorld(ctx, pinch_[hand].pinchPoint());
    const Quat q = handOrientation(ctx.xr.hands[hand]);

    SandboxObject obj;
    obj.item = item;
    obj.body = def.sphere
        ? physics_->physicsWorld().addSphere(def.halfExtent.x, at, q,
                                             BodyMotion::Dynamic)
        : physics_->physicsWorld().addBox(def.halfExtent, at, q,
                                          BodyMotion::Dynamic);
    if (obj.body == INVALID_PHYSICS_BODY) return;
    obj.posPrev = obj.posCurr = at;
    obj.quatPrev = obj.quatCurr = q;

    if (def.joint != JointKind::None) {
        const Vec3 subAt = at + q.rotate(def.subOffset);
        obj.subBody = physics_->physicsWorld().addBox(def.subHalfExtent, subAt,
                                                      q, BodyMotion::Dynamic);
        if (obj.subBody != INVALID_PHYSICS_BODY) {
            const Vec3 pivot = at + q.rotate(def.pivotOffset);
            const Vec3 axis = q.rotate(def.axis);
            obj.jointId =
                def.joint == JointKind::Hinge
                    ? physics_->physicsWorld().addHinge(
                          obj.body, obj.subBody, pivot, axis, def.minLimit,
                          def.maxLimit, def.jointSpring)
                    : physics_->physicsWorld().addSlider(
                          obj.body, obj.subBody, pivot, axis, def.minLimit,
                          def.maxLimit, def.jointSpring);
        }
        obj.subPosPrev = obj.subPosCurr = subAt;
        obj.subQuatPrev = obj.subQuatCurr = q;
    }

    objects_.push_back(obj);
    const int idx = static_cast<int>(objects_.size()) - 1;
    LOG_INFO("[xr] spawned %s into %s hand (%zu objects)", def.name,
             hand == 0 ? "left" : "right", objects_.size());
    hook(ctx, hand, Pick{idx, false}, /*pinchHold=*/true, {});
}

void HandInteractionSystem::updateHandPresence(FrameContext& ctx) {
    // The articulated hand: a kinematic capsule per finger bone plus a palm
    // sphere on the HAND layer (shoves dynamic bodies, passes through the
    // room's static mesh), driven from adaptively-filtered joints — the
    // physical presence that lets an open hand shove, backhand, and scatter
    // blocks with finger-shaped contacts. Kinematic drive means Jolt derives
    // real velocities: a swing imparts a swing's impulse.
    const Real dt = ctx.frameDelta;
    const Real moveDt = std::max(ctx.frameDelta, 1.0 / 240.0);
    for (int h = 0; h < 2; h++) {
        const XrHand& hand = ctx.xr.hands[h];
        if (!hand.tracked) {
            if (handActive_[h]) {
                // Park far below the room; teleport (not a kinematic sweep
                // — that would batter everything on the way down).
                const Vec3 park =
                    ctx.xr.originBase + Vec3(h == 0 ? -2 : 2, -100, 0);
                for (HandBone& bone : handBones_[h])
                    physics_->physicsWorld().teleport(bone.body, park,
                                                      Quat::identity());
                if (palmBody_[h] != INVALID_PHYSICS_BODY)
                    physics_->physicsWorld().teleport(palmBody_[h], park,
                                                      Quat::identity());
                handActive_[h] = false;
            }
            continue;
        }

        // Filter every finger joint + wrist (position only; capsule
        // orientation derives from the filtered endpoints).
        const bool reacquired = !handActive_[h];
        for (int j = XR_JOINT_WRIST; j < XR_HAND_JOINT_COUNT; j++) {
            if (!hand.joints[j].tracked) continue;
            const Vec3 target = handWorld(ctx, xrJointPosition(
                hand, static_cast<XrHandJointId>(j)));
            if (reacquired)
                jointFilter_[h][j].reset(target, Quat::identity());
            else
                jointFilter_[h][j].feed(target, Quat::identity(), dt);
        }

        // Build the bone set once, at first sighting: every finger joint
        // whose parent is another finger joint (the wrist-rooted metacarpal
        // bones span the palm — the palm sphere covers those).
        if (handBones_[h].empty()) {
            for (int j = XR_JOINT_THUMB_KNUCKLE; j < XR_HAND_JOINT_COUNT;
                 j++) {
                const int parent = xrHandJointParent(j);
                if (parent < XR_JOINT_THUMB_KNUCKLE) continue;
                if (!hand.joints[j].tracked || !hand.joints[parent].tracked)
                    continue;
                HandBone bone;
                bone.jointA = parent;
                bone.jointB = j;
                bone.radius = (j <= XR_JOINT_THUMB_TIP) ? 0.011 : 0.009;
                const XrBoneCapsule c = xrBoneCapsule(
                    jointFilter_[h][parent].position(),
                    jointFilter_[h][j].position(), bone.radius);
                bone.body = physics_->physicsWorld().addCapsule(
                    c.halfHeight, bone.radius, c.center, c.orientation,
                    BodyMotion::Kinematic);
                if (bone.body != INVALID_PHYSICS_BODY) {
                    physics_->physicsWorld().setBodyLayer(
                        bone.body, PhysicsWorld::BodyLayer::Hand);
                    handBones_[h].push_back(bone);
                }
            }
        }
        if (palmBody_[h] == INVALID_PHYSICS_BODY &&
            hand.joints[XR_JOINT_WRIST].tracked) {
            palmBody_[h] = physics_->physicsWorld().addSphere(
                kPalmRadius, jointFilter_[h][XR_JOINT_WRIST].position(),
                Quat::identity(), BodyMotion::Kinematic);
            if (palmBody_[h] != INVALID_PHYSICS_BODY)
                physics_->physicsWorld().setBodyLayer(
                    palmBody_[h], PhysicsWorld::BodyLayer::Hand);
        }

        for (const HandBone& bone : handBones_[h]) {
            if (!hand.joints[bone.jointA].tracked ||
                !hand.joints[bone.jointB].tracked)
                continue;
            const XrBoneCapsule c = xrBoneCapsule(
                jointFilter_[h][bone.jointA].position(),
                jointFilter_[h][bone.jointB].position(), bone.radius);
            if (reacquired)
                physics_->physicsWorld().teleport(bone.body, c.center,
                                                  c.orientation);
            else
                physics_->physicsWorld().moveKinematic(bone.body, c.center,
                                                       c.orientation, moveDt);
        }
        if (palmBody_[h] != INVALID_PHYSICS_BODY &&
            hand.joints[XR_JOINT_WRIST].tracked) {
            const Vec3 p = jointFilter_[h][XR_JOINT_WRIST].position();
            if (reacquired)
                physics_->physicsWorld().teleport(palmBody_[h], p,
                                                  Quat::identity());
            else
                physics_->physicsWorld().moveKinematic(palmBody_[h], p,
                                                       Quat::identity(),
                                                       moveDt);
        }
        handActive_[h] = true;
    }
}

void HandInteractionSystem::updatePalette(FrameContext& ctx) {
    XrPalette::Inputs in;
    for (int h = 0; h < 2; h++) {
        in.palm[h] = xrPalmPose(ctx.xr.hands[h], h == 0);
        if (in.palm[h].valid)
            in.palm[h].position = handWorld(ctx, in.palm[h].position);
        in.handBusy[h] = hold_[h].active();
        const XrHand& hand = ctx.xr.hands[h];
        in.fingertipValid[h] =
            hand.tracked && hand.joints[XR_JOINT_INDEX_TIP].tracked;
        if (in.fingertipValid[h])
            in.fingertip[h] =
                handWorld(ctx, xrJointPosition(hand, XR_JOINT_INDEX_TIP));
    }
    palette_.update(in, timeSeconds_);

    if (palette_.shownEdge())
        LOG_INFO("[xr] palette shown (%s palm)",
                 palette_.anchorHand() == 0 ? "left" : "right");
    if (palette_.hiddenEdge()) LOG_INFO("[xr] palette hidden");
    if (palette_.hoverChanged() && palette_.hoverItem() >= 0)
        LOG_INFO("[xr] palette hover %s", kItems[palette_.hoverItem()].name);

    // Free-hand pinch on a hovered item spawns it into that hand. Grips ran
    // first this frame, so a pinch that landed near an existing object
    // already took it and set handBusy.
    const int anchor = palette_.anchorHand();
    if (anchor >= 0 && palette_.hoverItem() >= 0) {
        const int free = 1 - anchor;
        if (pinch_[free].began() && !hold_[free].active()) {
            spawnIntoHand(ctx, palette_.hoverItem(), free);
            consumePinchUntil_ = timeSeconds_ + 0.4;
        }
    }
}

void HandInteractionSystem::updateGrips(FrameContext& ctx) {
    const Real moveDt = std::max(ctx.frameDelta, 1.0 / 240.0);
    std::vector<std::pair<int, Vec3>> tips;

    for (int h = 0; h < 2; h++) {
        const XrHand& hand = ctx.xr.hands[h];
        const bool tracked =
            hand.tracked && hand.joints[XR_JOINT_WRIST].tracked;

        // The grip frame (pinch point + wrist orientation) runs through
        // HAND MEMORY — a mid-throw dropout coasts on the fitted velocity
        // instead of freezing the carried object — then the adaptive filter.
        Vec3 rawP(0, 0, 0);
        Quat rawQ;
        if (tracked) {
            rawP = handWorld(ctx, pinch_[h].pinchPoint());
            rawQ = handOrientation(hand);
        }
        const bool wasCoasting = handMemory_[h].coasting();
        handMemory_[h].feed(tracked, rawP, rawQ, timeSeconds_);
        if (!handMemory_[h].available()) continue;

        // A dropout at speed while holding is a throw losing its hand
        // mid-swing (fast motion is exactly when tracking drops): release
        // NOW — unhook restores the peak velocity — instead of coasting
        // the hold through the throw and dragging the object back down
        // with the reacquired, already-decelerated hand.
        if (!wasCoasting && handMemory_[h].coasting() && hold_[h].active() &&
            handMemory_[h].peakVelocity(0.15).length() > kDropoutThrowSpeed) {
            unhook(ctx, h);
        }

        if (gripBody_[h] == INVALID_PHYSICS_BODY) {
            gripBody_[h] = physics_->physicsWorld().addSphere(
                kGripAnchorRadius, handMemory_[h].position(),
                handMemory_[h].orientation(), BodyMotion::Kinematic);
            if (gripBody_[h] == INVALID_PHYSICS_BODY) continue;
            physics_->physicsWorld().setBodyLayer(
                gripBody_[h], PhysicsWorld::BodyLayer::Grip);
            carry_[h].reset(handMemory_[h].position(),
                            handMemory_[h].orientation());
        }
        carry_[h].feed(handMemory_[h].position(), handMemory_[h].orientation(),
                       ctx.frameDelta);
        physics_->physicsWorld().moveKinematic(
            gripBody_[h], carry_[h].position(), carry_[h].orientation(),
            moveDt);

        Hold& hold = hold_[h];
        if (hold.active()) {
            if (hold.pick.obj >= static_cast<int>(objects_.size())) {
                hold = Hold{};   // object vanished under us (should not
                continue;        // happen: makeRoom skips held objects)
            }
            if (hold.pinchHold) {
                if (pinch_[h].released()) unhook(ctx, h);
            } else {
                // Closure release: live fingertips off their remembered
                // anchors past hysteresis. A brief dwell filters single
                // noisy frames, but a DECISIVE open (1.5x) releases the
                // same frame — a toss must let go at the apex, not 150ms
                // after (the spring drags the object back down with the
                // decelerating hand otherwise). Untracked tips are skipped
                // and no-data reads as closed (opening() contract), so a
                // dropout mid-hold coasts instead of dropping.
                const PhysicsBodyId body = shapeBody(hold.pick);
                liveFingertips(ctx, h, tips);
                const Real open = hold.grip.opening(
                    tips, physics_->physicsWorld().bodyPosition(body),
                    physics_->physicsWorld().bodyOrientation(body));
                // Fingers EXTENDING fast is release intent (opening rate,
                // not just magnitude) — reported finger poses lag real
                // fingers at speed, so waiting for the full drift releases
                // after the throw is over.
                const Real openVel =
                    ctx.frameDelta > 1e-4
                        ? (open - hold.lastOpening) / ctx.frameDelta
                        : 0.0;
                hold.lastOpening = open;
                if (open > XrGripMemory::kReleaseOpening * 1.5 ||
                    (open > XrGripMemory::kReleaseOpening * 0.6 &&
                     openVel > kOpeningVelRelease)) {
                    unhook(ctx, h);
                } else if (open > XrGripMemory::kReleaseOpening) {
                    if (hold.openSince < 0) hold.openSince = timeSeconds_;
                    else if (timeSeconds_ - hold.openSince > kOpenDwell)
                        unhook(ctx, h);
                } else {
                    hold.openSince = -1;
                }
            }
            continue;
        }

        if (!tracked) continue;

        // Hook, path 1 — pinch near a shape's oriented surface (xr_touch:
        // the pick region is the object, whatever way it lies).
        if (pinch_[h].began() && timeSeconds_ >= consumePinchUntil_) {
            const Pick pick = nearestGrabbable(h, rawP, kGrabSurfaceReach);
            if (pick.valid()) {
                hook(ctx, h, pick, /*pinchHold=*/true,
                     probeShape(ctx, h, pick, kPinchSeedReach));
                consumePinchUntil_ = timeSeconds_ + 0.4;
                continue;
            }
        }

        // Hook, path 2 — CLOSURE: two-plus fingertips ON a shape (proximity
        // probe, deliberately predictive — see xrProbeGrip) with opposed
        // press directions ("held on each side"). No pinch, no cooldown —
        // this is how a fist wraps a pillar, and how a catch takes a flying
        // object the moment the fingers close around it.
        for (size_t i = 0; i < objects_.size() && !hold_[h].active(); i++) {
            const int idx = static_cast<int>(i);
            Pick candidates[2] = {{idx, false}, {idx, true}};
            const int n = objects_[i].subBody != INVALID_PHYSICS_BODY ? 2 : 1;
            for (int c = 0; c < n; c++) {
                if (recentUnhook_[h].obj == candidates[c].obj &&
                    recentUnhook_[h].sub == candidates[c].sub &&
                    timeSeconds_ - recentUnhookAt_[h] < kRehookDelay)
                    continue;   // just let go of this — let it leave
                const std::vector<XrGripPoint> points =
                    probeShape(ctx, h, candidates[c], kClosureReach);
                if (points.size() < 2) continue;
                XrGripMemory probe;
                const PhysicsBodyId body = shapeBody(candidates[c]);
                const Vec3 objP = physics_->physicsWorld().bodyPosition(body);
                probe.capture(points, objP,
                              physics_->physicsWorld().bodyOrientation(body));
                if (!probe.opposed()) continue;
                // Anti-restick: a just-tossed object still passes near the
                // opening fingers for a few frames — never re-hook a shape
                // RECEDING from the grip (an approaching one is a catch, a
                // resting one is a regrip; both still hook).
                const Vec3 away = objP - carry_[h].position();
                if (away.length() > 1e-6) {
                    const Vec3 rel =
                        physics_->physicsWorld().getLinearVelocity(body) -
                        physics_->physicsWorld().getLinearVelocity(
                            gripBody_[h]);
                    if (dot(rel, away * (1.0 / away.length())) > kRecedeSpeed)
                        continue;
                }
                hook(ctx, h, candidates[c], /*pinchHold=*/false, points);
                break;
            }
        }
    }
}

void HandInteractionSystem::update(FrameContext& ctx) {
    if (!physics_ || !ctx.xr.active || !ctx.xr.originBaseValid) return;
    timeSeconds_ += ctx.frameDelta;
    previewSpin_ += static_cast<float>(ctx.frameDelta);
    placeAssist_ = ctx.settings.getBool("xr.placeAssist", true);

    pinch_[0].feed(ctx.xr.hands[0]);
    pinch_[1].feed(ctx.xr.hands[1]);

    updateHandPresence(ctx);
    // Grips BEFORE the palette: a pinch that lands near an existing object
    // means "pick that up", even with the palette open.
    updateGrips(ctx);
    settleHeldLayers(ctx);
    updatePalette(ctx);

    // While hands are interacting, the SYSTEM gaze+pinch must not also fire
    // gaze-driven consumers (this system registers before XrSurfaceSystem).
    if (palette_.anchorHand() >= 0 || hold_[0].active() ||
        hold_[1].active() || timeSeconds_ < consumePinchUntil_) {
        ctx.xr.pinchEnded = false;
        ctx.xr.pinchBegan = false;
    }
}

void HandInteractionSystem::fixedUpdate(FrameContext& ctx) {
    if (!physics_) return;
    // Post-step pose snapshots (PhysicsSystem registers first, so the step
    // already ran): render() interpolates between these with
    // ctx.interpolation, exactly the Transform/PrevTransform contract
    // RenderSystem applies to entities — raw fixed-step poses stair-step
    // 60Hz physics onto a 90Hz display, which reads as jitter.
    (void)ctx;
    for (SandboxObject& obj : objects_) {
        obj.posPrev = obj.posCurr;
        obj.quatPrev = obj.quatCurr;
        obj.posCurr = physics_->physicsWorld().bodyPosition(obj.body);
        obj.quatCurr = physics_->physicsWorld().bodyOrientation(obj.body);
        if (obj.subBody != INVALID_PHYSICS_BODY) {
            obj.subPosPrev = obj.subPosCurr;
            obj.subQuatPrev = obj.subQuatCurr;
            obj.subPosCurr = physics_->physicsWorld().bodyPosition(obj.subBody);
            obj.subQuatCurr =
                physics_->physicsWorld().bodyOrientation(obj.subBody);
        }
    }
}

void HandInteractionSystem::render(FrameContext& ctx) {
    if (!physics_ || !ctx.xr.active || !ctx.xr.originBaseValid) return;

    // Spawned objects — all dynamic, all interpolated between the last two
    // fixed steps (a held object is just a dynamic object with a spring on
    // it, so there is no carry-pose special case anymore).
    const Real alpha = ctx.interpolation;
    RenderMaterial material;
    material.metallic = 0.0f;
    material.roughness = 0.6f;
    for (const SandboxObject& obj : objects_) {
        material.albedo = kItems[obj.item].color;
        const Vec3 p = obj.posPrev + (obj.posCurr - obj.posPrev) * alpha;
        const Quat q = Quat::slerp(obj.quatPrev, obj.quatCurr, alpha);
        const Mat4 world = Mat4::translate(p.x, p.y, p.z) * q.toMat4();
        ctx.renderer.drawMesh(itemMesh(ctx, obj.item), world, material);
        if (obj.subBody != INVALID_PHYSICS_BODY) {
            material.albedo = kItems[obj.item].subColor;
            const Vec3 sp =
                obj.subPosPrev + (obj.subPosCurr - obj.subPosPrev) * alpha;
            const Quat sq = Quat::slerp(obj.subQuatPrev, obj.subQuatCurr,
                                        alpha);
            const Mat4 subWorld =
                Mat4::translate(sp.x, sp.y, sp.z) * sq.toMat4();
            ctx.renderer.drawMesh(subMesh(ctx, obj.item), subWorld, material);
        }
    }

    for (int h = 0; h < 2; h++) {
        // Holding: the remembered grip anchors as contact dots — heating
        // from gold toward red as the opening approaches release, so
        // letting go is legible — the placement-assist ghost, and a
        // straight-down DROP SHADOW under the held shape (the depth cue for
        // where it will land).
        if (hold_[h].active()) {
            const Hold& hold = hold_[h];
            const PhysicsBodyId body = shapeBody(hold.pick);
            const Vec3 objP = physics_->physicsWorld().bodyPosition(body);
            const Quat objQ = physics_->physicsWorld().bodyOrientation(body);
            const Real heat = hold.pinchHold
                ? 0.0
                : std::min(Real(1),
                           hold.lastOpening / XrGripMemory::kReleaseOpening);
            const Vec3 anchorColor =
                Vec3(1.0, 0.85 - 0.65 * heat, 0.3 - 0.25 * heat);
            for (size_t i = 0; i < hold.grip.contactCount(); i++)
                ctx.debug.sphere(hold.grip.worldAnchor(i, objP, objQ), 0.006,
                                 anchorColor, 0, 8);
            if (placeAssist_ && !hold.pick.sub && !shapeIsSphere(hold.pick)) {
                const XrAlignedPose target = placementTarget(hold.pick);
                if (target.valid)
                    ctx.debug.box(target.position, shapeHalfExtent(hold.pick),
                                  target.orientation, Vec3(0.4, 0.8, 1.0));
            }

            Vec3 hit;
            if (physics_->physicsWorld().castRay(objP, Vec3(0, -2.5, 0),
                                                 hit)) {
                if (!shadowMesh_.valid())
                    shadowMesh_ = ctx.renderer.uploadMesh(
                        MeshBuilder::box(Vec3(1.0, 0.004, 1.0)));
                const Vec3 he = shapeHalfExtent(hold.pick);
                const Mat4 m = objQ.toMat4();
                auto ext = [&](int row) {
                    return std::fabs(m.m[row][0]) * he.x +
                           std::fabs(m.m[row][1]) * he.y +
                           std::fabs(m.m[row][2]) * he.z;
                };
                RenderMaterial shadow;
                shadow.albedo = Vec3(0.02, 0.02, 0.02);
                shadow.metallic = 0.0f;
                shadow.roughness = 1.0f;
                shadow.flags = RenderMaterial::FLAG_STIPPLE;
                ctx.renderer.drawMesh(
                    shadowMesh_,
                    Mat4::translate(hit.x, hit.y + 0.003, hit.z) *
                        Mat4::scale(2.0 * ext(0), 1.0, 2.0 * ext(2)),
                    shadow);
            }
            continue;
        }

        // Reaching: the candidate shows an ORIENTED wire box — white far,
        // green in pinch range, BRIGHT green the moment a fist closure
        // would take it — and every fingertip near the surface projects a
        // dot that turns green as that finger arrives on it (thumb + index
        // keep their ruler lines). This is the "how am I grabbing it"
        // readout: two green dots on opposite sides = it's yours.
        if (!pinch_[h].tracking()) continue;
        const Vec3 at = handWorld(ctx, pinch_[h].pinchPoint());
        const Pick pick = nearestGrabbable(h, at, kApproachRange);
        if (!pick.valid()) continue;
        const Real surface = shapeSurfaceDistance(pick, at);
        const Real glow = 1.0 - surface / kApproachRange;
        const bool inReach = surface < kGrabSurfaceReach;

        bool closureReady = false;
        {
            const std::vector<XrGripPoint> points =
                probeShape(ctx, h, pick, kClosureReach);
            if (points.size() >= 2) {
                XrGripMemory probe;
                probe.capture(points,
                              physics_->physicsWorld().bodyPosition(
                                  shapeBody(pick)),
                              physics_->physicsWorld().bodyOrientation(
                                  shapeBody(pick)));
                closureReady = probe.opposed();
            }
        }
        const Vec3 color =
            closureReady
                ? Vec3(0.3, 1.0, 0.5) * 1.4
                : (inReach ? Vec3(0.2, 1.0, 0.4) : Vec3(1.0, 1.0, 1.0)) *
                      (0.35 + 0.65 * glow);
        const PhysicsBodyId body = shapeBody(pick);
        ctx.debug.box(physics_->physicsWorld().bodyPosition(body),
                      shapeHalfExtent(pick) * 1.08,
                      physics_->physicsWorld().bodyOrientation(body), color);

        const XrHand& hand = ctx.xr.hands[h];
        for (const XrHandJointId tip : kTipJoints) {
            if (!hand.joints[tip].tracked) continue;
            const Vec3 tipWorld = handWorld(ctx, xrJointPosition(hand, tip));
            const Vec3 onSurface = shapeNearestPoint(pick, tipWorld);
            const Real d = (onSurface - tipWorld).length();
            if (d > 0.15) continue;
            // White approaching, green ON the surface (closure distance).
            const Real on =
                1.0 - std::min(Real(1), std::max(Real(0),
                                                 (d - kClosureReach) / 0.10));
            const Vec3 tipColor = Vec3(1.0 - 0.7 * on, 1.0, 1.0 - 0.5 * on);
            ctx.debug.sphere(onSurface, 0.006 + 0.003 * on, tipColor, 0, 8);
            if (tip == XR_JOINT_THUMB_TIP || tip == XR_JOINT_INDEX_TIP)
                ctx.debug.line(tipWorld, onSurface, tipColor * 0.5);
        }
    }

    // The palette: item previews spinning above the anchor palm, hover ring
    // under the hovered one.
    const int anchor = palette_.anchorHand();
    if (anchor >= 0) {
        XrPalmPose palm = xrPalmPose(ctx.xr.hands[anchor], anchor == 0);
        if (palm.valid) {
            palm.position = handWorld(ctx, palm.position);
            const Quat spin =
                Quat::fromAxisAngle(Vec3(0, 1, 0), previewSpin_ * 1.2);
            for (int i = 0; i < kItemCount; i++) {
                const Vec3 slot = palette_.slotPosition(palm, i);
                const ItemDef& def = kItems[i];
                // Fit each preview into a ~6cm cell regardless of item size.
                const Real maxDim =
                    2.0 * std::max(def.halfExtent.x,
                                   std::max(def.halfExtent.y,
                                            def.halfExtent.z));
                const Real s =
                    (i == palette_.hoverItem() ? 0.075 : 0.06) / maxDim;
                material.albedo = def.color;
                const Mat4 world = Mat4::translate(slot.x, slot.y, slot.z) *
                                   spin.toMat4() * Mat4::scale(s, s, s);
                ctx.renderer.drawMesh(itemMesh(ctx, i), world, material);
                if (i == palette_.hoverItem()) {
                    const Vec3 a = palm.across * 0.035;
                    const Vec3 b = palm.fingers * 0.035;
                    const Vec3 base = slot - palm.normal * 0.03;
                    const Vec3 ring = Vec3(0.3, 1.0, 0.5);
                    ctx.debug.line(base - a - b, base + a - b, ring);
                    ctx.debug.line(base + a - b, base + a + b, ring);
                    ctx.debug.line(base + a + b, base - a + b, ring);
                    ctx.debug.line(base - a + b, base - a - b, ring);
                }
            }
        }
    }
}

void HandInteractionSystem::onStop(FrameContext& ctx) {
    if (physics_) {
        for (int h = 0; h < 2; h++) {
            if (hold_[h].active())
                physics_->physicsWorld().removeConstraint(hold_[h].spring);
            hold_[h] = Hold{};
            if (gripBody_[h] != INVALID_PHYSICS_BODY) {
                physics_->physicsWorld().removeBody(gripBody_[h]);
                gripBody_[h] = INVALID_PHYSICS_BODY;
            }
            handMemory_[h] = XrHandMemory{};
            for (const HandBone& bone : handBones_[h])
                physics_->physicsWorld().removeBody(bone.body);
            handBones_[h].clear();
            if (palmBody_[h] != INVALID_PHYSICS_BODY) {
                physics_->physicsWorld().removeBody(palmBody_[h]);
                palmBody_[h] = INVALID_PHYSICS_BODY;
            }
        }
        for (const SandboxObject& obj : objects_) {
            if (obj.jointId != INVALID_CONSTRAINT)
                physics_->physicsWorld().removeConstraint(obj.jointId);
            if (obj.subBody != INVALID_PHYSICS_BODY)
                physics_->physicsWorld().removeBody(obj.subBody);
            physics_->physicsWorld().removeBody(obj.body);
        }
    }
    objects_.clear();
    layerClear_.clear();
    handActive_[0] = handActive_[1] = false;
    lastUnhookObj_ = -1;
    recentUnhook_[0] = recentUnhook_[1] = Pick{};
    recentUnhookAt_[0] = recentUnhookAt_[1] = -10;
    palette_ = XrPalette(paletteConfig(kItemCount));
    for (auto& mesh : itemMeshes_) {
        if (mesh.valid()) {
            ctx.renderer.removeMesh(mesh);
            mesh = MeshHandle{};
        }
    }
    for (auto& mesh : subMeshes_) {
        if (mesh.valid()) {
            ctx.renderer.removeMesh(mesh);
            mesh = MeshHandle{};
        }
    }
    if (shadowMesh_.valid()) {
        ctx.renderer.removeMesh(shadowMesh_);
        shadowMesh_ = MeshHandle{};
    }
}

}  // namespace engine
