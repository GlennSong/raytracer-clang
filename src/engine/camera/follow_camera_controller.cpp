#include "follow_camera_controller.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// Critically-damped spring step (the classic SmoothDamp): moves `current`
// toward `target` with persistent `velocity`, never overshooting. smoothTime
// is the time constant — ~63% of the error closes per smoothTime.
Real smoothDamp(Real current, Real target, Real& velocity, Real smoothTime,
                Real dt) {
    if (smoothTime <= 0.0 || dt <= 0.0) { velocity = 0.0; return target; }
    const Real omega = 2.0 / smoothTime;
    const Real x = omega * dt;
    const Real decay = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
    const Real change = current - target;
    const Real temp = (velocity + omega * change) * dt;
    velocity = (velocity - omega * temp) * decay;
    return target + (change + temp) * decay;
}

// Degrees wrapped to [-180, 180] — yaw smoothing must take the short arc, or
// a 350° -> 10° heading change swings the camera the long way round.
Real wrapDegrees(Real a) {
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

}  // namespace

Vec3 FollowCameraController::forward() const {
    Real yawRad = degreesToRadians(smoothedYaw + orbitYaw);
    Real pitchRad = degreesToRadians(orbitPitch);
    Real cp = std::cos(pitchRad);
    return Vec3(std::sin(yawRad) * cp,
                std::sin(pitchRad),
                -std::cos(yawRad) * cp);
}

void FollowCameraController::applyShoulderPreset() {
    distance = SHOULDER_ARM;
    minDistance = SHOULDER_ARM_MIN;
    maxDistance = SHOULDER_ARM_MAX;
    lateralOffset = SHOULDER_SIDE;
    targetHeight = SHOULDER_HEIGHT;
    orbitPitch = SHOULDER_PITCH;
}

void FollowCameraController::update(const CameraInput& input, Real dt) {
    // Orbit the camera around the target; pitch is clamped so it can't flip over
    // or bury itself in the ground.
    orbitYaw += input.lookYawDelta;
    orbitPitch = std::clamp(orbitPitch + input.lookPitchDelta, PITCH_MIN, PITCH_MAX);
    // Scroll dollies the chase distance (negative zoomDelta pushes out).
    distance = std::clamp(distance - input.zoomDelta * 0.8, minDistance, maxDistance);

    // The target's velocity, low-passed (the feed is quantized by the physics step).
    if (dt > 1e-6) {
        if (velocityPrimed) {
            const Vec3 raw = (targetPos - velocityPrev) / dt;
            const Real k = velocitySmoothTime > 0 ? 1.0 - std::exp(-dt / velocitySmoothTime) : 1.0;
            targetVelocity = targetVelocity + (raw - targetVelocity) * k;
        }
        velocityPrev = targetPos;
        velocityPrimed = true;
    }
    // The safety net: a rig this far behind (a target moved without setTarget) cuts to it.
    const Vec3 gap = targetPos - smoothedPos;
    if (gap.x * gap.x + gap.y * gap.y + gap.z * gap.z > maxTrail * maxTrail) {
        smoothedPos = targetPos;
        smoothedYaw = targetYaw;
        posVelocity = Vec3(0, 0, 0);
        yawVelocity = 0.0;
        return;
    }
    // Spring toward the target led by velocity x smoothTime (feed-forward, header): no steady trail. The
    // discrete spring's own lag is half a frame short of smoothTime, so lead by that much less (measured:
    // without it the rig ran v x dt / 2 AHEAD of the car at every frame rate).
    const Vec3 goal = targetPos + targetVelocity * std::max(Real(0), posSmoothTime - 0.5 * dt);
    smoothedPos.x = smoothDamp(smoothedPos.x, goal.x, posVelocity.x, posSmoothTime, dt);
    smoothedPos.y = smoothDamp(smoothedPos.y, goal.y, posVelocity.y, posSmoothTime, dt);
    smoothedPos.z = smoothDamp(smoothedPos.z, goal.z, posVelocity.z, posSmoothTime, dt);
    // Yaw takes the short arc: smooth the wrapped delta around the current
    // heading, then re-anchor.
    const Real yawGoal = smoothedYaw + wrapDegrees(targetYaw - smoothedYaw);
    smoothedYaw = wrapDegrees(
        smoothDamp(smoothedYaw, yawGoal, yawVelocity, yawSmoothTime, dt));
}

CameraState FollowCameraController::cameraState(float aspect) const {
    // Aim above the SMOOTHED pose (the spring's output), shifted sideways in
    // the follow frame (the full yaw: heading + orbit) — the over-the-shoulder
    // offset turns with it.
    Real yawRad = degreesToRadians(smoothedYaw + orbitYaw);
    Vec3 right(std::cos(yawRad), 0, std::sin(yawRad));
    Vec3 aim = smoothedPos + Vec3(0, targetHeight, 0) + right * lateralOffset;
    Vec3 dir = forward();

    CameraState state;
    state.position = aim - dir * distance;   // sit `distance` back along the look dir
    state.target = aim;
    state.up = Vec3(0, 1, 0);
    state.projection = orthoEnabled ? CameraProjection::Orthographic
                                    : CameraProjection::Perspective;
    state.fovDegrees = static_cast<float>(fovDegrees);
    Real fovRad = degreesToRadians(fovDegrees);
    state.orthoHeight = static_cast<float>(2.0 * distance * std::tan(fovRad * 0.5));
    state.aspectRatio = aspect;
    state.nearPlane = static_cast<float>(nearPlane);
    state.farPlane = static_cast<float>(farPlane);
    return state;
}

}  // namespace engine
