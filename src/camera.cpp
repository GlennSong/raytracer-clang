#include "camera.h"

namespace engine {

Camera::Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up,
               double fovDegrees, double aspectRatio)
    : Camera(lookFrom, lookAt, up, fovDegrees, aspectRatio, LensParams{}, 0.0) {
    // Delegating with worldUnitsPerMeter 0 forces lensRadius to 0: an exact
    // pinhole, so existing renders are bit-identical.
}

Camera::Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up,
               double fovDegrees, double aspectRatio,
               const LensParams& lensParams, double worldUnitsPerMeter)
    : lens(lensParams) {
    double theta = degreesToRadians(fovDegrees);
    double halfHeight = std::tan(theta / 2.0);
    double halfWidth = aspectRatio * halfHeight;

    basisW = normalize(lookFrom - lookAt);
    basisU = normalize(cross(up, basisW));
    basisV = cross(basisW, basisU);

    // The image rectangle sits on the focal plane, so every in-focus target is
    // shared by all aperture samples. A pinhole focuses "at 1" — the scale
    // cancels out of the normalized direction.
    double focusDist = (lens.focusDistance > 0.0) ? lens.focusDistance : 1.0;
    lensRadius = lens.apertureDiameter() * worldUnitsPerMeter / 2.0;
    if (lensRadius <= 0.0) {
        lensRadius = 0.0;
        focusDist = 1.0;
    }

    position = lookFrom;
    lowerLeft = position -
                focusDist * (halfWidth * basisU + halfHeight * basisV + basisW);
    horizontal = 2.0 * focusDist * halfWidth * basisU;
    vertical = 2.0 * focusDist * halfHeight * basisV;
}

Ray Camera::generateRay(double u, double v) const {
    return generateRay(u, v, 0.0, 0.0);
}

Ray Camera::generateRay(double u, double v, double lensU, double lensV,
                        double radialScale) const {
    // Brown radial distortion (+ the caller's per-channel scale) on centered
    // image coordinates; k1 = k2 = 0 and radialScale = 1 is exact passthrough.
    double s = 2.0 * u - 1.0;
    double t = 2.0 * v - 1.0;
    double r2 = s * s + t * t;
    double warp = radialScale *
                  (1.0 + lens.distortionK1 * r2 + lens.distortionK2 * r2 * r2);
    u = (s * warp + 1.0) / 2.0;
    v = (t * warp + 1.0) / 2.0;

    Vec3 target = lowerLeft + u * horizontal + v * vertical;

    Vec3 origin = position;
    if (lensRadius > 0.0) {
        double r = lensRadius * std::sqrt(lensU);
        double phi = 2.0 * PI * lensV;
        origin += r * std::cos(phi) * basisU + r * std::sin(phi) * basisV;
    }
    return {origin, normalize(target - origin)};
}

}  // namespace engine
