#include "camera.h"

Camera::Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up,
               double fovDegrees, double aspectRatio) {
    double theta = degreesToRadians(fovDegrees);
    double halfHeight = std::tan(theta / 2.0);
    double halfWidth = aspectRatio * halfHeight;

    Vec3 w = normalize(lookFrom - lookAt);
    Vec3 u = normalize(cross(up, w));
    Vec3 v = cross(w, u);

    position = lookFrom;
    lowerLeft = position - halfWidth * u - halfHeight * v - w;
    horizontal = 2.0 * halfWidth * u;
    vertical = 2.0 * halfHeight * v;
}

Ray Camera::generateRay(double u, double v) const {
    Vec3 direction = normalize(lowerLeft + u * horizontal + v * vertical - position);
    return {position, direction};
}
