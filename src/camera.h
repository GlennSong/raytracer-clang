#ifndef RAYTRACER_CAMERA_H
#define RAYTRACER_CAMERA_H

#include "rt_math.h"

namespace engine {

class Camera {
public:
    Vec3 position;
    Vec3 lowerLeft;
    Vec3 horizontal;
    Vec3 vertical;

    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up, double fovDegrees, double aspectRatio);

    Ray generateRay(double u, double v) const;
};


}  // namespace engine

#endif
