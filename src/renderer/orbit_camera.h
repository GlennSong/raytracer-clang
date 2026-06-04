#ifndef RAYTRACER_ORBIT_CAMERA_H
#define RAYTRACER_ORBIT_CAMERA_H

#include "renderer.h"
#include "window.h"

class OrbitCamera {
public:
    Vec3 target;
    float distance;
    float yaw;
    float pitch;
    float fovDegrees;
    float moveSpeed;
    float rotateSpeed;
    float zoomSpeed;
    bool orthographic;

    OrbitCamera();

    void update(const InputState& input, double deltaTime);
    CameraState getCameraState(float aspectRatio) const;
    Vec3 getPosition() const;
};

#endif
