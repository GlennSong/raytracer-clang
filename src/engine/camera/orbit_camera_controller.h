#ifndef RAYTRACER_ENGINE_ORBIT_CAMERA_CONTROLLER_H
#define RAYTRACER_ENGINE_ORBIT_CAMERA_CONTROLLER_H

#include "camera_controller.h"

// Orbits a target point: look deltas rotate around it, move axes pan the target,
// zoom changes orbit distance. Good for inspecting a fixed subject. (Port of the
// former renderer/OrbitCamera onto the controller seam.)
class OrbitCameraController : public CameraController {
public:
    Vec3 target{0, 1, 0};
    Real distance = 8.0;
    Real yaw = 0.0;     // degrees
    Real pitch = 25.0;  // degrees
    Real fovDegrees = 60.0;
    Real panSpeed = 3.0;
    Real zoomSpeed = 1.0;

    void update(const CameraInput& input, Real dt) override;
    CameraState cameraState(float aspect) const override;

    Vec3 position() const;
};

#endif
