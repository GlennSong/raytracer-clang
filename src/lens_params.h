#ifndef RAYTRACER_LENS_PARAMS_H
#define RAYTRACER_LENS_PARAMS_H

#include "rt_math.h"

namespace engine {

// Physical lens parameterization (docs/virtual-camera-plan.md). Framing is
// expressed the way a real camera is set up — focal length + sensor size — and
// FOV is derived, never stored. One parameter set feeds both render paths: the
// offline tracer samples a thin lens with it (physically honest), the realtime
// viewer approximates it with post-process passes. Standard-library + rt_math
// only, so the offline tracer and the renderer seam can both include it.
struct LensParams {
    Real focalLength = 50.0;    // mm
    Real sensorHeight = 24.0;   // mm (full-frame 36x24 default)
    Real fStop = 8.0;           // aperture f-number
    Real focusDistance = 10.0;  // world units
    Real nearPlane = 0.1;
    Real farPlane = 8000.0;   // cover distant LOD terrain; fog fades the far field

    // Image-forming aberrations; 0 = ideal lens.
    Real distortionK1 = 0.0;          // Brown radial terms
    Real distortionK2 = 0.0;
    Real chromaticAberration = 0.0;   // lateral CA strength
    Real vignette = 0.0;

    Real verticalFovDegrees() const {
        if (focalLength <= 0.0) return 60.0;  // degenerate lens; nominal default
        return radiansToDegrees(2.0 * std::atan(sensorHeight / (2.0 * focalLength)));
    }

    Real apertureDiameter() const {   // world units; 0 = pinhole (fStop <= 0)
        if (fStop <= 0.0) return 0.0;
        return (focalLength / fStop) / 1000.0;  // mm -> meters
    }

    bool hasAberrations() const {
        return distortionK1 != 0.0 || distortionK2 != 0.0 ||
               chromaticAberration != 0.0 || vignette != 0.0;
    }
};

}  // namespace engine

#endif
