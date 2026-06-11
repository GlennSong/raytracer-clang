#ifndef RAYTRACER_CAMERA_H
#define RAYTRACER_CAMERA_H

#include "rt_math.h"
#include "lens_params.h"

namespace engine {

// Offline path-tracer camera. A pinhole by default; given LensParams it
// becomes a thin lens (docs/virtual-camera-plan.md, Phase 3): rays start on
// the aperture disk and converge on the focal plane, so depth of field falls
// out of the existing multi-sampling. Radial (Brown) distortion warps the
// image-plane mapping; lateral chromatic aberration is realized by the caller
// tracing per-channel rays with slightly different radial scales.
class Camera {
public:
    Vec3 position;
    Vec3 lowerLeft;     // of the focal-plane rectangle, world units
    Vec3 horizontal;
    Vec3 vertical;

    LensParams lens;

    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up, double fovDegrees, double aspectRatio);

    // worldUnitsPerMeter scales the physical aperture into scene units (the
    // Cornell box is ~555 units across, so pass ~100 for meter-scale optics).
    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up, double fovDegrees, double aspectRatio,
           const LensParams& lensParams, double worldUnitsPerMeter = 1.0);

    // Pinhole-compatible: the ray through (u, v) from the aperture center.
    Ray generateRay(double u, double v) const;

    // Thin-lens: lensU/lensV in [0, 1) choose the aperture sample (0,0 is the
    // center). radialScale scales the distorted image coordinates — pass
    // 1 +/- chromaticAberration for the R/B channels, 1.0 for G.
    Ray generateRay(double u, double v, double lensU, double lensV,
                    double radialScale = 1.0) const;

    bool hasChromaticAberration() const { return lens.chromaticAberration != 0.0; }

private:
    Vec3 basisU, basisV, basisW;   // right, up, backward
    double lensRadius = 0.0;       // world units; 0 = pinhole
};

}  // namespace engine

#endif
