// Lens effects (virtual-camera plan Phase 4): depth-of-field gather and the
// final image-space warp (distortion + chromatic aberration + vignette).
// Depth linearization comes from post_common.metal (linearizeReverseZ).

// Thin-lens circle-of-confusion radius in pixels. Sensor-plane diameter is
// A * f * |z - zf| / (z * (zf - f)); cocScale converts meters to pixels.
float dofCocRadius(float linZ, constant DOFUniforms& dof) {
    float denom = max(linZ * (dof.focusDistance - dof.focalLength), 1e-4);
    float cocMeters = dof.aperture * dof.focalLength
                    * abs(linZ - dof.focusDistance) / denom;
    return min(cocMeters * dof.cocScale * 0.5, dof.maxCocPixels);
}

// Depth of field: single-pass scatter-as-gather on the HDR scene color, before
// composite. Sky pixels (reverse-Z depth <= 0) sit at far-plane CoC and their
// blur IS visible — the composite passes the scene image through for sky, so
// the skybox defocuses consistently with distant geometry.
kernel void dofGather(
    texture2d<float, access::read> sceneColor [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant DOFUniforms& dof [[buffer(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint2 size = uint2(output.get_width(), output.get_height());
    if (gid.x >= size.x || gid.y >= size.y) return;

    float4 center = sceneColor.read(gid);
    float near = camera.nearPlane;
    float far = camera.farPlane;
    float centerCoc = dofCocRadius(linearizeReverseZ(depthTex.read(gid).x, near, far), dof);

    // In focus — passthrough (sub-pixel blur radius)
    if (centerCoc < 0.5) {
        output.write(center, gid);
        return;
    }

    // Golden-angle spiral gather over the center CoC disk. Each tap is weighted
    // by whether its own CoC reaches this pixel (scatter-as-gather), so sharp
    // in-focus geometry doesn't smear onto blurred neighbors.
    const int TAPS = 24;
    const float GOLDEN_ANGLE = 2.39996323;
    float3 sum = center.rgb;
    float weightSum = 1.0;

    for (int i = 0; i < TAPS; i++) {
        float tapDist = centerCoc * sqrt((float(i) + 0.5) / float(TAPS));
        float angle = float(i) * GOLDEN_ANGLE;
        float2 off = float2(cos(angle), sin(angle)) * tapDist;
        int2 coord = clamp(int2(gid) + int2(round(off.x), round(off.y)),
                           int2(0), int2(size) - 1);

        float sampleCoc = dofCocRadius(
            linearizeReverseZ(depthTex.read(uint2(coord)).x, near, far), dof);
        // Full weight when the sample's blur circle covers this pixel, soft
        // 1px transition below that.
        float w = saturate(sampleCoc - tapDist + 1.0);
        sum += sceneColor.read(uint2(coord)).rgb * w;
        weightSum += w;
    }

    output.write(float4(sum / weightSum, center.a), gid);
}

// Final lens-warp pass: Brown radial distortion + lateral chromatic aberration
// + vignette in one resample of the composited LDR image. With all parameters
// zero this is an exact passthrough — the encoder is skipped instead (the
// renderer only runs the pass when LensParams::hasAberrations()). Out-of-range
// samples from barrel warp rely on the clamp-to-edge sampler.
fragment float4 fragmentLensWarp(
    CompositeOut in [[stage_in]],
    texture2d<float> src [[texture(0)]],
    constant LensPostUniforms& lens [[buffer(0)]],
    sampler smp [[sampler(0)]]
) {
    // Center and aspect-correct so the warp radius is circular on screen
    float2 centered = in.uv - 0.5;
    float2 p = centered * float2(lens.aspect, 1.0);
    float r2 = dot(p, p);

    // Brown radial model: sample at uv * (1 + k1 r^2 + k2 r^4). k1 > 0 pulls
    // samples outward (barrel), k1 < 0 pincushion; k = 0 is exact passthrough.
    float distort = 1.0 + lens.k1 * r2 + lens.k2 * r2 * r2;

    // Lateral CA: R/B sampled at slightly different radial scales. Offset is
    // linear in the centered coordinate (~ca * 0.5 * screenHeight px at the
    // frame edge), so ca = 0.01 is a clearly visible few-pixel fringe at 1080p.
    float2 uvR = 0.5 + centered * distort * (1.0 + lens.chromaticAberration);
    float2 uvG = 0.5 + centered * distort;
    float2 uvB = 0.5 + centered * distort * (1.0 - lens.chromaticAberration);

    float3 color = float3(src.sample(smp, uvR).r,
                          src.sample(smp, uvG).g,
                          src.sample(smp, uvB).b);

    // Vignette: smooth falloff in r^2 reaching the full `vignette` fraction of
    // darkening near the corners (r2 ≈ 1 for a 16:9 frame) — 0.5 reads clearly.
    color *= 1.0 - lens.vignette * smoothstep(0.0, 1.0, r2);

    return float4(color, 1.0);
}
