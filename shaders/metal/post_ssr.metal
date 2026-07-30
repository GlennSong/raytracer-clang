// Screen-space reflections: ray march + separable bilateral blur.
// Depends on post_common.metal (ssrViewPos, linearizeReverseZ, bilateralDepthWeight).

kernel void ssrRayMarch(
    texture2d<float, access::read> sceneColor [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> ssrResult [[texture(2)]],
    texture2d<float, access::read> normalTex [[texture(3)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant SSRUniforms& params [[buffer(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    // SSR runs at half resolution; depth/scene/normals are full resolution
    uint2 outSize = uint2(ssrResult.get_width(), ssrResult.get_height());
    if (gid.x >= outSize.x || gid.y >= outSize.y) return;

    float2 uv = (float2(gid) + 0.5) / float2(outSize);
    float2 fullSize = camera.screenSize;
    uint2 fullMax = uint2(fullSize) - 1;

    // Diagnostic mode (debug view 2): instead of the reflected color, write a
    // flat color code (alpha 1) so the debug view shows exactly where SSR exits.
    //   black     = sky / background (no surface)
    //   red       = roughness gate: surface too rough to reflect
    //   yellow    = ray points behind the camera (degenerate)
    //   cyan      = ray too short on screen (< 1px) to march
    //   blue      = marched the full ray, found no hit on screen
    //   green     = SSR hit; brightness == the confidence (alpha) the composite
    //               blends by. Bright green reaches the screen; near-black green
    //               means the confidence term is crushing an otherwise-valid hit.
    bool dbg = params.debug != 0.0;

    // Read depth at full resolution
    uint2 depthCoord = min(uint2(uv * fullSize), fullMax);
    float depth = depthTex.read(depthCoord).x;

    if (depth <= 0.0) {   // reverse-Z background (cleared far value)
        ssrResult.write(float4(0.0), gid);
        return;
    }

    // Reconstruct view-space position from depth, read normal from G-buffer
    float3 viewPos = ssrViewPos(depth, uv, camera.invProjection);
    float4 normalSample = normalTex.read(depthCoord);
    float3 viewNormal = normalize(normalSample.xyz * 2.0 - 1.0);

    // Roughness gate (packed in normal.w): rough surfaces scatter reflections to
    // nothing — only smooth/metallic/wet surfaces get SSR. Fades to 0 at
    // maxRoughness, killing the bogus forest-on-ground/foliage mirror look and
    // skipping the ray march entirely for the (common) rough pixels.
    float reflectivity = 1.0 - smoothstep(0.0, params.maxRoughness, normalSample.w);
    if (reflectivity <= 0.0) {
        ssrResult.write(dbg ? float4(1.0, 0.0, 0.0, 1.0) : float4(0.0), gid);
        return;
    }

    float3 viewDir = normalize(viewPos);
    float3 reflectDir = reflect(viewDir, viewNormal);

    // Project start and a point along the ray into screen space
    float4 startProj = camera.projection * float4(viewPos, 1.0);
    float3 startNDC = startProj.xyz / startProj.w;

    float maxRayDist = params.maxRayDist;
    float3 rayEnd = viewPos + reflectDir * maxRayDist;
    float4 endProj = camera.projection * float4(rayEnd, 1.0);
    if (endProj.w <= 0.0) {
        // Ray goes behind camera — clip to near plane
        float t = (-viewPos.z - 0.1) / reflectDir.z;
        if (t <= 0.0) {
            ssrResult.write(dbg ? float4(1.0, 1.0, 0.0, 1.0) : float4(0.0), gid);
            return;
        }
        rayEnd = viewPos + reflectDir * t * 0.99;
        endProj = camera.projection * float4(rayEnd, 1.0);
        maxRayDist = t * 0.99;
    }
    float3 endNDC = endProj.xyz / endProj.w;

    // Convert NDC to UV
    float2 startScreenUV = float2(startNDC.x * 0.5 + 0.5, -startNDC.y * 0.5 + 0.5);
    float2 endScreenUV   = float2(endNDC.x * 0.5 + 0.5,   -endNDC.y * 0.5 + 0.5);

    float2 deltaUV = endScreenUV - startScreenUV;
    float2 deltaPixels = deltaUV * fullSize;
    float pixelDist = max(abs(deltaPixels.x), abs(deltaPixels.y));
    if (pixelDist < 1.0) {
        ssrResult.write(dbg ? float4(0.0, 1.0, 1.0, 1.0) : float4(0.0), gid);
        return;
    }

    const int MAX_STEPS = 48;
    const int BINARY_STEPS = 6;
    float pixelStride = params.stride;
    int stepCount = min(MAX_STEPS, int(pixelDist / pixelStride));
    if (stepCount < 1) stepCount = 1;

    // Perspective-correct interpolation: NDC z
    float startInvW = 1.0 / startProj.w;
    float endInvW   = 1.0 / endProj.w;
    float startZoW  = startNDC.z * startInvW;
    float endZoW    = endNDC.z * endInvW;

    // Precompute near/far for NDC→linear conversion
    float near = camera.nearPlane;
    float far  = camera.farPlane;

    // Per-pixel jitter
    float hash = fract(sin(dot(float2(gid), float2(127.1, 311.7))) * 43758.5453);

    bool hit = false;
    float2 hitUV = float2(0.0);
    float hitDist = 0.0;

    for (int i = 0; i < stepCount; i++) {
        float t = (float(i) + 1.0 + hash) / float(stepCount);
        float2 sampleUV = startScreenUV + deltaUV * t;
        if (any(sampleUV < float2(0.0)) || any(sampleUV > float2(1.0))) break;

        uint2 sampleCoord = min(uint2(sampleUV * fullSize), fullMax);
        float sceneDepth = depthTex.read(sampleCoord).x;
        if (sceneDepth <= 0.0) continue;   // reverse-Z background

        // Perspective-correct ray depth in NDC (reverse-Z)
        float invW = mix(startInvW, endInvW, t);
        float rayDepth = mix(startZoW, endZoW, t) / invW;

        // Compare in linear eye space: monotonic and sign-stable under reverse-Z.
        // depthDiff > 0 means the ray has passed behind the scene surface.
        float rayLinZ = linearizeReverseZ(rayDepth, near, far);
        float sceneLinZ = linearizeReverseZ(sceneDepth, near, far);
        float depthDiff = rayLinZ - sceneLinZ;

        // Distance-aware surface thickness, directly in world/eye units.
        float thicknessWorld = mix(params.thickness, params.thicknessFar, saturate(rayLinZ / 30.0));

        if (depthDiff > 0.0 && depthDiff < thicknessWorld) {
            // Binary search refinement — linear-eye comparison, no matrix math
            float tLo = (float(i) + hash) / float(stepCount);
            float tHi = t;
            float2 bestUV = sampleUV;
            for (int b = 0; b < BINARY_STEPS; b++) {
                float tMid = (tLo + tHi) * 0.5;
                float2 midUV = startScreenUV + deltaUV * tMid;
                midUV = clamp(midUV, float2(0.0), float2(1.0));
                uint2 midCoord = min(uint2(midUV * fullSize), fullMax);

                float midInvW = mix(startInvW, endInvW, tMid);
                float midRayZ = mix(startZoW, endZoW, tMid) / midInvW;
                float midRayLinZ = linearizeReverseZ(midRayZ, near, far);
                float midSceneLinZ = linearizeReverseZ(depthTex.read(midCoord).x, near, far);

                if (midRayLinZ > midSceneLinZ) {   // behind surface -> pull closer
                    tHi = tMid; bestUV = midUV;
                } else {
                    tLo = tMid;
                }
            }
            hit = true;
            hitUV = bestUV;
            float finalT = (tLo + tHi) * 0.5;
            float finalInvW = mix(startInvW, endInvW, finalT);
            float finalRayZ = mix(startZoW, endZoW, finalT) / finalInvW;
            float finalLinZ = linearizeReverseZ(finalRayZ, near, far);
            float startLinZ = linearizeReverseZ(startNDC.z, near, far);
            hitDist = abs(finalLinZ - startLinZ);
            break;
        }
    }

    if (!hit) {
        ssrResult.write(dbg ? float4(0.0, 0.0, 1.0, 1.0) : float4(0.0), gid);
        return;
    }

    uint2 hitCoord = min(uint2(hitUV * fullSize), fullMax);
    float3 hitColor = sceneColor.read(hitCoord).rgb;

    float2 edgeFade = smoothstep(float2(0.0), float2(0.05), hitUV) *
                       smoothstep(float2(0.0), float2(0.05), 1.0 - hitUV);
    float confidence = edgeFade.x * edgeFade.y;
    confidence *= 1.0 - saturate(hitDist / maxRayDist);

    // Grazing-angle emphasis, but with a high floor instead of a dielectric F0.
    // A pure 0.04 + ... Schlick term crushes head-on reflections to ~4%, and SSR
    // has no per-pixel metalness to recover the metal/smooth surfaces it gates in
    // — so head-on hits (cubes, wedge, the metal torus) went invisible. Keep the
    // grazing boost; let the roughness gate (reflectivity) + blendStrength shape
    // the overall intensity.
    float NdotV = saturate(dot(viewNormal, -viewDir));
    float fresnel = mix(0.7, 1.0, pow(1.0 - NdotV, 5.0));
    confidence *= fresnel * reflectivity;

    // Diagnostic: green brightness == the real confidence (alpha) that the
    // composite blends by. A bright green hit means SSR is reaching the screen;
    // a near-black green means confidence (fresnel/edge/distance) is crushing it
    // even though the ray hit. Pre-blur, pre-blend — see ssrBlurH/composite.
    if (dbg) { ssrResult.write(float4(0.0, confidence, 0.0, 1.0), gid); return; }

    ssrResult.write(float4(hitColor, confidence), gid);
}

// Bilateral blur for SSR (horizontal) — half-res input, full-res depth
kernel void ssrBlurH(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    constant CameraUniforms& camera [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = input.get_width();
    uint h = input.get_height();
    if (gid.x >= w || gid.y >= h) return;

    // Map half-res gid to full-res depth coordinate
    uint2 fullSize = uint2(depthTex.get_width(), depthTex.get_height());
    uint2 depthCenter = min(uint2(float2(gid) / float2(w, h) * float2(fullSize)), fullSize - 1);
    float centerDepth = depthTex.read(depthCenter).x;
    float4 centerColor = input.read(gid);

    if (centerColor.a < 0.001) {
        output.write(float4(0.0), gid);
        return;
    }

    float4 total = centerColor;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(clamp(int(gid.x) + i, 0, int(w) - 1), gid.y);
        float4 s = input.read(coord);

        uint2 depthCoord = min(uint2(float2(coord) / float2(w, h) * float2(fullSize)), fullSize - 1);
        float sd = depthTex.read(depthCoord).x;

        float spatial = exp(-float(i * i) / 10.0);
        float depthW = bilateralDepthWeight(centerDepth, sd,
                                            camera.nearPlane, camera.farPlane, 0.05);
        float wt = spatial * depthW;

        total += s * wt;
        weightSum += wt;
    }

    output.write(total / weightSum, gid);
}

// Bilateral blur for SSR (vertical) — half-res input, full-res depth
kernel void ssrBlurV(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    constant CameraUniforms& camera [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = input.get_width();
    uint h = input.get_height();
    if (gid.x >= w || gid.y >= h) return;

    uint2 fullSize = uint2(depthTex.get_width(), depthTex.get_height());
    uint2 depthCenter = min(uint2(float2(gid) / float2(w, h) * float2(fullSize)), fullSize - 1);
    float centerDepth = depthTex.read(depthCenter).x;
    float4 centerColor = input.read(gid);

    if (centerColor.a < 0.001) {
        output.write(float4(0.0), gid);
        return;
    }

    float4 total = centerColor;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(gid.x, clamp(int(gid.y) + i, 0, int(h) - 1));
        float4 s = input.read(coord);

        uint2 depthCoord = min(uint2(float2(coord) / float2(w, h) * float2(fullSize)), fullSize - 1);
        float sd = depthTex.read(depthCoord).x;

        float spatial = exp(-float(i * i) / 10.0);
        float depthW = bilateralDepthWeight(centerDepth, sd,
                                            camera.nearPlane, camera.farPlane, 0.05);
        float wt = spatial * depthW;

        total += s * wt;
        weightSum += wt;
    }

    output.write(total / weightSum, gid);
}
