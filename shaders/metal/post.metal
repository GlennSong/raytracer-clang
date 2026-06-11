// Post-processing: screen-space reflections, GTAO, their bilateral blurs,
// bloom, and the composite (tone map + gamma) pass.

// --- Screen-Space Reflections (SSR) ---

// Reconstruct view-space position from NDC depth and screen UV
float3 ssrViewPos(float depth, float2 uv, float4x4 invProjection) {
    float4 clip = float4(uv.x * 2.0 - 1.0, -(uv.y * 2.0 - 1.0), depth, 1.0);
    float4 vp = invProjection * clip;
    return vp.xyz / vp.w;
}

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

    // Read depth at full resolution
    uint2 depthCoord = min(uint2(uv * fullSize), fullMax);
    float depth = depthTex.read(depthCoord).x;

    if (depth >= 0.999) {
        ssrResult.write(float4(0.0), gid);
        return;
    }

    // Reconstruct view-space position from depth, read normal from G-buffer
    float3 viewPos = ssrViewPos(depth, uv, camera.invProjection);
    float3 viewNormal = normalTex.read(depthCoord).xyz * 2.0 - 1.0;
    viewNormal = normalize(viewNormal);

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
        if (t <= 0.0) { ssrResult.write(float4(0.0), gid); return; }
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
    if (pixelDist < 1.0) { ssrResult.write(float4(0.0), gid); return; }

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
        if (sceneDepth >= 0.999) continue;

        // Perspective-correct ray depth in NDC
        float invW = mix(startInvW, endInvW, t);
        float rayDepth = mix(startZoW, endZoW, t) / invW;

        float depthDiff = rayDepth - sceneDepth;

        // Convert ray's linear depth for distance-aware thickness
        // linearZ = near * far / (far - ndcZ * (far - near))
        float rayLinZ = near * far / (far - rayDepth * (far - near));
        float thicknessWorld = mix(params.thickness, params.thicknessFar, saturate(rayLinZ / 30.0));
        // Convert world thickness to NDC at this depth:
        // dNDC/dZ ≈ near * far / (linearZ^2 * (far - near) / (far))
        //         = near * far^2 / (linearZ^2 * (far - near))  ... but simpler:
        float offsetZ = rayLinZ + thicknessWorld;
        float ndcAtOffset = far * (offsetZ - near) / (offsetZ * (far - near));
        float thicknessNDC = ndcAtOffset - rayDepth;

        if (depthDiff > 0.0 && depthDiff < thicknessNDC) {
            // Binary search refinement — pure NDC comparison, no matrix math
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

                if (midRayZ > depthTex.read(midCoord).x) {
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
            float finalLinZ = near * far / (far - finalRayZ * (far - near));
            float startLinZ = near * far / (far - startNDC.z * (far - near));
            hitDist = finalLinZ - startLinZ;
            break;
        }
    }

    if (!hit) {
        ssrResult.write(float4(0.0), gid);
        return;
    }

    uint2 hitCoord = min(uint2(hitUV * fullSize), fullMax);
    float3 hitColor = sceneColor.read(hitCoord).rgb;

    float2 edgeFade = smoothstep(float2(0.0), float2(0.05), hitUV) *
                       smoothstep(float2(0.0), float2(0.05), 1.0 - hitUV);
    float confidence = edgeFade.x * edgeFade.y;
    confidence *= 1.0 - saturate(hitDist / maxRayDist);

    float NdotV = saturate(dot(viewNormal, -viewDir));
    float fresnel = 0.04 + 0.96 * pow(1.0 - NdotV, 5.0);
    confidence *= fresnel;

    ssrResult.write(float4(hitColor, confidence), gid);
}

// Bilateral blur for SSR (horizontal) — half-res input, full-res depth
kernel void ssrBlurH(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
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
        float depthW = exp(-abs(sd - centerDepth) / 0.003);
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
        float depthW = exp(-abs(sd - centerDepth) / 0.003);
        float wt = spatial * depthW;

        total += s * wt;
        weightSum += wt;
    }

    output.write(total / weightSum, gid);
}

// --- Screen-Space Ambient Occlusion (GTAO) ---
// Full-resolution compute with fewer samples per pixel.

kernel void gtaoCompute(
    texture2d<float, access::read> depthTex [[texture(0)]],
    texture2d<float, access::write> aoResult [[texture(1)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant SSAOUniforms& aoParams [[buffer(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint2 texSize = uint2(aoResult.get_width(), aoResult.get_height());
    if (gid.x >= texSize.x || gid.y >= texSize.y) return;

    float depth = depthTex.read(gid).x;

    // Sky pixels — no occlusion
    if (depth >= 0.999) {
        aoResult.write(float4(1.0), gid);
        return;
    }

    float2 uv = (float2(gid) + 0.5) / float2(texSize);

    // Reconstruct view-space position
    float3 viewPos = ssrViewPos(depth, uv, camera.invProjection);

    // Reconstruct normal from depth neighbors (central differences, pick best pair)
    uint2 maxCoord = texSize - 1;
    float depthL = depthTex.read(uint2(max(int(gid.x) - 1, 0), gid.y)).x;
    float depthR = depthTex.read(uint2(min(gid.x + 1, maxCoord.x), gid.y)).x;
    float depthU = depthTex.read(uint2(gid.x, max(int(gid.y) - 1, 0))).x;
    float depthD = depthTex.read(uint2(gid.x, min(gid.y + 1, maxCoord.y))).x;

    float2 texel = 1.0 / float2(texSize);
    float3 viewPosL = ssrViewPos(depthL, uv - float2(texel.x, 0), camera.invProjection);
    float3 viewPosR = ssrViewPos(depthR, uv + float2(texel.x, 0), camera.invProjection);
    float3 viewPosU = ssrViewPos(depthU, uv - float2(0, texel.y), camera.invProjection);
    float3 viewPosD = ssrViewPos(depthD, uv + float2(0, texel.y), camera.invProjection);

    float3 ddx = (abs(depthR - depth) < abs(depthL - depth))
                 ? (viewPosR - viewPos) : (viewPos - viewPosL);
    float3 ddy = (abs(depthD - depth) < abs(depthU - depth))
                 ? (viewPosD - viewPos) : (viewPos - viewPosU);
    float3 viewNormal = normalize(cross(ddy, ddx));
    if (viewNormal.z < 0.0) viewNormal = -viewNormal;

    const int NUM_DIRECTIONS = aoParams.directions;
    const int NUM_STEPS = aoParams.steps;
    float radius = aoParams.radius;
    float intensity = aoParams.intensity;
    float bias = aoParams.bias;

    // Per-pixel rotation: interleaved gradient noise (Jimenez, SIGGRAPH 2014)
    // Produces well-distributed values in [0, 2π] with no visible tiling
    float rotAngle = fract(52.9829189 * fract(0.06711056 * float(gid.x) + 0.00583715 * float(gid.y))) * 2.0 * M_PI_F;

    // Screen-space pixel radius (adapts to depth)
    float projScale = camera.projection[1][1] * float(texSize.y) * 0.5;
    float screenRadius = clamp(radius * projScale / (-viewPos.z), 3.0, 64.0);

    float occlusion = 0.0;

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        float angle = rotAngle + float(d) * (2.0 * M_PI_F / float(NUM_DIRECTIONS));
        float2 dir = float2(cos(angle), sin(angle));

        float horizonCos = bias;

        for (int s = 1; s <= NUM_STEPS; s++) {
            float t = float(s) / float(NUM_STEPS);
            float2 sampleOffset = dir * t * screenRadius / float2(texSize);
            float2 sampleUV = uv + sampleOffset;

            if (any(sampleUV < float2(0.0)) || any(sampleUV > float2(1.0))) continue;

            uint2 sampleCoord = min(uint2(sampleUV * float2(texSize)), maxCoord);
            float sampleDepth = depthTex.read(sampleCoord).x;
            if (sampleDepth >= 0.999) continue;

            float3 samplePos = ssrViewPos(sampleDepth, sampleUV, camera.invProjection);
            float3 horizonVec = samplePos - viewPos;
            float horizonDist = length(horizonVec);
            if (horizonDist < 0.01) continue;

            float h = dot(normalize(horizonVec), viewNormal);
            // Smooth falloff: gentle fade from 80% of radius to the edge
            float distRatio = horizonDist / radius;
            float distFade = 1.0 - smoothstep(0.3, 1.0, distRatio);
            h *= distFade;

            horizonCos = max(horizonCos, h);
        }

        occlusion += horizonCos;
    }

    occlusion /= float(NUM_DIRECTIONS);
    float ao = 1.0 - saturate(occlusion * intensity);

    aoResult.write(float4(ao, ao, ao, 1.0), gid);
}

// AO bilateral blur (horizontal) — same resolution, depth-aware
kernel void aoBlurH(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = output.get_width();
    uint h = output.get_height();
    if (gid.x >= w || gid.y >= h) return;

    float centerAO = input.read(gid).r;
    float centerDepth = depthTex.read(gid).x;

    float total = centerAO;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(clamp(int(gid.x) + i, 0, int(w) - 1), gid.y);
        float sampleAO = input.read(coord).r;
        float sampleDepth = depthTex.read(coord).x;

        float spatial = exp(-float(i * i) / 8.0);
        float depthDiff = abs(sampleDepth - centerDepth);
        float depthW = exp(-depthDiff * depthDiff * 100000.0);
        float wt = spatial * depthW;

        total += sampleAO * wt;
        weightSum += wt;
    }

    output.write(float4(total / weightSum, 0, 0, 1), gid);
}

// AO bilateral blur (vertical) — same resolution, depth-aware
kernel void aoBlurV(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = output.get_width();
    uint h = output.get_height();
    if (gid.x >= w || gid.y >= h) return;

    float centerAO = input.read(gid).r;
    float centerDepth = depthTex.read(gid).x;

    float total = centerAO;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(gid.x, clamp(int(gid.y) + i, 0, int(h) - 1));
        float sampleAO = input.read(coord).r;
        float sampleDepth = depthTex.read(coord).x;

        float spatial = exp(-float(i * i) / 8.0);
        float depthDiff = abs(sampleDepth - centerDepth);
        float depthW = exp(-depthDiff * depthDiff * 100000.0);
        float wt = spatial * depthW;

        total += sampleAO * wt;
        weightSum += wt;
    }

    output.write(float4(total / weightSum, 0, 0, 1), gid);
}

// --- Bloom: threshold extract + progressive downsample/upsample ---

// Extract bright pixels and downsample to first mip (half-res).
// Uses a 13-tap filter (Jimenez 2014 / Call of Duty) for stable downsampling.
kernel void bloomDownsample(
    texture2d<float, access::read>  src  [[texture(0)]],
    texture2d<float, access::write> dst  [[texture(1)]],
    constant BloomUniforms& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;

    // Map output texel to source center (each output pixel = 2x2 source block)
    float2 srcCoord = float2(gid) * 2.0 + 0.5;

    // 13-tap downsample: 4 corner samples + 4 edge samples + 1 center + 4 diagonal
    // Weights: center cross = 0.5, corners = 0.125 each (energy preserving)
    float3 a = src.read(uint2(clamp(srcCoord + float2(-1, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 b = src.read(uint2(clamp(srcCoord + float2( 0, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 c = src.read(uint2(clamp(srcCoord + float2( 1, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 d = src.read(uint2(clamp(srcCoord + float2(-1,  0), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 e = src.read(uint2(clamp(srcCoord,                  float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 f = src.read(uint2(clamp(srcCoord + float2( 1,  0), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 g = src.read(uint2(clamp(srcCoord + float2(-1,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 h = src.read(uint2(clamp(srcCoord + float2( 0,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 i = src.read(uint2(clamp(srcCoord + float2( 1,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;

    // Weighted average (box filter with center emphasis)
    float3 color = e * 0.25
                 + (b + d + f + h) * 0.125
                 + (a + c + g + i) * 0.0625;

    // Soft threshold on first pass (srcWidth == full resolution means this is pass 0)
    if (params.srcWidth > int(dst.get_width()) * 3) {
        float brightness = max(color.r, max(color.g, color.b));
        float soft = brightness - params.threshold + params.knee;
        soft = clamp(soft, 0.0, 2.0 * params.knee);
        soft = soft * soft / (4.0 * params.knee + 1e-5);
        float contribution = max(soft, brightness - params.threshold) / max(brightness, 1e-5);
        color *= max(contribution, 0.0);
    }

    dst.write(float4(color, 1.0), gid);
}

// Upsample and additively blend with the next-higher mip.
// Uses a 3x3 tent filter for smooth upsampling.
kernel void bloomUpsample(
    texture2d<float, access::read>  src     [[texture(0)]],  // smaller mip (being upsampled)
    texture2d<float, access::read>  higher  [[texture(1)]],  // larger mip to blend into
    texture2d<float, access::write> dst     [[texture(2)]],  // output (same size as higher)
    constant BloomUniforms& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    int dstW = int(dst.get_width());
    int dstH = int(dst.get_height());
    if (int(gid.x) >= dstW || int(gid.y) >= dstH) return;

    // Bilinear sample from smaller mip using 3x3 tent filter
    float2 srcCoord = (float2(gid) + 0.5) * float2(src.get_width(), src.get_height())
                    / float2(dstW, dstH) - 0.5;

    int srcW = int(src.get_width());
    int srcH = int(src.get_height());

    float3 sum = float3(0);
    // 3x3 tent: weights 1,2,1 / 2,4,2 / 1,2,1 (sum=16)
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int2 coord = int2(srcCoord) + int2(dx, dy);
            coord = clamp(coord, int2(0), int2(srcW - 1, srcH - 1));
            float w = float((2 - abs(dx)) * (2 - abs(dy)));
            sum += src.read(uint2(coord)).rgb * w;
        }
    }
    sum /= 16.0;

    // Add to higher-resolution mip
    float3 higherColor = higher.read(gid).rgb;
    dst.write(float4(higherColor + sum, 1.0), gid);
}

// --- Composite pass: tone map + gamma from HDR scene texture to LDR drawable ---

struct CompositeOut {
    float4 position [[position]];
    float2 uv;
};

vertex CompositeOut vertexComposite(uint vid [[vertex_id]]) {
    float2 uv = float2((vid << 1) & 2, vid & 2);
    CompositeOut out;
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    out.uv = float2(uv.x, 1.0 - uv.y);  // flip Y for Metal texture origin
    return out;
}

fragment float4 fragmentComposite(
    CompositeOut in [[stage_in]],
    texture2d<float> sceneColor [[texture(0)]],
    texture2d<float> ssrTexture [[texture(1)]],
    texture2d<float> aoTexture [[texture(2)]],
    depth2d<float> depthTex [[texture(3)]],
    texture2d<float> normalTexture [[texture(4)]],
    texture2d<float> bloomTexture [[texture(5)]],
    texturecube<float> envCube [[texture(6)]],
    texture2d<float> equirectMap [[texture(7)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant CompositeUniforms& params [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    sampler smp [[sampler(0)]],
    sampler envSampler [[sampler(1)]]
) {
    // Read depth at this fragment's pixel position (no sampler needed)
    float depth = depthTex.read(uint2(in.position.xy));

    // --- Debug visualization modes ---
    if (params.debugView == 1) {
        // AO only — white = no occlusion, black = full occlusion
        float ao = aoTexture.sample(smp, in.uv).r;
        if (depth >= 0.999) ao = 1.0;
        return float4(ao, ao, ao, 1.0);
    }
    if (params.debugView == 2) {
        // SSR only — show reflected color where SSR hit, black elsewhere
        float4 ssr = ssrTexture.sample(smp, in.uv);
        return float4(ssr.rgb * ssr.a, 1.0);
    }
    if (params.debugView == 3) {
        // Depth — linearized, white=near black=far
        float lin = camera.nearPlane / (camera.farPlane - depth * (camera.farPlane - camera.nearPlane));
        lin = saturate(lin * camera.farPlane * 0.1);
        return float4(lin, lin, lin, 1.0);
    }
    if (params.debugView == 4) {
        // View-space normals visualization
        float3 n = normalTexture.read(uint2(in.position.xy)).xyz;
        if (depth >= 0.999) return float4(0.5, 0.5, 1.0, 1.0);
        return float4(n, 1.0);
    }
    if (params.debugView == 5) {
        // Shadow factor (the lit pass wrote grayscale into sceneColor) — raw, no
        // tone map. White = lit, black = shadowed. Sky shows mid-gray.
        if (depth >= 0.999) return float4(0.3, 0.3, 0.3, 1.0);
        return float4(sceneColor.sample(smp, in.uv).rgb, 1.0);
    }
    if (params.debugView == 6) {
        // Albedo (the lit pass wrote raw material color into sceneColor) — no
        // exposure or tone map, gamma only, so material color shows directly.
        if (depth >= 0.999) return float4(0.0, 0.0, 0.0, 1.0);
        float3 a = sceneColor.sample(smp, in.uv).rgb;
        return float4(pow(a, float3(1.0 / 2.2)), 1.0);
    }

    // --- Normal rendering ---
    float3 hdrColor;
    if (depth >= 0.999) {
        // Sky pixel — re-derive the active environment directly in composite.
        float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
        float4 nearWorld = camera.invViewProjection * float4(ndc, 0.0, 1.0);
        float4 farWorld  = camera.invViewProjection * float4(ndc, 1.0, 1.0);
        float3 rayDir = normalize(farWorld.xyz / farWorld.w - nearWorld.xyz / nearWorld.w);
        if (params.envMode == 1) {
            // HDR environment: sample the equirect directly by the (verified)
            // composite rayDir. The equirect→cube bake used the broken vertexSkybox
            // direction path, so the cube was mis-oriented; a direct lat-long lookup
            // is cheap for a fullscreen pass and bypasses the bake entirely.
            hdrColor = sampleEquirect(equirectMap, envSampler, rayDir);
        } else {
            hdrColor = sampleEnvironment(rayDir, lightData);
            hdrColor = applyClouds(hdrColor, rayDir, lightData);
        }
    } else {
        float4 hdr = sceneColor.sample(smp, in.uv);

        // Apply ambient occlusion if enabled
        if (params.ssaoEnabled != 0) {
            float ao = aoTexture.sample(smp, in.uv).r;
            hdr.rgb *= max(ao, params.aoFloor);
        }

        // Blend SSR reflections if enabled
        if (params.ssrEnabled != 0) {
            float4 ssr = ssrTexture.sample(smp, in.uv);
            hdr.rgb = mix(hdr.rgb, ssr.rgb, ssr.a * params.ssrBlendStrength);
        }

        hdrColor = hdr.rgb;
    }

    // Camera exposure — the single scene-referred multiplier, applied uniformly
    // to sky and objects before tone mapping (nothing upstream pre-exposes).
    hdrColor *= lightData.exposure;

    // Add bloom
    if (params.bloomEnabled != 0) {
        float3 bloom = bloomTexture.sample(smp, in.uv).rgb;
        hdrColor += bloom * params.bloomIntensity;
    }

    // ACES filmic tone mapping
    float3 x = hdrColor;
    float3 color = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    color = saturate(color);

    // Gamma correction
    color = pow(color, float3(1.0 / 2.2));

    return float4(color, 1.0);
}
