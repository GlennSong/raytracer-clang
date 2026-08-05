// --- Screen-Space Ambient Occlusion (GTAO) ---
// Full-resolution compute with fewer samples per pixel.

// AO runs at half the G-buffer resolution. Map an AO-buffer integer coord to the
// matching full-res depth/normal texel center, so the AO kernels read the
// G-buffer at the right place regardless of the AO/G-buffer size ratio.
static inline uint2 aoCoordToGBuffer(uint2 aoCoord, float2 aoSize, uint2 gbSize) {
    return min(uint2((float2(aoCoord) + 0.5) / aoSize * float2(gbSize)),
               gbSize - uint2(1));
}

kernel void gtaoCompute(
    texture2d<float, access::read> depthTex [[texture(0)]],
    texture2d<float, access::write> aoResult [[texture(1)]],
    texture2d<float, access::read> normalTex [[texture(2)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant SSAOUniforms& aoParams [[buffer(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint2 texSize = uint2(aoResult.get_width(), aoResult.get_height());
    if (gid.x >= texSize.x || gid.y >= texSize.y) return;

    // AO is half-res; sample depth/normal at the matching full-res G-buffer texel.
    uint2 gbSize = uint2(depthTex.get_width(), depthTex.get_height());
    uint2 centerCoord = aoCoordToGBuffer(gid, float2(texSize), gbSize);
    float depth = depthTex.read(centerCoord).x;

    // Sky pixels — no occlusion (reverse-Z background = cleared far value 0)
    if (depth <= 0.0) {
        aoResult.write(float4(1.0), gid);
        return;
    }

    float2 uv = (float2(gid) + 0.5) / float2(texSize);

    // Reconstruct view-space position
    float3 viewPos = ssrViewPos(depth, uv, camera.invProjection);

    // Use the real view-space normal from the G-buffer (same encoding as SSR).
    // Reconstructing the normal from depth neighbours sprays garbage normals over
    // thin/edgy geometry (foliage), which made AO crawl and block under motion.
    float3 viewNormal = normalize(normalTex.read(centerCoord).xyz * 2.0 - 1.0);

    const int NUM_DIRECTIONS = aoParams.directions;
    const int NUM_STEPS = aoParams.steps;
    float radius = aoParams.radius;
    float intensity = aoParams.intensity;
    float bias = aoParams.bias;

    // Per-pixel rotation: interleaved gradient noise (Jimenez, SIGGRAPH 2014)
    // Produces well-distributed values in [0, 2π] with no visible tiling.
    // Plus a per-frame rotation (aoParams.frameRotation) so the temporal resolve
    // averages directions across frames — few directions then look banding-free.
    float rotAngle = fract(52.9829189 * fract(0.06711056 * float(gid.x) + 0.00583715 * float(gid.y))) * 2.0 * M_PI_F;
    rotAngle += aoParams.frameRotation;

    // Screen-space pixel radius (adapts to depth). Capped at 32px: the world
    // radius projects to hundreds of px up close, so a large cap + few steps put
    // samples ~16px apart -> coarse, blocky AO that shifts as the camera moves.
    // A tighter cap keeps the (NUM_STEPS) samples dense enough to be smooth.
    float projScale = camera.projection[1][1] * float(texSize.y) * 0.5;
    float screenRadius = clamp(radius * projScale / (-viewPos.z), 3.0, 32.0);

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

            uint2 sampleCoord = min(uint2(sampleUV * float2(gbSize)), gbSize - uint2(1));
            float sampleDepth = depthTex.read(sampleCoord).x;
            if (sampleDepth <= 0.0) continue;   // reverse-Z background

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
    constant CameraUniforms& camera [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = output.get_width();
    uint h = output.get_height();
    if (gid.x >= w || gid.y >= h) return;

    // AO is half-res; read depth at the matching full-res G-buffer texel.
    uint2 gbSize = uint2(depthTex.get_width(), depthTex.get_height());
    float2 aoSize = float2(w, h);
    float centerAO = input.read(gid).r;
    float centerDepth = depthTex.read(aoCoordToGBuffer(gid, aoSize, gbSize)).x;

    float total = centerAO;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(clamp(int(gid.x) + i, 0, int(w) - 1), gid.y);
        float sampleAO = input.read(coord).r;
        float sampleDepth = depthTex.read(aoCoordToGBuffer(coord, aoSize, gbSize)).x;

        float spatial = exp(-float(i * i) / 8.0);
        float depthW = bilateralDepthWeight(centerDepth, sampleDepth,
                                            camera.nearPlane, camera.farPlane, 0.03);
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
    constant CameraUniforms& camera [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = output.get_width();
    uint h = output.get_height();
    if (gid.x >= w || gid.y >= h) return;

    // AO is half-res; read depth at the matching full-res G-buffer texel.
    uint2 gbSize = uint2(depthTex.get_width(), depthTex.get_height());
    float2 aoSize = float2(w, h);
    float centerAO = input.read(gid).r;
    float centerDepth = depthTex.read(aoCoordToGBuffer(gid, aoSize, gbSize)).x;

    float total = centerAO;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(gid.x, clamp(int(gid.y) + i, 0, int(h) - 1));
        float sampleAO = input.read(coord).r;
        float sampleDepth = depthTex.read(aoCoordToGBuffer(coord, aoSize, gbSize)).x;

        float spatial = exp(-float(i * i) / 8.0);
        float depthW = bilateralDepthWeight(centerDepth, sampleDepth,
                                            camera.nearPlane, camera.farPlane, 0.03);
        float wt = spatial * depthW;

        total += sampleAO * wt;
        weightSum += wt;
    }

    output.write(float4(total / weightSum, 0, 0, 1), gid);
}

// Temporal AO resolve: blends the (blurred) current-frame AO with last frame's
// AO reprojected to this pixel. Foliage is sub-pixel, so with no AA each pixel
// flips between leaf and background as the camera moves, swinging AO 0.5<->1.0
// frame to frame; both values are individually valid, so only accumulating over
// time stabilizes it. Ghosting/disocclusion are bounded by clamping the history
// sample to the 3x3 neighborhood of the current AO (TAA-style neighborhood clamp)
// and by rejecting history that reprojects off-screen.
kernel void aoTemporal(
    texture2d<float, access::read>   currentAO  [[texture(0)]],
    texture2d<float, access::read>   depthTex   [[texture(1)]],
    texture2d<float, access::sample> historyAO  [[texture(2)]],
    texture2d<float, access::write>  resolvedAO [[texture(3)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant AOTemporalUniforms& t  [[buffer(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint2 size = uint2(resolvedAO.get_width(), resolvedAO.get_height());
    if (gid.x >= size.x || gid.y >= size.y) return;

    // AO is half-res; read depth at the matching full-res G-buffer texel.
    uint2 gbSize = uint2(depthTex.get_width(), depthTex.get_height());
    float cur = currentAO.read(gid).r;
    float depth = depthTex.read(aoCoordToGBuffer(gid, float2(size), gbSize)).x;

    // No history to blend (first frame / resize), or sky: pass current through.
    if (t.alpha <= 0.0 || depth <= 0.0) {   // reverse-Z background
        resolvedAO.write(float4(cur), gid);
        return;
    }

    // Reconstruct this pixel's world position, then project it into last frame.
    float2 uv = (float2(gid) + 0.5) / float2(size);
    float2 ndc = float2(uv.x * 2.0 - 1.0, -(uv.y * 2.0 - 1.0));
    float4 world = camera.invViewProjection * float4(ndc, depth, 1.0);
    world /= world.w;
    float4 prevClip = t.prevViewProjection * world;
    float resolved = cur;
    if (prevClip.w > 0.0) {
        float2 prevNdc = prevClip.xy / prevClip.w;
        float2 prevUV = float2(prevNdc.x * 0.5 + 0.5, 0.5 - prevNdc.y * 0.5);
        if (all(prevUV >= float2(0.0)) && all(prevUV <= float2(1.0))) {
            // Neighborhood min/max of the current AO bounds plausible values;
            // clamping the reprojected history to it suppresses ghosting and
            // disocclusion bleed without needing a history-depth buffer.
            float mn = cur, mx = cur;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    uint2 c = uint2(clamp(int2(gid) + int2(dx, dy),
                                          int2(0), int2(size) - 1));
                    float s = currentAO.read(c).r;
                    mn = min(mn, s);
                    mx = max(mx, s);
                }
            constexpr sampler hsmp(filter::linear, address::clamp_to_edge);
            float hist = clamp(historyAO.sample(hsmp, prevUV).r, mn, mx);
            resolved = mix(cur, hist, t.alpha);
        }
    }
    resolvedAO.write(float4(resolved), gid);
}
