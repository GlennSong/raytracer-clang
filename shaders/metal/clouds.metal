// Volumetric clouds — Metal port of the Vulkan clouds.frag (procedural-planet-plan
// P4), WIRED by the cinematic-sky phase. A shared density + lighting + march core
// over two domains selected by uni.layer.w: 0 = SLAB (the cloudscape over a ground
// scene — the procgen city/terrain sky, depth-occluded) and 1 = SHELL (a planet's
// cloud deck from space). Density samples the tileable 3D Perlin-Worley texture
// set baked CPU-side (src/renderer/metal/cloud_noise.h, Schneider/Nubis-style:
// 128^3 base shape + 32^3 detail erosion — this replaced the inline value-noise
// fbm whose thresholded blobs read as small potatoes, not cumulus); lighting is
// a sun light-march (Beer + Powder) with a Henyey-Greenstein phase. Output is
// scene-linear HDR; the composite pass tone maps.
//
// Wiring (metal_renderer.mm): fragmentClouds renders the layer at HALF resolution
// into an offscreen RGBA16F target as a premultiplied overlay — rgb = in-scattered
// radiance, a = transmittance — reading only the scene DEPTH (reverse-Z) for
// occlusion. fragmentCloudComposite then upsamples it bilaterally (depth-guided,
// via cloudOverlaySample) over the full-res HDR scene with One + SourceAlpha
// blending; the composite pass reuses cloudOverlaySample for sky pixels, which it
// re-derives analytically. CloudUniforms lives in shader_types.h (shared C++/MSL).

// The baked Perlin-Worley set tiles, so repeat addressing is the whole wrap
// story; linear filtering does the octave smoothing the ALU fbm used to pay
// for. Bound by the march pass at texture(1)/(2).
constexpr sampler cl_noiseSamp(coord::normalized, filter::linear, address::repeat);

static float cl_remap(float x, float a, float b, float c, float d) {
    return c + (x - a) / max(b - a, 1e-5) * (d - c);
}

static float cl_layerHeight(float3 p, constant CloudUniforms& u) {
    float h = (u.layer.w < 0.5) ? p.y : (length(p - u.planetCenter.xyz) - u.layer.z);
    return clamp((h - u.layer.x) / max(1e-4, u.layer.y - u.layer.x), 0.0, 1.0);
}

static float cl_density(float3 p, constant CloudUniforms& u,
                        texture3d<float> baseNoise, texture3d<float> detailNoise) {
    float hf = cl_layerHeight(p, u);
    if (hf <= 0.0 || hf >= 1.0) return 0.0;
    float profile = smoothstep(0.0, 0.15, hf) * smoothstep(1.0, 0.55, hf);
    float3 wind = float3(u.params.w * u.skyAmbient.w, 0.0, 0.0);
    float3 q = (p + wind) * u.params.z;
    // Base shape: R = perlin-worley, GBA = worley fBm at rising frequency.
    // The 0.25 uv scale puts the texture's freq-4 perlin cell at ~1/noiseScale
    // metres — the same feature scale the old value fbm gave params.z, so
    // levels keep their meaning. The worley fBm remap carves the billows.
    float4 nse = baseNoise.sample(cl_noiseSamp, q * 0.25);
    float lowFbm = nse.g * 0.625 + nse.b * 0.25 + nse.a * 0.125;
    float base = clamp(cl_remap(nse.r, lowFbm - 1.0, 1.0, 0.0, 1.0), 0.0, 1.0);
    // Coverage knob: unchanged semantics — 1-coverage is the threshold, the
    // remainder renormalised — so level JSON keeps meaning what it meant.
    float d = clamp((base - (1.0 - u.params.x)) / max(1e-3, u.params.x), 0.0, 1.0);
    // Edge erosion: subtract high-frequency worley scaled by (1 - d), so
    // interiors stay solid and edges wisp away (the standard erode trick).
    float3 det = detailNoise.sample(cl_noiseSamp, q * 2.0).rgb;
    float detailFbm = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
    d = clamp(d - detailFbm * u.detail.x * (1.0 - d), 0.0, 1.0);
    return d * profile * u.params.y;
}

static float cl_phaseHG(float mu, float g) {
    float g2 = g * g;
    return (1.0 - g2) / (4.0 * M_PI_F * pow(1.0 + g2 - 2.0 * g * mu, 1.5));
}

static bool cl_layerInterval(float3 origin, float3 dir, constant CloudUniforms& u,
                             thread float& t0, thread float& t1) {
    if (u.layer.w < 0.5) {
        if (fabs(dir.y) < 1e-5) {
            if (origin.y < u.layer.x || origin.y > u.layer.y) return false;
            t0 = 0.0; t1 = u.march.w; return true;
        }
        float ta = (u.layer.x - origin.y) / dir.y;
        float tb = (u.layer.y - origin.y) / dir.y;
        t0 = max(min(ta, tb), 0.0);
        t1 = max(ta, tb);
        return t1 > 0.0;
    }
    float3 oc = origin - u.planetCenter.xyz;
    float b = dot(oc, dir);
    float cOut = dot(oc, oc) - u.layer.y * u.layer.y;
    float discOut = b * b - cOut;
    if (discOut < 0.0) return false;
    float sOut = sqrt(discOut);
    float outerFar = -b + sOut;
    if (outerFar < 0.0) return false;
    t0 = max(-b - sOut, 0.0);
    t1 = outerFar;
    float cIn = dot(oc, oc) - u.layer.x * u.layer.x;
    float discIn = b * b - cIn;
    if (discIn > 0.0) {
        float innerNear = -b - sqrt(discIn);
        if (innerNear > 0.0) t1 = min(t1, innerNear);
    }
    return t1 > t0;
}

struct CloudsOut {
    float4 position [[position]];
    float2 uv;
};

vertex CloudsOut vertexClouds(uint vid [[vertex_id]]) {
    CloudsOut out;
    float2 uv = float2((vid << 1) & 2, vid & 2);
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    // Flip Y to Metal's texture origin (top-left), exactly like vertexAtmosphere
    // — the fragment's ndc reconstruction does `-(uv.y*2-1)` and the depth
    // sample both assume this y-down uv. Without the flip the ray Y is inverted
    // and the cloud deck mirrors vertically as the camera pitches.
    out.uv = float2(uv.x, 1.0 - uv.y);
    return out;
}

// Half-resolution cloud march. Premultiplied overlay out: rgb = in-scattered
// radiance, a = view transmittance (1 = no cloud). Composite: scene*a + rgb.
fragment float4 fragmentClouds(CloudsOut in [[stage_in]],
                               depth2d<float> sceneDepth [[texture(0)]],
                               texture3d<float> baseNoise [[texture(1)]],
                               texture3d<float> detailNoise [[texture(2)]],
                               constant CloudUniforms& u [[buffer(0)]]) {
    constexpr sampler samp(coord::normalized, filter::linear, address::clamp_to_edge);
    const float4 CLEAR = float4(0.0, 0.0, 0.0, 1.0);

    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = u.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 camPos = u.cameraPosition.xyz;
    float3 dir = normalize(world.xyz / world.w - camPos);
    float3 sunDir = normalize(u.sunDirection.xyz);

    float t0, t1;
    if (!cl_layerInterval(camPos, dir, u, t0, t1)) return CLEAR;

    // Scene depth occlusion (reverse-Z: background depth == 0 -> far).
    float depth = sceneDepth.sample(samp, in.uv);
    float tScene = u.march.w;
    if (depth > 0.0) {
        float4 w = u.invViewProjection * float4(ndc, depth, 1.0);
        tScene = length(w.xyz / w.w - camPos);
    }
    t1 = min(t1, tScene);
    if (t1 <= t0) return CLEAR;

    int viewSteps = int(u.march.x);
    int lightSteps = int(u.march.y);
    float ds = (t1 - t0) / float(max(2, viewSteps));
    float mu = dot(dir, sunDir);
    float phase = cl_phaseHG(mu, u.march.z);

    // Per-pixel start jitter hides the banding a fixed step grid produces at
    // these step counts; the bilateral upsample averages it away.
    float jitter = fract(sin(dot(in.position.xy, float2(12.9898, 78.233))) * 43758.5453);

    float transmittance = 1.0;
    float3 scattered = float3(0.0);
    float3 sunLight = u.sunColor.rgb * u.sunColor.w;

    for (int i = 0; i < viewSteps; i++) {
        float3 p = camPos + dir * (t0 + ds * (float(i) + jitter));
        float density = cl_density(p, u, baseNoise, detailNoise);
        if (density > 0.001) {
            float lightOD = 0.0;
            float lds = (u.layer.y - u.layer.x) / float(max(1, lightSteps));
            for (int j = 0; j < lightSteps; j++) {
                float3 lp = p + sunDir * (lds * (float(j) + 0.5));
                lightOD += cl_density(lp, u, baseNoise, detailNoise) * lds;
            }
            float beer = exp(-lightOD);
            float powder = 1.0 - exp(-2.0 * density * ds);
            // Ambient grades with height in the layer: bases sit in their own
            // shadow, tops face the open sky — gives the deck its underside.
            float hf = cl_layerHeight(p, u);
            float3 ambient = u.skyAmbient.rgb * (0.5 + 0.5 * hf);
            float3 lum = (sunLight * beer * phase * powder + ambient) * density;
            float stepT = exp(-density * ds);
            scattered += transmittance * lum * ds;
            transmittance *= stepT;
            if (transmittance < 0.01) break;
        }
    }

    // AERIAL FADE (device: "a lot of white glare from the clouds at the
    // horizon"): edge-on rays enter the slab kilometres out and stack into a
    // solid white wall. Real distant clouds dissolve into the haze — fade this
    // pixel's cloud contribution by its march ENTRY distance, on the same
    // farDistance scale the march already carries (march.w): gone by ~far/3.
    // Near-overhead cloud (small t0) is untouched.
    float horizonFade = exp(-3.0 * t0 / max(u.march.w, 1.0));
    scattered *= horizonFade;
    transmittance = mix(1.0, transmittance, horizonFade);

    return float4(scattered, transmittance);
}

// Depth-guided bilateral upsample of the half-res cloud overlay at a full-res
// pixel. Weighted 4-tap: bilinear weights modulated by how well each half-res
// texel's depth matches this pixel (sky texels stay off building silhouettes
// and vice versa). Shared by the scene composite pass below and by the sky
// branch of fragmentComposite (declared there, defined here — one concatenated
// translation unit).
float4 cloudOverlaySample(float2 uv, texture2d<float> cloudTex,
                          depth2d<float> depthTex, constant CameraUniforms& camera) {
    float2 cSize = float2(cloudTex.get_width(), cloudTex.get_height());
    uint2 fullMax = uint2(depthTex.get_width() - 1, depthTex.get_height() - 1);
    float2 fullSize = float2(depthTex.get_width(), depthTex.get_height());

    float d0 = depthTex.read(min(uint2(uv * fullSize), fullMax));

    float2 pos = uv * cSize - 0.5;
    float2 f = fract(pos);
    int2 base = int2(floor(pos));
    float4 acc = float4(0.0);
    float wSum = 0.0;
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 2; i++) {
            int2 c = clamp(base + int2(i, j), int2(0), int2(cSize) - 1);
            float2 cuv = (float2(c) + 0.5) / cSize;
            float dd = depthTex.read(min(uint2(cuv * fullSize), fullMax));
            float bw = (i != 0 ? f.x : 1.0 - f.x) * (j != 0 ? f.y : 1.0 - f.y);
            float w = bw * bilateralDepthWeight(d0, dd, camera.nearPlane,
                                                camera.farPlane, 0.10) + 1e-5;
            acc += cloudTex.read(uint2(c)) * w;
            wSum += w;
        }
    }
    return acc / wSum;
}

// Full-res composite of the half-res cloud overlay onto the HDR scene target.
// Blend state: RGB = One + SourceAlpha (dest * transmittance + scattered),
// alpha untouched. Sky pixels get the same overlay again in fragmentComposite
// (which re-derives the sky analytically); geometry pixels come from here.
fragment float4 fragmentCloudComposite(CloudsOut in [[stage_in]],
                                       texture2d<float> cloudTex [[texture(0)]],
                                       depth2d<float> depthTex [[texture(1)]],
                                       constant CameraUniforms& camera [[buffer(1)]]) {
    return cloudOverlaySample(in.uv, cloudTex, depthTex, camera);
}
