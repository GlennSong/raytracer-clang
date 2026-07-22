// Volumetric clouds — Metal port of the Vulkan clouds.frag (procedural-planet-plan
// P4). A shared density + lighting + march core over two domains selected by
// uni.layer.w: 0 = SLAB (the cloudscape over a ground scene — the procgen
// city/terrain sky, depth-occluded) and 1 = SHELL (a planet's cloud deck from
// space). One shader for the city AND the planet; supersedes the flat 2D FBM sky
// clouds. Density is a compact inline hash-fbm (a 3D Perlin-Worley texture set is
// the on-device upgrade); lighting is a sun light-march (Beer + Powder) with a
// Henyey-Greenstein phase. Output is scene-linear HDR; the composite pass tone maps.
//
// Concatenation-ready like the other shaders/metal/*.metal (relies on the prepended
// metal_stdlib / shader_types.h). To wire on a Mac: add "clouds.metal" to the concat
// list in metal_renderer.mm, move CloudUniforms to shader_types.h, build a pipeline
// from vertexClouds / fragmentClouds, and insert the pass after the scene (with the
// scene color + depth bound). Mirrors the SPIR-V-verified GLSL 1:1; MSL syntax here
// is compile-unverified (no macOS toolchain in CI).

// Move to shader_types.h (shared C++/MSL) when wiring the pipeline.
struct CloudUniforms {
    float4x4 invViewProjection;
    float4   cameraPosition;
    float4   sunDirection;
    float4   sunColor;        // rgb, w intensity
    float4   skyAmbient;      // rgb ambient, w time
    float4   planetCenter;
    float4   layer;           // x bottom, y top, z planetRadius, w domainMode
    float4   params;          // x coverage, y densityScale, z noiseScale, w windSpeed
    float4   march;           // x viewSteps, y lightSteps, z phaseG, w farDistance
};

struct CloudsOut {
    float4 position [[position]];
    float2 uv;
};

static float cl_hash13(float3 p) {
    p = fract(p * 0.3183099 + float3(0.1, 0.2, 0.3));
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

static float cl_valueNoise(float3 p) {
    float3 i = floor(p);
    float3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = cl_hash13(i + float3(0, 0, 0));
    float n100 = cl_hash13(i + float3(1, 0, 0));
    float n010 = cl_hash13(i + float3(0, 1, 0));
    float n110 = cl_hash13(i + float3(1, 1, 0));
    float n001 = cl_hash13(i + float3(0, 0, 1));
    float n101 = cl_hash13(i + float3(1, 0, 1));
    float n011 = cl_hash13(i + float3(0, 1, 1));
    float n111 = cl_hash13(i + float3(1, 1, 1));
    return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
               mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}

static float cl_fbm(float3 p) {
    float sum = 0.0, amp = 0.5;
    for (int i = 0; i < 5; i++) { sum += cl_valueNoise(p) * amp; p *= 2.02; amp *= 0.5; }
    return sum;
}

static float cl_layerHeight(float3 p, constant CloudUniforms& u) {
    float h = (u.layer.w < 0.5) ? p.y : (length(p - u.planetCenter.xyz) - u.layer.z);
    return clamp((h - u.layer.x) / max(1e-4, u.layer.y - u.layer.x), 0.0, 1.0);
}

static float cl_density(float3 p, constant CloudUniforms& u) {
    float hf = cl_layerHeight(p, u);
    if (hf <= 0.0 || hf >= 1.0) return 0.0;
    float profile = smoothstep(0.0, 0.15, hf) * smoothstep(1.0, 0.55, hf);
    float3 wind = float3(u.params.w * u.skyAmbient.w, 0.0, 0.0);
    float3 q = (p + wind) * u.params.z;
    float base = cl_fbm(q);
    float d = clamp((base - (1.0 - u.params.x)) / max(1e-3, u.params.x), 0.0, 1.0);
    float detail = cl_fbm(q * 3.1 + 4.0);
    d = clamp(d - detail * 0.35 * (1.0 - d), 0.0, 1.0);
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

vertex CloudsOut vertexClouds(uint vid [[vertex_id]]) {
    CloudsOut out;
    float2 uv = float2((vid << 1) & 2, vid & 2);
    out.uv = uv;
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    return out;
}

fragment float4 fragmentClouds(CloudsOut in [[stage_in]],
                               texture2d<float> sceneColor [[texture(0)]],
                               depth2d<float> sceneDepth [[texture(1)]],
                               constant CloudUniforms& u [[buffer(0)]]) {
    constexpr sampler samp(coord::normalized, filter::linear, address::clamp_to_edge);

    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = u.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 camPos = u.cameraPosition.xyz;
    float3 dir = normalize(world.xyz / world.w - camPos);
    float3 sunDir = normalize(u.sunDirection.xyz);
    float3 scene = sceneColor.sample(samp, in.uv).rgb;

    float t0, t1;
    if (!cl_layerInterval(camPos, dir, u, t0, t1)) return float4(scene, 1.0);

    // Scene depth occlusion (reverse-Z: background depth == 0 -> far).
    float depth = sceneDepth.sample(samp, in.uv);
    float tScene = u.march.w;
    if (depth > 0.0) {
        float4 w = u.invViewProjection * float4(ndc, depth, 1.0);
        tScene = length(w.xyz / w.w - camPos);
    }
    t1 = min(t1, tScene);
    if (t1 <= t0) return float4(scene, 1.0);

    int viewSteps = int(u.march.x);
    int lightSteps = int(u.march.y);
    float ds = (t1 - t0) / float(max(2, viewSteps));
    float mu = dot(dir, sunDir);
    float phase = cl_phaseHG(mu, u.march.z);

    float transmittance = 1.0;
    float3 scattered = float3(0.0);
    float3 sunLight = u.sunColor.rgb * u.sunColor.w;

    for (int i = 0; i < viewSteps; i++) {
        float3 p = camPos + dir * (t0 + ds * (float(i) + 0.5));
        float density = cl_density(p, u);
        if (density > 0.001) {
            float lightOD = 0.0;
            float lds = (u.layer.y - u.layer.x) / float(max(1, lightSteps));
            for (int j = 0; j < lightSteps; j++) {
                float3 lp = p + sunDir * (lds * (float(j) + 0.5));
                lightOD += cl_density(lp, u) * lds;
            }
            float beer = exp(-lightOD);
            float powder = 1.0 - exp(-2.0 * density * ds);
            float3 lum = (sunLight * beer * phase * powder + u.skyAmbient.rgb) * density;
            float stepT = exp(-density * ds);
            scattered += transmittance * lum * ds;
            transmittance *= stepT;
            if (transmittance < 0.01) break;
        }
    }

    return float4(scene * transmittance + scattered, 1.0);
}
