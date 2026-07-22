// Planetary atmosphere — Metal port of the CPU reference (engine/procgen/
// atmosphere.cpp) and the Vulkan atmosphere.frag, single-scattering Rayleigh + Mie
// (procedural-planet-plan P3). A fullscreen pass: reconstruct the world ray per
// pixel, raymarch the atmosphere shell, composite the in-scattered light over the
// HDR scene by its transmittance. The planet is an analytic sphere so the march
// clips at the surface without a depth fetch.
//
// This file is written to be CONCATENATED into the Metal library like the other
// shaders/metal/*.metal (it relies on the prepended metal_stdlib / shader_types.h,
// per metal_renderer.mm). To wire it on a Mac: (1) add "atmosphere.metal" to the
// concatenation list in metal_renderer.mm's shader load, (2) move AtmosphereUniforms
// to shader_types.h (shared C++/MSL), (3) build a pipeline from vertexAtmosphere /
// fragmentAtmosphere and insert the pass after the scene, before the composite tone
// map. The maths matches the SPIR-V-verified atmosphere.frag 1:1; only the MSL
// syntax here is compile-unverified (no macOS toolchain in CI).

// AtmosphereUniforms is defined in shader_types.h (prepended by the shader loader).

struct AtmosphereOut {
    float4 position [[position]];
    float2 uv;
};

static bool atmRaySphere(float3 origin, float3 dir, float radius,
                         thread float& t0, thread float& t1) {
    float b = dot(origin, dir);
    float c = dot(origin, origin) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.0) return false;
    float s = sqrt(disc);
    t0 = -b - s;
    t1 = -b + s;
    return t1 >= 0.0;
}

static float atmDensity(float3 q, float planetRadius, float scaleH) {
    float h = length(q) - planetRadius;
    return exp(-max(h, 0.0) / scaleH);
}

// Returns (odRayleigh, odMie) along pa->pb.
static float2 atmOpticalDepth(float3 pa, float3 pb, int steps,
                              float planetRadius, float rH, float mH) {
    float2 od = float2(0.0);
    float3 seg = pb - pa;
    float len = length(seg);
    if (len < 1e-6) return od;
    float3 d = seg / len;
    float ds = len / float(steps);
    for (int i = 0; i < steps; i++) {
        float3 q = pa + d * (ds * (float(i) + 0.5));
        od.x += atmDensity(q, planetRadius, rH) * ds;
        od.y += atmDensity(q, planetRadius, mH) * ds;
    }
    return od;
}

vertex AtmosphereOut vertexAtmosphere(uint vid [[vertex_id]]) {
    AtmosphereOut out;
    float2 uv = float2((vid << 1) & 2, vid & 2);
    out.uv = uv;
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    return out;
}

fragment float4 fragmentAtmosphere(AtmosphereOut in [[stage_in]],
                                   texture2d<float> sceneColor [[texture(0)]],
                                   constant AtmosphereUniforms& a [[buffer(0)]]) {
    constexpr sampler samp(coord::normalized, filter::linear, address::clamp_to_edge);
    const float PI = 3.14159265359;

    // Metal NDC has a flipped Y vs the UV (see post.metal ssrViewPos).
    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = a.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 camPos = a.cameraPosition.xyz;
    float3 dir = normalize(world.xyz / world.w - camPos);
    float3 sunDir = normalize(a.sunDirection.xyz);

    float3 scene = sceneColor.sample(samp, in.uv).rgb;

    float planetRadius = a.radii.x, atmosRadius = a.radii.y;
    float rH = a.radii.z, mH = a.radii.w;
    float mieCoeff = a.mie.x, mieG = a.mie.y;
    int viewSamples = int(a.mie.z), lightSamples = int(a.mie.w);
    float3 rayleighCoeff = a.rayleighCoeff.rgb;

    float3 origin = camPos - a.planetCenter.xyz;

    float aEnter, aExit;
    if (!atmRaySphere(origin, dir, atmosRadius, aEnter, aExit)) return float4(scene, 1.0);

    float tMax = aExit;
    float g0, g1;
    if (atmRaySphere(origin, dir, planetRadius, g0, g1) && g0 > 0.0) tMax = min(tMax, g0);
    float tEnter = max(0.0, aEnter);
    if (tMax <= tEnter) return float4(scene, 1.0);

    int steps = max(2, viewSamples);
    float ds = (tMax - tEnter) / float(steps);

    float mu = dot(dir, sunDir);
    float phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float g = mieG;
    float phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
                   ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));

    float odViewR = 0.0, odViewM = 0.0;
    float3 inscatR = float3(0.0), inscatM = float3(0.0);

    for (int i = 0; i < steps; i++) {
        float t = tEnter + ds * (float(i) + 0.5);
        float3 q = origin + dir * t;

        float dR = atmDensity(q, planetRadius, rH) * ds;
        float dM = atmDensity(q, planetRadius, mH) * ds;
        odViewR += dR; odViewM += dM;

        float s0, s1;
        bool shadowed = atmRaySphere(q, sunDir, planetRadius, s0, s1) && s1 > 0.0 && s0 > 0.0;
        if (shadowed) continue;

        float la0, la1;
        atmRaySphere(q, sunDir, atmosRadius, la0, la1);
        float3 lightExit = q + sunDir * max(0.0, la1);
        float2 odL = atmOpticalDepth(q, lightExit, lightSamples, planetRadius, rH, mH);

        float3 tau = rayleighCoeff * (odViewR + odL.x) + float3(mieCoeff * (odViewM + odL.y));
        float3 tr = exp(-tau);
        inscatR += tr * dR;
        inscatM += tr * dM;
    }

    float3 rayleigh = rayleighCoeff * inscatR * phaseR;
    float3 mieTerm = inscatM * (mieCoeff * phaseM);
    float3 inScatter = a.sunColor.rgb * (rayleigh + mieTerm) * a.sunColor.w;

    float3 tauView = rayleighCoeff * odViewR + float3(mieCoeff * odViewM);
    float3 viewTransmittance = exp(-tauView);

    return float4(scene * viewTransmittance + inScatter, 1.0);
}

// Additive variant used by the realtime pass: outputs ONLY the in-scattered light
// (no scene sample, no transmittance), for a fullscreen triangle blended One+One
// over the HDR scene. The limb halo + haze then bloom in the post pass. This is the
// entry point the Metal renderer's atmospherePipeline binds.
fragment float4 fragmentAtmosphereGlow(AtmosphereOut in [[stage_in]],
                                       constant AtmosphereUniforms& a [[buffer(0)]]) {
    const float PI = 3.14159265359;
    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = a.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 camPos = a.cameraPosition.xyz;
    float3 dir = normalize(world.xyz / world.w - camPos);
    float3 sunDir = normalize(a.sunDirection.xyz);

    float planetRadius = a.radii.x, atmosRadius = a.radii.y;
    float rH = a.radii.z, mH = a.radii.w;
    float mieCoeff = a.mie.x, mieG = a.mie.y;
    int viewSamples = int(a.mie.z), lightSamples = int(a.mie.w);
    float3 rayleighCoeff = a.rayleighCoeff.rgb;

    float3 origin = camPos - a.planetCenter.xyz;

    float aEnter, aExit;
    if (!atmRaySphere(origin, dir, atmosRadius, aEnter, aExit)) return float4(0.0);
    float tMax = aExit;
    float g0, g1;
    if (atmRaySphere(origin, dir, planetRadius, g0, g1) && g0 > 0.0) tMax = min(tMax, g0);
    float tEnter = max(0.0, aEnter);
    if (tMax <= tEnter) return float4(0.0);

    int steps = max(2, viewSamples);
    float ds = (tMax - tEnter) / float(steps);
    float mu = dot(dir, sunDir);
    float phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float g = mieG;
    float phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
                   ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));

    float odViewR = 0.0, odViewM = 0.0;
    float3 inscatR = float3(0.0), inscatM = float3(0.0);
    for (int i = 0; i < steps; i++) {
        float t = tEnter + ds * (float(i) + 0.5);
        float3 q = origin + dir * t;
        float dR = atmDensity(q, planetRadius, rH) * ds;
        float dM = atmDensity(q, planetRadius, mH) * ds;
        odViewR += dR; odViewM += dM;
        float s0, s1;
        bool shadowed = atmRaySphere(q, sunDir, planetRadius, s0, s1) && s1 > 0.0 && s0 > 0.0;
        if (shadowed) continue;
        float la0, la1;
        atmRaySphere(q, sunDir, atmosRadius, la0, la1);
        float3 lightExit = q + sunDir * max(0.0, la1);
        float2 odL = atmOpticalDepth(q, lightExit, lightSamples, planetRadius, rH, mH);
        float3 tau = rayleighCoeff * (odViewR + odL.x) + float3(mieCoeff * (odViewM + odL.y));
        float3 tr = exp(-tau);
        inscatR += tr * dR;
        inscatM += tr * dM;
    }
    float3 rayleigh = rayleighCoeff * inscatR * phaseR;
    float3 mieTerm = inscatM * (mieCoeff * phaseM);
    float3 inScatter = a.sunColor.rgb * (rayleigh + mieTerm) * a.sunColor.w;
    return float4(inScatter, 1.0);
}
