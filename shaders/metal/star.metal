// Star / sun (procedural-planet-plan P6) — Metal port of star.frag. The local star
// as a bright HDR disc with limb darkening + a blackbody colour from its temperature,
// plus a bloom-friendly glow, composited over the background (gated on the reverse-Z
// far plane). Concatenation-ready (relies on the prepended metal_stdlib). Mirrors the
// SPIR-V-verified GLSL; MSL is compile-unverified in CI. Wire on device: add to the
// concat list, move StarUniforms to shader_types.h, build a pipeline from
// vertexStar / fragmentStar, run it over the scene with color + depth bound.

struct StarUniforms {
    float4x4 invViewProjection;
    float4   cameraPosition;
    float4   starDirection;   // xyz toward the star, w angular radius (rad)
    float4   tune;            // x temperatureK, y intensity, z glowFalloff, w limbDark
};

struct StarOut {
    float4 position [[position]];
    float2 uv;
};

static float3 blackbody(float kelvin) {
    float t = clamp(kelvin, 1000.0, 40000.0) / 100.0;
    float r, g, b;
    if (t <= 66.0) r = 1.0;
    else r = clamp(1.292 * pow(t - 60.0, -0.1332), 0.0, 1.0);
    if (t <= 66.0) g = clamp(0.390 * log(t) - 0.631, 0.0, 1.0);
    else g = clamp(1.130 * pow(t - 60.0, -0.0755), 0.0, 1.0);
    if (t >= 66.0) b = 1.0;
    else if (t <= 19.0) b = 0.0;
    else b = clamp(0.543 * log(t - 10.0) - 1.196, 0.0, 1.0);
    return float3(r, g, b);
}

vertex StarOut vertexStar(uint vid [[vertex_id]]) {
    StarOut out;
    float2 uv = float2((vid << 1) & 2, vid & 2);
    out.uv = uv;
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    return out;
}

fragment float4 fragmentStar(StarOut in [[stage_in]],
                             texture2d<float> sceneColor [[texture(0)]],
                             depth2d<float> sceneDepth [[texture(1)]],
                             constant StarUniforms& s [[buffer(0)]]) {
    constexpr sampler samp(coord::normalized, filter::linear, address::clamp_to_edge);
    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = s.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 dir = normalize(world.xyz / world.w - s.cameraPosition.xyz);
    float3 toStar = normalize(s.starDirection.xyz);
    float3 scene = sceneColor.sample(samp, in.uv).rgb;

    float depth = sceneDepth.sample(samp, in.uv);
    if (depth > 0.0) return float4(scene, 1.0);

    float ang = acos(clamp(dot(dir, toStar), -1.0, 1.0));
    float radius = max(1e-4, s.starDirection.w);
    float3 color = blackbody(s.tune.x);
    float intensity = s.tune.y;

    float3 add = float3(0.0);
    if (ang < radius) {
        float r = ang / radius;
        float limb = mix(1.0, s.tune.w, r * r);
        add = color * intensity * limb;
    } else {
        float d = (ang - radius) / radius;
        add = color * intensity * exp(-d * s.tune.z);
    }
    return float4(scene + add, 1.0);
}
