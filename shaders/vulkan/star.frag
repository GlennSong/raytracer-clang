#version 450
// Star / sun (procedural-planet-plan P6): the local star as a bright HDR disc with
// limb darkening and a blackbody colour from its temperature, plus a soft glow that
// the bloom pass blooms. A fullscreen pass composited over the background (gated on
// the reverse-Z far plane so it stays behind scene geometry). Output is scene-linear
// HDR. Vertex: composite.vert.

layout(set = 0, binding = 0) uniform Star {
    mat4  invViewProjection;
    vec4  cameraPosition;   // xyz
    vec4  starDirection;    // xyz toward the star, w angular radius (radians)
    vec4  tune;             // x temperatureK, y intensity, z glowFalloff, w limbDark
} s;

layout(set = 0, binding = 1) uniform sampler2D sceneColor;
layout(set = 0, binding = 2) uniform sampler2D sceneDepth;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Planckian-locus approximation: blackbody temperature (K) -> linear RGB tint.
// A compact fit (Helland/Bartlett style) over ~1000..40000 K, normalized to ~1 at
// the brightest channel.
vec3 blackbody(float kelvin) {
    float t = clamp(kelvin, 1000.0, 40000.0) / 100.0;
    float r, gg, b;
    // Red
    if (t <= 66.0) r = 1.0;
    else r = clamp(1.292 * pow(t - 60.0, -0.1332), 0.0, 1.0);
    // Green
    if (t <= 66.0) gg = clamp(0.390 * log(t) - 0.631, 0.0, 1.0);
    else gg = clamp(1.130 * pow(t - 60.0, -0.0755), 0.0, 1.0);
    // Blue
    if (t >= 66.0) b = 1.0;
    else if (t <= 19.0) b = 0.0;
    else b = clamp(0.543 * log(t - 10.0) - 1.196, 0.0, 1.0);
    return vec3(r, gg, b);
}

void main() {
    vec4 world = s.invViewProjection * vec4(inUV * 2.0 - 1.0, 1.0, 1.0);
    vec3 dir = normalize(world.xyz / world.w - s.cameraPosition.xyz);
    vec3 toStar = normalize(s.starDirection.xyz);

    vec3 scene = texture(sceneColor, inUV).rgb;

    // Only draw where nothing closer was rendered (reverse-Z background == 0).
    float depth = texture(sceneDepth, inUV).r;
    if (depth > 0.0) { outColor = vec4(scene, 1.0); return; }

    float cosAng = clamp(dot(dir, toStar), -1.0, 1.0);
    float ang = acos(cosAng);
    float radius = max(1e-4, s.starDirection.w);

    vec3 color = blackbody(s.tune.x);
    float intensity = s.tune.y;

    vec3 add = vec3(0.0);
    if (ang < radius) {
        // Inside the disc: limb darkening (dimmer toward the edge).
        float r = ang / radius;
        float limb = mix(1.0, s.tune.w, r * r);   // w = edge brightness fraction
        add = color * intensity * limb;
    } else {
        // Outside: an exponential glow that bloom will pick up.
        float d = (ang - radius) / radius;
        add = color * intensity * exp(-d * s.tune.z);
    }

    outColor = vec4(scene + add, 1.0);
}
