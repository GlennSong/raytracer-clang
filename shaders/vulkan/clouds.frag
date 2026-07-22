#version 450
// Volumetric clouds — a fullscreen raymarched cloud pass (procedural-planet-plan
// P4). The density + lighting + march core is SHARED between two domains, selected
// by `domainMode`:
//   0 = SLAB   — a horizontal layer between two altitudes: the cloudscape over a
//                ground scene (the procgen city / terrain sky). Occluded by the
//                scene depth so clouds hide behind buildings and hills.
//   1 = SHELL  — a spherical layer around a planet centre: the planet's cloud deck
//                seen from space.
// So this one shader serves the city AND the planet, and supersedes the flat 2D
// FBM sky clouds (sky.frag applyClouds) with real volumetrics. Density here is a
// compact inline hash-fbm (a Perlin-Worley 3D-texture set is the on-device upgrade);
// lighting is a sun light-march (Beer + Powder) with a Henyey-Greenstein phase.
// Output is scene-linear HDR; the composite pass tone maps. Vertex: composite.vert.

layout(set = 0, binding = 0) uniform Clouds {
    mat4  invViewProjection;
    vec4  cameraPosition;    // xyz
    vec4  sunDirection;      // xyz toward the sun
    vec4  sunColor;          // rgb, w = intensity
    vec4  skyAmbient;        // rgb ambient toward the shadowed side, w = time (drift)
    vec4  planetCenter;      // xyz (shell mode)
    vec4  layer;             // x bottom, y top (altitude for slab / radius for shell),
                             //   z planetRadius (shell), w domainMode (0 slab, 1 shell)
    vec4  params;            // x coverage[0..1], y densityScale, z noiseScale, w windSpeed
    vec4  march;             // x viewSteps, y lightSteps, z phaseG, w farDistance
} c;

layout(set = 0, binding = 1) uniform sampler2D sceneColor;
layout(set = 0, binding = 2) uniform sampler2D sceneDepth;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

const float PI = 3.14159265359;

// --- inline hash value-noise + fbm (stand-in for a 3D Perlin-Worley texture) ---
float hash13(vec3 p) {
    p = fract(p * 0.3183099 + vec3(0.1, 0.2, 0.3));
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}
float valueNoise(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = hash13(i + vec3(0, 0, 0));
    float n100 = hash13(i + vec3(1, 0, 0));
    float n010 = hash13(i + vec3(0, 1, 0));
    float n110 = hash13(i + vec3(1, 1, 0));
    float n001 = hash13(i + vec3(0, 0, 1));
    float n101 = hash13(i + vec3(1, 0, 1));
    float n011 = hash13(i + vec3(0, 1, 1));
    float n111 = hash13(i + vec3(1, 1, 1));
    return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
               mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}
float fbm(vec3 p) {
    float sum = 0.0, amp = 0.5;
    for (int i = 0; i < 5; i++) {
        sum += valueNoise(p) * amp;
        p *= 2.02;
        amp *= 0.5;
    }
    return sum;
}

// Height fraction [0,1] across the cloud layer for a world point, per domain.
float layerHeight(vec3 p) {
    float h;
    if (c.layer.w < 0.5) h = p.y;                                  // slab altitude
    else h = length(p - c.planetCenter.xyz) - c.layer.z;           // shell altitude
    return clamp((h - c.layer.x) / max(1e-4, c.layer.y - c.layer.x), 0.0, 1.0);
}

// Cloud density [0,1] at a world point.
float cloudDensity(vec3 p) {
    float hf = layerHeight(p);
    if (hf <= 0.0 || hf >= 1.0) return 0.0;
    // Round-bottom / wispy-top vertical profile.
    float profile = smoothstep(0.0, 0.15, hf) * smoothstep(1.0, 0.55, hf);

    vec3 wind = vec3(c.params.w * c.skyAmbient.w, 0.0, 0.0);
    vec3 q = (p + wind) * c.params.z;
    float base = fbm(q);
    // Coverage threshold carves the sky between clouds; renormalize the remainder.
    float d = clamp((base - (1.0 - c.params.x)) / max(1e-3, c.params.x), 0.0, 1.0);
    // Erode the edges with higher-frequency detail.
    float detail = fbm(q * 3.1 + 4.0);
    d = clamp(d - detail * 0.35 * (1.0 - d), 0.0, 1.0);
    return d * profile * c.params.y;
}

// Henyey-Greenstein phase.
float phaseHG(float mu, float g) {
    float g2 = g * g;
    return (1.0 - g2) / (4.0 * PI * pow(1.0 + g2 - 2.0 * g * mu, 1.5));
}

// Entry/exit of the cloud layer along the ray, per domain. Returns false if missed.
bool layerInterval(vec3 origin, vec3 dir, out float t0, out float t1) {
    if (c.layer.w < 0.5) {
        // Slab between y = bottom and y = top.
        if (abs(dir.y) < 1e-5) {
            float y = origin.y;
            if (y < c.layer.x || y > c.layer.y) return false;
            t0 = 0.0; t1 = c.march.w; return true;
        }
        float ta = (c.layer.x - origin.y) / dir.y;
        float tb = (c.layer.y - origin.y) / dir.y;
        t0 = max(min(ta, tb), 0.0);
        t1 = max(ta, tb);
        return t1 > 0.0;
    }
    // Shell: between concentric spheres (inner = bottom radius, outer = top radius).
    vec3 oc = origin - c.planetCenter.xyz;
    float b = dot(oc, dir);
    // Outer sphere.
    float cOut = dot(oc, oc) - c.layer.y * c.layer.y;
    float discOut = b * b - cOut;
    if (discOut < 0.0) return false;
    float sOut = sqrt(discOut);
    float outerNear = -b - sOut, outerFar = -b + sOut;
    if (outerFar < 0.0) return false;
    t0 = max(outerNear, 0.0);
    t1 = outerFar;
    // Clip the far end at the inner sphere (the deck's underside / planet).
    float cIn = dot(oc, oc) - c.layer.x * c.layer.x;
    float discIn = b * b - cIn;
    if (discIn > 0.0) {
        float sIn = sqrt(discIn);
        float innerNear = -b - sIn;
        if (innerNear > 0.0) t1 = min(t1, innerNear);
    }
    return t1 > t0;
}

// Distance to the nearest opaque scene surface along the ray (so clouds behind
// geometry are occluded). Returns farDistance for background pixels.
float sceneDistance(vec3 camPos, vec3 dir) {
    float depth = texture(sceneDepth, inUV).r;
    if (depth <= 0.0) return c.march.w;   // reverse-Z background (far plane = 0)
    vec4 world = c.invViewProjection * vec4(inUV * 2.0 - 1.0, depth, 1.0);
    return length(world.xyz / world.w - camPos);
}

void main() {
    vec4 world = c.invViewProjection * vec4(inUV * 2.0 - 1.0, 1.0, 1.0);
    vec3 camPos = c.cameraPosition.xyz;
    vec3 dir = normalize(world.xyz / world.w - camPos);
    vec3 sunDir = normalize(c.sunDirection.xyz);
    vec3 scene = texture(sceneColor, inUV).rgb;

    float t0, t1;
    if (!layerInterval(camPos, dir, t0, t1)) { outColor = vec4(scene, 1.0); return; }
    t1 = min(t1, sceneDistance(camPos, dir));
    if (t1 <= t0) { outColor = vec4(scene, 1.0); return; }

    int viewSteps = int(c.march.x);
    int lightSteps = int(c.march.y);
    float ds = (t1 - t0) / float(max(2, viewSteps));
    float mu = dot(dir, sunDir);
    float phase = phaseHG(mu, c.march.z);

    float transmittance = 1.0;
    vec3 scattered = vec3(0.0);
    vec3 sunLight = c.sunColor.rgb * c.sunColor.w;

    for (int i = 0; i < viewSteps; i++) {
        vec3 p = camPos + dir * (t0 + ds * (float(i) + 0.5));
        float density = cloudDensity(p);
        if (density > 0.001) {
            // Light march toward the sun: optical depth through the cloud.
            float lightOD = 0.0;
            float lds = (c.layer.y - c.layer.x) / float(max(1, lightSteps));
            for (int j = 0; j < lightSteps; j++) {
                vec3 lp = p + sunDir * (lds * (float(j) + 0.5));
                lightOD += cloudDensity(lp) * lds;
            }
            // Beer + a Powder term for the silver-lining darkening near the edges.
            float beer = exp(-lightOD);
            float powder = 1.0 - exp(-2.0 * density * ds);
            vec3 sunTerm = sunLight * beer * phase;
            vec3 ambient = c.skyAmbient.rgb;
            vec3 lum = (sunTerm * powder + ambient) * density;

            float stepT = exp(-density * ds);
            scattered += transmittance * lum * ds;
            transmittance *= stepT;
            if (transmittance < 0.01) break;
        }
    }

    outColor = vec4(scene * transmittance + scattered, 1.0);
}
