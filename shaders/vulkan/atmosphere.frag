#version 450
// Planetary atmosphere — realtime port of the CPU reference (engine/procgen/
// atmosphere.cpp), single-scattering Rayleigh + Mie (Nishita/O'Neil core). A
// fullscreen post pass (procedural-planet-plan P3): reconstruct the world ray per
// pixel, raymarch the atmosphere shell, and composite the in-scattered light over
// the already-rendered scene by its transmittance. The planet is an analytic sphere
// so the march clips at the surface without a depth fetch (from-orbit relief is far
// below the shell thickness). Output is scene-linear HDR; the composite pass tone
// maps. Multiple scattering (Hillaire) is the follow-up; this matches the tested CPU
// model 1:1. Vertex stage: composite.vert (fullscreen triangle → outUV).

layout(set = 0, binding = 0) uniform Atmosphere {
    mat4  invViewProjection;
    vec4  cameraPosition;     // xyz
    vec4  sunDirection;       // xyz, toward the sun
    vec4  planetCenter;       // xyz
    vec4  sunColor;           // rgb, w = sun intensity
    vec4  rayleighCoeff;      // rgb (per length), a unused
    vec4  radii;              // x planetRadius, y atmosphereRadius, z rayleighH, w mieH
    vec4  mie;                // x mieCoeff, y mieG, z viewSamples, w lightSamples
} a;

layout(set = 0, binding = 1) uniform sampler2D sceneColor;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

const float PI = 3.14159265359;

// t0<t1 where origin+t*dir meets the sphere of `radius` at the origin. Returns false
// on a miss or a sphere fully behind the ray.
bool raySphere(vec3 origin, vec3 dir, float radius, out float t0, out float t1) {
    float b = dot(origin, dir);
    float c = dot(origin, origin) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.0) return false;
    float s = sqrt(disc);
    t0 = -b - s;
    t1 = -b + s;
    return t1 >= 0.0;
}

float densityAt(vec3 q, float planetRadius, float scaleH) {
    float h = length(q) - planetRadius;
    return exp(-max(h, 0.0) / scaleH);
}

void opticalDepth(vec3 pa, vec3 pb, int steps, float planetRadius, float rH, float mH,
                  out float odR, out float odM) {
    odR = 0.0; odM = 0.0;
    vec3 seg = pb - pa;
    float len = length(seg);
    if (len < 1e-6) return;
    vec3 d = seg / len;
    float ds = len / float(steps);
    for (int i = 0; i < steps; i++) {
        vec3 q = pa + d * (ds * (float(i) + 0.5));
        odR += densityAt(q, planetRadius, rH) * ds;
        odM += densityAt(q, planetRadius, mH) * ds;
    }
}

void main() {
    // Reconstruct the world-space view ray for this pixel.
    vec4 world = a.invViewProjection * vec4(inUV * 2.0 - 1.0, 1.0, 1.0);
    vec3 camPos = a.cameraPosition.xyz;
    vec3 dir = normalize(world.xyz / world.w - camPos);
    vec3 sunDir = normalize(a.sunDirection.xyz);

    vec3 scene = texture(sceneColor, inUV).rgb;

    float planetRadius = a.radii.x, atmosRadius = a.radii.y;
    float rH = a.radii.z, mH = a.radii.w;
    float mieCoeff = a.mie.x, mieG = a.mie.y;
    int viewSamples = int(a.mie.z), lightSamples = int(a.mie.w);
    vec3 rayleighCoeff = a.rayleighCoeff.rgb;

    // Work relative to the planet centre (the sphere maths assume it at the origin).
    vec3 origin = camPos - a.planetCenter.xyz;

    float aEnter, aExit;
    if (!raySphere(origin, dir, atmosRadius, aEnter, aExit)) {
        outColor = vec4(scene, 1.0);   // ray never touches the atmosphere
        return;
    }

    // Clip the march at the planet surface if the ray hits it.
    float tMax = aExit;
    float g0, g1;
    if (raySphere(origin, dir, planetRadius, g0, g1) && g0 > 0.0) tMax = min(tMax, g0);
    float tEnter = max(0.0, aEnter);
    if (tMax <= tEnter) { outColor = vec4(scene, 1.0); return; }

    int steps = max(2, viewSamples);
    float ds = (tMax - tEnter) / float(steps);

    float mu = dot(dir, sunDir);
    float phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float g = mieG;
    float phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
                   ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));

    float odViewR = 0.0, odViewM = 0.0;
    vec3 inscatR = vec3(0.0), inscatM = vec3(0.0);

    for (int i = 0; i < steps; i++) {
        float t = tEnter + ds * (float(i) + 0.5);
        vec3 q = origin + dir * t;

        float dR = densityAt(q, planetRadius, rH) * ds;
        float dM = densityAt(q, planetRadius, mH) * ds;
        odViewR += dR; odViewM += dM;

        // Skip samples the planet shadows from the sun.
        float s0, s1;
        bool shadowed = raySphere(q, sunDir, planetRadius, s0, s1) && s1 > 0.0 && s0 > 0.0;
        if (shadowed) continue;

        float la0, la1;
        raySphere(q, sunDir, atmosRadius, la0, la1);
        vec3 lightExit = q + sunDir * max(0.0, la1);
        float odLR, odLM;
        opticalDepth(q, lightExit, lightSamples, planetRadius, rH, mH, odLR, odLM);

        vec3 tau = rayleighCoeff * (odViewR + odLR) + vec3(mieCoeff * (odViewM + odLM));
        vec3 tr = exp(-tau);
        inscatR += tr * dR;
        inscatM += tr * dM;
    }

    vec3 rayleigh = rayleighCoeff * inscatR * phaseR;
    vec3 mie = inscatM * (mieCoeff * phaseM);
    vec3 inScatter = a.sunColor.rgb * (rayleigh + mie) * a.sunColor.w;

    vec3 tauView = rayleighCoeff * odViewR + vec3(mieCoeff * odViewM);
    vec3 viewTransmittance = exp(-tauView);

    outColor = vec4(scene * viewTransmittance + inScatter, 1.0);
}
