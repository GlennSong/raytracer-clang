// THE PHYSICAL SKY, SAMPLED (ADR-0367): shared by sky.frag (the sky you see) and mesh.frag (its light on surfaces and
// the haze), included after the Globals block. sky_view.comp draws the dome each frame in Hillaire's sky-view mapping
// from the camera's height, lit by the active light (g.skySunDir: the sun by day, the moon by night) and the other;
// radiance in engine light units (ADR-0366). envMode (g.counts.z) 2 when it is on.

layout(set = 0, binding = 11) uniform sampler2D skyViewTex;   // RGB radiance (units)
layout(set = 0, binding = 12) uniform sampler2D skyTransTex;  // RGB transmittance (Bruneton r, mu)

bool skyIsPhysical() { return g.counts.z == 2; }

// the transmittance from the camera toward a direction with cosine-zenith mu (0 into the ground)
vec3 skyTransmittanceToward(float mu) {
    float rg = g.skyPhysical.x, rt = g.skyPhysical2.z;
    float r = rg + max(g.skyPhysical.y, 0.5);
    float horizonMu = -sqrt(max(1.0 - (rg * rg) / (r * r), 0.0));
    if (mu < horizonMu) return vec3(0.0);
    float H = sqrt(max(rt * rt - rg * rg, 0.0));
    float rho = sqrt(max(r * r - rg * rg, 0.0));
    float dd = r * r * (mu * mu - 1.0) + rt * rt;
    float d = max(-r * mu + sqrt(max(dd, 0.0)), 0.0);
    float dMin = rt - r, dMax = rho + H;
    return textureLod(skyTransTex, vec2(clamp((d - dMin) / max(dMax - dMin, 1e-4), 0.0, 1.0), clamp(rho / max(H, 1e-4), 0.0, 1.0)), 0.0).rgb;
}

vec2 skyViewUv(float viewCos, float azim) {
    float rg = g.skyPhysical.x;
    float r = rg + max(g.skyPhysical.y, 0.5);
    float vHorizon = sqrt(max(r * r - rg * rg, 0.0));
    float beta = acos(clamp(vHorizon / r, -1.0, 1.0));
    float zenithHorizon = 3.14159265 - beta;
    float viewZenith = acos(clamp(viewCos, -1.0, 1.0));
    float v;
    if (viewZenith < zenithHorizon) {
        float c = viewZenith / max(zenithHorizon, 1e-4);
        v = (1.0 - sqrt(max(1.0 - c, 0.0))) * 0.5;
    } else {
        float c = (viewZenith - zenithHorizon) / max(beta, 1e-4);
        v = 0.5 + 0.5 * sqrt(clamp(c, 0.0, 1.0));
    }
    return vec2(sqrt(clamp(azim / 3.14159265, 0.0, 1.0)), v);
}

// The dome's radiance toward `dir` (the sky; no disc). Just under the horizon it blends back toward the horizon's own
// radiance over a few degrees: on a finite world that band stands for distant land and reads as haze, not soil (the
// Metal scattering sky's rule).
vec3 skyViewRadiance(vec3 dir) {
    vec2 vh = dir.xz, sh = g.skySunDir.xz;
    float vl = length(vh), sl = length(sh);
    float azim = (vl > 1e-4 && sl > 1e-4) ? acos(clamp(dot(vh / vl, sh / sl), -1.0, 1.0)) : 0.0;
    vec3 col = textureLod(skyViewTex, skyViewUv(dir.y, azim), 0.0).rgb;
    float rg = g.skyPhysical.x;
    float r = rg + max(g.skyPhysical.y, 0.5);
    float zenithHorizon = 3.14159265 - acos(clamp(sqrt(max(r * r - rg * rg, 0.0)) / r, -1.0, 1.0));
    float dip = acos(clamp(dir.y, -1.0, 1.0)) - zenithHorizon;
    if (dip > 0.0) {
        float w = 1.0 - clamp(dip / 0.10, 0.0, 1.0);
        vec3 horizonCol = textureLod(skyViewTex, skyViewUv(cos(zenithHorizon * 0.995), azim), 0.0).rgb;
        col = mix(col, horizonCol, w * w);
    }
    // the natural night sky's floor (airglow and starlight, ~2e-4 cd/m^2): never quite black
    col += vec3(0.55, 0.62, 0.85) * 1.0e-8;
    return col;
}

// The active light's disc, reddened and dimmed by the air on the way, with its aureole: radiance (units). The sun's
// disc is ~1.6e9 cd/m^2 at noon; kept as it is, pre-exposed it would overflow fp16, so it is held to a bright that
// blooms (the Metal sky's rule: an "artistic disc").
vec3 skyLightDisc(vec3 dir) {
    float top = g.skyPhysical.z;
    if (top <= 0.0 || g.skyPhysical3.x < 0.5) return vec3(0.0);   // (the moon draws as itself: sky.frag moonDisc)
    vec3 t = skyTransmittanceToward(dir.y);
    float cosL = dot(dir, g.skySunDir.xyz);
    float discCos = g.skyPhysical.w;
    float disc = smoothstep(discCos - 0.0002, discCos, cosL) * g.skyPhysical2.x;
    // the real disc's radiance over its solid angle (2 pi (1 - cos r)), capped at 2000x the top illuminance
    float solid = max(6.2831853 * (1.0 - discCos), 1e-6);
    vec3 col = t * top * min(1.0 / solid, 2000.0) * disc;
    float c2 = clamp(cosL, 0.0, 1.0);
    col += t * top * (pow(c2, 4000.0) * 0.5 + pow(c2, 250.0) * 0.02) * g.skyPhysical2.x;
    return col;
}
