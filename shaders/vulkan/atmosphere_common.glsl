// THE ATMOSPHERE'S SHARED PIECES (ADR-0367, Hillaire 2020 over Bruneton's tables; engine/sky/sky_atmosphere.cpp the
// CPU original): the ray's distances to the air's top and the ground, the transmittance and multiple-scattering table
// lookups, the ozone layer, the two phase functions, and the sky view's parameterisation with its below-horizon blend.
// One copy for sky_view.comp, aerial_volume.comp, sky_cube.comp and sky_physical.glsl (each passes its own planet: a
// push constant or the globals). rg the ground radius, rt the air's top, r the point's radius, mu its cosine-zenith.
#ifndef RT_ATMOSPHERE_COMMON_GLSL
#define RT_ATMOSPHERE_COMMON_GLSL

float atmDistToTop(float r, float mu, float rt) {
    float d = r * r * (mu * mu - 1.0) + rt * rt;
    return max(-r * mu + sqrt(max(d, 0.0)), 0.0);
}
// (-1: the ray misses the ground)
float atmDistToGround(float r, float mu, float rg) {
    float d = r * r * (mu * mu - 1.0) + rg * rg;
    if (d < 0.0 || mu >= 0.0) return -1.0;
    float t = -r * mu - sqrt(d);
    return t > 0.0 ? t : -1.0;
}
// the transmittance from radius r toward mu, to the air's top (0 into the ground)
vec3 atmTransmittance(sampler2D lut, float r, float mu, float rg, float rt) {
    r = clamp(r, rg, rt);
    float horizonMu = -sqrt(max(1.0 - (rg * rg) / (r * r), 0.0));
    if (mu < horizonMu) return vec3(0.0);
    float H = sqrt(max(rt * rt - rg * rg, 0.0));
    float rho = sqrt(max(r * r - rg * rg, 0.0));
    float d = atmDistToTop(r, mu, rt), dMin = rt - r, dMax = rho + H;
    vec2 uv = vec2(clamp((d - dMin) / max(dMax - dMin, 1e-4), 0.0, 1.0), clamp(rho / max(H, 1e-4), 0.0, 1.0));
    return textureLod(lut, uv, 0.0).rgb;
}
vec3 atmMultiScatter(sampler2D lut, float r, float muS, float rg, float rt) {
    vec2 uv = vec2(clamp(muS * 0.5 + 0.5, 0.0, 1.0), clamp((r - rg) / (rt - rg), 0.0, 1.0));
    return textureLod(lut, uv, 0.0).rgb;
}
// the ozone layer's density at height h: a tent peaking at 25 km
float atmOzoneDensity(float h) { return max(0.0, 1.0 - abs(h - 25000.0) / 15000.0); }

float atmRayleighPhase(float nu) { return 3.0 / (16.0 * 3.14159265) * (1.0 + nu * nu); }
// Cornette-Shanks, asymmetry g
float atmMiePhase(float nu, float g) {
    float g2 = g * g;
    return 3.0 / (8.0 * 3.14159265) * ((1.0 - g2) * (1.0 + nu * nu)) / ((2.0 + g2) * pow(1.0 + g2 - 2.0 * g * nu, 1.5));
}

// THE SKY VIEW's texel for a view (its cosine-zenith, its azimuth from the light) seen from `height` above the ground:
// the horizon at the middle row, the rows packed toward it
vec2 atmSkyViewUv(float viewCos, float azim, float rg, float height) {
    float r = rg + max(height, 0.5);
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
// The dome's radiance toward `dir` from the sky view (`light` the view's reference direction). Just under the horizon
// it blends back toward the horizon's own radiance over a few degrees: on a finite world that band stands for distant
// land and reads as haze, not soil (the Metal scattering sky's rule).
vec3 atmSkyViewRadiance(sampler2D view, vec3 dir, vec3 light, float rg, float height) {
    vec2 vh = dir.xz, sh = light.xz;
    float vl = length(vh), sl = length(sh);
    float azim = (vl > 1e-4 && sl > 1e-4) ? acos(clamp(dot(vh / vl, sh / sl), -1.0, 1.0)) : 0.0;
    vec3 col = textureLod(view, atmSkyViewUv(dir.y, azim, rg, height), 0.0).rgb;
    float r = rg + max(height, 0.5);
    float zenithHorizon = 3.14159265 - acos(clamp(sqrt(max(r * r - rg * rg, 0.0)) / r, -1.0, 1.0));
    float dip = acos(clamp(dir.y, -1.0, 1.0)) - zenithHorizon;
    if (dip > 0.0) {
        float w = 1.0 - clamp(dip / 0.10, 0.0, 1.0);
        col = mix(col, textureLod(view, atmSkyViewUv(cos(zenithHorizon * 0.995), azim, rg, height), 0.0).rgb, w * w);
    }
    return col;
}

#endif
