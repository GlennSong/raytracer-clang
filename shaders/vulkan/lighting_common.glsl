// THE LIGHT ON A SURFACE (ADR-0372): the GGX terms, the lights' falloff, the sun's cascaded and the mountains' shadow,
// and one light's contribution -- shared by mesh.frag and probe_relight.comp (include after globals.glsl).

float distributionGGX(float NdotH, float a2) {
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-6);
}
float visibilitySmithGGX(float NdotV, float NdotL, float a2) {
    float gv = NdotL * sqrt(NdotV * NdotV * (1.0 - a2) + a2);
    float gl = NdotV * sqrt(NdotL * NdotL * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}
vec3 fresnelSchlickVec(float cosTheta, vec3 f0) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}
float distanceAttenuation(float dist, float range) {
    float ratio2 = (dist * dist) / max(range * range, 1e-4);
    float window = clamp(1.0 - ratio2 * ratio2, 0.0, 1.0);
    return window * window / max(dist * dist, 1e-4);
}

// Roughness-aware Fresnel for the ambient/IBL term.
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    vec3 r = max(vec3(1.0 - roughness), F0);
    return F0 + (r - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}


// ---- cascaded shadows (ported from shadows.metal) --------------------------
// Vulkan: no uv y-flip (the shadow VP is unflipped, so write and read use the
// same NDC->uv mapping). Forward-Z light depth in [0,1].
float sampleCascade(int slice, vec3 worldPos, vec3 N) {
    vec3 biased = worldPos + N * g.shadowParams.x;          // normalBias
    vec4 clip = g.cascadeVP[slice] * vec4(biased, 1.0);
    vec3 ndc = clip.xyz / clip.w;
    vec2 uv = ndc.xy * 0.5 + 0.5;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return 1.0;
    if (ndc.z < 0.0 || ndc.z > 1.0) return 1.0;
    float texel = 1.0 / g.shadowParams.z;                   // 1 / mapSize
    float pcf = g.shadowParams.y;                           // pcfRadius
    float s = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x) {
            vec2 off = vec2(float(x), float(y)) * texel * pcf;
            s += texture(shadowMap, vec4(uv + off, float(slice), ndc.z));
        }
    return s / 9.0;
}

// TERRAIN HORIZON: mountain shadows at landscape scale (see
// engine/procgen/terrain_horizon.h). sin(horizon elevation) toward the
// slot-0 light over the world square; the light below it is behind the
// ridge. A soft edge of ~1.4 deg so the shadow line does not shimmer as
// the sun moves; outside the map, nothing is known and nothing shadows.
float terrainShadow(vec3 worldPos, vec3 L) {
    if (g.terrainHorizon.w < 0.5) return 1.0;
    vec2 uv = (worldPos.xz - g.terrainHorizon.xy) * g.terrainHorizon.z;
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) return 1.0;
    float h = g.terrainHorizon2.x + g.terrainHorizon2.y * texture(terrainHorizonMap, uv).r;
    return smoothstep(h - 0.012, h + 0.012, L.y);
}

// THE CLOUDS' SHADOW (ADR-0387; Glenn, 2026-11-08, the weather plan's part B): the light's line from this point read
// against the weather map where it crosses each cloud type's middle -- a cloud's footprint there takes up to `w` of the
// sun (a heart ~85 %: the rest comes through and round it, scattered), less for a small shallow cell. Soft, as a
// cloud's shadow on the ground is (the footprint's own edge, ~100 m). Above the deck, or past the map: none.
// Each type at its own height: read at the whole deck's middle, a fair-weather cumulus at 1.5 km under a storm's
// range (the deck to 12 km) was read at ~5 km and cast its shadow kilometres along the sun (until 2026-11-09). A read
// counts only where the map's cell is that type (G: the slot), so a type's line never borrows another's footprint.
float cloudShadowAt(vec2 xz, vec3 L, float y, float height, out vec3 m) {
    float t = (height - y) / L.y;
    m = vec3(0.0);
    if (t <= 0.0) return 0.0;
    vec2 uv = (xz + L.xz * t + g.cloudShadow2.xy - g.cloudShadow.xy) * g.cloudShadow.z;
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) return 0.0;
    m = textureLod(cloudShadowMap, uv, 0.0).rgb;
    float depth = 0.55 + 0.45 * clamp(m.b + m.g * 0.5, 0.0, 1.0);   // a deep or severe cell shades more
    return smoothstep(0.2, 0.7, m.r) * depth;
}
float cloudShadow(vec3 worldPos, vec3 L) {
    if (g.cloudShadow.w <= 0.0 || L.y <= 0.02) return 1.0;
    float shade = 0.0;
    bool typed = false;
    for (int k = 0; k < 4; ++k) {
        if (g.cloudTypeMid[k] <= g.cloudTypeBase[k]) continue;
        typed = true;
        vec3 m;
        float s = cloudShadowAt(worldPos.xz, L, worldPos.y, g.cloudTypeMid[k], m);
        if (abs(m.g * 3.0 - float(k)) < 0.5) shade = max(shade, s);
    }
    if (!typed) {   // (no types handed over: the deck's middle, as before)
        vec3 m;
        shade = cloudShadowAt(worldPos.xz, L, worldPos.y, g.cloudShadow2.z, m);
    }
    return 1.0 - g.cloudShadow.w * shade;
}

// THE TERRAIN NORMAL MAP (ADR-0205, engine/procgen/terrain_normal_map.h): beyond the distance where the CDLOD mesh
// grows coarser than a texel, the ground lights by the baked slope of what it covers rather than its vertices' (16-64
// m apart on a far tile) -- the erosion's gullies and spurs on the far mountains. Blended in over one level's range,
// so no tile edge shows; outside the map's square, the vertex normal.
vec3 terrainMapNormal(vec3 worldPos, vec3 vertexN) {
    if (g.terrainNormal.w < 0.5) return vertexN;
    const vec2 uv = (worldPos.xz - g.terrainNormal.xy) * g.terrainNormal.z;
    const vec2 t = texture(terrainNormalMap, uv).rg * 2.0 - 1.0;   // (uniform flow: a draw is terrain or not)
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) return vertexN;
    const vec3 mapN = vec3(t.x, sqrt(max(1.0 - dot(t, t), 0.0)), t.y);
    const float w = clamp((distance(worldPos, g.cameraPosition.xyz) - g.terrainNormal2.x) * g.terrainNormal2.y, 0.0, 1.0);
    return normalize(mix(vertexN, mapN, w));
}

float computeShadow(vec3 worldPos, vec3 N, float viewDepth) {
    int count = g.counts.y;
    if (count <= 0) return 1.0;
    int c = count - 1;
    for (int i = 0; i < count; ++i)
        if (viewDepth < g.cascadeSplit[i]) { c = i; break; }

    float v = sampleCascade(c, worldPos, N);
    if (c + 1 < count) {
        float splitFar = g.cascadeSplit[c];
        float splitNear = (c == 0) ? 0.0 : g.cascadeSplit[c - 1];
        float bandStart = mix(splitFar, splitNear, 0.15);
        if (viewDepth > bandStart) {
            float t = clamp((viewDepth - bandStart) / (splitFar - bandStart), 0.0, 1.0);
            v = mix(v, sampleCascade(c + 1, worldPos, N), t);
        }
    } else {
        float farSplit = g.cascadeSplit[count - 1];
        float fadeStart = farSplit * 0.8;
        if (viewDepth > fadeStart) {
            float t = clamp((viewDepth - fadeStart) / (farSplit - fadeStart), 0.0, 1.0);
            v = mix(v, 1.0, t);
        }
    }
    return v;
}

// ---- main ------------------------------------------------------------------
// A room's light (ADR-0365) reaches only inside its room's box: 1 inside, 0 past its walls -- its own walls' inner
// faces lit, the next room's not
float roomLightInside(Light light, vec3 worldPos) {
    vec2 d = worldPos.xz - light.directionInner.xy;
    vec2 u = light.directionInner.zw;
    float out_ = max(max(abs(dot(d, u)) - light.typeRange.z, abs(dot(d, vec2(-u.y, u.x))) - light.typeRange.w),
                     light.colorOuter.w - 0.05 - worldPos.y);
    return 1.0 - smoothstep(0.02, 0.12, out_);
}

// THE ROOM'S BOUNCE (Glenn, 2026-11-08: "lights in the interior at night are too dim"): a room's light reaches the
// ceiling and the walls beside it mostly off the floor and the other walls -- in a pale room about as much again as
// it gives directly. Each room light's irradiance at a distance softened by the room's height (the bounce comes from
// everywhere round it), facing no way, at half (a room's average reflectance), kept to its room. Irradiance: the
// caller takes albedo / pi of it.
vec3 roomBounce(Light light, vec3 worldPos) {
    if (int(light.typeRange.x) != 3) return vec3(0.0);
    float dist = length(light.positionIntensity.xyz - worldPos);
    float ratio2 = (dist * dist) / max(light.typeRange.y * light.typeRange.y, 1e-4);
    float window = clamp(1.0 - ratio2 * ratio2, 0.0, 1.0);
    const float kSoften2 = 2.5 * 2.5, kReflectance = 0.5;
    return light.colorOuter.rgb * (light.positionIntensity.w * kReflectance * window * window / (dist * dist + kSoften2)) *
           roomLightInside(light, worldPos);
}

// One light's direct contribution (the GGX specular + Lambert diffuse, the sun's shadow on the sun only).
vec3 shadeLight(Light light, vec3 worldPos, vec3 N, vec3 V, vec3 albedo, float metallic, vec3 f0, float a2,
                float NdotV, vec3 sunShadow) {
    int type = int(light.typeRange.x);
    vec3 L;
    float attenuation;
    if (type == 1) {                       // directional
        L = normalize(light.directionInner.xyz);
        attenuation = light.positionIntensity.w;
    } else {
        L = light.positionIntensity.xyz - worldPos;
        float dist = length(L);
        L = normalize(L);
        attenuation = light.positionIntensity.w * distanceAttenuation(dist, light.typeRange.y);
        if (type == 2) {                   // spot
            float theta = dot(-L, normalize(light.directionInner.xyz));
            attenuation *= smoothstep(light.colorOuter.w, light.directionInner.w, theta);
        } else if (type == 3) {            // a room's light (ADR-0365): only inside its room's box
            attenuation *= roomLightInside(light, worldPos);
        }
    }
    float NdotL = max(dot(N, L), 0.0);
    if (NdotL <= 0.0 || attenuation <= 0.0) return vec3(0.0);

    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);
    float D = distributionGGX(NdotH, a2);
    float Vis = visibilitySmithGGX(NdotV, NdotL, a2);
    vec3 F = fresnelSchlickVec(VdotH, f0);
#ifdef SHADE_DIFFUSE_ONLY
    // a probe's relight (ADR-0374): a highlight is where the eye stands, and the probe's eye is not the viewer's --
    // baked in, the sun's glint off a far window smeared over every rough surface that reads the probe
    vec3 specular = vec3(0.0);
#else
    vec3 specular = D * Vis * F;
#endif
    vec3 diffuse = (1.0 - F) * (1.0 - metallic) * albedo / PI;
    vec3 sh = (type == 1) ? sunShadow : vec3(1.0);   // only the sun casts shadows
    return (diffuse + specular) * light.colorOuter.rgb * (attenuation * NdotL) * sh;
}

// THE FRAME'S LIGHTS ON A SURFACE: the one light loop of the scene pass, a probe's relight (ADR-0373) and a reflection
// ray's hit (ADR-0376). Clustered (ADR-0361), the sun and the moon come from the 32 and every other light from the
// point's froxel; a point the camera does not see (a probe texel behind it, a ray's hit off screen) has no froxel and
// takes the 32 instead -- the sun and the first 31 published. (Until 2026-11-09 the probes and the rays always took the
// 32, so a car's headlamps and the player's torch, published after the street's lamps, never showed in a reflection.)
// `indoor`: a room's surface, which takes its lights' bounce too (roomBounce).
vec3 evaluateLighting(vec3 worldPos, vec3 N, vec3 V, vec3 albedo,
                      float metallic, float roughness, vec3 f0, vec3 sunShadow, bool indoor) {
    vec3 directLight = vec3(0.0);
    vec3 bounce = vec3(0.0);
    float a = max(roughness * roughness, 0.002);
    float a2 = a * a;
    float NdotV = max(dot(N, V), 1e-4);

    int count = min(g.counts.x, 32);
    uint off, n;
    if (clusterCellAt(worldPos, off, n)) {
        for (int i = 0; i < count; ++i)
            if (int(g.lights[i].typeRange.x) == 1)
                directLight += shadeLight(g.lights[i], worldPos, N, V, albedo, metallic, f0, a2, NdotV, sunShadow);
        for (uint k = 0u; k < n; ++k) {
            Light light = clusterLights[clusterIndices[off + k]];
            directLight += shadeLight(light, worldPos, N, V, albedo, metallic, f0, a2, NdotV, sunShadow);
            if (indoor) bounce += roomBounce(light, worldPos);
        }
        return directLight + bounce * albedo * (1.0 - metallic) / PI;
    }
    for (int i = 0; i < count; ++i) {
        directLight += shadeLight(g.lights[i], worldPos, N, V, albedo, metallic, f0, a2, NdotV, sunShadow);
        if (indoor) bounce += roomBounce(g.lights[i], worldPos);
    }
    return directLight + bounce * albedo * (1.0 - metallic) / PI;
}
