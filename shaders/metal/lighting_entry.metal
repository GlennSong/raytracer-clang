// --- Vertex/fragment entry points ---

vertex VertexOut vertexMain(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& camera [[buffer(1)]],
    constant ModelUniforms& model [[buffer(2)]],
    uint vid [[vertex_id]]
) {
    VertexOut out;
    float4 worldPos = model.model * float4(vertices[vid].position, 1.0);
    out.position = camera.viewProjection * worldPos;
    out.worldPosition = worldPos.xyz;
    out.worldNormal = normalize((model.normalMatrix * float4(vertices[vid].normal, 0.0)).xyz);
    out.worldTangent = normalize((model.model * float4(float3(vertices[vid].tangent), 0.0)).xyz);
    out.texcoord = vertices[vid].texcoord;
    out.vertexColor = vertices[vid].color;
    return out;
}

// CDLOD terrain (ADR-0036). The node mesh is world-space (identity model); its
// `tangent` holds the morph-target position — the height this vertex collapses to
// on the next-coarser grid. Morph by camera distance so a node has already matched
// the coarser silhouette by the time its parent takes over (no pop, no crack). The
// output is the same VertexOut as vertexMain, so it feeds the shared lit fragment.
vertex VertexOut terrainVertexMain(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& camera [[buffer(1)]],
    constant TerrainUniforms& terrain [[buffer(2)]],
    uint vid [[vertex_id]]
) {
    float3 pos = float3(vertices[vid].position);
    float3 morphTarget = float3(vertices[vid].tangent);
    float dist = distance(camera.cameraPosition, pos);
    float k = saturate((dist - terrain.morphStart) /
                       max(terrain.morphEnd - terrain.morphStart, 1e-3));
    float3 worldPos = mix(pos, morphTarget, k);

    VertexOut out;
    out.position = camera.viewProjection * float4(worldPos, 1.0);
    out.worldPosition = worldPos;
    out.worldNormal = normalize(float3(vertices[vid].normal));
    out.worldTangent = float3(1.0, 0.0, 0.0);   // unused: terrain has no normal map
    out.texcoord = vertices[vid].texcoord;
    out.vertexColor = vertices[vid].color;
    return out;
}

vertex FragmentData vertexMainInstanced(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& camera [[buffer(1)]],
    const device GPUInstanceData* instances [[buffer(2)]],
    uint vid [[vertex_id]],
    uint iid [[instance_id]]
) {
    GPUInstanceData inst = instances[iid];
    FragmentData out;
    float4 worldPos = inst.model * float4(vertices[vid].position, 1.0);

    // Wind sway (FLAG_WIND): displace in the wind direction, weighted by height
    // above the instance origin (base planted, tips move) and phase-offset by
    // world position so a field doesn't sway in unison.
    if (int(inst.flags) & 4) {
        float baseY = inst.model[3].y;
        float weight = saturate((worldPos.y - baseY) / max(camera.windHeight, 0.001));
        weight *= weight;
        float phase = camera.windTime * camera.windFrequency +
                      dot(worldPos.xz, float2(0.15, 0.1));
        float gust = sin(phase) + 0.3 * sin(phase * 2.3 + 1.7);
        worldPos.xz += camera.windDir.xz * (camera.windAmplitude * weight * gust);
    }

    out.position = camera.viewProjection * worldPos;
    out.worldPosition = worldPos.xyz;
    out.worldNormal = normalize((inst.normalMatrix * float4(vertices[vid].normal, 0.0)).xyz);
    out.worldTangent = normalize((inst.model * float4(float3(vertices[vid].tangent), 0.0)).xyz);
    out.texcoord = vertices[vid].texcoord;
    out.vertexColor = vertices[vid].color;
    out.albedo = inst.albedo.xyz;
    out.metallic = inst.metallic;
    out.roughness = inst.roughness;
    out.opacity = inst.opacity;
    out.flags = inst.flags;
    out.emission = inst.emission.xyz;
    out.textureFlags = inst.textureFlags;
    return out;
}

fragment GBufferOut fragmentMain(
    VertexOut in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    constant MaterialUniforms& material [[buffer(3)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]],
    device const GPUReflectionProbe* probes [[buffer(7)]],
    constant EnvUniforms& env [[buffer(8)]],
    depth2d_array<float> shadowMap [[texture(0)]],
    texturecube_array<float> cubemapArray [[texture(1)]],
    texture2d<float> brdfLUT [[texture(2)]],
    texture2d<float> albedoMap [[texture(3)]],
    texture2d<float> metalRoughMap [[texture(4)]],
    texture2d<float> normalMap [[texture(5)]],
    texture2d<float> aoMap [[texture(6)]],
    texture2d<float> emissiveMap [[texture(7)]],
    texturecube<float> prefilteredEnv [[texture(8)]],
    texturecube<float> irradianceEnv [[texture(9)]],
    sampler shadowSampler [[sampler(0)]],
    sampler envSampler [[sampler(1)]],
    sampler texSampler [[sampler(2)]]
) {
    // Wireframe pass: lines bypass shading and draw the flat override colour.
    if (camera.wireColor.w > 0.5) {
        GBufferOut wf; wf.color = float4(camera.wireColor.rgb, 1.0);
        wf.viewNormal = float4(0.0, 0.0, 1.0, 0.0); return wf;
    }
    SurfaceGeometry geom = {in.worldPosition, in.worldNormal,
                            in.worldTangent, in.texcoord};
    // Per-vertex tint (default white) modulates the material albedo — this is
    // the procedural coloration channel (e.g. terrain grass/rock by height/slope).
    SurfaceMaterial mat = {material.albedo * in.vertexColor, material.metallic,
                           material.roughness, material.opacity,
                           material.flags, material.emission,
                           material.textureFlags};
    return shadeSurface(geom, mat, camera, lightData, shadowData,
                        probeParams, probes, env,
                        shadowMap, cubemapArray, brdfLUT,
                        albedoMap, metalRoughMap, normalMap, aoMap,
                        emissiveMap, prefilteredEnv, irradianceEnv,
                        shadowSampler, envSampler, texSampler);
}

fragment GBufferOut fragmentMainInstanced(
    FragmentData in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]],
    device const GPUReflectionProbe* probes [[buffer(7)]],
    constant EnvUniforms& env [[buffer(8)]],
    depth2d_array<float> shadowMap [[texture(0)]],
    texturecube_array<float> cubemapArray [[texture(1)]],
    texture2d<float> brdfLUT [[texture(2)]],
    texture2d<float> albedoMap [[texture(3)]],
    texture2d<float> metalRoughMap [[texture(4)]],
    texture2d<float> normalMap [[texture(5)]],
    texture2d<float> aoMap [[texture(6)]],
    texture2d<float> emissiveMap [[texture(7)]],
    texturecube<float> prefilteredEnv [[texture(8)]],
    texturecube<float> irradianceEnv [[texture(9)]],
    sampler shadowSampler [[sampler(0)]],
    sampler envSampler [[sampler(1)]],
    sampler texSampler [[sampler(2)]]
) {
    if (camera.wireColor.w > 0.5) {                 // wireframe lines: flat colour
        GBufferOut wf; wf.color = float4(camera.wireColor.rgb, 1.0);
        wf.viewNormal = float4(0.0, 0.0, 1.0, 0.0); return wf;
    }
    SurfaceGeometry geom = {in.worldPosition, in.worldNormal,
                            in.worldTangent, in.texcoord};
    SurfaceMaterial mat = {in.albedo * in.vertexColor, in.metallic, in.roughness,
                           in.opacity, in.flags, in.emission, in.textureFlags};
    return shadeSurface(geom, mat, camera, lightData, shadowData,
                        probeParams, probes, env,
                        shadowMap, cubemapArray, brdfLUT,
                        albedoMap, metalRoughMap, normalMap, aoMap,
                        emissiveMap, prefilteredEnv, irradianceEnv,
                        shadowSampler, envSampler, texSampler);
}

// --- Foliage depth prepass (perf) ---
// Alpha-cut foliage is shaded by fragmentMainInstanced, which calls
// discard_fragment() — that forces LATE depth testing, so every overlapping leaf
// card close-up runs the full lit shader (severe overdraw). Splitting foliage
// into a depth prepass + an early-depth-tested lit pass shades each pixel once.
//
// Prepass: depth-only (returns void), does only the leaf alpha cut so silhouette
// holes don't write depth. Reuses vertexMainInstanced, so the depth it writes
// matches the lit pass bit-for-bit (same wind sway) and the Equal test is exact.
fragment void fragmentFoliageDepthInstanced(
    FragmentData in [[stage_in]],
    texture2d<float> albedoMap [[texture(3)]],
    sampler texSampler [[sampler(2)]]
) {
    if ((uint(in.textureFlags) & 1u) && (int(in.flags) & 2)) {
        if (albedoMap.sample(texSampler, in.texcoord).a < 0.5) discard_fragment();
    }
}

// Lit pass: identical to fragmentMainInstanced, but [[early_fragment_tests]] runs
// the Equal depth test (against the prepass) BEFORE this expensive shader, so
// occluded leaves are rejected unshaded. Depth write is off in this pass, so the
// early test is safe despite the alpha-cut discard inside shadeSurface (the
// front-most leaf that won the prepass always passes its own alpha test).
[[early_fragment_tests]]
fragment GBufferOut fragmentMainInstancedFoliage(
    FragmentData in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]],
    device const GPUReflectionProbe* probes [[buffer(7)]],
    constant EnvUniforms& env [[buffer(8)]],
    depth2d_array<float> shadowMap [[texture(0)]],
    texturecube_array<float> cubemapArray [[texture(1)]],
    texture2d<float> brdfLUT [[texture(2)]],
    texture2d<float> albedoMap [[texture(3)]],
    texture2d<float> metalRoughMap [[texture(4)]],
    texture2d<float> normalMap [[texture(5)]],
    texture2d<float> aoMap [[texture(6)]],
    texture2d<float> emissiveMap [[texture(7)]],
    texturecube<float> prefilteredEnv [[texture(8)]],
    texturecube<float> irradianceEnv [[texture(9)]],
    sampler shadowSampler [[sampler(0)]],
    sampler envSampler [[sampler(1)]],
    sampler texSampler [[sampler(2)]]
) {
    if (camera.wireColor.w > 0.5) {                 // wireframe lines: flat colour
        GBufferOut wf; wf.color = float4(camera.wireColor.rgb, 1.0);
        wf.viewNormal = float4(0.0, 0.0, 1.0, 0.0); return wf;
    }
    SurfaceGeometry geom = {in.worldPosition, in.worldNormal,
                            in.worldTangent, in.texcoord};
    SurfaceMaterial mat = {in.albedo * in.vertexColor, in.metallic, in.roughness,
                           in.opacity, in.flags, in.emission, in.textureFlags};
    return shadeSurface(geom, mat, camera, lightData, shadowData,
                        probeParams, probes, env,
                        shadowMap, cubemapArray, brdfLUT,
                        albedoMap, metalRoughMap, normalMap, aoMap,
                        emissiveMap, prefilteredEnv, irradianceEnv,
                        shadowSampler, envSampler, texSampler);
}
