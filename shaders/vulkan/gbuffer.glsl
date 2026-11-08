// THE SCENE PASS'S SECOND TARGET (RGBA8; ADR-0376): rg the world normal, octahedral; b the roughness (what SSR and the
// reflection rays gate on); a the specular weight -- what the surface multiplies its environment reflection by (the
// split-sum F0 * scale + bias, its luminance, times the ambient shadow), so the rays replace exactly the reflection
// the probes gave. One encoding, every writer and reader (mesh.frag, mesh_wire.frag, ssao, ssr, the composite, the
// rays).

vec2 gbufOct(vec3 n) {
    n /= max(abs(n.x) + abs(n.y) + abs(n.z), 1e-6);
    vec2 e = n.xy;
    if (n.z < 0.0) e = (1.0 - abs(n.yx)) * vec2(n.x >= 0.0 ? 1.0 : -1.0, n.y >= 0.0 ? 1.0 : -1.0);
    return e * 0.5 + 0.5;
}
vec4 gbufEncode(vec3 n, float roughness, float specWeight) {
    return vec4(gbufOct(normalize(n)), roughness, clamp(specWeight, 0.0, 1.0));
}
vec3 gbufNormal(vec4 t) {
    vec2 e = t.rg * 2.0 - 1.0;
    vec3 n = vec3(e, 1.0 - abs(e.x) - abs(e.y));
    if (n.z < 0.0) n.xy = (1.0 - abs(n.yx)) * vec2(n.x >= 0.0 ? 1.0 : -1.0, n.y >= 0.0 ? 1.0 : -1.0);
    return normalize(n);
}
float gbufRoughness(vec4 t) { return t.b; }
float gbufSpecular(vec4 t) { return t.a; }
