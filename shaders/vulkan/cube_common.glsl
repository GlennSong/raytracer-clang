// THE CUBE PASSES' SHARED PIECES (sky_cube, sky_prefilter, probe_prefilter, probe_relight): a texel's direction, and the
// GGX lobe's importance samples.
#ifndef RT_CUBE_COMMON_GLSL
#define RT_CUBE_COMMON_GLSL

// a cube texel's direction (Vulkan's face order +X -X +Y -Y +Z -Z; renderer/cube_faces.h on the CPU)
vec3 cubeDir(int face, vec2 uv) {
    vec2 p = uv * 2.0 - 1.0;
    if (face == 0) return normalize(vec3(1.0, -p.y, -p.x));
    if (face == 1) return normalize(vec3(-1.0, -p.y, p.x));
    if (face == 2) return normalize(vec3(p.x, 1.0, p.y));
    if (face == 3) return normalize(vec3(p.x, -1.0, -p.y));
    if (face == 4) return normalize(vec3(p.x, -p.y, 1.0));
    return normalize(vec3(-p.x, -p.y, -1.0));
}

// the i-th of n low-discrepancy points on the unit square
vec2 hammersley(uint i, uint n) {
    uint b = i;
    b = (b << 16u) | (b >> 16u);
    b = ((b & 0x55555555u) << 1u) | ((b & 0xAAAAAAAAu) >> 1u);
    b = ((b & 0x33333333u) << 2u) | ((b & 0xCCCCCCCCu) >> 2u);
    b = ((b & 0x0F0F0F0Fu) << 4u) | ((b & 0xF0F0F0F0u) >> 4u);
    b = ((b & 0x00FF00FFu) << 8u) | ((b & 0xFF00FF00u) >> 8u);
    return vec2(float(i) / float(n), float(b) * 2.3283064365386963e-10);
}

// a GGX halfway vector about N (tx, ty its tangent frame; a2 = roughness^4) for the point xi (Karis 2013)
vec3 ggxHalfway(vec3 N, vec3 tx, vec3 ty, float a2, vec2 xi) {
    const float phi = 2.0 * 3.14159265 * xi.x;
    const float cosT = min(sqrt((1.0 - xi.y) / (1.0 + (a2 - 1.0) * xi.y)), 1.0);
    const float sinT = sqrt(max(1.0 - cosT * cosT, 0.0));   // (rounding put cosT past 1 at low roughness: NaN)
    return normalize(tx * (sinT * cos(phi)) + ty * (sinT * sin(phi)) + N * cosT);
}

#endif
