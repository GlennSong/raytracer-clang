#version 450
// THE CLOUDS' HISTORY (#187; the shimmer as the camera moves, #59): the half-res march (clouds.frag) blended with last
// frame's resolved clouds, reprojected to where that cloud was on screen then, so the march's per-frame start offsets
// average away instead of a pattern fixed to the screen crawling across the clouds. The history is clamped to this
// frame's 3x3 neighbourhood (no ghost where a cloud edge or a hill's silhouette moved), and the blend is
// 1 - historyWeight of the new frame. Same set layout as the march: the uniforms, then three images.
layout(set = 0, binding = 0) uniform CloudUniforms {
    mat4 invViewProjection;
    vec4 cameraPosition;
    vec4 sunDirection;
    vec4 sunColor;
    vec4 skyAmbient;
    vec4 planetCenter;
    vec4 layer;           // x bottom, y top
    vec4 params;
    vec4 march;           // w farDistance
    vec4 detail;
    mat4 prevViewProjection;
    vec4 frame;           // x dither offset, y history weight (0: reset)
} u;
layout(set = 0, binding = 1) uniform sampler2D current;   // this frame's march (rgb in-scatter, a transmittance)
layout(set = 0, binding = 2) uniform sampler2D history;   // last frame's resolve
layout(set = 0, binding = 3) uniform sampler2D sceneDepth;
layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

void main() {
    vec4 cur = texture(current, inUV);
    if (u.frame.y <= 0.0) { outColor = cur; return; }
    // the neighbourhood's range, this frame
    vec2 texel = 1.0 / vec2(textureSize(current, 0));
    vec4 lo = cur, hi = cur;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x) {
            if (x == 0 && y == 0) continue;
            vec4 s = texture(current, inUV + vec2(x, y) * texel);
            lo = min(lo, s);
            hi = max(hi, s);
        }
    // WHERE THIS PIXEL'S CLOUD WAS: the point along its ray at the cloud deck's middle (or the scene, nearer), seen
    // through last frame's camera (the flipped convention of clouds.frag: ndc = uv * 2 - 1)
    vec2 ndc = inUV * 2.0 - 1.0;
    vec4 wf = u.invViewProjection * vec4(ndc, 1.0, 1.0);
    vec3 cam = u.cameraPosition.xyz;
    vec3 dir = normalize(wf.xyz / wf.w - cam);
    float mid = 0.5 * (u.layer.x + u.layer.y);
    float t = u.march.w;
    if (abs(dir.y) > 1e-4) {
        float tm = (mid - cam.y) / dir.y;
        if (tm > 0.0) t = min(t, tm);
    }
    float depth = texture(sceneDepth, inUV).r;
    if (depth > 0.0) {
        vec4 w = u.invViewProjection * vec4(ndc, depth, 1.0);
        t = min(t, length(w.xyz / w.w - cam));
    }
    vec4 prevClip = u.prevViewProjection * vec4(cam + dir * t, 1.0);
    vec2 prevUV = prevClip.xy / prevClip.w * 0.5 + 0.5;
    if (prevClip.w <= 0.0 || any(lessThan(prevUV, vec2(0.0))) || any(greaterThan(prevUV, vec2(1.0)))) {
        outColor = cur;   // off last frame's screen: nothing to blend
        return;
    }
    vec4 hist = clamp(texture(history, prevUV), lo, hi);
    outColor = mix(cur, hist, u.frame.y);
}
