// Post-processing helpers shared across the post stack: view-space position
// reconstruction, reverse-Z linearization, and the bilateral depth edge-stop.
// Used by SSR, GTAO *and* the composite pass, so it precedes all three.

// Reconstruct view-space position from NDC depth and screen UV. invProjection is
// the inverse of the reverse-Z projection (ADR-0034 Phase 0), so this unprojects
// reverse-Z depth correctly without any per-call sign handling.
float3 ssrViewPos(float depth, float2 uv, float4x4 invProjection) {
    float4 clip = float4(uv.x * 2.0 - 1.0, -(uv.y * 2.0 - 1.0), depth, 1.0);
    float4 vp = invProjection * clip;
    return vp.xyz / vp.w;
}

// Reverse-Z NDC depth -> positive linear eye distance (near at z=1, far at z=0).
// Inverse of z = near*(far - d) / (d*(far - near)). Monotonic in distance, so
// depth comparisons done in this space are convention-independent.
float linearizeReverseZ(float z, float near, float far) {
    return near * far / (near + z * (far - near));
}

// Bilateral depth edge-stop for the SSR/AO blurs. Compares depths in linear eye
// space (so it's reverse-Z-correct) and as a *relative* difference, so a single
// sigma behaves the same near and far — the old kernels compared raw NDC depth
// with a fixed sigma tuned for the forward-Z distribution, which under reverse-Z
// rejected most same-surface neighbors up close and almost none far away.
// Returns ~1 on the same surface and falls to 0 across a depth discontinuity.
float bilateralDepthWeight(float zCenter, float zSample, float near, float far,
                           float sigmaRel) {
    float c = linearizeReverseZ(zCenter, near, far);
    float s = linearizeReverseZ(zSample, near, far);
    float rel = (s - c) / max(c, near);
    return exp(-rel * rel / (2.0 * sigmaRel * sigmaRel));
}
