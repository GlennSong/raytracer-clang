# Single-source shaders — plan (proposal)

**Status: proposed, not started.** Successor to
[shader-transpile-study.md](shader-transpile-study.md) (which proved the
tooling) — this is the adoption plan, written after two real divergence bugs
motivated it. Companion to ADR-0057/0058; graduating this to an ADR in
`decisions.md` is Phase 0's exit criterion.

## Why (what actually went wrong)

The three hand-written shader trees (Metal MSL, Vulkan GLSL, WebGPU WGSL) are
kept in lockstep by discipline alone. In practice, the web backend shipped two
real divergences within one review cycle:

1. **Lossy port:** the WGSL SSR dropped `ssr.frag`'s binary-refinement loop —
   striped reflections. The reference even had a comment describing the exact
   artifact; the port simply lost the code below it.
2. **Fork drift at the algorithm level:** `ssr.frag` itself was a "fixed-step
   approximation" of Metal's screen-space DDA — never visually verified,
   because the Vulkan backend has no working device target yet. The web build
   inherited an implementation no one had ever seen render.

Both bug classes are *structurally impossible* with one source: there is
nothing to port and nothing to approximate.

## Pros / cons (honest)

**Pros**
- Eliminates the port-drift bug class entirely (see above — not hypothetical).
- One place to fix/tune; a shader improvement lands on every platform at once.
- The hand-written trees total ~2,900 lines across three dialects for one set
  of algorithms; a single tree roughly thirds the shader surface area.
- Unit-testable pipeline: generated output can be snapshot-diffed in CI, so a
  toolchain bump that changes codegen is visible, not silent.
- A fourth backend (or a compute pass) starts from the shared source for free.

**Cons**
- **Three build-time tool dependencies** (glslang, SPIRV-Cross, Tint) in a
  project whose identity is "no new deps without an ADR". Mitigation: commit
  the generated outputs, so only shader *editing* needs the tools (like the
  committed `.spv` files today).
- **Debugging through generated code.** Browser/Xcode shader errors point at
  transpiled source, not what you wrote. Mitigation: keep generated files
  human-readable and committed (diffable), never minified.
- **The superset problem.** The trees genuinely differ today: Metal has probes
  / lens / DoF and uses *compute* kernels for SSR/AO; Vulkan/WebGPU use
  fragment passes. One source must either carry the superset behind flags or
  exclude some stages. This is the real cost — not the transpilation.
- **Loss of hand-tuning latitude** on Metal (function constants, `half`
  precision) unless expressed portably.
- **Toolchain fragility:** Tint/SPIRV-Cross version bumps can change codegen;
  WGSL's uniformity analysis can reject patterns GLSL allows. Mitigation:
  pinned versions + committed outputs + the snapshot diff.

**Verdict:** worth doing *incrementally*, starting where the cost is lowest and
the payoff proven — and explicitly **not** starting with Metal.

## Does the backend architecture have to converge?

Mostly no — one layer of it yes:

- **Unchanged:** the whole-renderer `Renderer` seam, one self-contained backend
  per platform, per-backend pass orchestration (`endFrame` stages), resource
  lifetime code. Single-sourcing shaders does not require an RHI.
- **Must converge — the shader-facing contract:**
  - **Binding model:** one canonical set/group numbering and uniform-block
    layout, designed once, that every backend's CPU code conforms to. (Vulkan
    `set=N binding=M` maps 1:1 to WGSL `group/binding`; Metal argument indices
    are assigned by SPIRV-Cross and the Metal backend adapts.) Today each
    backend invented its own layout; that is what "rewriting the CPU-side
    binding" means, and it is the bulk of the work.
  - **Push-constant strategy:** WebGPU has none, so the canonical source uses a
    UBO for per-draw data (Vulkan keeps push constants only if expressed as an
    alternate binding of the same block; simplest is to standardize on the UBO).
  - **Convention knobs handled in ONE place:** reverse-Z (Metal/Vulkan) vs
    standard-Z (WebGPU) and Vulkan's clip-space Y-flip become specialization
    constants / generated prologue defines — never hand-edited per tree. The
    shared `cascade_fit.h` already models this pattern on the CPU side.
  - **Stage parity per pass:** a pass must be the same *kind* everywhere it is
    shared (fragment vs compute). Practically: Vulkan/WebGPU already match;
    Metal's compute post passes stay Metal-local until/unless converted.

## Toolchain (fixed by the study)

`GLSL (canonical, shaders/src/) --glslang--> SPIR-V --SPIRV-Cross--> MSL`
and `--Tint--> WGSL`. naga was tested and cannot handle this codebase's
shaders. Generated WGSL/MSL/SPIR-V are **committed**; a
`tools/build-shaders.sh` regenerates them, and CI (when it exists) diffs
regenerated output against the committed files.

## Phases

### Phase 0 — pilot on one small shader (~half a day)
Author `bloom` (or `blit`) once in GLSL; generate the WGSL via glslang+Tint;
diff against the hand-written WGSL; ship the generated one in the web build and
verify structurally + on-device. **Exit criteria:** generated shader renders
identically; the workflow (edit GLSL → regenerate → commit) feels sane; write
the ADR. If Tint output is unacceptable, stop here at zero architectural cost.

### Phase 1 — canonical binding layout + tooling (~1–2 days)
Design the canonical set/group layout (document in `docs/rendering.md`);
`tools/build-shaders.sh` + pinned tool versions; move the Vulkan GLSL to
`shaders/src/` as the canonical tree (Vulkan is near-zero cost — GLSL is
already its source).

### Phase 2 — unify the post stack for Vulkan + WebGPU (~2–3 days)
composite, bloom, ssao, ssr, brdf_lut, blit: small uniforms, self-contained,
and exactly where the two divergence bugs happened. WebGPU's CPU bind-group
code conforms to the canonical layout. Metal untouched.

### Phase 3 — the mesh/terrain/shadow family (~3–5 days)
The big one (multi-light + surface library + shadows + IBL + wind + morph).
Superset flags for per-backend features; WebGPU retires its hand WGSL tree.

### Phase 4 — Metal, only if wanted (defer)
Metal is mature, visually proven, hand-tuned, and compute-based in post —
the study's "most disruptive, least to gain" case. Recommendation: leave it
hand-written; revisit only when a shared-shader change keeps getting
hand-ported to MSL and that tax annoys.

**End state (recommended):** one canonical GLSL tree feeding Vulkan (native
SPIR-V) and WebGPU (generated WGSL); Metal remains a curated port with the
lockstep rule scoped to it alone. Full three-way unification stays available
but is a separate, later decision.

## Risks / revisit triggers
- Tint uniformity-analysis rejections on real shaders → restructure at the
  GLSL level (usually `textureLod` in divergent flow); acceptable.
- Generated-WGSL perf regression vs hand WGSL → measure in Phase 0; the study
  predicts negligible, verify anyway.
- If Phase 2 takes >2× the estimate, stop and reassess — the hand trees with
  the lockstep rule remain a valid steady state.
