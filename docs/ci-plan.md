# CI Plan — automated build + render verification

## Why (the gap this closes)

There is **no CI today**, and the render backends are a blind spot. The Vulkan
backend (`vulkan_renderer.cpp` + `shaders/vulkan/*`) only compiles when a Vulkan
SDK is present, which no machine in the loop had — so it shipped through Phases
0–3 **never once compiled**. The first real compile (this Windows session) found
real bugs the author could not have seen: a reserved GLSL keyword (`patch`), a
wrong uniform field (`g.lightCount` vs `g.counts.x`), and 43 logging calls
written against an API the project doesn't have. The Metal backend has the **same
class of gap**: its shaders are compiled at runtime (`newLibraryWithSource`), so
`shaders/metal/*.metal` is only validated when the app actually runs on a Mac.

Goal: never again merge render code that hasn't at least *compiled*, and ideally
*rendered*, on both backends — automatically.

## Staged plan (do in order; each stage stands alone)

### Stage 0 — Headless build + unit/physics tests  *(easy, highest ROI)*
Gates the bulk of the codebase (math, ECS, procgen, physics, scripting) — all
already headless and cross-platform (verified building + passing on Windows clang
this session).

- GitHub Actions, matrix: `ubuntu-latest` + `macos-latest`, compiler clang.
- `cmake -S . -B build -G Ninja` (physics on by default) → `cmake --build build`
  → `ctest --test-dir build`.
- No GPU required. This is the regression gate for everything non-render.

### Stage 1 — Render-backend *compile* coverage  *(closes the exact bug class above)*
Make CI compile the render code + validate shaders offline. No GPU needed to
compile.

- **Vulkan (Linux):** install `libvulkan-dev`, `libglfw3-dev`,
  `glslang-tools`/`glslc` (or the LunarG SDK). `find_package(Vulkan)` + GLFW then
  resolve, so `viewer` builds with the Vulkan backend and the CMake custom command
  runs `glslc` over `shaders/vulkan/*` — catching shader + C++ errors at build time.
- **Metal (macOS):** `brew install glfw`; build `viewer` with the Metal backend.
  Because Metal shaders are runtime-compiled today, add an **offline shader check**:
  `xcrun -sdk macosx metal -c shaders/metal/*.metal` (resolve the runtime include
  concatenation — see `metal_renderer.mm`’s `newLibraryWithSource` list) so the
  `.metal` sources are compile-validated in CI like `glslc` does for Vulkan.

### Stage 2 — Headless *render* tests + cross-backend parity  *(the "always good" guarantee)*
Render a fixed scene offscreen on each backend and assert (a) zero validation
errors/warnings and (b) the image matches a golden reference within tolerance.
Catches runtime regressions like the vertex-attribute validation warnings seen
this session.

- **New code needed:** a headless offscreen render entrypoint (render N frames to
  an image with no window/swapchain), e.g. `viewer --render-once <level> --out
  <png> --frames K`, plus a tolerance image comparator test (MAE/SSIM over the PNG;
  the offline tracer already writes PNGs, so reuse that path).
- **Vulkan offscreen (Linux, no GPU):** Mesa **lavapipe** (CPU Vulkan ICD) via
  `VK_ICD_FILENAMES`/`VK_DRIVER_FILES`, validation layers on. SwiftShader is the
  fallback ICD.
- **Metal offscreen (macOS runner):** render-to-texture + readback headless; the
  hosted macOS runners support offscreen Metal.
- **Parity test:** the project targets "Vulkan 1:1 with Metal" — rendering the same
  scene on both and diffing the two images (plus each against its golden) is the
  strongest single regression guard.
- Start with a **smoke test** (init backend, render 1 frame, assert non-empty +
  no validation errors) before full golden-image diffing; keep goldens small and
  deterministic (fixed seed/camera/frame).

## Sequencing / priority
1. **Stage 0 now** — cheap, reliable, covers most code. One afternoon.
2. **Stage 1 next** — directly prevents the never-compiled-render-code failure that
   motivated this. Medium effort (CI deps + a Metal shader-compile step).
3. **Stage 2 last** — most valuable for "always in a good state," but needs the
   headless render entrypoint + lavapipe/Metal-offscreen plumbing. Largest effort.

See also `docs/TECH_DEBT.md` → "Verification gap (the meta-debt)".
