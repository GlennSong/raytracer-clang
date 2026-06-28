# `src/renderer/` — Agent Guide

The rendering + windowing layer: the two platform seams from ADR-0001, plus the
GPU backends that implement them. Engine/game code never reaches past this
directory for a graphics or OS symbol.

> Read this before touching a backend. For backend internals see
> `metal/AGENTS.md` and `vulkan/AGENTS.md`. For the *why* behind decisions, see
> `docs/decisions.md` (ADR-0001 platform seams, ADR-0011 ImGui, ADR-0013
> gamepad, ADR-0016 environment, ADR-0017 lighting, ADR-0036 CDLOD terrain,
> ADR-0057 Vulkan).

> **Parity is a hard requirement (Metal ↔ Vulkan).** When you add or change a
> renderer feature, update `docs/renderer-parity.md` in the same change: set the
> touched backend's status, mark the other backend's gap if it now lags, and
> leave new code `🟡` (implemented, unverified) until it's confirmed on real
> hardware — then flip it to `✅` and add a dated line to that file's
> Verification Log. The matrix is the cross-platform backlog; keep it honest.

## The two seams

| Seam | Interface | Crosses as | Implementations |
| --- | --- | --- | --- |
| Rendering (RHI) | `Renderer` (`renderer.h`) | backend-neutral structs only | `metal/`, `vulkan/` (planned), `null_renderer.cpp` |
| Windowing + input | `Window` (`window.h`) | our `Event`/`KeyCode`, opaque `void*` native handle | `window.cpp` (GLFW), `hosted_window.*` (editor embed) |

**The rule (load-bearing, from `AGENTS.md`):** no `GLFW_*`, `NSWindow`/`HWND`,
or Metal/Vulkan type appears outside the file implementing that seam. A backend
is added by writing an implementation, not by threading `#ifdef`s through call
sites. CMake selects the backend at build time (`CMakeLists.txt`, the
`if(APPLE)…else()` block in the viewer section).

## `renderer.h` — the contract every backend implements

`Renderer` is a pure-virtual interface; `Renderer::create()` returns the
platform's concrete backend. The surface is small and concrete — a resource
uploader + draw dispatcher, **not** a command-buffer abstraction:

- **Lifecycle:** `initialize(void* nativeWindowHandle, w, h)`, `shutdown`,
  `resize`.
- **Resources:** `uploadMesh`/`removeMesh`/`getMeshBounds`,
  `uploadTexture`/`uploadTextureHDR`/`removeTexture`. Handles are
  `Handle<Tag>` (`MeshHandle`, `TextureHandle`) backed by a `SlotMap` in the
  backend (stale-handle safe).
- **Per frame:** `beginFrame` → `setCamera` → `setLights` →
  `setReflectionProbes` → `drawMesh`/`drawMeshInstanced`/`drawTerrain` →
  `endFrame`. The backend *queues* draws during the frame and executes the whole
  pass graph in `endFrame`.
- **Environment:** `setEnvironmentMap(equirect)` (bind HDR → bakes cubemap+IBL;
  invalid handle = procedural sky).
- **Debug UI:** `initDebugUi`/`shutdownDebugUi` (no-ops unless `RT_ENABLE_IMGUI`).
- **Live tuning state** (public fields, not virtuals): `ssaoEnabled`,
  `ssrEnabled`, `bloomEnabled`/`bloomParams`, `tonemapOperator`/`gradeParams`,
  `ssrParams`, `ssaoParams`, `shadowParams`, `debugView`, `wireframe`,
  `lensEffectsEnabled`/`dofEnabled`, `environmentMapEnabled`,
  `depthPrepassEnabled`, etc. A backend reads these each frame; the UI/engine
  writes them. **Adding a knob = add a field here, read it in every backend.**

Engine-side math is already done before it reaches a backend: `CameraState`
carries position/target/fov; the backend builds view/projection (and applies its
own clip-space/depth convention there). Lights/materials/shadow config are the
structs in `renderer.h` (`SceneLighting`, `RenderMaterial`, `ShadowConfig`, …).

## Data that crosses to the GPU

GPU-visible struct layouts are **shared between C++ and shader code** via a
single header per backend (`shaders/<backend>/shader_types.h`), so the CPU upload
and the shader agree on layout. When you add a uniform/field, change it there and
it updates both sides. `RenderMaterial::Surface` (the analytic surface library:
brick/concrete/asphalt/…) ids must stay in lockstep across `renderer.h`,
`material.h`, and the shader `SURFACE_*` constants.

## Conventions that bite

- **Winding:** front faces are **clockwise** (engine-wide rule). Backends
  back-cull by it; the offline tracer is two-sided and silently tolerates bad
  winding, so a winding bug only shows in the viewer. Builders must emit through
  `MeshBuilder` helpers, never hand-rolled index order.
- **Native handle is opaque.** `Window::nativeWindowHandle()` returns a `void*`
  (`NSWindow*` on macOS, `HWND`/X11/Wayland elsewhere). A backend unwraps it
  internally; it must not know it came from GLFW. The window is created with
  `GLFW_NO_API` (`window.cpp`), so it carries no GL context and is GPU-agnostic.
- **NullRenderer is the floor.** `null_renderer.cpp` is a valid `Renderer` that
  draws nothing — it keeps the viewer linking on platforms with no GPU backend
  (Linux/CI today). Don't let optional virtuals become required: new `Renderer`
  methods should have a sane default so `NullRenderer` and partial backends stay
  valid.

## File index

| File | Role |
| --- | --- |
| `renderer.h` | RHI interface + all backend-neutral render structs |
| `null_renderer.cpp` | No-op backend (link floor for GPU-less platforms) |
| `window.h` / `window.cpp` | GLFW windowing + input seam (pimpl, no GLFW in header) |
| `hosted_window.h` / `.cpp` | Window embedded in the Qt editor viewport |
| `event.h` | Backend-neutral `Event` / `KeyCode` / `MouseButton` |
| `gamepad.h` | Backend-neutral gamepad state |
| `gamepad_gc.h` / `.mm` | Apple GCController backend (macOS only); GLFW joystick elsewhere |
| `settings.h` / `.cpp` | Persisted viewer settings |
| `metal/` | Metal backend (macOS) — see `metal/AGENTS.md` |
| `vulkan/` | Vulkan backend (Linux/Windows, in progress) — see `vulkan/AGENTS.md` |

## When you add a backend

1. Implement `Renderer` in `<backend>/<backend>_renderer.{h,cpp}`; return it from
   `Renderer::create()` under the right build guard.
2. If you need a native surface, add a pimpl'd method to `Window` that creates it
   without leaking the windowing type (e.g. Vulkan's `createVulkanSurface`).
3. Port shaders into `shaders/<backend>/`, reusing the shared `shader_types.h`.
4. Wire CMake (find the SDK, add sources, link libs) in the viewer block.
5. Match the pass graph and conventions documented in `metal/AGENTS.md` — the
   Metal backend is the reference for feature parity.
