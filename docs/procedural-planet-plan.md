# Procedural Planet Plan

A phased plan for a **procedural planet seen from space** — a body you orbit and
look at, not one you land on. The look is the goal: physically-based, not a bag
of analytic fakes. This is a scoped, high-fidelity slice of the ROADMAP Tier-4
capstone ("Procedural planet — cube-sphere quadtree LOD + spherical terrain +
atmosphere"), deliberately dropping the *landable* apparatus (planetary quadtree
LOD streaming, floating-origin rebasing) that a from-orbit view never exercises.

Scope + technique decision recorded in **ADR-0076**.

---

## What "from space" buys us (the scope line)

Think of the work as three concentric rings. **We build rings 1–2 and stop.**

- **Ring 1 — Core (reads as a planet):** displaced sphere, surface colour, a
  detail normal map, one directional light (the star), a starfield, an orbit
  camera.
- **Ring 2 — Realism (the payoff):** physically-based atmosphere, clouds, ocean,
  polar caps, bloomed star, and **axial rotation** (the planet spins on its
  axis).
- **Ring 3 — Simulation (OUT of scope):** the planet *revolving* around the star,
  n-body gravity, moons, astronomically-accurate scale, seasons-from-geometry.
  This is a physics project, and it fights the renderer's `float` GPU precision.

The trap is thinking Ring 3 is needed to place the star. It is not. The star is a
**direction** (`SceneLighting.sun`, already consumed by the PBR shaders) plus an
optional bright emissive disc far along it. The planet sits at the **origin and
never moves**; the **camera orbits**, and the planet **spins on its axis** (a
cheap `Quat` on its `Transform`). That is the entire "space" we need — no Kepler,
no ephemeris.

### Coordinates & units (read before coding)

- Work in **planet radii**, not kilometres. Planet radius `R = 1000` world units;
  camera orbits at `2R–6R`; atmosphere shell at `1.02R–1.08R`. Do **not** use
  6.371e6 — vertices upload to the GPU as `float` (`GpuVertex`), so real
  astronomical coordinates z-fight and lose precision even though `Real = double`
  on the CPU.
- **Planet at the origin, always** — sidesteps the large-world precision problem
  (ADR-0034's floating-origin note). A second body at astronomical distance is
  exactly what would force camera-relative rendering, which is why Ring 3 is out.
- **Sun is a unit direction**, e.g. `normalize({1, 0.3, -0.5})`. The terminator
  falls where `dot(surfaceNormal, sunDir) = 0`; rotate the *sun direction* (or spin
  the planet) for a moving terminator rather than moving the planet.
- **Vertical exaggeration is mandatory.** Real relief is ~0.1–0.3 % of radius
  (Everest is 0.14 % of Earth's). Displacement amplitude
  `= exaggeration × R × reliefFraction`, `reliefFraction ≈ 0.003`,
  `exaggeration ≈ 3–15`. At true scale you cannot see relief; exaggerating it is
  standard, not a cheat.

---

## The fidelity bar

Every element uses the real technique, not the cheap analytic stand-in.

| Element | Rejected (cheap) | Built (good) | Reference |
| --- | --- | --- | --- |
| Atmosphere | rim-glow fresnel | physically-based **multiple-scattering** LUTs, ground→space | Hillaire 2020; Bruneton–Neyret 2008 |
| Sphere | UV/icosphere | **cube-sphere**, COBE-warped, cubemap-native, LOD-ready | standard planet base |
| Relief | normal map only | displaced geometry **+** detail normals **+** baked horizon self-shadow **+** AO | cone-step / horizon mapping |
| Craters | fbm bumps | **impact-distribution** crater field (Worley + rim/ejecta profile) | Shoemaker crater morphology |
| Ocean | flat specular shell | **PBR water**: GGX sun-glint, Fresnel, depth-tinted transmission | — |
| Clouds | scrolling texture | **volumetric** raymarch, multiple scattering, shadows onto surface | Schneider "Nubis" |
| Gas giant | static baked bands | **curl-noise fluid advection** (evolving storms) + differential rotation + limb darkening | curl-noise flow |
| Star / space | flat dot | **blackbody**-coloured HDR disc + limb darkening + magnitude-distributed starfield | Planck blackbody |

The single biggest jump in perceived quality is the **atmosphere** — it is the
centrepiece, not an afterthought, and everything else composites through it.

---

## Foundation already in the engine

- **Linear HDR pipeline** ✅ — RGBA16Float scene, ACES/AgX tone map, bloom, grade.
- **Physical light units** ✅ — ADR-0017: `sun.color × intensity` is illuminance,
  exactly the `S` a scattering model wants as solar irradiance.
- **Compute-pass precedent** ✅ — IBL prefilter, BRDF LUT, SSAO, bloom already run
  as compute. The atmosphere LUTs are the same pattern (new compute pipelines).
- **Reverse-Z depth** ✅ — needed for the planet + far starfield in one frame.
- **Cube-face convention** ✅ — `src/renderer/cube_faces.h` (`cubeFaceDirection`)
  defines the cubemap texel↔direction mapping; the cube-sphere is built on it so
  per-face planet cubemaps line up with the mesh for free.
- **Radial displacement precedent** ✅ — `generateRock` already displaces a sphere
  radially by noise and rebuilds normals; the rocky planet generalises it.
- **New shader work, mirrored** — atmosphere/ocean/cloud/gas shaders are new
  `.frag`/`.metal` + pipelines, added per the documented "new geometry/post
  pipeline" path and kept at Vulkan↔Metal parity (`docs/renderer-parity.md`).
  This is the real cost of "good": device-verified, not headless.

---

## Phases

Generation (P0–P1 terrain, P5 gas fields, texture bakes) is CPU and
**headless/CI-testable** — the `(params, seed) → content` contract of ADR-0021.
Anything marked *shader/LUT* is GPU/device work, verified on macOS (Metal) or a
Vulkan GPU.

### P0 — Cube-sphere base primitive  *(headless)* — **DONE**
`MeshBuilder::cubeSphere(radius, faceRes, warp)` (`tests/test_cube_sphere.cpp`).

- 6 faces, each a `faceRes×faceRes` grid, projected to the sphere and **welded
  across seams** into one watertight manifold.
- **COBE / tangent-adjustment warp** `f(s) = tan(s·π/4)` equalises the raw
  cube→sphere corner-bunching (~5:1 area ratio → ~1.4:1).
- **Per-face cubemap textures** (albedo/normal/AO/horizon) → no equirect pole
  distortion, no seam. Each face is a quadtree root — the dormant LOD/capstone
  path.
- `faceRes ≈ 256` → ~786k tris: enough to resolve terminator relief on a full
  disc without runtime tessellation. (Screen-space adaptive tessellation is the
  zoom-to-surface upgrade, i.e. the capstone — not now.)
- **Tests:** watertight (every edge shared by exactly two triangles), radius,
  clockwise-front winding, no degenerate/non-finite triangles, warp shrinks the
  max/min triangle-area ratio, determinism.

### P1 — Rocky surface: feature terrain + PBR shading  *(headless gen; device look)* — **DONE (gen); detail bakes pending**
Landed: `generatePlanet(PlanetParams, seed)` in `src/engine/procgen/planet.{h,cpp}`
— a **composed** radial height field (fbm continents + ridged mountains + Worley
impact craters + domain warp) displacing the P0 cube-sphere, with biome vertex
colours (`ColorRamp`), latitude polar caps, and an optional ocean shell; Mars /
Moon / Earth-like presets. Fills the noise gaps: `cellular.{h,cpp}` (Worley) and
`color_ramp.h`. Headless-tested (`tests/test_planet.cpp`): watertight after
displacement, relief bounds, determinism, crater morphology, caps, ocean.
**Still pending (device/bake):** the tangent-space detail normal map + AO +
horizon self-shadow bakes, layered PBR material blend, analytic (gradient) normals.

Original design notes: `generatePlanetSurface(PlanetParams, seed)` — a **composed**
height field, not raw fbm — generalising `generateRock`:

```
h(dir) = continents(warp(dir))       // low-freq fbm, thresholded landmass mask
       + ridged(dir) * tectonicMask  // mountain belts on the mask gradient
       + craters(dir)                // impact field (below)
       + erode(...)                  // optional: repo hydraulic+thermal erosion
```

- **Craters (new — fills a noise gap):** Worley/cellular cells, each a profile
  `rimUplift − bowl(depth) + ejecta(falloff)`, sized by a power law (many small,
  few huge). What makes the Moon/Mars read as *cratered*, not *lumpy*.
- **Displacement:** coarse+mid bands → real cube-sphere vertex displacement
  (radial). Fine band → **tangent-space detail normal map** per face. Analytic
  normals from the height gradient, not face-averaging.
- **Relief self-shadow (the detail most fakes skip):** bake a **horizon-angle /
  cone-step map** per face so crater rims shadow their floors and ridges cast
  along the terminator — a normal map can't self-shadow, and grazing terminator
  light is exactly where it shows. Bake an **AO map** too.
- **Layered PBR material:** regolith / rock / ice / dust albedo+roughness sets
  blended by elevation, slope, latitude (ragged polar caps
  `|lat| > capLat − capNoise(dir)`), reusing the `terrainColor` biome pattern.
  Mars = oxide dust + basins + thin caps; Moon = mare vs highland albedo;
  Earth-like = biomes + oceans.

### P2 — Ocean (PBR water)  *(shader)*
Separate shell at sea-level radius, shaded properly: GGX **sun glint** smearing
the star toward the terminator, **Fresnel** grazing-limb brightening,
**depth-tinted transmission** (Beer–Lambert on the sea-floor height delta →
turquoise shallows, dark deeps). Feeds the atmosphere's aerial perspective.

### P3 — Atmosphere: physically-based scattering  *(shader/LUT — the centrepiece)* — **CPU reference DONE; GPU port pending**
Landed: `atmosphere.{h,cpp}` — a single-scattering Rayleigh + Mie raymarch
(Nishita/O'Neil core): `transmittance`, `atmosphereScatter` (view-ray march with
per-sample transmittance to the sun + planet-shadow test), `raySphere`, Earth /
Mars presets. Pure and headless — unit-tested for the physical invariants
(`tests/test_atmosphere.cpp`: transmittance ∈ (0,1] decreasing with path, blue
scatters more than red, blue zenith sky, dim shadowed night side, Earth-blue vs
Mars-red) and it drives `planet_preview` so the blue limb, soft terminator, and
Mars's thin haze are visible without a GPU.

Realtime shaders authored (a fullscreen scattering pass porting the CPU reference)
for all three backends: `shaders/vulkan/atmosphere.frag` (**SPIR-V compile-verified
with glslang**), `shaders/metal/atmosphere.metal`, `shaders/webgpu/atmosphere.wgsl`
(compile-unverified — no Metal/WGSL toolchain in CI). **Pending (device):** the
per-backend **pipeline wiring** (upload `AtmosphereUniforms`, insert the pass after
the scene / before the composite tone map) — tracked in `docs/renderer-parity.md` —
plus **multiple scattering** (Hillaire) for the glowing daylit limb.

Full design (the realtime target) — implement **Hillaire 2020**:
1. **Transmittance LUT** (2D: altitude × sun-zenith) — Rayleigh + Mie + ozone
   optical depth.
2. **Multiple-scattering LUT** (2D) — the term that keeps the daylit limb glowing
   instead of going black; what separates "good" from a single-scatter demo.
3. **Sky-view / aerial-perspective** — raymarch the medium: aerial perspective
   over the surface (distant terrain hazes out), limb integration for the
   halo-from-space.
4. **Composite** into the HDR scene: blue limb, **reddened sunset terminator**
   (falls out of the transmittance LUT), correct planetary shadow.

Physical params: `β_Rayleigh ∝ λ⁻⁴` per channel, Mie `g`, planet radius,
atmosphere thickness, ozone. Mars = thin, dusty (high Mie, brownish); Earth = the
classic blue. New shader + LUT compute, mirrored across backends.

### P4 — Clouds (volumetric)  *(shader)*
Cloud shell raymarched in a fragment/compute pass. Density from **Perlin-Worley**
3D noise × a 2D coverage/weather field, edge-eroded by detail noise. Lighting: a
**multiple-scattering approximation** (Beer–Powder silver lining), sun energy
along the light ray, ambient from the P3 sky LUT. **Cast cloud shadows onto the
surface** (sample cloud density along the sun ray in the surface shader) — the
detail that welds clouds to the planet. A shadowed 2D-shell fallback exists if the
volumetric budget is too high, but volumetric is the target.

### P5 — Gas giant (evolving fluid) + rings  *(headless fields; shader look)* — **static texture DONE; animation/rings pending**
Landed (headless): `generateGasGiantTexture(GasGiantParams, seed)` bakes a seamless
equirectangular albedo — latitudinal belts/zones warped by turbulent fbm flow, plus
vortex storms — onto the smooth cube-sphere (`generateGasGiantMesh`), Jupiter /
Neptune presets. Tested (`tests/test_planet.cpp`): longitude-seamless (the wrap step
is no worse than an interior neighbour step), latitudinal banding, determinism,
watertight sphere. **Still pending (device):** the evolving curl-noise fluid
*animation* (ping-pong advection), limb darkening in-shader, and rings with mutual
shadowing.

Original design notes:
- **Curl-noise advection:** a band+detail field in a ping-pong texture, advected
  each frame by a curl-noise velocity field + **differential rotation** (angular
  speed varies by latitude). Storms are seeded rotating vortices that shear and
  merge over time — evolving, not scrolling.
- Shading: banded albedo, **limb darkening**, subtle haze; the P3 shell wraps it
  for the soft edge.
- **Rings:** annulus mesh with a radial banded opacity/colour; **mutual shadowing**
  (planet shadow sweeps the rings; ring shadow bands onto the planet) and
  **forward-scattering translucency** (rings glow back-lit). Mutual shadowing is
  what makes Saturn read as photographed.

### P6 — Star & deep space  *(shader)*
- **Blackbody star colour** from temperature (Planck → chromaticity); HDR
  intensity so the existing bloom blooms it naturally.
- **Star disc with limb darkening** (a real sun edge is dimmer than centre).
- **Starfield:** magnitude-distributed procedural stars with colour spread (blue-
  white → red), not uniform white dots; sits at the reverse-Z far plane.
- **Night side:** emissive city lights on the dark hemisphere for Earth-likes.

### P7 — Motion, camera, scene, authoring — **Lua binding DONE; system/scene pending**
- **Lua binding** ✅ `planet.rocky{ preset=, seed=, radius=, face_res=, relief=,
  sea_level= }` → displaced biome mesh; `planet.gas{ preset= }` → smooth sphere;
  `planet.gas_texture{ preset=, seed= }` → equirect albedo Image. Registered in
  `procgen_bindings`, tested in `tests/test_script_vm.cpp` (runs under CMake, which
  links Lua — `make test` is Lua-free). Planets are now first-class procgen content
  like `flora.lua` / `city.lua` (ADR-0023/0025).
- **Axial spin + tilt** (pending) — a small `PlanetSystem` advancing
  `Transform.orientation`; clouds and gas bands on their own clocks.
- **Orbit camera** ✅ `OrbitCameraController` targets the origin; clamp near/far
  around the body + starfield.
- **Scene** (pending, device) — a `planet_demo` level wiring a planet + atmosphere +
  starfield + sun into the viewer.

---

## Sequencing (biggest quality-per-effort first)

1. **P0 cube-sphere + P1 surface** — a real cratered, self-shadowing Mars on the
   existing PBR path. Foundation, mostly headless.
2. **P3 atmosphere (Hillaire)** — the biggest visual leap; before clouds/ocean
   because everything composites through it.
3. **P2 ocean + P6 star/starfield** — cheap relative to impact once HDR +
   scattering exist.
4. **P4 volumetric clouds** — heaviest; slot in once the P3 sky LUTs light them.
5. **P5 gas giant + rings** — a parallel track reusing the P3 shell.

**Minimum lovable planet:** P0 + P1 + P3 + P6 — a spinning, atmosphere-haloed Mars
against real stars, none of it touching Ring 3.

## Testing split

- **Headless / CI:** cube-sphere, height field, craters, displacement, normal/AO/
  horizon bakes, gas-giant fields, texture bakes, determinism (`seed → identical
  mesh`), watertightness, bounds.
- **GPU / on-device:** atmosphere scattering, terminator shading, ocean glint,
  volumetric clouds, animated flow, bloom/star. Vulkan (Linux, with a GPU) or
  Metal (macOS).
- **Headless preview:** `make planet_preview` (`tools/planet_preview.cpp`) renders
  orthographic "from space" discs of the presets to `out_*.png` — a GPU-free way to
  eyeball relief, biomes, caps, craters, and gas bands as the generators change.

---

*Living document. Update as phases land; add ADRs to `docs/decisions.md` for
significant decisions. Companion decision: ADR-0076.*
