# The duplicate-systems sweep (#117)

Glenn (2026-10-26): "make a sweep of the code base to make sure that we don't have a lot of duplicate systems." The
rule is AGENTS.md's *one system per job*. This is the issue's gate: every job found with more than one implementation,
and the verdict on it. Each verdict was one of merge, delete, or keep with a reason. The plan and its waves are in the
issue; each row's ADR (docs/decisions.md) says what was kept and why.

Done 2026-10-27 to 2026-10-31: PR #139 (wave 3) and PR #170 (waves 0-2).

## Wave 0: dead code and copies

| # | Job | What there was | Verdict | Where |
|---|---|---|---|---|
| 1 | Walkers' door graph | `ped_graph` beside NavGraph, never wired | delete | ADR-0294 |
| 2 | Swim rise | `SwimState::verticalSpeed` beside `swimVerticalStep` | delete the unused one | ADR-0294 |
| 3 | GLSL shaders | four files Vulkan never loaded | stop compiling | ADR-0294 |
| 4 | Car body mesh | the lofted `buildCarShell` beside the mesh kit's cars | delete | ADR-0294 |
| 5 | AI driving stack | audited as dead; it drives the possessed car | keep; delete three uncalled pieces | ADR-0294 |
| 6 | The kerb side | `rightOf` written three times | one, in nav_graph.h | ADR-0294 |
| 7 | Near forest trees | a local copy of the tree part | `RecipeTreePart` | ADR-0294 |
| 8 | Triangle BVH | named `KdTree` | rename `TriangleBVH`; `AABB` its own header | ADR-0294 |
| 9 | Stale words | about 45 comments and docs that disagreed with the code | the code wins; 15 ADRs get a note | nine doc commits |

## Wave 1: mechanical merges

| # | Job | What there was | Verdict | Where |
|---|---|---|---|---|
| 10 | Thread pools | nine procgen spots with their own threads, two private pools | `sharedJobs()` / `parallelRows` | ADR-0288 |
| 11 | Box blur | three separable blurs | `boxBlurGrid` (the weather bake's joins at the next weather tag bump) | ADR-0289 |
| 12 | CDLOD morph target | written in both meshers | one stencil and pin rule | ADR-0290 |
| 13 | Viewer socket clients | four in tools/ | `tools/rtctl.py` | ADR-0291 |
| 14 | Eye height | four copies of 0.7 m | `kEyeAboveCentre` | ADR-0287 |
| 15 | Road ear-clipper | already earcut | nothing to do | ADR-0293 |
| 16 | Lane arrows | citysim's angle-guessed copy beside the lanes deck's | the lanes deck's | ADR-0293 |
| 17 | Building a frame | the trace rig's drifted copy | `FrameOwners` | ADR-0292 |

## Wave 2: real merges

| # | Job | What there was | Verdict | Where |
|---|---|---|---|---|
| 18 | Road classes | the planner's, the builder's defaults, the island's constants | `roadClasses()`, one table | ADR-0276 |
| 19 | Terrain router | `routeRoad` beside `routeOnTerrain` | `routeOnTerrain` | ADR-0277 |
| 20 | Lattice road passes | still running behind a switched-off flag | delete | ADR-0278 |
| 21 | Drainage routing | hydrology's flood beside stream power's | `floodDrainage` (hydrology joins at the next hydro tag bump) | ADR-0279 |
| 22 | Height samplers | three triangle splits, two pickers, `SurfaceField` | one of each; `SurfaceField` deleted | ADR-0280 |
| 23 | Vehicle handling | three config builders, two readers | the recipe, one reader | ADR-0273 |
| 24 | AI driving controller | three | Stanley over a lane frame | ADR-0274 |
| 25 | Crash measures | on foot vs in a seat | keep both, different questions | ADR-0281 |
| 26 | Sitting in a vehicle | the driver's own path beside the riding component | the riding component | ADR-0275 |
| 27 | Roles | an enum beside the catalog | the catalog (`RoleId`) | ADR-0282 |
| 28 | Engine sound | two voice pools | `EngineVoicePool` | ADR-0285 |
| 29 | Gun | C++ `ShootingSystem` beside gun.lua | gun.lua | ADR-0283 |
| 30 | Which water is here | drawn sea vs swimmable sea | one set of cells | ADR-0286 |
| 31 | Instance bounds | the sphere under an instance transform, twice | `instanceCentre/instanceScale`; the three culls keep their jobs | ADR-0284 |

## Wave 3: architecture

| # | Job | What there was | Verdict | Where |
|---|---|---|---|---|
| 32 | Commands and events | Settings used as a string message bus | the control bus: typed requests and named events | ADR-0253 |
| 33 | Shader languages | MSL, GLSL, WGSL by hand | its own issue | #141 |
| 34 | The player's body | three drawers and a seated manikin | one posed body with views | ADR-0254 |
| 35 | Buildings | two massing paths, two face types, three malls, many floor planners, party walls twice, two grow entries | one of each | ADR-0258..0264, 0268, 0270..0272 |
| 36 | Cubic curves | five types | two kernels in curve.h | ADR-0255 |
| 37 | Cache keys | six schemes | one hash, usage records, `rt_bake --prune` | ADR-0256 |
| 38 | Random streams | `std::mt19937` beside `ProcRng` | `ProcRng` | ADR-0265 |
| 39 | Erosion | droplets beside the weather bake | the weather bake | ADR-0257 |

Before the waves: the `ride` verb was handled twice; one handler (48518900).

## Kept on purpose

- Two tree systems: Glenn's exception while reconciling them would cost frame time or memory; #135 measures it.
- The GPU crowd and the CPU far-crowd fallback, for renderers without compute.
- The nav and lot-nav twins (ADR-0115).
- The grapple rope moves onto Jolt in its own issue, #118.

## Waiting on the batched re-bake

Two merges are written but change baked output, so they ship with their tag bumps (#171): hydrology onto
`floodDrainage` (item 21, `kHydroCarveCodeTag`) and the weather bake's range blur onto `boxBlurGrid` (item 11,
`kWeatherCodeTag`).
