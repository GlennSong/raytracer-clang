# History: plans and prototypes that never merged

These docs come from branches that never reached main. Their ideas fed what we built, and some of the ideas were dropped. The branches were deleted on 2026-10-23, after each tip was kept as an `archive/<branch>` tag. To see a branch exactly as it was:

```
git checkout archive/<branch>
```

Nothing under an archive tag is built or tested. Main has one road system, the lanes builder (`src/engine/procgen/city/roads/lanes/`).

| doc | branch | tip | what it is |
|---|---|---|---|
| [compute-shaders-plan.md](compute-shaders-plan.md) | `claude/compute-shaders-apis-ru8dja` | `1c976aee` | Where GPU compute could take work off the CPU (07-01) |
| [vision-pro-plan.md](vision-pro-plan.md) | `claude/compute-shaders-apis-ru8dja` | `1c976aee` | Vision Pro port: shared space first, then immersive stereo (07-01) |
| [shader-single-source-plan.md](shader-single-source-plan.md) | `claude/engine-webgpu-compile-ipm65w` | `0556af14` | One shader source for Metal, Vulkan and WebGPU: pros, cons, phases (07-03). Pushed after PR #8 merged, so it never landed |
| [road-network-v3-plan.md](road-network-v3-plan.md) | `claude/road-network-redesign-cjasmz` | `648335f5` | Road network v3, a total-conversion plan (07-18) |
| [roadlab-prototype.md](roadlab-prototype.md) | `claude/road-system-architecture-wxkehz` | `3ca6ee09` | The August clean-room road prototype (`proto/roadlab/`, 34 commits). Covers the strip-stack cross-section, markings as a shader, slope earthworks, interchanges with gores, and OpenDRIVE export and import. Main has no OpenDRIVE code; the prototype is the place to start from if we ever need it. Also the fallback census, design lint, and 23,450 standalone test assertions |
| [lot-system-plan.md](lot-system-plan.md) | `claude/procgen-city-building-kpuxzd` | `261c3df2` | The Lot System: Shape2 kernel, plan grammar, mass stack, parceller, Lua vocabulary (08-10..12) |
| [lot-lab-findings.md](lot-lab-findings.md) | `claude/procgen-city-building-kpuxzd` | `261c3df2` | What the Lot System lab measured |
| [lot-system-handoff.md](lot-system-handoff.md) | `claude/procgen-city-building-kpuxzd` | `261c3df2` | Handing the Lot System to a local session |

Branches with no docs of their own, kept by tag only:

| branch | tip | what it is |
|---|---|---|
| `claude/vision-pro-ar-capabilities-32ofop` | `ab0ab676` | 30 commits of AR work (`src/engine/xr/`): grabbing, tossing, a physics gun, shadow catchers on planes (08-15) |
| `fix/pass-cost-restore-guard` | `a03ef26c` | A 14-line `pass_cost` fix: restore only what was saved (08-09); main's file has changed since |

`claude/roadlab` (the September lanelab, ADR-0083/0084) did land. It became the lanes builder. Its branch stays while its worktree is checked out.
