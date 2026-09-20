# Freeway rules — enumeration, application, expected outcomes

Requested on-device: "It should start with an enumeration of the rules, how
they're applied and what the expected outcomes are in an easy to see
fashion." This file IS that enumeration. Every rule lists WHERE it runs,
WHAT you should see, and WHICH lab proves it in isolation.

## The pipeline (what happens, in order)

| # | Stage | Where | Output |
|---|-------|-------|--------|
| 0 | Hubs + MST links | `metro.cpp` (buildMetro) | hub positions, hub-to-hub links |
| 1 | Route chaining | `metro.cpp` | links chained into through-routes (anchor polylines, `RoadNet.freewayPlans`) |
| 2 | Planner rules | `level_loader.cpp` synth block | routes deconflicted: bridge bumps, drops; diamonds stamped through global gates |
| 3 | Profile solve | same | terrain-eased PVIs + rule-2 bridge tents |
| 4 | Anchor resolution | same (per exit) | each ramp claims a street junction node (side/flow/degree/spacing gates) |
| 5 | Construction | `corridor_mesh.cpp` | deck, gore bands, free runs, mouths, piers, flatten — §11 parts, shared-vertex welds |
| 6 | Graph + welds | loader + `city_render.cpp` | one unified road graph; ramp terminals welded; junction stubs grafted |

Street growth currently happens BEFORE stage 2 — that inversion is why
`metropolis_roads` shows dead-ended streets (cut pass) and freeways that
"just stop". The intended order (§10.4, not yet built) is corridors FIRST,
then streets grow around them, seeded toward the ramp landings.

## THIS WHOLE PIPELINE IS LEGACY (2026-09-20)

Glenn, after looking at it in the viewer: *"that's all legacy, and now that I see it's confusing
you we need to deprecate that and eventually remove it from the code base ... the new road system
should be given priority."*

The corridor freeway — this planner, `corridor_plan`, `corridor_mesh` and the bake — is the OLD
road system's freeway, and it does not merge properly with the streets around it. It stays only for
the levels that already ship with it (freeway_lab, freeway_variants, hillcity, metropolis,
metropolis_roads, metropolis_sky). **No new level should set `corridor_freeways`**, and the path
now says so once per process when it runs. The lanes builder
(`procgen/city/roads/lanes`) is the road system going forward; what is worth keeping from the text
below is the ROUTE planning — where a freeway should go, as an anchor polyline — which is cheap,
geometry-free, and is the input the lanes builder needs. The rest goes when lanes can build a
freeway.

### The trapdoor that hid it: no hubs, no freeway

Worth recording because it explains why metro never had a freeway. Stage 0 is gated
`if (p.freeways && H >= 2)`, and `H` was read near the top of `buildMetro` — **before** the
footprint skeleton runs. Footprint mode fills the hub list itself, so for every footprint city `H`
was a stale 0 and the whole freeway pipeline was skipped without a word:

```
[metro/freeway] skeleton=footprint  H=0  hots=3  freeways=1  corridorFreeways=1
```

Three hubs present, count says zero, the recipe asking for freeways, nothing built. The comment
above the skeleton made the skip look deliberate — and half of it was: footprint mode means to skip
the LEGACY street backbone. The corridor planner just shared the block. The count is now taken
where it is used (`RT_METRO_TRACE_FREEWAY=1` prints the gate and every cull after it), which means
the gate is honest — but nothing opts in.

**metro_v2_test keeps the city it has.** Its `corridor_freeways` key is now `false`, which is what
it has always built; left at `true` it would have grown a legacy freeway the moment the count was
fixed. A `metro_v3` with the freeway on was built, looked at, and deleted: 2028 m of elevated
legacy corridor, 688 m of ramps, and three gates broken (10 signal poles standing in a carriageway,
1014 terrain samples poking the deck, bus coverage halved). Not worth fixing in the old system.

## The rules

### Stage-1 rules (route shape)
| Rule | Statement | Applied | Expected outcome | Verified in |
|------|-----------|---------|------------------|-------------|
| R1.1 | A route continues through a hub only via the straightest unvisited link, never turning > ~78° | `metro.cpp` walk() | routes read as through-routes, not zigzags | metropolis_roads (aerial) |
| R1.2 | Turn budget (~120°/3 hops) in the walk + self-approach TRUNCATION on final polylines | `metro.cpp` walk / loader planner | no generated loop routes; an authored curl truncates with a warn | rules_lab specimen L |
| R1.3 | Terminus rule (REVISED on-device): the mainline is its own system — it ends FULL-WIDTH as a dead end; ramps are the ONLY couplings to streets. (The taper+graft machinery exists but is off for generated routes.) | loader synth (`taperEnds=false`) | freeway ends read as intentional stubs, never a fake dissolve into a street | all labs (route ends) |
| R1.3b (PLANNED) | Generated routes EXTEND their end legs to the domain boundary, so dead ends live at the map edge, not mid-city | — | freeways read as passing THROUGH the region | metropolis_roads |
| R1.3c (PLANNED) | Wander/trip goals never snap to Freeway/Ramp nodes (a corridor tip is not a destination) | — | no cars parked at dead-end tips | any lab (warmed) |

### Stage-2 rules (network deconfliction)
| Rule | Statement | Applied | Expected outcome | Verified in |
|------|-----------|---------|------------------|-------------|
| R2a | Route contact < 140 m = CROSSING → the SHORTER route gets a bridge tent (9 m clearance, 5% grade, 60 m curves) | loader synth | one route flies over the other; piers never under the upper deck | rules_lab **specimen X** |
| R2b | Route contact > 140 m = PARALLEL OVERLAP → shorter route dropped, warn | loader synth | one route visibly absent + log line | rules_lab **specimen P** |
| R2c | Bents dodge other corridors (their centrelines join the pier-avoid graph, width 34) | loader → `corridor_mesh` | no flyover column lands on a deck below | rules_lab specimen X (close-up) |

### Stage-2 rules (interchange stamping — all gates must pass or the ramp is never stamped)
| Rule | Statement | Expected outcome | Verified in |
|------|-----------|------------------|-------------|
| R3a | Landings ≥ 60 m apart, GLOBALLY | no crowded double landings | rules_lab **specimen R** |
| R3b | A ramp run may not cross another ramp's run | zero criss-cross | specimen R |
| R3c | A ramp run may not graze another corridor at grade (< 26 m) | ramps never thread under/through a neighbouring freeway | specimen R |
| R3d | No diamond inside a bridge zone (± 320 m) | no exits on flyovers | specimen X |
| R3e | Diamonds only where the route is long enough (≥ 420 m; decel ≤ 28% of length) | short spurs get no half-baked ramps | variants lab |

### Stage-4 rules (anchor resolution — where a ramp lands)
| Rule | Statement | Expected outcome | Verified in |
|------|-----------|------------------|-------------|
| R4a | Candidates are AUTHORED street nodes on the ramp's own side of the deck (≥ 12 m clear of centreline) | never a target across the mainline (no under-deck dives) | variants lab |
| R4b | Exits land DOWNSTREAM, on-ramps feed from UPSTREAM (flow-aware, per carriageway) | no U-turn ramps | variants lab |
| R4c | Junction-degree nodes preferred; dead-end stubs last resort | landings are real intersections | variants lab |
| R4d | 70–340 m from the gore; unresolvable ramps are DROPPED loudly | no ribbons to nowhere | fold/ocean cases |

### Stage-5 rules (construction — §11 parts)
| Rule | Statement | Expected outcome | Verified in |
|------|-----------|------------------|-------------|
| R5a | The gore band's inner edge IS the deck edge (same ribs, shared vertices) | a ramp can never detach from or slide under its own deck | variants lab close-ups |
| R5b | Free runs must stay out of the mainline footprint (guard = max half-width + 1.5) | warn + visible only as a deliberate error | variants lab |
| R5c | Aux lanes taper smoothstep (accel 90 m, decel 40 m); dashes stop below half-width | merges read as gradual merges | variants lab |
| R5d | Aux spans clamp ≥ 30 m from corridor ends | no peel-offs to nowhere at the tips | variants lab |
| R5e | Mouths flare over the last 45 m into a 2-lane throat; street paint owns the junction | landings read as real approaches | variants lab |
| R5f | Elevation: structure box + piers from 0.35 m; bents every 28 m (slide ≤ 12 m to clear roads below, else longer span) | no berms, no columns in carriageways | variants + roads lab |

### Stage-6 rules (graph)
| Rule | Statement | Expected outcome | Verified in |
|------|-----------|------------------|-------------|
| R6a | Carriageways are one-way chains at each carriageway's centreline; lanes span full width | cars centred in real lanes, never on the median | variants lab (drive) |
| R6b | The nav ramp chain is built from the DRAWN path (`rampPaths`) — one source of truth | mesh and graph cannot disagree | all labs |
| R6c | Ramp terminals weld to the grafted stub's outer node; too-shallow stubs downgrade to weld-only | every drawn ramp is drivable end to end | variants lab |
| R6d | Pedestrian routing excludes Freeway/Ramp links | no walkers on carriageways | metropolis |

## The labs
- **freeway_variants** — stage-5 parts in isolation (4 specimens: lanes × elevation).
- **rules_lab** — stage-2/3 planner rules in isolation (authored raw plans; one specimen per rule, expected outcome in the entity name).
- **metropolis_roads** — integration sandbox (corridors opt-in). NOTE: currently reuses streets grown WITH the old freeway edges — dead ends and stopped freeways here are the pipeline-order inversion (see above), not rule failures.
- **metropolis** — ships with corridors OFF until the above are green.
