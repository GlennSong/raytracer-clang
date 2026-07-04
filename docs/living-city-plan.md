# Living City — design plan (PROPOSAL)

**Status:** proposal, pending sign-off. Working assumptions are marked **⟨A⟩** —
change any and the plan flexes. No code until the shape here is agreed.

## The vision

Normal city life, made legible. Buildings that *mean* something (a home, a
shop, an office, a park); NPCs with roles and routines who route to real
destinations and act when they get there; tools they can pick up and use (a gun
is just one tool among brooms and briefcases); a player who can watch the city
or step into it. Not a combat sim — a *living place*.

Nearly all the machinery already exists (ADR-0059…0065): the NavGraph + A*, the
CitySim planner + cognition loop, the data-driven `StateMachine` + Lua goal
tables, articulated people with a walk cycle, spectate/third-person cameras to
watch it, the debug HUD + "how the city thinks" page to read it, and the
promotion model to interact. The genuinely *new* engineering is small and
sharp: a **places layer** and a **pedestrian navigation graph**.

## Working assumptions ⟨A⟩

| # | Decision | Assumed | Alternatives |
|---|----------|---------|--------------|
| 1 | How places are defined | **Hybrid** — the generator tags building types + entrances automatically; Lua/level data can override or name specific ones | procedural-only; authored-only |
| 2 | NPC↔building depth | **Route-to-door & vanish** — walk to the entrance, "go inside" (despawn/idle), reappear later. No interiors | a few enterable shells; full interiors |
| 3 | First target | **Vertical slice** — a small hand-labeled town proving the whole loop, then scale | label grown.json; full procedural district |
| 4 | Roles & tools in v1 | **A few visible roles** — shopkeeper, commuter, stroller + one tool-user | richer schedules only; full role+tool system |
| 5 | Player role | **Undecided** — participant (has a home/job) vs observer (free agent). Doesn't block phases 1–3; decide by phase 4 | — |

## The new architecture

Two new pieces; everything else is reuse.

1. **Places / POI layer.** Each building gains: a **type** (home / shop /
   office / park / …), an **entrance** point snapped to the nearest sidewalk,
   and light metadata (open hours, capacity). A `PlaceMap` indexes places by
   type so an agent can ask *"a shop near me"* and get a destination. Authored
   in the slice level's JSON first (⟨A⟩ hybrid → later emitted by the generator).

2. **Pedestrian navigation graph.** A walkable graph — sidewalk segments +
   crosswalk edges + building-entrance connector edges — that the existing A*
   traverses. Today walkers move *along* road sidewalks; this adds the mid-block
   entrance links and the crossing edges so a walker can actually go
   house → corner → crosswalk → shop door. Built from the road net + the places.

3. **(reuse) Place-aware goals.** A `GoTo(placeType)` goal-table action resolves
   to a concrete place via the `PlaceMap` and routes on the pedestrian graph.
   Roles are just goal-table *variants* — a shopkeeper's table says "go to the
   shop, stay until close"; a commuter's is today's home↔work. All Lua data
   (ADR-0064), no C++ per role.

4. **(reuse) Capabilities / tools.** A `Capability`/`Holdable` component + a
   `Tool` world object + `equip`/`use` actions the `StateMachine` triggers. The
   player's existing `gun.lua` generalizes into the same system — the payoff of
   "everyone is an agent": an NPC picks up and uses a tool through the exact
   machinery the player does.

## Phased build — each phase shippable and testable

- **Phase 1 — Places data model (headless).** `Place` struct, `PlaceMap`,
  building-type tagging (hand-authored JSON in the slice), entrance snapping to
  the nearest walkable edge. *Tests:* places indexed by type; nearest-place
  query; every entrance lands on a sidewalk. No visible change yet — pure
  foundation, fully unit-tested.

- **Phase 2 — Pedestrian nav-graph (headless).** Build the walkable graph
  (sidewalk + crosswalk + entrance edges) from the road net + places; A* routes
  house → shop-door. *Tests:* a route exists and never leaves walkable edges;
  determinism (ADR-0002) preserved.

- **Phase 3 — Place-aware goals (headless sim).** `GoTo(placeType)`; agents draw
  home/work/errands from places of the right type; route on the ped graph;
  arrive-at-door → "enter" (idle/despawn) → reappear on schedule. *Tests:*
  agents reach their target place; a shop's customers arrive and leave; the day
  schedule holds; still deterministic.

- **Phase 4 — Roles + the vertical-slice level (device).** Author a small
  labeled town (homes, a shop, an office, a park); role goal-tables in Lua
  (shopkeeper holds the shop during hours; commuters; strollers). Watch it live,
  spectate a shopkeeper's day. *This is the first "it's alive" moment.*

- **Phase 5 — Tools / capabilities.** `Capability` component + a `Tool` world
  object + equip/use; one role uses a tool; fold the player's gun into the same
  system. *Tests:* the capability predicate + equip/use logic headless; the
  pickup/render device-gated.

- **Phase 6 — Scale to procedural.** Extend the city generator to tag building
  types, emit the `PlaceMap`, and build the pedestrian graph in one pass, so a
  whole generated city is instantly alive. The hand-authored slice from phase 4
  becomes the regression oracle.

## What we're leveraging (already shipped)

NavGraph + A* · CitySim planner + cognition (memory/prediction/commitment) ·
`StateMachine` + Lua goal tables · articulated people + walk cycle ·
spectate + third-person cameras · debug HUD + docs page · promotion (interact) ·
Lua-authored car fleet + light seams. Phases 1–3 are pure, headless, and
determinism-safe — they build entirely on tested foundations.

## Open questions for you

1. **The "alive" scene** you'd point at to say it's working — the shopkeeper
   opening at dawn? a crowd at a stop? recognizing someone from yesterday?
2. **Player: participant or observer?** (Forks phase 4/5 — does the player have
   a home/job, or is the city a world they move through?)
3. **Interiors, ever?** Is route-to-door the enduring model, or is walking
   *inside* buildings a goal we should leave room for?

---

*This is a proposal doc, not an ADR — it records a plan to be built, not a
decision already made. It graduates into ADR-0066+ as each phase lands.*
