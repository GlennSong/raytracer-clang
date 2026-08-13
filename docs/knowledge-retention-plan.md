# Knowledge Retention — turning lessons into things that fail

## The problem, stated precisely

Four to five months of development have produced a large body of hard-won,
*correct* knowledge: decision documents, engine rules, invariants recorded next to
the bugs that caused them. And it keeps getting rediscovered — at the cost of days.

This is not a documentation problem. Everything below was already written down.

### The evidence (one session, 2026-08-11/13)

| lesson | where it lived | what happened |
| --- | --- | --- |
| road clearance — a building corner must clear `edge_width/2` of every centreline | two arguments on `growLotBuildings`, with the device report in the comment | the replacement pass was written with a narrower signature. Nothing failed to compile. Buildings stood in the carriageway again. |
| the rooftop kit, `offsetPlan`'s mitre | file-local `static`s in `shape_grammar.cpp` | unreachable from the new pipeline, so both were reimplemented — worse. A parapet became a zero-thickness ribbon beside a working `emitPlanParapet`. |
| **collidable by default** | **`AGENTS.md` § Playable Scenes** | **violated anyway.** The rule names road surfaces explicitly and predicts the exact symptom ("flying over it hides that the actor is falling through everything"). The roadlab bridge spawns `Renderable` with no collider. |
| **junction welding** | **`docs/junction-weld-decision.md`** — a decision with measurements | **bypassed.** The doc says keep the lattice bodies and replace the per-degree patch zoo with one full-ring quad meshing routine; it measures the failure at 16% double-cover and quotes the ≥5-arm fan's own "Stopgap" comment. A separate junction system was then written in `proto/roadlab/`. |
| `fitArcs` idempotence | `docs/lot-system-plan.md` §5 **and a test** | the test was **red at handoff** while the commit message claimed "1197/1197". A tolerated red gate is worth exactly what prose is worth. |

The third and fourth rows are the ones that matter. They kill the intuitive
answer. The knowledge was written down, in the files everyone reads, in the
project's own rule file and its own decision record — and it did not survive
contact with a new subsystem.

**More prose cannot fix a prose problem.** `AGENTS.md` is already ~400 lines;
every line added lowers the odds any single line is honoured. A longer preflight
is the same medicine that just failed.

### What *did* survive

The winding convention. The tower triangle budget. Not because they are better
documented — because `lotmesh_winding_matches_the_engine_convention` **fails**
when you break it.

> **Knowledge survives only in a form that can fail.**
> Everything else is a hope with a timestamp.

That gives a ladder of enforcement, weakest first:

1. **Prose** — zero enforcement. Right home for *why*; useless for *don't*.
2. **Doc + test** — only as strong as the gate's discipline.
3. **Test in a gate that is always green** — real enforcement.
4. **Type / signature** — the compiler enforces it; cannot be forgotten.
5. **Harness hook** — enforced before work can be called done.

Today's failures are each a rung-1 or rung-2 item that needed to be rung 3 or 4.

---

## The work

Five items, ordered by leverage. Each stands alone and each is small.

### 1. Executable invariants for the Engine Rules

Every bullet in an `AGENTS.md` "(Engine Rule)" section either gets a test that
fails when it is violated, or is explicitly demoted to **advice**. The rules file
becomes an *index of gates* rather than a list of intentions, and the demotions
are honest rather than aspirational.

Start with the three broken this session:

- **`tests/test_levels_playable.cpp`** — over every `assets/levels/*.json`: it has
  a `player`; it has at least one collider; there is collidable ground beneath the
  spawn. This is the "Collidable by default" and "Always a player start" rules,
  made executable. It fails today on the roadlab levels.
- **Road clearance** — grow a city from a road net with *mixed* widths (a 12 m
  street and a 17 m arterial on the same block, because a single scalar margin
  cannot serve both) and assert no `LotBuilding::plan` vertex falls within
  `edge_width/2` of any centreline. A draft exists; see "Carried over" below.
- **Registry completeness** — already landed on the lot-system branch as
  `BuildingMesh::unbuiltElements` + `every_element_a_stock_recipe_asks_for_is_built`.
  Keep it as the template for the shape of these tests: count the silent drop,
  then assert it is zero.

### 2. The gate must be green, and honest

A red gate that is tolerated trains everyone to ignore gates, and it cost real
hours this session — the handoff claimed 1197/1197 while `fitArcs` was failing, so
the next session had to re-derive whether it had broken something.

- CI fails on a red `make test` / `ctest` (`docs/ci-plan.md` Stage 0 already
  specifies this; confirm it actually blocks rather than reports).
- A `Stop` hook in `.claude/settings.json` that refuses to end a turn with a red
  gate, or requires the turn to state plainly that it is red and why.
- Never hand off with a red gate silently. If it must be red, that is the
  handoff's **first line**, not a footnote.

### 3. Run the duplication scanner that already exists

`make health` (`tools/code-health.py`) exists and **nothing runs it** — neither
`.github/workflows/build.yml` nor `deploy-web.yml` references it. A duplication
scanner nobody runs is how a second `offsetPlan` and a second junction welder both
got written.

Add it to CI. If it is too noisy to block, make it report a *delta* and block on
regressions only. Then extend it toward the failure that actually bit: not
copy-pasted lines, but **two implementations of one concept** — the signal is a
new symbol whose name or call graph mirrors an existing one across module
boundaries.

### 4. A standing capability ledger

`docs/lot-system-plan.md` §18 was written this session as a one-off audit, and it
was the highest-value artifact produced: it converted "why didn't we reuse it?"
into a table of *what exists, where it lives, and whether it is reachable*.

Make it permanent as **`docs/capabilities.md`**: capability → where it lives →
status (`live` / `file-local` / `declared-but-unbuilt` / `superseded`) → the
decision doc that governs it.

Seed it from what is already known:

- the swept-lattice mesher and its junction patch (`road_lattice.cpp`), governed by
  `docs/junction-weld-decision.md`
- the rooftop plant planner (`roof_plant.h`), the mitred ring offset
  (`offsetPlan`), the facade element emitters
- `CityInstanceGroup` prop instancing; `LotFixtures` (built, never consumed)
- the two building pipelines and the flag that switches them

**The rule that gives it teeth:** before building a subsystem that overlaps an
existing capability, update the ledger *first* — state what exists and why it is
not being used. That single step, taken before `proto/roadlab/junction.cpp`, would
have surfaced `junction-weld-decision.md` while it still cost nothing.

### 5. Task-shaped preflight — a Skill, not a longer rules file

A generic preamble is weak; a checklist that fires for *this kind of work* is
strong. Encode it as a Skill (or a `PreToolUse` hook on the relevant paths), short
enough to actually be read:

> **You are adding a subsystem that produces world geometry.**
> 1. What already does this? (`docs/capabilities.md`, and is there a decision doc?)
> 2. What collider does it get, in the same recipe that makes the geometry?
> 3. What test proves it, and does that test fail before you write the code?

Three questions, asked once, at the moment they are cheap.

---

## Sequencing

Do 1 and 2 first: they are the smallest and they convert this session's actual
failures into gates. 3 is nearly free. 4 is the highest long-term leverage and the
one that would have prevented the largest waste. 5 is worth doing only *after* 4,
because it depends on the ledger existing.

Definition of done: the three tests exist and pass (after their fixes land), CI
blocks on `make test` and `make health`, `docs/capabilities.md` exists and is
referenced from `AGENTS.md`, and every "(Engine Rule)" bullet is either backed by
a named test or marked as advice.

---

## Carried over from the session that produced this

- A drafted road-clearance test (mixed 12 m / 17 m widths, asserts no footprint
  vertex within `edge_width/2` of a centreline) is in that session's scratchpad as
  `test_lot_mesh.WITH_clearance_test.cpp`. It is written to **fail** on current
  `main`-line behaviour — that is deliberate; write the gate before the fix.
- The related bug it guards: `level_params.cpp` derives the lot road margin as
  `4.0 + sidewalk`, where `4.0` is a hardcoded "road half" unrelated to the level's
  actual road widths. On a 17 m arterial that puts the lot line inside the
  carriageway. The shipping pass compensated with a per-edge test (`clearOfRoads`,
  `city_lots.cpp`); the Lot System does not receive it.
- `make test` on `claude/road-system-architecture-wxkehz` currently fails to link:
  `assetRoot()` / `assetPath()` are referenced from `script_assets.o`, and
  `src/engine/asset_root.cpp` appears in the Makefile's source lists zero times.

## A note on where knowledge lives

An agent's private memory is *not* the system of record. This session held useful
notes — verify in the editor, collidable by default, the roads plans — and none of
them could help a concurrent session working the same repository. Anything that
must survive across agents belongs in the repository, behind a gate.
