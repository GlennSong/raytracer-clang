# Retrospective — the first three months

**2026-06-01 → 2026-08-31.** 92 days, 1,299 commits, 83 architecture decision
records, ~171,000 lines of C++, 158 test files. Started as a Cornell box with a
KD-tree. Ended as a city you can walk into a building in, on three render
backends, with a Qt editor, a physics vehicle fleet, a sky with a real moon, and
an offline path tracer that still agrees with all of it.

This document is the honest version. Every number below is checkable in the
repo — a commit, a census, a test name. Where something is still broken it says
so, and where a plan was wrong it says which measurement refuted it.

---

## The spine

**June — the foundation blitz.** 45 ADRs, nine of them on June 3 alone.
Window/Renderer seams, fixed timestep, sparse-set ECS, the `Application`+`System`
spine, Jolt behind a Jolt-free wall, the property layer, Lua as the one procgen
authoring path. Then in the back half of the month the whole procgen substrate —
L-systems, curves, CDLOD terrain, the road model, the city grammar. The rate of
*decisions* in June is the highest the project has ever had, and almost all of
them are still standing.

**July — systems, and the city that had to use them.** 11 ADRs: the EventBus,
debug draw, Tracy, audio, the animation skeleton, glTF skinning, the dependency
policy. Underneath, the city stopped being a demo and became a simulation —
agents with goals, perception, a car fleet, the Living City's places and routing.
ADR-0075 (terrain conform as a grading cascade) lands on July 9 and sets up most
of August's hardest work.

**August — contact with reality.** 6 ADRs, and a different character entirely.
CI exists for the first time on August 5 — **day 66 of 92**. The control channel
(ADR-0078) lets an agent drive the viewer and *look*, on August 17. On August 23
the Vulkan backend runs on real Linux hardware for the first time. The last week
is the sky, the road network's joint vertical solve, the earthwork field, and
buildings you can walk into.

The shape of that curve — 45 decisions, then 11, then 6 — is not maturity. Some
of it is: the foundational questions were genuinely answered. But two of August's
six exist only because this retrospective was being written: the world clock and
the earthwork field are ADR-0081 and ADR-0082, **recorded retroactively, days
after they shipped**. A third change of the same weight — the Qt editor reaching
Linux with a working viewport — still has no ADR, only a plan that had been
stale for two and a half months until today. And 21 of the 83 records carry no
date at all. Decision-recording decayed while the decisions got bigger.

---

## What worked

### 1. The seams held, and we know because the trigger fired

ADR-0001 (2026-06-03) put platform specifics behind two seams, `Window` and
`Renderer`, and honestly recorded its own weakness: *"the abstraction is unproven
by a second backend."* It set an explicit revisit trigger — *"adding a second
backend (Vulkan) — at which point validate the seam holds."*

That trigger fired **81 days later**, on 2026-08-23, when Vulkan first rendered
on an RTX 3080. Six real bugs surfaced that day. Every one of them was *inside*
the backend — a descriptor-pool cap, back-face culling, a shader hash, a baked
albedo being overwritten, a camera sign, a Qt platform plugin. **None was a leak
through the seam.** The June bet paid, and it paid in a form that could be
checked, because the person making it wrote down what would prove them wrong.

Writing the revisit trigger is the cheap half. Actually reaching it and asking
the question is what made it worth anything.

### 2. Instrument before you fix

The strongest single pattern in the codebase. The clearest case is the T-junction
at (-877,-423) on metro_v2: a collector arriving at -78% into a +33.5% arterial
with a 9 m cliff at the pad edge. **Five different plausible explanations fit the
screenshot** — a fixed feather, authored-deck flags, and three more, each of which
would have justified a different fix.

None of them was right. A new dump (`RT_JUNCTION_DUMP`, every surface at 1 m
stations from the node) showed the three arms meeting the node at 51.32 / 48.64 /
43.30 m and named the real cause: each chain solved its profile alone and nothing
ever re-agreed the shared node. The fix followed in one pass, and `nodeSpread`
went 8.02 m → 0.

The generalized version is now standing equipment: `RT_POKE_REPORT`,
`[bank-census]`, `RT_ELEVATION_MAP`, `RT_JUNCTION_DUMP`, `RT_FURNITURE_SVG`, the
sky chart, the frame ledger (ADR-0077), and the control channel (ADR-0078) that
lets an agent pose the camera and take the shot. **Censuses are computed from
probe grids independent of the thing being measured** — the bank census samples a
4 m grid that knows nothing about the roads or the CDLOD vertices — which is why
they can contradict the code that made them.

### 3. Measurements that refuted their own plan

The earthwork plan asserted that the displacement field `D` *"can only reduce"*
corner overshoot. The census said otherwise: LOD0 pokes were identical **cell for
cell**, 134 → 134. Following that non-result found a separate defect — 86 of the
134 are a building-pad stamp winning *inside* the road footprint and lifting
terrain above the deck.

That is the system working. The plan was wrong, the instrument was independent
enough to say so, and the disagreement was written into the commit message
instead of being smoothed over.

### 4. Named reds beat silent greens

Several gates are deliberately left failing, with the number, the cause, and what
is owed recorded at the point of failure. `block_grading_leaves_no_pits_between_roads`
reads 0.86 against a 0.55 gate — and the commit that caused it **predicted 0.86 in
its own message**, alongside the three variants measured before choosing, and the
trade it bought (banks beside roads 2041 → 950, worst bank 177% → 120%, the poke
gate red → green).

A red you predicted is a decision. A red you discover is a regression. Keeping
them distinguishable is most of what makes a test suite worth running.

### 5. Knowledge survives only in a form that can fail

`docs/knowledge-retention-plan.md` states the project's hardest-won lesson, and
it earned it the expensive way. Its own evidence table lists four cases where
correct knowledge — in `AGENTS.md`, in a decision doc *with measurements* — was
read and violated anyway. What survived instead were the winding convention and
the tower triangle budget, not because they were better documented but because
`lotmesh_winding_matches_the_engine_convention` **fails** when you break it.

> Knowledge survives only in a form that can fail. Everything else is a hope with
> a timestamp.

---

## What went wrong

### 1. The verification gap — the meta-debt

The single largest failure, and it is structural rather than any one bug.

- **No CI existed until day 66.** Three months of "it compiles here" was, for
  most of the project, a claim about one machine.
- **The Vulkan backend shipped Phases 0–3 having never been compiled.** No SDK
  was in the loop. The first real compile — in a Windows session — found a
  reserved-keyword shader bug, a wrong uniform field, and 43 broken logging
  calls. Code review had passed all of it.
- **Metal shaders are runtime-compiled**, so nothing validates them until someone
  runs the app on the right machine.
- **Linux CI was red on arrival** and stayed that way, which converts a build
  signal into noise. Worse, the list of known-reds decayed *in both directions*:
  one named red (`zoo_acute_four_way`) was fixed in passing and nobody noticed,
  another (`pedestrians_never_stand_inside_cars`) went green unremarked, and a
  new one appeared without being added.
- **"Written here, verified there"** was the operating mode for the editor for
  three months — the shell compiled on Linux while only a Mac could see a pixel.

The through-line: the project got very good at *proving things it could measure*
and had almost no defence against things nothing was measuring at all.

### 2. Documentation decays in every direction, silently

Found while writing this document, all of it live in the repo today:

- 21 of 83 ADRs carry **no date**.
- **Two different ADRs are both numbered 0066** — the typed EventBus
  (2026-07-06) and The Living City (2026-07-09). Nothing checks uniqueness of
  the one identifier the whole document set cross-references by.
- `docs/editor-app-plan.md` said "A2 skeleton done" for **2.5 months** while the
  editor grew a Vulkan viewport, two native docks and its own test target.
- The TECH_DEBT entry tracking Linux failures named **the wrong two tests**.
- `CLAUDE.md` — the file every session reads first — instructed `cmake -S . -B
  build`, which **cannot work** on any machine where `make` has run, because the
  Makefile owns `build/`. It fails with `not a CMake build directory`. That
  instruction sat there through the entire Linux round.

None of these were noticed by anyone reading. They were noticed by someone *acting
on them* and hitting the wall.

### 3. The prose problem

`AGENTS.md` is ~400 lines. Every line added lowers the odds that any single line
is honoured, and the retention plan documents four cases where it wasn't — including
a rule that named the exact symptom it was meant to prevent. Adding a longer
preflight is the same medicine that already failed.

### 4. Scale arrived before the ergonomics did

`shape_grammar.cpp`, `city_lots.cpp`, `level_loader.cpp` and `vulkan_renderer.cpp`
all carry dead locals and unused functions that the compiler flags on every build.
`make health` exists (ADR-0077) and a dead-code audit ran on 2026-08-15, so the
problem is known — but warnings that appear on every single build are warnings
nobody reads.

---

## What to change next, in priority order

1. **A per-platform expected-fail list the harness reads.** A red *not* on the
   list fails the build; a green *on* the list fails just as loudly. This single
   mechanism fixes the decay in both directions and has been owed since
   2026-08-05. It is the highest-leverage item on this list by a distance.
2. **Finish stage 2 of `docs/ci-plan.md`** — headless offscreen render plus
   cross-backend golden-image parity (lavapipe / Metal). Parity is currently a
   hand-maintained ledger in `docs/renderer-parity.md`; it should be a gate.
3. **Gate `docs/decisions.md` itself.** A ~20-line scanner that fails the build
   on a record with no date and on a duplicated ADR number — the two decay modes
   already present. Then the same for revisit triggers: a trigger whose condition
   has been met should be *asked*, not archived. ADR-0001's trigger was honoured
   by luck and attention, not by anything that would have complained.
4. **Turn the retention plan's ladder into actual gates.** It is queued work that
   has been correct since it was written; the longer it waits the more evidence it
   accumulates for its own thesis.
5. **Drive the warnings to zero and keep them there** — `-Werror` on a clean
   subset first, widening as files are cleaned.
6. **Verify the Metal half of the editor.** The risk has inverted: Metal is now
   the unrun path for the NSView + CAMetalLayer + retina viewport, and every
   editor feature landing from here adds to that debt.

## Open threads, honestly

- `drive_freeway_mainline_is_clear` has failed on Linux since CI began, is
  byte-identical before and after three months of road work, and is still
  unexplained. Suspected floating-point precision in freeway-weld geometry.
- **The fixture and the city disagree, and nobody knows why.** Building the
  earthwork field under the synthetic ring fixture makes its riser *worse*
  (1.51 m) while the shipped city's road-side banks fall by half. The fixture
  keeps its original contract until this is understood.
- The rim correction owed by ADR-0082 — a graded block should meet its streets
  exactly (ADR-0075's deferred "sampled patch").
- Building-pad stamps win inside road footprints and lift terrain above the deck:
  86 of 134 LOD0 pokes. Found by accident; not yet fixed.
- The star field, moon disc, light pollution and terrain horizon shadows are
  Vulkan-verified only; the Metal and WebGPU mirrors are uncompiled.
- ADR-0080's owed list: dog-leg stairs, real interior lighting, walker routing
  through doors.

---

## The one-sentence version

The project's method — instrument first, measure the trade, name the red, write
the revisit trigger — is genuinely good and produced a lot of correct work fast;
its failure mode is uniform and equally clear, which is that **anything no
instrument was pointed at drifted**, from a shader that had never been compiled
to a build command in the file every session reads.
