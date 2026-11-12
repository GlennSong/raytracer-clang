# Driving model — task briefs and how to hand them out

The plan is `docs/driving-model-plan.md`. The design is `docs/driving-model-design.md` ("the design" below). Its
section 17 lists every task; section 17.1 grades them. This folder holds a brief for each **hard** task: the ones where
the design leaves a judgement to make, or that work inside the 8,500-line `city_sim.cpp`. A brief makes those
judgements so the implementer doesn't have to, and names the traps.

Plain and simple tasks have no brief: their row in section 17 plus the sections it points to is the whole job.

## The briefs

| Brief | Task | Needs first |
|---|---|---|
| `A2-seam.md` | Separate everything that reads the old car state | — |
| `B2-connectors.md` | A drivable path for every turn at every junction | A3, B1 |
| `B3-crossing-points.md` | Where paths through a junction cross, join and split | B2 |
| `B10-planner-lanes.md` | The planner places bus lanes, bike lanes and parking | B9 |
| `C6-tick-and-following.md` | The three-step tick, speed and following | A1, A3, C2, C4, C5 |
| `C12-threads.md` | The tick on several threads, same result | C6 and everything it runs |
| `D1-junction-plan.md` | Who gives way to whom, worked out at load | B3, C3 |
| `D3-give-way.md` | Announcements, the give-way decision, patience | D1, C6 |
| `D4-inside-the-junction.md` | First in goes first; waiting inside for a far turn | D3 |
| `D5-room-beyond.md` | Don't block the box, as a claim | D4 |
| `E1-lane-changes.md` | Lane changes, letting in, missed turns | D6 |
| `E3a-reversing.md` | Paths driven backwards | E3 |
| `E7-levels-of-detail.md` | Middle distance, out of sight, arriving in an area | E1 |
| `E8-physical-cars.md` | Physics bodies steering along the path | E7, A2 |
| `E9-cities-switch.md` | Cities move to the new model; the old one is deleted | all |

## Which model does what

| Grade (design 17.1) | Who implements | Who reviews |
|---|---|---|
| Hard | Opus, at high effort, given the brief | The strongest model available (Fable), then Glenn where the brief says "show Glenn" |
| Plain | Opus, given the task row and its design sections | A different model from the one that wrote it |
| Simple | Sonnet (Haiku for pure data entry such as catalogue rows) | Opus, quickly |

Sourcing a rule-set number (finding it in the Uniform Vehicle Code, the MUTCD or the Highway Capacity Manual) is
graded simple but must be reviewed by a person or a stronger model: a wrong number looks exactly like a right one.

## What every hand-out contains

One task, one implementer, one message. The message has five parts and nothing else:

1. **The task row** from design section 17 (its "done when" is the contract).
2. **The brief**, if there is one.
3. **The design sections to read**, by number. Never "read the design": it is about 1,400 lines and most of it is
   not about the task in hand.
4. **The standing rules** below, pasted in.
5. **Where to work:** the folder and branch, and the build and test commands.

The implementer reports back with: what changed (files), the test output, anything it could not do and why, and
anything in the brief or design it found to be wrong. That last part matters: a brief that turns out wrong gets
fixed here, not worked round in code.

Before starting, the implementer checks the hand-out message against the brief and the GitHub issue; where they
differ, the brief and the issue win, and the difference is reported.

## Standing rules (paste into every hand-out)

From `AGENTS.md`, which the implementer should also read:

- **No kind names in C++.** No sign, vehicle kind, country, or kind of junction control named in logic. They are
  rows in Lua. If a test can only pass by naming one, stop and report.
- **No escape hatches.** No timer that unsticks a car, no teleport, no "push through after N seconds". If cars get
  stuck, that is a finding to report with the wait-for graph, not a thing to patch.
- **Near turn / far turn, kerb side / offside.** Never left and right in rules or driving code.
- **Numbers live in data** (the rule set, the vehicle data) or in one named constant. No bare numbers in logic.
- **The rule language does not grow.** If a rule can't be written, add a fact.
- **Same result every run.** No shared random generator; no iteration over an unordered map; nothing that depends
  on thread timing.
- **The old driving code is not changed**, except by task A2 (which changes no behaviour) and E9.
- No `auto` outside lambdas. One header and one `.cpp` per module. Faces through `MeshBuilder::emitQuad`.
- Metres and seconds in C++. The rule set converts from mph when it loads.
- A `CREDITS.md` line for every outside source, in the same change. An ADR for each decision that is hard to undo.
- Documents, comments and reports in plain language: say what a thing does before what it's called.
- Edit source with the editor tools, not shell string replacement.
- One working branch. Do not create branches. Do not stash. Do not commit unless told to.
- A task that changes what the road builder produces needs Glenn's OK before the rebuild (a lanes tag bump), and
  such tasks land together.

## The review, for every task

The reviewer checks, in this order:

1. The "done when" is met, by running the tests, not by reading the report.
2. A search of the changed files for kind names, timers, bare numbers, "left"/"right", and unordered iteration.
3. The lab's measurements did not get worse (once task C10 exists).
4. For hard tasks: each "decision already made" in the brief was followed, or the report says why not.

## How to run it in practice

- **One at a time on the one branch** is the default. The five lines of work in design 17.1 touch different files, so
  their tasks can be taken in any order.
- **A session can hand a task to another model** as a sub-agent, choosing the model per task, then review the result
  itself. That is the simplest way to follow the table above: the session keeps the thread, the sub-agent does one
  task and reports.
- **Several at once** needs Glenn's say, because it needs separate work folders.
- **Each task should have a GitHub issue** under its phase's issue (#226–#233, #222, #224), carrying the same five
  parts, so the hand-out and the record are the same text.
