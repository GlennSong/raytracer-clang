# C12 — The tick on several threads, same result

**Read first:** design 7.1. Brief C6. `src/job_system.h`.

## The goal

The plan and move steps of the tick run on several threads through the engine's job system, and the result is
identical, bit for bit, to one thread.

## Decisions already made

| Step | How it is split | What it may write |
|---|---|---|
| Announce | by car, in ranges | each car's own announcements, into a buffer per thread |
| Plan | by car, in ranges | only that car's own wants and its own trace |
| Settle claims | one job per junction, one per section | only that junction's or section's grants |
| Move | by car, in ranges | only that car's own new state; path hops go into a buffer per thread |
| Tidy | one job per path, after merging the hop buffers | that path's list |

- **Buffers per thread are merged in a fixed order** (by crossing then time then id for announcements; by path then
  position then id for hops; by car id for events). The merge is what makes thread count not matter.
- **Nothing shared is filled in lazily during the plan step.** Any lookup that caches its answers is filled before
  the step or not cached.
- **Sums of floating-point numbers** (the learned travel times, the measurements) are collected per thread and added
  up in a fixed order afterwards. Adding the same numbers in a different order gives a different last digit, and a
  different last digit is a different run.
- **A junction and the sections next to it** can both want to grant something about the same car in one tick (enter
  the junction; change lane just before it). Settle junctions first, then sections, each reading the other's grants.

## Traps

- Two cars' state next to each other in memory being written by different threads is slow, not wrong. Split work in
  ranges of a few hundred cars, not one at a time.
- The trace ring belongs to its car; the wait-for graph is built after the tick, on one thread.
- The host's own registered functions (announcers, design 4) run on one thread before the plan step. Don't call
  them from jobs.

## Done when

- The state hash is identical at every tick of a ten-minute lab run with 1, 2, 4 and 8 threads.
- The same under a thread-checking build, if one is set up for this project, with no reports.
- The cost per 1,000 drivers per tick, near and middle, at each thread count, is written in the ADR.

## Show Glenn

The cost table.
