# Profiling & Code Health

How to find out where the engine is slow and where the code is patchy —
and how to prove a change helped. The tooling exists so that "measure before
optimizing" (design principle #3) is the easy path, on every surface the
engine runs on: macOS viewer, Qt editor, Vision Pro, Linux headless, web.

Three tools, three questions:

| Question | Tool | Where it works |
|---|---|---|
| *Is* the frame slow? Where does the budget go? Did my change help? | **Frame ledger** (`FrameStats`, ADR-0077): overlay panel, CSV capture, `tools/frame-report.py` | Every build, every host, always on |
| *Why* is this frame slow — which call, which thread, which spike? | **Tracy** (ADR-0068): `-DRT_ENABLE_PROFILER=ON` + the Tracy UI | Any build you can connect a UI to |
| Where is the code patchy — duplication, god-functions, debt? | **Code health** (`make health`, `tools/code-health.py`) | The repo itself, no build needed |

Use them in that order: the ledger tells you *whether* and *roughly where*,
Tracy tells you *why*, and the fix should leave the code healthier — not add
another layer (see "Engineering Ethos" in AGENTS.md).

## The frame ledger (always on)

Every build keeps a 240-frame ring of per-phase CPU times — update /
fixedUpdate / render / FPS-cap wait — plus the renderer's `RenderStats`
counters (draw calls, instances, triangles) per frame. Cost when nobody
looks: a few clock reads per frame.

**On screen:** backtick opens the debug overlay (ImGui builds); the
**Performance** header shows avg / p95 / max frame time, the per-phase
split, a frame-time graph scaled to the FPS budget, and a capture button.
p95 is the number to watch — avg hides spikes.

**To a file (any host, no UI needed):**

```bash
RT_FRAME_STATS=capture.csv ./build/viewer            # record every frame from boot
RT_FRAME_STATS_LOG=5 ./build/viewer                  # a summary log line every 5 s
```

One CSV row per frame: `frame, total_ms, update_ms, fixed_ms, render_ms,
wait_ms, host_delta_ms, fixed_steps, draw_calls, instances, triangles`.
The overlay's "Start CSV capture" button writes `frame-capture.csv` in the
working directory. `Application::stats()` exposes the same data to hosts
without ImGui (the visionOS panel reads it the way it reads the settings
seam).

**Reading a capture:**

```bash
python3 tools/frame-report.py capture.csv                       # -> capture.html
python3 tools/frame-report.py after.csv --compare before.csv    # before/after
python3 tools/frame-report.py capture.csv --budget-fps 90       # Vision Pro budget
```

The report (self-contained HTML, stdlib-only script) shows the frame-time
series with spike peaks, a stacked where-does-the-frame-go chart, a
distribution histogram against the 30/60/90 fps lines, and a summary table.
`--compare` overlays a baseline in grey — the honest way to claim "this made
it faster". Keep the *before* capture from the same scene, camera, and
duration as the *after*.

**Interpreting phases:** `wait` is the FPS-cap sleep — headroom, not cost.
`render` is CPU submit + present; a fat `render` with low draw calls usually
means the CPU is *waiting* on the GPU — the ledger's known blind spot is GPU
time, so confirm with Xcode's GPU capture (Metal) before optimizing CPU-side.
The gap between the phase stack and total is unattributed engine time; if it
grows, a bracket is missing — add one rather than guessing.

## Tracy (the deep dive)

```bash
git submodule update --init third_party/tracy
cmake -S . -B build -DRT_ENABLE_PROFILER=ON && cmake --build build
./build/viewer     # then connect the Tracy UI (v0.13.1) from any machine on the LAN
```

`TRACY_ON_DEMAND` is set: an instrumented build records only while a UI is
attached, so the flag is safe to leave on for a whole session. Zones live
permanently in the code behind `RT_PROFILE_*` (`src/profile.h`, ADR-0068) and
compile to nothing otherwise. Current coverage: the frame phases
(`application.cpp`), JobSystem workers, physics step, render submit, script
tick, terrain LOD, level load + vegetation, SDF polygonize, mesh upload,
kd-tree build, path-tracer scanlines — plus `RT_PROFILE_PLOT` graphs of draw
calls / instances / triangles. Coverage grows with need: when a ledger phase
is fat and Tracy shows an unattributed span, add a zone there and leave it.

Headless captures work too (`tracy-capture` from the Tracy repo saves a
`.tracy` file without the UI) — useful on the Linux box for job-system work.

## Vision Pro

The budget is 11.1 ms (90 Hz) and the wearer can't see your desktop, which is
why the ledger exists:

- **Glanceable numbers:** the visionOS settings panel can read
  `Application::stats()` (`summarize()` / `lastFrame()`) exactly as it reads
  the settings seam. Until that panel row exists, use the log line.
- **Console-only:** launch with `RT_FRAME_STATS_LOG=5` (device: an Xcode
  scheme environment variable; simulator: `SIMCTL_CHILD_RT_FRAME_STATS_LOG=5
  xcrun simctl launch ...`, the same prefix trick as `RT_DEBUG_VIEW`). The
  summary line lands in the `[vision]`-adjacent app log — remember
  `devicectl --console` misses os_log; use Xcode's console.
- **CSV on device:** the bundle is read-only — point `RT_FRAME_STATS` at the
  Documents directory (where `settings.json` already goes), then pull the
  file via Xcode's container download and run `frame-report.py --budget-fps 90`.
- **Tracy on device** is a network capture from the paired Mac; the client is
  wired into the build via the normal CMake option but is *compile-unverified*
  for the visionOS toolchain — treat the first `-DRT_ENABLE_PROFILER=ON`
  visionOS configure as an experiment. For GPU time, Xcode's Metal debugger /
  Instruments remains the tool (the ledger and Tracy are CPU-side; ADR-0077's
  revisit trigger covers GPU zones).
- **The simulator's timings are fiction** (see `src/visionos_app/AGENTS.md`)
  — capture on device before believing any number.

## Code health (finding the patches)

```bash
make health                          # scan src/, human-readable report
python3 tools/code-health.py src tools --json health.json   # snapshot
```

Four detectors, all heuristic, all stdlib-only:

- **Duplicate blocks** — runs of ≥ 8 significant lines repeated verbatim,
  clustered so one copy-pasted region is one finding. This is the "second
  copy drifts" hazard: the entity pack/unpack split documented in
  TECH_DEBT.md is the canonical case.
- **Long functions** (≥ 100 body lines) — seam candidates, not crimes;
  `LevelLoader::load` tops the list for a reason.
- **Debt markers** — TODO/FIXME/HACK/"workaround"/"for now" density: where
  past sessions *knowingly* patched.
- **Include fan-in** — headers included by many TUs, weighted by their size:
  rebuild-cost and coupling hotspots.

It is a **finder, not a gate** — no CI failure, no score to game. The
workflow: run it when picking cleanup work or reviewing a grown subsystem;
promote real findings into TECH_DEBT.md or the decisions.md register with
context; commit a `--json` snapshot before and after a cleanup so the trend
is visible in the diff. When a finding is a *deliberate* duplication (the
renderer backends repeat structure by design), say so where you found it
rather than suppressing the tool.

## The loop

1. **Capture** before touching anything (ledger CSV, and a Tracy session if
   the ledger already shows where).
2. **Diagnose** until you can say *why* in one sentence naming a mechanism —
   "per-instance cull re-allocates a `std::vector<Mat4>` per group per frame",
   not "culling is slow".
3. **Fix the cause** — and if the cause is patch-shaped code, fix the shape
   (`code-health` findings are often the same places the ledger flags).
4. **Prove it**: `frame-report.py after.csv --compare before.csv` in the PR.
5. **Gate it** where possible — a zone left in place, a counter on the HUD,
   a register row for what a device still needs to verify.
