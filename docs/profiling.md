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

**Every capture records its own conditions.** The first line of a CSV is a
`# framebuffer=… megapixels=… ssao=… ssr=…` header, and the report shows it
under *Capture conditions*. This exists because two captures of the same scene
came out 2× apart and nothing in the data could say why. `--compare` checks
the two headers and **refuses to let a mismatch pass quietly** — different
resolution or pass config means the delta you are looking at may be that
difference, not your change.

**A repeatable capture protocol** (do this the same way every time, or the
numbers aren't comparable):

1. Same window size, same display, and **plugged in** — laptop GPUs throttle
   hard on battery, which alone can halve the frame rate.
2. Let it run ~10 seconds before you start caring; the first frames are level
   load, shader compilation, and probe bake.
3. Capture 60–90 seconds of *representative* play — the same route and the
   same actions each time.
4. Quit cleanly (don't kill the process) so the capture is flushed and closed.
5. Check the *Capture conditions* line matches your previous run before
   comparing anything.

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

The report (self-contained HTML, stdlib-only script) opens with a plain-language
verdict — which bound you are in and what to do next — then:

- **Chart 1, frame time over the session.** Hover for one frame's exact
  breakdown, drag across to zoom (the y-scale follows the zoom), *Jump to
  worst frame* for the biggest hitch. Scaled to the visible 98th percentile so
  a single load spike can't flatten everything else; clipped frames are
  counted beside the buttons.
- **Chart 2, where the frame goes.** Stacked phases, scaled to typical frames.
  Click a legend colour to hide that layer — the fastest way to see what's
  underneath the dominant one.
- **Chart 4, spike anatomy.** The slowest 1% of frames against typical ones,
  phase by phase, sorted by growth. **The top row is what a spike actually
  is** — the summary table can't show this, because an average frame and a
  spike often differ in *which* phase grew. Plus the ten worst frames with
  their full breakdown; early frame numbers usually mean load, upload, or
  shader compilation rather than steady-state cost.
- **Chart 3**, the distribution against the 30/60/90 fps lines.

`--compare` overlays a baseline in grey — the honest way to claim "this made
it faster". Keep the *before* capture from the same scene, camera, and
duration as the *after*.

**Steady-state cost and hitching are different problems.** If p99 is more than
about 3× the median, the average is being dragged by a minority of very slow
frames and the report says so: fix the hitch and the average moves without the
typical frame changing at all. Chart 4 tells you which of the two you have.

**Interpreting phases.** `wait` is the FPS-cap sleep — headroom, not cost.
The gap between the phase stack and the total is unattributed engine time; if
it grows, a bracket is missing — add one rather than guessing. `render` splits
three ways, and the split is the whole diagnosis:

| Phase | What it is | Fat means |
|---|---|---|
| `acquire` | `beginFrame` — getting a drawable to render into. **Blocks** when the GPU is behind or the vsync deadline was missed. | **GPU-bound.** The CPU is *idle* here. Optimising C++ changes nothing; the cost is in shaders/passes. |
| `encode` | The states' `render()` hooks: world walk, culling, describing draws. | CPU cost that scales with scene size — cull, batch, or reduce entity count. |
| `submit` | `endFrame` — building every pass's command buffers, then commit. | The pass graph's own CPU cost; usually means too many passes/encoders. |

`poll` (window/OS event pump) and `dispatch` (event-bus drain + end-of-frame
state swap, where a level load runs) cover the parts of the frame that used to
fall between the named phases. Anything still uncovered shows as
**unattributed** — a derived band in chart 2 and a row in chart 4. Watch it:
a real capture once hid a 307 ms stall there, 90% of the worst frame, simply
because no bracket claimed that code. A growing unattributed band means *add a
bracket*, not *guess*.

`gpu_ms` is the GPU timeline of the frame's command buffer (Metal only today;
other backends report 0). **Read it as an upper bound, not a cost.** It lags a
frame or two, and — more importantly — the timed command buffer also carries
`presentDrawable`, so its window includes waiting on the display for a free
drawable. The giveaway is `gpu_ms` exceeding the frame's own duration, which is
impossible for pure work; the report detects that and says so. Timing a buffer
that does not present (split submission, or a scheduled-handler present) is the
fix, and it needs a device to verify — until then Xcode's GPU capture is the
trustworthy per-pass number.

**Vsync quantisation.** If frame times cluster on a multiple of the refresh
interval (33.3 ms on a 60 Hz display = two intervals), the engine missed the
deadline and is waiting for the next scanout, so it runs at a locked lower
rate. The measured frame time then includes that waiting and the true cost is
somewhere between one interval and the observed value — `gpu_ms` pins it down.
`frame-report.py` detects this and says so.

**Once you know it is GPU-bound**, rank the passes. The unattended way — one
command, no interaction, no way to get the procedure wrong:

```bash
RT_PASS_SWEEP=passes.csv ./build/viewer assets/levels/arena.json
```

It resizes the window through four sizes (100%, 75%, 50%, 35% of what you
launched with), measures every pass configuration at each, writes `passes.csv`,
prints a per-size summary, restores your window, and quits. Several sizes
matter because a pass whose cost scales with area is the one worth attacking
at high resolution — the sweep shows that scaling directly instead of leaving
it to be inferred.

The interactive equivalent is the Performance panel's **Rank post passes**
button. It **turns presentation sync off first**
— this is the whole trick — then holds each configuration (all on, then SSAO /
SSR / bloom disabled in turn) for two seconds, discards the frames while the
pipeline settles, takes the *median* of the rest, restores your settings and
your vsync, and prints what each pass costs.

Why sync must be off: with vsync on, frame time is **quantised to the refresh
interval**, so a genuine 4 ms saving reads as `0.00` — the frame just waits the
same. Worse, a frame sitting *on* the boundary flips between one and two
intervals, which reads as a pass costing a *negative* amount. A real run
produced `17.11 / 33.27 / 17.81 / 33.20` — pure boundary flipping, no pass
information at all.

The tool refuses to report a ranking it can't trust: if any pass appears to
cost negative time, or the medians look quantised to 60/90/120 Hz, or the
backend can't disable sync (a compositor-driven surface owns its pacing), it
prints the raw times marked **UNRELIABLE** and points at a GPU capture instead.

Separately, halving the window is the pixel-bound test: a hard drop means the
cost scales with pixels, and a Retina framebuffer is 4× its logical size.

For per-pass GPU numbers beyond that, use Xcode's GPU capture (Metal) —
per-pass timestamps inside the engine are the ADR-0077 follow-up, not built.

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
