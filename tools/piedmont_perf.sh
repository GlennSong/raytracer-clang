#!/usr/bin/env bash
# Piedmont performance harness (8km-city plan P8.2).
#
# Every misleading perf number this project produced came from comparing runs
# that were not in the same STATE, so this script makes state explicit:
#
#   1. WAIT FOR THE CITY. The level builds for minutes; a fixed sleep once
#      measured a beautiful 25.5 fps with no traffic in the world at all.
#      Measurement starts only after the "[citysim] fleet:" line proves the
#      city (and its agents) exist.
#   2. PIN THE CAMERA. The viewer persists its camera on exit, so a repeated
#      capture silently frames somewhere else — and GPU cost is viewpoint
#      bound. The camera is written fresh for every run.
#   3. SAMPLE A FIXED SIM-HOUR WINDOW. Wall time is not comparable: a slower
#      run drops more clock backlog and so reaches a given hour later, and the
#      workload swings hugely across the day (everyone parked downtown vs
#      everyone spread across the network). Runs are sampled between
#      RT_PERF_HOUR_FROM and RT_PERF_HOUR_TO of SIM time, so two runs cover
#      the same slice of the city's day.
#   4. COOL DOWN FIRST. Measured on a fanless machine: after ~25 minutes of
#      back-to-back runs the SAME build on the SAME census read 179 ms/frame
#      where it had read 99 ms — a 1.8x thermal drift, larger than most changes
#      being tested. A sweep of variants run one after another therefore says
#      nothing about the variants. RT_PERF_COOLDOWN idles before launching, and
#      the run PRINTS its cooldown so a comparison made without one is visibly
#      suspect. Re-run your baseline last: if it no longer matches the first
#      baseline, the whole batch is thermal and must be redone.
#   5. PRINT THE CENSUS NEXT TO THE FRAMERATE. Workload varies with the sim
#      hour (rush hour costs more than mid-afternoon). A comparison whose
#      active/far/moving counts disagree is INVALID, not interesting — the
#      summary puts them side by side so that is obvious.
#
# Usage:
#   tools/piedmont_perf.sh <label> [key=value ...]
#
# key=value pairs override citysim knobs in the level for that run only (the
# level is restored afterwards), e.g.:
#   tools/piedmont_perf.sh baseline
#   tools/piedmont_perf.sh rate30 localHz=30 adaptiveRate=false
#
# Output: /tmp/piedmont_perf/<label>.log plus a summary on stdout.

set -uo pipefail
cd "$(dirname "$0")/.."

LABEL="${1:?usage: piedmont_perf.sh <label> [key=value ...]}"
shift || true

LEVEL="assets/levels/piedmont.json"
VIEWER="./build/viewer"
OUT="/tmp/piedmont_perf"
LOG="$OUT/$LABEL.log"
BUILD_WAIT="${RT_PERF_BUILD_WAIT:-900}"   # seconds to allow for the city build
# The window is RELATIVE to whatever hour the city comes up at (the build
# takes minutes, during which the clock runs).
#
# It is expressed in SIM hours but sized from REAL seconds, because the two are
# only related by the level's own hoursPerSecond — and that varies by world
# size. This used to be hardcoded for 0.05 h/s; on a level authored at 0.004 the
# same numbers ask for a window 12.5x longer in real time than the wall-clock
# cap allows, so the script silently sampled the wrong slice and said nothing.
SETTLE_REAL="${RT_PERF_SETTLE_REAL:-12}"  # real seconds to let the frame settle
SAMPLE_REAL="${RT_PERF_SAMPLE_REAL:-60}"  # real seconds of frames to average
HPS=$(python3 -c "
import json,sys
d=json.load(open('$LEVEL')).get('citysim',{})
print(d.get('hoursPerSecond', 0.05))")
HOUR_LEAD="${RT_PERF_HOUR_LEAD:-$(awk "BEGIN{print $SETTLE_REAL * $HPS}")}"
HOUR_SPAN="${RT_PERF_HOUR_SPAN:-$(awk "BEGIN{print $SAMPLE_REAL * $HPS}")}"
# Wall-clock cap: generous enough for the window we just asked for, even if the
# frame is slow (the sim falls behind real time when it drops backlog).
MEASURE="${RT_PERF_MEASURE:-$(awk "BEGIN{print int(($SETTLE_REAL + $SAMPLE_REAL) * 4)}")}"

# Thermal settle (see trap 4). Default 0 for a single ad-hoc run; set it for a
# SWEEP, where every run after the first is measuring a hotter machine.
COOLDOWN="${RT_PERF_COOLDOWN:-0}"
if [ "$COOLDOWN" -gt 0 ]; then
  echo "[perf] $LABEL: cooling down ${COOLDOWN}s before measuring..."
  sleep "$COOLDOWN"
fi

mkdir -p "$OUT"
[ -x "$VIEWER" ] || { echo "error: $VIEWER not built"; exit 1; }

LEVEL_BAK="$(mktemp)"; SETTINGS_BAK="$(mktemp)"
cp "$LEVEL" "$LEVEL_BAK"; cp settings.json "$SETTINGS_BAK"
restore() {
  cp "$LEVEL_BAK" "$LEVEL"; cp "$SETTINGS_BAK" settings.json
  rm -f "$LEVEL_BAK" "$SETTINGS_BAK"
  [ -n "${PID:-}" ] && kill "$PID" 2>/dev/null
  rm -f "$LEVEL.cameras.json"
}
trap restore EXIT

# --- knob overrides + a FIXED camera (downtown, street level) ----------------
python3 - "$LEVEL" "$@" <<'PY'
import json, os, sys
lvl = sys.argv[1]
d = json.load(open(lvl))
cs = d.setdefault("citysim", {})
for kv in sys.argv[2:]:
    k, _, v = kv.partition("=")
    if v.lower() in ("true", "false"): val = v.lower() == "true"
    else:
        try: val = float(v)
        except ValueError: val = v
    cs[k] = val
json.dump(d, open(lvl, "w"), indent=1, sort_keys=True)
# THE CAMERA IS AN INPUT, NOT A CONSTANT. A single pinned downtown viewpoint
# measured towers and almost no sky, so it could not see the two things that
# actually cost: residential foliage and the cloud march. RT_PERF_CAM names one;
# RT_PERF_EYE="x,y,z[,yaw,pitch]" overrides outright. Cross-run numbers are only
# comparable within the SAME camera — the label carries it so runs cannot be
# mixed up after the fact.
CAMS = {
    # downtown street level: towers, no sky. The original, kept bit-identical
    # so every number recorded before this change stays comparable.
    "downtown":    (1030.0, 6.0, 858.0, 0.0, -3.0),
    # residential street level: this is where the 12 fps lives (trees, low
    # houses, open sky). PLACEHOLDER — must be replaced with the centre of the
    # densest LayerFoliage cluster from an RT_DUMP_DRAWS audit. Until it is,
    # this camera measures an arbitrary point and its numbers mean nothing.
    "residential": (1631.0, 8.0, 2109.0, 0.0, -3.0),   # UNVERIFIED
    # up and tilted at the horizon: mostly sky, so the cloud march is actually
    # in frame. A street camera reports the cloud cost as ~0 and means nothing.
    "sky":         (1030.0, 90.0, 858.0, 0.0, 12.0),
}
name = os.environ.get("RT_PERF_CAM", "downtown")
if name not in CAMS and not os.environ.get("RT_PERF_EYE"):
    sys.exit("unknown RT_PERF_CAM %r (have: %s)" % (name, ", ".join(sorted(CAMS))))
ex, ey, ez, yaw, pitch = CAMS.get(name, CAMS["downtown"])
if os.environ.get("RT_PERF_EYE"):
    parts = [float(v) for v in os.environ["RT_PERF_EYE"].split(",")]
    ex, ey, ez = parts[0], parts[1], parts[2]
    if len(parts) > 3: yaw = parts[3]
    if len(parts) > 4: pitch = parts[4]
s = json.load(open("settings.json"))
s.update({"flyEyeX": ex, "flyEyeY": ey, "flyEyeZ": ez,
          "flyPitch": pitch, "flyYaw": yaw, "flyOrtho": False,
          "cameraMode": "fly"})
json.dump(s, open("settings.json", "w"), indent=2)
PY

echo "[perf] $LABEL: building the city (up to ${BUILD_WAIT}s)..."
RT_DUMP_STATS=1 RT_GPU_TIME=1 "$VIEWER" "$LEVEL" > "$LOG" 2>&1 &
PID=$!

# 1. WAIT FOR THE CITY — the fleet line is the proof that agents exist.
built=0
for _ in $(seq 1 "$BUILD_WAIT"); do
  kill -0 "$PID" 2>/dev/null || { echo "[perf] viewer exited during build"; exit 1; }
  if grep -q "\[citysim\] fleet:" "$LOG" 2>/dev/null; then built=1; break; fi
  sleep 1
done
[ "$built" = 1 ] || { echo "[perf] TIMEOUT: city never built — result would be meaningless"; exit 1; }

# 2. WAIT FOR THE SIM HOUR, then sample until it leaves the window. This is
#    what makes two runs the same experiment rather than two different days.
simhour() { grep -o "| hour [0-9.]*" "$LOG" 2>/dev/null | tail -1 | awk '{print $3}'; }
# The first stats line only appears after 300 fixed steps, and the frames right
# after the city comes up are the worst of the run (mesh uploads, terrain
# streaming) — so this wait has to be generous. A hardcoded 60 s silently failed
# whole runs on a slow frame: the city was up, nothing had reported yet, and the
# script called it INVALID.
HOUR_WAIT="${RT_PERF_HOUR_WAIT:-240}"
H0=""
for _ in $(seq 1 "$HOUR_WAIT"); do H0=$(simhour); [ -n "$H0" ] && break; sleep 1; done
[ -n "$H0" ] || { echo "[perf] the sim never reported an hour in ${HOUR_WAIT}s — INVALID (raise RT_PERF_HOUR_WAIT)"; exit 1; }
FROM=$(awk "BEGIN{print $H0 + $HOUR_LEAD}")
TO=$(awk "BEGIN{print $FROM + $HOUR_SPAN}")
echo "[perf] $LABEL: city up at hour $H0; sampling sim hours $FROM -> $TO"
for _ in $(seq 1 "$MEASURE"); do
  kill -0 "$PID" 2>/dev/null || break
  h=$(simhour); [ -n "$h" ] && awk "BEGIN{exit !($h >= $FROM)}" && break
  sleep 1
done
MARK=$(wc -l < "$LOG" | tr -d " ")   # macOS pads wc output
for _ in $(seq 1 "$MEASURE"); do
  kill -0 "$PID" 2>/dev/null || break
  h=$(simhour); [ -n "$h" ] && awk "BEGIN{exit !($h >= $TO)}" && break
  sleep 1
done
kill "$PID" 2>/dev/null; wait "$PID" 2>/dev/null
PID=""

# 3. Summarise: census FIRST, so an invalid comparison is obvious.
tail -n "+$MARK" "$LOG" > "$OUT/$LABEL.window"
python3 - "$OUT/$LABEL.window" "$LABEL" <<'PY'
import re, sys, statistics as st
rows = open(sys.argv[1]).read().splitlines()
def nums(pat, cast=float):
    out = []
    for l in rows:
        m = re.search(pat, l)
        if m: out.append(cast(m.group(1)))
    return out
# Tolerant of extra census fields: the line has grown dormant/rolling/asleep
# counters and will grow more. Each field is matched on its own name so a new
# one can never silently blank the whole census again.
def field(name):
    return [int(m) for m in re.findall(name + r"[^0-9]*(\d+)", "\n".join(rows))]
census = list(zip(field(r"active\(K/P\)"), field(r"far\(V\)"),
                  field(r"moving")))
rolling  = field(r"rolling")
dormantN = field(r"dormant\(D\)")
asleepN  = field(r"asleep")
hours = nums(r"\| hour ([\d.]+)")
fps   = nums(r"\[stats\] ([\d.]+) fps")
tris  = nums(r"tris ([\d.]+)M")
draws = nums(r"draws (\d+)")
parked = nums(r"parked cars drawn (\d+)")
gpu   = nums(r"avg frame ([\d.]+) ms")
fixed = nums(r"fixedUpdate ([\d.]+) ms/frame")
steps = nums(r"over ([\d.]+) steps/frame")
sim   = nums(r"citysim step ([\d.]+) ms")
med = lambda v: round(st.median(v), 2) if v else None
print(f"\n=== {sys.argv[2]} ===")
import os
cd = os.environ.get("RT_PERF_COOLDOWN", "0")
if cd == "0":
    print("  WARNING   no cooldown — only comparable against a run made in the")
    print("            same thermal state (see trap 4 in this script's header)")
if census:
    a = med([c[0] for c in census]); f = med([c[1] for c in census]); m = med([c[2] for c in census])
    print(f"  census    active(K/P) {a}   far(V) {f}   moving {m}   <-- compare THIS first")
    if rolling:
        print(f"            rolling {med(rolling)} of {m} moving   "
              f"<-- the gap is cars stopped in traffic")
    if dormantN or asleepN:
        print(f"            dormant(D) {med(dormantN)}   asleep {med(asleepN)}")
else:
    print("  census    (none — the sim never reported; treat everything below as INVALID)")
print(f"  frame     {med(fps)} fps")
print(f"  gpu       {med(gpu)} ms")
print(f"  fixed     {med(fixed)} ms/frame over {med(steps)} steps/frame")
print(f"  sim step  {med(sim)} ms")
print(f"  triangles {med(tris)}M   draws {med(draws)}")
if parked:
    print(f"  parked cars drawn {med(parked)}")
if hours:
    print(f"  sim hour  {min(hours):.2f} -> {max(hours):.2f}   "
          f"<-- two runs are only comparable over the SAME slice")
PY
