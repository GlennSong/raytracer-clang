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
#   3. PRINT THE CENSUS NEXT TO THE FRAMERATE. Workload varies with the sim
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
MEASURE="${RT_PERF_MEASURE:-90}"          # seconds of steady-state sampling

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
import json, sys
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
s = json.load(open("settings.json"))
s.update({"flyEyeX": 1030.0, "flyEyeY": 6.0, "flyEyeZ": 858.0,
          "flyPitch": -3.0, "flyYaw": 0.0, "flyOrtho": False,
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

# 2. Discard the build-transient frames, then sample steady state.
echo "[perf] $LABEL: city up; settling, then sampling ${MEASURE}s..."
sleep 15
MARK=$(wc -l < "$LOG" | tr -d " ")   # macOS pads wc output
sleep "$MEASURE"
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
census = [ (int(a), int(b), int(c)) for a, b, c in
           re.findall(r"active\(K/P\) (\d+), far\(V\) (\d+), moving (\d+)",
                      "\n".join(rows)) ]
fps   = nums(r"\[stats\] ([\d.]+) fps")
tris  = nums(r"tris ([\d.]+)M")
gpu   = nums(r"avg frame ([\d.]+) ms")
fixed = nums(r"fixedUpdate ([\d.]+) ms/frame")
steps = nums(r"over ([\d.]+) steps/frame")
sim   = nums(r"citysim step ([\d.]+) ms")
med = lambda v: round(st.median(v), 2) if v else None
print(f"\n=== {sys.argv[2]} ===")
if census:
    a = med([c[0] for c in census]); f = med([c[1] for c in census]); m = med([c[2] for c in census])
    print(f"  census    active(K/P) {a}   far(V) {f}   moving {m}   <-- compare THIS first")
else:
    print("  census    (none — the sim never reported; treat everything below as INVALID)")
print(f"  frame     {med(fps)} fps")
print(f"  gpu       {med(gpu)} ms")
print(f"  fixed     {med(fixed)} ms/frame over {med(steps)} steps/frame")
print(f"  sim step  {med(sim)} ms")
print(f"  triangles {med(tris)}M")
PY
