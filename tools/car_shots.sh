#!/usr/bin/env bash
# Car-lab visual acceptance battery.
#
# Drives the viewer headlessly via RT_FRAME_DUMP (see metal_renderer.mm) to capture a fixed
# set of labeled views of the car lab, so any change to the vehicle generator can be eyeballed
# in one command. Directly modelled on tools/road_shots.sh.
#
# The fly camera is positioned by writing flyEye*/flyYaw/flyPitch into settings.json — the
# viewer REWRITES that file on exit, so we back it up and restore it on any exit path.
#
# Camera convention (fly_camera_controller.cpp): forward = (sin(yaw)cos(pitch), sin(pitch),
# -cos(yaw)cos(pitch)); yaw=pitch=0 looks down -Z. We compute yaw/pitch from an eye->target.
#
# NOTE --edit, not --play. In play mode the camera ORBITS THE PLAYER (flyEye* is only read
# when playFromHere is set), so shots cannot be framed on a parked car; edit mode drives the
# fly camera directly. Edit mode draws the toolbar over the top-left of the frame, so we set
# hideEditorUi, which suppresses the editor chrome for exactly this purpose.
#
# The car sits at the origin facing +Z, wheels on the pad, roughly 4.6 x 1.8 x 1.45 m.
#
# Usage:   tools/car_shots.sh [output_dir]   (default: /tmp/car_shots)
# Requires a built ./build/viewer (cmake --build build --target viewer).

set -euo pipefail
cd "$(dirname "$0")/.."

OUT="${1:-/tmp/car_shots}"
LEVEL="assets/levels/car_lab.json"
VIEWER="./build/viewer"
SETTINGS="settings.json"
BACKUP="$(mktemp)"

mkdir -p "$OUT"
[ -x "$VIEWER" ] || { echo "error: $VIEWER not built (cmake --build build --target viewer)"; exit 1; }
cp "$SETTINGS" "$BACKUP"
restore() { cp "$BACKUP" "$SETTINGS"; rm -f "$BACKUP"; }
trap restore EXIT

# name | eyeX,eyeY,eyeZ | targetX,targetY,targetZ
# Eye heights near 1.2-1.6 m keep the camera at a human's eye line, which is how the
# car will actually be seen in game — a top-down hero shot flatters proportions that
# read badly from the kerb.
SHOTS=(
  "front34|4.4,1.9,5.0|0,0.7,0.2"       # the money shot: nose, greenhouse, flank
  "side|7.5,1.1,0.0|0,0.75,0"           # silhouette, DLO, arch openings
  "rear34|-4.2,1.9,-5.2|0,0.7,-0.2"     # deck, backlight, C-pillar
  "front|0.2,1.3,6.4|0,0.75,0"          # windshield + A-pillars head on
  "top|0.0,8.5,0.6|0,0,0"               # plan view: taper, roof, aperture inset
  "kerb|3.0,1.55,2.2|0,0.9,0"           # close, eye level - how it reads on a street
  "cabin|1.9,1.85,1.5|-0.1,0.55,-0.1"   # down through the glass: seats, dash, wheel
  "occupant|2.6,1.5,0.9|-0.45,0.65,0.35"  # through the driver's window at the driver
  "driver|-0.35,1.05,0.15|-0.35,0.95,2.5"  # from the driver's eye point, looking out
)

patch_cam () {  # $1=eye  $2=target
  python3 - "$1" "$2" <<'PY'
import json, math, sys
ex,ey,ez = map(float, sys.argv[1].split(','))
tx,ty,tz = map(float, sys.argv[2].split(','))
dx,dy,dz = tx-ex, ty-ey, tz-ez
L = math.sqrt(dx*dx+dy*dy+dz*dz) or 1.0
dx,dy,dz = dx/L, dy/L, dz/L
pitch = math.degrees(math.asin(max(-1.0,min(1.0,dy))))
yaw   = math.degrees(math.atan2(dx, -dz))
s = json.load(open("settings.json"))
s["flyEyeX"], s["flyEyeY"], s["flyEyeZ"] = ex, ey, ez
s["flyPitch"], s["flyYaw"], s["flyOrtho"] = pitch, yaw, False
s["cameraMode"] = "fly"
s["hideEditorUi"] = True
json.dump(s, open("settings.json","w"), indent=2)
PY
}

capture () {  # $1=outfile
  rm -f "$1"
  RT_FRAME_DUMP="$1" "$VIEWER" "$LEVEL" --edit >/dev/null 2>&1 &
  local pid=$!
  for _ in $(seq 1 60); do [ -f "$1" ] && break; sleep 0.25; done
  sleep 0.3
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

echo "Capturing car lab battery -> $OUT"
for shot in "${SHOTS[@]}"; do
  IFS='|' read -r name eye tgt <<< "$shot"
  patch_cam "$eye" "$tgt"
  out="$OUT/$name.png"
  capture "$out"
  if [ -f "$out" ]; then echo "  [ok] $name"; else echo "  [FAIL] $name (no frame written)"; fi
done
echo "Done. $(ls "$OUT"/*.png 2>/dev/null | wc -l | tr -d ' ') frames in $OUT"
