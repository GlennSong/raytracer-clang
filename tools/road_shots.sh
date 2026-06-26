#!/usr/bin/env bash
# Road-network visual acceptance battery (Phase 0 of docs/road-network-execution-plan.md).
#
# Drives the viewer headlessly via RT_FRAME_DUMP (see metal_renderer.mm) to capture a fixed
# set of labeled views of the showcase road scene, so any change to the road system can be
# eyeballed in one command. The fly camera is positioned by writing flyEye*/flyYaw/flyPitch
# into settings.json (the viewer rewrites that file on exit, so we back it up and restore it).
#
# Camera convention (fly_camera_controller.cpp): forward = (sin(yaw)cos(pitch), sin(pitch),
# -cos(yaw)cos(pitch)); yaw=pitch=0 looks down -Z. We compute yaw/pitch from an eye->target.
#
# Usage:   tools/road_shots.sh [output_dir]   (default: /tmp/road_shots)
# Requires a built ./build/viewer (cmake --build build --target viewer).

set -euo pipefail
cd "$(dirname "$0")/.."

OUT="${1:-/tmp/road_shots}"
LEVEL="assets/levels/showcase.json"
VIEWER="./build/viewer"
SETTINGS="settings.json"
BACKUP="$(mktemp)"

mkdir -p "$OUT"
[ -x "$VIEWER" ] || { echo "error: $VIEWER not built (cmake --build build --target viewer)"; exit 1; }
cp "$SETTINGS" "$BACKUP"
restore() { cp "$BACKUP" "$SETTINGS"; rm -f "$BACKUP"; }
trap restore EXIT

# Feature coordinates (from showcase.lua): grid x[-150,-30] z[-60,60]; roundabout (-215,0) R28;
# highway x~130-152 z[-130,130]; arterial z=0 x[-30,130]; ramps near (100-130, 0).
# name | eyeX,eyeY,eyeZ | targetX,targetY,targetZ
SHOTS=(
  "overhead|-45,340,1|-45,0,0"            # whole network, top-down
  "hero|40,150,210|-70,0,-10"             # 3/4 oblique of the whole net
  "grid|-70,40,55|-95,0,0"                # look into the city grid
  "grid_junction|-108,16,-6|-110,0,-20"   # tight on one grid intersection (curb/markings)
  "roundabout|-215,55,80|-215,0,5"        # ring + spokes
  "highway_ramps|85,45,75|128,0,5"        # arterial T into highway + on/off ramps
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
json.dump(s, open("settings.json","w"), indent=2)
PY
}

capture () {  # $1=outfile
  rm -f "$1"
  RT_FRAME_DUMP="$1" "$VIEWER" "$LEVEL" --play >/dev/null 2>&1 &
  local pid=$!
  for _ in $(seq 1 60); do [ -f "$1" ] && break; sleep 0.25; done
  sleep 0.3
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

echo "Capturing road acceptance battery -> $OUT"
for shot in "${SHOTS[@]}"; do
  IFS='|' read -r name eye tgt <<< "$shot"
  patch_cam "$eye" "$tgt"
  out="$OUT/$name.png"
  capture "$out"
  if [ -f "$out" ]; then echo "  [ok] $name"; else echo "  [FAIL] $name (no frame written)"; fi
done
echo "Done. $(ls "$OUT"/*.png 2>/dev/null | wc -l | tr -d ' ') frames in $OUT"
