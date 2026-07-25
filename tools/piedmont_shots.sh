#!/usr/bin/env bash
# Piedmont (8km-city plan) visual acceptance battery: terrain + multi-site road
# plan review shots via RT_FRAME_DUMP. Same conventions as road_shots.sh, with
# first-bake-friendly timeouts (the 8km terrain erodes + conforms on first load;
# later runs hit the content-hash caches).
#
# Usage: tools/piedmont_shots.sh [output_dir] [level]   (default: /tmp/piedmont_shots)

set -euo pipefail
cd "$(dirname "$0")/.."

OUT="${1:-/tmp/piedmont_shots}"
LEVEL="${2:-assets/levels/piedmont_roads.json}"
VIEWER="./build/viewer"
SETTINGS="settings.json"
BACKUP="$(mktemp)"

mkdir -p "$OUT"
[ -x "$VIEWER" ] || { echo "error: $VIEWER not built"; exit 1; }
cp "$SETTINGS" "$BACKUP"
restore() { cp "$BACKUP" "$SETTINGS"; rm -f "$BACKUP"; }
trap restore EXIT

# name | eyeX,eyeY,eyeZ | targetX,targetY,targetZ
SHOTS=(
  "map_orbit|900,5200,2400|900,0,600"        # whole map, high oblique
  "city_plan|900,2600,1100|900,0,880"        # city network, near top-down
  "downtown|1500,420,1650|900,20,900"        # 3/4 into the core
  "east_town|3400,320,880|3050,15,400"       # oldtown satellite
  "south_town|700,320,3500|300,15,3050"      # residential satellite
  "connector_east|2000,260,700|2900,20,430"  # city->east arterial
  "range_vista|500,140,500|-3200,700,-3200"  # NW mountains from downtown edge
)

patch_cam () {
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

capture () {  # $1=outfile $2=waitquarters
  rm -f "$1"
  # --edit, not --play: a citysim level's play state OWNS the camera every frame
  # (player/spectate) and ignores the fly settings; the editor state honors them.
  RT_FRAME_DUMP="$1" "$VIEWER" "$LEVEL" --edit >/dev/null 2>&1 &
  local pid=$!
  for _ in $(seq 1 "$2"); do [ -f "$1" ] && break; sleep 0.25; done
  sleep 0.5
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

echo "Piedmont battery -> $OUT (level: $LEVEL)"
first=1200   # first load may bake: up to 5 min
rest=240     # cached loads: up to 1 min
for shot in "${SHOTS[@]}"; do
  IFS='|' read -r name eye tgt <<< "$shot"
  patch_cam "$eye" "$tgt"
  out="$OUT/$name.png"
  capture "$out" "$first"
  first=$rest
  if [ -f "$out" ]; then echo "  [ok] $name"; else echo "  [FAIL] $name"; fi
done
echo "Done. $(ls "$OUT"/*.png 2>/dev/null | wc -l | tr -d ' ') frames in $OUT"
