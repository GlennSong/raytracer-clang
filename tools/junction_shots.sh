#!/bin/zsh
# Overhead screenshots of intersections, aimed by the road GRAPH (no guessing).
# Usage: junction_shots.sh <level.json> <out-prefix> <n-junctions> [alt] [pick=first|deg4|wide]
set -e
REPO=/Users/glennsong/git/raytracer-clang
SC=/private/tmp/claude-501/-Users-glennsong-git-raytracer-clang/561eb2d9-6b40-455c-ba1f-97cb5dbedc56/scratchpad
LEVEL=$1; OUT=$2; NPICK=${3:-6}; ALT=${4:-55}; PICK=${5:-spread}
cd $REPO
JF=$SC/junctions.txt
SHOT="$(dirname $LEVEL)/.shot_$(basename $LEVEL)"
python3 -c "import json;l=json.load(open('$LEVEL'));l.pop('player',None);json.dump(l,open('$SHOT','w'))"

# 1) One load to DUMP the junction positions (the road graph knows them all).
RT_DUMP_JUNCTIONS=$JF RT_LATTICE_STREETS=1 RT_FRAME_DUMP=$SC/.warm.png \
  ./build/viewer "$SHOT" --play > $SC/js.log 2>&1 &
P=$!; for i in $(seq 1 200); do [ -f "$JF" ] && [ -f "$SC/.warm.png" ] && break; sleep 1; done
sleep 1; kill $P 2>/dev/null; wait $P 2>/dev/null; rm -f $SC/.warm.png
[ -f "$JF" ] || { echo "no junctions dumped"; tail -5 $SC/js.log; exit 1; }
echo "dumped $(wc -l < $JF) junctions"

# 2) Pick N of them, spread across the list, and shoot each top-down.
python3 - "$JF" "$NPICK" "$PICK" > $SC/.pick.txt <<'EOF'
import sys
rows=[l.split() for l in open(sys.argv[1]) if l.strip()]
rows=[(float(x),float(z),int(d)) for x,z,d in rows]
n=int(sys.argv[2]); pick=sys.argv[3]
if pick=='deg4': rows=[r for r in rows if r[2]>=4] or rows
step=max(1,len(rows)//max(1,n))
for r in rows[::step][:n]: print(f"{r[0]:.2f} {r[1]:.2f} {r[2]}")
EOF

i=0
while read x z d; do
  cp settings.json $SC/s.bak.json
  python3 - "$x" "$z" "$ALT" <<'EOF'
import json,sys
s=json.load(open('settings.json'))
s.update({"cameraMode":"fly","cameraStartDetached":True,
  "flyEyeX":float(sys.argv[1]),"flyEyeY":float(sys.argv[3]),"flyEyeZ":float(sys.argv[2]),
  "flyPitch":-89.0,"flyYaw":0.0,"flyOrtho":False})
json.dump(s,open('settings.json','w'),indent=2)
EOF
  F="${OUT}_${i}_d${d}.png"; rm -f "$F"
  RT_LATTICE_STREETS=1 RT_FRAME_DUMP="$F" ./build/viewer "$SHOT" --play > $SC/js.log 2>&1 &
  P=$!; for k in $(seq 1 150); do [ -f "$F" ] && break; sleep 1; done; sleep 1
  kill $P 2>/dev/null; wait $P 2>/dev/null
  cp $SC/s.bak.json settings.json
  echo "shot $F  @ ($x,$z) deg$d"
  i=$((i+1))
done < $SC/.pick.txt
rm -f "$SHOT"
