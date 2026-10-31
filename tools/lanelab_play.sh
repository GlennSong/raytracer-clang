#!/usr/bin/env bash
# Launch the engine on a lane-lab level from anywhere (the loader resolves asset paths from
# the repo root, so this cds there first). Default: the ring city, in play.
#   tools/lanelab_play.sh                 # the ring city (loads in seconds)
#   tools/lanelab_play.sh hill --edit     # the hill junction, in the editor state
#   tools/lanelab_play.sh ring --editor   # the Qt editor shell instead of the viewer
# A whole planned city is a level in assets/levels (metro_v2_test, metro_planned, ...): pass its path.
set -euo pipefail
cd "$(dirname "$0")/.."
case "${1:-ring}" in
  ring)  LEVEL=assets/lanelab/levels/ring.json ;;
  hill)  LEVEL=assets/lanelab/levels/hill.json ;;
  *.json) LEVEL="$1" ;;
  *) echo "usage: $0 [ring|hill|<level.json>] [--edit] [--editor]"; exit 1 ;;
esac
shift $(( $# > 0 ? 1 : 0 ))
BIN=build-viewer/viewer
ARGS=()
for a in "$@"; do
  case "$a" in
    --editor) BIN=build-viewer/editor_app ;;
    *) ARGS+=("$a") ;;
  esac
done
[ -x "$BIN" ] || { echo "error: $BIN not built (cmake --build build-viewer --target $(basename "$BIN"))"; exit 1; }
echo "launching $BIN $LEVEL ${ARGS[*]:-} (a lane-lab level builds its roads at load)"
exec "$BIN" "$LEVEL" "${ARGS[@]}"
