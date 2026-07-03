#!/bin/bash
# Bake the scene-gallery thumbnails: path-trace every level that has a placed
# camera (a <level>.json.cameras.json sidecar) with the offline renderer, then
# convert to WebP for the gallery (web/thumbs/<id>.webp — PNGs are gitignored).
#
# Usage:   tools/bake-thumbs.sh [level-id ...]     # default: all sidecar levels
# Needs:   ./raytracer (make release), and a WebP encoder: cwebp on PATH or
#          python3 + Pillow (pip install Pillow).
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
SIZE=400 SPP=24 QUALITY=80

[ -x ./raytracer ] || { echo "./raytracer missing — run 'make release' first"; exit 1; }

to_webp() {  # $1 = png, $2 = webp
    if command -v cwebp >/dev/null; then
        cwebp -quiet -q "$QUALITY" "$1" -o "$2"
    else
        python3 - "$1" "$2" "$QUALITY" <<'PY'
import sys
from PIL import Image
Image.open(sys.argv[1]).convert('RGB').save(sys.argv[2], 'WEBP',
                                            quality=int(sys.argv[3]), method=6)
PY
    fi
}

ids=("$@")
if [ ${#ids[@]} -eq 0 ]; then
    for f in assets/levels/*.json; do
        case "$f" in *.cameras.json) continue;; esac
        b="$(basename "$f" .json)"
        [ -f "assets/levels/$b.json.cameras.json" ] && ids+=("$b")
    done
fi

mkdir -p web/thumbs
for id in "${ids[@]}"; do
    png="$(mktemp --suffix=.png)"
    echo "=== $id"
    ./raytracer --level "assets/levels/$id.json" --out "$png" \
                --size "$SIZE" --spp "$SPP" >/dev/null 2>&1
    to_webp "$png" "web/thumbs/$id.webp"
    rm -f "$png"
done
echo "baked ${#ids[@]} thumbnail(s) into web/thumbs/"
