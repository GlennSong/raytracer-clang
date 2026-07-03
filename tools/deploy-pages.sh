#!/bin/bash
# Deploy the web build to GitHub Pages (the gh-pages branch).
#
# Usage:   tools/deploy-pages.sh ["commit message"]
# Expects: a completed web build in build-web/ (see docs/web-build.md), i.e.
#          viewer_web.{js,wasm,data} + the copied front-end. Publishes them to
#          the gh-pages branch via a temporary worktree and pushes.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
MSG="${1:-Pages: update}"
WT="$(mktemp -d)/ghp"

cd "$REPO"
for f in index.html viewer.html about.html scenes.json \
         viewer_web.js viewer_web.wasm viewer_web.data; do
    [ -f "build-web/$f" ] || { echo "missing build-web/$f — build first (docs/web-build.md)"; exit 1; }
done

# Refuse to publish a viewer.html whose build token was never injected (e.g.
# a raw copy from web/ pasted over the CMake-processed one) — it would silently
# fall back to per-visit cache busting and every visitor would re-download.
if grep -q "__RT_BUILD__" build-web/viewer.html; then
    echo "build-web/viewer.html still has the __RT_BUILD__ placeholder — rebuild"
    echo "(or re-run: cmake -DSRC=web/viewer.html -DDST=build-web/viewer.html \\"
    echo "            -DWASM=build-web/viewer_web.wasm -P cmake/inject_build_id.cmake)"
    exit 1
fi

git fetch origin gh-pages -q
git worktree add "$WT" gh-pages -q
trap 'cd "$REPO"; git worktree remove "$WT" --force 2>/dev/null || true; git worktree prune' EXIT

cp build-web/index.html build-web/viewer.html build-web/about.html build-web/scenes.json \
   build-web/viewer_web.js build-web/viewer_web.wasm build-web/viewer_web.data "$WT"/
mkdir -p "$WT/thumbs"
rm -f "$WT"/thumbs/*.png "$WT"/thumbs/*.webp   # drop stale formats before recopying
cp build-web/thumbs/*.webp "$WT/thumbs/" 2>/dev/null || true
touch "$WT/.nojekyll"

git -C "$WT" add -A
if git -C "$WT" diff --cached --quiet; then
    echo "nothing changed — not deploying."
    exit 0
fi
git -C "$WT" commit -q -m "$MSG"
git -C "$WT" push origin gh-pages
echo "deployed."
