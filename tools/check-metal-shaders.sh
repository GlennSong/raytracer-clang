#!/bin/bash
# Offline-compile the Metal shader library exactly as the runtime does.
#
# The Metal backend compiles its shaders at runtime (newLibraryWithSource), so
# shaders/metal/* is normally only validated by actually launching the app on a
# Mac — the exact gap docs/ci-plan.md Stage 1 closes for Vulkan with glslc.
# This script extracts the load-bearing concatenation list straight from
# metal_renderer.mm (the single source of truth for order AND membership) and
# feeds the same concatenation through the offline compiler, so CI fails on
# shader errors without needing to boot a renderer. #line directives keep
# diagnostics pointing at the real file, same as the runtime path.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$REPO/src/renderer/metal/metal_renderer.mm"
OUT="${TMPDIR:-/tmp}/rt_metal_concat.metal"

files=$(sed -n 's/^[[:space:]]*@"\(shaders\/metal\/[^"]*\)".*/\1/p' "$SRC")
[ -n "$files" ] || { echo "error: no shader list found in $SRC" >&2; exit 1; }

: > "$OUT"
while IFS= read -r f; do
    [ -f "$REPO/$f" ] || { echo "error: $f listed in metal_renderer.mm but missing" >&2; exit 1; }
    printf '#line 1 "%s"\n' "$(basename "$f")" >> "$OUT"
    cat "$REPO/$f" >> "$OUT"
    printf '\n' >> "$OUT"
done <<< "$files"

xcrun -sdk macosx metal -c "$OUT" -o /dev/null
echo "ok: metal shader library compiles clean ($(echo "$files" | wc -l | tr -d ' ') modules)"
