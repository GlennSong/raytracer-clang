#!/bin/sh
# Build and run the Lot System's SVG sheets — the paper review path.
#
# There is no mesher for the new pipeline yet, so the only way to judge whether
# a plan, a parcelling or a site layout is any good is to draw it. These sheets
# are generated from the SHIPPING modules (not a sketch of them), so what they
# show is what the engine builds.
#
#   tools/lot_sheets.sh [outdir]        # default: /tmp/lot-sheets
set -e
cd "$(dirname "$0")/.."
OUT=${1:-/tmp/lot-sheets}
mkdir -p "$OUT"

CITY=src/engine/procgen/city
PROC=src/engine/procgen
NEW="$CITY/shape2.cpp $CITY/shape_ops.cpp $CITY/plan_grammar.cpp $CITY/mass_stack.cpp
$CITY/facade_plan.cpp $CITY/lot_program.cpp $CITY/site_plan.cpp
$CITY/material_set.cpp $CITY/building_recipe.cpp $CITY/lot_fixtures.cpp $CITY/parcel_block.cpp"
SRC="
$CITY/architect.cpp $CITY/shape_grammar.cpp $CITY/polygon.cpp $CITY/parcel.cpp
$CITY/road_mesh.cpp $CITY/road_offset.cpp $CITY/triangulate.cpp $CITY/street_kit.cpp
$CITY/city_lots.cpp $CITY/road_network.cpp $CITY/road_net.cpp $CITY/road_rules.cpp
$CITY/road_constraints.cpp $CITY/road_crossings.cpp $CITY/district.cpp
$CITY/road_spec.cpp $CITY/road_semantics.cpp $CITY/metro.cpp $CITY/structure_set.cpp
$CITY/corridor_plan.cpp $CITY/corridor_mesh.cpp $CITY/corridor_bake.cpp
$CITY/road_lattice.cpp $CITY/block_grade.cpp $CITY/buildability.cpp
$CITY/alignment.cpp $CITY/water_mesh.cpp $CITY/surface_field.cpp
$CITY/city_footprint.cpp $CITY/arterial_skeleton.cpp $CITY/patch_fabric.cpp
$PROC/surface_maps.cpp $PROC/terrain.cpp $PROC/terrain_lod.cpp $PROC/terrain_field.cpp
$PROC/noise.cpp $PROC/lsystem.cpp $PROC/skeleton.cpp $PROC/erosion.cpp $PROC/sdf.cpp
$PROC/tree.cpp $PROC/scatter.cpp $PROC/cellular.cpp $PROC/rock.cpp $PROC/texture_field.cpp
src/engine/mesh_builder.cpp src/engine/ai/nav_graph.cpp
src/rt_math.cpp src/curve.cpp src/log.cpp
"

# Both tools take the output directory from OUT and a seed as argv[1].
for tool in lot_system_sheet lot_block_sheet; do
    # shellcheck disable=SC2086
    c++ -std=c++17 -O1 -isystem third_party -I. "tools/$tool.cpp" $NEW $SRC \
        -o "$OUT/$tool"
    OUT="$OUT" "$OUT/$tool"
done
