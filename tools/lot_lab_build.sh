#!/bin/sh
# Build the Lot Lab against the REAL city module, so `engine.svg` draws what the
# shipping pipeline actually produces rather than a reconstruction of it.
#
# Everything here is headless — no GLFW, no Metal, no Jolt — which is the point:
# the whole road -> block -> parcel -> architect -> building chain runs and can
# be inspected in an environment that cannot open a 3-D viewer.
#
#   ./tools/lot_lab_build.sh && OUT=/tmp /tmp/lot_lab 7
set -e
cd "$(dirname "$0")/.."

CITY=src/engine/procgen/city
PROC=src/engine/procgen

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

# shellcheck disable=SC2086
c++ -std=c++17 -O2 -isystem third_party -I. tools/lot_lab.cpp $SRC \
    -o "${OUTBIN:-/tmp/lot_lab}"
echo "built ${OUTBIN:-/tmp/lot_lab}"
