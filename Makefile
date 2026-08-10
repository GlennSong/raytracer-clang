CXX = clang++
# -MMD -MP emit header dependency files (.d) so a changed header forces every
# dependent .o to rebuild. Without this, editing a struct in a header (e.g.
# city.h / terrain.h) left stale object files with a mismatched layout — the
# "terrain but no city" class of bug. -include pulls the .d files in below.
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic -pthread -isystem third_party -MMD -MP -DRT_NO_GPU_EROSION
DEBUG_FLAGS = -g -O0
RELEASE_FLAGS = -O2

# Lua scripting in the offline tracer (ADR-0042): so shape:"script" entities run
# the same recipes as the viewer and a procgen scene renders offline through the
# same pipeline. Lua is vendored C — compiled as C with warnings off (-w), kept
# separate from our C++. Scoped to the raytracer target so the test build stays
# Lua-free (level_scene's script path is guarded by RT_ENABLE_SCRIPTING).
CC = clang
LUA_DIR = third_party/lua
# Exclude the standalone mains (lua.c/luac.c), the single-TU amalgamation
# (onelua.c — it re-defines every symbol) and the internal test harness
# (ltests.c). Linking the individual objects directly, duplicates would clash.
LUA_SRCS = $(filter-out $(LUA_DIR)/lua.c $(LUA_DIR)/luac.c $(LUA_DIR)/onelua.c \
                        $(LUA_DIR)/ltests.c,$(wildcard $(LUA_DIR)/*.c))
LUA_OBJS = $(patsubst $(LUA_DIR)/%.c,$(BUILD_DIR)/lua/%.o,$(LUA_SRCS))
SCRIPT_FLAGS = -DRT_ENABLE_SCRIPTING=1 -isystem $(LUA_DIR)

SRC_DIR = src
BUILD_DIR = build
TARGET = raytracer

# Offline path tracer only. The interactive viewer (viewer_main.cpp, renderer/,
# engine/) needs GLFW + a platform backend and is built via CMake instead.
SRCS = \
	$(SRC_DIR)/main.cpp \
	$(SRC_DIR)/rt_math.cpp \
	$(SRC_DIR)/job_system.cpp \
	$(SRC_DIR)/log.cpp \
	$(SRC_DIR)/image.cpp \
	$(SRC_DIR)/camera.cpp \
	$(SRC_DIR)/geometry.cpp \
	$(SRC_DIR)/material.cpp \
	$(SRC_DIR)/scene.cpp \
	$(SRC_DIR)/kdtree.cpp \
	$(SRC_DIR)/instance.cpp \
	$(SRC_DIR)/curve.cpp \
	$(SRC_DIR)/level_scene.cpp \
	$(SRC_DIR)/engine/level_params.cpp \
	$(SRC_DIR)/path_tracer.cpp \
	$(SRC_DIR)/engine/mesh_builder.cpp \
	$(SRC_DIR)/engine/editable_curve.cpp \
	$(SRC_DIR)/engine/curve_edit.cpp \
	$(SRC_DIR)/engine/animation_path.cpp \
	$(SRC_DIR)/engine/handle_source.cpp \
	$(SRC_DIR)/engine/path_edit_tool.cpp \
	$(SRC_DIR)/engine/model_importer.cpp \
	$(SRC_DIR)/engine/procgen/noise.cpp \
	$(SRC_DIR)/engine/procgen/terrain.cpp \
	$(SRC_DIR)/engine/procgen/terrain_lod.cpp \
	$(SRC_DIR)/engine/procgen/sdf.cpp \
	$(SRC_DIR)/engine/procgen/lsystem.cpp \
	$(SRC_DIR)/engine/procgen/skeleton.cpp \
	$(SRC_DIR)/engine/procgen/tree.cpp \
	$(SRC_DIR)/engine/procgen/surface_maps.cpp \
	$(SRC_DIR)/engine/procgen/vehicle_mesh.cpp \
	$(SRC_DIR)/engine/procgen/vehicle/car_mesh.cpp \
	$(SRC_DIR)/engine/procgen/vehicle/car_interior.cpp \
	$(SRC_DIR)/engine/procgen/vehicle/occupant.cpp \
	$(SRC_DIR)/engine/procgen/texture_field.cpp \
	$(SRC_DIR)/engine/procgen/terrain_field.cpp \
	$(SRC_DIR)/engine/procgen/erosion.cpp \
	$(SRC_DIR)/engine/procgen/city/polygon.cpp \
	$(SRC_DIR)/engine/procgen/city/shape2.cpp \
	$(SRC_DIR)/engine/procgen/city/shape_ops.cpp \
	$(SRC_DIR)/engine/procgen/city/plan_grammar.cpp \
	$(SRC_DIR)/engine/procgen/city/mass_stack.cpp \
	$(SRC_DIR)/engine/procgen/city/facade_plan.cpp \
	$(SRC_DIR)/engine/procgen/city/lot_program.cpp \
	$(SRC_DIR)/engine/procgen/city/site_plan.cpp \
	$(SRC_DIR)/engine/procgen/city/shape_grammar.cpp \
	$(SRC_DIR)/engine/procgen/city/parcel.cpp \
	$(SRC_DIR)/engine/procgen/city/road_network.cpp \
	$(SRC_DIR)/engine/procgen/city/street_kit.cpp \
	$(SRC_DIR)/engine/procgen/city/street_furniture.cpp \
	$(SRC_DIR)/engine/procgen/city/alignment.cpp \
	$(SRC_DIR)/engine/procgen/city/corridor_mesh.cpp \
	$(SRC_DIR)/engine/procgen/city/road_mesh.cpp \
	$(SRC_DIR)/engine/procgen/city/triangulate.cpp \
	$(SRC_DIR)/engine/procgen/city/corridor_bake.cpp \
	$(SRC_DIR)/engine/procgen/city/corridor_plan.cpp \
	$(SRC_DIR)/engine/procgen/city/road_spec.cpp \
	$(SRC_DIR)/engine/procgen/city/road_net.cpp \
	$(SRC_DIR)/engine/procgen/city/road_lattice.cpp \
	$(SRC_DIR)/engine/procgen/city/road_constraints.cpp \
	$(SRC_DIR)/engine/procgen/city/road_semantics.cpp \
	$(SRC_DIR)/engine/procgen/city/road_rules.cpp \
	$(SRC_DIR)/engine/procgen/city/road_offset.cpp \
	$(SRC_DIR)/engine/procgen/city/district.cpp \
	$(SRC_DIR)/engine/procgen/city/metro.cpp \
	$(SRC_DIR)/engine/procgen/city/water_mesh.cpp \
	$(SRC_DIR)/engine/procgen/city/buildability.cpp \
	$(SRC_DIR)/engine/procgen/city/road_crossings.cpp \
	$(SRC_DIR)/engine/procgen/city/city.cpp \
	$(SRC_DIR)/engine/procgen/city/architect.cpp \
	$(SRC_DIR)/engine/procgen/city/city_lots.cpp \
	$(SRC_DIR)/engine/procgen/city/surface_field.cpp \
	$(SRC_DIR)/engine/procgen/city/structure_set.cpp \
	$(SRC_DIR)/engine/procgen/city/block_grade.cpp \
	$(SRC_DIR)/engine/procgen/city/city_footprint.cpp \
	$(SRC_DIR)/engine/procgen/city/arterial_skeleton.cpp \
	$(SRC_DIR)/engine/procgen/city/patch_fabric.cpp \
	$(SRC_DIR)/engine/procgen/scatter.cpp \
	$(SRC_DIR)/engine/procgen/cellular.cpp \
	$(SRC_DIR)/engine/procgen/planet.cpp \
	$(SRC_DIR)/engine/procgen/atmosphere.cpp \
	$(SRC_DIR)/engine/scripting/script_vm.cpp \
	$(SRC_DIR)/engine/scripting/procgen_bindings.cpp \
	$(SRC_DIR)/engine/scripting/script_modules.cpp \
	$(SRC_DIR)/engine/script_assets.cpp \
	$(SRC_DIR)/engine/ai/nav_graph.cpp \
	$(SRC_DIR)/engine/ai/pathfind.cpp
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

# Unit tests. Header-only core (math, Handle/SlotMap, SparseSet) plus the few
# engine .cpp units the tests exercise. No GPU/windowing deps, so this builds
# and runs anywhere the offline tracer does (including Linux CI).
TEST_DIR = tests
TEST_SRCS = \
	$(TEST_DIR)/main.cpp \
	$(TEST_DIR)/test_math.cpp \
	$(TEST_DIR)/test_slot_map.cpp \
	$(TEST_DIR)/test_sparse_set.cpp \
	$(TEST_DIR)/test_world.cpp \
	$(TEST_DIR)/test_clock.cpp \
	$(TEST_DIR)/test_frame_stats.cpp \
	$(TEST_DIR)/test_event_bus.cpp \
	$(TEST_DIR)/test_debug_draw.cpp \
	$(TEST_DIR)/test_skeleton.cpp \
	$(TEST_DIR)/test_skin.cpp \
	$(TEST_DIR)/test_input_map.cpp \
	$(TEST_DIR)/test_player_input.cpp \
	$(TEST_DIR)/test_camera.cpp \
	$(TEST_DIR)/test_follow_camera.cpp \
	$(TEST_DIR)/test_scene_camera.cpp \
	$(TEST_DIR)/test_camera_store.cpp \
	$(TEST_DIR)/test_lens.cpp \
	$(TEST_DIR)/test_level_scene.cpp \
	$(TEST_DIR)/test_pbr_lighting.cpp \
	$(TEST_DIR)/test_level_writer.cpp \
	$(TEST_DIR)/test_hosted_window.cpp \
	$(TEST_DIR)/test_properties.cpp \
	$(TEST_DIR)/test_undo.cpp \
	$(TEST_DIR)/test_hierarchy.cpp \
	$(TEST_DIR)/test_job_system.cpp \
	$(TEST_DIR)/test_frustum.cpp \
	$(TEST_DIR)/test_cascade_fit.cpp \
	$(TEST_DIR)/test_fall_respawn.cpp \
	$(TEST_DIR)/test_day_night.cpp \
	$(TEST_DIR)/test_cube_faces.cpp \
	$(TEST_DIR)/test_path_tracer.cpp \
	$(TEST_DIR)/test_asset_manager.cpp \
	$(TEST_DIR)/test_mesh_builder.cpp \
	$(TEST_DIR)/test_shape2.cpp \
	$(TEST_DIR)/test_shape_ops.cpp \
	$(TEST_DIR)/test_plan_grammar.cpp \
	$(TEST_DIR)/test_facade_plan.cpp \
	$(TEST_DIR)/test_site_plan.cpp \
	$(TEST_DIR)/test_cube_sphere.cpp \
	$(TEST_DIR)/test_noise.cpp \
	$(TEST_DIR)/test_terrain.cpp \
	$(TEST_DIR)/test_terrain_lod.cpp \
	$(TEST_DIR)/test_lsystem.cpp \
	$(TEST_DIR)/test_tree.cpp \
	$(TEST_DIR)/test_rock.cpp \
	$(TEST_DIR)/test_planet.cpp \
	$(TEST_DIR)/test_atmosphere.cpp \
	$(TEST_DIR)/test_scatter.cpp \
	$(TEST_DIR)/test_sdf.cpp \
	$(TEST_DIR)/test_curve.cpp \
	$(TEST_DIR)/test_city.cpp \
	$(TEST_DIR)/test_building_lod.cpp \
	$(TEST_DIR)/test_road_chunks.cpp \
	$(TEST_DIR)/test_road_net.cpp \
	$(TEST_DIR)/test_road_rules.cpp \
	$(TEST_DIR)/test_road_weld.cpp \
	$(TEST_DIR)/test_curb_return.cpp \
	$(TEST_DIR)/test_road_constraints.cpp \
	$(TEST_DIR)/test_road_semantics.cpp \
	$(TEST_DIR)/test_road_layers.cpp \
	$(TEST_DIR)/test_road_offset.cpp \
	$(TEST_DIR)/test_road_crossings.cpp \
	$(TEST_DIR)/test_editable_curve.cpp \
	$(TEST_DIR)/test_curve_edit.cpp \
	$(TEST_DIR)/test_animation_path.cpp \
	$(TEST_DIR)/test_handle_source.cpp \
	$(TEST_DIR)/test_path_edit_tool.cpp \
	$(TEST_DIR)/test_surface_maps.cpp \
	$(TEST_DIR)/test_vehicle_mesh.cpp \
	$(TEST_DIR)/test_car_mesh.cpp \
	$(TEST_DIR)/test_vehicle_lamps.cpp \
	$(TEST_DIR)/test_model_importer.cpp \
	$(TEST_DIR)/test_instance_tlas.cpp \
	$(TEST_DIR)/test_texture_field.cpp \
	$(TEST_DIR)/test_terrain_field.cpp \
	$(TEST_DIR)/test_nav_graph.cpp \
	$(TEST_DIR)/test_pathfind.cpp \
	$(TEST_DIR)/test_perception.cpp \
	$(TEST_DIR)/test_agent_memory.cpp \
	$(TEST_DIR)/test_state_machine.cpp \
	$(TEST_DIR)/test_traffic_signal.cpp \
	$(TEST_DIR)/test_traffic_rules.cpp \
	$(TEST_DIR)/test_city_sim.cpp \
	$(TEST_DIR)/test_city_goals.cpp \
	$(TEST_DIR)/test_city_signals.cpp \
	$(TEST_DIR)/test_city_perception.cpp \
	$(TEST_DIR)/test_driver_agent.cpp \
	$(TEST_DIR)/test_lane_follow.cpp \
	$(TEST_DIR)/test_traffic_sense.cpp \
	$(TEST_DIR)/test_city_steering.cpp \
	$(TEST_DIR)/test_city_driver_fsm.cpp \
	$(TEST_DIR)/test_city_fleet.cpp \
	$(TEST_DIR)/test_city_drive.cpp \
	$(TEST_DIR)/test_city_render.cpp \
	$(TEST_DIR)/test_script_vm.cpp \
	$(TEST_DIR)/test_script_modules.cpp \
	$(TEST_DIR)/test_script_system.cpp \
	$(TEST_DIR)/test_gun_script.cpp \
	$(TEST_DIR)/test_flora.cpp \
	$(TEST_DIR)/test_agent_goals.cpp \
	$(TEST_DIR)/test_vehicle_body.cpp \
	$(TEST_DIR)/test_car_lamps.cpp \
	$(TEST_DIR)/test_city_flow.cpp \
	$(TEST_DIR)/test_city_spectate.cpp \
	$(TEST_DIR)/test_places.cpp \
	$(TEST_DIR)/test_screen_project.cpp \
	$(TEST_DIR)/test_relationships.cpp \
	$(TEST_DIR)/test_ped_graph.cpp \
	$(TEST_DIR)/test_city_generated.cpp \
	$(TEST_DIR)/test_city_lots.cpp \
	$(TEST_DIR)/test_alignment.cpp \
	$(TEST_DIR)/test_surface_field.cpp \
	$(TEST_DIR)/test_architect.cpp
TEST_ENGINE_SRCS = \
	$(SRC_DIR)/job_system.cpp \
	$(SRC_DIR)/log.cpp \
	$(SRC_DIR)/engine/frame_stats.cpp \
	$(SRC_DIR)/engine/day_night_cycle.cpp \
	$(SRC_DIR)/engine/event_bus.cpp \
	$(SRC_DIR)/engine/debug_draw.cpp \
	$(SRC_DIR)/engine/anim/skeleton.cpp \
	$(SRC_DIR)/engine/anim/mannequin.cpp \
	$(SRC_DIR)/engine/anim/skinned_mesh.cpp \
	$(SRC_DIR)/engine/anim/skin_import.cpp \
	$(SRC_DIR)/engine/world.cpp \
	$(SRC_DIR)/engine/components.cpp \
	$(SRC_DIR)/engine/clock.cpp \
	$(SRC_DIR)/engine/input/input_map.cpp \
	$(SRC_DIR)/engine/input/player_input.cpp \
	$(SRC_DIR)/engine/camera/orbit_camera_controller.cpp \
	$(SRC_DIR)/engine/camera/fly_camera_controller.cpp \
	$(SRC_DIR)/engine/camera/follow_camera_controller.cpp \
	$(SRC_DIR)/engine/camera/scene_camera.cpp \
	$(SRC_DIR)/engine/camera_store.cpp \
	$(SRC_DIR)/engine/level_writer.cpp \
	$(SRC_DIR)/renderer/hosted_window.cpp \
	$(SRC_DIR)/renderer/settings.cpp \
	$(SRC_DIR)/engine/editor_bridge.cpp \
	$(SRC_DIR)/engine/city_planner.cpp \
	$(SRC_DIR)/engine/properties.cpp \
	$(SRC_DIR)/engine/component_registry.cpp \
	$(SRC_DIR)/engine/property_json.cpp \
	$(SRC_DIR)/engine/undo_stack.cpp \
	$(SRC_DIR)/engine/model_importer.cpp \
	$(SRC_DIR)/engine/mesh_builder.cpp \
	$(SRC_DIR)/engine/editable_curve.cpp \
	$(SRC_DIR)/engine/curve_edit.cpp \
	$(SRC_DIR)/engine/animation_path.cpp \
	$(SRC_DIR)/engine/handle_source.cpp \
	$(SRC_DIR)/engine/path_edit_tool.cpp \
	$(SRC_DIR)/engine/asset_manager.cpp \
	$(SRC_DIR)/engine/road_chunks.cpp \
	$(SRC_DIR)/engine/procgen/noise.cpp \
	$(SRC_DIR)/engine/procgen/terrain.cpp \
	$(SRC_DIR)/engine/procgen/terrain_lod.cpp \
	$(SRC_DIR)/engine/procgen/lsystem.cpp \
	$(SRC_DIR)/engine/procgen/skeleton.cpp \
	$(SRC_DIR)/engine/procgen/tree.cpp \
	$(SRC_DIR)/engine/procgen/surface_maps.cpp \
	$(SRC_DIR)/engine/procgen/vehicle_mesh.cpp \
	$(SRC_DIR)/engine/procgen/vehicle/car_mesh.cpp \
	$(SRC_DIR)/engine/procgen/vehicle/car_interior.cpp \
	$(SRC_DIR)/engine/procgen/vehicle/occupant.cpp \
	$(SRC_DIR)/engine/procgen/texture_field.cpp \
	$(SRC_DIR)/engine/procgen/terrain_field.cpp \
	$(SRC_DIR)/engine/procgen/erosion.cpp \
	$(SRC_DIR)/engine/procgen/rock.cpp \
	$(SRC_DIR)/engine/procgen/cellular.cpp \
	$(SRC_DIR)/engine/procgen/planet.cpp \
	$(SRC_DIR)/engine/procgen/atmosphere.cpp \
	$(SRC_DIR)/engine/procgen/scatter.cpp \
	$(SRC_DIR)/engine/procgen/sdf.cpp \
	$(SRC_DIR)/engine/procgen/city/polygon.cpp \
	$(SRC_DIR)/engine/procgen/city/shape2.cpp \
	$(SRC_DIR)/engine/procgen/city/shape_ops.cpp \
	$(SRC_DIR)/engine/procgen/city/plan_grammar.cpp \
	$(SRC_DIR)/engine/procgen/city/mass_stack.cpp \
	$(SRC_DIR)/engine/procgen/city/facade_plan.cpp \
	$(SRC_DIR)/engine/procgen/city/lot_program.cpp \
	$(SRC_DIR)/engine/procgen/city/site_plan.cpp \
	$(SRC_DIR)/engine/procgen/city/shape_grammar.cpp \
	$(SRC_DIR)/engine/procgen/city/parcel.cpp \
	$(SRC_DIR)/engine/procgen/city/road_network.cpp \
	$(SRC_DIR)/engine/procgen/city/street_kit.cpp \
	$(SRC_DIR)/engine/procgen/city/street_furniture.cpp \
	$(SRC_DIR)/engine/procgen/city/alignment.cpp \
	$(SRC_DIR)/engine/procgen/city/corridor_mesh.cpp \
	$(SRC_DIR)/engine/procgen/city/road_mesh.cpp \
	$(SRC_DIR)/engine/procgen/city/triangulate.cpp \
	$(SRC_DIR)/engine/procgen/city/corridor_bake.cpp \
	$(SRC_DIR)/engine/procgen/city/corridor_plan.cpp \
	$(SRC_DIR)/engine/procgen/city/road_spec.cpp \
	$(SRC_DIR)/engine/procgen/city/road_net.cpp \
	$(SRC_DIR)/engine/procgen/city/road_lattice.cpp \
	$(SRC_DIR)/engine/procgen/city/road_constraints.cpp \
	$(SRC_DIR)/engine/procgen/city/road_semantics.cpp \
	$(SRC_DIR)/engine/procgen/city/road_rules.cpp \
	$(SRC_DIR)/engine/procgen/city/road_offset.cpp \
	$(SRC_DIR)/engine/procgen/city/district.cpp \
	$(SRC_DIR)/engine/procgen/city/metro.cpp \
	$(SRC_DIR)/engine/procgen/city/water_mesh.cpp \
	$(SRC_DIR)/engine/procgen/city/buildability.cpp \
	$(SRC_DIR)/engine/procgen/city/road_crossings.cpp \
	$(SRC_DIR)/engine/procgen/city/city.cpp \
	$(SRC_DIR)/engine/procgen/city/architect.cpp \
	$(SRC_DIR)/engine/procgen/city/city_lots.cpp \
	$(SRC_DIR)/engine/procgen/city/surface_field.cpp \
	$(SRC_DIR)/engine/procgen/city/structure_set.cpp \
	$(SRC_DIR)/engine/procgen/city/block_grade.cpp \
	$(SRC_DIR)/engine/procgen/city/city_footprint.cpp \
	$(SRC_DIR)/engine/procgen/city/arterial_skeleton.cpp \
	$(SRC_DIR)/engine/procgen/city/patch_fabric.cpp \
	$(SRC_DIR)/engine/ai/nav_graph.cpp \
	$(SRC_DIR)/engine/ai/pathfind.cpp \
	$(SRC_DIR)/apps/citysim/traffic_signal.cpp \
	$(SRC_DIR)/apps/citysim/traffic_rules.cpp \
	$(SRC_DIR)/apps/citysim/city_goals.cpp \
	$(SRC_DIR)/apps/citysim/places.cpp \
	$(SRC_DIR)/apps/citysim/ped_graph.cpp \
	$(SRC_DIR)/apps/citysim/relationships.cpp \
	$(SRC_DIR)/apps/citysim/city_sim.cpp \
	$(SRC_DIR)/apps/citysim/city_render.cpp \
	$(SRC_DIR)/apps/citysim/city_spectate.cpp \
	$(SRC_DIR)/apps/citysim/city_meshes.cpp \
	$(SRC_DIR)/engine/script_assets.cpp \
	$(SRC_DIR)/engine/scripting/script_vm.cpp \
	$(SRC_DIR)/engine/scripting/script_modules.cpp \
	$(SRC_DIR)/engine/scripting/procgen_bindings.cpp \
	$(SRC_DIR)/engine/scripting/gameplay_bindings.cpp \
	$(SRC_DIR)/engine/scripting/script_system.cpp \
	$(SRC_DIR)/engine/scripting/vehicle_spec.cpp \
	$(SRC_DIR)/apps/citysim/scripting/agent_goals.cpp \
	$(SRC_DIR)/apps/citysim/scripting/vehicle_body.cpp \
	$(SRC_DIR)/camera.cpp \
	$(SRC_DIR)/level_scene.cpp \
	$(SRC_DIR)/engine/level_params.cpp \
	$(SRC_DIR)/scene.cpp \
	$(SRC_DIR)/geometry.cpp \
	$(SRC_DIR)/kdtree.cpp \
	$(SRC_DIR)/instance.cpp \
	$(SRC_DIR)/curve.cpp \
	$(SRC_DIR)/image.cpp \
	$(SRC_DIR)/path_tracer.cpp \
	$(SRC_DIR)/rt_math.cpp
TEST_TARGET = run_tests

.PHONY: all release test planet_preview health clean

all: CXXFLAGS += $(DEBUG_FLAGS)
all: $(TARGET)

release: CXXFLAGS += $(RELEASE_FLAGS)
release: $(TARGET)

# The raytracer links Lua + scripting; the flag propagates to its prerequisite
# objects (so level_scene.o compiles its shape:"script" path), but not to the
# separately-compiled test target.
$(TARGET): CXXFLAGS += $(SCRIPT_FLAGS)
$(TARGET): $(OBJS) $(LUA_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Vendored Lua: compiled as C (clang, not clang++), warnings off.
$(BUILD_DIR)/lua/%.o: $(LUA_DIR)/%.c | $(BUILD_DIR)
	@mkdir -p $(@D)
	$(CC) -O2 -w -c -o $@ $<

# Pull in the generated header-dependency files so header edits force rebuilds.
-include $(OBJS:.o=.d)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

test: CXXFLAGS += $(DEBUG_FLAGS)
test: $(TEST_TARGET)
	./$(TEST_TARGET)

# The test build carries Lua. Scripting is ENGINE INFRASTRUCTURE, not a build
# option: the city's cars, agent goals and procgen recipes are DATA, and a test
# build without Lua does not exercise the game — it exercises the C++ fallbacks
# that exist only because this build once had no Lua. Keeping the two apart is
# what let "the fleet is scripted" and "the fleet is boxes" both look green.
# RT_SOURCE_DIR lets a test read the shipped assets/scripts/*.lua regardless of
# the working directory it is run from.
$(TEST_TARGET): CXXFLAGS += $(SCRIPT_FLAGS) -DRT_SOURCE_DIR='"$(CURDIR)"'
#
# Depends on the Makefile itself: this target compiles its whole source list in
# one command with no .o files, so ADDING a source cannot make the binary out of
# date on its own — the new file is older than the last link, and make declares
# the stale binary up to date. That silently kept newly-listed tests out of the
# run (they were "passing" by not existing).
$(TEST_TARGET): $(TEST_SRCS) $(TEST_ENGINE_SRCS) $(LUA_OBJS) Makefile
	$(CXX) $(CXXFLAGS) -o $@ $(filter-out Makefile,$^)

# Code-health report (docs/profiling.md): duplication, god-functions, debt
# markers, include fan-in. Informational — a finder, not a gate.
health:
	python3 tools/code-health.py

# Headless "from space" preview of the procedural planets (procedural-planet-plan):
# orthographic disc renders to out_*.png — a GPU-free way to eyeball the generator.
# Standalone: just the generator + its deps, no windowing/renderer.
PLANET_PREVIEW_SRCS = tools/planet_preview.cpp \
	$(SRC_DIR)/engine/procgen/planet.cpp \
	$(SRC_DIR)/engine/procgen/cellular.cpp \
	$(SRC_DIR)/engine/procgen/atmosphere.cpp \
	$(SRC_DIR)/engine/procgen/noise.cpp \
	$(SRC_DIR)/engine/mesh_builder.cpp \
	$(SRC_DIR)/rt_math.cpp
planet_preview: $(PLANET_PREVIEW_SRCS)
	$(CXX) $(CXXFLAGS) -O2 -I$(SRC_DIR) -o $@ $^

clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(TEST_TARGET) planet_preview out_*.png
