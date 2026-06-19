CXX = clang++
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic -pthread -isystem third_party
DEBUG_FLAGS = -g -O0
RELEASE_FLAGS = -O2

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
	$(SRC_DIR)/curve.cpp \
	$(SRC_DIR)/level_scene.cpp \
	$(SRC_DIR)/path_tracer.cpp \
	$(SRC_DIR)/engine/mesh_builder.cpp \
	$(SRC_DIR)/engine/model_importer.cpp \
	$(SRC_DIR)/engine/procgen/noise.cpp \
	$(SRC_DIR)/engine/procgen/terrain.cpp \
	$(SRC_DIR)/engine/procgen/terrain_lod.cpp \
	$(SRC_DIR)/engine/procgen/sdf.cpp \
	$(SRC_DIR)/engine/procgen/lsystem.cpp \
	$(SRC_DIR)/engine/procgen/skeleton.cpp \
	$(SRC_DIR)/engine/procgen/tree.cpp \
	$(SRC_DIR)/engine/procgen/erosion.cpp \
	$(SRC_DIR)/engine/procgen/city/polygon.cpp \
	$(SRC_DIR)/engine/procgen/city/shape_grammar.cpp \
	$(SRC_DIR)/engine/procgen/city/parcel.cpp \
	$(SRC_DIR)/engine/procgen/city/road_network.cpp \
	$(SRC_DIR)/engine/procgen/city/city.cpp
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
	$(TEST_DIR)/test_input_map.cpp \
	$(TEST_DIR)/test_player_input.cpp \
	$(TEST_DIR)/test_camera.cpp \
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
	$(TEST_DIR)/test_day_night.cpp \
	$(TEST_DIR)/test_cube_faces.cpp \
	$(TEST_DIR)/test_path_tracer.cpp \
	$(TEST_DIR)/test_asset_manager.cpp \
	$(TEST_DIR)/test_mesh_builder.cpp \
	$(TEST_DIR)/test_noise.cpp \
	$(TEST_DIR)/test_terrain.cpp \
	$(TEST_DIR)/test_terrain_lod.cpp \
	$(TEST_DIR)/test_lsystem.cpp \
	$(TEST_DIR)/test_tree.cpp \
	$(TEST_DIR)/test_rock.cpp \
	$(TEST_DIR)/test_scatter.cpp \
	$(TEST_DIR)/test_sdf.cpp \
	$(TEST_DIR)/test_curve.cpp \
	$(TEST_DIR)/test_city.cpp
TEST_ENGINE_SRCS = \
	$(SRC_DIR)/job_system.cpp \
	$(SRC_DIR)/log.cpp \
	$(SRC_DIR)/engine/day_night_cycle.cpp \
	$(SRC_DIR)/engine/world.cpp \
	$(SRC_DIR)/engine/components.cpp \
	$(SRC_DIR)/engine/clock.cpp \
	$(SRC_DIR)/engine/input/input_map.cpp \
	$(SRC_DIR)/engine/input/player_input.cpp \
	$(SRC_DIR)/engine/camera/orbit_camera_controller.cpp \
	$(SRC_DIR)/engine/camera/fly_camera_controller.cpp \
	$(SRC_DIR)/engine/camera/scene_camera.cpp \
	$(SRC_DIR)/engine/camera_store.cpp \
	$(SRC_DIR)/engine/level_writer.cpp \
	$(SRC_DIR)/renderer/hosted_window.cpp \
	$(SRC_DIR)/engine/editor_bridge.cpp \
	$(SRC_DIR)/engine/properties.cpp \
	$(SRC_DIR)/engine/component_registry.cpp \
	$(SRC_DIR)/engine/property_json.cpp \
	$(SRC_DIR)/engine/undo_stack.cpp \
	$(SRC_DIR)/engine/model_importer.cpp \
	$(SRC_DIR)/engine/mesh_builder.cpp \
	$(SRC_DIR)/engine/asset_manager.cpp \
	$(SRC_DIR)/engine/procgen/noise.cpp \
	$(SRC_DIR)/engine/procgen/terrain.cpp \
	$(SRC_DIR)/engine/procgen/terrain_lod.cpp \
	$(SRC_DIR)/engine/procgen/lsystem.cpp \
	$(SRC_DIR)/engine/procgen/skeleton.cpp \
	$(SRC_DIR)/engine/procgen/tree.cpp \
	$(SRC_DIR)/engine/procgen/erosion.cpp \
	$(SRC_DIR)/engine/procgen/rock.cpp \
	$(SRC_DIR)/engine/procgen/scatter.cpp \
	$(SRC_DIR)/engine/procgen/sdf.cpp \
	$(SRC_DIR)/engine/procgen/city/polygon.cpp \
	$(SRC_DIR)/engine/procgen/city/shape_grammar.cpp \
	$(SRC_DIR)/engine/procgen/city/parcel.cpp \
	$(SRC_DIR)/engine/procgen/city/road_network.cpp \
	$(SRC_DIR)/engine/procgen/city/city.cpp \
	$(SRC_DIR)/camera.cpp \
	$(SRC_DIR)/level_scene.cpp \
	$(SRC_DIR)/scene.cpp \
	$(SRC_DIR)/geometry.cpp \
	$(SRC_DIR)/kdtree.cpp \
	$(SRC_DIR)/curve.cpp \
	$(SRC_DIR)/image.cpp \
	$(SRC_DIR)/path_tracer.cpp \
	$(SRC_DIR)/rt_math.cpp
TEST_TARGET = run_tests

.PHONY: all release test clean

all: CXXFLAGS += $(DEBUG_FLAGS)
all: $(TARGET)

release: CXXFLAGS += $(RELEASE_FLAGS)
release: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

test: CXXFLAGS += $(DEBUG_FLAGS)
test: $(TEST_TARGET)
	./$(TEST_TARGET)

$(TEST_TARGET): $(TEST_SRCS) $(TEST_ENGINE_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(TEST_TARGET)
