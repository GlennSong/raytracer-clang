CXX = clang++
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic
DEBUG_FLAGS = -g -O0
RELEASE_FLAGS = -O2

SRC_DIR = src
BUILD_DIR = build
TARGET = raytracer

# Offline path tracer only. The interactive viewer (viewer_main.cpp, renderer/,
# engine/) needs GLFW + a platform backend and is built via CMake instead.
SRCS = \
	$(SRC_DIR)/main.cpp \
	$(SRC_DIR)/math.cpp \
	$(SRC_DIR)/log.cpp \
	$(SRC_DIR)/image.cpp \
	$(SRC_DIR)/camera.cpp \
	$(SRC_DIR)/geometry.cpp \
	$(SRC_DIR)/material.cpp \
	$(SRC_DIR)/scene.cpp \
	$(SRC_DIR)/kdtree.cpp
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
	$(TEST_DIR)/test_clock.cpp
TEST_ENGINE_SRCS = \
	$(SRC_DIR)/engine/world.cpp \
	$(SRC_DIR)/engine/clock.cpp
TEST_TARGET = run_tests

.PHONY: all release test clean

all: CXXFLAGS += $(DEBUG_FLAGS)
all: $(TARGET)

release: CXXFLAGS += $(RELEASE_FLAGS)
release: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
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
