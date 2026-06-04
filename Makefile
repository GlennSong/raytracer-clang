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

.PHONY: all release clean

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

clean:
	rm -rf $(BUILD_DIR) $(TARGET)
