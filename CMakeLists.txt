cmake_minimum_required(VERSION 3.20)
project(raytracer LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared math/core sources (used by both targets)
set(CORE_SOURCES
    src/math.cpp
    src/log.cpp
    src/image.cpp
    src/camera.cpp
    src/geometry.cpp
    src/material.cpp
    src/scene.cpp
    src/kdtree.cpp
)

# ============================================================
# Offline path tracer (no GPU dependencies)
# ============================================================
add_executable(raytracer
    src/main.cpp
    ${CORE_SOURCES}
)
target_compile_options(raytracer PRIVATE -Wall -Wextra -Wpedantic)

# ============================================================
# Unit tests (no GPU dependencies)
# ============================================================
add_executable(run_tests
    tests/main.cpp
    tests/test_math.cpp
    tests/test_slot_map.cpp
    tests/test_sparse_set.cpp
    tests/test_world.cpp
    tests/test_clock.cpp
    tests/test_input_map.cpp
    tests/test_player_input.cpp
    tests/test_camera.cpp
    src/engine/world.cpp
    src/engine/clock.cpp
    src/engine/input/input_map.cpp
    src/engine/input/player_input.cpp
    src/engine/camera/orbit_camera_controller.cpp
    src/engine/camera/fly_camera_controller.cpp
)
target_compile_options(run_tests PRIVATE -Wall -Wextra -Wpedantic)
enable_testing()
add_test(NAME unit_tests COMMAND run_tests)

# ============================================================
# Interactive viewer (GPU + windowing)
# ============================================================
find_package(glfw3 REQUIRED)

set(VIEWER_SOURCES
    src/viewer_main.cpp
    src/renderer/window.cpp
    src/renderer/settings.cpp
    src/engine/clock.cpp
    src/engine/world.cpp
    src/engine/components.cpp
    src/engine/application.cpp
    src/engine/input/input_map.cpp
    src/engine/input/player_input.cpp
    src/engine/camera/orbit_camera_controller.cpp
    src/engine/camera/fly_camera_controller.cpp
    src/engine/systems/camera_system.cpp
    src/engine/systems/motion_system.cpp
    src/engine/systems/render_system.cpp
    src/engine/systems/dev_control_system.cpp
    src/engine/systems/debug_overlay_system.cpp
)

# Platform-specific renderer backend
if(APPLE)
    enable_language(OBJCXX)
    list(APPEND VIEWER_SOURCES src/renderer/metal/metal_renderer.mm)
    set(PLATFORM_LIBS
        "-framework Metal"
        "-framework MetalKit"
        "-framework QuartzCore"
        "-framework Cocoa"
        "-framework IOKit"
    )
    set(PLATFORM_DEFINES USE_METAL=1)
# elseif(WIN32 OR UNIX)
#     find_package(Vulkan REQUIRED)
#     list(APPEND VIEWER_SOURCES src/renderer/vulkan/vulkan_renderer.cpp)
#     set(PLATFORM_LIBS Vulkan::Vulkan)
#     set(PLATFORM_DEFINES USE_VULKAN=1)
endif()

add_executable(viewer
    ${VIEWER_SOURCES}
    ${CORE_SOURCES}
)

target_compile_options(viewer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(viewer PRIVATE glfw ${PLATFORM_LIBS})
target_compile_definitions(viewer PRIVATE ${PLATFORM_DEFINES})

# Set Obj-C++ flags for .mm files
if(APPLE)
    set_source_files_properties(
        src/renderer/metal/metal_renderer.mm
        PROPERTIES COMPILE_FLAGS "-fobjc-arc"
    )
endif()

# ============================================================
# Dear ImGui debug UI (ADR-0011) — off by default.
# When ON: add the submodule at third_party/imgui, compile its core + the
# glfw/metal backends, and define RT_ENABLE_IMGUI. The seam in window.cpp /
# metal_renderer.mm / DebugOverlaySystem lights up; off, they are no-ops.
# ============================================================
option(RT_ENABLE_IMGUI "Build the viewer with Dear ImGui debug UI" OFF)
if(RT_ENABLE_IMGUI)
    set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
    target_sources(viewer PRIVATE
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    )
    if(APPLE)
        target_sources(viewer PRIVATE ${IMGUI_DIR}/backends/imgui_impl_metal.mm)
        set_source_files_properties(
            ${IMGUI_DIR}/backends/imgui_impl_metal.mm
            PROPERTIES COMPILE_FLAGS "-fobjc-arc"
        )
    endif()
    target_include_directories(viewer PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends)
    target_compile_definitions(viewer PRIVATE RT_ENABLE_IMGUI=1)
endif()
