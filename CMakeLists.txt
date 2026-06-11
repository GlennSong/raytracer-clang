cmake_minimum_required(VERSION 3.20)
project(raytracer LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Threads back the JobSystem (and the offline tracer that now uses it).
find_package(Threads REQUIRED)

# The Jolt job adapter derives from a JPH polymorphic base. Jolt is built without
# C++ RTTI, so this one TU must match — otherwise its typeinfo references a base
# typeinfo symbol Jolt never emitted (link error). Applies in every target that
# compiles the file (physics_tests, viewer).
set_source_files_properties(src/engine/physics/jolt_job_adapter.cpp
    PROPERTIES COMPILE_OPTIONS "-fno-rtti")

# Shared math/core sources (used by both targets)
set(CORE_SOURCES
    src/rt_math.cpp
    src/job_system.cpp
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
target_link_libraries(raytracer PRIVATE Threads::Threads)

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
    tests/test_scene_camera.cpp
    tests/test_job_system.cpp
    tests/test_frustum.cpp
    tests/test_day_night.cpp
    src/job_system.cpp
    src/engine/day_night_cycle.cpp
    src/engine/world.cpp
    src/engine/clock.cpp
    src/engine/input/input_map.cpp
    src/engine/input/player_input.cpp
    src/engine/camera/orbit_camera_controller.cpp
    src/engine/camera/fly_camera_controller.cpp
    src/engine/camera/scene_camera.cpp
)
target_compile_options(run_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(run_tests PRIVATE Threads::Threads)
enable_testing()
add_test(NAME unit_tests COMMAND run_tests)

# ============================================================
# Physics (Jolt) — cross-platform C++, builds and runs headless (ADR-0012).
# Jolt's PUBLIC compile definitions propagate to anything linking the `Jolt`
# target, so the JPH_* config stays consistent automatically.
# ============================================================
option(RT_ENABLE_PHYSICS "Build Jolt physics and its headless tests" ON)
if(RT_ENABLE_PHYSICS)
    # The Jolt submodule isn't fetched by a plain clone/branch sync. If it's
    # missing, try to initialize it; otherwise fail with a clear instruction
    # rather than a cryptic add_subdirectory error.
    set(JOLT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/JoltPhysics)
    if(NOT EXISTS ${JOLT_DIR}/Build/CMakeLists.txt)
        find_package(Git QUIET)
        if(GIT_FOUND AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
            message(STATUS "Jolt submodule missing — running 'git submodule update --init'")
            execute_process(
                COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        third_party/JoltPhysics
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        endif()
    endif()
    if(NOT EXISTS ${JOLT_DIR}/Build/CMakeLists.txt)
        message(FATAL_ERROR
            "Jolt physics submodule not found at third_party/JoltPhysics.\n"
            "Fetch it with:  git submodule update --init --recursive\n"
            "or configure with -DRT_ENABLE_PHYSICS=OFF to build without physics.")
    endif()

    # Configure Jolt before pulling it in. Keep it from clobbering global flags
    # or forcing LTO; single precision (ADR-0005 deferral); drop unused bits.
    set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "" FORCE)
    set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
    set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
    set(ENABLE_OBJECT_STREAM OFF CACHE BOOL "" FORCE)
    set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF CACHE BOOL "" FORCE)
    set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
    add_subdirectory(third_party/JoltPhysics/Build ${CMAKE_BINARY_DIR}/jolt)

    add_executable(physics_tests
        tests/main.cpp
        tests/test_physics.cpp
        tests/test_physics_system.cpp
        src/engine/physics/physics_world.cpp
        src/engine/physics/jolt_job_adapter.cpp
        src/engine/systems/physics_system.cpp
        src/engine/systems/motion_system.cpp
        src/engine/world.cpp
        src/engine/components.cpp
        src/job_system.cpp
    )
    target_include_directories(physics_tests PRIVATE src)
    target_compile_options(physics_tests PRIVATE -Wall -Wextra)
    target_link_libraries(physics_tests PRIVATE Jolt Threads::Threads)
    add_test(NAME physics_tests COMMAND physics_tests)
endif()

# ============================================================
# Interactive viewer (GPU + windowing) — only when GLFW is available, so the
# rest of the project still configures on headless/CI machines.
# ============================================================
find_package(glfw3 QUIET)
if(glfw3_FOUND)
set(VIEWER_SOURCES
    src/viewer_main.cpp
    src/renderer/window.cpp
    src/renderer/settings.cpp
    src/engine/clock.cpp
    src/engine/world.cpp
    src/engine/components.cpp
    src/engine/application.cpp
    src/engine/state_stack.cpp
    src/engine/states/playing_state.cpp
    src/engine/states/debug_overlay_state.cpp
    src/engine/mesh_builder.cpp
    src/engine/level_loader.cpp
    src/engine/model_importer.cpp
    src/engine/day_night_cycle.cpp
    src/game/arena_state.cpp
    src/engine/input/input_map.cpp
    src/engine/input/player_input.cpp
    src/engine/camera/orbit_camera_controller.cpp
    src/engine/camera/fly_camera_controller.cpp
    src/engine/camera/scene_camera.cpp
    src/engine/systems/camera_system.cpp
    src/engine/systems/motion_system.cpp
    src/engine/systems/render_system.cpp
    src/engine/systems/dev_control_system.cpp
    src/engine/systems/debug_overlay_system.cpp
    src/engine/systems/day_night_system.cpp
)

if(RT_ENABLE_PHYSICS)
    list(APPEND VIEWER_SOURCES
        src/engine/physics/physics_world.cpp
        src/engine/physics/jolt_job_adapter.cpp
        src/engine/systems/physics_system.cpp
        src/engine/systems/player_system.cpp
        src/engine/systems/shooting_system.cpp
    )
endif()

# Platform-specific renderer backend
if(APPLE)
    enable_language(OBJCXX)
    list(APPEND VIEWER_SOURCES
        src/renderer/metal/metal_renderer.mm
        src/renderer/gamepad_gc.mm
    )
    set(PLATFORM_LIBS
        "-framework Metal"
        "-framework MetalKit"
        "-framework QuartzCore"
        "-framework Cocoa"
        "-framework IOKit"
        "-framework GameController"
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
# Vendored headers (tinygltf/stb/nlohmann) are SYSTEM so our strict warnings
# apply only to our own code, not to libraries we don't maintain.
target_include_directories(viewer SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(viewer PRIVATE glfw Threads::Threads ${PLATFORM_LIBS})
target_compile_definitions(viewer PRIVATE ${PLATFORM_DEFINES})

if(RT_ENABLE_PHYSICS)
    target_link_libraries(viewer PRIVATE Jolt)
    target_compile_definitions(viewer PRIVATE RT_ENABLE_PHYSICS=1)
endif()

# Set Obj-C++ flags for .mm files
if(APPLE)
    set_source_files_properties(
        src/renderer/metal/metal_renderer.mm
        src/renderer/gamepad_gc.mm
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
    # Fetch the submodule if a plain clone/branch sync left it empty.
    if(NOT EXISTS ${IMGUI_DIR}/imgui.cpp)
        find_package(Git QUIET)
        if(GIT_FOUND AND EXISTS ${CMAKE_SOURCE_DIR}/.git)
            message(STATUS "Dear ImGui submodule missing — running 'git submodule update --init'")
            execute_process(
                COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        third_party/imgui
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        endif()
    endif()
    if(NOT EXISTS ${IMGUI_DIR}/imgui.cpp)
        message(FATAL_ERROR
            "Dear ImGui submodule not found at third_party/imgui.\n"
            "Fetch it with:  git submodule update --init --recursive")
    endif()
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
    target_include_directories(viewer SYSTEM PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends)
    target_compile_definitions(viewer PRIVATE RT_ENABLE_IMGUI=1)
endif()
else()
    message(STATUS "glfw3 not found — skipping the interactive viewer target")
endif()
