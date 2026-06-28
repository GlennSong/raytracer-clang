cmake_minimum_required(VERSION 3.20)
project(raytracer LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Threads back the JobSystem (and the offline tracer that now uses it).
find_package(Threads REQUIRED)

# On Windows the UCRT headers gate M_PI (and the other M_* math constants)
# behind _USE_MATH_DEFINES. clang targets the MSVC runtime here, so define it
# project-wide before any translation unit includes <cmath>. No-op elsewhere.
if(WIN32)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

# The Jolt job adapter derives from a JPH polymorphic base. Jolt is built without
# C++ RTTI, so this one TU must match — otherwise its typeinfo references a base
# typeinfo symbol Jolt never emitted (link error). Applies in every target that
# compiles the file (physics_tests, viewer).
set_source_files_properties(src/engine/physics/jolt_job_adapter.cpp
    PROPERTIES COMPILE_OPTIONS "-fno-rtti")

# ============================================================
# engine_core — the platform-free engine library (editor-app plan, Phase A1).
# ECS, systems, cameras, the level document, input: one library linked by
# every host (viewer/runtime, the future editor application, tests, tracer),
# so the editor viewport and the game are the same code by construction.
# Excluded on purpose: window/render *backends* (each host brings its own,
# ADR-0001) and Jolt-coupled systems (hosts that enable physics add them).
# ============================================================
add_library(engine_core STATIC
    src/rt_math.cpp
    src/curve.cpp
    src/log.cpp
    src/job_system.cpp
    src/renderer/settings.cpp
    src/renderer/hosted_window.cpp
    src/engine/clock.cpp
    src/engine/world.cpp
    src/engine/components.cpp
    src/engine/application.cpp
    src/engine/state_stack.cpp
    src/engine/states/playing_state.cpp
    src/engine/states/debug_overlay_state.cpp
    src/engine/states/editor_state.cpp
    src/engine/mesh_builder.cpp
    src/engine/editable_curve.cpp
    src/engine/curve_edit.cpp
    src/engine/animation_path.cpp
    src/engine/handle_source.cpp
    src/engine/path_edit_tool.cpp
    src/engine/asset_manager.cpp
    src/engine/procgen/noise.cpp
    src/engine/procgen/terrain.cpp
    src/engine/procgen/terrain_lod.cpp
    src/engine/procgen/lsystem.cpp
    src/engine/procgen/skeleton.cpp
    src/engine/procgen/tree.cpp
    src/engine/procgen/surface_maps.cpp
    src/engine/procgen/texture_field.cpp
    src/engine/procgen/terrain_field.cpp
    src/engine/procgen/erosion.cpp
    src/engine/procgen/rock.cpp
    src/engine/procgen/scatter.cpp
    src/engine/procgen/sdf.cpp
    src/engine/procgen/city/polygon.cpp
    src/engine/procgen/city/shape_grammar.cpp
    src/engine/procgen/city/parcel.cpp
    src/engine/procgen/city/road_network.cpp
    src/engine/procgen/city/street_kit.cpp
    src/engine/procgen/city/road_mesh.cpp
    src/engine/procgen/city/road_net.cpp
    src/engine/procgen/city/road_constraints.cpp
    src/engine/procgen/city/road_rules.cpp
    src/engine/procgen/city/road_offset.cpp
    src/engine/procgen/city/district.cpp
    src/engine/procgen/city/road_crossings.cpp
    src/engine/procgen/city/city.cpp
    src/engine/level_loader.cpp
    src/engine/level_writer.cpp
    src/engine/model_importer.cpp
    src/engine/camera_store.cpp
    src/engine/editor_bridge.cpp
    src/engine/properties.cpp
    src/engine/component_registry.cpp
    src/engine/property_json.cpp
    src/engine/imgui_properties.cpp
    src/engine/undo_stack.cpp
    src/engine/day_night_cycle.cpp
    src/engine/input/input_map.cpp
    src/engine/input/player_input.cpp
    src/engine/camera/orbit_camera_controller.cpp
    src/engine/camera/fly_camera_controller.cpp
    src/engine/camera/scene_camera.cpp
    src/engine/systems/camera_system.cpp
    src/engine/systems/camera_panel_system.cpp
    src/engine/systems/editor_system.cpp
    src/engine/systems/motion_system.cpp
    src/engine/systems/render_system.cpp
    src/engine/systems/dev_control_system.cpp
    src/engine/systems/debug_overlay_system.cpp
    src/engine/systems/day_night_system.cpp
    src/engine/systems/terrain_lod_system.cpp
)
target_include_directories(engine_core SYSTEM PUBLIC ${CMAKE_SOURCE_DIR}/third_party)
target_compile_options(engine_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(engine_core PUBLIC Threads::Threads)

# Dear ImGui (ADR-0011): core + ImGuizmo compile inside engine_core (the
# panels/gizmos live there); platform backends (GLFW/Metal) attach per host.
option(RT_ENABLE_IMGUI "Build with the Dear ImGui debug UI" OFF)
if(RT_ENABLE_IMGUI)
    set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
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
    target_sources(engine_core PRIVATE
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
    )
    target_include_directories(engine_core SYSTEM PUBLIC ${IMGUI_DIR})
    target_compile_definitions(engine_core PUBLIC RT_ENABLE_IMGUI=1)

    set(IMGUIZMO_DIR ${CMAKE_SOURCE_DIR}/third_party/ImGuizmo)
    if(EXISTS ${IMGUIZMO_DIR}/src/ImGuizmo.cpp)
        target_sources(engine_core PRIVATE ${IMGUIZMO_DIR}/src/ImGuizmo.cpp)
        target_include_directories(engine_core SYSTEM PRIVATE ${IMGUIZMO_DIR}/src)
    else()
        message(WARNING "ImGuizmo submodule missing - editor gizmos disabled; "
                        "run: git submodule update --init third_party/ImGuizmo")
    endif()
endif()

# ============================================================
# Lua scripting (ADR-0023) — pure ANSI-C, builds and runs headless like Jolt, so
# it compiles into engine_core (no GPU/OS deps). Sealed behind ScriptVM; the
# sandbox opens only the pure stdlibs. The lua/lua source ships no build, so we
# compile its core ourselves, excluding the lua.c/luac.c stand-alone mains.
# ============================================================
option(RT_ENABLE_SCRIPTING "Build the Lua scripting layer" ON)
if(RT_ENABLE_SCRIPTING)
    set(LUA_DIR ${CMAKE_SOURCE_DIR}/third_party/lua)
    if(NOT EXISTS ${LUA_DIR}/lua.h)
        find_package(Git QUIET)
        if(GIT_FOUND AND EXISTS ${CMAKE_SOURCE_DIR}/.git)
            message(STATUS "Lua submodule missing — running 'git submodule update --init'")
            execute_process(
                COMMAND ${GIT_EXECUTABLE} submodule update --init third_party/lua
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        endif()
    endif()
    if(NOT EXISTS ${LUA_DIR}/lua.h)
        message(FATAL_ERROR
            "Lua submodule not found at third_party/lua.\n"
            "Fetch it with:  git submodule update --init third_party/lua\n"
            "or configure with -DRT_ENABLE_SCRIPTING=OFF to build without scripting.")
    endif()

    file(GLOB LUA_SOURCES ${LUA_DIR}/*.c)
    list(REMOVE_ITEM LUA_SOURCES ${LUA_DIR}/lua.c ${LUA_DIR}/luac.c)
    add_library(lua_static STATIC ${LUA_SOURCES})
    target_include_directories(lua_static SYSTEM PUBLIC ${LUA_DIR})
    target_compile_options(lua_static PRIVATE -w)   # vendored C; not our warnings

    target_sources(engine_core PRIVATE
        src/engine/scripting/script_vm.cpp
        src/engine/scripting/procgen_bindings.cpp
        src/engine/scripting/gameplay_bindings.cpp
        src/engine/scripting/script_system.cpp
    )
    target_link_libraries(engine_core PUBLIC lua_static)
    target_compile_definitions(engine_core PUBLIC RT_ENABLE_SCRIPTING=1)
endif()

# Offline path-tracer sources (the tracer's own scene/camera/image stack).
set(CORE_SOURCES
    src/image.cpp
    src/camera.cpp
    src/geometry.cpp
    src/material.cpp
    src/scene.cpp
    src/kdtree.cpp
    src/instance.cpp
    src/level_scene.cpp
    src/path_tracer.cpp
)

# ============================================================
# Offline path tracer (no GPU dependencies)
# ============================================================
add_executable(raytracer
    src/main.cpp
    ${CORE_SOURCES}
)
target_compile_options(raytracer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(raytracer PRIVATE engine_core)

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
    tests/test_camera_store.cpp
    tests/test_lens.cpp
    tests/test_level_scene.cpp
    tests/test_pbr_lighting.cpp
    tests/test_level_writer.cpp
    tests/test_hosted_window.cpp
    tests/test_properties.cpp
    tests/test_undo.cpp
    tests/test_hierarchy.cpp
    tests/test_job_system.cpp
    tests/test_frustum.cpp
    tests/test_day_night.cpp
    tests/test_cube_faces.cpp
    tests/test_path_tracer.cpp
    tests/test_asset_manager.cpp
    tests/test_mesh_builder.cpp
    tests/test_noise.cpp
    tests/test_terrain.cpp
    tests/test_lsystem.cpp
    tests/test_tree.cpp
    tests/test_rock.cpp
    tests/test_scatter.cpp
    tests/test_sdf.cpp
    tests/test_city.cpp
    tests/test_surface_maps.cpp
    tests/test_model_importer.cpp
    tests/test_instancing.cpp
    tests/test_instance_tlas.cpp
    tests/test_texture_field.cpp
    tests/test_terrain_field.cpp
    tests/test_winding.cpp
    tests/test_road_constraints.cpp
    tests/test_road_layers.cpp
    tests/test_road_offset.cpp
    tests/test_road_crossings.cpp
    tests/test_road_net.cpp
    tests/test_handle_source.cpp
    tests/test_path_edit_tool.cpp
    tests/test_editable_curve.cpp
    tests/test_curve.cpp
    tests/test_curve_edit.cpp
    tests/test_curb_return.cpp
    tests/test_animation_path.cpp
    tests/test_terrain_lod.cpp
    ${CORE_SOURCES}
)
target_compile_options(run_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(run_tests PRIVATE engine_core)
if(RT_ENABLE_SCRIPTING)
    target_sources(run_tests PRIVATE
        tests/test_script_vm.cpp
        tests/test_script_system.cpp
        tests/test_gun_script.cpp
        tests/test_flora.cpp)
    # Lets the gun test read the actual shipped assets/scripts/gun.lua.
    target_compile_definitions(run_tests PRIVATE RT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
endif()
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
# Host-side only: the GLFW window backend, the game, and (per-platform) the
# render backend. Everything engine lives in engine_core.
set(VIEWER_SOURCES
    src/viewer_main.cpp
    src/renderer/window.cpp
    src/game/arena_state.cpp
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
else()
    # Non-Apple (Linux/Windows): use the Vulkan backend when the SDK is present,
    # else fall back to NullRenderer so the target still links (e.g. headless CI
    # with no Vulkan SDK). Exactly one of the two provides Renderer::create().
    # ADR-0057; see src/renderer/vulkan/AGENTS.md.
    find_package(Vulkan QUIET)
    if(Vulkan_FOUND)
        list(APPEND VIEWER_SOURCES src/renderer/vulkan/vulkan_renderer.cpp)
        set(PLATFORM_LIBS Vulkan::Vulkan)

        # Compile GLSL -> SPIR-V offline (ADR-0057): keeps the runtime dependency
        # -free (no libshaderc). glslc ships with the Vulkan SDK.
        if(NOT Vulkan_GLSLC_EXECUTABLE)
            message(FATAL_ERROR
                "glslc not found — the Vulkan SDK's GLSL compiler is required to "
                "build the viewer's shaders. Install the Vulkan SDK or configure "
                "without the viewer.")
        endif()
        set(VK_SHADER_SRC_DIR ${CMAKE_SOURCE_DIR}/shaders/vulkan)
        set(VK_SHADER_OUT_DIR ${CMAKE_BINARY_DIR}/shaders/vulkan)
        file(MAKE_DIRECTORY ${VK_SHADER_OUT_DIR})
        set(VK_SHADERS mesh.vert mesh.frag mesh_shadow.vert sky.vert sky.frag
                       composite.vert composite.frag bloom.frag)
        set(VK_SPV_FILES "")
        foreach(shader ${VK_SHADERS})
            add_custom_command(
                OUTPUT ${VK_SHADER_OUT_DIR}/${shader}.spv
                COMMAND ${Vulkan_GLSLC_EXECUTABLE}
                        ${VK_SHADER_SRC_DIR}/${shader}
                        -o ${VK_SHADER_OUT_DIR}/${shader}.spv
                DEPENDS ${VK_SHADER_SRC_DIR}/${shader}
                COMMENT "glslc ${shader} -> ${shader}.spv"
                VERBATIM)
            list(APPEND VK_SPV_FILES ${VK_SHADER_OUT_DIR}/${shader}.spv)
        endforeach()
        add_custom_target(viewer_vulkan_shaders DEPENDS ${VK_SPV_FILES})

        # RT_HAVE_VULKAN also gates the Vulkan surface code in window.cpp, so the
        # windowing seam (Window::createVulkanSurface) compiles only where the SDK
        # exists and macOS never pulls in vulkan.h. RT_VULKAN_SHADER_DIR is where
        # the backend loads the compiled .spv at runtime.
        set(PLATFORM_DEFINES RT_HAVE_VULKAN=1 "RT_VULKAN_SHADER_DIR=\"${VK_SHADER_OUT_DIR}\"")
        message(STATUS "Vulkan found — building the viewer with the Vulkan backend")
    else()
        list(APPEND VIEWER_SOURCES src/renderer/null_renderer.cpp)
        message(STATUS "Vulkan not found — viewer uses NullRenderer (no GPU output)")
    endif()
endif()

add_executable(viewer
    ${VIEWER_SOURCES}
)

target_compile_options(viewer PRIVATE -Wall -Wextra -Wpedantic)
# Vendored headers (tinygltf/stb/nlohmann) are SYSTEM so our strict warnings
# apply only to our own code, not to libraries we don't maintain.
target_include_directories(viewer SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(viewer PRIVATE engine_core glfw Threads::Threads ${PLATFORM_LIBS})
target_compile_definitions(viewer PRIVATE ${PLATFORM_DEFINES})
# Build the SPIR-V shaders before the viewer that loads them (Vulkan builds only).
if(TARGET viewer_vulkan_shaders)
    add_dependencies(viewer viewer_vulkan_shaders)
endif()

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
if(RT_ENABLE_IMGUI)
    # Platform backends for the standalone viewer (GLFW + Metal). The hosted
    # editor needs neither: HostedWindow feeds ImGui's io directly and the
    # Metal backend submits the draw data either way.
    target_sources(viewer PRIVATE
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    )
    if(APPLE)
        target_sources(viewer PRIVATE ${IMGUI_DIR}/backends/imgui_impl_metal.mm)
        set_source_files_properties(
            ${IMGUI_DIR}/backends/imgui_impl_metal.mm
            PROPERTIES COMPILE_FLAGS "-fobjc-arc"
        )
    endif()
    target_include_directories(viewer SYSTEM PRIVATE ${IMGUI_DIR}/backends)
endif()
else()
    message(STATUS "glfw3 not found — skipping the interactive viewer target")
endif()

# ============================================================
# Editor application (editor-app plan, Phase A2): a Qt shell hosting the
# engine through the HostedWindow seam. Builds wherever Qt6 is installed;
# the viewport renders on platforms with a GPU backend (Metal today) and is
# a NullRenderer elsewhere (UI still runs, e.g. for Linux development).
# ============================================================
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
    add_executable(editor_app
        src/editor_app/editor_main.cpp
        src/editor_app/property_inspector.cpp
        src/game/arena_state.cpp
    )
    target_compile_options(editor_app PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(editor_app PRIVATE engine_core Qt6::Widgets Threads::Threads)

    if(RT_ENABLE_PHYSICS)
        target_sources(editor_app PRIVATE
            src/engine/physics/physics_world.cpp
            src/engine/physics/jolt_job_adapter.cpp
            src/engine/systems/physics_system.cpp
            src/engine/systems/player_system.cpp
            src/engine/systems/shooting_system.cpp
        )
        target_link_libraries(editor_app PRIVATE Jolt)
        # arena_state.cpp gates its physics/player systems on this define;
        # without it the editor's Play mode silently loses the first-person
        # controller (the viewer always had it — see the viewer block above).
        target_compile_definitions(editor_app PRIVATE RT_ENABLE_PHYSICS=1)
    endif()

    if(APPLE)
        target_sources(editor_app PRIVATE
            src/renderer/metal/metal_renderer.mm
            src/renderer/gamepad_gc.mm
        )
        target_link_libraries(editor_app PRIVATE ${PLATFORM_LIBS})
        if(RT_ENABLE_IMGUI)
            # The Metal backend submits ImGui draw data; no GLFW backend here.
            target_sources(editor_app PRIVATE ${IMGUI_DIR}/backends/imgui_impl_metal.mm)
        endif()
    else()
        target_sources(editor_app PRIVATE src/renderer/null_renderer.cpp)
    endif()
    # Headless widget-interaction tests for the generated inspector (the
    # path that once crashed via a dangling visitor capture). Offscreen Qt.
    add_executable(editor_qt_tests
        tests/test_property_inspector_qt.cpp
        src/editor_app/property_inspector.cpp
        src/renderer/null_renderer.cpp   # satisfies Renderer::create; never renders
    )
    target_compile_options(editor_qt_tests PRIVATE -Wall -Wextra)
    target_link_libraries(editor_qt_tests PRIVATE engine_core Qt6::Widgets Threads::Threads)
    add_test(NAME editor_qt_tests COMMAND editor_qt_tests)

    message(STATUS "Qt6 found — building the editor application")
else()
    message(STATUS "Qt6 not found — skipping the editor application")
endif()
