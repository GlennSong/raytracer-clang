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
    src/engine/event_bus.cpp
    src/engine/debug_draw.cpp
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
    src/engine/procgen/vehicle_mesh.cpp
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
    src/engine/procgen/city/triangulate.cpp
    src/engine/procgen/city/metro.cpp
    src/engine/procgen/city/buildability.cpp
    src/engine/procgen/city/road_net.cpp
    src/engine/procgen/city/road_constraints.cpp
    src/engine/procgen/city/road_rules.cpp
    src/engine/procgen/city/road_offset.cpp
    src/engine/procgen/city/district.cpp
    src/engine/procgen/city/road_crossings.cpp
    src/engine/procgen/city/city.cpp
    src/engine/procgen/city/architect.cpp
    src/engine/procgen/city/city_lots.cpp
    src/engine/ai/nav_graph.cpp
    src/engine/ai/pathfind.cpp
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
    src/engine/camera/follow_camera_controller.cpp
    src/engine/camera/scene_camera.cpp
    src/engine/systems/camera_system.cpp
    src/engine/systems/camera_panel_system.cpp
    src/engine/systems/editor_system.cpp
    src/engine/systems/motion_system.cpp
    src/engine/anim/skeleton.cpp
    src/engine/anim/mannequin.cpp
    src/engine/anim/skinned_mesh.cpp
    src/engine/anim/skin_import.cpp
    src/engine/audio/audio_engine.cpp
    src/engine/audio/sfx.cpp
    src/engine/systems/audio_system.cpp
    src/engine/systems/render_system.cpp
    src/engine/systems/debug_draw_system.cpp
    src/engine/systems/dev_control_system.cpp
    src/engine/systems/debug_overlay_system.cpp
    src/engine/systems/day_night_system.cpp
    src/engine/systems/terrain_lod_system.cpp
    # NOTE: the agent-based city simulation (ADR-0060…0066) is NOT here. It is an
    # APPLICATION of the engine, built as its own `citysim` static library below
    # (src/apps/citysim). engine_core is the platform-free, city-free engine; a
    # host that wants the living world links `citysim` on top. Keeping it out of
    # the core is what lets a plain engine app carry none of the city's code.
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
    # lua.c/luac.c are the standalone interpreter/compiler mains; onelua.c is the
    # single-file amalgamation that #includes every other .c — leaving it in the
    # archive defines every Lua symbol twice. Desktop linkers never happened to
    # pull that member, but wasm-ld does and the web link exploded in duplicates.
    list(REMOVE_ITEM LUA_SOURCES ${LUA_DIR}/lua.c ${LUA_DIR}/luac.c ${LUA_DIR}/onelua.c)
    add_library(lua_static STATIC ${LUA_SOURCES})
    target_include_directories(lua_static SYSTEM PUBLIC ${LUA_DIR})
    target_compile_options(lua_static PRIVATE -w)   # vendored C; not our warnings

    # NOTE: the citysim-specific Lua readers (agent_goals, vehicle_body) are NOT
    # here — they read citysim types, so they live in the `citysim` library
    # (src/apps/citysim/scripting) to keep engine_core free of any app dependency.
    target_sources(engine_core PRIVATE
        src/engine/scripting/script_vm.cpp
        src/engine/scripting/procgen_bindings.cpp
        src/engine/scripting/gameplay_bindings.cpp
        src/engine/scripting/script_system.cpp
        src/engine/scripting/vehicle_spec.cpp
    )
    target_link_libraries(engine_core PUBLIC lua_static)
    target_compile_definitions(engine_core PUBLIC RT_ENABLE_SCRIPTING=1)
endif()

# ============================================================
# citysim — the living-world APPLICATION library (ADR-0060…0066). The agent
# city sim, traffic, places, and its render bridge: an application built ON the
# engine, not part of it. Links engine_core (the only arrow — citysim depends on
# the engine, never the reverse), so a host that wants the living world links
# `citysim`; a plain engine app links only engine_core and carries none of this.
# Headless-safe: no GPU/windowing here (the Jolt-coupled bridges — city_physics
# /vehicles/walkers/player_body — stay in RT_PHYSICS_HOST_SOURCES, compiled per
# host alongside Jolt, exactly as engine_core keeps Jolt systems out).
# ============================================================
add_library(citysim STATIC
    src/apps/citysim/traffic_signal.cpp
    src/apps/citysim/traffic_rules.cpp
    src/apps/citysim/city_goals.cpp
    src/apps/citysim/places.cpp          # places layer: buildings → routable POIs (ADR-0066)
    src/apps/citysim/ped_graph.cpp       # pedestrian nav-graph: door-to-door walking (ADR-0066)
    src/apps/citysim/relationships.cpp   # surface-level social graph (ADR-0066)
    src/apps/citysim/city_sim.cpp
    src/apps/citysim/city_render.cpp
    src/apps/citysim/city_spectate.cpp   # spectate camera: cycle + follow NPCs (headless-safe)
    src/apps/citysim/city_meshes.cpp
)
target_compile_options(citysim PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(citysim PUBLIC engine_core)
if(RT_ENABLE_SCRIPTING)
    # The citysim-specific Lua readers (moved out of engine_core): they turn Lua
    # goal/fleet tables into citysim types, so they belong to the app, not the core.
    target_sources(citysim PRIVATE
        src/apps/citysim/scripting/agent_goals.cpp
        src/apps/citysim/scripting/vehicle_body.cpp
    )
endif()

# ============================================================
# Audio (ADR-0069) — miniaudio, a single-header C submodule pinned at a
# release tag, sealed behind AudioEngine. Cross-platform and device-optional
# (null-device fallback, or Manual mode with no device at all), so it builds
# and tests headless. OFF leaves the AudioEngine stub in place (silence) —
# audio_engine.cpp compiles either way, so the engine API is always present.
# Skipped under Emscripten until the web audio path is designed.
# ============================================================
option(RT_ENABLE_AUDIO "Build the miniaudio-backed audio engine" ON)
if(RT_ENABLE_AUDIO AND NOT EMSCRIPTEN)
    set(MINIAUDIO_DIR ${CMAKE_SOURCE_DIR}/third_party/miniaudio)
    if(NOT EXISTS ${MINIAUDIO_DIR}/miniaudio.h)
        find_package(Git QUIET)
        if(GIT_FOUND AND EXISTS ${CMAKE_SOURCE_DIR}/.git)
            message(STATUS "miniaudio submodule missing — running 'git submodule update --init'")
            execute_process(
                COMMAND ${GIT_EXECUTABLE} submodule update --init third_party/miniaudio
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        endif()
    endif()
    if(NOT EXISTS ${MINIAUDIO_DIR}/miniaudio.h)
        message(FATAL_ERROR
            "miniaudio submodule not found at third_party/miniaudio.\n"
            "Fetch it with:  git submodule update --init third_party/miniaudio\n"
            "or configure with -DRT_ENABLE_AUDIO=OFF to build without audio.")
    endif()
    # The single implementation TU, compiled as C with warnings off (the Lua
    # pattern). dl/pthread/m are miniaudio's documented Linux link deps.
    add_library(miniaudio_static STATIC src/engine/audio/miniaudio_impl.c)
    target_include_directories(miniaudio_static SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
    target_compile_options(miniaudio_static PRIVATE -w)
    target_link_libraries(miniaudio_static PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
    if(NOT APPLE AND NOT WIN32)
        target_link_libraries(miniaudio_static PUBLIC m)
    endif()
    if(APPLE)
        target_link_libraries(miniaudio_static PUBLIC
            "-framework CoreFoundation" "-framework CoreAudio"
            "-framework AudioToolbox")
    endif()

    target_link_libraries(engine_core PUBLIC miniaudio_static)
    target_compile_definitions(engine_core PUBLIC RT_ENABLE_AUDIO=1)
endif()

# ============================================================
# Tracy profiler (ADR-0068) — off by default. When ON: compile the Tracy
# client into engine_core and define RT_ENABLE_PROFILER, which turns the
# RT_PROFILE_* macros (src/profile.h) into real zones. Capture by connecting
# the Tracy UI to the running process (works headless, over the network).
# ============================================================
option(RT_ENABLE_PROFILER "Build with the Tracy profiler client" OFF)
if(RT_ENABLE_PROFILER)
    set(TRACY_DIR ${CMAKE_SOURCE_DIR}/third_party/tracy)
    if(NOT EXISTS ${TRACY_DIR}/CMakeLists.txt)
        find_package(Git QUIET)
        if(GIT_FOUND AND EXISTS ${CMAKE_SOURCE_DIR}/.git)
            message(STATUS "Tracy submodule missing — running 'git submodule update --init'")
            execute_process(
                COMMAND ${GIT_EXECUTABLE} submodule update --init third_party/tracy
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        endif()
    endif()
    if(NOT EXISTS ${TRACY_DIR}/CMakeLists.txt)
        message(FATAL_ERROR
            "Tracy submodule not found at third_party/tracy.\n"
            "Fetch it with:  git submodule update --init third_party/tracy\n"
            "or configure with -DRT_ENABLE_PROFILER=OFF.")
    endif()
    # Record only while a profiler UI is connected, so an instrumented build
    # left running doesn't grow an unbounded event buffer.
    set(TRACY_ON_DEMAND ON CACHE BOOL "" FORCE)
    add_subdirectory(third_party/tracy ${CMAKE_BINARY_DIR}/tracy)
    target_link_libraries(engine_core PUBLIC TracyClient)
    target_compile_definitions(engine_core PUBLIC RT_ENABLE_PROFILER=1)
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
    tests/test_event_bus.cpp
    tests/test_debug_draw.cpp
    tests/test_skeleton.cpp
    tests/test_skin.cpp
    tests/test_input_map.cpp
    tests/test_player_input.cpp
    tests/test_camera.cpp
    tests/test_follow_camera.cpp
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
    tests/test_vehicle_mesh.cpp
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
    tests/test_nav_graph.cpp
    tests/test_pathfind.cpp
    tests/test_perception.cpp
    tests/test_agent_memory.cpp
    tests/test_state_machine.cpp
    tests/test_cascade_fit.cpp
    tests/test_fall_respawn.cpp
    tests/test_traffic_signal.cpp
    tests/test_traffic_rules.cpp
    tests/test_city_sim.cpp
    tests/test_city_goals.cpp
    tests/test_city_signals.cpp
    tests/test_city_perception.cpp
    tests/test_driver_agent.cpp
    tests/test_lane_follow.cpp
    tests/test_traffic_sense.cpp
    tests/test_city_steering.cpp
    tests/test_city_driver_fsm.cpp
    tests/test_city_fleet.cpp
    tests/test_city_drive.cpp
    tests/test_city_render.cpp
    tests/test_car_lamps.cpp
    tests/test_city_flow.cpp
    tests/test_city_spectate.cpp
    tests/test_places.cpp
    tests/test_screen_project.cpp
    tests/test_relationships.cpp
    tests/test_ped_graph.cpp
    tests/test_city_generated.cpp
    tests/test_city_lots.cpp
    ${CORE_SOURCES}
)
target_compile_options(run_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(run_tests PRIVATE citysim engine_core)
if(RT_ENABLE_SCRIPTING)
    target_sources(run_tests PRIVATE
        tests/test_script_vm.cpp
        tests/test_script_system.cpp
        tests/test_gun_script.cpp
        tests/test_flora.cpp
        tests/test_agent_goals.cpp
        tests/test_vehicle_body.cpp)
    # Lets the gun test read the actual shipped assets/scripts/gun.lua.
    target_compile_definitions(run_tests PRIVATE RT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
endif()
if(RT_ENABLE_AUDIO AND NOT EMSCRIPTEN)
    # Headless audio tests: Manual mode (no device), the mix pumped by hand.
    target_sources(run_tests PRIVATE tests/test_audio.cpp)
endif()
enable_testing()
add_test(NAME unit_tests COMMAND run_tests)

# ============================================================
# Headless demos (tools/): small programs that show the engine systems in
# action without a GPU or an audio device — the path tracer renders the
# DebugDraw ribbons to a PNG, and Manual-mode audio mixes a spatial scene to
# a WAV. Handy on CI/remote machines; also fine on a dev box.
# ============================================================
option(RT_BUILD_DEMOS "Build the headless demo tools" ON)
if(RT_BUILD_DEMOS)
    add_executable(debug_draw_demo tools/debug_draw_demo.cpp ${CORE_SOURCES})
    target_include_directories(debug_draw_demo PRIVATE src)
    target_compile_options(debug_draw_demo PRIVATE -Wall -Wextra)
    target_link_libraries(debug_draw_demo PRIVATE engine_core)

    # Three posed mannequins path-traced to a still (character-posing-plan P0).
    add_executable(pose_demo tools/pose_demo.cpp ${CORE_SOURCES})
    target_include_directories(pose_demo PRIVATE src)
    target_compile_options(pose_demo PRIVATE -Wall -Wextra)
    target_link_libraries(pose_demo PRIVATE engine_core)

    # Rigid vs CPU-skinned bend, side by side (character-posing-plan P2).
    add_executable(skin_demo tools/skin_demo.cpp ${CORE_SOURCES})
    target_include_directories(skin_demo PRIVATE src)
    target_compile_options(skin_demo PRIVATE -Wall -Wextra)
    target_link_libraries(skin_demo PRIVATE engine_core)

    if(RT_ENABLE_AUDIO AND NOT EMSCRIPTEN)
        add_executable(audio_demo tools/audio_demo.cpp)
        target_include_directories(audio_demo PRIVATE src)
        target_compile_options(audio_demo PRIVATE -Wall -Wextra)
        target_link_libraries(audio_demo PRIVATE engine_core)
    endif()
endif()

# The physics->sound chain, audible (ADR-0071): Jolt contacts -> EventBus ->
# procedural impact cues -> a WAV. Needs both physics and audio, so it is
# declared with the physics block below (after the Jolt target exists).

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

    # The physics-host source set every host target (viewer, editor_app,
    # viewer_web) compiles alongside engine_core: the Jolt world + job adapter,
    # the gameplay systems that need it, drivable vehicles, and the living-world
    # citysim bridges (ADR-0062/0063). Defined once so the three targets cannot
    # drift apart.
    set(RT_PHYSICS_HOST_SOURCES
        src/engine/physics/physics_world.cpp
        src/engine/physics/jolt_job_adapter.cpp
        src/engine/systems/physics_system.cpp
        src/engine/systems/player_system.cpp
        src/engine/systems/shooting_system.cpp
        src/engine/systems/vehicle_system.cpp
        src/apps/citysim/city_physics.cpp   # kinematic colliders for AI cars (needs Jolt)
        src/apps/citysim/city_vehicles.cpp  # NPC cars as real engine Vehicles (ADR-0062)
        src/apps/citysim/city_walkers.cpp   # NPC walkers as physics characters (ADR-0062)
        src/apps/citysim/city_player_body.cpp  # player renders as a person in third person (ADR-0064)
    )

    add_executable(physics_tests
        tests/main.cpp
        tests/test_physics.cpp
        tests/test_physics_system.cpp
        src/engine/physics/physics_world.cpp
        src/engine/physics/jolt_job_adapter.cpp
        src/engine/systems/physics_system.cpp
        src/engine/systems/motion_system.cpp
        src/engine/event_bus.cpp
        src/engine/world.cpp
        src/engine/components.cpp
        src/job_system.cpp
    )
    target_include_directories(physics_tests PRIVATE src)
    target_compile_options(physics_tests PRIVATE -Wall -Wextra)
    target_link_libraries(physics_tests PRIVATE Jolt Threads::Threads)
    add_test(NAME physics_tests COMMAND physics_tests)

    # The audible physics demo (see the demos block above for the rationale).
    if(RT_BUILD_DEMOS AND RT_ENABLE_AUDIO AND NOT EMSCRIPTEN)
        add_executable(impact_demo
            tools/impact_demo.cpp
            src/engine/physics/physics_world.cpp
            src/engine/physics/jolt_job_adapter.cpp
            src/engine/systems/physics_system.cpp
        )
        target_include_directories(impact_demo PRIVATE src)
        target_compile_options(impact_demo PRIVATE -Wall -Wextra)
        target_link_libraries(impact_demo PRIVATE engine_core Jolt Threads::Threads)
    endif()
endif()

# ============================================================
# Vulkan SPIR-V shaders (ADR-0057). Compile GLSL -> SPIR-V offline with glslc
# so the runtime stays dependency-free (no libshaderc). Shared by the viewer
# and the editor app, so build the target once (guard on NOT TARGET) and export
# VK_SHADER_OUT_DIR to the caller's scope. Requires find_package(Vulkan) first.
# ============================================================
function(rt_build_vulkan_shaders)
    set(VK_SHADER_OUT_DIR ${CMAKE_BINARY_DIR}/shaders/vulkan)
    set(VK_SHADER_OUT_DIR ${VK_SHADER_OUT_DIR} PARENT_SCOPE)
    if(TARGET vulkan_shaders)
        return()
    endif()
    if(NOT Vulkan_GLSLC_EXECUTABLE)
        message(FATAL_ERROR
            "glslc not found — the Vulkan SDK's GLSL compiler is required to "
            "build the Vulkan shaders. Install the Vulkan SDK or configure "
            "without the Vulkan targets.")
    endif()
    set(VK_SHADER_SRC_DIR ${CMAKE_SOURCE_DIR}/shaders/vulkan)
    file(MAKE_DIRECTORY ${VK_SHADER_OUT_DIR})
    set(VK_SHADERS mesh.vert mesh.frag mesh_wire.frag mesh_shadow.vert terrain.vert
                   sky.vert sky.frag composite.vert composite.frag bloom.frag
                   ssao.frag ssao_blur.frag ssr.frag dof.frag brdf_lut.frag)
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
    add_custom_target(vulkan_shaders DEPENDS ${VK_SPV_FILES})
endfunction()

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
    list(APPEND VIEWER_SOURCES ${RT_PHYSICS_HOST_SOURCES})
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

        # Compile the SPIR-V shaders (shared target; sets VK_SHADER_OUT_DIR).
        rt_build_vulkan_shaders()

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
target_link_libraries(viewer PRIVATE citysim engine_core glfw Threads::Threads ${PLATFORM_LIBS})
target_compile_definitions(viewer PRIVATE ${PLATFORM_DEFINES})
# Build the SPIR-V shaders before the viewer that loads them (Vulkan builds only).
if(TARGET vulkan_shaders)
    add_dependencies(viewer vulkan_shaders)
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
    elseif(Vulkan_FOUND)
        # ImGui's Vulkan backend; vulkan_renderer.cpp drives it (ADR-0057).
        target_sources(viewer PRIVATE ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp)
        target_link_libraries(viewer PRIVATE Vulkan::Vulkan)
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
    target_link_libraries(editor_app PRIVATE citysim engine_core Qt6::Widgets Threads::Threads)

    if(RT_ENABLE_PHYSICS)
        target_sources(editor_app PRIVATE ${RT_PHYSICS_HOST_SOURCES})
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
        # Non-Apple: render the viewport with the Vulkan backend when the SDK is
        # present (ADR-0057), wiring the Qt viewport's native handle into a
        # VkSurfaceKHR via vulkan_viewport.cpp. Without Vulkan, fall back to the
        # NullRenderer so the shell (panels, document loop) still builds and runs.
        find_package(Vulkan QUIET)
        if(Vulkan_FOUND)
            rt_build_vulkan_shaders()   # shared target; sets VK_SHADER_OUT_DIR
            target_sources(editor_app PRIVATE
                src/renderer/vulkan/vulkan_renderer.cpp
                src/editor_app/vulkan_viewport.cpp
            )
            target_link_libraries(editor_app PRIVATE Vulkan::Vulkan)
            target_compile_definitions(editor_app PRIVATE
                RT_HAVE_VULKAN=1
                "RT_VULKAN_SHADER_DIR=\"${VK_SHADER_OUT_DIR}\"")
            add_dependencies(editor_app vulkan_shaders)
            if(RT_ENABLE_IMGUI)
                # The Vulkan backend submits ImGui draw data (no GLFW backend).
                target_sources(editor_app PRIVATE
                    ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp)
                target_include_directories(editor_app SYSTEM PRIVATE
                    ${IMGUI_DIR}/backends)
            endif()
            message(STATUS "Vulkan found — editor viewport uses the Vulkan backend")
        else()
            target_sources(editor_app PRIVATE src/renderer/null_renderer.cpp)
            message(STATUS "Vulkan not found — editor uses NullRenderer (no GPU output)")
        endif()
    endif()

    # Deploy the Qt platform plugin next to editor_app so it launches
    # self-contained on Windows. vcpkg copies the Qt DLLs beside the exe but not
    # the plugins, and Qt searches <exe-dir>/platforms/ for the windowing
    # integration; without it the app aborts with "Could not find the Qt platform
    # plugin 'windows'". The imported target resolves to the config-matching
    # (debug/release) plugin. No-op on macOS/Linux (handled by frameworks/rpath).
    if(WIN32 AND TARGET Qt6::QWindowsIntegrationPlugin)
        add_custom_command(TARGET editor_app POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                    $<TARGET_FILE_DIR:editor_app>/platforms
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:Qt6::QWindowsIntegrationPlugin>
                    $<TARGET_FILE_DIR:editor_app>/platforms/
            VERBATIM)
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

# ============================================================
# Web target (WebGPU + WebAssembly) — built under the Emscripten toolchain
# (`emcmake cmake -S . -B build-web`). ADR-0058. The WebGPU backend
# (src/renderer/webgpu/webgpu_renderer.cpp) is the web implementation of the
# `Renderer` seam and provides Renderer::create(); the GLFW window backend is
# Emscripten's built-in GLFW3 shim (-sUSE_GLFW=3), so window.cpp is reused as-is.
# Single-threaded by design (Application forces JobSystem synchronous mode under
# __EMSCRIPTEN__), so the build needs no -pthread / SharedArrayBuffer / COOP-COEP.
# Physics/Qt/Vulkan paths above are skipped on this toolchain (their
# find_package calls don't resolve), so only this block needs adding.
# ============================================================
if(EMSCRIPTEN)
    set(WEB_SOURCES
        src/web_main.cpp
        src/renderer/window.cpp
        src/renderer/webgpu/webgpu_renderer.cpp
        src/game/arena_state.cpp
    )
    # engine_core's terrain_lod_system references PhysicsWorld, so the host must
    # bring the physics sources + Jolt (same as the desktop viewer). Jolt is
    # cross-platform C++ and compiles to wasm.
    if(RT_ENABLE_PHYSICS)
        # Same set as the desktop viewer — Jolt compiles to wasm.
        list(APPEND WEB_SOURCES ${RT_PHYSICS_HOST_SOURCES})
    endif()
    # WGSL sources live in shaders/webgpu/*.wgsl; a generated header embeds
    # them into the backend (no runtime fetch). Regenerates when any .wgsl or
    # the embed script changes.
    file(GLOB RT_WGSL_SOURCES ${CMAKE_SOURCE_DIR}/shaders/webgpu/*.wgsl)
    set(RT_WGSL_GEN ${CMAKE_BINARY_DIR}/generated/webgpu_shaders_gen.h)
    add_custom_command(OUTPUT ${RT_WGSL_GEN}
        COMMAND ${CMAKE_COMMAND}
                -DSHADER_DIR=${CMAKE_SOURCE_DIR}/shaders/webgpu
                -DOUT=${RT_WGSL_GEN}
                -P ${CMAKE_SOURCE_DIR}/cmake/embed_wgsl.cmake
        DEPENDS ${RT_WGSL_SOURCES} ${CMAKE_SOURCE_DIR}/cmake/embed_wgsl.cmake
        COMMENT "Embedding WGSL shaders")
    add_custom_target(webgpu_shaders DEPENDS ${RT_WGSL_GEN})

    add_executable(viewer_web ${WEB_SOURCES})
    add_dependencies(viewer_web webgpu_shaders)
    target_include_directories(viewer_web PRIVATE ${CMAKE_BINARY_DIR}/generated)
    if(RT_ENABLE_PHYSICS)
        target_link_libraries(viewer_web PRIVATE Jolt)
        target_compile_definitions(viewer_web PRIVATE RT_ENABLE_PHYSICS=1)
    endif()
    # WebGPU comes from the emdawnwebgpu port (Dawn's standardized webgpu.h) —
    # the legacy -sUSE_WEBGPU binding was removed in Emscripten 6.x. Default is
    # the remote port (auto-downloaded); override with a local emdawnwebgpu.port.py
    # path (-DRT_EMDAWN_PORT=...) for offline builds. The port flag is needed at
    # both compile (header include path) and link (JS glue).
    set(RT_EMDAWN_PORT "emdawnwebgpu" CACHE STRING
        "emdawnwebgpu port name, or path to a local emdawnwebgpu.port.py")
    target_compile_options(viewer_web PRIVATE -Wall -Wextra "--use-port=${RT_EMDAWN_PORT}")
    target_include_directories(viewer_web SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
    target_link_libraries(viewer_web PRIVATE citysim engine_core)

    # The GLFW shim, a growable heap, and MODULARIZE so the page instantiates the
    # module by name (web/viewer.html). ASYNCIFY lets the backend await the async
    # WebGPU adapter/device request inside the synchronous Renderer::initialize().
    # The assets are baked into the virtual FS so the level/scripts load via FS.
    target_link_options(viewer_web PRIVATE
        "--use-port=${RT_EMDAWN_PORT}"
        "-sUSE_GLFW=3"
        "-sASYNCIFY=1"
        "-sALLOW_MEMORY_GROWTH=1"
        # Jolt's collision narrow-phase recurses deeply; the 64 KB Emscripten
        # default stack overflows the moment play mode steps physics (it
        # surfaces in -O2 as a corrupt "table index is out of bounds"). Match the
        # native 8 MB thread stack so gameplay has the headroom it expects.
        "-sSTACK_SIZE=8388608"
        "-sMODULARIZE=1"
        "-sEXPORT_NAME=createViewer"
        "-sENVIRONMENT=web"
        # Hooks the page calls (on-screen sticks/buttons + debug panel; keep _main).
        "-sEXPORTED_FUNCTIONS=_main,_rt_web_key,_rt_web_axis,_rt_web_button,_rt_web_look,_rt_web_camera,_rt_web_flag,_rt_web_stat,_rt_web_cam,_rt_web_play,_rt_web_env,_rt_web_sun,_rt_web_post,_rt_web_pause,_rt_web_city"
        # web_main's EM_ASM uses stringToNewUTF8; Emscripten doesn't dependency-
        # scan EM_ASM bodies, so pin it explicitly or a toolchain bump can DCE it.
        "-sEXPORTED_RUNTIME_METHODS=stringToNewUTF8"
        "--preload-file" "${CMAKE_SOURCE_DIR}/assets@/assets"
    )
    # Emit viewer_web.js (+ .wasm + .data); web/viewer.html loads it.
    set_target_properties(viewer_web PROPERTIES OUTPUT_NAME "viewer_web")

    # Copy the hand-written shell + gallery next to the build output so the
    # directory is servable as-is (the MODULARIZE build has no auto-generated
    # HTML). index.html = scene gallery (landing), viewer.html = the renderer,
    # scenes.json = the gallery manifest, thumbs/ = baked scene thumbnails.
    add_custom_command(TARGET viewer_web POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/web/index.html
                $<TARGET_FILE_DIR:viewer_web>/index.html
        # viewer.html gets __RT_BUILD__ replaced with the wasm content hash so
        # its cache-busting ?v= changes exactly when the engine bytes do.
        COMMAND ${CMAKE_COMMAND}
                -DSRC=${CMAKE_SOURCE_DIR}/web/viewer.html
                -DDST=$<TARGET_FILE_DIR:viewer_web>/viewer.html
                -DWASM=$<TARGET_FILE_DIR:viewer_web>/viewer_web.wasm
                -P ${CMAKE_SOURCE_DIR}/cmake/inject_build_id.cmake
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/web/about.html
                $<TARGET_FILE_DIR:viewer_web>/about.html
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/web/city-sim.html
                $<TARGET_FILE_DIR:viewer_web>/city-sim.html
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/web/city-build.html
                $<TARGET_FILE_DIR:viewer_web>/city-build.html
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/web/buildings.html
                $<TARGET_FILE_DIR:viewer_web>/buildings.html
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/web/procgen.html
                $<TARGET_FILE_DIR:viewer_web>/procgen.html
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/web/scenes.json
                $<TARGET_FILE_DIR:viewer_web>/scenes.json
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_SOURCE_DIR}/web/thumbs
                $<TARGET_FILE_DIR:viewer_web>/thumbs
        VERBATIM)

    message(STATUS "Emscripten toolchain — building the WebGPU/WASM viewer_web target")
endif()
