cmake_minimum_required(VERSION 3.20)
project(raytracer LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared math/core sources (used by both targets)
set(CORE_SOURCES
    src/math.cpp
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
# Interactive viewer (GPU + windowing)
# ============================================================
find_package(glfw3 REQUIRED)

set(VIEWER_SOURCES
    src/viewer_main.cpp
    src/renderer/window.cpp
    src/renderer/orbit_camera.cpp
    src/renderer/settings.cpp
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
