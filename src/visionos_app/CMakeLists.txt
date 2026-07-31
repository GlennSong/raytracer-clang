# visionOS plumbing spike — standalone build.
#
# NOT part of the main CMake tree, and deliberately so: this cross-compiles for
# visionOS (a different sysroot and architecture from the host tools, tests and
# viewer), so it needs its own configure. Wiring it into the root CMakeLists
# would mean every macOS build carried a target it cannot build.
#
# The Xcode generator is required rather than preferred — Ninja/Makefiles cannot
# produce a signed visionOS .app bundle with a Swift entry point.
#
#   Simulator:
#     cmake -S src/visionos_app -B build-visionos-sim -G Xcode \
#           -DCMAKE_SYSTEM_NAME=visionOS -DCMAKE_OSX_SYSROOT=xrsimulator
#     cmake --build build-visionos-sim
#
#   Device (needs a signing identity):
#     cmake -S src/visionos_app -B build-visionos -G Xcode \
#           -DCMAKE_SYSTEM_NAME=visionOS -DCMAKE_OSX_SYSROOT=xros
#     cmake --build build-visionos

cmake_minimum_required(VERSION 3.28)   # first release with visionOS support

project(VisionSpike LANGUAGES CXX OBJCXX Swift)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_Swift_LANGUAGE_VERSION 5)

add_executable(VisionSpike MACOSX_BUNDLE
    VisionSpikeApp.swift
    vision_spike.mm
    vision_spike.h
)

target_link_libraries(VisionSpike PRIVATE
    "-framework CompositorServices"
    "-framework Metal"
    "-framework SwiftUI"
    "-framework ARKit"
    "-framework Foundation"
)

set_target_properties(VisionSpike PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
    MACOSX_BUNDLE_GUI_IDENTIFIER "com.glennsong.raytracer.visionspike"
    MACOSX_BUNDLE_BUNDLE_NAME "VisionSpike"
    MACOSX_BUNDLE_BUNDLE_VERSION "1"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0"
    XCODE_ATTRIBUTE_SWIFT_OBJC_BRIDGING_HEADER
        "${CMAKE_CURRENT_SOURCE_DIR}/VisionSpike-Bridging-Header.h"
    XCODE_ATTRIBUTE_SWIFT_VERSION "5.0"
    # The spike runs unsigned in the simulator; a device build needs a real team.
    XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED "NO"
    XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED "NO"
    XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC "YES"
)
