# Building & running on Windows

The viewer and the Qt editor both run on Windows through the Vulkan backend
(ADR-0057). This is the concrete, verified setup — the cross-platform overview is
in the [README](../README.md); this page covers the Windows-specific bits that
aren't obvious (clang vs MSVC, the GPU/Qt dependencies, and Qt deployment).

## Prerequisites

| What | How | Notes |
| --- | --- | --- |
| **clang + cmake + ninja** | Install **Visual Studio Build Tools 2022** (the "Desktop development with C++" workload). | Bundles clang 19, cmake, ninja, the MSVC linker, and the Windows SDK — nothing else needed for the core build. |
| **Vulkan SDK** | [vulkan.lunarg.com](https://vulkan.lunarg.com/) | Provides headers, the loader, validation layers, and `glslc`. Sets `VULKAN_SDK`. Needs a Vulkan-capable GPU/driver to run (check `vulkaninfoSDK --summary`). |
| **GLFW** (viewer) | `vcpkg install glfw3:x64-windows` | The viewer's window/input backend. |
| **Qt 6** (editor) | `vcpkg install qtbase:x64-windows` | Enables the `editor_app` target. Large, slow build (Qt + deps from source). Only needed for the editor. |

[vcpkg](https://vcpkg.io): `git clone https://github.com/microsoft/vcpkg`, then
`.\vcpkg\bootstrap-vcpkg.bat`. Keep it outside the repo (e.g. `H:\software\vcpkg`).

## One-time: a developer PowerShell

clang/cmake/ninja need the VS developer environment on `PATH`. Either open
**"Developer PowerShell for VS 2022"** from the Start menu, or load it in every
shell from your `$PROFILE`:

```powershell
$vs = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools"
Import-Module "$vs\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
```

## Configure & build

Configure with **clang explicitly** — CMake defaults to MSVC on Windows, which
rejects the project's `-Wall -Wextra -Wpedantic` flags (`D8021 /Wextra`). Point it
at the vcpkg toolchain so GLFW + Qt are found. From a fresh dev shell (so
`VULKAN_SDK` is live):

```powershell
cmake -S . -B build -G Ninja `
  -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ `
  -DCMAKE_TOOLCHAIN_FILE=<vcpkg>\scripts\buildsystems\vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build
ctest --test-dir build          # unit + physics tests
```

The compiler is cached, so the `-DCMAKE_*_COMPILER` flags are only needed when
creating or wiping `build/`. The Dear ImGui debug overlay
(`-DRT_ENABLE_IMGUI=ON`, default) toggles in-app with the tilde/grave key; pass
`-DRT_ENABLE_IMGUI=OFF` to leave it out.

Configure messages confirm what's wired up:
`Vulkan found — building the viewer with the Vulkan backend`,
`Vulkan found — editor viewport uses the Vulkan backend`,
`Qt6 found — building the editor application`.

## Run

**Always run from the repo root** — shaders load from an absolute baked path, but
`assets/` is resolved relative to the working directory.

```powershell
cd <repo-root>
.\build\viewer.exe          # the game/viewer (boots into play; --edit for edit mode)
.\build\editor_app.exe      # the Qt editor (its viewport renders via Vulkan)
```

- **`glfw3.dll`** is copied next to `viewer.exe` automatically (vcpkg applocal).
- **The editor is self-contained.** vcpkg deploys the Qt DLLs next to
  `editor_app.exe`, and a CMake POST_BUILD step copies the Qt platform plugin into
  `build\platforms\` (vcpkg doesn't deploy plugins). So no `QT_QPA_PLATFORM_PLUGIN_PATH`
  is needed. If you ever see *"Could not find the Qt platform plugin 'windows'"*,
  the plugin didn't get deployed — rebuild `editor_app`.
- **Debug build caveat:** the default `CMAKE_BUILD_TYPE=Debug` links the debug Qt +
  debug CRT (`ucrtbased.dll`), which are present inside a VS dev shell. A Release /
  RelWithDebInfo build drops that dependency (and uses the release Qt DLLs).

## Gamepad

The **viewer** has controller support (GLFW + `gamecontrollerdb.txt`, loaded from
the repo root): left stick moves, right stick looks, triggers raise/lower, bumpers
boost/fire — tested with an Xbox controller. The **editor** has no gamepad input
yet: it runs through `HostedWindow`, which doesn't poll a gamepad ("none in hosted
mode"); the `HostedWindow::setGamepads` seam is the hook to add it.

## Convenience

A `viewer` / `editor` PowerShell function (cd to the repo root + launch) makes this
one word. See `docs/vulkan-renderer-plan.md` → "Device-verified findings" for the
running log of what's been verified on Windows hardware.
