# Building the web (WebGPU / WebAssembly) target

This guide spells out, step by step, how to install the toolchain and compile the
engine to run in a browser. The web build compiles the **same** C++ engine to
WebAssembly and renders through **WebGPU** (the browser's `navigator.gpu`). It is
the Emscripten implementation of the `Renderer` seam — see
[`src/renderer/webgpu/AGENTS.md`](../src/renderer/webgpu/AGENTS.md) and
[ADR-0058](decisions.md).

No GPU drivers or SDKs are required to *build* — WebGPU lives in the browser. You
only need the **Emscripten SDK** (which bundles clang/LLVM configured to emit
WebAssembly) and CMake. The WebGPU headers/JS glue are pulled in automatically by
Emscripten's `emdawnwebgpu` port.

---

## Prerequisites

- **CMake ≥ 3.16** (tested with 3.28).
- **Python 3** (used by the Emscripten SDK installer and to serve the build).
- **Git**.
- A **WebGPU-capable browser** to run the result: Chrome/Edge 113+, Safari 18+,
  or Firefox 141+ (behind a flag on some platforms).

You do **not** need the native graphics stack (Metal/Vulkan), Jolt binaries, or
GLFW installed — those come from submodules or Emscripten's shims.

---

## Step 1 — Fetch the project submodules

Jolt (physics) and Lua (scripting) are git submodules and compile to wasm too.
From the repository root:

```bash
git submodule update --init --recursive
```

## Step 2 — Install the Emscripten SDK (emsdk)

The emsdk is the Emscripten toolchain: `emcc`/`em++` (clang front-ends that emit
WebAssembly), plus `emcmake` (a CMake wrapper) and the Node/Python glue. Install
it **once**, anywhere on disk (a sibling directory to the repo is fine):

```bash
# 1. Clone the SDK
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# 2. Download + install the toolchain (this project is verified on 6.0.1)
./emsdk install 6.0.1

# 3. Make that version the active one
./emsdk activate 6.0.1
```

> Windows: run `emsdk.bat install 6.0.1` then `emsdk.bat activate 6.0.1` from a
> Developer Command Prompt.

## Step 3 — Put the toolchain on your PATH (every new shell)

`emcmake`, `emcc`, etc. are not global — you activate them per shell by sourcing
the SDK's env script. Do this in **every terminal** where you build:

```bash
# from the emsdk directory (adjust the path to wherever you cloned it)
source ./emsdk_env.sh          # Linux / macOS
# emsdk_env.bat                # Windows
```

Verify it worked:

```bash
emcc --version                 # should print an Emscripten version, e.g. 6.0.1
```

## Step 4 — Configure the build with `emcmake`

`emcmake` runs CMake with Emscripten's toolchain file, so CMake produces a wasm
build instead of a native one. From the **repository root**:

```bash
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
```

- `-S .` — source directory (the repo root).
- `-B build-web` — where to put the build (a fresh directory).
- `-DCMAKE_BUILD_TYPE=Release` — optimized (`-O2`). Use `Debug` while developing;
  `MinSizeRel` (`-Oz`) for the smallest download.

The first configure downloads the `emdawnwebgpu` WebGPU port automatically (a
small ~130 KB header + JS-glue package). CMake's `if(EMSCRIPTEN)` block selects
the WebGPU backend and skips the Metal/Vulkan/Qt paths (their `find_package`
calls don't resolve under Emscripten).

## Step 5 — Compile

```bash
cmake --build build-web --target viewer_web
```

This produces **three files** in `build-web/`, plus the copied web front-end:

| File | What it is |
| --- | --- |
| `viewer_web.wasm` | the compiled engine (WebAssembly) |
| `viewer_web.js`   | glue that loads the wasm and emulates window/filesystem/WebGPU |
| `viewer_web.data` | the preloaded `assets/` tree (levels, scripts) baked into a virtual filesystem |
| `index.html`      | the scene **gallery** (landing page) |
| `viewer.html`     | the **renderer** page that loads `viewer_web.js` |
| `scenes.json`, `thumbs/` | gallery manifest + baked scene thumbnails |

## Step 6 — Serve it over HTTP and open it

WebGPU requires a **secure context**. `localhost` counts, but opening the file
directly (`file://`) does **not** — you must serve it over HTTP:

```bash
python3 -m http.server --directory build-web 8000
```

Then open **http://localhost:8000/** in a WebGPU-capable browser. You'll land on
the scene gallery; click a scene (or go straight to
`http://localhost:8000/viewer.html?level=city`).

---

## One-shot rebuild loop

After the initial setup, the day-to-day loop is just:

```bash
source /path/to/emsdk/emsdk_env.sh                 # once per shell
cmake --build build-web --target viewer_web        # rebuild
python3 -m http.server --directory build-web 8000  # serve (leave running)
```

> If you add or change files under `assets/`, force a re-link so the `.data`
> repackages (the preload is a link-time step): `rm build-web/viewer_web.*` then
> rebuild.

---

## Deploying to GitHub Pages

The live site (https://glennsong.github.io/raytracer-clang/) is served from the
`gh-pages` branch, which holds only built artifacts — never edit it by hand.
After building (Steps 4–5):

```bash
tools/deploy-pages.sh "Pages: <what changed>"
```

The script copies `viewer_web.{js,wasm,data}`, the front-end pages
(`index.html`, `viewer.html`, `about.html`, `scenes.json`), and `thumbs/` from
`build-web/` onto `gh-pages` via a temporary worktree, commits, and pushes.
Pages picks the commit up within a minute or two. Cache-busting is automatic:
the build injected a wasm content hash into `viewer.html` (see
`cmake/inject_build_id.cmake`), so browsers cache the engine between visits and
re-fetch exactly when the bytes change.

---

## Offline / air-gapped builds

By default the `emdawnwebgpu` port is fetched from the network on first configure.
For an offline build, download the `emdawnwebgpu_pkg-*.zip` from
[Dawn's GitHub releases](https://github.com/google/dawn/releases), extract it, and
point CMake at the local port script:

```bash
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release \
  -DRT_EMDAWN_PORT=/absolute/path/to/emdawnwebgpu.port.py
```

---

## What the build flags do (reference)

The web target's link options live in the `if(EMSCRIPTEN)` block of the root
[`CMakeLists.txt`](../CMakeLists.txt). The load-bearing ones:

| Flag | Why |
| --- | --- |
| `--use-port=emdawnwebgpu` | pulls in Dawn's standardized `webgpu.h` + JS glue (compile **and** link). The legacy `-sUSE_WEBGPU` was removed in Emscripten 6. |
| `-sASYNCIFY=1` | lets the synchronous `Renderer::initialize()` "await" the async WebGPU device request without freezing the page. |
| `-sMODULARIZE=1 -sEXPORT_NAME=createViewer` | the output is a factory the page calls (`createViewer({canvas})`); no auto-run, no auto-HTML. |
| `-sALLOW_MEMORY_GROWTH=1` | the wasm heap can grow for large scenes. |
| `-sSTACK_SIZE=8388608` | 8 MB stack — Jolt's collision narrow-phase overflows the 64 KB default the instant play mode steps physics. |
| `-sUSE_GLFW=3` | Emscripten's GLFW shim, so `window.cpp` is reused unchanged. |
| `--preload-file assets@/assets` | bakes `assets/` into a virtual in-memory filesystem so `fopen("assets/…")` works with no network. |
| `-sEXPORTED_FUNCTIONS=_main,_rt_web_*` | keeps the C hooks the HTML panel/sticks call from being dead-code-eliminated. |

---

## Troubleshooting

- **`emcmake: command not found`** — you didn't source `emsdk_env.sh` in this
  shell (Step 3). It's per-terminal.
- **`WebGPU not available` in the browser** — the browser is too old or WebGPU is
  disabled. Update Chrome/Edge/Safari, or enable it (Firefox:
  `dom.webgpu.enabled`). Also confirm you're on `http://localhost`, not `file://`.
- **Blank canvas but the log overlay shows frames** — usually a real-GPU vs.
  software-renderer difference; try a different browser/machine. (Headless
  software rasterizers famously don't composite the WebGPU canvas at all.)
- **A level change doesn't show new assets** — the `.data` didn't repackage;
  `rm build-web/viewer_web.*` and rebuild to force a re-link.
- **`table index is out of bounds` after entering play mode** — a stack overflow;
  make sure `-sSTACK_SIZE` is set (it is, in this repo — see the flags table).
