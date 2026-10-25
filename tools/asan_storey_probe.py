"""Step through island towers' storeys under AddressSanitizer: the regression check for #114 (ADR-0241).

Usage: python3 tools/asan_storey_probe.py [minutes] [cx cz] [viewer]
Build the sanitizer viewer first (its own folder; build-viewer is untouched):
    cmake -S . -B build-asan -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DRT_BUILD_DEMOS=OFF \
        "-DCMAKE_C_FLAGS=-fsanitize=address -fno-omit-frame-pointer" \
        "-DCMAKE_CXX_FLAGS=-fsanitize=address -fno-omit-frame-pointer" "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address"
    cmake --build build-asan --target viewer
Asks the viewer for the tall buildings round (cx, cz) (`towers?`), then goes INSIDE the tallest at 2, 50, 90, 2 and
70 % of their height in turn: each a different storey window of one tower's interior streaming in, which is what
once indexed a stale cache. Prints interiors built and window changes handled (`[indoors] ... another interior
window`). The island needs ~20-30 GB under ASan: one heavy job at a time. ASan's report lands in out/asan/asan.<pid>.
"""
import glob, math, os, random, subprocess, sys, time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tools"))
from lanelab_shots import send

minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
cx, cz = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (-3805.0, -604.0)
viewer = sys.argv[4] if len(sys.argv) > 4 else "build-asan/viewer"
OUT = os.path.join(REPO, "out/asan")
os.makedirs(OUT, exist_ok=True)
logp = os.path.join(OUT, "viewer.log")
env = dict(os.environ, RT_PROBE_TAG="asan", RT_AUDIO="null",
           ASAN_OPTIONS=f"detect_leaks=0:halt_on_error=1:abort_on_error=1:log_path={OUT}/asan:"
                        "detect_stack_use_after_return=1:symbolize=1")
proc = subprocess.Popen([os.path.join(REPO, viewer), "assets/levels/island_8_nature.json", "--play"],
                        cwd=REPO, env=env, stdout=open(logp, "wb"), stderr=subprocess.STDOUT)
sock = f"/tmp/raytracer-viewer-{proc.pid}.sock"
print("pid", proc.pid, flush=True)
rng = random.Random(114)


def alive():
    return proc.poll() is None


def tp(x, y, z, pitch, yaw):
    send(sock, f"teleport {x:.1f} {y:.1f} {z:.1f} {pitch:.1f} {yaw:.1f}")
    for _ in range(100):
        if not alive() or "pending" not in send(sock, "teleport?"):
            break
        time.sleep(0.1)
    time.sleep(1.5)


try:
    t0 = time.time()
    while not os.path.exists(sock):
        if not alive() or time.time() - t0 > 900:
            sys.exit("no socket (exit %s)" % proc.poll())
        time.sleep(1)
    send(sock, "camera?", timeout=1800)
    time.sleep(20)
    r = send(sock, f"towers? {cx} {cz} 700 50")
    towers = [tuple(float(v) for v in part.split()) for part in r.split("|")[1:]]
    print(f"{len(towers)} towers:", " ".join(f"({t[0]:.0f},{t[1]:.0f} {t[3]:.0f}m)" for t in towers[:12]), flush=True)
    if not towers:
        sys.exit("no towers near " + r)
    end = time.time() + minutes * 60
    visits = 0
    while time.time() < end and alive():
        for tx, tz, gy, h in towers[:12]:
            if time.time() >= end or not alive():
                break
            # INSIDE the tower at street level, halfway, near the top, back at the street, 70 %: each a different
            # storey window of the one tower's interior -- what used to index a stale cache (#114)
            for f in (0.02, 0.5, 0.9, 0.02, 0.7):
                if not alive():
                    break
                tp(tx, gy + h * f + 1.6, tz, 0, rng.uniform(-180, 180))
                time.sleep(3.0)
                visits += 1
            L = open(logp, errors="ignore").read()
            print(f"{(time.time() - t0) / 60:5.1f} min, {visits} visits, interiors built {L.count('[interior] built')}, "
                  f"window changes handled {L.count('another interior window')}, alive {alive()}", flush=True)
finally:
    code = proc.poll()
    if code is None:
        proc.terminate()
        try:
            proc.wait(20)
        except Exception:
            proc.kill()
    print("exit", code, flush=True)
    for f in sorted(glob.glob(os.path.join(OUT, "asan.*"))):
        print("ASAN REPORT:", f)
        print(open(f, errors="ignore").read()[:6000])
