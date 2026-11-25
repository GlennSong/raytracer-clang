#!/usr/bin/env python3
"""THE YARDSTICK for #315 (prefabs in the real city): one number for "is the frame time and the GPU memory worse?".

It starts a viewer on a level with present sync off and no sound, leaves the player at the level's own start, puts
the camera at a fixed place, leaves the clock at the level's own hour, lets the world settle, then takes the frame ledger's summary over 30 seconds
and the GPU memory's first line. Run it before and after a change, on the same machine with nothing else running.

    python3 tools/prefab_yardstick.py                       # the island, from the player's start in Saltwood
    python3 tools/prefab_yardstick.py assets/levels/prefab_street.json --camera "45 9 -20 -25 0" --teleport "45 1.7 -40"

Prints one line: the level, frame avg / p95 / max in ms, and the memory line. The capture is also written as a CSV
beside the log (build-viewer/yardstick/), for tools/frame-report.py.
"""
import argparse, os, re, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import rtctl

ap = argparse.ArgumentParser()
ap.add_argument("level", nargs="?", default="assets/levels/island_8_nature.json")
ap.add_argument("--camera", default="", help='"x y z pitch yaw"; default: 3 m over the player\'s start, looking toward -z, 8 degrees down')
ap.add_argument("--teleport", default="", help='"x y z": move the player there first (what streams in follows the player)')
ap.add_argument("--settle", type=float, default=45.0, help="seconds to let the world stream in before measuring")
ap.add_argument("--seconds", type=float, default=30.0)
ap.add_argument("--wait", type=float, default=2400.0, help="how long to wait for the level to load (a first load bakes)")
ap.add_argument("--viewer", default=os.path.join(ROOT, "build-viewer", "viewer"), help="another build's viewer, to measure it from this folder (the same assets and bakes)")
a = ap.parse_args()

out = os.path.join(ROOT, "build-viewer", "yardstick")
os.makedirs(out, exist_ok=True)
name = os.path.splitext(os.path.basename(a.level))[0]
env = dict(os.environ, RT_VSYNC="0", RT_AUDIO="null")
proc, sock = rtctl.launch(a.viewer, os.path.join(ROOT, a.level),
                          log=open(os.path.join(out, name + ".log"), "wb"), env=env, play=True, wait=a.wait)
try:
    rtctl.send(sock, "camera?", timeout=a.wait)
    if a.teleport:
        rtctl.send(sock, "teleport " + a.teleport, timeout=120)
        time.sleep(3)
    # (the clock is left to run from the level's own hour: `daynight <hour>` jumps the sim's clock, which wakes every
    # resting person it jumps past, and the burst of trips that follows was two thirds of the island's frame)
    cam = a.camera
    if not cam:
        m = re.search(r"pos (-?[\d.]+) (-?[\d.]+) (-?[\d.]+)", rtctl.send(sock, "move?", timeout=30))
        x, y, z = map(float, m.groups())
        cam = "%.2f %.2f %.2f -8 0" % (x, y + 3.0, z)
    rtctl.send(sock, "camera " + cam, timeout=60)
    time.sleep(a.settle)
    csv = os.path.join(out, name + ".csv")
    rtctl.send(sock, "ledger start " + csv, timeout=30)
    time.sleep(a.seconds)
    rtctl.send(sock, "ledger stop", timeout=30)
    summary = rtctl.send(sock, "ledger summary", timeout=30)
    mem = rtctl.send(sock, "mem?", timeout=60).split("\n")[0][:160]
    models = rtctl.send(sock, "models?", timeout=60)
    print("%s | camera %s | %s | %s | %d stamped prefabs" % (name, cam, summary.replace("ok ", ""), mem.replace("ok ", ""), len(re.findall(r"\d+ \S+ at ", models))))
finally:
    rtctl.stop(proc)
