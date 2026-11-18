#!/usr/bin/env python3
"""CROSSWALKS IN THE VIEWER (ADR-0452): brain_crowd at its busy minutes, four pictures, each found by asking the viewer's
`crosswalks?` (people at the road model's crosswalks, drivers giving way at one, cars on a green arrow):

  1 people waiting at a kerb end on DON'T WALK;
  2 people crossing on WALK with a turning car waiting for them;
  3 a car turning on a green arrow;
  4 a car at a stop sign waiting for someone on the crosswalk.

    python3 tools/crosswalk_probe.py --suffix _4b

Pictures ~/Pictures/Raytracer/brain/signals_*<suffix>.png. Starts and stops its own viewer, by PID (tools/rtctl.py)."""
import argparse
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtctl  # noqa: E402
from brain_lab_probe import shoot  # noqa: E402

BUSY_MINUTE = 8.0


def parse(line):
    """The `crosswalks?` answer as lists: walkers (index, on, crosswalk, x, z, control, signal), giving way (car, crosswalk,
    x, z, speed), arrows (car, x, z, speed)."""
    walkers, giving, arrows = [], [], []
    for part in line.split(" | "):
        m = re.match(r"walker (\d+) (on|waiting at) crosswalk (\d+) at (-?[\d.]+) (-?[\d.]+) \((\w+), ([^,]+), junction", part)
        if m:
            walkers.append((int(m.group(1)), m.group(2) == "on", int(m.group(3)), float(m.group(4)), float(m.group(5)),
                            m.group(6), m.group(7)))
            continue
        m = re.match(r"car (\d+) gives way at crosswalk (\d+), at (-?[\d.]+) (-?[\d.]+), ([\d.]+) m/s", part)
        if m:
            giving.append((int(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))))
            continue
        m = re.match(r"car (\d+) turns on a green arrow, at (-?[\d.]+) (-?[\d.]+), ([\d.]+) m/s", part)
        if m:
            arrows.append((int(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))))
    return walkers, giving, arrows


def look_at(sock, x, z, height=12.0, back=10.0):
    """The camera above and to the side of a plan point, looking down at it."""
    cx, cz = x + back * 0.7, z + back * 0.7
    yaw = math.degrees(math.atan2(x - cx, -(z - cz)))
    pitch = -math.degrees(math.atan2(height, math.hypot(x - cx, z - cz)))
    rtctl.send(sock, f"camera {cx} {height} {cz} {pitch} {yaw}")


def find(sock, want, timeout):
    """Polls `crosswalks?` until want(walkers, giving, arrows) gives a place (x, z) and words; None after timeout."""
    t = time.time()
    while time.time() - t < timeout:
        line = rtctl.send(sock, "crosswalks?")
        got = want(*parse(line))
        if got:
            return got, line
        time.sleep(0.25)
    return None, ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--level", default="assets/levels/brain_crowd.json")
    ap.add_argument("--shots", default=os.path.expanduser("~/Pictures/Raytracer/brain"))
    ap.add_argument("--log", default="out/zz_4b/crosswalk_probe_viewer.log")
    ap.add_argument("--suffix", default="_4b")
    args = ap.parse_args()
    os.makedirs(args.shots, exist_ok=True)
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    t0 = time.time()
    proc, sock = rtctl.launch(args.viewer, args.level, log=open(args.log, "wb"))
    print("viewer pid", proc.pid)
    ok = 0
    try:
        rtctl.send(sock, "camera?", timeout=1500)
        print(rtctl.send(sock, "brain labels off"))
        print(rtctl.send(sock, "brain follow off"))
        print(rtctl.send(sock, "detach off"))
        rtctl.send(sock, "sim speed 16")
        while time.time() - t0 < 1500:
            m = re.search(r"over (\d+) ticks", rtctl.send(sock, "driving?"))
            if m and int(m.group(1)) >= BUSY_MINUTE * 60 * 60:
                break
            time.sleep(0.5)
        rtctl.send(sock, "sim speed 1")

        def waiting_on_dont_walk(walkers, giving, arrows):
            for w in walkers:
                if not w[1] and w[5] == "lights" and w[6] == "don't walk":
                    return (w[3], w[4]), f"walker {w[0]} waits at crosswalk {w[2]} on DON'T WALK"
            return None

        def crossing_with_car_waiting(walkers, giving, arrows):
            for g in giving:
                for w in walkers:
                    if w[1] and w[2] == g[1] and w[5] == "lights" and g[4] < 0.3:
                        return ((w[3] + g[2]) / 2, (w[4] + g[3]) / 2), f"walker {w[0]} crosses on {w[6]}, car {g[0]} waits"
            return None

        def on_an_arrow(walkers, giving, arrows):
            for a in arrows:
                if a[3] > 1.0:
                    return (a[1], a[2]), f"car {a[0]} turns on a green arrow"
            return None

        def stop_sign_waits(walkers, giving, arrows):
            for g in giving:
                for w in walkers:
                    if w[1] and w[2] == g[1] and w[5] == "signs" and g[4] < 0.3:
                        return ((w[3] + g[2]) / 2, (w[4] + g[3]) / 2), f"car {g[0]} at a stop sign waits for walker {w[0]}"
            return None

        for name, want in (("1_waiting_at_the_kerb_on_dont_walk", waiting_on_dont_walk),
                           ("2_crossing_on_walk_a_turning_car_waits", crossing_with_car_waiting),
                           ("3_a_green_arrow_turners_go", on_an_arrow),
                           ("4_a_stop_sign_car_waits_for_someone_crossing", stop_sign_waits)):
            got, line = find(sock, want, 240)
            if not got:
                print(f"  {name}: not seen in 240 s")
                continue
            (x, z), words = got
            rtctl.send(sock, "sim pause")
            look_at(sock, x, z)
            time.sleep(1.0)
            shoot(sock, args.shots, f"signals_{name}{args.suffix}", t0, words)
            rtctl.send(sock, "sim resume")
            rtctl.send(sock, "detach off")
            ok += 1
        print(rtctl.send(sock, "near?")[-160:])
    finally:
        rtctl.stop(proc)
    print("probe", ok, "of 4 pictures", f"({time.time() - t0:.0f} s)")
    return 0 if ok == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
