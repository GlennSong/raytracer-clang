#!/usr/bin/env python3
"""Fall probe (Glenn, 2026-09-26: "there's a lot of falling through the floor"): launch the viewer in
play mode, teleport the PLAYER onto the ground at each spot, let physics run, and compare where the
player ends up with the drawn ground there. A player metres under the ground fell through it.

  tools/fall_probe.py LEVEL OUT --spots x,z:x,z:... [--wait 3] [--walk 0]
"""
import argparse, os, subprocess, sys, time
sys.path.insert(0, os.path.dirname(__file__))
import rtctl   # the control channel (ADR-0291)
from rtctl import send

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level"); ap.add_argument("out")
    ap.add_argument("--spots", required=True)
    ap.add_argument("--wait", type=float, default=3.0)
    ap.add_argument("--dt", type=float, default=0.5)
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--load-timeout", type=float, default=2900.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    log = open(os.path.join(a.out, "viewer.log"), "wb")
    t0 = time.time()
    proc, sock = rtctl.launch(a.viewer, a.level, log=log)
    try:
        send(sock, "camera?", timeout=a.load_timeout)
        time.sleep(5)
        fell = 0
        spots = [tuple(map(float, s.split(","))) for s in a.spots.split(":")]
        for x, z in spots:
            send(sock, f"ground? {x} {z}"); time.sleep(0.3)
            g = send(sock, "ground?")
            send(sock, f"teleport {x} {z}")
            for _ in range(50):
                r = send(sock, "teleport?")
                if "pending" not in r: break
                time.sleep(0.1)
            samples = []
            for _ in range(int(a.wait / a.dt)):
                time.sleep(a.dt)
                w = send(sock, "where?").split()
                samples.append(float(w[2]))
            gy = None
            try: gy = float(g.split("carved=")[1].split()[0])
            except Exception: pass
            low = min(samples)
            bad = gy is not None and low < gy - 2.0
            fell += bad
            print("   samples:", " ".join(f"{v:.1f}" for v in samples))
            print(f"({x:8.1f}, {z:8.1f}) ground {g!s:28} teleport {r!s:40} eye y {samples[0]:8.2f} -> {samples[-1]:8.2f} (min {low:8.2f}){'  FELL' if bad else ''}", flush=True)
        print(f"{fell}/{len(spots)} spots fell through")
    finally:
        rtctl.stop(proc)

if __name__ == "__main__":
    main()
