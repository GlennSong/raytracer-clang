#!/usr/bin/env python3
"""agent_lab_probe: the one-agent lab (ADR-0415) run with nobody at the keyboard.

Launches the viewer on agent_lab, waits for the city, then for `trip near` and then `trip far`: sends it, polls
`trip?` until the person has arrived (or the timeout), prints the plan and the time taken, and takes one screenshot
mid-trip (the follow camera's view) into ~/Pictures/Raytracer/agent-lab/. Stops the viewer it started, by its own
process (never by name: the owner runs `viewer` too).

    python3 tools/agent_lab_probe.py                    # real time
    python3 tools/agent_lab_probe.py --speed 4          # the sim at 4x (`sim speed 4`)
"""
import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtctl  # noqa: E402


def wait_for_file(path, timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return True
        time.sleep(0.25)
    return False


def run_trip(sock, kind, number, shots, timeout):
    print(f"\n=== trip {kind} ===")
    print("send:", rtctl.send(sock, f"trip {kind}"))
    t0 = time.time()
    # the trip begins on the viewer's next frame: until trip? names it, it still describes the last one
    while time.time() - t0 < 30:
        line = rtctl.send(sock, "trip?", timeout=30)
        if line.startswith(f"ok trip {number}:"):
            break
        if "refused" in line or "trip near:" in line or "trip far:" in line:
            print("not started:", line)
            return False
        time.sleep(0.2)
    first_left = None
    shot_taken = False
    last_leg = None
    line = ""
    while time.time() - t0 < timeout:
        time.sleep(1.0)
        line = rtctl.send(sock, "trip?", timeout=30)
        if first_left is None and "| options:" in line:
            print("plan:", line)
        leg = re.search(r"\| leg (\d+)/(\d+) (\S+) (under way|starting)", line)
        left = re.search(r"(\d+) m left", line)
        if leg and leg.group(1) != last_leg:
            last_leg = leg.group(1)
            print(f"  {time.time() - t0:6.1f} s  leg {leg.group(1)}/{leg.group(2)} by {leg.group(3)}"
                  + (f", {left.group(1)} m left" if left else ""))
        if left:
            metres = int(left.group(1))
            if first_left is None:
                first_left = metres
            if not shot_taken and first_left and metres <= first_left * 0.5:
                path = os.path.join(shots, "trip_%s_mid.png" % re.sub(r"[^A-Za-z0-9]+", "_", kind))
                if os.path.exists(path):
                    os.remove(path)
                rtctl.send(sock, f"shot {path}")
                shot_taken = wait_for_file(path)
                print(f"  {time.time() - t0:6.1f} s  shot {path}" + ("" if shot_taken else " (did not land)"))
        if "arrived after" in line or "no way there" in line:
            break
    took = time.time() - t0
    print("final:", line)
    done = "arrived after" in line
    print(f"trip {kind}: {'ARRIVED' if done else 'NOT ARRIVED'} in {took:.1f} s real")
    return done


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--level", default="assets/levels/agent_lab.json")
    ap.add_argument("--speed", type=float, default=0.0, help="sim speed (0: leave it)")
    ap.add_argument("--timeout", type=float, default=900.0, help="real seconds a trip may take")
    ap.add_argument("--shots", default=os.path.expanduser("~/Pictures/Raytracer/agent-lab"))
    ap.add_argument("--log", default="out/zz_agentlab/probe_viewer.log")
    ap.add_argument("--trips", nargs="+", default=["near", "far"],
                    help="what to send after `trip`, in order (e.g. far, or \"to 96\")")
    args = ap.parse_args()
    os.makedirs(args.shots, exist_ok=True)
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    t0 = time.time()
    proc, sock = rtctl.launch(args.viewer, args.level, log=open(args.log, "wb"))
    print("viewer pid", proc.pid, "log", args.log)
    ok = True
    try:
        rtctl.send(sock, "camera?", timeout=1500)
        # the city builds after the first frames: wait until the trip system can see its person
        while True:
            line = rtctl.send(sock, "trip?", timeout=60)
            if (line.startswith("ok") and "not built" not in line) or time.time() - t0 > 1500:
                break
            time.sleep(1.0)
        print(f"loaded in {time.time() - t0:.0f} s: {line}")
        if args.speed > 0:
            print("sim speed:", rtctl.send(sock, f"sim speed {args.speed}"))
        for number, kind in enumerate(args.trips, start=1):
            ok = run_trip(sock, kind, number, args.shots, args.timeout) and ok
    finally:
        rtctl.stop(proc)
    print("\nprobe", "OK" if ok else "FAILED", f"({time.time() - t0:.0f} s in all)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
