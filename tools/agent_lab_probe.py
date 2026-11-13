#!/usr/bin/env python3
"""agent_lab_probe: the one-agent lab (ADR-0415) run with nobody at the keyboard.

Launches the viewer on agent_lab, waits for the city, then for `trip near` and then `trip far`: sends it (or taps the
lab's key, --keys), polls `trip?` and the person's `who?` until they have arrived (or the timeout), prints the plan,
the time taken and every time the car stood still, and takes the follow camera's screenshots of the transitions --
before getting in, setting off, a junction turn, just out of the car, arrived -- into ~/Pictures/Raytracer/agent-lab/. Stops the viewer it started, by its own
process (never by name: the owner runs `viewer` too).

    python3 tools/agent_lab_probe.py                    # real time
    python3 tools/agent_lab_probe.py --speed 4          # the sim at 4x (`sim speed 4`)
    python3 tools/agent_lab_probe.py --keys --trips far # one far trip, started with the I key
    python3 tools/agent_lab_probe.py --keys --trips freeway   # the freeway trip (ADR-0416), started with the U key

On the drive it says every change of road kind (street / ramp / freeway) with the car's speed, the top speed on the
freeway, and shoots the transitions onto it, along it and off it.
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


def shoot(sock, shots, name, t0):
    path = os.path.join(shots, name + ".png")
    if os.path.exists(path):
        os.remove(path)
    rtctl.send(sock, f"shot {path}")
    ok = wait_for_file(path)
    print(f"  {time.time() - t0:6.1f} s  shot {path}" + ("" if ok else " (did not land)"))


def who(sock):
    """The person's line from `who?` (staged on agent 0 at the start): position, speed, state, and the kind of road
    under them (street, ramp, freeway, footpath)."""
    line = rtctl.send(sock, "who?", timeout=30)
    m = re.search(r"pos ([-\d.]+) ([-\d.]+) speed ([-\d.]+) .*? state (\w+)", line)
    k = re.search(r"link -?\d+( \((\w+)\))?", line)
    road = (k.group(2) if k and k.group(2) else "street")
    return (float(m.group(1)), float(m.group(2)), float(m.group(3)), m.group(4), road) if m else None


def run_trip(sock, kind, number, shots, timeout, key=None):
    """One trip, sent as `trip <kind>` or, with `key`, by tapping that key through the viewer's input. Polls `trip?`
    and the person's `who?` line four times a second; takes the screenshots of the transitions (before getting in,
    setting off, a junction turn, just out of the car, arrived) and reports every time the car stood still for over a
    second during the drive, where, and in what state (waiting: a red light; yielding: someone in the way)."""
    print(f"\n=== trip {kind} ===")
    print("send:", rtctl.send(sock, f"tap {key}" if key else f"trip {kind}"))
    tag = re.sub(r"[^A-Za-z0-9]+", "_", kind)
    t0 = time.time()
    # the trip begins on the viewer's next frame: until trip? names it, it still describes the last one
    while time.time() - t0 < 30:
        line = rtctl.send(sock, "trip?", timeout=30)
        if line.startswith(f"ok trip {number}:"):
            break
        if "refused" in line or "trip near:" in line or "trip far:" in line or "trip freeway:" in line:
            print("not started:", line)
            return False
        time.sleep(0.2)
    print("plan:", line)
    rtctl.send(sock, "who? 0")
    taken = set()
    last_leg = None
    drive_start = None
    still_since = None
    first_left = None
    stops = []
    road_was = None
    road_since = 0.0
    on_freeway_top = 0.0
    been_on_freeway = False
    line = ""
    while time.time() - t0 < timeout:
        time.sleep(0.25)
        line = rtctl.send(sock, "trip?", timeout=30)
        if "arrived after" in line or "no way there" in line:
            break
        leg = re.search(r"\| leg (\d+)/(\d+) (\S+) (under way|starting)", line)
        left = re.search(r"(\d+) m left", line)
        w = who(sock)
        now = time.time() - t0
        if not leg:
            continue
        n, total, way = int(leg.group(1)), int(leg.group(2)), leg.group(3)
        if leg.group(1) != last_leg:
            last_leg = leg.group(1)
            print(f"  {now:6.1f} s  leg {n}/{total} by {way}" + (f", {left.group(1)} m left" if left else ""))
            if way == "car":
                drive_start = now
            if n > 1 and way == "walk" and total > 1 and "out" not in taken:
                taken.add("out")
                shoot(sock, shots, f"trip_{tag}_3_out", t0)
        if total > 1 and n == 1 and way == "walk" and w and "in" not in taken:
            # before getting in: within a few metres of the end of the walk to the car (its metres are the first
            # leg's in the chosen option's list)
            parts = re.findall(r"\[walk (\d+) m [\d.]+ min, car (\d+) m [\d.]+ min, walk (\d+) m", line)
            if parts and left and int(left.group(1)) - int(parts[-1][1]) - int(parts[-1][2]) <= 4:
                taken.add("in")
                shoot(sock, shots, f"trip_{tag}_0_before_in", t0)
        if total == 1 and left and "mid" not in taken:   # a one-leg trip: one shot halfway
            first_left = first_left or int(left.group(1))
            if int(left.group(1)) <= first_left * 0.5:
                taken.add("mid")
                shoot(sock, shots, f"trip_{tag}_2_mid", t0)
        if w and n == int(total) and way == "walk" and w[2] < 0.05 and w[3] == "waiting":
            # (a kerb or a crossing light): one entry per wait, its length grown while it lasts
            if stops and stops[-1][4] == "walker waiting" and now - (stops[-1][0] + stops[-1][1]) < 0.6:
                stops[-1] = (stops[-1][0], now - stops[-1][0], stops[-1][2], stops[-1][3], stops[-1][4])
            else:
                stops.append((now, 0.25, w[0], w[1], "walker waiting"))
        if way == "car" and w:
            x, z, speed, state, road = w
            # THE ROADS (ADR-0416): every change of road kind, the speed at it; shots onto, along and off the freeway
            if road != road_was:
                print(f"  {now:6.1f} s  onto {road} at {x:.0f} {z:.0f}, {speed * 3.6:.0f} km/h ({state})")
                if road in ("ramp", "freeway") and "fw_onto" not in taken:
                    taken.add("fw_onto")
                    shoot(sock, shots, f"trip_{tag}_fw_1_onto_{road}", t0)
                if road == "street" and been_on_freeway and "fw_off" not in taken:
                    taken.add("fw_off")
                    shoot(sock, shots, f"trip_{tag}_fw_3_off", t0)
                road_was, road_since = road, now
            if road == "freeway":
                been_on_freeway = True
                on_freeway_top = max(on_freeway_top, speed)
                if "fw_on" not in taken and (speed > 22.0 or now - road_since > 8.0):
                    taken.add("fw_on")
                    shoot(sock, shots, f"trip_{tag}_fw_2_on_freeway", t0)
            if "off" not in taken and drive_start is not None and now - drive_start > 2.0 and speed > 1.0:
                taken.add("off")
                shoot(sock, shots, f"trip_{tag}_1_set_off", t0)
            if "turn" not in taken and drive_start is not None and now - drive_start > 8.0 and state == "turning" and speed > 2.0:
                taken.add("turn")
                shoot(sock, shots, f"trip_{tag}_2_turn", t0)
            if speed < 0.05:
                if still_since is None:
                    still_since = (now, x, z, state)
            elif still_since is not None:
                if now - still_since[0] > 1.0:
                    stops.append((still_since[0], now - still_since[0], still_since[1], still_since[2], still_since[3]))
                still_since = None
    took = time.time() - t0
    if "arrived after" in line:
        shoot(sock, shots, f"trip_{tag}_4_arrived", t0)
    print("final:", line)
    if been_on_freeway:
        print(f"  top speed on the freeway: {on_freeway_top * 3.6:.0f} km/h")
    for at, dur, x, z, state in stops:
        print(f"  {'walker' if state.startswith('walker') else 'car'} stood {dur:5.1f} s from {at:6.1f} s at {x:.1f} {z:.1f} ({state})")
    done = "arrived after" in line
    print(f"trip {kind}: {'ARRIVED' if done else 'NOT ARRIVED'} in {took:.1f} s real, {len(stops)} stops")
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
    ap.add_argument("--keys", action="store_true",
                    help="start near, far and freeway trips with the lab's keys (tap Y / I / U) instead of the verb")
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
            key = {"near": "Y", "far": "I", "freeway": "U"}.get(kind) if args.keys else None
            ok = run_trip(sock, kind, number, args.shots, args.timeout, key) and ok
    finally:
        rtctl.stop(proc)
    print("\nprobe", "OK" if ok else "FAILED", f"({time.time() - t0:.0f} s in all)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
