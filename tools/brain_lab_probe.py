#!/usr/bin/env python3
"""brain_lab_probe: the resident with the agent brain (ADR-0441, ADR-0442) watched with nobody at the keyboard.

Launches the viewer on brain_lab, waits for the city, sends `goto <marker>` and polls `brain?` four times a second
until the resident stands at the marker (or the timeout). At each moment worth seeing it takes the follow camera's
screenshot and prints the `brain?` line: at home beside the parked car, in the car, the car sliding out of its space,
stopped on the drive (a light or a stop line), sliding into its space, walking to the marker, standing at it. The
pictures go to ~/Pictures/Raytracer/brain/. While nothing moves (the two minutes of getting in, the three of getting
out, which are travel.lua's own) the sim is run faster (--wait-speed). Stops the viewer it started, by its own process
(never by name: the owner runs `viewer` too).

A picture taken from behind a car cannot say where its nose is against a line painted on the road (the ground beyond
the roof is hidden, and the ground seen level with the rear window is metres ahead of it). So where that matters the
probe also lets the camera go (`brain follow off`), holds the sim still, and takes the same moment FROM ABOVE and from
the side: the person at a marker, the car at each stop on the drive, the car in its space.

    python3 tools/brain_lab_probe.py                 # home to work
    python3 tools/brain_lab_probe.py --to shop --trait has_board
    python3 tools/brain_lab_probe.py --suffix _v2    # the pictures' names end in _v2
"""
import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtctl  # noqa: E402

ABOVE_HEIGHT = 15.0   # metres above the ground the camera looks down from
SIDE_AWAY = 7.5       # metres to the side of the body the side view is taken from
SIDE_HEIGHT = 1.7     # ...and its height, about a standing person's eyes
STOPS_SHOT = 3        # how many stops on the drive are taken from above


def wait_for_file(path, timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return True
        time.sleep(0.25)
    return False


def shoot(sock, shots, name, t0, line):
    path = os.path.join(shots, name + ".png")
    if os.path.exists(path):
        os.remove(path)
    rtctl.send(sock, f"shot {path}")
    ok = wait_for_file(path)
    print(f"  {time.time() - t0:6.1f} s  shot {path}" + ("" if ok else " (did not land)"))
    print(f"           brain? {line}")


def state_of(line):
    """The state's name from a `brain?` line: the part after the plan and the leg, before ` since`."""
    m = re.search(r"\| (\w+) since [\d.]+ s", line)
    return m.group(1) if m else ""


def body_at(line):
    """Where the person is (in the car: the car's middle), from a `brain?` line: plan x and z."""
    m = re.search(r"is at (-?[\d.]+) (-?[\d.]+)", line)
    return (float(m.group(1)), float(m.group(2))) if m else None


def from_above_and_the_side(sock, shots, name, t0, line, above=True, side=True):
    """The same moment with the camera let go: (above) straight down from ABOVE_HEIGHT, and (side) level from SIDE_AWAY
    to the side. The sim is held still meanwhile; afterwards the camera follows the person again. (A marker is on a
    sidewalk under the street's trees, which hide it from above: there only the side is taken.)"""
    at = body_at(line)
    if not at:
        print("           (no position in the brain? line: not taken from above)")
        return
    x, z = at
    rtctl.send(sock, "sim pause")
    rtctl.send(sock, "brain follow off")
    if above:
        rtctl.send(sock, f"camera {x} {ABOVE_HEIGHT} {z} -89 0")
        time.sleep(0.8)
        shoot(sock, shots, name + "_from_above", t0, line)
    if side:
        # yaw 0 looks along -z, so standing SIDE_AWAY toward +z of the body and looking at it
        rtctl.send(sock, f"camera {x} {SIDE_HEIGHT} {z + SIDE_AWAY} -6 0")
        time.sleep(0.8)
        shoot(sock, shots, name + "_from_the_side", t0, line)
    rtctl.send(sock, "brain follow")
    rtctl.send(sock, "sim resume")
    time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--level", default="assets/levels/brain_lab.json")
    ap.add_argument("--to", default="work", help="the marker to send the resident to")
    ap.add_argument("--trait", default="", help="a trait to give them first (has_board)")
    ap.add_argument("--wait-speed", type=float, default=8.0, help="sim speed while they sit in the parked car")
    ap.add_argument("--timeout", type=float, default=900.0, help="real seconds the trip may take")
    ap.add_argument("--shots", default=os.path.expanduser("~/Pictures/Raytracer/brain"))
    ap.add_argument("--log", default="out/zz_brain2/probe_viewer.log")
    ap.add_argument("--tag", default="", help="a prefix for the pictures' names")
    ap.add_argument("--suffix", default="", help="an ending for the pictures' names (_v2), so an earlier set is kept")
    args = ap.parse_args()
    os.makedirs(args.shots, exist_ok=True)
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    t0 = time.time()
    proc, sock = rtctl.launch(args.viewer, args.level, log=open(args.log, "wb"))
    print("viewer pid", proc.pid, "log", args.log)
    done = False
    tag = args.tag or args.to

    def named(name):
        return f"{tag}_{name}{args.suffix}"

    try:
        rtctl.send(sock, "camera?", timeout=1500)
        while True:
            line = rtctl.send(sock, "brain?", timeout=60)
            if line.startswith("ok") or time.time() - t0 > 1500:
                break
            time.sleep(1.0)
        print(f"loaded in {time.time() - t0:.0f} s: {line}")
        print(rtctl.send(sock, "markers?"))
        print(rtctl.send(sock, "brain day off"))
        print(rtctl.send(sock, "brain follow"))
        time.sleep(3.0)
        line = rtctl.send(sock, "brain?")
        shoot(sock, args.shots, named("0_at_home"), t0, line)
        from_above_and_the_side(sock, args.shots, named("0_at_home"), t0, line, above=False)
        if args.trait:
            print(rtctl.send(sock, f"brain trait {args.trait} on"))
        print(rtctl.send(sock, f"goto {args.to}"))
        taken = set()
        speed = 1.0
        entered = {}
        last = ""
        stops = 0
        was_moving = False
        while time.time() - t0 < args.timeout:
            time.sleep(0.25)
            line = rtctl.send(sock, "brain?", timeout=30)
            state = state_of(line)
            now = time.time()
            if state != last:
                print(f"  {now - t0:6.1f} s  {last or '-'} -> {state}")
                entered[state] = entered.get(state, 0) + 1
                last = state
                since = now
            sliding_out = "sliding out of its space" in line
            sliding_in = "sliding into its space" in line
            # run the sim faster only while the person sits in the car in its space
            want = args.wait_speed if (state in ("pulling_out", "parking") and "is standing" in line) else 1.0
            if want != speed:
                rtctl.send(sock, f"sim speed {want}")
                speed = want
            car = re.search(r"([\d.]+) m/s", line)
            car_speed = float(car.group(1)) if car else -1.0

            def once(key, name, above=False, side=False):
                if key in taken:
                    return
                taken.add(key)
                shoot(sock, args.shots, named(name), t0, line)
                if above or side:
                    from_above_and_the_side(sock, args.shots, named(name), t0, line, above, side)

            if state in ("walking", "on_small_wheels") and entered.get(state, 0) == 1 and now - since > 1.5:
                once("walk1", "1_setting_out")
            if state == "pulling_out" and now - since > 1.0:
                once("in_car", "2_in_the_car", above=True, side=True)
            if sliding_out:
                once("out", "3_pulling_out")
            on_road = state == "driving" and "on the road" in line
            if on_road and car_speed > 6.0:
                once("moving", "4_driving")
            if on_road and car_speed > 1.0:
                was_moving = True
            if on_road and was_moving and 0.0 <= car_speed < 0.05 and stops < STOPS_SHOT:
                # each stop on the drive (a stop line, a red light): from behind, then from above
                was_moving = False
                stops += 1
                once(f"stopped{stops}", f"5_stopped_on_the_drive_{stops}", above=True)
            if sliding_in:
                once("in", "6_sliding_in")
            if state == "parking" and "is standing" in line:
                once("parked", "7_in_its_space", above=True, side=True)
            if state == "walking" and entered.get("parking", 0) >= 1 and now - since > 1.0:
                once("walk2", "8_walking_to_the_marker")
            if state == "standing" and "plan done" in line:
                time.sleep(1.0)
                once("there", "9_standing_at_the_marker", side=True)
                done = True
                break
        print("final:", rtctl.send(sock, "brain?"))
    finally:
        rtctl.stop(proc)
    print("\nprobe", "OK" if done else "FAILED", f"({time.time() - t0:.0f} s in all)")
    return 0 if done else 1


if __name__ == "__main__":
    sys.exit(main())
