#!/usr/bin/env python3
"""A headless ELEVATOR RIDE for frames (skyscrapers v2 M5): launch the viewer on a
level with the player spawned at a hoistway door (RT_SPAWN), call the cab over the
control channel, teleport the player into it facing the door, pick a floor, go, and
shoot the PLAY camera every so often through the ride — what a rider sees.

Usage: tools/elevator_ride.py LEVEL OUT_DIR --door x,z --normal nx,nz --floor-y Y
       [--floors 20] [--viewer build-viewer/viewer] [--hour 11] [--every 1.5] [--seconds 14]
`--door` is the hoistway door's foot, `--normal` its outward (lobby-facing) normal,
`--floor-y` the ground storey's floor height; RT_SPAWN is derived (the door, 1.2 m out).
"""
import argparse, math, os, re, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lanelab_shots import send

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level"); ap.add_argument("out")
    ap.add_argument("--door", required=True); ap.add_argument("--normal", required=True)
    ap.add_argument("--floor-y", type=float, required=True)
    ap.add_argument("--floors", type=int, default=20)
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--hour", type=float, default=11.0)
    ap.add_argument("--every", type=float, default=1.5)
    ap.add_argument("--seconds", type=float, default=14.0)
    ap.add_argument("--settle", type=float, default=8.0)
    a = ap.parse_args()
    dx, dz = [float(v) for v in a.door.split(",")]
    nx, nz = [float(v) for v in a.normal.split(",")]
    nl = math.hypot(nx, nz); nx, nz = nx / nl, nz / nl
    os.makedirs(a.out, exist_ok=True)
    env = dict(os.environ)
    env["RT_SPAWN"] = f"{dx + nx * 1.2:.2f},{a.floor_y + 0.9:.2f},{dz + nz * 1.2:.2f}"
    log = open(os.path.join(a.out, "ride.log"), "wb")
    proc = subprocess.Popen([a.viewer, a.level, "--play"], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
    sock = f"/tmp/raytracer-viewer-{proc.pid}.sock"
    t0 = time.time()
    try:
        while not os.path.exists(sock):
            if proc.poll() is not None or time.time() - t0 > 60: sys.exit("viewer did not open its control socket")
            time.sleep(0.2)
        print("level:", send(sock, "camera?", timeout=900), flush=True)
        time.sleep(a.settle)
        if a.hour >= 0: send(sock, f"daynight {a.hour:g}"); send(sock, "daynight hold")
        def shot(name, pitch=None):
            # A pitch re-poses the player where they stand (a teleport keeps x y z).
            if pitch is not None:
                nums = re.findall(r"-?\d+\.?\d*", send(sock, "where?"))
                if len(nums) >= 5:
                    x, y, z, _, yw = [float(v) for v in nums[:5]]
                    send(sock, f"teleport {x:.2f} {y - 0.7:.2f} {z:.2f} {pitch:.0f} {yw:.1f}")
                    time.sleep(0.25)
            out = os.path.abspath(os.path.join(a.out, name + ".png"))
            if os.path.exists(out): os.remove(out)
            send(sock, f"shot {out}")
            t1 = time.time()
            while not os.path.exists(out) and time.time() - t1 < 30: time.sleep(0.1)
            print(f"  [{'ok' if os.path.exists(out) else 'FAIL'}] {name}  {send(sock, 'elevator?')}", flush=True)
        # At the door, facing it (into the shaft): the hall call.
        ox, oz = dx + nx * 1.5, dz + nz * 1.5
        yawIn = math.degrees(math.atan2(-nx, nz))
        print("to door:", send(sock, f"teleport {ox:.2f} {a.floor_y + 0.9:.2f} {oz:.2f} -5 {yawIn:.1f}"), flush=True)
        time.sleep(1.5)
        print("at door:", send(sock, "elevator?"), send(sock, "where?"), flush=True)
        print("call:", send(sock, "elevator call"), flush=True)
        time.sleep(3.5)
        shot("ride_00_lobby_cab_open")
        # Into the cab, 1 m past the door, facing back out at it.
        ix, iz = dx - nx * 1.0, dz - nz * 1.0
        yawOut = math.degrees(math.atan2(nx, -nz))
        print("teleport in:", send(sock, f"teleport {ix:.2f} {a.floor_y + 0.9:.2f} {iz:.2f} 0 {yawOut:.1f}"), flush=True)
        time.sleep(1.0)
        print("in cab?", send(sock, "elevator?"), "where", send(sock, "where?"), flush=True)
        shot("ride_01_in_cab_doors_open")
        shot("ride_01b_in_cab_look_down", pitch=-55)
        shot("ride_01c_in_cab_look_up", pitch=55)
        print("pick:", send(sock, f"elevator pick {a.floors}"), flush=True)
        time.sleep(0.3)
        print("go:", send(sock, "elevator call"), flush=True)
        t1 = time.time(); k = 2
        pitches = [0, -55, 55]
        while time.time() - t1 < a.seconds:
            time.sleep(a.every)
            shot(f"ride_{k:02d}_t{time.time() - t1:04.1f}s_p{pitches[k % 3]:+d}", pitch=pitches[k % 3])
            k += 1
        shot("ride_end_level", pitch=0)
        print("where:", send(sock, "where?"), flush=True)
    finally:
        proc.terminate()
        try: proc.wait(5)
        except subprocess.TimeoutExpired: proc.kill()

if __name__ == "__main__":
    main()
