#!/usr/bin/env python3
"""Headless lanelab acceptance shots (ADR-0083) over the viewer's control channel
(ADR-0078, docs/control-channel.md): launch the viewer ONCE on a lab level, then for
each shot stage the freecam (`camera x y z pitch yaw`, which detaches it from the player
so play mode — physics live, no editor chrome — frames correctly) and arm `shot path`.

Cameras come from a lanelab stats.json ("cameras": one per anchored gore, engine world
coordinates x, height, z) and/or explicit --shot name=ex,ey,ez:tx,ty,tz arguments.
--near adds a second, closer frame per camera (half the distance, 3 m up) for reading
the merge markings.

Usage: tools/lanelab_shots.py LEVEL OUT_DIR [--cams stats.json] [--names a,b] [--shot ...]
       [--near] [--edit] [--viewer build-viewer/viewer] [--settle 3]
"""
import argparse, json, math, os, re, socket, subprocess, sys, time

def pose(eye, target):
    ex, ey, ez = eye; tx, ty, tz = target
    dx, dy, dz = tx - ex, ty - ey, tz - ez
    L = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, dy / L))))
    yaw = math.degrees(math.atan2(dx / L, -dz / L))
    return pitch, yaw

def send(sock_path, line, timeout=180.0):
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(timeout); s.connect(sock_path); s.sendall((line + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk: raise RuntimeError("viewer closed the connection")
            buf += chunk
    return buf.split(b"\n", 1)[0].decode()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level"); ap.add_argument("out")
    ap.add_argument("--cams"); ap.add_argument("--names")
    ap.add_argument("--shot", action="append", default=[])
    ap.add_argument("--near", action="store_true"); ap.add_argument("--edit", action="store_true")
    ap.add_argument("--teleport", action="append", default=[], help="name=x,z: put the player there (ground-snapped) and shoot from the play camera")
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--settle", type=float, default=3.0, help="seconds after the level answers before the first shot")
    ap.add_argument("--cmd", action="append", default=[], help="a raw control-channel line to send once the level answers (e.g. 'citymap out/map.svg blocks,lots')")
    ap.add_argument("--hour", type=float, default=11.0, help="hold the day/night clock at this hour for the shots (the lab levels' clock runs; night frames are useless). <0 leaves it alone")
    ap.add_argument("--load-timeout", type=float, default=900.0, help="seconds to wait for the level to answer (a lanelab build at load can take minutes)")
    a = ap.parse_args()
    shots = []
    if a.cams:
        want = set(a.names.split(",")) if a.names else None
        for c in json.load(open(a.cams)).get("cameras", []):
            if want is None or c["name"] in want: shots.append((c["name"], c["eye"], c["target"]))
    for s in a.shot:
        name, rest = s.split("=", 1); e, t = rest.split(":")
        shots.append((name, [float(v) for v in e.split(",")], [float(v) for v in t.split(",")]))
    if a.near:
        near = []
        for name, e, t in shots:
            near.append((name + "_near", [t[0] + (e[0] - t[0]) * 0.5, t[1] + 3.0, t[2] + (e[2] - t[2]) * 0.5], t))
        shots += near
    for t in a.teleport:
        name, rest = t.split("=", 1); x, z = [float(v) for v in rest.split(",")]
        shots.append((name, None, (x, z)))
    if not shots and not a.cmd: sys.exit("no shots: give --cams, --shot, --teleport or --cmd")
    os.makedirs(a.out, exist_ok=True)
    log = open(os.path.join(a.out, "viewer.log"), "wb")
    proc = subprocess.Popen([a.viewer, a.level, "--edit" if a.edit else "--play"],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    sock_path = f"/tmp/raytracer-viewer-{proc.pid}.sock"
    t0 = time.time()
    try:
        while not os.path.exists(sock_path):
            if proc.poll() is not None or time.time() - t0 > 60: sys.exit("viewer did not open its control socket")
            time.sleep(0.2)
        print(f"viewer up ({time.time() - t0:.1f}s); waiting for the level:", send(sock_path, "camera?", timeout=a.load_timeout), flush=True)
        time.sleep(a.settle)
        if a.hour >= 0: send(sock_path, f"daynight {a.hour:g}"); send(sock_path, "daynight hold")
        for c in a.cmd: print("   ", c, "->", send(sock_path, c), flush=True)
        ok = 0
        queue = list(shots); extra = queue
        for name, eye, target in queue:   # teleport shots append their plan/oblique frames to this queue
            if eye is None:   # a teleport shot: the play camera, wherever the player lands
                send(sock_path, f"teleport {target[0]:.1f} {target[1]:.1f}"); time.sleep(1.5)
                where = send(sock_path, "where?")
                nums = re.findall(r"-?\d+\.?\d*", where)
                if len(nums) >= 3:   # the viewpoint stands near the player: a plan view and an oblique from there too
                    x, y, z = float(nums[0]), float(nums[1]), float(nums[2])
                    extra.append((name + "_top", [x, y + 45.0, z + 0.01], [x, y, z]))
                    extra.append((name + "_oblique", [x + 28.0, y + 16.0, z + 28.0], [x, y, z]))
            else:
                pitch, yaw = pose(eye, target)
                send(sock_path, f"camera {eye[0]:.2f} {eye[1]:.2f} {eye[2]:.2f} {pitch:.2f} {yaw:.2f}")
                time.sleep(0.4)
            out = os.path.abspath(os.path.join(a.out, name + ".png"))
            if os.path.exists(out): os.remove(out)
            reply = send(sock_path, f"shot {out}")
            t1 = time.time()
            while not os.path.exists(out) and time.time() - t1 < 30: time.sleep(0.1)
            time.sleep(0.2)
            got = os.path.exists(out); ok += got
            print(f"  [{'ok' if got else 'FAIL'}] {name}  ({reply})", flush=True)
        print(f"{ok}/{len(queue)} frames in {a.out} ({time.time() - t0:.0f}s)")
    finally:
        proc.terminate()
        try: proc.wait(5)
        except subprocess.TimeoutExpired: proc.kill()

if __name__ == "__main__":
    main()
