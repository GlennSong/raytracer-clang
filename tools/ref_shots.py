#!/usr/bin/env python3
"""THE REALISM PASS'S FIXED SHOTS (#187 step 0): the island from the poses that match the reference stills (Glenn's
four, from the North Atlantic islands video), each at its own hour, so every change to the water, the light, the ground
or the clouds is judged side by side with the same views. Each pose names its height ABOVE THE GROUND under it, so a
re-plan that moves the ground keeps the view.

  python3 tools/ref_shots.py LEVEL OUT [--only NAME,...] [--env K=V ...]

OUT/<name>.png for every pose; --env sets an RT_* switch for an A/B (RT_CLOUD_HISTORY=0).
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtctl
from rtctl import send
from walk_probe import eye_at

# name: (x, z, metres above the ground, pitch, yaw, hour) -- the reference each mirrors in the comment
POSES = {
    "beach_curve": (1460, 6620, 90, -18, 75, 17.2),    # black-sand beach curving away, surf along it, from above the sea
    "heath_slope": (-560, 5600, 3, -10, 20, 17.6),     # a low eye on a hillside, the heath down to the coast and the sea
    "coast_aerial": (600, 6500, 420, -88, 30, 16.4),   # straight down on the coast: the shallows, the surf, the forest
    "forest_lake": (-1450, 3400, 4, -2, 300, 11.0),    # a lake at eye level, the shore's trees in it
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level")
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--env", nargs="*", default=[])
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ, RT_VSYNC="0", RT_PROBE_TAG="ref_shots", RT_AUDIO="null")
    for kv in a.env:
        k, v = kv.split("=", 1)
        env[k] = v
    want = [n for n in a.only.split(",") if n] or list(POSES)
    proc, sock = rtctl.launch("build-viewer/viewer", a.level, log=open(os.path.join(out, "viewer.log"), "wb"), env=env)
    try:
        send(sock, "info", timeout=2400)
        send(sock, "overlay hud off")
        for name in want:
            x, z, up, pitch, yaw, hour = POSES[name]
            send(sock, f"daynight {hour}")
            g = eye_at(sock, x, z)
            g = (g - 1.6) if g is not None else 0.0   # (the eye: the ground is under it)
            send(sock, f"camera {x} {g + up:.1f} {z} {pitch} {yaw}")
            time.sleep(10)
            print(name, send(sock, f"shot {os.path.join(out, name + '.png')}"), flush=True)
            time.sleep(2)
    finally:
        try: send(sock, "quit", timeout=5)
        except Exception: pass
        time.sleep(3)
        if proc.poll() is None: proc.terminate()


if __name__ == "__main__":
    main()
