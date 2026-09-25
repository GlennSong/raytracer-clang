#!/usr/bin/env python3
"""Walk the camera along a path and report the frame times: where does it HITCH?

    python3 tools/walk_probe.py assets/levels/river_valley.json out/walk \
        --path -560,330:-400,150:-250,250 --speed 5

Launches the viewer (--play) with RT_FRAME_STATS=<out>/frames.csv, waits for the level,
samples the player's eye height along the path (teleport + `where?`), then streams `camera` poses at ~60 Hz at that
height, looking along the path. Afterwards it prints the frame-time spread
(p50/p95/p99/max) and every hitch (a frame over --hitch x the median) with its phases and
upload counts, which is usually enough to name the culprit. Stdlib only.
"""

import argparse, csv, math, os, statistics, subprocess, sys, time

sys.path.insert(0, os.path.dirname(__file__))
from lanelab_shots import send   # the control-socket helper


def eye_at(sock, x, z):
    """The player's eye over (x, z): teleport there and read the viewpoint back."""
    send(sock, f"teleport {x:.2f} {z:.2f}")
    for _ in range(100):
        time.sleep(0.03)
        if "pending" not in send(sock, "teleport?"): break
    time.sleep(0.15)   # a frame for the camera to follow
    try:
        return float(send(sock, "where?").split()[2])
    except (IndexError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level")
    ap.add_argument("out")
    ap.add_argument("--path", required=True, help="x,z:x,z:... waypoints")
    ap.add_argument("--speed", type=float, default=5.0, help="m/s")
    ap.add_argument("--hitch", type=float, default=2.0, help="a hitch is a frame over this x the median")
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--load-timeout", type=float, default=1400)
    ap.add_argument("--settle", type=float, default=5)
    a = ap.parse_args()

    pts = [tuple(float(v) for v in p.split(",")) for p in a.path.split(":")]
    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.abspath(os.path.join(a.out, "frames.csv"))
    env = dict(os.environ, RT_FRAME_STATS=csv_path)
    env.setdefault("RT_VSYNC", "0")
    log = open(os.path.join(a.out, "viewer.log"), "wb")
    proc = subprocess.Popen([a.viewer, a.level, "--play"], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
    sock = f"/tmp/raytracer-viewer-{proc.pid}.sock"
    t0 = time.time()
    try:
        while not os.path.exists(sock):
            if proc.poll() is not None or time.time() - t0 > 60: sys.exit("viewer did not open its control socket")
            time.sleep(0.2)
        send(sock, "camera?", timeout=a.load_timeout)
        # the path, resampled every 4 m, with the ground under each sample
        samples = []
        for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
            n = max(1, int(math.hypot(x1 - x0, z1 - z0) / 4.0))
            for k in range(n):
                t = k / n
                samples.append((x0 + (x1 - x0) * t, z0 + (z1 - z0) * t))
        samples.append(pts[-1])
        # the player's eye along the path (every 4th sample; linear between)
        hs, last = [], 0.0
        for k, (x, z) in enumerate(samples):
            if k % 4 == 0 or k == len(samples) - 1:
                h = eye_at(sock, x, z)
                last = h if h is not None else last
            hs.append(last)
        for k in range(len(hs)):   # fill the skipped samples by interpolation
            if k % 4 and k < len(hs) - 1:
                k0, k1 = k - k % 4, min(len(hs) - 1, k - k % 4 + 4)
                hs[k] = hs[k0] + (hs[k1] - hs[k0]) * (k - k0) / (k1 - k0)
        time.sleep(a.settle)
        # walk: stream poses at ~60 Hz
        d = [0.0]
        for i in range(1, len(samples)):
            d.append(d[-1] + math.hypot(samples[i][0] - samples[i - 1][0], samples[i][1] - samples[i - 1][1]))
        total, start = d[-1], time.time()
        mark = send(sock, "camera?").split("frame=")[-1]
        print(f"walking {total:.0f} m at {a.speed} m/s from frame {mark}", flush=True)
        i = 0
        while True:
            s = (time.time() - start) * a.speed
            if s >= total: break
            while i + 1 < len(d) - 1 and d[i + 1] < s: i += 1
            t = (s - d[i]) / max(1e-6, d[i + 1] - d[i])
            x = samples[i][0] + (samples[i + 1][0] - samples[i][0]) * t
            z = samples[i][1] + (samples[i + 1][1] - samples[i][1]) * t
            y = hs[i] + (hs[i + 1] - hs[i]) * t
            yaw = math.degrees(math.atan2(samples[i + 1][0] - samples[i][0], -(samples[i + 1][1] - samples[i][1])))
            send(sock, f"camera {x:.2f} {y:.2f} {z:.2f} -4 {yaw:.2f}")
            time.sleep(1 / 60)
        end = send(sock, "camera?").split("frame=")[-1]
    finally:
        proc.terminate()
        try: proc.wait(10)
        except subprocess.TimeoutExpired: proc.kill()

    rows = list(csv.DictReader(l for l in open(csv_path) if not l.startswith("#")))
    fcol = "frame" if rows and "frame" in rows[0] else None
    walk = rows[-(int(end) - int(mark)):] if not fcol else [r for r in rows if int(mark) <= int(r[fcol]) <= int(end)]
    ms = [float(r["total_ms"]) for r in walk]
    if not ms: sys.exit("no frames captured during the walk")
    q = sorted(ms)
    pct = lambda p: q[min(len(q) - 1, int(p * len(q)))]
    med = statistics.median(ms)
    lines = [f"walk frames {mark}..{end}",
             f"{len(ms)} frames: p50 {med:.1f}  p95 {pct(.95):.1f}  p99 {pct(.99):.1f}  max {q[-1]:.1f} ms"]
    cols = [c for c in ("update_ms", "fixed_ms", "render_ms", "wait_ms", "encode_ms", "submit_ms", "gpu_ms",
                        "fixed_steps", "mesh_uploads", "texture_uploads", "draw_calls", "instances") if c in walk[0]]
    hitches = [r for r in walk if float(r["total_ms"]) > a.hitch * med]
    lines.append(f"{len(hitches)} hitches (> {a.hitch:g} x median):")
    for r in hitches[:40]:
        lines.append("  " + f"frame {r.get('frame', '?')}  {float(r['total_ms']):6.1f} ms  " +
                     "  ".join(f"{c.replace('_ms', '')}={r[c]}" for c in cols))
    print("\n".join(lines))
    with open(os.path.join(a.out, "summary.txt"), "w") as fsum:   # kept beside the capture
        fsum.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
