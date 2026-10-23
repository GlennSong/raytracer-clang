#!/usr/bin/env python3
"""Fly the camera over a level and measure what POPS: what arrives late, how close, how often.

    python3 tools/fly_probe.py assets/levels/island_8_nature.json out/fly \
        --path -5200,-440:-2900,-440 --agl 40 --speed 30 --pitch -10 --shots 8

Launches the viewer (--play) with RT_FRAME_STATS, RT_RESIDENCY_CSV (one line per streamed item that
arrives: how close the camera already was, how long it waited) and RT_GRASS_LOG (bare grass tiles
once a second). It samples the ground along the path (teleport + `where?`), keeps the camera --agl
metres above the highest ground in a 400 m window (no bobbing over every roof), streams `camera`
poses at ~60 Hz, and takes --shots evenly spaced screenshots. Afterwards it writes summary.txt:

- frames: p50/p95/p99/max and the hitches (as walk_probe);
- per streamed client: arrivals, how many arrived LATE (the camera already inside the band where the
  item is drawn, so the stand-in showed and was swapped in view), the late share, how far away they
  were when they arrived (p10/p50), their wait, and the item-seconds late;
- grass: the most bare tiles at once, the nearest bare tile, tile-seconds bare.

The jump at the start (the teleport onto the path) is excluded. Stdlib only.
"""

import argparse, csv, json, math, os, re, statistics, subprocess, sys, time

sys.path.insert(0, os.path.dirname(__file__))
from lanelab_shots import send   # the control-socket helper
from walk_probe import eye_at


def frame_now(sock):
    # `info`, not `camera?`: any question (`name?`) switches the island-wide debug counts on for 60 s
    # (telemetry.askedAt), a 20-70 ms frame every 30-60 frames -- the probe would measure itself
    return re.search(r"frame=(\d+)", send(sock, "info")).group(1)


def pct(q, p):
    return q[min(len(q) - 1, int(p * len(q)))] if q else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level")
    ap.add_argument("out")
    ap.add_argument("--path", required=True, help="x,z:x,z:... waypoints")
    ap.add_argument("--agl", type=float, default=60.0, help="metres above the ground")
    ap.add_argument("--speed", type=float, default=50.0, help="m/s")
    ap.add_argument("--pitch", type=float, default=-10.0, help="degrees, down is negative")
    ap.add_argument("--shots", type=int, default=6, help="screenshots along the flight")
    ap.add_argument("--hitch", type=float, default=2.0)
    ap.add_argument("--min-ms", type=float, default=25.0)
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--load-timeout", type=float, default=1400)
    ap.add_argument("--settle", type=float, default=65,
                    help="seconds at the start pose before flying: past the 60 s the citysim's ? counts run after any "
                         "question (the ground sampling's where?), which put a 25 ms frame every 30 into the flight")
    ap.add_argument("--report", action="store_true", help="no flight: summarise OUT's flight again from its files")
    a = ap.parse_args()
    if a.report:
        out = os.path.abspath(a.out)
        report(a, out, json.load(open(os.path.join(out, "marks.json"))))
        return

    pts = [tuple(float(v) for v in p.split(",")) for p in a.path.split(":")]
    os.makedirs(a.out, exist_ok=True)
    out = os.path.abspath(a.out)
    frames_csv, arrivals_csv = os.path.join(out, "frames.csv"), os.path.join(out, "arrivals.csv")
    env = dict(os.environ, RT_FRAME_STATS=frames_csv, RT_RESIDENCY_CSV=arrivals_csv, RT_GRASS_LOG="1",
               RT_RESIDENCY_LOG="1")
    env.setdefault("RT_VSYNC", "0")
    env.setdefault("RT_PROBE_TAG", "fly_probe")
    log_path = os.path.join(out, "viewer.log")
    log = open(log_path, "wb")
    proc = subprocess.Popen([a.viewer, a.level, "--play"], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
    sock = f"/tmp/raytracer-viewer-{proc.pid}.sock"
    t0 = time.time()
    try:
        while not os.path.exists(sock):
            if proc.poll() is not None or time.time() - t0 > 60: sys.exit("viewer did not open its control socket")
            time.sleep(0.2)
        send(sock, "info", timeout=a.load_timeout)
        print(f"loaded in {time.time() - t0:.0f} s", flush=True)
        # the path every 10 m; the ground every 100 m
        samples = []
        for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
            n = max(1, int(math.hypot(x1 - x0, z1 - z0) / 10.0))
            samples += [(x0 + (x1 - x0) * k / n, z0 + (z1 - z0) * k / n) for k in range(n)]
        samples.append(pts[-1])
        ground = {}
        for k in list(range(0, len(samples), 10)) + [len(samples) - 1]:
            h = eye_at(sock, *samples[k])
            if h is not None: ground[k] = h
        keys = sorted(ground)
        if not keys: sys.exit("no ground heights along the path")
        hs = []
        for k in range(len(samples)):   # linear between the sampled ones
            lo = max([g for g in keys if g <= k], default=keys[0]); hi = min([g for g in keys if g >= k], default=keys[-1])
            hs.append(ground[lo] if hi == lo else ground[lo] + (ground[hi] - ground[lo]) * (k - lo) / (hi - lo))
        w = 20   # the highest ground within +-200 m, so the camera holds height over a town instead of bobbing
        ys = [max(hs[max(0, k - w):k + w + 1]) + a.agl for k in range(len(hs))]
        d = [0.0]
        for i in range(1, len(samples)):
            d.append(d[-1] + math.hypot(samples[i][0] - samples[i - 1][0], samples[i][1] - samples[i - 1][1]))
        total = d[-1]

        def pose(s):
            i = 0
            while i + 1 < len(d) - 1 and d[i + 1] < s: i += 1
            t = min(1.0, (s - d[i]) / max(1e-6, d[i + 1] - d[i]))
            x = samples[i][0] + (samples[i + 1][0] - samples[i][0]) * t
            z = samples[i][1] + (samples[i + 1][1] - samples[i][1]) * t
            y = ys[i] + (ys[i + 1] - ys[i]) * t
            yaw = math.degrees(math.atan2(samples[i + 1][0] - samples[i][0], -(samples[i + 1][1] - samples[i][1])))
            return x, y, z, yaw

        x, y, z, yaw = pose(0.0)
        send(sock, f"camera {x:.2f} {y:.2f} {z:.2f} {a.pitch:.1f} {yaw:.2f}")
        time.sleep(a.settle)   # the jump onto the path loads and settles here
        start_mark = frame_now(sock)
        with open(log_path, "rb") as f: log_from = len(f.read())
        arrivals_from = sum(1 for _ in open(arrivals_csv)) if os.path.exists(arrivals_csv) else 0
        print(f"flying {total:.0f} m at {a.speed:g} m/s, {a.agl:g} m up, from frame {start_mark}", flush=True)
        shot_at = [total * (k + 0.5) / a.shots for k in range(a.shots)] if a.shots > 0 else []
        shot_frames = []   # a capture stalls its frame ~0.5 s: those frames are the probe's, not the level's
        start = time.time()
        while True:
            s = (time.time() - start) * a.speed
            if s >= total: break
            x, y, z, yaw = pose(s)
            send(sock, f"camera {x:.2f} {y:.2f} {z:.2f} {a.pitch:.1f} {yaw:.2f}")
            if shot_at and s >= shot_at[0]:
                send(sock, f"shot {os.path.join(out, f'shot_{a.shots - len(shot_at):02d}_{int(x)}_{int(z)}.png')}")
                shot_frames.append(int(frame_now(sock)))
                shot_at.pop(0)
            time.sleep(1 / 60)
        end_mark = frame_now(sock)
        time.sleep(1.5)   # the last log lines
    finally:
        proc.terminate()
        try: proc.wait(15)
        except subprocess.TimeoutExpired: proc.kill()

    marks = dict(start_mark=int(start_mark), end_mark=int(end_mark), shot_frames=shot_frames, log_from=log_from,
                 arrivals_from=arrivals_from, total=total)
    with open(os.path.join(out, "marks.json"), "w") as f:
        json.dump(marks, f)
    report(a, out, marks)


def report(a, out, marks):
    """The summary of a flight already flown (also --report OUT: again, from its files)."""
    frames_csv, arrivals_csv, log_path = (os.path.join(out, n) for n in ("frames.csv", "arrivals.csv", "viewer.log"))
    start_mark, end_mark, shot_frames = marks["start_mark"], marks["end_mark"], marks["shot_frames"]
    log_from, arrivals_from, total = marks["log_from"], marks["arrivals_from"], marks["total"]
    lines = [f"{a.level}: {total:.0f} m at {a.speed:g} m/s, {a.agl:g} m above the ground, pitch {a.pitch:g}",
             f"path {a.path}", ""]
    # FRAMES
    rows = [r for r in csv.DictReader(l for l in open(frames_csv) if not l.startswith("#")) if r.get("total_ms")]
    fly = [r for r in rows if "frame" in r and int(start_mark) <= int(r["frame"]) <= int(end_mark)
           and not any(-2 <= int(r["frame"]) - f <= 6 for f in shot_frames)]
    ms = sorted(float(r["total_ms"]) for r in fly)
    if ms:
        med = statistics.median(ms)
        lines.append(f"frames: {len(ms)}  p50 {med:.1f}  p95 {pct(ms, .95):.1f}  p99 {pct(ms, .99):.1f}  max {ms[-1]:.1f} ms")
        hitches = [r for r in fly if float(r["total_ms"]) > max(a.hitch * med, a.min_ms)]
        cols = [c for c in ("update_ms", "render_ms", "encode_ms", "gpu_ms", "mesh_uploads") if c in fly[0]]
        lines.append(f"  {len(hitches)} hitches (> {a.hitch:g} x median and > {a.min_ms:g} ms)")
        for r in hitches[:15]:
            lines.append("    " + f"frame {r['frame']}  {float(r['total_ms']):6.1f} ms  " + "  ".join(f"{c.replace('_ms', '')}={r[c]}" for c in cols))
    # STREAMED ITEMS
    arr = list(csv.DictReader(open(arrivals_csv)))[max(0, arrivals_from - 1):] if os.path.exists(arrivals_csv) else []
    arr = [r for r in arr if r["how"] != "jump"]
    # where each tier is DRAWN: an outer tier (the flat facades) only beyond the detail ring, so arriving inside it is
    # no pop (read from the level, so a viewer built before Residency::Item::drawnFrom measures the same)
    cs = json.load(open(a.level)).get("citysim", {})
    drawn_from = {"building facades": float(cs.get("detailDistance", 700.0))}
    for r in arr:
        r["late"] = "1" if drawn_from.get(r["client"], 0.0) <= float(r["dist"]) <= float(r["due"]) else "0"
    lines.append("")
    for client in sorted({r["client"] for r in arr}):
        rs = [r for r in arr if r["client"] == client]
        late = [r for r in rs if r["late"] == "1"]
        deep = sorted(float(r["dist"]) for r in late)
        wait = sorted(float(r["waited_ms"]) for r in rs)
        lines.append(f"{client}: {len(rs)} arrived, {len(late)} LATE ({100.0 * len(late) / max(1, len(rs)):.0f}%)"
                     + (f", in view at {pct(deep, .1):.0f} m (p10) / {pct(deep, .5):.0f} m (p50) from the camera"
                        f" (drawn {drawn_from.get(client, 0.0):.0f}-{float(late[0]['due']):.0f} m)" if late else "")
                     + f"; waited p50 {pct(wait, .5):.0f} ms, p90 {pct(wait, .9):.0f} ms")
    with open(log_path, "rb") as f:
        tail = f.read()[log_from:].decode(errors="replace")
    res = re.findall(r"\[residency\] (.+?): .*?late (\d+) \(nearest (\S+) m\), (\d+) arrived late, (\S+) item-s late", tail)
    worst = {}
    for client, now, nearest, _, secs in res:
        w0 = worst.get(client, (0, 1e9, 0.0))
        nr = float(nearest) if float(nearest) >= 0 else 1e9
        worst[client] = (max(w0[0], int(now)), min(w0[1], nr), float(secs))
    for client, (most, nearest, secs) in sorted(worst.items()):
        lines.append(f"  {client}: at most {most} late at once (5 s samples), nearest late {nearest if nearest < 1e9 else '-'} m, "
                     f"{secs:.0f} item-s late since load")
    grass = re.findall(r"\[grass\] bare (\d+) tiles, nearest (\S+) m, (\S+) tile-s bare in all", tail)
    if grass:
        g0 = float(grass[0][2])
        near = [float(n) for b, n, _ in grass if int(b) > 0 and float(n) >= 0]
        lines.append(f"grass: at most {max(int(b) for b, _, _ in grass)} tiles bare at once, nearest bare "
                     f"{min(near) if near else '-'} m, {float(grass[-1][2]) - g0:.0f} tile-s bare over the flight")
    print("\n".join(lines))
    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
