#!/usr/bin/env python3
"""CLOUD FLICKER, measured (#187 step 1, #59): a still camera, a burst of consecutive frames, and how much the sky changes
frame to frame that is NOT the clouds' smooth drift -- each frame against the mean of its two neighbours, so a cloud
moving steadily on the wind scores near zero and a shimmer, a jitter or a pattern crawling scores its full size.

  python3 tools/cloud_flicker.py LEVEL OUT [--views NAME:x:y:z:pitch:yaw ...] [--frames 32] [--pan DEG_PER_FRAME]

Writes OUT/<view>_NNN.png and OUT/summary.txt: per view, the sky's high-frequency temporal noise (mean |f - (prev+next)/2|
over the sky pixels, 0..255) and its 99th percentile, and a heat map OUT/<view>_flicker.png of where it lives. Run it
before and after a change to the clouds and compare the numbers. The sky is the pixels brighter than the ground's
median that lie above the frame's middle, in the first frame (no scene mask is needed for a fixed camera).
"""
import argparse, glob, os, sys, time
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
import rtctl
from rtctl import send


def measure(paths):
    fr = [np.asarray(Image.open(p).convert("L"), dtype=np.float32)[60:] for p in sorted(paths)]   # (the PAUSED label off)
    if len(fr) < 3:
        return None
    a = np.stack(fr)
    hf = np.abs(a[1:-1] - 0.5 * (a[:-2] + a[2:]))          # each frame against its neighbours' mean
    h, w = fr[0].shape
    sky = np.zeros((h, w), bool)
    sky[: h // 2] = fr[0][: h // 2] > np.median(fr[0][h // 2:])
    if sky.sum() < 100:
        sky[: h // 3] = True
    per = hf[:, sky]
    heat = hf.mean(0)
    return float(per.mean()), float(np.percentile(per, 99)), heat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level")
    ap.add_argument("out")
    ap.add_argument("--views", nargs="*", default=[])
    ap.add_argument("--frames", type=int, default=32)
    ap.add_argument("--pan", type=float, default=0.0, help="yaw degrees per frame (0: still)")
    ap.add_argument("--move", type=float, default=0.0, help="metres sideways per frame (0: still)")
    ap.add_argument("--settle", type=float, default=0.8, help="s to let each moved camera settle before its frame")
    ap.add_argument("--fly", type=float, default=0.0,
                    help="m/s: fly the camera sideways continuously through one burst of consecutive frames (the real "
                         "moving case; --move's frames are seconds apart, so the wind's drift swamps them)")
    ap.add_argument("--hour", type=float, default=15.0)
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ, RT_VSYNC="0", RT_PROBE_TAG="cloud_flicker", RT_AUDIO="null")
    proc, sock = rtctl.launch("build-viewer/viewer", a.level, log=open(os.path.join(out, "viewer.log"), "wb"), env=env)
    lines = []
    try:
        send(sock, "info", timeout=2400)
        send(sock, f"daynight {a.hour}")
        send(sock, "sim pause")
        for v in a.views:
            name, x, y, z, pitch, yaw = v.split(":")
            send(sock, f"camera {x} {y} {z} {pitch} {yaw}")
            time.sleep(6)                                     # let streaming and the clouds' history settle
            prefix = os.path.join(out, name)
            if a.fly != 0.0:                                  # flown through a burst of consecutive frames
                import math
                yr = math.radians(float(yaw))
                sx, sz = math.cos(yr), math.sin(yr)
                send(sock, f"shots {prefix} {a.frames}")
                t0 = time.time()
                while time.time() - t0 < a.frames * 0.05 + 3.0:
                    d = a.fly * (time.time() - t0)
                    send(sock, f"camera {float(x) + sx * d} {y} {float(z) + sz * d} {pitch} {yaw}")
                    time.sleep(0.008)
            elif a.move != 0.0:                                 # a slow sideways move, one settled frame per step
                import math
                yr = math.radians(float(yaw))
                sx, sz = math.cos(yr), math.sin(yr)
                for k in range(a.frames):
                    send(sock, f"camera {float(x) + sx * a.move * k} {y} {float(z) + sz * a.move * k} {pitch} {yaw}")
                    time.sleep(a.settle)
                    send(sock, f"shots {prefix}_{k:03d}x 1")
                    time.sleep(0.3)
            elif a.pan == 0.0:
                send(sock, f"shots {prefix} {a.frames}")
                time.sleep(a.frames * 0.2 + 4)
            else:                                             # a slow pan, one frame per step
                for k in range(a.frames):
                    send(sock, f"camera {x} {y} {z} {pitch} {float(yaw) + a.pan * k}")
                    time.sleep(a.settle)
                    send(sock, f"shots {prefix}_{k:03d}x 1")
                    time.sleep(0.3)
            time.sleep(2)
            files = sorted(glob.glob(prefix + "_*.png"))
            r = measure(files)
            if r is None:
                lines.append(f"{name}: too few frames ({len(files)})")
                continue
            mean, p99, heat = r
            Image.fromarray(np.clip(heat * 12.0, 0, 255).astype(np.uint8)).save(os.path.join(out, f"{name}_flicker.png"))
            lines.append(f"{name}: sky temporal noise mean {mean:.2f}, p99 {p99:.1f} (0..255) over {len(files)} frames")
            print(lines[-1], flush=True)
    finally:
        try: send(sock, "quit", timeout=5)
        except Exception: pass
        time.sleep(3)
        if proc.poll() is None: proc.terminate()
    open(os.path.join(out, "summary.txt"), "w").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
