#!/usr/bin/env python3
"""THE CLOUD LAB (#187): the volumetric clouds from fixed poses with the wind pinned, so a change to the clouds is judged
on the same skies -- near, mid-distance, at the horizon (where they went blocky), from inside the deck and from above it
-- with the GPU's time for each.

  python3 tools/cloud_lab.py OUT [--level assets/levels/cloud_lab.json] [--only NAME,...] [--env K=V ...]
                                 [--wind 400] [--hour 15] [--against DIR]

OUT/<pose>.png for every pose, OUT/gpu.txt (RT_GPU_TIMES per pose), OUT/sheet.png (all poses, labelled). With --against
an earlier OUT, OUT/ab_<pose>.png puts the two side by side (that run left, this one right) and OUT/ab_sheet.png all of
them. --env sets RT_* switches for the run (an A/B of a shader change behind one).
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtctl
from rtctl import send
from PIL import Image, ImageDraw

# name: (x, y above the sea, z, pitch, yaw) -- the deck is 900..1600 m on the island's sky; the ground-level poses stand
# on the sea 3.7 km off terrain_lab's island (its radius ~2.2 km), so nothing but sea and sky is in the way
POSES = {
    "overhead": (2600, 20, -2600, 55, 0),     # straight up into the near deck
    "mid": (2600, 20, -2600, 12, 45),         # clouds 2-8 km off
    "horizon": (2600, 20, -2600, 2, 135),     # the far deck over the open sea: where the blocks were
    "sunward": (2600, 20, -2600, 8, 270),     # into the light: silver linings, dark cores
    "inside": (2600, 1250, -2600, 0, 90),     # in the deck
    "above": (2600, 2600, -2600, -18, 45),    # over the tops
    # the far deck all round, low: where the island's clouds went blocky (a bank 8-15 km off, a few degrees up)
    "far_n": (2600, 90, -2600, 3, 0),
    "far_e": (2600, 90, -2600, 3, 90),
    "far_s": (2600, 90, -2600, 3, 180),
    "far_w": (2600, 90, -2600, 3, 270),
    # island_8_nature's beach_curve camera (tools/ref_shots.py), where Glenn saw the far clouds blocky: the same sky here
    "isle_beach": (1460, 90, 6620, -14, 25),
}


def sheet(paths, labels, out, cols=3, w=853):
    ims = [Image.open(p).convert("RGB") for p in paths]
    h = int(w * ims[0].height / ims[0].width)
    rows = (len(ims) + cols - 1) // cols
    S = Image.new("RGB", (cols * w, rows * h), "black")
    for k, (im, lab) in enumerate(zip(ims, labels)):
        t = im.resize((w, h))
        ImageDraw.Draw(t).text((8, 6), lab, fill=(255, 255, 0))
        S.paste(t, ((k % cols) * w, (k // cols) * h))
    S.save(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--level", default="assets/levels/cloud_lab.json")
    ap.add_argument("--only", default="")
    ap.add_argument("--env", nargs="*", default=[])
    ap.add_argument("--wind", type=float, default=400.0, help="the pinned wind time (s)")
    ap.add_argument("--hour", type=float, default=15.0)
    ap.add_argument("--against", default="")
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ, RT_VSYNC="0", RT_PROBE_TAG="cloud_lab", RT_AUDIO="null", RT_GPU_TIMES="1",
               RT_WIND_TIME=str(a.wind))
    for kv in a.env:
        k, v = kv.split("=", 1)
        env[k] = v
    want = [n for n in a.only.split(",") if n] or list(POSES)
    logp = os.path.join(out, "viewer.log")
    proc, sock = rtctl.launch("build-viewer/viewer", a.level, log=open(logp, "wb"), env=env)
    gpu = []
    try:
        send(sock, "info", timeout=2400)
        for o in ("hud", "ui", "debug"):
            send(sock, f"overlay {o} off")
        send(sock, f"daynight {a.hour}")
        for name in want:
            x, y, z, pitch, yaw = POSES[name]
            send(sock, f"camera {x} {y} {z} {pitch} {yaw}")
            time.sleep(8)                                  # streaming, and the clouds' history converging
            n0 = len(open(logp, errors="ignore").read().splitlines())
            time.sleep(5)                                  # (RT_GPU_TIMES logs every 120 frames)
            lines = [l for l in open(logp, errors="ignore").read().splitlines()[n0:] if "[gpu] ms per frame" in l]
            gpu.append(f"{name}: " + (lines[-1].split("ms per frame:")[1].strip() if lines else "no gpu line"))
            print(send(sock, f"shot {os.path.join(out, name + '.png')}"), "|", gpu[-1], flush=True)
            time.sleep(2)
    finally:
        try: send(sock, "quit", timeout=5)
        except Exception: pass
        time.sleep(3)
        if proc.poll() is None: proc.terminate()
    open(os.path.join(out, "gpu.txt"), "w").write("\n".join(gpu) + "\n")
    shots = [n for n in want if os.path.exists(os.path.join(out, n + ".png"))]
    if shots:
        sheet([os.path.join(out, n + ".png") for n in shots], shots, os.path.join(out, "sheet.png"))
    if a.against:
        ab = []
        for n in shots:
            p0 = os.path.join(os.path.abspath(a.against), n + ".png")
            if not os.path.exists(p0):
                continue
            L, R = Image.open(p0).convert("RGB"), Image.open(os.path.join(out, n + ".png")).convert("RGB")
            AB = Image.new("RGB", (L.width * 2 + 8, L.height), "white")
            AB.paste(L, (0, 0)); AB.paste(R, (L.width + 8, 0))
            AB.save(os.path.join(out, f"ab_{n}.png"))
            ab.append(n)
        if ab:
            sheet([os.path.join(out, f"ab_{n}.png") for n in ab], [f"{n}: before | after" for n in ab],
                  os.path.join(out, "ab_sheet.png"), cols=2, w=1280)


if __name__ == "__main__":
    main()
