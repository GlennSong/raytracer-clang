#!/usr/bin/env python3
"""Pictures of an rt_erode bake (ADR-0123): hillshaded before | after, and the water model's maps.

    python3 tools/erosion_preview.py OUT_DIR [--crop x0,z0,x1,z1]   (crop in cells)

Writes OUT_DIR/compare.png (before | after, hillshade) and, when the water maps exist,
OUT_DIR/water.png (hillshade with water depth in blue) and OUT_DIR/wet.png (wetness).
"""
import json
import os
import sys

import numpy as np
from PIL import Image


def load(d, name, n):
    p = os.path.join(d, name)
    if not os.path.exists(p):
        return None
    return np.fromfile(p, dtype=np.float32).reshape(n, n)


def hillshade(h, cell, az=315.0, alt=40.0):
    gz, gx = np.gradient(h, cell)
    slope = np.arctan(np.hypot(gx, gz))
    aspect = np.arctan2(-gx, gz)
    a, z = np.radians(az), np.radians(90.0 - alt)
    s = np.cos(z) * np.cos(slope) + np.sin(z) * np.sin(slope) * np.cos(a - aspect)
    return np.clip(s, 0, 1)


def main():
    d = sys.argv[1]
    meta = json.load(open(os.path.join(d, "meta.json")))
    n, cell = meta["n"], meta["size"] / (meta["n"] - 1)
    crop = None
    if "--crop" in sys.argv:
        crop = [int(v) for v in sys.argv[sys.argv.index("--crop") + 1].split(",")]
    before, after = load(d, "before.f32", n), load(d, "after.f32", n)

    def cut(a):
        return a if crop is None or a is None else a[crop[1]:crop[3], crop[0]:crop[2]]

    b, a = cut(before), cut(after)
    lo, hi = np.percentile(b, 1), np.percentile(b, 99)

    def shade(h):
        tint = np.clip((h - lo) / max(1e-6, hi - lo), 0, 1)
        base = np.stack([0.35 + 0.45 * tint, 0.45 + 0.35 * tint, 0.30 + 0.35 * tint], -1)
        return base * (0.25 + 0.75 * hillshade(h, cell)[..., None])

    img = np.concatenate([shade(b), np.ones((b.shape[0], 4, 3)), shade(a)], 1)
    Image.fromarray((img * 255).astype(np.uint8)).save(os.path.join(d, "compare.png"))
    water = cut(load(d, "water.f32", n))
    if water is not None:
        sh = shade(a)
        depth = np.clip(water / 2.0, 0, 1)[..., None]
        blue = np.array([0.10, 0.30, 0.75])
        Image.fromarray(((sh * (1 - depth) + blue * depth) * 255).astype(np.uint8)).save(os.path.join(d, "water.png"))
        wet = cut(load(d, "wet.f32", n))
        if wet is not None:
            w = np.log1p(wet) / max(1e-6, np.log1p(np.percentile(wet, 99.5)))
            Image.fromarray((np.clip(w, 0, 1) * 255).astype(np.uint8)).save(os.path.join(d, "wet.png"))
    dh = a - b
    print(f"height change: mean {dh.mean():+.2f} m, cut min {dh.min():+.1f} m, fill max {dh.max():+.1f} m, "
          f"rms {np.sqrt((dh ** 2).mean()):.2f} m | backend {meta['backend']}, erode {meta['erodeSeconds']:.1f} s")
    if water is not None:
        print(f"water: {(water > 0.05).mean() * 100:.1f}% of cells over 5 cm, max {water.max():.1f} m, "
              f"cells over 1 m: {(water > 1.0).sum()}")


if __name__ == "__main__":
    main()
