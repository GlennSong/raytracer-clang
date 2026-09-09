#!/usr/bin/env python3
"""Rasterise a lanelab plan SVG (writePlanSvg: rect, path M/L/Z, circle, text) with matplotlib.
magick ran out of memory on the 10 MB metro plan; this draws the same polygons as PathPatches.
Usage: svg2png.py plan.svg out.png [width_px]"""
import re, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch, Circle

args = [a for a in sys.argv[1:] if not a.startswith("--layers=")]; keep = [a.split("=", 1)[1].split(",") for a in sys.argv[1:] if a.startswith("--layers=")]
src, dst = args[0], args[1]; width = int(args[2]) if len(args) > 2 else 2400
svg = open(src).read()
# the city map's legend is a click-to-toggle panel meant for a browser; in a raster it hides a quadrant
svg = re.sub(r"<g id='layer-legend'.*?</g>", "", svg, flags=re.S)
if keep:   # a city map: keep only the named layers (roads,curbs,sidewalks,blocks,lots,buildings,...)
    svg = re.sub(r"<g id='layer-([a-z]+)'.*?</g>", lambda m: m.group(0) if m.group(1) in keep[0] else "", svg, flags=re.S)
m = re.search(r"viewBox='([\d.\-]+) ([\d.\-]+) ([\d.\-]+) ([\d.\-]+)'", svg); vx, vy, vw, vh = map(float, m.groups())
dpi = 100; fig = plt.figure(figsize=(width / dpi, width * vh / vw / dpi), dpi=dpi); ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(vx, vx + vw); ax.set_ylim(vy + vh, vy); ax.set_axis_off()
def attr(tag, name, default=None):
    mm = re.search(name + r"='([^']*)'", tag); return mm.group(1) if mm else default
n = 0
# group styles: the city map sets fill/stroke on <g id='layer-…'> and its polygons inherit them
def group_style_at(pos):
    style = {}
    for g in re.finditer(r"<g\b([^>]*)>|</g>", svg[:pos]):
        if g.group(0) == "</g>": style = {}; continue
        style = {k: v for k, v in re.findall(r"([a-z\-]+)='([^']*)'", g.group(1)) if k in ("fill", "stroke", "stroke-width", "fill-opacity", "stroke-opacity")}
    return style
def attr(t, name, default=None):
    mm = re.search(name + r"='([^']*)'", t)
    if mm: return mm.group(1)
    return _gs.get(name, default)
_gs = {}
for tag in re.finditer(r"<(rect|path|circle|text|polygon|polyline|line)\b[^>]*?(?:/>|>.*?</\1>)", svg, re.S):
    kind, t = tag.group(1), tag.group(0); _gs = group_style_at(tag.start())
    if kind == "rect":
        w, h = attr(t, "width", "100%"), attr(t, "height", "100%")
        if w.endswith("%") or float(w) >= 0.9 * vw: ax.add_patch(plt.Rectangle((vx, vy), vw, vh, color=attr(t, "fill", "#888"), zorder=0))
        else: ax.add_patch(plt.Rectangle((float(attr(t, "x", "0")), float(attr(t, "y", "0"))), float(w), float(h), color=attr(t, "fill", "#000"), lw=0, zorder=3))
        continue
    if kind == "circle":
        ax.add_patch(Circle((float(attr(t, "cx")), float(attr(t, "cy"))), float(attr(t, "r", "1")), color=attr(t, "fill", "#000"), lw=0, zorder=3)); continue
    if kind == "text":
        txt = re.sub(r"<[^>]+>", "", t).strip()
        ax.text(float(attr(t, "x")), float(attr(t, "y")), txt, fontsize=float(attr(t, "font-size", "8")) * 0.9, color=attr(t, "fill", "#000"), zorder=5, ha="center"); continue
    if kind in ("polygon", "polyline"):
        nums = list(map(float, re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", attr(t, "points", ""))))
        pts = [(nums[i], nums[i + 1]) for i in range(0, len(nums) - 1, 2)]
        if len(pts) >= 2:
            fill = attr(t, "fill", "none"); stroke = attr(t, "stroke", "none"); sw = float(attr(t, "stroke-width", "0.5"))
            if kind == "polygon": ax.add_patch(plt.Polygon(pts, closed=True, facecolor=(fill if fill != "none" else "none"), edgecolor=(stroke if stroke != "none" else "none"), lw=sw * width / vw * 0.8, alpha=float(attr(t, "fill-opacity", "1")) if fill != "none" else 1.0, zorder=2))
            else: ax.plot([p[0] for p in pts], [p[1] for p in pts], color=(stroke if stroke != "none" else "#000"), lw=sw * width / vw * 0.8, zorder=2)
            n += 1
        continue
    if kind == "line":
        stroke = attr(t, "stroke", "#000"); sw = float(attr(t, "stroke-width", "0.5"))
        ax.plot([float(attr(t, "x1", "0")), float(attr(t, "x2", "0"))], [float(attr(t, "y1", "0")), float(attr(t, "y2", "0"))], color=stroke, lw=sw * width / vw * 0.8, zorder=2); n += 1
        continue
    d = attr(t, "d", ""); verts, codes = [], []
    for cmd, xs in re.findall(r"([MLZ])([^MLZ]*)", d):
        if cmd == "Z": codes.append(Path.CLOSEPOLY); verts.append(verts[-1] if verts else (0, 0)); continue
        nums = list(map(float, re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", xs)))
        for i in range(0, len(nums) - 1, 2):
            verts.append((nums[i], nums[i + 1])); codes.append(Path.MOVETO if (cmd == "M" and i == 0) else Path.LINETO)
    if len(verts) < 2: continue
    fill = attr(t, "fill", "none"); stroke = attr(t, "stroke", "none"); sw = float(attr(t, "stroke-width", "0.5"))
    ax.add_patch(PathPatch(Path(verts, codes), facecolor=(fill if fill != "none" else "none"), edgecolor=(stroke if stroke != "none" else "none"),
                           lw=sw * width / vw * 0.8, zorder=1 if fill != "none" else 2)); n += 1
fig.savefig(dst, dpi=dpi); print(f"{dst}: {n} paths, {width}x{int(width * vh / vw)}")
