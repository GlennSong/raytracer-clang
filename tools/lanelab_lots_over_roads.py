#!/usr/bin/env python3
"""Lot outlines over the lane lab's own pavement plan. Inputs: the plan SVG from `lanelab_tool
build` (paved footprints, world -> pixels as X=(x-minX)*s, Y=(maxY-y)*s over the graph's terrain
bounds), the graph JSON (for those bounds) and the RT_LOT_PLAN_SVG dump (blocks, lots, built plans,
world x/z). Output: one PNG of the whole plan and optional zooms.
Usage: lanelab_lots_over_roads.py plan.svg graph.json lotplan.svg out.png [--zoom name=x,z,radius ...]"""
import json, re, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch

plan, graph, lots, out = sys.argv[1:5]
zooms = [a.split("=", 1) for a in sys.argv[5:] if a.startswith("--zoom=")] if False else []
args = sys.argv[5:]
for i, a in enumerate(args):
    if a == "--zoom": name, spec = args[i + 1].split("="); x, z, r = map(float, spec.split(",")); zooms.append((name, x, z, r))
b = json.load(open(graph))["terrain"]["bounds"]; W, H = b[1] - b[0], b[3] - b[2]; s = 1600.0 / max(W, H)
X = lambda x: (x - b[0]) * s; Y = lambda z: (b[3] - z) * s
svg = open(plan).read(); m = re.search(r"viewBox='([\d.\-]+) ([\d.\-]+) ([\d.\-]+) ([\d.\-]+)'", svg); vx, vy, vw, vh = map(float, m.groups())
def attr(t, n, d=None):
    mm = re.search(n + r"='([^']*)'", t); return mm.group(1) if mm else d
def polys(chunk, attrsel):
    out = []
    for t in re.finditer(r"<polygon[^>]*%s[^>]*/>" % attrsel, chunk):
        nums = list(map(float, re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", attr(t.group(0), "points", ""))))
        pts = [(X(nums[i]), Y(nums[i + 1])) for i in range(0, len(nums) - 1, 2)]
        if len(pts) >= 3: out.append(pts)
    return out
lsvg = open(lots).read()
blocks, lotp, built = polys(lsvg, "fill='#cfd8c0'"), polys(lsvg, "stroke='#c0392b'"), polys(lsvg, "fill='#333333'")
def draw(ax, lw_scale):
    ax.add_patch(plt.Rectangle((vx, vy), vw, vh, color="#8fa36b", zorder=0))
    for t in re.finditer(r"<path[^>]*/>", svg):   # the pavement plan: paths M/L/Z
        tag = t.group(0); d = attr(tag, "d", ""); verts, codes = [], []
        for cmd, xs in re.findall(r"([MLZ])([^MLZ]*)", d):
            if cmd == "Z": codes.append(Path.CLOSEPOLY); verts.append(verts[-1] if verts else (0, 0)); continue
            nums = list(map(float, re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", xs)))
            for i in range(0, len(nums) - 1, 2): verts.append((nums[i], nums[i + 1])); codes.append(Path.MOVETO if (cmd == "M" and i == 0) else Path.LINETO)
        if len(verts) < 2: continue
        fill = attr(tag, "fill", "none"); stroke = attr(tag, "stroke", "none")
        ax.add_patch(PathPatch(Path(verts, codes), facecolor=fill if fill != "none" else "none", edgecolor=stroke if stroke != "none" else "none", lw=0.3 * lw_scale, zorder=1 if fill != "none" else 2))
    for p in blocks: ax.add_patch(plt.Polygon(p, closed=True, facecolor="#2255aa", alpha=0.18, edgecolor="#2255aa", lw=1.0 * lw_scale, zorder=3))
    for p in lotp: ax.add_patch(plt.Polygon(p, closed=True, facecolor="none", edgecolor="#e01b24", lw=0.7 * lw_scale, zorder=4))
    for p in built: ax.add_patch(plt.Polygon(p, closed=True, facecolor="#222222", alpha=0.85, edgecolor="none", zorder=5))
dpi = 100; width = 2200
fig = plt.figure(figsize=(width / dpi, width * vh / vw / dpi), dpi=dpi); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(vx, vx + vw); ax.set_ylim(vy + vh, vy); ax.set_axis_off()
draw(ax, 1.0); fig.savefig(out, dpi=dpi); plt.close(fig); print(out)
for name, x, z, r in zooms:
    fig = plt.figure(figsize=(14, 14), dpi=dpi); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(X(x - r), X(x + r)); ax.set_ylim(Y(z - r), Y(z + r)); ax.set_axis_off()
    draw(ax, 4.0); ax.plot([X(x)], [Y(z)], marker="+", color="#ff00ff", ms=30, mew=3)
    o = out.replace(".png", f"_{name}.png"); fig.savefig(o, dpi=dpi); plt.close(fig); print(o)
