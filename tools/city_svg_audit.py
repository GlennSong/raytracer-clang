#!/usr/bin/env python3
"""AUDIT A CITY FROM ITS OWN SVG MAP (Glenn, 2026-09-18: "for the lot you should
use the svg map and look for broken lots that way... Sometimes looking at the raw
data is more useful than looking at it in engine").

The engine-side `lot?` probe reads BuildingRecords, and a great many lots are not
buildings -- greens, parks, plazas, the pads that trees get scattered onto. So a
green lot lying across a street is invisible to it, which is exactly the bug that
kept reading "clean" while Glenn was looking straight at one.

The SVG the running city writes (`citymap out.svg all`) carries the layers in
WORLD COORDINATES with no transform: layer-roads is <line> segments carrying
their own stroke-width (the carriageway), layer-lots is every lot polygon,
layer-curbs the asphalt boundary. So the raw map answers the question the
runtime cannot.

    tools/city_svg_audit.py city.svg [--margin 0.0] [--top 20]

Reports every lot polygon that overlaps a carriageway, worst first, with a
teleport-ready position.
"""
import argparse
import math
import re
import sys
from collections import defaultdict


def layer(svg, name):
    m = re.search(r"<g id='layer-%s'[^>]*>(.*?)</g>" % name, svg, re.S)
    return m.group(1) if m else ""


def roads_of(svg):
    """(x1, y1, x2, y2, half_width) for every carriageway segment."""
    out = []
    for m in re.finditer(
        r"<line x1='(-?[\d.]+)' y1='(-?[\d.]+)' x2='(-?[\d.]+)' y2='(-?[\d.]+)'"
        r"[^>]*?stroke-width='([\d.]+)'", layer(svg, "roads")):
        x1, y1, x2, y2, w = (float(g) for g in m.groups())
        out.append((x1, y1, x2, y2, w * 0.5))
    return out


def polys_of(svg, name):
    out = []
    for m in re.finditer(r"<polygon points='([^']+)'", layer(svg, name)):
        pts = []
        for pair in m.group(1).split():
            if "," not in pair:
                continue
            a, b = pair.split(",", 1)
            try:
                pts.append((float(a), float(b)))
            except ValueError:
                pass
        if len(pts) >= 3:
            out.append(pts)
    return out


def seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-12 else ((px - x1) * dx + (py - y1) * dy) / l2
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    qx, qy = x1 + dx * t, y1 + dy * t
    return math.hypot(px - qx, py - qy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("svg")
    ap.add_argument("--margin", type=float, default=0.0,
                    help="metres INSIDE the carriageway edge before it counts "
                         "(0 = the asphalt itself)")
    ap.add_argument("--step", type=float, default=2.0,
                    help="edge sampling step, metres")
    ap.add_argument("--top", type=int, default=20)
    ap.add_argument("--annotate", metavar="OUT.svg",
                    help="write a copy of the map with the offending lots "
                         "ringed and labelled, so the defect can be SEEN "
                         "rather than read off a list")
    ap.add_argument("--zoom", metavar="X,Z,HALF",
                    help="crop the annotated map to a square around a world "
                         "point, e.g. -790,437,120")
    a = ap.parse_args()

    svg = open(a.svg).read()
    roads = roads_of(svg)
    lots = polys_of(svg, "lots")
    if not roads or not lots:
        print("no roads (%d) or lots (%d) in that map -- was it written with "
              "`citymap out.svg roads,lots,...`?" % (len(roads), len(lots)))
        return 1
    print("%d carriageway segments, %d lot polygons" % (len(roads), len(lots)))

    # Grid the roads so each lot only tests its own neighbourhood.
    CELL = 40.0
    grid = defaultdict(list)
    for i, (x1, y1, x2, y2, hw) in enumerate(roads):
        lo_x, hi_x = min(x1, x2) - hw, max(x1, x2) + hw
        lo_y, hi_y = min(y1, y2) - hw, max(y1, y2) + hw
        for cx in range(int(lo_x // CELL), int(hi_x // CELL) + 1):
            for cy in range(int(lo_y // CELL), int(hi_y // CELL) + 1):
                grid[(cx, cy)].append(i)

    def worst_at(px, py):
        """How far INSIDE a carriageway this point is; <= 0 is clear."""
        worst = -1e30
        cx, cy = int(px // CELL), int(py // CELL)
        for gx in (cx - 1, cx, cx + 1):
            for gy in (cy - 1, cy, cy + 1):
                for i in grid.get((gx, gy), ()):
                    x1, y1, x2, y2, hw = roads[i]
                    into = hw - seg_dist(px, py, x1, y1, x2, y2)
                    if into > worst:
                        worst = into
        return worst

    bad = []
    for pts in lots:
        worst, at = -1e30, None
        n = len(pts)
        for i in range(n):
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % n]
            steps = max(1, int(math.hypot(bx - ax, by - ay) / a.step))
            for s in range(steps + 1):
                t = s / steps
                px, py = ax + (bx - ax) * t, ay + (by - ay) * t
                w = worst_at(px, py)
                if w > worst:
                    worst, at = w, (px, py)
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        wc = worst_at(cx, cy)
        if wc > worst:
            worst, at = wc, (cx, cy)
        if worst > a.margin:
            bad.append((worst, at, (cx, cy), n))

    bad.sort(key=lambda r: -r[0])
    if a.annotate:
        # Draw on top of the map itself: a red ring at the worst point of each
        # offending lot, the lot outlined, and a label. A picture of WHERE is
        # worth more than a table of coordinates when the next question is
        # "is that the one I was looking at".
        marks = ["<g id='layer-AUDIT' fill='none' stroke='#d81b1b' "
                 "stroke-width='1.6'>"]
        for i, (worst, at, c, n) in enumerate(bad):
            marks.append(
                "<circle cx='%.2f' cy='%.2f' r='9' stroke='#d81b1b' "
                "stroke-width='1.8' fill='#d81b1b' fill-opacity='0.18'/>"
                % (at[0], at[1]))
            marks.append(
                "<line x1='%.2f' y1='%.2f' x2='%.2f' y2='%.2f' stroke='#d81b1b' "
                "stroke-width='1.0' stroke-dasharray='3 2'/>"
                % (at[0], at[1], c[0], c[1]))
            marks.append(
                "<text x='%.2f' y='%.2f' font-size='11' fill='#a01010' "
                "stroke='none'>%d: %.2f m into the lane  (%.0f, %.0f)</text>"
                % (at[0] + 12, at[1] - 10, i + 1, worst, c[0], c[1]))
        marks.append("</g>")
        out = svg.replace("</svg>", "\n".join(marks) + "\n</svg>")
        if a.zoom:
            try:
                zx, zz, half = (float(v) for v in a.zoom.split(","))
                out = re.sub(r"viewBox='[^']*'",
                             "viewBox='%.1f %.1f %.1f %.1f'"
                             % (zx - half, zz - half, half * 2, half * 2), out, count=1)
                out = re.sub(r"width='[\d.]+' height='[\d.]+'",
                             "width='900' height='900'", out, count=1)
            except ValueError:
                print("--zoom wants X,Z,HALF")
        open(a.annotate, "w").write(out)
        print("annotated map -> %s (%d marked)" % (a.annotate, len(bad)))

    print("%d of %d lots reach into a carriageway (margin %.2f m)"
          % (len(bad), len(lots), a.margin))
    for worst, at, c, n in bad[:a.top]:
        print("  %6.2f m into the lane at (%8.1f, %8.1f)   lot centre "
              "(%8.1f, %8.1f) %2d verts   ->  teleport %.2f %.2f"
              % (worst, at[0], at[1], c[0], c[1], n, c[0], c[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
