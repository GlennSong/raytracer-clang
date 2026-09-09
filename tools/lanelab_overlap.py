#!/usr/bin/env python3
"""Do blocks, lots or buildings sit on the roads? Measures it from the loader's own map
(`citymap <path> roads,curbs,sidewalks,blocks,lots,buildings` over the control channel, or the
RT_LOT_PLAN_SVG dump plus a lanelab graph) with shapely: road ribbons = centrelines buffered
by half their drawn width; report every block / lot / building polygon that overlaps a ribbon
by more than `--min` m², with the worst offenders' positions.
Usage: lanelab_overlap.py --citymap out/x/citymap.svg [--min 0.5]
       lanelab_overlap.py --plan out/x/lotplan.svg --graph assets/lanelab/metro_v2/metropolis_sky_lanelab.json"""
import argparse, json, re, sys
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

def layer(svg, name):
    m = re.search(r"<g id='layer-%s'.*?</g>" % name, svg, re.S); return m.group(0) if m else ""
def polys(chunk):
    out = []
    for pts in re.findall(r"<polygon[^>]*points='([^']*)'", chunk):
        nums = list(map(float, re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", pts))); p = [(nums[i], nums[i+1]) for i in range(0, len(nums)-1, 2)]
        if len(p) >= 3:
            g = Polygon(p)
            if not g.is_valid: g = g.buffer(0)
            if not g.is_empty: out.append(g)
    return out
def lines(chunk):
    out = []
    for m in re.finditer(r"<line x1='([^']*)' y1='([^']*)' x2='([^']*)' y2='([^']*)'[^>]*stroke-width='([^']*)'", chunk):
        x1, y1, x2, y2, w = map(float, m.groups()); out.append((LineString([(x1, y1), (x2, y2)]), w))
    return out

ap = argparse.ArgumentParser(); ap.add_argument("--citymap"); ap.add_argument("--plan"); ap.add_argument("--graph"); ap.add_argument("--min", type=float, default=0.5); ap.add_argument("--streets-only", action="store_true", help="lanelab: ignore freeway and ramp ribbons (blocks under a deck are re-zoned on purpose)")
a = ap.parse_args()
sets = {}
if a.citymap:
    svg = open(a.citymap).read()
    ribbons = unary_union([l.buffer(w / 2, cap_style=2) for l, w in lines(layer(svg, "roads"))])
    sets = {"blocks": polys(layer(svg, "blocks")), "lots": polys(layer(svg, "lots")), "buildings": polys(layer(svg, "buildings"))}
    curbs = polys(layer(svg, "sidewalks"))
    curbBand = unary_union(curbs) if curbs else None
else:
    svg = open(a.plan).read(); g = json.load(open(a.graph)); cls = g["classes"]; ribs = []
    for e in g["edges"]:
        if e["path"]["type"] != "polyline": continue   # ramps/arcs are not block frontage
        if a.streets_only and e["class"] in ("freeway", "ramp"): continue
        c = cls[e["class"]]; ln = e.get("lanes", {}); w = ln.get("w", c.get("w", 3.5)); n = ln.get("fwd", c.get("fwd", 1)) + ln.get("back", c.get("back", 1))
        width = n * w + ln.get("gap", c.get("gap", 0)) + 2 * c.get("shoulder", 0)
        ribs.append(LineString([tuple(p) for p in e["path"]["points"]]).buffer(width / 2, cap_style=2))
    ribbons = unary_union(ribs)
    # the plan dump: blocks (fill #cfd8c0), lots (stroke #c0392b), buildings (fill #333333)
    def by(attr): return polys("".join(m.group(0) for m in re.finditer(r"<polygon[^>]*%s[^>]*/>" % attr, svg)))
    sets = {"blocks": by("fill='#cfd8c0'"), "lots": by("stroke='#c0392b'"), "buildings": by("fill='#333333'")}
    curbBand = None
print(f"road ribbon area {ribbons.area:,.0f} m²")
for name, gs in sets.items():
    hits = []
    for g in gs:
        ov = g.intersection(ribbons).area
        if ov > a.min: c = g.centroid; hits.append((ov, c.x, c.y, g.area))
    hits.sort(reverse=True)
    tot = sum(h[0] for h in hits)
    print(f"{name:9s}: {len(gs):5d} polygons, {len(hits):4d} overlap a carriageway by > {a.min} m² (total {tot:,.0f} m²)")
    for ov, x, y, ar in hits[:5]: print(f"           {ov:7.1f} m² of {ar:7.0f} m² at ({x:.0f}, {y:.0f})")
    if curbBand is not None and name == "buildings":
        cb = [(g.intersection(curbBand).area, g.centroid) for g in gs]; cb = [(o, c) for o, c in cb if o > a.min]
        print(f"           buildings on the SIDEWALK band by > {a.min} m²: {len(cb)}")
