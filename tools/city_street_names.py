#!/usr/bin/env python3
"""NAME THE STREETS (Glenn, 2026-09-18: "we should name all of the streets. A
street should have a start and end point and we should be able to name all the
streets on the svg map at least. I think that also helps with bus routes and
agents addresses").

A road graph has EDGES, not streets. What a person calls a street is a CHAIN of
edges that runs straight on through junctions -- you stay on Elm when Elm
crosses Oak. So this walks the citymap's own road layer, chains the segments
that continue into one another, and names each chain.

Chaining rule: from an endpoint, keep taking the continuation that turns least,
stop when the best turn exceeds a threshold or the width changes class. That is
what makes "Elm crosses Oak" two streets rather than four stubs.

    (in the viewer)  citymap /tmp/city.svg roads,curbs,lots,blocks,buildings
    tools/city_street_names.py /tmp/city.svg /tmp/streets.svg [--json /tmp/streets.json]

The JSON is the useful half: every street with its name, class, endpoints and
polyline, ready for bus-route descriptions and agent addresses.
"""
import argparse
import json
import math
import re
import sys
from collections import defaultdict

# A deterministic name bank. Real cities mix trees, presidents, numbers and
# landmarks; the mix matters more than the individual words for making a map
# feel navigable.
TREES = ["Oak", "Maple", "Elm", "Cedar", "Willow", "Birch", "Aspen", "Alder",
         "Chestnut", "Hawthorn", "Laurel", "Linden", "Magnolia", "Juniper",
         "Poplar", "Sycamore", "Hazel", "Rowan", "Spruce", "Walnut"]
NAMED = ["Lincoln", "Jefferson", "Franklin", "Madison", "Monroe", "Jackson",
         "Harrison", "Sherman", "Grant", "Hamilton", "Adams", "Clay",
         "Kearny", "Bryant", "Folsom", "Mission", "Larkin", "Geary"]
PLACES = ["Harbour", "Market", "Mill", "Quarry", "Foundry", "Cannery",
          "Depot", "Union", "Commerce", "Exchange", "Granary", "Wharf"]
ORDINAL = ["First", "Second", "Third", "Fourth", "Fifth", "Sixth", "Seventh",
           "Eighth", "Ninth", "Tenth", "Eleventh", "Twelfth"]

# Suffix by road class: an arterial is a Boulevard or Avenue, a local street a
# Street or Lane. Width is the only class signal the SVG carries.
def suffix_for(width, rng):
    if width >= 16:
        return rng.choice(["Boulevard", "Avenue", "Parkway"])
    if width >= 13:
        return rng.choice(["Avenue", "Road", "Way"])
    if width >= 10:
        return rng.choice(["Street", "Road", "Drive"])
    return rng.choice(["Lane", "Court", "Alley", "Walk"])


class Rng:
    """Tiny deterministic PRNG so a city always gets the same street names."""
    def __init__(self, seed):
        self.s = seed & 0xFFFFFFFF or 1

    def next(self):
        self.s ^= (self.s << 13) & 0xFFFFFFFF
        self.s ^= self.s >> 17
        self.s ^= (self.s << 5) & 0xFFFFFFFF
        return self.s

    def choice(self, seq):
        return seq[self.next() % len(seq)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("city_svg")
    ap.add_argument("out_svg")
    ap.add_argument("--json")
    ap.add_argument("--seed", type=int, default=20260918)
    ap.add_argument("--turn", type=float, default=14.0,
                    help="degrees: a continuation turning more than this ends "
                         "the street")
    ap.add_argument("--min-len", type=float, default=70.0,
                    help="streets shorter than this are not labelled")
    ap.add_argument("--zoom", metavar="X,Z,HALF")
    a = ap.parse_args()

    svg = open(a.city_svg).read()
    m = re.search(r"<g id='layer-roads'[^>]*>(.*?)</g>", svg, re.S)
    if not m:
        print("no layer-roads in that map")
        return 1

    # Segments, deduplicated by endpoint pair so the two directions of one road
    # do not become two streets.
    segs = []
    seen = set()
    for mm in re.finditer(
            r"<line x1='(-?[\d.]+)' y1='(-?[\d.]+)' x2='(-?[\d.]+)' y2='(-?[\d.]+)'"
            r"[^>]*?stroke-width='([\d.]+)'", m.group(1)):
        x1, y1, x2, y2, w = (float(g) for g in mm.groups())
        key = (round(min(x1, x2), 1), round(min(y1, y2), 1),
               round(max(x1, x2), 1), round(max(y1, y2), 1), round(w, 1))
        if key in seen:
            continue
        seen.add(key)
        segs.append((x1, y1, x2, y2, w))
    if not segs:
        print("no road segments")
        return 1

    # Node ids by rounded position, and the segments at each.
    def nid(x, y):
        return (round(x, 1), round(y, 1))

    at = defaultdict(list)
    for i, (x1, y1, x2, y2, w) in enumerate(segs):
        at[nid(x1, y1)].append(i)
        at[nid(x2, y2)].append(i)

    def heading(i, frm):
        x1, y1, x2, y2, _ = segs[i]
        if nid(x1, y1) == frm:
            return math.degrees(math.atan2(y2 - y1, x2 - x1)), nid(x2, y2)
        return math.degrees(math.atan2(y1 - y2, x1 - x2)), nid(x1, y1)

    used = [False] * len(segs)
    streets = []
    for start in range(len(segs)):
        if used[start]:
            continue
        # Grow both ways from this segment, always taking the straightest
        # continuation of the same rough width -- which is what keeps you on
        # Elm when Elm crosses Oak.
        chain = [start]
        used[start] = True
        for direction in (0, 1):
            cur = start
            node = nid(segs[cur][0], segs[cur][1]) if direction else nid(segs[cur][2], segs[cur][3])
            hdg, node = heading(cur, nid(segs[cur][2], segs[cur][3]) if direction
                                else nid(segs[cur][0], segs[cur][1]))
            while True:
                best, bestTurn = -1, 1e9
                for j in at[node]:
                    if used[j]:
                        continue
                    jh, _ = heading(j, node)
                    turn = abs((jh - hdg + 180) % 360 - 180)
                    if abs(segs[j][4] - segs[cur][4]) > 3.0:
                        continue
                    if turn < bestTurn:
                        best, bestTurn = j, turn
                if best < 0 or bestTurn > a.turn:
                    break
                used[best] = True
                hdg, node = heading(best, node)
                cur = best
                chain.append(best) if direction == 0 else chain.insert(0, best)
        streets.append(chain)

    rng = Rng(a.seed)
    banks = [TREES, NAMED, PLACES, ORDINAL]
    out_json, labels = [], []
    named = 0
    for chain in streets:
        pts = []
        for i in chain:
            x1, y1, x2, y2, _ = segs[i]
            if not pts:
                pts.append((x1, y1))
            pts.append((x2, y2))
        length = sum(math.dist(pts[k], pts[k + 1]) for k in range(len(pts) - 1))
        width = sum(segs[i][4] for i in chain) / len(chain)
        if length < a.min_len:
            continue
        bank = banks[rng.next() % len(banks)]
        name = "%s %s" % (rng.choice(bank), suffix_for(width, rng))
        named += 1
        out_json.append({
            "name": name, "width": round(width, 1), "length": round(length, 1),
            "start": [round(pts[0][0], 1), round(pts[0][1], 1)],
            "end": [round(pts[-1][0], 1), round(pts[-1][1], 1)],
            "points": [[round(p[0], 1), round(p[1], 1)] for p in pts],
        })
        # Label at the middle of the chain, rotated along it.
        mid = pts[len(pts) // 2]
        nxt = pts[min(len(pts) // 2 + 1, len(pts) - 1)]
        ang = math.degrees(math.atan2(nxt[1] - mid[1], nxt[0] - mid[0]))
        if ang > 90 or ang < -90:
            ang += 180
        size = 9.0 if width < 13 else 11.5
        labels.append(
            "<text x='%.1f' y='%.1f' font-size='%.1f' fill='#2f2a24' "
            "stroke='#f4f1ea' stroke-width='2.2' paint-order='stroke' "
            "text-anchor='middle' transform='rotate(%.1f %.1f %.1f)'>%s</text>"
            % (mid[0], mid[1] - 2.0, size, ang, mid[0], mid[1] - 2.0, name))

    g = ["<g id='layer-street-names' font-family='Helvetica, Arial'>"] + labels + ["</g>"]
    out = svg.replace("</svg>", "\n".join(g) + "\n</svg>")
    if a.zoom:
        zx, zz, half = (float(v) for v in a.zoom.split(","))
        out = re.sub(r"viewBox='[^']*'",
                     "viewBox='%.1f %.1f %.1f %.1f'"
                     % (zx - half, zz - half, half * 2, half * 2), out, count=1)
        out = re.sub(r"width='[\d.]+' height='[\d.]+'",
                     "width='1100' height='1100'", out, count=1)
    open(a.out_svg, "w").write(out)
    if a.json:
        json.dump({"streets": out_json}, open(a.json, "w"), indent=1)
    print("%d segments -> %d chains, %d named streets (>= %.0f m)"
          % (len(segs), len(streets), named, a.min_len))
    for s in sorted(out_json, key=lambda r: -r["length"])[:10]:
        print("  %-22s %6.0f m  width %4.1f  (%7.1f,%7.1f) -> (%7.1f,%7.1f)"
              % (s["name"], s["length"], s["width"],
                 s["start"][0], s["start"][1], s["end"][0], s["end"][1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
