#!/usr/bin/env python3
"""DRAW THE BUS NETWORK OVER THE CITY MAP (Glenn, 2026-09-18: "can you map out
the bus routes on the map using the SVG tools?").

The routes are DERIVED at runtime from the nav graph, so `citymap` -- which
draws what procgen produced -- has never heard of them. The viewer dumps the
network with RT_BUS_MAP=<path.json>; this lays it over any citymap SVG, in the
same colours the stop signs wear in game, so "where does route 2 actually go"
is a picture rather than a list of coordinates.

    RT_BUS_MAP=/tmp/bus.json build-viewer/viewer <level> --play    # once
    (in the viewer)  citymap /tmp/city.svg roads,curbs,lots,blocks,buildings
    tools/city_bus_map.py /tmp/city.svg /tmp/bus.json /tmp/transit.svg

It also prints the per-route geometry that decides whether a bus is worth
taking: loop length, stop spacing, and the headway a given fleet gives.
"""
import argparse
import json
import math
import re
import sys

# The sign colours from bus_stop_props.cpp, so the map matches the street.
PALETTE = ["#d92e28", "#216bcc", "#f2ad1a", "#29994f",
           "#993db8", "#0d9ea8", "#eb6b1f", "#73737a"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("city_svg")
    ap.add_argument("bus_json")
    ap.add_argument("out_svg")
    ap.add_argument("--bus-speed", type=float, default=9.0,
                    help="m/s, for the headway estimate")
    ap.add_argument("--buses", type=int, default=0,
                    help="buses on the network (split evenly across routes); "
                         "0 skips the headway estimate")
    ap.add_argument("--zoom", metavar="X,Z,HALF")
    a = ap.parse_args()

    svg = open(a.city_svg).read()
    net = json.load(open(a.bus_json))
    routes = net.get("routes", [])
    if not routes:
        print("no routes in %s" % a.bus_json)
        return 1

    g = ["<g id='layer-transit' fill='none' stroke-linecap='round' "
         "stroke-linejoin='round'>"]
    per_route = max(1, a.buses // max(1, len(routes))) if a.buses else 0
    for r in routes:
        idx = r.get("route", 0)
        stops = [(float(x), float(z)) for x, z in r.get("stops", [])]
        if len(stops) < 2:
            continue
        col = PALETTE[idx % len(PALETTE)]
        # The line as DRIVEN. `path` is every nav node the bus passes, so the
        # route follows streets; falling back to the stop chords only if an
        # older dump has no path in it.
        drive = [(float(x), float(z)) for x, z in r.get("path", [])] or stops
        pts = " ".join("%.1f,%.1f" % p for p in drive)
        g.append("<polyline points='%s %.1f,%.1f' stroke='%s' stroke-width='3.6' "
                 "stroke-opacity='0.6'/>" % (pts, drive[0][0], drive[0][1], col))
        hubset = set(r.get("hubStops", []))
        for i, (x, z) in enumerate(stops):
            if i in hubset:   # an interchange: bigger, ringed dark
                g.append("<circle cx='%.1f' cy='%.1f' r='8' fill='%s' "
                         "stroke='#1b1b1b' stroke-width='2.2'><title>route %d "
                         "HUB stop %d</title></circle>" % (x, z, col, idx, i))
            else:
                g.append("<circle cx='%.1f' cy='%.1f' r='4' fill='%s' "
                         "stroke='#ffffff' stroke-width='1.1'><title>route %d "
                         "stop %d</title></circle>" % (x, z, col, idx, i))
        dep = r.get("depot")
        if dep:
            g.append("<rect x='%.1f' y='%.1f' width='13' height='13' fill='none' "
                     "stroke='%s' stroke-width='2.4'><title>route %d depot"
                     "</title></rect>" % (dep[0] - 6.5, dep[1] - 6.5, col, idx))

        # The geometry that decides whether anyone rides: loop length, spacing,
        # and what headway the fleet buys.
        loop = sum(math.dist(drive[i], drive[i + 1]) for i in range(len(drive) - 1))
        loop += math.dist(drive[-1], drive[0])
        spacing = loop / len(stops)
        line = ("route %d: %2d stops, loop %6.0f m, mean spacing %5.0f m"
                % (idx, len(stops), loop, spacing))
        if per_route:
            headway = (loop / a.bus_speed) / per_route
            line += (", %d buses -> headway %4.0f s (mean wait %3.0f s)"
                     % (per_route, headway, headway / 2))
        print(line)
    for hx, hz in net.get("hubs", []):
        g.append("<circle cx='%.1f' cy='%.1f' r='13' fill='none' stroke='#1b1b1b' "
                 "stroke-width='2.0' stroke-dasharray='3 2'><title>transit hub"
                 "</title></circle>" % (float(hx), float(hz)))
    g.append("</g>")

    # THE LEGEND (Glenn: "I'd like to see a legend that includes the colors of
    # each routes and the exact stops"). Drawn in a panel BESIDE the city rather
    # than over it -- 56 stop coordinates on top of the map would bury the thing
    # they describe. The viewBox is widened to make room.
    vb = re.search(r"viewBox='(-?[\d.]+) (-?[\d.]+) ([\d.]+) ([\d.]+)'", svg)
    legend = []
    if vb:
        vx, vy, vw, vh = (float(v) for v in vb.groups())
        PW = 430.0                      # panel width, world units
        px = vx + vw + 20.0
        legend.append("<g id='layer-legend-transit' font-family='Helvetica, Arial'>")
        legend.append("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' "
                      "fill='#ffffff' fill-opacity='0.94' stroke='#b9b2a6' "
                      "stroke-width='1.5'/>" % (px, vy, PW, vh))
        y = vy + 34.0
        legend.append("<text x='%.1f' y='%.1f' font-size='20' font-weight='bold' "
                      "fill='#2b2b2b'>Bus network</text>" % (px + 16, y))
        y += 26
        if per_route:
            legend.append("<text x='%.1f' y='%.1f' font-size='12' fill='#6b6b6b'>"
                          "%d buses over %d routes — %d per route</text>"
                          % (px + 16, y, a.buses, len(routes), per_route))
            y += 24
        for r in routes:
            idx = r.get("route", 0)
            stops = [(float(x), float(z)) for x, z in r.get("stops", [])]
            if len(stops) < 2:
                continue
            col = PALETTE[idx % len(PALETTE)]
            drive = [(float(x), float(z)) for x, z in r.get("path", [])] or stops
            loop = sum(math.dist(drive[i], drive[i + 1]) for i in range(len(drive) - 1))
            loop += math.dist(drive[-1], drive[0])
            legend.append("<rect x='%.1f' y='%.1f' width='22' height='11' "
                          "fill='%s'/>" % (px + 16, y - 9, col))
            nhub = len(r.get("hubStops", []))
            head = ("route %d — %d stops (%d hubs), loop %.0f m"
                    % (idx, len(stops), nhub, loop))
            if per_route:
                head += ", headway %.0f s" % ((loop / a.bus_speed) / per_route)
            legend.append("<text x='%.1f' y='%.1f' font-size='13' "
                          "font-weight='bold' fill='#2b2b2b'>%s</text>"
                          % (px + 46, y, head))
            y += 17
            dep = r.get("depot")
            if dep:
                legend.append("<text x='%.1f' y='%.1f' font-size='11' "
                              "fill='#8a6d1f'>depot (%.0f, %.0f)</text>"
                              % (px + 46, y, dep[0], dep[1]))
                y += 15
            # The exact stops, two to a line so 14 of them do not run off.
            for i in range(0, len(stops), 2):
                chunk = "  ".join("%2d (%.0f, %.0f)" % (j, stops[j][0], stops[j][1])
                                  for j in range(i, min(i + 2, len(stops))))
                legend.append("<text x='%.1f' y='%.1f' font-size='10.5' "
                              "fill='#4a4a4a' font-family='monospace'>%s</text>"
                              % (px + 46, y, chunk))
                y += 13
            y += 12
        legend.append("</g>")

    out = svg.replace("</svg>", "\n".join(g + legend) + "\n</svg>")
    if vb and legend:
        vx, vy, vw, vh = (float(v) for v in vb.groups())
        out = re.sub(r"viewBox='[^']*'",
                     "viewBox='%.1f %.1f %.1f %.1f'" % (vx, vy, vw + 470.0, vh),
                     out, count=1)
        out = re.sub(r"width='[\d.]+' height='[\d.]+'",
                     "width='%.1f' height='%.1f'" % (vw + 470.0, vh), out, count=1)
    if a.zoom:
        zx, zz, half = (float(v) for v in a.zoom.split(","))
        out = re.sub(r"viewBox='[^']*'",
                     "viewBox='%.1f %.1f %.1f %.1f'"
                     % (zx - half, zz - half, half * 2, half * 2), out, count=1)
        out = re.sub(r"width='[\d.]+' height='[\d.]+'",
                     "width='900' height='900'", out, count=1)
    open(a.out_svg, "w").write(out)
    print("transit map -> %s (%d routes)" % (a.out_svg, len(routes)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
