#!/usr/bin/env python3
"""Compare two furniture dumps (run_tests furniture_pieces_dump_their_numbers with RT_FURNITURE_DUMP=<file>).

    python3 tools/furniture_ab.py OLD NEW [--area 0.05] [--bounds 0.01] [--colour 0.02]

Per piece and variant: the footprint and flags must match; per finish, the bounds within --bounds metres, the surface
area within --area (a share), the area-weighted colour within --colour. Triangle counts are listed, not judged (a port
onto other words tessellates its own way). Prints each difference and a summary; exits 1 when any piece differs."""
import argparse
import sys

MATS = ["wood", "fabric", "hard", "metal", "ceramic", "light"]


def read(path):
    pieces, cur = {}, None
    for line in open(path):
        w = line.split()
        if not w:
            continue
        if w[0] == "piece":
            cur = (w[1], int(w[2]))
            pieces[cur] = {"head": w[3:], "mats": {}}
        elif w[0] == "mat":
            m = int(w[1])
            v = dict(zip(w[2::2], w[3::2]))
            pieces[cur]["mats"][m] = {
                "tris": int(v["tris"]),
                "area": float(v["area"]),
                "lo": [float(x) for x in w[w.index("lo") + 1:w.index("lo") + 4]],
                "hi": [float(x) for x in w[w.index("hi") + 1:w.index("hi") + 4]],
                "col": [float(x) for x in w[w.index("col") + 1:w.index("col") + 4]],
            }
    return pieces


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("old")
    ap.add_argument("new")
    ap.add_argument("--area", type=float, default=0.05)
    ap.add_argument("--bounds", type=float, default=0.01)
    ap.add_argument("--colour", type=float, default=0.02)
    a = ap.parse_args()
    old, new = read(a.old), read(a.new)
    bad, tris_old, tris_new = set(), 0, 0
    for key in sorted(old):
        name, var = key
        if key not in new:
            print(f"{name} v{var}: missing from NEW")
            bad.add(name)
            continue
        o, n = old[key], new[key]
        if o["head"] != n["head"]:
            print(f"{name} v{var}: footprint/flags {' '.join(o['head'])} -> {' '.join(n['head'])}")
            bad.add(name)
        for m in sorted(set(o["mats"]) | set(n["mats"])):
            om, nm = o["mats"].get(m), n["mats"].get(m)
            if om is None or nm is None:
                print(f"{name} v{var} {MATS[m]}: {'gained' if om is None else 'lost'} the finish")
                bad.add(name)
                continue
            tris_old += om["tris"]
            tris_new += nm["tris"]
            why = []
            if abs(nm["area"] - om["area"]) > a.area * max(om["area"], 1e-6):
                why.append(f"area {om['area']:.4f} -> {nm['area']:.4f} ({100 * (nm['area'] / max(om['area'], 1e-9) - 1):+.1f}%)")
            d = max(max(abs(x - y) for x, y in zip(om["lo"], nm["lo"])), max(abs(x - y) for x, y in zip(om["hi"], nm["hi"])))
            if d > a.bounds:
                why.append(f"bounds off by {100 * d:.1f} cm ({om['lo']}..{om['hi']} -> {nm['lo']}..{nm['hi']})")
            c = max(abs(x - y) for x, y in zip(om["col"], nm["col"]))
            if c > a.colour:
                why.append(f"colour {om['col']} -> {nm['col']}")
            if why:
                print(f"{name} v{var} {MATS[m]}: " + "; ".join(why))
                bad.add(name)
    print(f"\n{len(old)} piece variants, {len({k[0] for k in old})} pieces; {len(bad)} pieces differ: {', '.join(sorted(bad))}")
    print(f"triangles {tris_old} -> {tris_new} ({100 * (tris_new / max(tris_old, 1) - 1):+.1f}%)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
