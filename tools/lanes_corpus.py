#!/usr/bin/env python3
"""The lane builder's robustness gate (docs/lanes-robustness.md).

Runs every scene in tools/lanes_corpus.json through lanes_tool — importing first
where the scene comes from a level recipe — parses what the tool already prints,
and reports one table. With --check it compares against the recorded baseline and
exits non-zero if anything got worse, so a change to the importer either improves
the number or does not.

    tools/lanes_corpus.py                 # run the fast scenes, print the table
    tools/lanes_corpus.py --all           # include the slow ones (metro)
    tools/lanes_corpus.py --check         # compare with the baseline, exit 1 on a regression
    tools/lanes_corpus.py --update        # record the current run as the baseline

Nothing here knows how to build a road: it reads lanes_tool's own output, which is
the same text a person reads when they run one scene by hand.
"""
import argparse, json, os, re, subprocess, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORPUS = os.path.join(ROOT, "tools", "lanes_corpus.json")
BASELINE = os.path.join(ROOT, "tools", "lanes_corpus_baseline.json")

# --- what lanes_tool prints, and what it means ------------------------------
RE_DECK = re.compile(r"deck (\d+): ([\d.]+) m², (\d+) verts, (\d+) tris, boundary (\d+), "
                     r"non-manifold (\d+), cracks (\d+), odd-boundary (\d+)")
RE_MISMATCH = re.compile(r"nodes/crossings: mismatch ([\d.]+) cm before, ([\d.]+) cm after; "
                         r"adjacent lane pairs (\d+)")
RE_TERRAIN = re.compile(r"terrain: cut ([\d.]+) m³, fill ([\d.]+) m³; above deck (\d+) of (\d+) "
                        r"\(max (-?[\d.]+) m\)")
RE_LAYERS = re.compile(r"layers: surface ([\d.]+) m², shoulder ([\d.]+), sidewalk ([\d.]+), "
                       r"median ([\d.]+); islands (\d+), enclosed blocks (\d+)")
RE_VERDICT = re.compile(r"^(PASS|FAIL)\s+([^:]+):?\s*(.*)$")
# the importer's own summary tail: "... N streets (M trimmed short of the freeway, K dropped), L landings, R ramps"
RE_IMPORT = re.compile(r"(\d+) streets \((\d+) trimmed short of the freeway, (\d+) dropped\), "
                       r"(\d+) landings, (\d+) ramps")
RE_RING = re.compile(r"ring redrawn at R >= ([\d.]+) m: ([\d.]+) m, min radius ([\d.]+) m")
RE_NO_RING = re.compile(r"no perimeter ring found")
RE_WROTE = re.compile(r"^wrote (\S+) \((\d+) edges\)")


def run(cmd, timeout):
    t0 = time.time()
    try:
        p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
        return p.stdout + p.stderr, p.returncode, time.time() - t0
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or "") + (e.stderr or "")
        return (out if isinstance(out, str) else out.decode("utf8", "replace")), 124, time.time() - t0


def parse_build(text):
    """Everything the gate watches, out of one build's output."""
    decks = [dict(zip(("i", "area", "verts", "tris", "boundary", "nonmanifold", "cracks", "odd"),
                      (int(m[0]), float(m[1]), int(m[2]), int(m[3]), int(m[4]), int(m[5]), int(m[6]), int(m[7]))))
             for m in RE_DECK.findall(text)]
    r = {
        "decks": len(decks),
        "tris": sum(d["tris"] for d in decks),
        "nonmanifold": sum(d["nonmanifold"] for d in decks),
        "cracks": sum(d["cracks"] for d in decks),
        "odd_boundary": sum(d["odd"] for d in decks),
    }
    m = RE_MISMATCH.search(text)
    if m:
        r["mismatch_before_cm"] = float(m.group(1))
        r["mismatch_after_cm"] = float(m.group(2))
        r["lane_pairs"] = int(m.group(3))
    m = RE_TERRAIN.search(text)
    if m:
        r["above_deck"] = int(m.group(3))
        r["above_deck_of"] = int(m.group(4))
        r["above_deck_max_m"] = float(m.group(5))
    m = RE_LAYERS.search(text)
    if m:
        r["islands"] = int(m.group(5))
        r["enclosed_blocks"] = int(m.group(6))
    verdicts = {}
    for line in text.splitlines():
        v = RE_VERDICT.match(line.strip())
        if v:
            name = v.group(2).strip()
            # the invariant list is printed once; a later line never downgrades an earlier PASS
            verdicts.setdefault(name, {"verdict": v.group(1), "detail": v.group(3).strip()[:200]})
    r["invariants"] = verdicts
    r["failed"] = sorted(k for k, v in verdicts.items() if v["verdict"] == "FAIL")
    return r


def parse_import(text):
    r = {}
    m = RE_IMPORT.search(text)
    if m:
        r.update(streets=int(m.group(1)), trimmed=int(m.group(2)), dropped=int(m.group(3)),
                 landings=int(m.group(4)), ramps=int(m.group(5)))
    m = RE_RING.search(text)
    if m:
        r.update(ring_target_r=float(m.group(1)), ring_len=float(m.group(2)), ring_min_r=float(m.group(3)))
    elif RE_NO_RING.search(text):
        r["ring"] = None
    m = RE_WROTE.search(text)
    if not m:
        for line in text.splitlines():
            m = RE_WROTE.match(line.strip())
            if m:
                break
    if m:
        r["graph"] = m.group(1)
        r["edges"] = int(m.group(2))
    return r


def scene_run(sc, tool, outroot, timeout):
    name, out = sc["name"], os.path.join(outroot, sc["name"])
    os.makedirs(out, exist_ok=True)
    res = {"name": name, "kind": sc["kind"], "tests": sc.get("tests", "")}
    graph = sc["source"]
    if sc["kind"] == "imported":
        text, rc, secs = run([tool, "from-level", sc["source"], "--out", out], timeout)
        imp = parse_import(text)
        res["import"] = imp
        res["import_seconds"] = round(secs, 1)
        if rc != 0 or "graph" not in imp:
            res["error"] = f"from-level failed (rc={rc})"
            return res
        graph = imp["graph"]
    cmd = [tool, "build", graph, "--out", os.path.join(out, "build")]
    if sc.get("quick"):
        cmd.append("--quick")          # no invariant sweep: the verdicts are UNKNOWN, not clean
    text, rc, secs = run(cmd, timeout)
    res["seconds"] = round(secs, 1)
    if rc == 124:
        res["error"] = f"build timed out after {timeout}s"
        return res
    res.update(parse_build(text))
    res["checked"] = not sc.get("quick")
    if rc != 0 and not res.get("invariants"):
        res["error"] = f"build failed (rc={rc})"
    return res


def table(rows):
    def g(r, k, d="—"):
        v = r.get(k)
        return d if v is None else v
    hdr = f"{'scene':<22}{'kind':<10}{'tris':>9}{'n-mf':>7}{'cracks':>8}{'mismatch cm':>15}{'above deck':>24}  invariants"
    print("\n" + hdr)
    print("-" * len(hdr))
    for r in rows:
        if r.get("error"):
            print(f"{r['name']:<22}{r['kind']:<10}{'ERROR ' + r['error']:>9}")
            continue
        mm = (f"{g(r,'mismatch_before_cm',0):.0f} → {g(r,'mismatch_after_cm',0):.0f}"
              if "mismatch_after_cm" in r else "—")
        ad = (f"{g(r,'above_deck',0)} of {g(r,'above_deck_of',0)} ({g(r,'above_deck_max_m',0):.1f} m)"
              if "above_deck" in r else "—")
        inv = ("not checked (--quick)" if not r.get("checked", True)
               else "all pass" if not r["failed"]
               else f"{len(r['failed'])} FAIL: " + ", ".join(r["failed"]))
        print(f"{r['name']:<22}{r['kind']:<10}{r['tris']:>9}{r['nonmanifold']:>7}{r['cracks']:>8}"
              f"{mm:>15}{ad:>24}  {inv[:70]}")
    imported = [r for r in rows if r["kind"] == "imported" and r.get("import")]
    if imported:
        print(f"\n{'scene':<15}{'edges':>7}{'streets':>9}{'dropped':>9}{'landings':>10}{'ramps':>7}"
              f"{'ring min R':>12}")
        print("-" * 70)
        for r in imported:
            i = r["import"]
            ring = "none" if i.get("ring", 0) is None else (f"{i['ring_min_r']:.0f} m of {i['ring_target_r']:.0f}"
                                                            if "ring_min_r" in i else "—")
            print(f"{r['name']:<15}{i.get('edges','—'):>7}{i.get('streets','—'):>9}{i.get('dropped','—'):>9}"
                  f"{i.get('landings','—'):>10}{i.get('ramps','—'):>7}{ring:>12}")


# A gate is only useful if it knows what "worse" means. Counts must not grow; the
# continuous numbers get a tolerance, because a mesher is not bit-stable across
# compilers and a gate that cries over 0.3 cm gets switched off.
WATCH = [("nonmanifold", 0), ("cracks", 0), ("odd_boundary", 0),
         ("mismatch_after_cm", 5.0), ("above_deck", 25), ("above_deck_max_m", 0.05)]


def check(rows, base):
    by = {r["name"]: r for r in base.get("scenes", [])}
    bad = []
    for r in rows:
        b = by.get(r["name"])
        if b is None:
            print(f"  NEW    {r['name']}: not in the baseline (run --update to record it)")
            continue
        if r.get("error"):
            bad.append(f"{r['name']}: {r['error']}")
            continue
        for k, tol in WATCH:
            if k in r and k in b and r[k] > b[k] + tol:
                bad.append(f"{r['name']}: {k} {b[k]} → {r[k]}")
        if not r.get("checked", True) or not b.get("checked", True):
            continue                    # one side never ran the sweep: compare numbers only
        new_fail = set(r["failed"]) - set(b.get("failed", []))
        if new_fail:
            bad.append(f"{r['name']}: new invariant failures: {', '.join(sorted(new_fail))}")
        fixed = set(b.get("failed", [])) - set(r["failed"])
        if fixed:
            print(f"  BETTER {r['name']}: now passes {', '.join(sorted(fixed))}")
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tool", default=os.path.join(ROOT, "build-viewer", "lanes_tool"))
    ap.add_argument("--out", default="/tmp/lanes-corpus")
    ap.add_argument("--only", help="run one scene by name")
    ap.add_argument("--all", action="store_true", help="include scenes marked slow")
    ap.add_argument("--timeout", type=int, default=2400)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--update", action="store_true")
    a = ap.parse_args()
    if not os.path.exists(a.tool):
        sys.exit(f"lanes_tool not found at {a.tool} (build it: ninja -C build-viewer lanes_tool)")
    scenes = json.load(open(CORPUS))["scenes"]
    if a.only:
        scenes = [s for s in scenes if s["name"] == a.only]
    elif not a.all:
        scenes = [s for s in scenes if not s.get("slow")]
    if not scenes:
        sys.exit("no scenes selected")
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for sc in scenes:
        print(f"[{sc['name']}] {sc['kind']} — {sc.get('tests','')}", flush=True)
        rows.append(scene_run(sc, a.tool, a.out, a.timeout))
    table(rows)
    report = {"scenes": rows, "when": time.strftime("%Y-%m-%d %H:%M"), "tool": a.tool}
    with open(os.path.join(a.out, "report.json"), "w") as f:
        json.dump(report, f, indent=1)
    print(f"\nreport: {os.path.join(a.out, 'report.json')}")
    if a.update:
        # The baseline records the NUMBERS and which invariants failed — not each
        # invariant's detail string, which changes with every metre and would bury
        # a real change in diff noise. The full detail stays in the run's report.
        lean = {"when": report["when"],
                "scenes": [{k: v for k, v in r.items() if k != "invariants"} for r in rows]}
        with open(BASELINE, "w") as f:
            json.dump(lean, f, indent=1)
        print(f"baseline updated: {BASELINE}")
        return 0
    if a.check:
        if not os.path.exists(BASELINE):
            sys.exit(f"no baseline at {BASELINE} (run --update once)")
        bad = check(rows, json.load(open(BASELINE)))
        if bad:
            print("\nREGRESSIONS:")
            for b in bad:
                print("  " + b)
            return 1
        print("\nno regressions against the baseline")
    return 0


if __name__ == "__main__":
    sys.exit(main())
