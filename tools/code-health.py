#!/usr/bin/env python3
"""Scan the codebase for patch-shaped code: duplication, god-functions,
debt markers, and rebuild-cost hotspots.

    python3 tools/code-health.py                 # scan src/, human report
    python3 tools/code-health.py src tools       # scan more roots
    python3 tools/code-health.py --json out.json # machine-readable snapshot

Standard library only. This is a *finder*, not a gate: it points a reviewer at
the places most likely to be layered patches — the same class of debt
TECH_DEBT.md tracks by hand (copy-pasted binding helpers, god-functions).
Findings are candidates to judge, not violations to zero out; commit a --json
snapshot before/after a cleanup to show the trend.

What it looks for:
  * duplicate blocks — runs of >= WINDOW significant lines repeated verbatim
    (whitespace-insensitive) within or across files: the "second copy drifts"
    hazard (e.g. the checkVec3/pushVec3 helpers duplicated across Lua binding
    surfaces, see TECH_DEBT.md). KNOWN BLIND SPOT: matching is verbatim, so a
    clone whose identifiers were renamed evades it (the two parseTerrainParams
    copies differed only by `tp` vs `p` and were caught by hand). Treat the
    list as a floor, not a census; identifier-normalized hashing is the
    planned upgrade.
  * long functions — bodies over --long-fn physical lines; candidates for a
    seam, like loadVegetation's species/scatter/spawn split.
  * debt markers — TODO/FIXME/HACK/XXX/"workaround"/"for now": where past
    sessions knowingly patched. Density (per 100 lines) beats raw count.
  * include fan-in — headers included by many translation units; a heavy,
    high-fan-in header is a rebuild-cost hotspot and a coupling smell.
"""

import argparse
import json
import os
import re
import sys

EXTENSIONS = {".h", ".hpp", ".cpp", ".cc", ".mm", ".m",
              ".metal", ".frag", ".vert", ".comp", ".glsl", ".wgsl"}
SKIP_DIRS = {"third_party", ".git", "build", "web"}
SKIP_DIR_PREFIXES = ("build-",)

MARKER_RE = re.compile(
    r"\b(TODO|FIXME|HACK|XXX)\b|workaround|for now|temporar|stop-?gap",
    re.IGNORECASE)
INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)
BLOCK_KEYWORDS = ("if", "else", "for", "while", "switch", "do", "catch",
                  "namespace", "class", "struct", "enum", "union", "extern",
                  "try")
FUNC_NAME_RE = re.compile(r"([~\w][\w:<>~]*)\s*\(")


def iter_source_files(roots):
    for root in roots:
        if os.path.isfile(root):
            yield root
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames
                           if d not in SKIP_DIRS
                           and not d.startswith(SKIP_DIR_PREFIXES)]
            for name in sorted(filenames):
                if os.path.splitext(name)[1] in EXTENSIONS:
                    yield os.path.join(dirpath, name)


def strip_comments(text):
    """Remove /* */ and // comments (string literals are not comment-aware —
    good enough for a report tool)."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", text)


def significant_lines(text):
    """(line_number, normalized) for lines that carry real content: comments
    stripped, whitespace collapsed, trivial brace/keyword lines dropped so
    duplicate windows measure logic, not formatting."""
    out = []
    for i, raw in enumerate(strip_comments(text).splitlines(), start=1):
        line = " ".join(raw.split())
        if len(line) < 5 or line.startswith("#"):
            continue
        out.append((i, line))
    return out


# --- duplicate blocks ---------------------------------------------------------

def find_duplicates(files, window, top):
    lines_by_file = {}
    windows = {}
    for path, text in files.items():
        sig = significant_lines(text)
        lines_by_file[path] = sig
        for i in range(len(sig) - window + 1):
            key = "\n".join(line for _, line in sig[i:i + window])
            windows.setdefault(key, []).append((path, i))

    # One duplicated region spawns many overlapping window groups (and a
    # region copied N times pairs up N ways), so cluster runs that touch the
    # same source lines into one *family* and report each family once.
    claimed = {}    # (path, sig_index) -> family id
    families = {}   # id -> {"lines": max run, "spans": {(path, a, b), ...}}
    next_id = 0
    for key, sites in sorted(windows.items(),
                             key=lambda kv: (-len(kv[1]), kv[0])):
        if len(sites) < 2:
            continue
        if all((site in claimed) for site in sites):
            continue
        # Grow the run while every site keeps matching line-for-line.
        length = window
        while True:
            nxt = []
            for path, i in sites:
                sig = lines_by_file[path]
                if i + length >= len(sig):
                    break
                nxt.append(sig[i + length][1])
            if len(nxt) == len(sites) and len(set(nxt)) == 1:
                length += 1
            else:
                break

        touched = {claimed[(path, j)]
                   for path, i in sites for j in range(i, i + length)
                   if (path, j) in claimed}
        if touched:
            family_id = min(touched)
            for other in touched - {family_id}:   # merge families this bridges
                families[family_id]["spans"] |= families.pop(other)["spans"]
                for cell, owner in claimed.items():
                    if owner == other:
                        claimed[cell] = family_id
        else:
            family_id = next_id
            next_id += 1
            families[family_id] = {"lines": 0, "spans": set()}

        family = families[family_id]
        family["lines"] = max(family["lines"], length)
        for path, i in sites:
            family["spans"].add((path, lines_by_file[path][i][0],
                                 lines_by_file[path][i + length - 1][0]))
            for j in range(i, i + length):
                claimed[(path, j)] = family_id

    runs = []
    for family in families.values():
        spans = merge_spans(family["spans"])
        if len(spans) < 2:
            continue
        runs.append({"lines": family["lines"], "sites": spans})
    runs.sort(key=lambda r: -r["lines"] * len(r["sites"]))
    return runs[:top]


def merge_spans(spans):
    """Collapse overlapping/adjacent (path, first, last) line spans per file."""
    merged = []
    for path, a, b in sorted(spans):
        if merged and merged[-1][0] == path and a <= merged[-1][2] + 1:
            merged[-1] = (path, merged[-1][1], max(merged[-1][2], b))
        else:
            merged.append((path, a, b))
    return merged


# --- long functions -----------------------------------------------------------

def find_long_functions(files, threshold, top):
    found = []
    for path, text in files.items():
        clean = strip_comments(text)
        lines = clean.splitlines()
        header = ""
        stack = []   # (is_function, name, start_line)
        for lineno, raw in enumerate(lines, start=1):
            line = raw.strip()
            if line.startswith("#"):
                continue
            for ch in raw:
                if ch == "{":
                    head = (header + " " + line).strip()
                    first_word = re.split(r"[\s(]", head.lstrip("} "), 1)[0]
                    is_fn = ("(" in head and ")" in head
                             and first_word not in BLOCK_KEYWORDS
                             and "=" not in head.split("(")[0])
                    name = ""
                    if is_fn:
                        matches = FUNC_NAME_RE.findall(head)
                        name = matches[-1] if matches else "?"
                        if name in BLOCK_KEYWORDS:
                            is_fn = False
                    # Only track top-level-ish functions, not nested lambdas.
                    if any(f for f, _, _ in stack):
                        is_fn = False
                    stack.append((is_fn, name, lineno))
                    header = ""
                elif ch == "}":
                    if stack:
                        is_fn, name, start = stack.pop()
                        body = lineno - start + 1
                        if is_fn and body >= threshold:
                            found.append({"file": path, "name": name,
                                          "line": start, "body_lines": body})
            if line.endswith((";", "{", "}")):
                header = ""
            else:
                header = (header + " " + line)[-300:]
    found.sort(key=lambda f: -f["body_lines"])
    return found[:top]


# --- debt markers -------------------------------------------------------------

def find_markers(files, top):
    per_file = []
    total = 0
    for path, text in files.items():
        count = len(MARKER_RE.findall(text))
        total += count
        if count:
            n_lines = text.count("\n") + 1
            per_file.append({"file": path, "markers": count,
                             "per_100_lines": round(100.0 * count / n_lines, 2)})
    per_file.sort(key=lambda f: -f["markers"])
    return total, per_file[:top]


# --- include fan-in -----------------------------------------------------------

def find_fan_in(files, top):
    fan_in = {}
    for path, text in files.items():
        seen = set()
        for match in INCLUDE_RE.finditer(text):
            base = os.path.basename(match.group(1))
            if base not in seen:
                seen.add(base)
                fan_in[base] = fan_in.get(base, 0) + 1
    sizes = {os.path.basename(p): t.count("\n") + 1 for p, t in files.items()}
    ranked = [{"header": h, "included_by": n, "lines": sizes.get(h, 0),
               "cost": n * sizes.get(h, 0)}
              for h, n in fan_in.items()]
    ranked.sort(key=lambda r: -r["cost"])
    return ranked[:top]


# --- report -------------------------------------------------------------------

def print_report(report, dup_window, long_fn):
    print(f"code health — {report['files']} files, "
          f"{report['total_lines']} lines scanned\n")

    print(f"== duplicate blocks (>= {dup_window} significant lines, "
          f"top {len(report['duplicates'])}) ==")
    if not report["duplicates"]:
        print("  none")
    for run in report["duplicates"]:
        sites = ", ".join(f"{path}:{a}-{b}" for path, a, b in run["sites"])
        print(f"  {run['lines']:3d} lines x {len(run['sites'])} sites: {sites}")

    print(f"\n== long functions (>= {long_fn} lines) ==")
    if not report["long_functions"]:
        print("  none")
    for fn in report["long_functions"]:
        print(f"  {fn['body_lines']:4d} lines  {fn['file']}:{fn['line']}  "
              f"{fn['name']}")

    print(f"\n== debt markers ({report['marker_total']} total) ==")
    for f in report["markers"]:
        print(f"  {f['markers']:4d} ({f['per_100_lines']:5.1f}/100 lines)  "
              f"{f['file']}")

    print("\n== include fan-in (rebuild-cost hotspots: included_by x lines) ==")
    for r in report["fan_in"]:
        print(f"  {r['included_by']:3d} includers x {r['lines']:5d} lines  "
              f"{r['header']}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("roots", nargs="*", default=["src"],
                        help="files or directories to scan (default: src)")
    parser.add_argument("--dup-window", type=int, default=8,
                        help="minimum duplicate run, in significant lines")
    parser.add_argument("--long-fn", type=int, default=100,
                        help="long-function threshold, in body lines")
    parser.add_argument("--top", type=int, default=20,
                        help="entries per section")
    parser.add_argument("--json", metavar="PATH",
                        help="also write the full report as JSON")
    args = parser.parse_args()

    files = {}
    for path in iter_source_files(args.roots):
        try:
            with open(path, encoding="utf-8", errors="replace") as f:
                files[path] = f.read()
        except OSError as e:
            print(f"skip {path}: {e}", file=sys.stderr)
    if not files:
        sys.exit("nothing to scan")

    marker_total, markers = find_markers(files, args.top)
    report = {
        "files": len(files),
        "total_lines": sum(t.count("\n") + 1 for t in files.values()),
        "duplicates": find_duplicates(files, args.dup_window, args.top),
        "long_functions": find_long_functions(files, args.long_fn, args.top),
        "marker_total": marker_total,
        "markers": markers,
        "fan_in": find_fan_in(files, args.top),
    }

    print_report(report, args.dup_window, args.long_fn)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=1)
        print(f"\njson: {args.json}")


if __name__ == "__main__":
    main()
