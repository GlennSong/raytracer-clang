#!/usr/bin/env python3
"""Render a FrameStats CSV capture (ADR-0077) as a self-contained HTML report.

The engine writes the capture: set RT_FRAME_STATS=<path.csv> before launch, or
press "Start CSV capture" in the debug overlay's Performance panel. Then:

    python3 tools/frame-report.py frame-capture.csv
    python3 tools/frame-report.py after.csv --compare before.csv -o report.html

Standard library only (no matplotlib), so it runs anywhere the repo clones.
The report answers, at a glance: where does the frame budget go (stacked
phases), how spiky is it (p95/p99 + histogram), and did a change help
(--compare overlays a baseline run in grey).
"""

import argparse
import csv
import html
import math
import os
import sys

PHASES = [
    # (csv column, label, fill color) — draw order is stack order, bottom-up.
    ("update_ms", "update", "#4e79a7"),
    ("fixed_ms", "fixed", "#f28e2b"),
    ("render_ms", "render", "#e15759"),
    ("wait_ms", "wait", "#bbbbbb"),
]


def load_capture(path):
    """Read a FrameStats CSV into a dict of column -> list of floats."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        need = {"total_ms", "update_ms", "fixed_ms", "render_ms", "wait_ms"}
        if not reader.fieldnames or not need.issubset(reader.fieldnames):
            sys.exit(f"{path}: not a FrameStats capture "
                     f"(missing columns {sorted(need)})")
        cols = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name, value in row.items():
                try:
                    cols[name].append(float(value))
                except (TypeError, ValueError):
                    cols[name].append(0.0)
    if not cols["total_ms"]:
        sys.exit(f"{path}: capture holds no frames")
    return cols


def percentile(sorted_values, fraction):
    if not sorted_values:
        return 0.0
    index = min(len(sorted_values) - 1, int(len(sorted_values) * fraction))
    return sorted_values[index]


def summarize(cols):
    totals = sorted(cols["total_ms"])
    n = len(totals)
    host = [d for d in cols.get("host_delta_ms", []) if d > 0]
    s = {
        "frames": n,
        "seconds": sum(host) / 1000.0 if host else 0.0,
        "avg": sum(totals) / n,
        "median": percentile(totals, 0.50),
        "p95": percentile(totals, 0.95),
        "p99": percentile(totals, 0.99),
        "max": totals[-1],
        "fps": (1000.0 * len(host) / sum(host)) if host else 0.0,
    }
    for column, label, _ in PHASES:
        s[label] = sum(cols[column]) / n
    for column in ("draw_calls", "instances", "triangles"):
        values = cols.get(column, [])
        s[column] = (sum(values) / n, max(values)) if values else (0, 0)
    return s


def bucket(values, max_points):
    """Downsample to at most max_points buckets of (mean, peak) pairs, so a
    100k-frame soak still charts — mean draws the area, peak keeps spikes."""
    n = len(values)
    if n <= max_points:
        return list(values), list(values)
    means, peaks = [], []
    step = n / max_points
    for i in range(max_points):
        chunk = values[int(i * step):max(int(i * step) + 1, int((i + 1) * step))]
        means.append(sum(chunk) / len(chunk))
        peaks.append(max(chunk))
    return means, peaks


def polyline(values, width, height, y_max, color, opacity=1.0, fill=None):
    if not values or y_max <= 0:
        return ""
    step = width / max(1, len(values) - 1)
    points = " ".join(
        f"{i * step:.1f},{height - min(v, y_max) / y_max * height:.1f}"
        for i, v in enumerate(values))
    if fill:
        return (f'<polygon points="0,{height} {points} {width},{height}" '
                f'fill="{fill}" opacity="{opacity}"/>')
    return (f'<polyline points="{points}" fill="none" stroke="{color}" '
            f'stroke-width="1.2" opacity="{opacity}"/>')


def budget_lines(width, height, y_max, budgets_ms):
    parts = []
    for ms, label in budgets_ms:
        if ms >= y_max:
            continue
        y = height - ms / y_max * height
        parts.append(f'<line x1="0" y1="{y:.1f}" x2="{width}" y2="{y:.1f}" '
                     f'stroke="#888" stroke-dasharray="4 3" stroke-width="1"/>')
        parts.append(f'<text x="4" y="{y - 3:.1f}" class="tick">{label}</text>')
    return "".join(parts)


def frame_time_chart(cols, baseline, y_max, budgets):
    width, height = 900, 220
    parts = [budget_lines(width, height, y_max, budgets)]
    if baseline:
        base_means, base_peaks = bucket(baseline["total_ms"], width // 2)
        parts.append(polyline(base_peaks, width, height, y_max, "#999", 0.5))
        parts.append(polyline(base_means, width, height, y_max, "#666", 0.8))
    means, peaks = bucket(cols["total_ms"], width // 2)
    parts.append(polyline(peaks, width, height, y_max, "#e15759", 0.45))
    parts.append(polyline(means, width, height, y_max, "#4e79a7"))
    return svg(width, height, "".join(parts))


def stacked_phase_chart(cols, y_max):
    width, height = 900, 220
    n_points = width // 2
    stacked = [0.0] * min(n_points, len(cols["total_ms"]))
    layers = []
    for column, _, color in PHASES:
        means, _ = bucket(cols[column], n_points)
        stacked = [s + v for s, v in zip(stacked, means)]
        layers.append((list(stacked), color))
    parts = []
    for values, color in reversed(layers):   # tallest first so lower shows
        parts.append(polyline(values, width, height, y_max, color, 0.9,
                              fill=color))
    parts.append(budget_lines(width, height, y_max,
                              [(16.67, "60 fps"), (11.11, "90 fps")]))
    return svg(width, height, "".join(parts))


def histogram_chart(values, y_max_ms):
    width, height, bins = 900, 160, 60
    top = max(y_max_ms, 1e-6)
    counts = [0] * bins
    for v in values:
        counts[min(bins - 1, int(v / top * bins))] += 1
    peak = max(counts) or 1
    bar_w = width / bins
    parts = []
    for i, c in enumerate(counts):
        if c == 0:
            continue
        bar_h = c / peak * (height - 14)
        parts.append(f'<rect x="{i * bar_w:.1f}" y="{height - bar_h:.1f}" '
                     f'width="{bar_w - 1:.1f}" height="{bar_h:.1f}" '
                     f'fill="#4e79a7"/>')
    for ms, label in [(16.67, "60 fps"), (11.11, "90 fps"), (33.33, "30 fps")]:
        if ms < top:
            x = ms / top * width
            parts.append(f'<line x1="{x:.1f}" y1="0" x2="{x:.1f}" '
                         f'y2="{height}" stroke="#888" '
                         f'stroke-dasharray="4 3"/>')
            parts.append(f'<text x="{x + 3:.1f}" y="12" class="tick">'
                         f'{label}</text>')
    return svg(width, height, "".join(parts))


def svg(width, height, body):
    return (f'<svg viewBox="0 0 {width} {height}" '
            f'style="width:100%;max-width:{width}px;background:#fafafa;'
            f'border:1px solid #ddd">{body}</svg>')


def summary_rows(label, s):
    return (f"<tr><th>{html.escape(label)}</th>"
            f"<td>{s['frames']}</td><td>{s['seconds']:.1f}s</td>"
            f"<td>{s['fps']:.1f}</td><td>{s['avg']:.2f}</td>"
            f"<td>{s['median']:.2f}</td><td>{s['p95']:.2f}</td>"
            f"<td>{s['p99']:.2f}</td><td>{s['max']:.2f}</td>"
            f"<td>{s['update']:.2f}</td><td>{s['fixed']:.2f}</td>"
            f"<td>{s['render']:.2f}</td><td>{s['wait']:.2f}</td>"
            f"<td>{s['draw_calls'][0]:.0f} / {s['draw_calls'][1]:.0f}</td>"
            f"<td>{s['triangles'][0] / 1e6:.2f}M</td></tr>")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", help="FrameStats CSV to report on")
    parser.add_argument("--compare", metavar="BASELINE",
                        help="second capture drawn in grey for before/after")
    parser.add_argument("-o", "--out", default=None,
                        help="output HTML path (default: <capture>.html)")
    parser.add_argument("--budget-fps", type=int, default=60,
                        help="frame budget the charts are scaled to (default 60)")
    args = parser.parse_args()

    cols = load_capture(args.capture)
    baseline = load_capture(args.compare) if args.compare else None
    out_path = args.out or os.path.splitext(args.capture)[0] + ".html"

    s = summarize(cols)
    budget_ms = 1000.0 / args.budget_fps
    all_p99 = [s["p99"]] + ([summarize(baseline)["p99"]] if baseline else [])
    y_max = max(budget_ms * 2, math.ceil(max(all_p99) * 1.2))

    rows = [summary_rows(os.path.basename(args.capture), s)]
    if baseline:
        rows.append(summary_rows(os.path.basename(args.compare) + " (baseline)",
                                 summarize(baseline)))

    legend = " ".join(
        f'<span style="color:{color}">&#9632; {label}</span>'
        for _, label, color in PHASES)
    compare_note = (" — baseline in grey" if baseline else "")

    doc = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Frame report — {html.escape(os.path.basename(args.capture))}</title>
<style>
body {{ font: 14px/1.5 -apple-system, "Segoe UI", sans-serif; margin: 2em auto;
       max-width: 940px; color: #222; }}
table {{ border-collapse: collapse; width: 100%; font-size: 13px; }}
th, td {{ border: 1px solid #ddd; padding: 4px 8px; text-align: right; }}
th {{ background: #f4f4f4; }} tr th:first-child {{ text-align: left; }}
h2 {{ margin-top: 1.6em; font-size: 16px; }}
.tick {{ font-size: 11px; fill: #666; }}
.note {{ color: #666; font-size: 13px; }}
.howto {{ background: #f6f8fa; border: 1px solid #ddd; border-radius: 6px;
          padding: 0.2em 1.2em 0.8em; font-size: 13px; margin: 1em 0; }}
.howto li {{ margin: 0.3em 0; }}
</style></head><body>
<h1>Frame report</h1>
<p class="note">{s['frames']} frames, {s['seconds']:.1f}s at {s['fps']:.1f} fps
avg. Scale: 0&ndash;{y_max:.0f} ms, budget {budget_ms:.2f} ms
({args.budget_fps} fps).</p>
<details class="howto" open><summary><b>How to read this report</b></summary>
<ul>
<li>Every number is <b>milliseconds per frame</b> — how long each drawn
picture took. The budget is {budget_ms:.1f} ms ({args.budget_fps} fps):
staying under it is the whole game.</li>
<li><b>avg</b> = a typical frame. <b>p95</b> = 19 of every 20 frames were
faster than this — the <i>stutter detector</i>: a good avg with a bad p95
means the game runs fast but hitches. <b>max</b> = the single worst hitch.</li>
<li>Phases: <b>update</b> = input + game logic, <b>fixed</b> = physics
steps, <b>render</b> = building + submitting the picture, <b>wait</b> =
finished early and slept. Wait is <i>headroom</i>, not cost.</li>
<li><b>Chart 1</b>: your session left to right; height = each frame's cost.
Blue under the dashed budget line = running fine; red spikes above it =
hitches you felt, at the moment you felt them.</li>
<li><b>Chart 2</b>: the same timeline, each frame's cost split into stacked
colored layers — <b>the fattest layer is where the time goes</b>; grey on
top is sleep (good).</li>
<li><b>Chart 3</b>: how often each frame cost occurred. One tight clump left
of the budget line = smooth; a tail smearing right = stutter.</li>
<li><b>The decision</b>: if <span style="color:#e15759">render</span>
dominates chart 2 while draw calls are modest, the frame is GPU-bound — use
the platform GPU profiler (Xcode's capture). If
<span style="color:#4e79a7">update</span>/<span style="color:#f28e2b">fixed</span>
dominate, it's CPU-bound — attach Tracy (docs/profiling.md).</li>
</ul></details>
<table><tr><th>capture</th><th>frames</th><th>time</th><th>fps</th>
<th>avg</th><th>median</th><th>p95</th><th>p99</th><th>max</th>
<th>update</th><th>fixed</th><th>render</th><th>wait</th>
<th>draws avg/max</th><th>tris</th></tr>{''.join(rows)}</table>
<h2>Chart 1 &mdash; Frame time over the session (blue typical, red worst{compare_note})</h2>
{frame_time_chart(cols, baseline, y_max, [(budget_ms, f"{args.budget_fps} fps"), (budget_ms * 2, f"{args.budget_fps // 2} fps")])}
<h2>Chart 2 &mdash; Where the frame goes, stacked: {legend}</h2>
{stacked_phase_chart(cols, y_max)}
<p class="note">Wait is the FPS-cap sleep &mdash; headroom, not cost. The gap
between the stack and the frame-time line is unattributed time (event
dispatch, state swaps, host overhead): if it grows, a phase bracket is
missing.</p>
<h2>Chart 3 &mdash; How often each frame cost occurred</h2>
{histogram_chart(cols["total_ms"], y_max)}
</body></html>
"""
    with open(out_path, "w") as f:
        f.write(doc)
    print(f"report: {out_path}  ({s['frames']} frames, avg {s['avg']:.2f} ms, "
          f"p95 {s['p95']:.2f} ms, max {s['max']:.2f} ms)")


if __name__ == "__main__":
    main()
