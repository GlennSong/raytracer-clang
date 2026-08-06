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


ALL_COLUMNS = ("total_ms", "update_ms", "fixed_ms", "render_ms", "wait_ms",
               "host_delta_ms", "fixed_steps", "draw_calls", "instances",
               "triangles", "acquire_ms", "encode_ms", "submit_ms", "gpu_ms")


def load_capture(path):
    """Read a FrameStats CSV into a dict of column -> list of floats.

    Tolerant of schema versions: captures predating the render split (no
    acquire/encode/submit/gpu columns) read fine, their missing columns
    filled with zeros, so an old capture still opens and still compares."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or "total_ms" not in reader.fieldnames:
            sys.exit(f"{path}: not a FrameStats capture (no total_ms column)")
        cols = {name: [] for name in set(reader.fieldnames) | set(ALL_COLUMNS)}
        rows = 0
        for row in reader:
            rows += 1
            for name in cols:
                try:
                    cols[name].append(float(row.get(name) or 0.0))
                except (TypeError, ValueError):
                    cols[name].append(0.0)
    if not rows:
        sys.exit(f"{path}: capture holds no frames")
    return cols


def has_render_split(cols):
    """True when the capture carries the acquire/encode/submit breakdown."""
    return any(v > 0 for v in cols["acquire_ms"]) or \
           any(v > 0 for v in cols["encode_ms"]) or \
           any(v > 0 for v in cols["submit_ms"])


def phases_for(cols):
    """Stack layers: the render split when present, else the flat render."""
    if has_render_split(cols):
        return [("update_ms", "update", "#4e79a7"),
                ("fixed_ms", "fixed", "#f28e2b"),
                ("encode_ms", "encode (cpu)", "#59a14f"),
                ("submit_ms", "submit (cpu)", "#b07aa1"),
                ("acquire_ms", "acquire (waiting on GPU)", "#e15759"),
                ("wait_ms", "wait", "#bbbbbb")]
    return PHASES


def percentile(sorted_values, fraction):
    if not sorted_values:
        return 0.0
    index = min(len(sorted_values) - 1, int(len(sorted_values) * fraction))
    return sorted_values[index]


def vsync_note(median_ms, budget_ms):
    """A frame that misses the display's deadline waits for the NEXT refresh, so
    costs cluster on multiples of the refresh interval (16.7 / 33.3 / 50 ms at
    60 Hz). Spotting that clustering matters: once frames are quantized, the
    measured time is 'work + waiting for the next scanout', so the real cost is
    hidden and only gpu_ms can reveal it."""
    for multiple in (1, 2, 3, 4):
        target = budget_ms * multiple
        if abs(median_ms - target) <= max(0.6, target * 0.03):
            if multiple == 1:
                return None      # hitting the budget; nothing to explain
            return (f"Frames are landing on <b>{target:.1f} ms</b> — exactly "
                    f"{multiple}&times; the {budget_ms:.1f} ms display "
                    f"interval. The engine is missing the deadline and waiting "
                    f"for the next refresh, so it runs at "
                    f"{1000.0 / target:.0f} fps in lockstep. The measured time "
                    f"includes that waiting, so the true cost is somewhere "
                    f"between {budget_ms:.1f} and {target:.1f} ms — the "
                    f"<code>gpu_ms</code> column is what pins it down.")
    return None


def verdict(cols, s, budget_ms):
    """Plain-language reading of what this capture says, and what to do next."""
    lines = []
    note = vsync_note(s["median"], budget_ms)
    if note:
        lines.append(note)

    cpu_render = s["encode"] + s["submit"]
    logic = s["update"] + s["fixed"]
    split = has_render_split(cols)
    draws = s["draw_calls"][0]
    tris = s["triangles"][0]

    if split and s["acquire"] > max(cpu_render, logic) and s["acquire"] > 1.0:
        lines.append(
            f"<b>The frame is GPU-bound.</b> {s['acquire']:.1f} ms of every "
            f"{s['avg']:.1f} ms frame is spent <i>blocked</i> waiting for the "
            f"GPU to hand back a drawable — the CPU is idle in that time. Your "
            f"own code is cheap: {logic:.2f} ms of game logic and "
            f"{cpu_render:.2f} ms building the frame. Optimising C++ will not "
            f"move this number; the work is in the shaders and passes.")
        if draws and draws < 200 and tris < 2e6:
            lines.append(
                f"Scene complexity is <b>not</b> the cause either: only "
                f"{draws:.0f} draw calls and {tris/1e6:.2f}M triangles per "
                f"frame. That points at the <b>screen-space passes</b> (SSAO, "
                f"SSR, bloom, shadow maps), whose cost scales with PIXELS, not "
                f"geometry. Two experiments that need no code: shrink the "
                f"window by half (if frame time drops hard, it is pixel-bound "
                f"— on a Retina display the framebuffer is 4&times; the logical "
                f"size), and toggle SSAO / SSR / Bloom in the Debug panel one "
                f"at a time, watching the Performance readout to rank them.")
    elif split and cpu_render > logic and cpu_render > 1.0:
        lines.append(
            f"<b>The frame is CPU-bound in the renderer.</b> {cpu_render:.1f} "
            f"ms goes to walking the world and building command buffers "
            f"(encode {s['encode']:.1f} + submit {s['submit']:.1f}), against "
            f"{logic:.2f} ms of game logic. Attach Tracy "
            f"(-DRT_ENABLE_PROFILER=ON) to see which system.")
    elif logic > 1.0 and logic > cpu_render:
        lines.append(
            f"<b>The frame is CPU-bound in game logic</b> ({logic:.1f} ms in "
            f"update+fixed). Attach Tracy (-DRT_ENABLE_PROFILER=ON) to see "
            f"which system.")
    elif not split:
        lines.append(
            "This capture predates the render split, so it can only say "
            "<i>render</i> is where the time goes — not whether that is real "
            "work or waiting on the GPU. Re-capture with a current build to "
            "get the acquire / encode / submit breakdown and the GPU column.")

    if s["gpu"] > 0:
        head = "GPU-bound" if s["gpu"] > cpu_render else "CPU-bound"
        lines.append(
            f"Measured GPU time: <b>{s['gpu']:.1f} ms/frame</b> against "
            f"{cpu_render:.1f} ms of CPU frame-building &mdash; {head}. "
            f"(Budget is {budget_ms:.1f} ms.)")
    elif split:
        lines.append(
            "No GPU timing in this capture — the backend does not report it "
            "(Metal does; Vulkan/WebGPU do not yet).")

    if s["max"] > 4 * budget_ms:
        lines.append(
            f"One-off hitches: the worst frame took <b>{s['max']:.0f} ms</b> "
            f"(p99 {s['p99']:.0f} ms). Spikes that big are usually a load, an "
            f"upload, or a shader compile rather than steady-state cost — find "
            f"them by where they sit in chart 1.")
    return lines


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
    for column in ("update_ms", "fixed_ms", "render_ms", "wait_ms",
                   "acquire_ms", "encode_ms", "submit_ms", "gpu_ms"):
        s[column[:-3]] = sum(cols[column]) / n
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
    for column, _, color in phases_for(cols):
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
            f"<td>{s['render']:.2f}</td><td>{s['acquire']:.2f}</td>"
            f"<td>{s['encode']:.2f}</td><td>{s['submit']:.2f}</td>"
            f"<td>{s['wait']:.2f}</td>"
            f"<td>{s['gpu']:.2f}</td>"
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
        for _, label, color in phases_for(cols))
    compare_note = (" — baseline in grey" if baseline else "")
    verdict_html = "".join(f"<p>{line}</p>" for line in
                           verdict(cols, s, budget_ms))

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
.verdict {{ background: #fff8e6; border: 1px solid #e8d8a8;
            border-left: 4px solid #e8b93b; border-radius: 6px;
            padding: 0.8em 1.2em; margin: 1em 0; }}
.verdict p {{ margin: 0.5em 0; }}
.verdict h2 {{ margin: 0 0 0.3em; font-size: 15px; }}
code {{ background: #eee; padding: 0 3px; border-radius: 3px; }}
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
<div class="verdict"><h2>What this capture says</h2>{verdict_html}</div>
<table><tr><th>capture</th><th>frames</th><th>time</th><th>fps</th>
<th>avg</th><th>median</th><th>p95</th><th>p99</th><th>max</th>
<th>update</th><th>fixed</th><th>render</th><th>acquire</th><th>encode</th>
<th>submit</th><th>wait</th><th>gpu</th>
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
