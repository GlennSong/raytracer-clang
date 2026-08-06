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
import json
import math
import os
import re
import sys
import textwrap

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
        # A mean above budget while the median frame fits inside it is
        # arithmetic, not a contradiction — and it changes what to fix.
        if s["gpu"] > budget_ms and s["median"] <= budget_ms * 1.05:
            lines.append(
                f"Note the tension: mean GPU time ({s['gpu']:.1f} ms) is over "
                f"budget, yet the median frame ({s['median']:.1f} ms) fits "
                f"inside it — a frame cannot take {s['median']:.1f} ms while "
                f"its own GPU work takes {s['gpu']:.1f}. Both are true because "
                f"the <i>mean</i> is inflated by the spikes. Your typical "
                f"frame is close to budget and the spikes are what break it, "
                f"so <b>chase the hitches first</b>; chart 4's <i>typical</i> "
                f"column is the steady-state number to judge by.")
    elif split:
        lines.append(
            "No GPU timing in this capture — the backend does not report it "
            "(Metal does; Vulkan/WebGPU do not yet).")

    # Steady-state cost and hitching are different problems with different
    # fixes, and an average blends them into one misleading number.
    if s["median"] > 0 and s["p99"] > 3 * s["median"]:
        lines.append(
            f"<b>Hitching is a separate problem here, and probably the one you "
            f"feel.</b> The typical frame is {s['median']:.1f} ms but 1 in 100 "
            f"takes {s['p99']:.0f} ms — {s['p99'] / s['median']:.0f}&times; "
            f"longer — and the worst hit {s['max']:.0f} ms. That is why the "
            f"average ({s['avg']:.1f} ms) sits so far above the median: a "
            f"minority of very slow frames is dragging it. Steady-state cost "
            f"and hitching need different fixes — see <b>chart 4</b> for which "
            f"phase actually grows during a spike, and use "
            f"<i>Jump to worst frame</i> on chart 1 to see where they fall.")
    elif s["max"] > 4 * budget_ms:
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


MAX_EMBEDDED = 40000   # frames sent to the browser verbatim (~2 MB of JSON)


def embed_series(cols, baseline, layers, budget_ms, y_max):
    """The per-frame data the page's charts render from.

    Sent verbatim (rounded to 2 dp) so hovering reports a REAL frame's numbers
    rather than a bucket average. Very long captures are strided down, but
    every frame at or above the 99th percentile is kept regardless — losing
    the spikes would lose the thing worth looking at."""
    n = len(cols["total_ms"])
    keep = list(range(n))
    if n > MAX_EMBEDDED:
        cut = percentile(sorted(cols["total_ms"]), 0.99)
        stride = math.ceil(n / MAX_EMBEDDED)
        keep = sorted({i for i in range(0, n, stride)} |
                      {i for i, v in enumerate(cols["total_ms"]) if v >= cut})

    def series(column):
        return [round(cols[column][i], 2) for i in keep]

    data = {
        "n": len(keep),
        "frameNo": [i + 1 for i in keep],
        "budget": round(budget_ms, 3),
        "yMax": round(y_max, 2),
        "total": series("total_ms"),
        "gpu": series("gpu_ms"),
        "draws": [int(cols["draw_calls"][i]) for i in keep],
        "tris": [int(cols["triangles"][i]) for i in keep],
        "layers": [{"key": key, "label": label, "color": color,
                    "values": series(key)} for key, label, color in layers],
        "strided": len(keep) < n,
    }
    if baseline:
        bn = len(baseline["total_ms"])
        step = max(1, math.ceil(bn / 2000))
        data["baseline"] = [round(baseline["total_ms"][i], 2)
                            for i in range(0, bn, step)]
    return data


def spike_anatomy(cols, layers):
    """Compare the composition of the slowest 1% of frames against typical
    ones, phase by phase. This is what a summary table cannot show: an average
    frame and a spike can differ in WHICH phase grew, and that difference names
    the cause (a stall waiting on the GPU vs a pipeline build vs a CPU hitch)."""
    totals = cols["total_ms"]
    n = len(totals)
    ordered = sorted(totals)
    p99 = percentile(ordered, 0.99)
    lo, hi = percentile(ordered, 0.40), percentile(ordered, 0.60)

    typical = [i for i, v in enumerate(totals) if lo <= v <= hi]
    slow = [i for i, v in enumerate(totals) if v >= p99]
    if not typical or not slow:
        return None

    def mean(indices, column):
        return sum(cols[column][i] for i in indices) / len(indices)

    every = range(n)
    rows = []
    for key, label, color in layers:
        t, sl = mean(typical, key), mean(slow, key)
        rows.append({"label": label, "color": color, "typical": t,
                     "mean": mean(every, key), "slow": sl, "delta": sl - t})
    rows.append({"label": "GPU (lags 1-2 frames)", "color": "#666",
                 "typical": mean(typical, "gpu_ms"),
                 "mean": mean(every, "gpu_ms"),
                 "slow": mean(slow, "gpu_ms"),
                 "delta": mean(slow, "gpu_ms") - mean(typical, "gpu_ms")})
    rows.sort(key=lambda r: -abs(r["delta"]))

    worst = sorted(range(n), key=lambda i: -totals[i])[:10]
    return {
        "typical_total": mean(typical, "total_ms"),
        "slow_total": mean(slow, "total_ms"),
        "typical_count": len(typical),
        "slow_count": len(slow),
        "rows": rows,
        "worst": [{"frame": i + 1, "total": totals[i],
                   "parts": [(label, cols[key][i]) for key, label, _ in layers],
                   "gpu": cols["gpu_ms"][i],
                   "draws": int(cols["draw_calls"][i])} for i in worst],
    }


def spike_section(a, budget_ms):
    if not a:
        return ""
    top = a["rows"][0]
    share = (top["delta"] / max(1e-6, a["slow_total"] - a["typical_total"])) * 100
    bars = []
    scale = max(r["slow"] for r in a["rows"]) or 1.0
    for r in a["rows"]:
        bars.append(
            f"<tr><td style='text-align:left'><span style='color:{r['color']}'>"
            f"&#9632;</span> {html.escape(r['label'])}</td>"
            f"<td>{r['typical']:.2f}</td><td>{r['mean']:.2f}</td>"
            f"<td>{r['slow']:.2f}</td>"
            f"<td><b>{r['delta']:+.2f}</b></td>"
            f"<td style='width:40%'><div style='background:{r['color']};"
            f"height:10px;width:{max(0.0, r['slow']) / scale * 100:.1f}%'></div>"
            f"</td></tr>")
    worst = "".join(
        f"<tr><td>{w['frame']}</td><td>{w['total']:.1f}</td>"
        + "".join(f"<td>{v:.2f}</td>" for _, v in w["parts"])
        + f"<td>{w['gpu']:.2f}</td><td>{w['draws']}</td></tr>"
        for w in a["worst"])
    part_heads = "".join(f"<th>{html.escape(label)}</th>"
                         for label, _ in a["worst"][0]["parts"])
    return f"""
<h2>Chart 4 &mdash; Spike anatomy: what grows when a frame goes slow</h2>
<p class="note">The slowest 1% ({a['slow_count']} frames, avg
<b>{a['slow_total']:.1f} ms</b>) against typical ones ({a['typical_count']}
frames near the median, avg {a['typical_total']:.1f} ms). Sorted by how much
each phase grew &mdash; <b>the top row is what a spike actually is</b>.</p>
<table><tr><th>phase</th><th>typical</th><th>mean (all)</th><th>slow 1%</th>
<th>growth</th><th>slow-frame size</th></tr>{''.join(bars)}</table>
<p class="note"><b>typical</b> is what a normal frame costs; <b>mean</b> is the
average the summary table above reports. When they differ, the mean is being
pulled up by the slow frames &mdash; read <i>typical</i> to judge steady-state
cost and <i>slow 1%</i> to judge hitching. (A mean GPU time above the budget
while the median frame is under it means exactly this: the typical frame fits,
the spikes do not.)</p>
<p class="note"><b>{html.escape(top['label'])}</b> accounts for
{share:.0f}% of the extra time in a slow frame ({top['delta']:+.2f} ms of
{a['slow_total'] - a['typical_total']:+.2f} ms). GPU time lags its frame by
one or two, so read that row as "around here", not exact.</p>
<h2>The ten worst frames</h2>
<p class="note">Frame numbers are positions in the capture &mdash; early ones
are usually load, upload, or shader compilation rather than steady-state
cost. Budget is {budget_ms:.1f} ms.</p>
<table><tr><th>frame</th><th>total</th>{part_heads}<th>gpu</th><th>draws</th>
</tr>{worst}</table>"""


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


# The timeline charts render in the browser from the embedded per-frame data:
# one renderer, so zoom/hover/layer-toggle reuse the same drawing code instead
# of a second copy of it in Python. Plain string (not an f-string) — JS is all
# braces. __DATA__ is replaced with the JSON payload.
CHART_JS = r"""
<script>
const R = __DATA__;
const W = 900, H = 220;
let view = {lo: 0, hi: R.n};
let hidden = {};

const svgNS = "http://www.w3.org/2000/svg";
const el = (id) => document.getElementById(id);

// Bucket [lo,hi) down to W/2 columns; keep the peak as well as the mean so a
// one-frame spike survives being drawn at 450 px wide.
function buckets(values, lo, hi, cols) {
  const out = {mean: [], peak: [], idx: []};
  const span = Math.max(1, hi - lo), step = span / cols;
  for (let i = 0; i < cols; i++) {
    const a = lo + Math.floor(i * step), b = Math.max(a + 1, lo + Math.floor((i + 1) * step));
    let sum = 0, peak = -Infinity, peakAt = a;
    for (let j = a; j < b && j < R.n; j++) {
      sum += values[j];
      if (values[j] > peak) { peak = values[j]; peakAt = j; }
    }
    const count = Math.max(1, Math.min(b, R.n) - a);
    out.mean.push(sum / count); out.peak.push(peak); out.idx.push(peakAt);
  }
  return out;
}

function yOf(v, yMax) { return H - Math.min(v, yMax) / yMax * H; }

function pathFor(values, yMax) {
  const step = W / Math.max(1, values.length - 1);
  return values.map((v, i) => (i ? "L" : "M") + (i * step).toFixed(1) + "," + yOf(v, yMax).toFixed(1)).join(" ");
}

function budgetLines(yMax) {
  let out = "";
  [[R.budget, (1000 / R.budget).toFixed(0) + " fps"],
   [R.budget * 2, (500 / R.budget).toFixed(0) + " fps"]].forEach(([ms, label]) => {
    if (ms >= yMax) return;
    const y = yOf(ms, yMax);
    out += `<line x1="0" y1="${y}" x2="${W}" y2="${y}" stroke="#888" stroke-dasharray="4 3"/>`;
    out += `<text x="4" y="${y - 3}" class="tick">${label}</text>`;
  });
  return out;
}

// Scale to the 98th percentile of what is VISIBLE, not the maximum: a single
// 700 ms load hitch would otherwise squash every normal frame into a flat line
// at the bottom (and make the stacked chart look empty). Outliers clip at the
// ceiling — still visible, and counted in the caption. Zooming into a spike
// raises the visible p98, so the scale follows you in.
function autoMax(lo, hi) {
  const vis = R.total.slice(lo, hi).sort((a, b) => a - b);
  if (!vis.length) return R.budget * 2;
  const p98 = vis[Math.min(vis.length - 1, Math.floor(vis.length * 0.98))];
  return Math.max(R.budget * 1.5, p98 * 1.15);
}

function clipCount(lo, hi, yMax) {
  let n = 0;
  for (let i = lo; i < hi; i++) if (R.total[i] > yMax) n++;
  return n;
}

// An <svg> only hit-tests PAINTED geometry — a CSS background is not painted,
// so without this transparent rect the mouse handlers never fire over empty
// chart area (found by driving the page headlessly, not by reading it).
const HIT = `<rect x="0" y="0" width="${W}" height="${H}" fill="transparent"/>`;

function drawTimeline(lo, hi, yMax) {
  const b = buckets(R.total, lo, hi, W / 2);
  let s = HIT + budgetLines(yMax);
  if (R.baseline) {
    const bb = buckets(R.baseline, 0, R.baseline.length, W / 2);
    s += `<path d="${pathFor(bb.mean, yMax)}" fill="none" stroke="#999" stroke-width="1.2" opacity="0.7"/>`;
  }
  s += `<path d="${pathFor(b.peak, yMax)}" fill="none" stroke="#e15759" stroke-width="1" opacity="0.5"/>`;
  s += `<path d="${pathFor(b.mean, yMax)}" fill="none" stroke="#4e79a7" stroke-width="1.3"/>`;
  el("chart1").innerHTML = s + '<rect id="sel1" x="0" y="0" width="0" height="' + H + '" fill="#4e79a7" opacity="0.15"/><line id="cross1" y1="0" y2="' + H + '" stroke="#333" stroke-width="0.7" opacity="0"/>';
}

// Chart 2 answers "where does a TYPICAL frame's time go", so it gets its own
// scale from its own (bucket-averaged) heights. Sharing chart 1's spike-driven
// scale flattened the whole stack into an unreadable strip at the bottom.
function drawStack(lo, hi) {
  const active = R.layers.filter(l => !hidden[l.key]);
  const cols = W / 2;
  let stacked = new Array(cols).fill(0), s = HIT;
  const bands = [];
  active.forEach(layer => {
    const b = buckets(layer.values, lo, hi, cols);
    stacked = stacked.map((v, i) => v + b.mean[i]);
    bands.push({color: layer.color, values: stacked.slice()});
  });
  const sorted = stacked.slice().sort((a, b) => a - b);
  const p95 = sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * 0.95))] || 0;
  const yMax = Math.max(R.budget * 1.2, p95 * 1.25);
  const step = W / Math.max(1, cols - 1);
  bands.slice().reverse().forEach(band => {
    const pts = band.values.map((v, i) => (i * step).toFixed(1) + "," + yOf(v, yMax).toFixed(1)).join(" ");
    s += `<polygon points="0,${H} ${pts} ${W},${H}" fill="${band.color}" opacity="0.92"/>`;
  });
  s += budgetLines(yMax);
  s += `<text x="${W - 4}" y="12" class="tick" text-anchor="end">0-${yMax.toFixed(1)} ms (mean per column)</text>`;
  el("chart2").innerHTML = s + '<line id="cross2" y1="0" y2="' + H + '" stroke="#333" stroke-width="0.7" opacity="0"/>';
}

function render() {
  const yMax = autoMax(view.lo, view.hi);
  drawTimeline(view.lo, view.hi, yMax);
  drawStack(view.lo, view.hi);
  const clipped = clipCount(view.lo, view.hi, yMax);
  el("range").textContent = "frames " + (R.frameNo[view.lo] || 1) + "-" +
    (R.frameNo[Math.min(view.hi, R.n) - 1] || R.n) + " of " + R.n +
    "  ·  scale 0-" + yMax.toFixed(1) + " ms" +
    (clipped ? "  ·  " + clipped + " frame" + (clipped > 1 ? "s" : "") +
               " clip above the top (drag over one to zoom in)" : "") +
    (R.strided ? "  ·  strided (spikes kept)" : "");
}

function frameAt(clientX, svgEl) {
  const box = svgEl.getBoundingClientRect();
  const t = Math.min(1, Math.max(0, (clientX - box.left) / box.width));
  return Math.min(R.n - 1, view.lo + Math.floor(t * (view.hi - view.lo)));
}

function showTip(evt, svgEl) {
  const i = frameAt(evt.clientX, svgEl);
  const tip = el("tip");
  let rows = `<b>frame ${R.frameNo[i]}</b> &nbsp; total <b>${R.total[i].toFixed(2)} ms</b>`;
  R.layers.forEach(l => {
    rows += `<br><span style="color:${l.color}">&#9632;</span> ${l.label}: ${l.values[i].toFixed(2)}`;
  });
  if (R.gpu[i] > 0) rows += `<br><span style="color:#666">&#9632;</span> gpu: ${R.gpu[i].toFixed(2)} <span style="opacity:.6">(lags)</span>`;
  rows += `<br><span style="opacity:.6">${R.draws[i]} draws · ${(R.tris[i] / 1e6).toFixed(2)}M tris</span>`;
  tip.innerHTML = rows;
  tip.style.display = "block";
  tip.style.left = Math.min(window.innerWidth - 220, evt.clientX + 14) + "px";
  tip.style.top = (evt.clientY + window.scrollY + 14) + "px";
  const frac = (i - view.lo) / Math.max(1, view.hi - view.lo);
  ["cross1", "cross2"].forEach(id => {
    const c = el(id);
    if (c) { c.setAttribute("x1", frac * W); c.setAttribute("x2", frac * W); c.setAttribute("opacity", "0.6"); }
  });
}

function hideTip() {
  el("tip").style.display = "none";
  ["cross1", "cross2"].forEach(id => { const c = el(id); if (c) c.setAttribute("opacity", "0"); });
}

// Drag across either chart to zoom into that span; both charts follow.
function wire(svgId) {
  const s = el(svgId);
  let dragFrom = null;
  s.addEventListener("mousemove", (e) => {
    showTip(e, s);
    if (dragFrom !== null) {
      const box = s.getBoundingClientRect();
      const a = Math.min(dragFrom, e.clientX) - box.left, b = Math.max(dragFrom, e.clientX) - box.left;
      const sel = el("sel1");
      if (sel) { sel.setAttribute("x", a / box.width * W); sel.setAttribute("width", (b - a) / box.width * W); }
    }
  });
  s.addEventListener("mouseleave", hideTip);
  s.addEventListener("mousedown", (e) => { dragFrom = e.clientX; e.preventDefault(); });
  s.addEventListener("mouseup", (e) => {
    if (dragFrom === null) return;
    const lo = frameAt(Math.min(dragFrom, e.clientX), s), hi = frameAt(Math.max(dragFrom, e.clientX), s);
    dragFrom = null;
    const sel = el("sel1"); if (sel) sel.setAttribute("width", 0);
    if (hi - lo >= 4) { view = {lo: lo, hi: hi + 1}; render(); }
  });
}

function resetZoom() { view = {lo: 0, hi: R.n}; render(); }

function jumpToWorst() {
  let worst = 0;
  for (let i = 0; i < R.n; i++) if (R.total[i] > R.total[worst]) worst = i;
  const pad = Math.max(30, Math.floor(R.n * 0.01));
  view = {lo: Math.max(0, worst - pad), hi: Math.min(R.n, worst + pad)};
  render();
}

function toggleLayer(key, node) {
  hidden[key] = !hidden[key];
  node.style.opacity = hidden[key] ? 0.35 : 1;
  node.style.textDecoration = hidden[key] ? "line-through" : "none";
  render();
}

wire("chart1"); wire("chart2"); render();
</script>"""


def plain(markup):
    """HTML fragment -> terminal text (the verdict is authored once, in HTML)."""
    text = re.sub(r"<[^>]+>", "", markup)
    return " ".join(html.unescape(text).split())


def text_report(name, s, cols, layers, budget_ms):
    """The report's findings as paste-able text — for sharing an analysis in a
    chat or a PR without attaching an HTML file."""
    out = [f"=== frame report: {name} ===",
           f"{s['frames']} frames, {s['seconds']:.1f}s, {s['fps']:.1f} fps avg",
           f"total ms  avg {s['avg']:.2f}  median {s['median']:.2f}  "
           f"p95 {s['p95']:.2f}  p99 {s['p99']:.2f}  max {s['max']:.2f}"
           f"   (budget {budget_ms:.2f})", ""]
    for line in verdict(cols, s, budget_ms):
        out += [textwrap.fill(plain(line), 78), ""]

    a = spike_anatomy(cols, layers)
    if a:
        out.append(f"--- spike anatomy: typical ({a['typical_count']} frames) "
                   f"vs slowest 1% ({a['slow_count']} frames) ---")
        out.append(f"{'phase':<28}{'typical':>9}{'mean':>9}{'slow 1%':>10}"
                   f"{'growth':>10}")
        for r in a["rows"]:
            out.append(f"{r['label'][:28]:<28}{r['typical']:>9.2f}"
                       f"{r['mean']:>9.2f}{r['slow']:>10.2f}{r['delta']:>+10.2f}")
        out += ["", "--- ten worst frames ---",
                "frame".rjust(7) + "total".rjust(9) +
                "".join(label[:9].rjust(10) for label, _ in a["worst"][0]["parts"]) +
                "gpu".rjust(9) + "draws".rjust(7)]
        for w in a["worst"]:
            out.append(f"{w['frame']:>7}{w['total']:>9.1f}" +
                       "".join(f"{v:>10.2f}" for _, v in w["parts"]) +
                       f"{w['gpu']:>9.2f}{w['draws']:>7}")
    return "\n".join(out)


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
    parser.add_argument("--text", action="store_true",
                        help="also print the verdict + spike anatomy to the "
                             "terminal (paste-able; no browser needed)")
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

    layers = phases_for(cols)
    legend = " ".join(
        f'<span class="lg" style="color:{color}" '
        f'onclick="toggleLayer(\'{key}\', this)">&#9632; {label}</span>'
        for key, label, color in layers)
    compare_note = (" — baseline in grey" if baseline else "")
    verdict_html = "".join(f"<p>{line}</p>" for line in
                           verdict(cols, s, budget_ms))
    chart_js = CHART_JS.replace(
        "__DATA__",
        json.dumps(embed_series(cols, baseline, layers, budget_ms, y_max),
                   separators=(",", ":")))
    spikes = spike_section(spike_anatomy(cols, layers), budget_ms)

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
.chart {{ width: 100%; max-width: 900px; background: #fafafa;
          border: 1px solid #ddd; cursor: crosshair; user-select: none; }}
.ctrl {{ margin: 0.4em 0; display: flex; gap: 8px; align-items: center; }}
.ctrl button {{ font: 12px inherit; padding: 3px 10px; cursor: pointer;
                border: 1px solid #ccc; border-radius: 4px; background: #fff; }}
.lg {{ cursor: pointer; margin-right: 10px; }}
#tip {{ position: absolute; display: none; background: #222; color: #fff;
        font: 12px/1.45 -apple-system, sans-serif; padding: 7px 10px;
        border-radius: 5px; pointer-events: none; z-index: 10;
        box-shadow: 0 2px 8px rgba(0,0,0,.3); }}
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
<li><b>Chart 4</b>: what actually grows when a frame goes slow — the slowest
1% of frames next to typical ones, phase by phase. <b>The top row is what a
spike is.</b> Steady-state cost and hitching are different problems; this
separates them.</li>
<li><b>Chart 3</b>: how often each frame cost occurred. One tight clump left
of the budget line = smooth; a tail smearing right = stutter.</li>
<li>The timeline charts are <b>interactive</b>: hover for one frame's exact
numbers, drag across to zoom (the scale follows), click a legend colour to
hide that layer, and <i>Jump to worst frame</i> goes straight to the biggest
hitch. Chart 1 is scaled to the 98th percentile so one load spike cannot
flatten everything else — clipped frames are counted next to the buttons.</li>
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
<div class="ctrl"><button onclick="resetZoom()">Reset zoom</button>
<button onclick="jumpToWorst()">Jump to worst frame</button>
<span class="note" id="range"></span></div>
<svg id="chart1" viewBox="0 0 900 220" class="chart"></svg>
<p class="note"><b>Hover</b> any point for that frame's exact breakdown;
<b>drag across</b> either chart to zoom into a span; the y-scale follows the
zoom, so zooming into a quiet stretch reveals detail the spikes were
flattening.</p>
<h2>Chart 2 &mdash; Where the frame goes, stacked: {legend}</h2>
<svg id="chart2" viewBox="0 0 900 220" class="chart"></svg>
<p class="note">Click a colour in the heading to hide that layer &mdash; the
quickest way to see what the rest look like without the dominant one. Wait is
the FPS-cap sleep (headroom, not cost); the gap between the stack and the
frame-time line is unattributed time (event dispatch, state swaps, host
overhead).</p>
{spikes}
<h2>Chart 3 &mdash; How often each frame cost occurred</h2>
{histogram_chart(cols["total_ms"], y_max)}
<div id="tip"></div>
{chart_js}
</body></html>
"""
    with open(out_path, "w") as f:
        f.write(doc)
    print(f"report: {out_path}  ({s['frames']} frames, avg {s['avg']:.2f} ms, "
          f"p95 {s['p95']:.2f} ms, max {s['max']:.2f} ms)")
    if args.text:
        print()
        print(text_report(os.path.basename(args.capture), s, cols, layers,
                          budget_ms))


if __name__ == "__main__":
    main()
