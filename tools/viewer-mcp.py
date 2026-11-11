#!/usr/bin/env python3
"""MCP server for the viewer's control channel (ADR-0078, docs/control-channel.md).

Speaks Model Context Protocol (JSON-RPC 2.0, one message per line on stdio) on
one side and the viewer's unix-socket line protocol on the other, so a Claude
Code session can frame cameras, take screenshots, toggle overlays, drive the
sim clock, and reload the level in a RUNNING viewer — including one a human
opened by hand. Registered via the repo's .mcp.json.

The tool list is the viewer's own command table (ADR-0253): one tool per verb,
read from a live viewer's `commands? json`, or from web/engine/commands.json
(the copy a test holds equal to the built-in table) when none is running. Only
the composite tools — the ones that do more than send one line — are written
here. When a viewer grows a verb (a newer build, a Lua script) the list is
re-read and the client told (notifications/tools/list_changed).

Standard library only (no `mcp` package, no venv), so it runs anywhere the
repo clones — same rule as frame-report.py / code-health.py.

Manual smoke test (what the build session runs before first registration):
  printf '%s\n' \
    '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{}}' \
    '{"jsonrpc":"2.0","id":2,"method":"tools/list"}' \
    | python3 tools/viewer-mcp.py
"""
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtctl   # the control channel (ADR-0291)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SITE_TABLE = os.path.join(REPO, "web", "engine", "commands.json")
SHOT_TIMEOUT = 30.0   # seconds to wait for a dump file to land


# --- control-channel client --------------------------------------------------

def _live_sockets():
    """Live socket paths, newest last (rtctl's, ADR-0291)."""
    return [path for _, _, path in reversed(rtctl.live_sockets())]

def _resolve(instance):
    try:
        return rtctl.newest(match=instance or None)
    except RuntimeError:
        raise RuntimeError(f"no viewer matching '{instance}' (live: {_live_sockets()})" if instance
                           else "no running viewer (use launch_viewer)")

def send_command(line, instance=None, timeout=10.0):
    """One command, one reply, fresh connection (the channel serves the newest
    client, so short-lived connections compose with a human's tools)."""
    return rtctl.send(_resolve(instance), line, timeout)

def _log_for(sock_path):
    return sock_path.replace(".sock", ".log")


# --- tools -------------------------------------------------------------------

def tool_viewer_status(args):
    rows = []
    for path in _live_sockets():
        info = send_command("info", os.path.basename(path))
        rows.append(f"{os.path.basename(path)}: {info}")
    return "\n".join(rows) if rows else "no running viewers"

def tool_launch_viewer(args):
    level = args["level"]
    mode = args.get("mode", "edit")
    viewer = os.path.join(os.getcwd(), "build", "viewer")
    if not os.path.exists(viewer):
        raise RuntimeError(f"{viewer} not built (cmake --build build)")
    proc = subprocess.Popen(
        [viewer, level, f"--{mode}"],
        stdout=open(f"/tmp/raytracer-viewer-pending.log", "wb"),
        stderr=subprocess.STDOUT,
        start_new_session=True)   # survives this server's exit
    # The pid names both the socket and the log; rename once we know it.
    log = f"/tmp/raytracer-viewer-{proc.pid}.log"
    os.replace("/tmp/raytracer-viewer-pending.log", log)
    sock = f"/tmp/raytracer-viewer-{proc.pid}.sock"
    deadline = time.time() + 30.0
    while time.time() < deadline:
        if os.path.exists(sock):
            return (f"launched pid {proc.pid} ({level}, --{mode}); socket up. "
                    f"NOTE: the level may still be baking — the first "
                    f"screenshot can take a minute on a fresh cache.")
        if proc.poll() is not None:
            raise RuntimeError(f"viewer exited early; see {log}")
        time.sleep(0.25)
    raise RuntimeError(f"socket never appeared; see {log}")

def tool_screenshot(args):
    inst = args.get("instance")
    if all(k in args for k in ("x", "y", "z", "pitch", "yaw")):
        send_command("camera {x} {y} {z} {pitch} {yaw}".format(**args), inst)
    path = args.get("path") or f"/tmp/viewer-shot-{int(time.time())}.png"
    reply = send_command(f"shot {path}", inst)
    if not reply.startswith("ok"):
        raise RuntimeError(reply)
    deadline = time.time() + SHOT_TIMEOUT
    while time.time() < deadline:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return f"wrote {path}"
        time.sleep(0.25)
    raise RuntimeError(f"dump never landed at {path} (still baking?)")

def tool_teleport(args):
    r = send_command(f"teleport {args['pose']}", args.get("instance"))
    if "staged" not in r: return r
    time.sleep(0.4)
    return send_command("teleport?", args.get("instance"))

def tool_render_params(args):
    # Convenience: write any given ssao./ssr./shadow./bloom./tonemap./grade.
    # keys, then apply them through the engine's settings->renderer mapping.
    inst = args.get("instance")
    for k, v in (args.get("params") or {}).items():
        send_command(f"set {k} {v}", inst)
    return send_command("render apply", inst)

def tool_viewer_command(args):
    # Escape hatch: a raw protocol line, for a whole line typed as the
    # protocol has it (docs/control-channel.md).
    return send_command(args["command"], args.get("instance"))

def tool_planner_stats(args):
    path = _resolve(args.get("instance"))
    info = send_command("info", args.get("instance"))
    log = _log_for(path)
    if not os.path.exists(log):
        return (f"{info}\n(no captured log for this instance — launched "
                f"outside launch_viewer; stats lines unavailable)")
    keep = ("[roadgraph]", "[citylots]", "[corridor]", "[grade]", "COVER")
    lines = [l.rstrip() for l in open(log, errors="replace")
             if any(k in l for k in keep)]
    tail = "\n".join(lines[-30:]) if lines else "(no planner lines yet)"
    return f"{info}\n{tail}"


# --- field reports (the game's F9, engine/field_report.h) ----------------------

def _reports_folder():
    """report.folder from the viewer's settings.json, else <screenshot folder>/reports (~/Pictures/Raytracer)."""
    cfg = {}
    try:
        with open(os.path.join(REPO, "settings.json")) as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        pass
    if cfg.get("report.folder"):
        return os.path.expanduser(cfg["report.folder"])
    shots = cfg.get("screenshot.folder") or ""
    if not shots:
        pictures = os.environ.get("XDG_PICTURES_DIR") or os.path.expanduser("~/Pictures")
        shots = os.path.join(pictures, "Raytracer")
    return os.path.join(os.path.expanduser(shots), "reports")

def _report_dirs():
    base = _reports_folder()
    if not os.path.isdir(base):
        return base, []
    dirs = [d for d in os.listdir(base) if os.path.exists(os.path.join(base, d, "report.json"))]
    return base, sorted(dirs, reverse=True)

def tool_reports(args):
    base, dirs = _report_dirs()
    rid = args.get("id")
    if not rid:
        if not dirs:
            return f"no reports in {base}"
        lines = [f"{len(dirs)} report(s) in {base}, newest first:"]
        for d in dirs[:int(args.get("count") or 10)]:
            try:
                with open(os.path.join(base, d, "report.json")) as f:
                    r = json.load(f)
            except (OSError, ValueError):
                continue
            issue = os.path.join(base, d, "issue.txt")
            filed = open(issue).read().strip() if os.path.exists(issue) else "not filed"
            lines.append(f"- {d}: {r.get('title', '')} [{os.path.basename(r.get('level', ''))}] ({filed})")
        return "\n".join(lines)
    match = [d for d in dirs if d.startswith(rid)] if rid != "latest" else dirs[:1]
    if not match:
        return f"no report {rid} in {base}"
    folder = os.path.join(base, match[0])
    with open(os.path.join(folder, "note.md")) as f:
        note = f.read()
    with open(os.path.join(folder, "report.json")) as f:
        facts = json.load(f).get("facts", {})
    shot = os.path.join(folder, "shot.png")
    out = [f"folder: {folder}", f"shot: {shot if os.path.exists(shot) else '(none)'}", "", note, "facts (raw):"]
    out += [f"  {k}: {v}" for k, v in facts.items()]
    return "\n".join(out)

def tool_file_report(args):
    base, dirs = _report_dirs()
    rid = args["id"]
    match = [d for d in dirs if d.startswith(rid)] if rid != "latest" else dirs[:1]
    if not match:
        return f"no report {rid} in {base}"
    r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "file_report.py"), os.path.join(base, match[0])],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.strip() or r.stdout.strip())
    return r.stdout.strip()

NUM = {"type": "number"}
STR = {"type": "string"}
CAM_PROPS = {"x": NUM, "y": NUM, "z": NUM, "pitch": NUM, "yaw": NUM,
             "instance": STR}
# The composite tools: each does more than send one line (launches a process,
# waits for a file, reads a log, sends several lines). Every other tool comes
# from the command table.
COMPOSITES = [
    ("viewer_status", "List running viewer instances (level, frame, state).",
     {}, [], tool_viewer_status),
    ("launch_viewer", "Launch ./build/viewer on a level, detached, with its "
     "log captured for planner_stats. Modes: edit (camera-friendly) or play.",
     {"level": STR, "mode": {"type": "string", "enum": ["edit", "play"]}},
     ["level"], tool_launch_viewer),
    ("screenshot", "Write the next composited frame to a PNG and return its "
     "path once it has landed (readable immediately). Optionally frame a "
     "camera pose first (pitch -89 looks straight down; yaw 0 looks toward "
     "-Z).", dict(CAM_PROPS, path=STR), [], tool_screenshot),
    ("teleport", "Move the PLAYER (or the camera when there is no player) to a "
     "pose: 'x y z [pitch yaw]' or 'x z' (on the ground), and return where it "
     "landed. The same format the Debug > Teleport panel copies and pastes, "
     "and `where?` prints.", {"pose": STR, "instance": STR}, ["pose"],
     tool_teleport),
    ("planner_stats", "Road-graph and city-lots numbers (COVER, rejections, "
     "corridor routing) scraped from the instance's captured log.",
     {"instance": STR}, [], tool_planner_stats),
    ("render_params", "Set renderer post/quality knobs and apply them live: "
     "pass {\"ssao.radius\": 1.2, \"grade.saturation\": 1.1, ...} — the "
     "families the debug panel owns (ssao.*, ssr.*, shadow.*, bloom.*, "
     "tonemap.op, grade.*, hud.show).",
     {"params": {"type": "object"}, "instance": STR}, [], tool_render_params),
    ("reports", "Field reports Glenn made in the game (F9): with no id, the newest (title, level, whether filed); "
     "with an id (a prefix, or 'latest'), its note, teleport line, every fact and the screenshot's path (read "
     "the PNG to see it). Works with no viewer running.",
     {"id": STR, "count": NUM}, [], tool_reports),
    ("file_report", "File a field report as a GitHub issue (tools/file_report.py: the shot pushed to the "
     "issue-shots branch, the note as the body). Public: only when Glenn asks.",
     {"id": STR}, ["id"], tool_file_report),
    ("viewer_command", "Raw control-channel line (see docs/control-channel.md),"
     " sent as typed.", {"command": STR, "instance": STR}, ["command"],
     tool_viewer_command),
]
# Verbs a composite already covers, so they get no tool of their own.
COVERED = {"shot", "teleport"}


# --- the command table -------------------------------------------------------

def _verb_table():
    """The viewer's verbs as [{name, usage, summary, owner}]: the newest live
    viewer's table, or the site's copy when no viewer runs (or it predates
    `commands?`)."""
    try:
        # Short wait: a viewer busy loading answers late, and the site's
        # copy is right for any build at or after it.
        reply = send_command("commands? json", timeout=2.0)
        if reply.startswith("ok "):
            return json.loads(reply[3:])
    except (RuntimeError, OSError, ValueError):
        pass
    with open(SITE_TABLE) as f:
        return json.load(f)

def _tool_name(verb):
    # MCP names are [a-zA-Z0-9_-]; a question's '?' becomes '_q', as on the
    # site's verbs page (camera? -> camera_q).
    return verb.replace("?", "_q")

def _verb_tool(entry):
    verb = entry["name"]
    def run(args):
        line = verb
        if args.get("args"):
            line += " " + str(args["args"]).strip()
        return send_command(line, args.get("instance"))
    desc = f"{entry['summary']} Usage: `{entry['usage']}`."
    if entry.get("owner", "engine") != "engine":
        desc += f" (Registered by {entry['owner']}.)"
    return (_tool_name(verb), desc,
            {"args": {"type": "string",
                      "description": "Everything after the verb, as in the usage."},
             "instance": STR}, [], run, verb.endswith("?"))

def _tools():
    """Composites first, then one tool per verb in the table."""
    tools = [t + (False,) for t in COMPOSITES]
    taken = {t[0] for t in tools}
    for entry in _verb_table():
        if entry["name"] in COVERED or _tool_name(entry["name"]) in taken:
            continue
        tools.append(_verb_tool(entry))
    return tools


# --- MCP over stdio ----------------------------------------------------------

# The calls after which the table is read again (a new viewer or build, a
# reloaded level), so each other call stays one round trip.
RELISTS = {"launch_viewer", "viewer_status", "reload"}

INSTRUCTIONS = (
    "Drives a running raytracer viewer through its control channel. Every "
    "viewer verb is a tool (questions end in _q: camera_q, where_q); pass "
    "the rest of the line as `args`. Every tool takes an optional `instance` "
    "(a socket basename substring); the default is the newest live viewer. "
    "Camera: pitch -89 is straight down, yaw 0 looks toward -Z. Verbs that "
    "stage work (teleport, ride, possess) answer with their `?` twin.")

def _tool_list(tools):
    out = []
    for n, d, p, r, _, question in tools:
        t = {"name": n, "description": d,
             "inputSchema": {"type": "object", "properties": p, "required": r}}
        if question:
            t["annotations"] = {"readOnlyHint": True}
        out.append(t)
    return out

def _call(tools, name, arguments):
    for n, _, _, _, fn, _ in tools:
        if n == name:
            return fn(arguments or {})
    raise RuntimeError(f"unknown tool {name}")

def _send(out, msg):
    out.write(json.dumps(msg) + "\n")
    out.flush()

def main():
    out = sys.stdout
    tools = _tools()
    for raw in sys.stdin:
        raw = raw.strip()
        if not raw:
            continue
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            continue
        mid = msg.get("id")
        method = msg.get("method", "")
        if mid is None:   # notification — nothing to answer
            continue
        if method == "initialize":
            result = {"protocolVersion":
                      msg.get("params", {}).get("protocolVersion",
                                                "2024-11-05"),
                      "capabilities": {"tools": {"listChanged": True}},
                      "serverInfo": {"name": "viewer-control",
                                     "version": "2.0"},
                      "instructions": INSTRUCTIONS}
        elif method == "tools/list":
            tools = _tools()
            result = {"tools": _tool_list(tools)}
        elif method == "tools/call":
            params = msg.get("params", {})
            try:
                text = _call(tools, params.get("name"), params.get("arguments"))
                result = {"content": [{"type": "text", "text": text}]}
            except Exception as e:   # tool errors are results, not RPC errors
                result = {"content": [{"type": "text", "text": f"error: {e}"}],
                          "isError": True}
        elif method == "ping":
            result = {}
        else:
            _send(out, {"jsonrpc": "2.0", "id": mid,
                        "error": {"code": -32601,
                                  "message": f"method not found: {method}"}})
            continue
        _send(out, {"jsonrpc": "2.0", "id": mid, "result": result})
        if method == "tools/call" and params.get("name") in RELISTS:
            # A launch or a reload can change the table; when it did, tell
            # the client to list again.
            fresh = _tools()
            if [t[0] for t in fresh] != [t[0] for t in tools]:
                tools = fresh
                _send(out, {"jsonrpc": "2.0",
                            "method": "notifications/tools/list_changed"})

if __name__ == "__main__":
    main()
