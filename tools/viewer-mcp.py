#!/usr/bin/env python3
"""MCP server for the viewer's control channel (ADR-0078, docs/control-channel.md).

Speaks Model Context Protocol (JSON-RPC 2.0, one message per line on stdio) on
one side and the viewer's unix-socket line protocol on the other, so a Claude
Code session can frame cameras, take screenshots, toggle overlays, drive the
sim clock, and reload the level in a RUNNING viewer — including one a human
opened by hand. Registered via the repo's .mcp.json.

Standard library only (no `mcp` package, no venv), so it runs anywhere the
repo clones — same rule as frame-report.py / code-health.py.

Manual smoke test (what the build session runs before first registration):
  printf '%s\n' \
    '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{}}' \
    '{"jsonrpc":"2.0","id":2,"method":"tools/list"}' \
    | python3 tools/viewer-mcp.py
"""
import glob
import json
import os
import socket
import subprocess
import sys
import time

SOCK_GLOB = "/tmp/raytracer-viewer-*.sock"
SHOT_TIMEOUT = 30.0   # seconds to wait for a dump file to land


# --- control-channel client --------------------------------------------------

def _live_sockets():
    """Live socket paths, newest last. Stale files (a killed viewer skips its
    unlink) are removed on sight so discovery stays truthful."""
    live = []
    for path in sorted(glob.glob(SOCK_GLOB), key=lambda p: os.path.getmtime(p)):
        try:
            with socket.socket(socket.AF_UNIX) as s:
                s.settimeout(2.0)
                s.connect(path)
            live.append(path)
        except OSError:
            try:
                os.unlink(path)
            except OSError:
                pass
    return live

def _resolve(instance):
    socks = _live_sockets()
    if not socks:
        raise RuntimeError("no running viewer (use launch_viewer)")
    if instance:
        for p in socks:
            if instance in p:
                return p
        raise RuntimeError(f"no viewer matching '{instance}' (live: {socks})")
    return socks[-1]   # newest

def send_command(line, instance=None):
    """One command, one reply, fresh connection (the channel serves the newest
    client, so short-lived connections compose with a human's tools)."""
    path = _resolve(instance)
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(10.0)
        s.connect(path)
        s.sendall((line + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                raise RuntimeError(f"viewer closed the connection ({path})")
            buf += chunk
    return buf.split(b"\n", 1)[0].decode()

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

def tool_set_camera(args):
    return send_command(
        "camera {x} {y} {z} {pitch} {yaw}".format(**args), args.get("instance"))

def tool_get_camera(args):
    return send_command("camera?", args.get("instance"))

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

def tool_overlay(args):
    on = "on" if args.get("on", True) else "off"
    return send_command(f"overlay {args['name']} {on}", args.get("instance"))

def tool_sim(args):
    cmd = f"sim {args['action']}"
    if "value" in args:
        cmd += f" {args['value']}"
    return send_command(cmd, args.get("instance"))

def tool_reload(args):
    return send_command("reload", args.get("instance"))

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


NUM = {"type": "number"}
STR = {"type": "string"}
CAM_PROPS = {"x": NUM, "y": NUM, "z": NUM, "pitch": NUM, "yaw": NUM,
             "instance": STR}
TOOLS = [
    ("viewer_status", "List running viewer instances (level, frame, state).",
     {}, [], tool_viewer_status),
    ("launch_viewer", "Launch ./build/viewer on a level, detached, with its "
     "log captured for planner_stats. Modes: edit (camera-friendly) or play.",
     {"level": STR, "mode": {"type": "string", "enum": ["edit", "play"]}},
     ["level"], tool_launch_viewer),
    ("set_camera", "Fly the camera to x,y,z with pitch/yaw in degrees "
     "(pitch -89 looks straight down; yaw 0 looks toward -Z). Detaches the "
     "camera so the pose holds in play mode too.",
     CAM_PROPS, ["x", "y", "z", "pitch", "yaw"], tool_set_camera),
    ("get_camera", "Current camera eye and angles.", {"instance": STR}, [],
     tool_get_camera),
    ("screenshot", "Write the next composited frame to a PNG and return its "
     "path (readable immediately). Optionally frame a camera pose first.",
     dict(CAM_PROPS, path=STR), [], tool_screenshot),
    ("overlay", "Toggle a panel/overlay: ui (ALL ImGui panels), hud, debug "
     "(the backtick overlay), and in PLAY mode the citysim layers "
     "master/agents/cones/nav/plan.",
     {"name": {"type": "string", "enum": ["ui", "hud", "debug", "master",
                                          "agents", "cones", "nav", "plan"]},
      "on": {"type": "boolean"}, "instance": STR},
     ["name"], tool_overlay),
    ("sim", "Drive the sim clock: pause, resume, step (one fixed step), or "
     "speed with a value (e.g. 0.25).",
     {"action": {"type": "string",
                 "enum": ["pause", "resume", "step", "speed"]},
      "value": NUM, "instance": STR}, ["action"], tool_sim),
    ("reload", "Rebuild the current level from its recipe (the same clean "
     "reset the editor-play loop uses).", {"instance": STR}, [], tool_reload),
    ("planner_stats", "Road-graph and city-lots numbers (COVER, rejections, "
     "corridor routing) scraped from the instance's captured log.",
     {"instance": STR}, [], tool_planner_stats),
]


# --- MCP over stdio ----------------------------------------------------------

def _tool_list():
    return [{"name": n, "description": d,
             "inputSchema": {"type": "object", "properties": p, "required": r}}
            for n, d, p, r, _ in TOOLS]

def _call(name, arguments):
    for n, _, _, _, fn in TOOLS:
        if n == name:
            return fn(arguments or {})
    raise RuntimeError(f"unknown tool {name}")

def main():
    out = sys.stdout
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
                      "capabilities": {"tools": {}},
                      "serverInfo": {"name": "viewer-control",
                                     "version": "1.0"}}
        elif method == "tools/list":
            result = {"tools": _tool_list()}
        elif method == "tools/call":
            params = msg.get("params", {})
            try:
                text = _call(params.get("name"), params.get("arguments"))
                result = {"content": [{"type": "text", "text": text}]}
            except Exception as e:   # tool errors are results, not RPC errors
                result = {"content": [{"type": "text", "text": f"error: {e}"}],
                          "isError": True}
        elif method == "ping":
            result = {}
        else:
            out.write(json.dumps(
                {"jsonrpc": "2.0", "id": mid,
                 "error": {"code": -32601,
                           "message": f"method not found: {method}"}}) + "\n")
            out.flush()
            continue
        out.write(json.dumps({"jsonrpc": "2.0", "id": mid,
                              "result": result}) + "\n")
        out.flush()

if __name__ == "__main__":
    main()
