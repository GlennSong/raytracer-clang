#!/usr/bin/env python3
"""rtctl: the one client for a viewer's control channel (docs/control-channel.md, ADR-0078; ADR-0291).

Every tool that talks to a viewer imports this -- viewer_ctl, viewer-mcp, citywalk, the shot and probe tools --
instead of carrying its own socket code:

    import rtctl
    proc, sock = rtctl.launch("build-viewer/viewer", "assets/levels/river_town.json", log=open("v.log", "wb"))
    print(rtctl.send(sock, "camera?", timeout=900))    # the first reply waits out the level load
    rtctl.stop(proc)

    sock = rtctl.newest()                                # or talk to a viewer already running

A viewer listens at /tmp/raytracer-viewer-<pid>.sock; one command line in, one reply line out, a fresh
connection each (the channel serves the newest client, so short connections compose with a human's tools).
"""
import glob
import os
import socket
import subprocess
import time

SOCK_GLOB = "/tmp/raytracer-viewer-*.sock"


def sock_path(pid):
    return f"/tmp/raytracer-viewer-{pid}.sock"


def live_sockets():
    """(mtime, pid, path) of every viewer still running, newest first. A killed viewer can leave its socket file
    behind; its pid is gone, so it is skipped (and left alone: it may be another user's)."""
    out = []
    for path in glob.glob(SOCK_GLOB):
        try:
            pid = int(path.rsplit("-", 1)[1].split(".")[0])
            os.kill(pid, 0)
        except (ValueError, ProcessLookupError, PermissionError):
            continue
        out.append((os.path.getmtime(path), pid, path))
    return sorted(out, reverse=True)


def newest(pid=None, match=None):
    """The socket of the newest live viewer -- or of `pid`, or the newest whose path contains `match`."""
    for _, p, path in live_sockets():
        if (pid is None or p == pid) and (match is None or match in path):
            return path
    raise RuntimeError("no running viewer found (looked for %s)" % SOCK_GLOB)


def send(path, line, timeout=180.0):
    """Send one command; return its reply line (without the newline)."""
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(timeout)
        s.connect(path)
        s.sendall((line.strip() + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                if buf:
                    break
                raise RuntimeError(f"viewer closed the connection ({path})")
            buf += chunk
    return buf.split(b"\n", 1)[0].decode().strip()


def launch(viewer, level, *args, log=None, env=None, wait=60.0, play=True):
    """Start a viewer on `level` (in --play unless `play` is False) in its own session; return (proc, socket path)
    once the socket exists. The level may still be loading: the first send waits for it."""
    cmd = [viewer, level] + (["--play"] if play else []) + list(args)
    proc = subprocess.Popen(cmd, stdout=log or subprocess.DEVNULL, stderr=subprocess.STDOUT, env=env,
                            start_new_session=True)
    path = sock_path(proc.pid)
    t0 = time.time()
    while not os.path.exists(path):
        if proc.poll() is not None:
            raise RuntimeError(f"viewer exited ({proc.returncode}) before opening its control socket")
        if time.time() - t0 > wait:
            stop(proc)
            raise RuntimeError("viewer did not open its control socket")
        time.sleep(0.1)
    return proc, path


def stop(proc, grace=5.0):
    """Stop a viewer this tool started (by its own process, never by name)."""
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(grace)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
