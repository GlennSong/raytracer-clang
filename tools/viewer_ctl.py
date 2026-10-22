#!/usr/bin/env python3
"""Send commands to a running viewer's control channel (docs/control-channel.md).

    tools/viewer_ctl.py "daynight run" "teleport -3960 -790"   # each argument is one command
    tools/viewer_ctl.py                                        # interactive: type commands, Ctrl-D to quit
    tools/viewer_ctl.py --pid 12345 "camera?"                  # a particular viewer when several are open

Every viewer listens at /tmp/raytracer-viewer-<pid>.sock; with no --pid this talks to the newest live one.
"""
import glob
import os
import socket
import sys


def live_sockets():
    out = []
    for path in glob.glob("/tmp/raytracer-viewer-*.sock"):
        try:
            pid = int(path.rsplit("-", 1)[1].split(".")[0])
            os.kill(pid, 0)   # still running?
        except (ValueError, ProcessLookupError, PermissionError):
            continue
        out.append((os.path.getmtime(path), pid, path))
    return sorted(out, reverse=True)


def send(path, cmd, timeout=60):
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(timeout)
    s.connect(path)
    s.sendall((cmd.strip() + "\n").encode())
    reply = b""
    while not reply.endswith(b"\n"):
        chunk = s.recv(65536)
        if not chunk:
            break
        reply += chunk
    s.close()
    return reply.decode().strip()


def main(argv):
    pid = None
    if len(argv) >= 2 and argv[0] == "--pid":
        pid, argv = int(argv[1]), argv[2:]
    socks = live_sockets()
    if pid is not None:
        socks = [x for x in socks if x[1] == pid]
    if not socks:
        sys.exit("no running viewer found (looked for /tmp/raytracer-viewer-*.sock)")
    path = socks[0][2]
    if len(socks) > 1 and pid is None:
        print(f"(several viewers open; talking to pid {socks[0][1]} -- use --pid to pick)", file=sys.stderr)
    if argv:
        for cmd in argv:
            print(f"{cmd} -> {send(path, cmd)}")
        return
    print(f"viewer {socks[0][1]}: type commands (camera?, daynight run, teleport x z, ... -- an unknown one lists them all); Ctrl-D to quit")
    for line in sys.stdin:
        if line.strip():
            print(send(path, line))


if __name__ == "__main__":
    main(sys.argv[1:])
