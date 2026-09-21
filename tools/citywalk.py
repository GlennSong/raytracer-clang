#!/usr/bin/env python3
"""Puppet one agent around the city through the viewer's control channel.

The director half of ADR-0091, phase 1: possess a pedestrian, pick somewhere to
go, walk there, notice arrival, decide again. The DECIDER is pluggable — this
phase ships a stub (nearest unvisited candidate) so the loop, the arrival
detection and the logging can be proved without any model in the way.

  citywalk.py --steps 5 --dry-run        # print what it would do, touch nothing
  citywalk.py --steps 5                  # actually walk
  citywalk.py --socket /tmp/raytracer-viewer-123.sock --radius 80

Every decision is appended to citywalk-log.jsonl: the state, the menu offered,
the choice, and what happened. That log is the replay trace and, later, the
training set (ADR-0091).

Standard library only, same rule as viewer-mcp.py / frame-report.py.
"""
import argparse
import glob
import json
import math
import os
import random
import socket
import time

SOCK_GLOB = "/tmp/raytracer-viewer-*.sock"
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "citywalk-log.jsonl")


class Channel:
    """One command line in, one reply line out (ADR-0078)."""

    def __init__(self, path=None, timeout=4.0):
        self.path = path or self.discover()
        self.timeout = timeout
        if not self.path:
            raise SystemExit("no live viewer socket found (is the viewer/editor running?)")

    @staticmethod
    def discover():
        """Newest socket that actually answers. Stale files outlive killed viewers."""
        for p in sorted(glob.glob(SOCK_GLOB), key=os.path.getmtime, reverse=True):
            try:
                s = socket.socket(socket.AF_UNIX)
                s.settimeout(1.5)
                s.connect(p)
                s.sendall(b"ping\n")
                if s.recv(64).startswith(b"ok"):
                    return p
            except OSError:
                continue
            finally:
                try:
                    s.close()
                except Exception:
                    pass
        return None

    def send(self, line):
        s = socket.socket(socket.AF_UNIX)
        s.settimeout(self.timeout)
        s.connect(self.path)
        try:
            s.sendall((line + "\n").encode())
            return s.recv(8192).decode().strip()
        finally:
            s.close()

    def get(self, key):
        r = self.send(f"get {key}")
        return r[3:].strip() if r.startswith("ok ") else ""

    def possess(self, what="walker"):
        return self.send(f"possess {what}")

    def status(self):
        """`possess?` -> state + pos/speed/remaining (ADR-0079). The settings key
        possess.status is the INTERNAL channel; the verb is what a director uses."""
        r = self.send("possess?")
        return r[3:].strip() if r.startswith("ok ") else r

    def walk_to(self, x, z):
        return self.send(f"walk_to {x} {z}")

    def release(self):
        return self.send("release")

    def shot(self, path):
        """`shot` arms the capture; the file lands on the next frame."""
        return self.send(f"shot {os.path.abspath(path)}")

    def set(self, key, value):
        return self.send(f"set {key} {value}")


def parse_status(text):
    """`possess?` replies e.g.
       walker state=walking pos=-227.3,7.9,-876.9 speed=1.3 remaining=41.2
    `pos` is ONE field holding x,y,z — do not split the line on commas."""
    out = {"raw": text}
    if not text or text == "none":
        return out
    for tok in text.split():
        if "=" not in tok:
            out.setdefault("kind", tok)
            continue
        k, v = tok.split("=", 1)
        if "," in v:
            nums = []
            for part in v.split(","):
                try:
                    nums.append(float(part))
                except ValueError:
                    nums.append(part)
            out[k] = nums
        else:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def position(status):
    """World x, z of the possessed avatar (pos is x,y,z)."""
    pos = status.get("pos")
    if isinstance(pos, list) and len(pos) >= 3:
        try:
            return float(pos[0]), float(pos[2])
        except (TypeError, ValueError):
            return None
    for kx, kz in (("x", "z"), ("px", "pz")):
        if kx in status and kz in status:
            try:
                return float(status[kx]), float(status[kz])
            except (TypeError, ValueError):
                pass
    return None


def candidates(pos, radius, rng, n=6, visited=()):
    """Phase 1 menu: points on a ring around the agent, skipping ones already
    used. Phase 3 replaces this with real destinations (shopfronts, stops,
    crossings) pulled from the generated map."""
    x, z = pos
    out = []
    for i in range(n):
        a = (i / n) * math.tau + rng.uniform(-0.2, 0.2)
        r = radius * rng.uniform(0.7, 1.3)
        p = (round(x + math.cos(a) * r, 1), round(z + math.sin(a) * r, 1))
        if all(math.dist(p, v) > radius * 0.5 for v in visited):
            out.append({"label": f"{'NESW'[i % 4]} {int(r)} m", "x": p[0], "z": p[1]})
    return out


def stub_decider(state, menu):
    """Phase 1 stand-in for the model: the nearest option, with a nudge toward
    variety so it does not oscillate between two points."""
    here = (state["x"], state["z"])
    ranked = sorted(menu, key=lambda c: math.dist(here, (c["x"], c["z"])))
    pick = ranked[0] if len(ranked) < 3 else ranked[min(1, len(ranked) - 1)]
    return menu.index(pick), "nearest-unvisited (stub)"


def log(entry):
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket")
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--radius", type=float, default=60.0, help="how far a leg is, metres")
    ap.add_argument("--arrive", type=float, default=6.0, help="metres that counts as arrived")
    ap.add_argument("--timeout", type=float, default=45.0, help="seconds before giving up on a leg")
    ap.add_argument("--dry-run", action="store_true", help="decide and print, send nothing")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--keep", action="store_true", help="stay possessed at the end")
    ap.add_argument("--shots", help="directory for a screenshot per leg")
    a = ap.parse_args()
    rng = random.Random(a.seed)

    ch = Channel(a.socket)
    info = ch.send("info")
    print(f"viewer: {info[:120]}\nsocket: {os.path.basename(ch.path)}")

    before = parse_status(ch.status())
    print(f"possess.status before: {before['raw']!r}")
    if a.dry_run:
        print("[dry-run] would send: set possess.cmd walker")
    else:
        print("possess walker ->", ch.possess("walker"))
        time.sleep(1.0)

    status = parse_status(ch.status())
    pos = position(status)
    print(f"possess.status after: {status['raw']!r}  parsed position: {pos}")
    if pos is None and not a.dry_run:
        print("no position in status — cannot navigate; releasing")
        ch.release()
        return
    if pos is None:
        pos = (0.0, 0.0)

    visited = [pos]
    for step in range(a.steps):
        state = {"step": step, "x": pos[0], "z": pos[1], "visited": len(visited),
                 "status": status["raw"]}
        menu = candidates(pos, a.radius, rng, visited=visited)
        if not menu:
            print("no candidates left"); break
        idx, why = stub_decider(state, menu)
        choice = menu[idx]
        print(f"\nstep {step}: at ({pos[0]:.1f}, {pos[1]:.1f}) -> {choice['label']} "
              f"({choice['x']}, {choice['z']})  [{why}]")
        entry = {"t": time.time(), "step": step, "state": state, "menu": menu,
                 "choice": idx, "why": why, "decider": "stub"}
        if a.dry_run:
            entry["result"] = "dry-run"
            log(entry)
            pos = (choice["x"], choice["z"])
            visited.append(pos)
            continue

        print("  walk_to ->", ch.walk_to(choice["x"], choice["z"]))
        t0, arrived, last = time.time(), False, None
        while time.time() - t0 < a.timeout:
            time.sleep(1.0)
            status = parse_status(ch.status())
            p = position(status)
            if p is None:
                continue
            last = p
            d = math.dist(p, (choice["x"], choice["z"]))
            if d <= a.arrive:
                arrived = True
                break
        pos = last or pos
        visited.append(pos)
        if a.shots:
            os.makedirs(a.shots, exist_ok=True)
            shot = os.path.join(a.shots, f"leg{step:02d}.png")
            ch.shot(shot)
            for _ in range(20):          # the file lands a frame or two later
                time.sleep(0.5)
                if os.path.exists(shot) and os.path.getsize(shot) > 0:
                    break
            entry["shot"] = shot
        entry["result"] = {"arrived": arrived, "seconds": round(time.time() - t0, 1),
                           "ended_at": pos, "status": status["raw"]}
        log(entry)
        print(f"  {'arrived' if arrived else 'gave up'} after "
              f"{entry['result']['seconds']}s at {pos}")

    if not a.dry_run and not a.keep:
        print("\nrelease ->", ch.release())
    print(f"log: {os.path.abspath(LOG)}")


if __name__ == "__main__":
    main()
