#!/usr/bin/env python3
"""Off-road test course (ADR-0141, #41; Glenn: "could that offroader climb over rocks, steep hills? Could we
make an obstacle course for that vehicle to test 4wd?"). Writes assets/levels/offroad_course.json:

  start       the off-roader (part-time 4WD: Z engages it) and a street sedan, facing down the course (-z)
  hills       two ladders of ramps, each rising 6 m to a flat top and back down:
                grip  (friction 0.9)   15 20 25 30 35 degrees
                mud   (friction 0.35)  10 15 20 25 30 degrees, each behind a 10 m mud apron
              a white stop line before each: start there at rest to test traction, not momentum
  ledges      square steps 0.2 0.3 0.4 0.5 m high
  rock garden boulders standing 0.15-0.55 m (up to a wheel radius), seeded
  twister     humps set diagonally left/right: one wheel up while its opposite drops
  logs        four logs across the track, 0.25-0.40 m thick (under the off-roader's belly)
  mud flat    40 m at friction 0.2

Every obstacle is a static primitive with a collider (box / sphere / capsule). Deterministic.
  tools/offroad_course.py [out.json]
"""
import json, math, random, sys

rng = random.Random(41)
ents = []
eid = [0]


def ent(shape, size, pos, color, mu, axis=None, deg=0.0, rough=0.9, name=None):
    eid[0] += 1
    e = {"id": eid[0], "name": name or f"{shape}{eid[0]}", "shape": shape, "size": size,
         "position": pos, "material": {"albedo": color, "roughness": rough, "metallic": 0.0},
         "physics": {"motion": "static", "friction": mu, "restitution": 0.0}}
    if axis and abs(deg) > 1e-9:
        e["orientation"] = {"axis": axis, "angleDeg": deg}
    ents.append(e)


DIRT, MUD, ROCK, WOOD, PAD = [0.42, 0.33, 0.22], [0.22, 0.17, 0.12], [0.45, 0.44, 0.42], [0.36, 0.24, 0.13], [0.30, 0.32, 0.34]

# the ground: a big grippy dirt pad
ent("box", [220, 1, 320], [0, -0.5, -110], DIRT, 0.85, name="ground")
ent("box", [30, 0.02, 16], [0, 0.01, 22], PAD, 0.9, name="start_pad")


def hill(x, z0, deg, mu, color, tag):
    """A ramp rising 6 m along -z from z0, a 6 m flat top, and a ramp back down."""
    h = 6.0
    a = math.radians(deg)
    run = h / math.tan(a)
    L = h / math.sin(a)
    # up: a slab whose top surface climbs from (z0, 0) to (z0 - run, h)
    cz, cy = z0 - run * 0.5, h * 0.5
    t = 0.6   # slab thickness, pushed down so the top surface is the ramp
    ent("box", [6, t, L], [x, cy - t * 0.5 * math.cos(a), cz - t * 0.5 * math.sin(a)], color, mu,
        [1, 0, 0], deg, name=f"{tag}_up")
    top0 = z0 - run
    ent("box", [6, h, 6], [x, h * 0.5, top0 - 3], color, mu, name=f"{tag}_top")
    z1 = top0 - 6
    ent("box", [6, t, L], [x, cy - t * 0.5 * math.cos(a), z1 - run * 0.5 + t * 0.5 * math.sin(a)], color, mu,
        [1, 0, 0], -deg, name=f"{tag}_down")


# HILLS: the grip ladder on the left, the mud ladder on the right, starting at z = 0
for i, deg in enumerate([15, 20, 25, 30, 35]):
    hill(-60 + i * 9, 0, deg, 0.9, DIRT, f"grip{deg}")
for i, deg in enumerate([10, 15, 20, 25, 30]):
    hill(20 + i * 9, 0, deg, 0.35, MUD, f"mud{deg}")
    # a mud APRON before each mud hill: on dirt, a 2WD truck's driven rear wheels grip until it is most of
    # the way up, and it climbed 25 degrees of mud from a standing start -- a mud hill has a muddy approach
    ent("box", [6, 0.02, 10], [20 + i * 9, 0.01, 5.0], MUD, 0.35, name=f"mud{deg}_apron")
# STOP LINES at every hill's foot (the rear tyres on the apron): start there, at rest, to test traction -- a
# run-up tests momentum, and carried even 2WD over 30 degrees of mud
for x in [-60 + i * 9 for i in range(5)] + [20 + i * 9 for i in range(5)]:
    ent("box", [6, 0.03, 0.3], [x, 0.015, 3.2], [0.9, 0.9, 0.85], 0.35 if x > 0 else 0.85, name="stop_line")

# LEDGES at z = -40: square steps in lanes down the middle
for i, h in enumerate([0.2, 0.3, 0.4, 0.5]):
    ent("box", [5, h, 6], [-12 + i * 8, h * 0.5, -42], ROCK, 0.85, name=f"ledge_{int(h * 100)}cm")

# ROCK GARDEN z = -60 .. -95, x = -15 .. 15: boulders standing 0.15-0.55 m out of the dirt -- up to a wheel's
# radius, what a crawler climbs. (The first cut buried 0.3-0.9 m spheres only a little: 53 of 90 stood over
# 0.8 m, walls no truck climbs, and the off-roader stopped dead on one -- Glenn: "I get stuck".)
for _ in range(90):
    r = rng.uniform(0.35, 0.9)
    top = rng.uniform(0.15, 0.55)
    x, z = rng.uniform(-15, 15), rng.uniform(-95, -60)
    ent("sphere", [r, r, r], [round(x, 2), round(top - r, 3), round(z, 2)], ROCK, 0.8, name="boulder")

# AXLE TWISTER z = -105 .. -140: humps diagonal left / right
for i in range(8):
    side = 1 if i % 2 == 0 else -1
    ent("box", [3.0, 0.45, 7.0], [side * 1.6, 0.1, -108 - i * 4.2], DIRT, 0.9, [0, 1, 0], side * 35, name=f"twist{i}")

# LOGS z = -150 .. -165: capsules lying across the track, 0.25-0.40 m thick -- under the off-roader's 0.42 m
# belly. (The first cut ran to 0.66 m: Glenn stopped across the 0.56 m log, the body on it, the tyres unloaded.)
for i in range(4):
    r = 0.125 + 0.025 * i
    ent("capsule", [r, 7.0, r], [0, r, -150 - i * 5], WOOD, 0.7, [0, 0, 1], 90, name=f"log_{int(r * 100)}cm")

# MUD FLAT z = -175 .. -215
ent("box", [14, 0.02, 40], [0, 0.01, -195], MUD, 0.2, name="mud_flat")

level = {
    "version": 1,
    "environment": {"skyColor": [0.55, 0.68, 0.86]},
    "lighting": {"ambientMultiplier": 0.7, "exposure": 0.7,
                 "sun": {"castsShadow": True, "color": [1, 0.98, 0.94], "direction": [0.4, 0.72, -0.32], "intensity": 6.0}},
    "player": {"collider": {"halfHeight": 0.4, "radius": 0.3, "shape": "capsule"}, "friction": 0.5,
               "position": [0.0, 1.2, 26.0]},
    "vehicles": [
        {"recipe": "offroad", "seed": 1, "position": [4.0, 1.4, 22.0], "yaw": 180},
        {"recipe": "sedan", "seed": 2, "position": [-4.0, 1.0, 22.0], "yaw": 180},
    ],
    "entities": ents,
}
out = sys.argv[1] if len(sys.argv) > 1 else "assets/levels/offroad_course.json"
with open(out, "w") as f:
    json.dump(level, f, indent=1)
print(f"{out}: {len(ents)} entities")
