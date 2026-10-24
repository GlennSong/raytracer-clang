#!/usr/bin/env python3
"""Writes assets/levels/movement_lab.json: the player-movement tuning course (engine/locomotion.h).

    python3 tools/movement_lab.py          # then: ./build-viewer/viewer assets/levels/movement_lab.json

Glenn, 2026-10-24: "some kind of player movement lab where we have different lengths and heights to jump on,
ledges to climb up and walls to climb ... Also a pool for swimming ... like the arena, not very big". You start in
the middle facing north. Every station is laid out by the numbers below, so a change here is a re-run, not a hand
edit of the JSON.

  NORTH   ledges: a row of blocks 0.3 .. 3.0 m high, coloured by how you get up them
            white  = a step (the controller walks it)      green = pull up from standing (jump at it)
            yellow = jump, then grab and pull up            red   = out of reach: climb it
  FAR N   gaps: run-ups at 1 m with landings 1 .. 5 m away
  WEST    climbing walls: 4, 8 and 15 m; one with ledges every 1.2 m; a 70 degree slab; a slight overhang;
          an outside corner and an inside corner
  SOUTH   slopes 30 .. 70 degrees (walkable to 50), a V between two slabs, rocks with a gap between them
  EAST    the pool: 14 x 10 m, 2.6 m deep, steps down at the west end, diving boards 3 and 6 m up its east end
  NE      a stair of ledges up to a 6 m roof, and rooftop gaps
  SE      the jump tower: a spiral ramp (walkable, 24 deg) up to a 72 m top -- 5 the parachute, 6 the wingsuit,
          then jump off and jump again in the air
  S       the playground: a swing set (E on a seat; W/S pump) and monkey bars (E under them; W/S along)
  FAR W   the skate run: a 4 m deck with a 40 m ramp down off it (6 deg) onto the flat, and two kickers --
          8 the skateboard, 9 the scooter
  FAR S   the swing park: gantries (two 18 m pillars, a beam across) every 14 m heading south -- 7 the grapple,
          hold the click on a beam, let go at the bottom of the swing, hook the next
"""
import json
import math
import os

ENTITIES = []


def box(pos, size, color, rough=0.75, rot=None, checker=False):
    e = {
        "id": len(ENTITIES) + 1,
        "material": {"albedo": list(color), "checkerboard": checker, "emission": [0, 0, 0], "metallic": 0.0,
                     "opacity": 1.0, "roughness": rough},
        "physics": {"friction": 0.6, "motion": "static", "restitution": 0.0},
        "position": [round(v, 4) for v in pos],
        "shape": "box",
        "size": [round(v, 4) for v in size],
    }
    if rot:
        e["orientation"] = {"angleDeg": rot[0], "axis": list(rot[1])}
    ENTITIES.append(e)


def block(x0, x1, z0, z1, y0, y1, color, **kw):
    """An axis-aligned block by its extents."""
    box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color, **kw)


WHITE, GREEN, YELLOW, RED = (0.85, 0.85, 0.82), (0.3, 0.65, 0.3), (0.9, 0.75, 0.2), (0.8, 0.25, 0.2)
CONCRETE, ROCK, TILE = (0.55, 0.55, 0.58), (0.45, 0.42, 0.38), (0.75, 0.82, 0.86)


def reach_colour(h):
    if h <= 0.55:
        return WHITE    # the step-up (player stepHeight 0.55)
    if h <= 1.35:
        return GREEN    # MovementTuning::groundMantleMax (1.45)
    if h <= 3.0:
        return YELLOW   # jump apex + mantleMaxHeight
    return RED


# --- the ground, with the pool cut out of it ----------------------------------
G = 300.0   # room to land a wingsuit
POOL = (18.0, 32.0, -5.0, 5.0)   # x0, x1, z0, z1
POOL_DEPTH = 2.6
px0, px1, pz0, pz1 = POOL
for (x0, x1, z0, z1) in [(-G, px0, -G, G), (px1, G, -G, G), (px0, px1, -G, pz0), (px0, px1, pz1, G)]:
    block(x0, x1, z0, z1, -3.0, 0.0, (0.7, 0.7, 0.7), rough=0.8, checker=True)
block(px0, px1, pz0, pz1, -3.0, -POOL_DEPTH, TILE, rough=0.3)                 # the pool floor
# a tiled lining: the ground's checker is projected from above and streaks down a wall (Glenn: "a mess of
# z-fighting"), so the walls are their own blocks, a coping lip just proud of the deck
L = 0.12
for (x0, x1, z0, z1) in [(px0, px0 + L, pz0, pz1), (px1 - L, px1, pz0, pz1),
                         (px0 + L, px1 - L, pz0, pz0 + L), (px0 + L, px1 - L, pz1 - L, pz1)]:
    block(x0, x1, z0, z1, -POOL_DEPTH, 0.04, TILE, rough=0.3)
for i, top in enumerate([-0.5, -1.0, -1.5, -2.0]):                            # steps down at the shallow end
    block(px0 + L + i * 0.8, px0 + L + (i + 1) * 0.8, pz0 + 3.0, pz1 - 3.0, -POOL_DEPTH, top, TILE, rough=0.3)
# diving boards over the deep end: a column you can climb, boards at 3 and 6 m
block(px1 + 0.5, px1 + 2.5, -1.0, 1.0, 0.0, 6.0, CONCRETE)
block(px1 - 3.0, px1 + 0.5, -0.6, 0.6, 2.8, 3.0, WHITE)
block(px1 - 2.0, px1 + 2.5, -0.6, 0.6, 5.8, 6.0, WHITE)

# --- north: ledges 0.3 .. 3.0 m -------------------------------------------------
heights = [0.3, 0.5, 0.7, 0.9, 1.1, 1.35, 1.6, 1.9, 2.2, 2.6, 3.0, 3.5]
for i, h in enumerate(heights):
    x = -14.0 + i * 2.6
    block(x - 0.8, x + 0.8, -12.0, -10.0, 0.0, h, reach_colour(h))

# --- far north: gaps from 1 m run-ups ----------------------------------------------
for i, gap in enumerate([1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0]):
    x = -12.0 + i * 3.2
    block(x - 1.0, x + 1.0, -22.0, -16.0, 0.0, 1.0, CONCRETE)                       # run-up
    block(x - 1.0, x + 1.0, -22.0 - gap - 3.0, -22.0 - gap, 0.0, 1.0,
          GREEN if gap <= 3.0 else YELLOW if gap <= 4.0 else RED)                   # landing

# --- west: climbing walls, faces toward +x ---------------------------------------
WX = -24.0
for i, h in enumerate([4.0, 8.0, 15.0]):
    z = -14.0 + i * 5.0
    block(WX - 1.0, WX, z - 2.0, z + 2.0, 0.0, h, ROCK, rough=0.9)
# a wall with ledges every 1.2 m (shelves 0.3 m deep you can stand a hand on)
z = 2.0
block(WX - 1.0, WX, z - 2.0, z + 2.0, 0.0, 9.6, CONCRETE)
for k in range(1, 8):
    block(WX, WX + 0.3, z - 2.0, z + 2.0, k * 1.2 - 0.15, k * 1.2, WHITE)
# a 70 degree slab (too steep to walk, a scramble to climb)
box((WX - 0.4, 3.6, 8.0), (8.0, 0.8, 4.0), ROCK, rough=0.9, rot=(-70.0, (0, 0, 1)))   # top face to +x
# a slight overhang: 10 degrees past vertical
box((WX - 0.2, 4.0, 13.0), (1.0, 8.4, 4.0), ROCK, rough=0.9, rot=(-10.0, (0, 0, 1)))
# an outside corner and an inside corner (8 m)
block(WX - 4.0, WX, 18.0, 22.0, 0.0, 8.0, ROCK, rough=0.9)                  # outside: round its NE edge
block(WX - 1.0, WX, 26.0, 32.0, 0.0, 8.0, ROCK, rough=0.9)                  # inside: two walls at a right angle
block(WX - 1.0, WX + 5.0, 32.0, 33.0, 0.0, 8.0, ROCK, rough=0.9)

# --- south: slopes, a V, rocks with a gap --------------------------------------------
for i, deg in enumerate([30, 40, 50, 55, 60, 70]):
    x = -12.0 + i * 3.4
    a = math.radians(deg)
    length = 6.0
    # a slab rising toward +z from the ground at z = 12
    cy = math.sin(a) * length / 2
    cz = 12.0 + math.cos(a) * length / 2
    box((x, cy, cz), (2.6, 0.4, length), GREEN if deg <= 50 else RED, rough=0.9, rot=(-deg, (1, 0, 0)))
# a V between two 55 degree slabs
for side in (-1, 1):
    a = math.radians(55)
    box((10.0 + side * math.cos(a) * 1.6, math.sin(a) * 1.6, 15.0), (3.2, 0.3, 4.0), ROCK, rough=0.9,
        rot=(side * 55.0, (0, 0, 1)))
# rocks side by side with a gap a foot can drop into
block(13.4, 15.4, 13.0, 17.0, 0.0, 1.2, ROCK, rough=0.9)
block(15.85, 17.85, 13.0, 17.0, 0.0, 1.0, ROCK, rough=0.9)

# --- north-east: a stair of ledges to a 6 m roof, rooftop gaps --------------------------
for k, h in enumerate([1.2, 2.4, 3.6, 4.8]):
    block(8.0 + k * 1.5, 9.5 + k * 1.5, -44.0, -40.0, 0.0, h, YELLOW if k else GREEN)
block(14.0, 22.0, -46.0, -38.0, 0.0, 6.0, CONCRETE)
for k, gap in enumerate([1.5, 2.5, 3.5]):
    x0 = 22.0 + gap + sum(6.0 + g for g in [1.5, 2.5, 3.5][:k])
    block(x0, x0 + 6.0, -46.0, -38.0, 0.0, 6.0 - 0.5 * (k + 1), CONCRETE)

# --- south-east: the jump tower ---------------------------------------------------------
# a core with a ramp spiralling round it: corner landings, ramps between them rising RISE each
CORNERS = [(32.0, 39.0), (42.0, 39.0), (42.0, 29.0), (32.0, 29.0)]   # ring corner centres, anticlockwise from above
SEGMENTS, RISE = 20, 3.6
TOP = SEGMENTS * RISE
block(33.0, 41.0, 30.0, 38.0, 0.0, TOP, CONCRETE)                       # the core
for k in range(SEGMENTS + 1):
    cx, cz = CORNERS[k % 4]
    h = k * RISE
    if k == SEGMENTS:   # the top landing reaches over onto the core
        block(cx - 1.0, cx + 2.0, cz - 2.0, cz + 1.0, h - 0.4, h, YELLOW)
    elif k:
        block(cx - 1.0, cx + 1.0, cz - 1.0, cz + 1.0, h - 0.4, h, YELLOW)
    if k == SEGMENTS:
        break
    (ax, az), (bx, bz) = CORNERS[k % 4], CORNERS[(k + 1) % 4]
    ang = math.degrees(math.atan2(RISE, 8.0))
    length = math.hypot(8.0, RISE) + 0.1
    mid = ((ax + bx) / 2, h + RISE / 2 - 0.2, (az + bz) / 2)
    if az == bz:   # along x: rotate about z, the far end up
        box(mid, (length, 0.4, 2.0), WHITE, rot=(ang if bx > ax else -ang, (0, 0, 1)))
    else:          # along z: rotate about x (a positive turn lowers +z)
        box(mid, (2.0, 0.4, length), WHITE, rot=(-ang if bz > az else ang, (1, 0, 0)))

# --- far south: the swing park -------------------------------------------------------------
for k in range(8):
    z = 60.0 + k * 14.0
    for x in (-8.0, 8.0):
        block(x - 0.6, x + 0.6, z - 0.6, z + 0.6, 0.0, 18.0, CONCRETE)
    block(-8.6, 8.6, z - 0.4, z + 0.4, 17.0, 18.0, YELLOW)
block(-6.0, 6.0, 168.0, 176.0, 0.0, 10.0, CONCRETE)   # a roof at the end to grapple up onto

# --- far west: the skate run ----------------------------------------------------------------
block(-112.0, -100.0, -6.0, 6.0, 0.0, 4.0, CONCRETE)                       # the deck
a = math.atan2(4.0, 40.0)
box((-80.0, 2.0 - 0.2 * math.cos(a), 0.0), (math.hypot(40.0, 4.0) + 0.2, 0.4, 12.0), WHITE,
    rot=(-math.degrees(a), (0, 0, 1)))                                     # down to the east
for z in (-3.5, 3.5):                                                      # kickers past its foot
    box((-52.0, 0.0, z), (2.0, 0.5, 2.0), YELLOW)
    ENTITIES[-1]["shape"] = "wedge"

# --- south: the playground (engine/playground.h: the seats and the bars are the level's "playground" list) ------
PLAY = []
SX, SZ = -4.0, 26.0           # the swing set: a beam along x at 2.5 m, two seats, swinging along z
for ex in (SX - 2.6, SX + 2.6):
    for dz in (-1.0, 1.0):
        box((ex, 1.25, SZ + dz * 0.55), (0.1, 2.7, 0.1), RED, rot=(dz * 22.0, (1, 0, 0)))
block(SX - 2.7, SX + 2.7, SZ - 0.06, SZ + 0.06, 2.45, 2.57, RED)
for sx in (SX - 1.1, SX + 1.1):
    PLAY.append({"kind": "swing", "pivot": [sx, 2.45, SZ], "length": 1.95, "forward": [0, 0, 1]})
BX0, BX1, BZ, BH = 4.0, 10.0, 26.0, 2.2   # the monkey bars: rungs across z every 0.4 m along x, 2.2 m up
for x in (BX0 - 0.1, BX1 + 0.1):
    for dz in (-0.35, 0.35):
        block(x - 0.05, x + 0.05, BZ + dz - 0.05, BZ + dz + 0.05, 0.0, BH + 0.05, YELLOW)
for k in range(int((BX1 - BX0) / 0.4) + 1):
    x = BX0 + k * 0.4
    block(x - 0.025, x + 0.025, BZ - 0.35, BZ + 0.35, BH - 0.025, BH + 0.025, WHITE)
for dz in (-0.35, 0.35):
    block(BX0 - 0.1, BX1 + 0.1, BZ + dz - 0.03, BZ + dz + 0.03, BH - 0.03, BH + 0.03, YELLOW)
PLAY.append({"kind": "bars", "a": [BX0, BH, BZ], "b": [BX1, BH, BZ]})

level = {
    "version": 1,
    "entities": ENTITIES,
    "environment": {"hdr": "../env/tree_lined_driveway_1k.hdr", "skyColor": [0.05, 0.05, 0.08]},
    "lighting": {
        "ambientMultiplier": 0.5,
        "exposure": 0.35,
        "sun": {"castsShadow": True, "color": [1.0, 0.95, 0.85], "direction": [0.45, 0.75, -0.35], "intensity": 9.5},
    },
    "player": {"collider": {"halfHeight": 0.4, "radius": 0.3, "shape": "capsule"}, "friction": 0.5,
               "position": [0.0, 1.0, 0.0]},
    "playground": PLAY,
    "pools": [{"min": [px0 + L, pz0 + L], "max": [px1 - L, pz1 - L], "level": -0.25, "depth": POOL_DEPTH - 0.25}],
}

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "levels", "movement_lab.json")
with open(out, "w") as f:
    json.dump(level, f, indent=1)
    f.write("\n")
print(f"wrote {os.path.relpath(out)}: {len(ENTITIES)} blocks")
