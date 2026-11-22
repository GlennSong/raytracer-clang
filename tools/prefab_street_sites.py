#!/usr/bin/env python3
"""Lay out the hand-made sites of the test level prefab_street (#191): a playground, a park, a vegetable garden,
a quad and a row of roof equipment, west of the buildings. Every thing is a prefab; every fence is fence.lua run
along a path. Run it again after changing a layout: it replaces only the entities whose names begin "site / ".

    python3 tools/prefab_street_sites.py
"""
import json, os

LEVEL = os.path.join(os.path.dirname(__file__), "..", "assets", "levels", "prefab_street.json")
Y = 0.03                      # the paving's top
FACE = {"+z": 0, "+x": 90, "-z": 180, "-x": 270}   # which way a thing's front (its own +z) points

ents, grounds = [], []

def put(site, prefab, x, z, face="+z", y=Y):
    e = {"name": "site / %s / %s" % (site, prefab), "shape": "prefab", "prefab": prefab, "position": [x, y, z]}
    deg = FACE.get(face, face)
    if deg: e["orientation"] = {"axis": [0, 1, 0], "angleDeg": deg}
    ents.append(e)

def fence(site, style, points, gates=(), **more):
    # (the fence's own origin is the middle of its points: how far away it is, for its level of detail, is measured
    # from there)
    cx, cz = sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points)
    args = {"points": [[round(p[0] - cx, 3), round(p[1] - cz, 3)] for p in points], "style": style}
    if gates: args["gates"] = [{"at": at, "width": w} for at, w in gates]
    args.update(more)
    ents.append({"name": "site / %s / fence (%s)" % (site, style), "shape": "prefab",
                 "prefab": {"recipe": "fence", "args": args}, "position": [cx, Y, cz]})

def path(x0, x1, z0, z1):
    grounds.append({"x": (x0 + x1) / 2, "z": (z0 + z1) / 2, "w": x1 - x0, "d": z1 - z0, "kind": "paving"})

# ---- THE PLAYGROUND (x -118..-78, z -45..-9): a low iron fence round it, its gate on the street side
S = "playground"
fence(S, "iron", [(-118, -9), (-78, -9), (-78, -45), (-118, -45)], gates=[(20, 1.2)], closed=True, height=1.1)
put(S, "garden_gate", -98, -9)
for prefab, x, z, face in [("playground_swing_four", -108, -36, "+z"), ("playground_swing", -108, -22, "+x"),
                           ("playground_slide_tall", -96, -38, "+z"), ("playground_slide", -88, -38, "+z"),
                           ("playground_roundabout", -97, -26, "+z"), ("playground_seesaw", -88, -27, "+z"),
                           ("playground_seesaw", -88, -22, "+z"), ("rocking_horse", -101, -17, "+z"), ("rocking_horse", -99, -17, "-z"),
                           ("park_bench", -106, -11, "-z"), ("park_bench", -90, -11, "-z"), ("park_bench", -116.5, -30, "+x"),
                           ("litter_bin", -100.5, -10.5, "+z"), ("drinking_fountain", -80, -14, "+z"), ("bike_rack", -80, -20, "+x"),
                           ("globe_lamp", -116, -11, "+z"), ("globe_lamp", -80, -43, "+z"), ("globe_lamp", -116, -43, "+z"),
                           ("globe_lamp", -80, -11, "+z"), ("notice_board", -95, -10.2, "+z")]:
    put(S, prefab, x, z, face)

# ---- THE PARK (x -118..-78, z 9..52): railings round it, two paths crossing at a fountain, benches beside the
# paths and facing them, lanterns where the paths meet the gates, low bollard lights along the way in
S = "park"
fence(S, "iron", [(-118, 9), (-78, 9), (-78, 52), (-118, 52)], gates=[(20, 3.0), (40 + 21, 3.0), (40 + 43 + 20, 3.0), (40 + 43 + 40 + 22, 3.0)], closed=True)
path(-104, -92, 24, 36)                                   # the square round the fountain
path(-99.5, -96.5, 9, 24); path(-99.5, -96.5, 36, 52)     # the walk from the street, and on to the far gate
path(-118, -104, 28.5, 31.5); path(-92, -78, 28.5, 31.5)  # the cross walk
put(S, "fountain_plaza", -98, 30)
for z in (14, 19, 41, 46):                                # benches either side of the long walk, facing it
    put(S, "park_bench", -100.4, z, "+x"); put(S, "park_bench", -95.6, z, "-x")
for x in (-112, -108, -88, -84):                          # and of the cross walk
    put(S, "park_bench", x, 27.6, "+z"); put(S, "park_bench", x, 32.4, "-z")
for x, z in [(-100.2, 10.5), (-95.8, 10.5), (-100.2, 50.5), (-95.8, 50.5), (-116.5, 27.8), (-116.5, 32.2), (-79.5, 27.8), (-79.5, 32.2)]:
    put(S, "park_lantern", x, z)
for x, z in [(-103.5, 24.5), (-92.5, 24.5), (-103.5, 35.5), (-92.5, 35.5)]: put(S, "globe_lamp", x, z)
for z in (16.5, 21.5, 38.5, 43.5, 48):
    put(S, "path_bollard_light", -99.9, z); put(S, "path_bollard_light", -96.1, z)
for prefab, x, z, face in [("picnic_table", -110, 44, "+z"), ("picnic_table", -106, 47, 30), ("picnic_table", -88, 44, "+z"),
                           ("bbq", -85, 47, "+z"), ("porch_swing", -112, 17, "+x"), ("litter_bin", -100.6, 24.8, "+z"),
                           ("litter_bin", -86, 41, "+z"), ("drinking_fountain", -95.6, 23.2, "-x"), ("sundial", -110, 20, "+z"),
                           ("stone_planter", -86, 18, "+z"), ("stone_planter", -84, 20, "+z"), ("park_sculpture", -87, 38, "+z"),
                           ("bike_rack", -93.5, 10.3, "+z"), ("clock_post", -91.5, 30, "+z")]:
    put(S, prefab, x, z, face)

# ---- THE VEGETABLE GARDEN (x -160..-126, z -45..-9): a picket fence, beds in rows either side of a middle path
S = "garden"
fence(S, "picket", [(-160, -9), (-126, -9), (-126, -45), (-160, -45)], gates=[(17, 1.2)], closed=True, paint="white")
put(S, "garden_gate", -143, -9)
path(-143.8, -142.2, -40, -9)
crops = ["cabbage", "carrot", "lettuce", "tomato", "lettuce", "cabbage", "tomato", "carrot"]
for i in range(4):
    z = -14 - i * 5.2
    for j, x in enumerate((-153.5, -149.5)): put(S, "raised_bed_" + crops[(i * 2 + j) % 8], x, z, "+x")
    for j, x in enumerate((-136.5, -132.5)): put(S, "raised_bed_" + crops[(i * 2 + j + 3) % 8], x, z, "+x")
for prefab, x, z, face in [("tool_shed", -156, -41.5, "+z"), ("greenhouse", -131, -40.5, "+z"), ("compost_bin", -151, -42.5, "+z"),
                           ("compost_bin", -149.6, -42.5, "+z"), ("water_butt", -154.2, -40.3, "+x"), ("wheelbarrow", -146, -38, 40),
                           ("bean_poles", -138, -36.5, "+z"), ("bean_poles", -138, -38.5, "+z"), ("scarecrow", -151.5, -24.4, 20),
                           ("cold_frame", -128.3, -14, "-x"), ("cold_frame", -128.3, -16, "-x"), ("cold_frame", -128.3, -18, "-x"),
                           ("raised_bed_bare", -141, -37, "+z"), ("park_bench", -145.2, -30, "+x"), ("path_bollard_light", -144.3, -11, "+z"),
                           ("path_bollard_light", -141.7, -11, "+z"), ("wall_lantern", -155.6, -40.7, "+z", ), ("notice_board", -139, -10.2, "+z")]:
    put(S, prefab, x, z, face, **({"y": 2.0} if prefab == "wall_lantern" else {}))

# ---- THE QUAD (x -160..-126, z 9..52): a low wall with a clipped hedge inside it, a ring walk round a statue,
# stone benches and planters, a clock, a bike shelter by the gate
S = "quad"
fence(S, "wall", [(-160, 9), (-126, 9), (-126, 52), (-160, 52)], gates=[(17, 4.0), (34 + 43 + 17, 4.0)], closed=True)
fence(S, "hedge", [(-158.6, 13), (-158.6, 50.6), (-127.4, 50.6), (-127.4, 13)], height=1.0)
path(-144.5, -141.5, 9, 23); path(-144.5, -141.5, 38, 52)
path(-150.5, -135.5, 23, 26); path(-150.5, -135.5, 35, 38); path(-150.5, -147.5, 26, 35); path(-138.5, -135.5, 26, 35)
path(-160, -150.5, 29, 32); path(-135.5, -126, 29, 32)
put(S, "statue", -143, 30.5)
for x, z, face in [(-143, 22.2, "+z"), (-143, 38.8, "-z"), (-151.3, 30.5, "+x"), (-134.7, 30.5, "-x")]: put(S, "stone_bench", x, z, face)
for x, z in [(-151.5, 22), (-134.5, 22), (-151.5, 39), (-134.5, 39)]: put(S, "stone_planter", x, z)
for x, z in [(-146, 10.5), (-140, 10.5), (-146, 50.5), (-140, 50.5), (-152, 27), (-134, 27), (-152, 34), (-134, 34)]: put(S, "globe_lamp", x, z)
for prefab, x, z, face in [("clock_post", -147, 17, "+z"), ("bike_shelter", -133, 13.5, "+z"), ("notice_board", -139.5, 14, "-x"),
                           ("flagpole", -139, 44, "+z"), ("sundial", -147, 44, "+z"), ("stone_bench", -155, 20, "+x"),
                           ("stone_bench", -155, 41, "+x"), ("stone_bench", -131, 41, "-x"), ("litter_bin", -145, 21, "+z")]:
    put(S, prefab, x, z, face)

# ---- A ROW OF ROOF EQUIPMENT on a deck (x -160..-80, z 64..84), to walk round; the helipad at its east end
S = "roof"
grounds.append({"x": -120, "z": 74, "w": 84, "d": 24, "kind": "paving"})
x = -158
for prefab, w in [("roof_access", 5), ("roof_hvac", 5.5), ("roof_hvac_large", 7), ("roof_condenser", 2), ("roof_condenser", 2), ("roof_condenser", 2.5),
                  ("roof_cooling_tower", 5), ("roof_water_tower", 5.5), ("roof_water_tank", 5.5), ("roof_dish", 3), ("roof_antenna", 3),
                  ("roof_vent", 1.6), ("roof_vent", 2), ("roof_stack", 3), ("roof_skylight", 4.5), ("roof_solar", 8)]:
    put(S, prefab, x + w / 2, 70)
    x += w
put(S, "roof_helipad", -90, 74)
put("roof", "car_park_lamp", -120, 82); put("roof", "pitch_floodlight", -104, 84.5, "-z")

d = json.load(open(LEVEL))
keep = [e for e in d["entities"] if not str(e.get("name", "")).startswith(("site / ", "park / ", "playground / "))]
nid = max(e.get("id", 0) for e in keep) + 1
for e in ents:
    e["id"] = nid; nid += 1
d["entities"] = keep + ents
for e in d["entities"]:
    if e.get("shape") == "script":
        o = e["opts"]
        o["street"]["x0"] = -166
        o["grounds"] = [g for g in o["grounds"] if g["x"] >= 0] + [{"x": -74, "z": 0, "w": 8, "d": 112, "kind": "paving"},
                                                                    {"x": -122, "z": 0, "w": 8, "d": 112, "kind": "paving"}] + grounds
    if e.get("name") == "ground":
        e["position"] = [60, -0.2, -10]; e["size"] = [500, 0.4, 240]
json.dump(d, open(LEVEL, "w"), indent=1)
print("%d site entities, %d paths" % (len(ents), len(grounds)))
