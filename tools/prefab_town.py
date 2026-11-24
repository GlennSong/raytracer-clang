#!/usr/bin/env python3
"""PREFAB TOWN: write the test level that stands prefabs in rooms, on a street and in a park (#191;
docs/model-record-design.md, "The test level").

    python3 tools/prefab_town.py            writes assets/levels/prefab_town.json (every piece still)
                                            and assets/levels/prefab_town_busy.json (each piece's `_in_use`
                                            prefab where it has one: every moving part working at once)

The level is two things, both from the tables below:
  - one script entity, assets/scripts/prefab_town.lua, which builds the street, the rooms' floors and walls and
    the park from ROOMS, STREET and GROUNDS (it makes no furniture);
  - one prefab entity for every line of a room's or a site's list.

A PLACEMENT is (prefab, x, z, facing, height). In a room, x runs across it and z from its middle toward its
doorway, in metres; facing is in degrees, 0 looking out through the doorway, 90 toward the room's +x; height is
how far above the floor it stands (a thing on a worktop). A site's placements are measured from the site's own
point the same way.

This table is the hand-made stand-in for what will place prefabs in the city: a site's own list, and the floor
planner's list of placements. Nothing here should grow into a second planner.
"""
import json
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

HALF, WALK, DEPTH = 4.0, 3.0, 8.0          # the carriageway's half width, a pavement's width, a room's depth
LINE = HALF + WALK                          # where the buildings' fronts stand
TOP = {"worktop": 0.9, "bar": 1.126, "desk": 0.74, "table": 0.76, "low": 0.418}   # heights of the kit's tops


def P(prefab, x, z, face=0.0, y=0.0):
    return (prefab, x, z, face, y)


def row(prefab, xs, z, face=0.0, y=0.0):
    return [P(prefab, x, z, face, y) for x in xs]


B = -DEPTH / 2                              # a room's back wall, in its own space
F = DEPTH / 2                               # and its front

ROOMS = [
    # the north side of the street, west to east: doorways face +z
    dict(name="konbini", w=10, yaw=0, door=3.24, floor="tile", things=[
        P("automatic_doors", 0, F), P("drinks_fridge", -3.6, B + 0.5), P("drinks_fridge", -1.5, B + 0.5),
        P("drinks_fridge_single", 0.2, B + 0.5), P("ice_cream_freezer", -2.5, -0.6),
        P("shelving", 0.8, 0.2, 90), P("shelving", 0.8, -1.2, 90), P("shelving", -0.2, 0.2, 270), P("shelving", -0.2, -1.2, 270),
        P("shop_counter", 3.6, 1.6, 270), P("kitchen_base", 4.6, B + 0.4), P("microwave", 4.6, B + 0.4, 0, TOP["worktop"]),
        P("kitchen_base", 4.0, B + 0.4), P("espresso_machine", 4.0, B + 0.4, 0, TOP["worktop"]),
        P("atm", 4.5, -1.0, 270), P("basket_stack", -1.9, 3.3), P("shopping_cart", -3.2, 3.0, 90),
        P("capsule_machines", -4.5, 1.2, 90), P("postcard_rack", 2.2, 3.2), P("wall_clock", 2.5, B + 0.09, 0, 2.3),
    ]),
    dict(name="cafe", w=9, yaw=0, door=1.02, floor="wood", things=[
        P("door_glass", 0, F), P("bar_counter", 2.2, -1.8), P("espresso_machine", 2.5, -1.8, 180, TOP["bar"]),
        P("cash_register", 1.6, -1.8, 180, TOP["bar"]), P("beer_taps", 3.0, -1.75, 0, TOP["bar"]),
        *row("bar_stool_swivel", [1.5, 2.2, 2.9], -0.9, 180), P("fridge", 3.9, B + 0.4), P("kitchen_base", 3.2, B + 0.4),
        P("stand_mixer", 3.2, B + 0.4, 0, TOP["worktop"]), P("round_table_turning", -2.3, 0.6),
        P("bistro_chair", -2.3, 1.7, 180), P("bistro_chair", -2.3, -0.5, 0), P("bistro_chair", -3.4, 0.6, 90), P("bistro_chair", -1.2, 0.6, 270),
        P("booth", -3.4, -2.9, 0), P("display_turntable", 0.9, -1.8, 0, TOP["bar"]), P("wall_clock", -1, B + 0.09, 0, 2.3),
        P("ceiling_fan", -1.5, 0.5, 0, 2.9), P("potted_plant", -4.2, 3.6),
    ]),
    dict(name="office", w=12, yaw=0, door=1.92, floor="carpet", things=[
        P("double_door", 0, F), *row("standing_desk", [-4.2, -2.2, -0.2], -2.2, 180),
        *row("office_chair_adjustable", [-4.2, -2.2, -0.2], -3.1, 0),
        *row("desk", [-4.2, -2.2, -0.2], 0.9, 180), *row("office_chair", [-4.2, -2.2, -0.2], 0.1, 0),
        *row("desk_lamp", [-4.7, -2.7, -0.7], 0.9, 180, TOP["desk"]), P("laptop", -4.0, 0.9, 0, TOP["desk"]),
        P("paper_trimmer", -1.9, 0.9, 0, TOP["desk"]), P("desk_fan", 0.2, 0.9, 0, TOP["desk"]),
        P("filing_cabinet", 1.0, 0.9, 270), P("desk_pedestal", -3.2, -2.3, 180),
        P("printer", 2.4, B + 0.4), P("shredder", 3.3, B + 0.3), P("store_cupboard", 4.3, B + 0.3), P("server_rack", 5.4, -1.5, 270),
        P("whiteboard", 3.2, 0.2, 200), P("flip_chart", 4.6, 2.0, 220), P("ceiling_fan", -2, 0, 0, 2.9),
        P("wall_clock", -2.2, B + 0.09, 0, 2.4), P("fire_hose_cabinet", -5.85, 2.2, 90, 0.9), P("room_safe", 5.5, 3.3, 270),
        P("potted_plant", -5.5, 3.5), P("monitor_arm", -2.2, 0.75, 0, TOP["desk"]),
    ]),
    dict(name="gym", w=10, yaw=0, door=1.02, floor="tile", things=[
        P("door", 0, F), *row("treadmill", [-3.8, -2.6], -2.6), *row("exercise_bike", [-0.9, 0.1], -2.9),
        P("rowing_machine", 1.6, -2.6), P("weight_machine", 3.6, -2.6), P("punch_bag", -3.5, 1.6, 90),
        P("lockers", 4.6, 1.2, 270), P("lockers", 4.6, 2.5, 270), P("stand_fan", 2.2, 3.3, 200),
        P("drinking_fountain", -4.5, 3.4, 90), P("wall_clock", 0, B + 0.09, 0, 2.4), P("turnstile", 1.6, 2.6),
    ]),
    dict(name="gym: free weights and mats", w=9, yaw=0, door=1.02, floor="wood", things=[
        P("door", 0, F), P("mirror_wall", -1.2, B + 0.13), P("dumbbell_rack", -3.0, B + 0.75), P("bench_press", -0.6, -1.6),
        P("bench_press", 1.4, -1.6), P("yoga_mats", -2.4, 1.6), P("gym_bench", 3.4, 2.6, 270), P("lockers", 3.9, -0.4, 270),
        P("lockers", 3.9, -1.7, 270), P("punch_bag", 2.6, 0.8, 270), P("towel_shelf", 3.4, B + 0.02, 0, 1.2),
    ]),
    dict(name="barber", w=9, yaw=0, door=1.02, floor="tile", things=[
        P("door_glass", 0, F), *row("barber_station", [-3.2, -1.4, 0.4], B + 0.26), *row("barber_chair", [-3.2, 0.4], -2.3, 180),
        P("barber_chair_oxblood", -1.4, -2.3, 180), P("backwash_chair", 3.4, -2.8), P("towel_shelf", 2.2, B + 0.02, 0, 1.2),
        P("salon_trolley", -2.3, -2.6), P("hood_dryer", 3.6, -0.6, 270), P("barber_waiting_seats", -2.6, 3.3, 180), P("barber_pole", 1.4, F + 0.25),
        P("cash_register", 3.4, 2.4, 270, TOP["worktop"]), P("shop_counter", 3.4, 2.4, 270), P("wall_clock", -0.5, B + 0.09, 0, 2.5),
    ]),
    dict(name="station hall", w=14, yaw=0, door=3.24, floor="tile", things=[
        P("automatic_doors", 0, F), *row("flap_gate", [-2.2, 0, 2.2], -0.5), P("turnstile_full", -4.6, -0.5),
        P("queue_posts", 4.4, -0.5), P("departure_board", 0, B + 0.1, 0, 1.6), P("ticket_machine", -6.5, 1.8, 90),
        P("ticket_machine", -6.5, 2.8, 90), P("coin_lockers", 6.5, 2.2, 270), P("waiting_seats", -4, B + 0.5),
        P("waiting_seats", 4, B + 0.5), P("baggage_scanner", 3.6, 2.4, 90), P("mall_directory", -3.2, 2.8),
        P("vending_machine_red", 6.5, -2.2, 270), P("ticket_dispenser", -1.6, 2.6), P("teller_window", -5.2, 1.2, 0, TOP["worktop"]),
        P("kitchen_base", -5.5, 1.2), P("kitchen_base", -4.9, 1.2), P("wall_clock", 5, B + 0.09, 0, 2.5), P("luggage_cart", 1.8, 3.0, 40),
    ]),
    # the south side, west to east: doorways face -z
    dict(name="flat: kitchen and living room", w=10, yaw=180, door=1.02, floor="wood", things=[
        P("door", 0, F), P("fridge", -4.6, B + 0.4), P("oven", -3.9, B + 0.4), P("kitchen_base", -3.3, B + 0.4),
        P("kitchen_sink", -2.7, B + 0.4), P("dishwasher", -2.1, B + 0.4), P("kitchen_base", -1.5, B + 0.4),
        P("toaster", -3.3, B + 0.4, 0, TOP["worktop"]), P("rice_cooker", -1.6, B + 0.4, 0, TOP["worktop"]),
        P("kitchen_wall_cupboard", -3.1, B + 0.26, 0, 1.45), P("kitchen_wall_cupboard", -1.7, B + 0.26, 0, 1.45),
        P("pedal_bin", -0.9, B + 0.35), P("dining_table", -2.8, 0.4), P("dining_chair", -3.2, 1.2, 180), P("dining_chair", -2.4, 1.2, 180),
        P("dining_chair", -3.2, -0.4, 0), P("dining_chair", -2.4, -0.4, 0), P("laptop", -2.8, 0.4, 0, TOP["table"]),
        P("tv_unit", 2.6, B + 0.33), P("sofa", 2.6, 0.9, 180), P("coffee_table", 2.6, -0.8), P("globe", 2.9, -0.8, 0, TOP["low"]), P("metronome", 2.25, -0.8, 0, TOP["low"]),
        P("recliner", 4.3, -0.6, 250), P("rocking_chair", 0.7, 0.2, 110), P("grandfather_clock", 4.6, B + 0.25),
        P("upright_piano", -0.4, 3.55, 180), P("record_player", 4.5, 2.6, 270, TOP["worktop"]),
        P("kitchen_base", 4.6, 2.6, 270), P("curtains", 2.6, 3.8, 180), P("ceiling_fan", 2.6, 0, 0, 2.9), P("potted_plant", 0.6, B + 0.3),
    ]),
    dict(name="flat: bedroom and bathroom", w=9, yaw=180, door=1.02, floor="wood", things=[
        P("door", 0, F), P("bed", -2.6, B + 1.1), P("nightstand", -3.9, B + 0.3), P("nightstand", -1.3, B + 0.3),
        P("sliding_wardrobe", 0.6, B + 0.33), P("wall_bed", -4.2, 2.0, 90), P("venetian_blind", -2.6, B + 0.1, 0, 1.2),
        P("toilet_lidded", 3.9, B + 0.45), P("vanity", 2.6, B + 0.3), P("bathroom_cabinet", 4.42, -1.9, 270, 1.3),
        P("shower", 3.95, 0.2, 270), P("bathtub", 3.6, 3.5, 180), P("pedal_bin", 3.2, B + 0.3),
        P("washing_machine", 1.6, 3.6, 180), P("dryer", 2.3, 3.6, 180), P("wall_clock", -2.6, 3.9, 180, 2.3),
    ]),
    dict(name="flat: child's room", w=6, yaw=180, door=1.02, floor="carpet", things=[
        P("door", 0, F), P("bunk_bed", -2.3, B + 1.05), P("toy_shelf", 0.2, B + 0.2), P("toy_chest", 2.2, B + 0.3),
        P("kids_desk", 2.4, 0.6, 270), P("play_mat", -0.4, 0.6), P("toy_blocks", -0.9, 1.1), P("rocking_horse", 1.0, 2.4, 210),
        P("dollhouse", -2.4, 2.6, 90), P("roller_blind", 0.2, B + 0.1, 0, 2.6),
    ]),
    dict(name="playroom and nursery", w=7, yaw=180, door=1.02, floor="carpet", things=[
        P("door", 0, F), P("cradle", -2.6, -2.6), P("baby_mobile", -2.6, -2.6, 0, 2.0), P("kids_bed_red", 2.6, B + 1.0),
        P("play_table", 0.2, 0.4), P("kids_easel", -2.6, 1.8, 90), P("toy_shelf", 0.0, B + 0.2), P("play_mat", 1.6, -1.0),
        P("rocking_chair", -1.2, -2.2, 30), P("toy_chest", 2.8, 2.2, 270),
    ]),
    dict(name="clinic", w=9, yaw=180, door=1.02, floor="tile", things=[
        P("door", 0, F), P("care_bed", -3.2, -2.8), P("care_bed", -0.6, -2.8), P("iv_stand", -2.4, -3.4), P("iv_stand", 0.3, -3.4),
        P("curtains", -1.9, -2.9, 90), P("exam_couch", 3.6, -2.8), P("exam_lamp", 2.6, -2.2), P("wheelchair", 1.6, 0.8, 160),
        P("waiting_seats", -2.6, 3.5, 180), P("desk", 2.8, 1.6, 180), P("reception_bell", 2.4, 1.7, 0, TOP["desk"]),
        P("office_chair", 2.8, 0.8, 0), P("ticket_dispenser", 0.9, 3.2, 180), P("store_cupboard", 4.2, 3.0, 270),
        P("wall_clock", 1.8, B + 0.09, 0, 2.4), P("fire_hose_cabinet", -4.35, 1.5, 90, 0.9),
    ]),
    dict(name="schoolroom and library", w=10, yaw=180, door=1.02, floor="wood", things=[
        P("door", 0, F), *row("flip_desk", [-4.0, -2.8, -1.6], -0.6), *row("flip_desk", [-4.0, -2.8, -1.6], 1.0),
        *row("school_chair", [-4.0, -2.8, -1.6], -0.05, 180), *row("school_chair", [-4.0, -2.8, -1.6], 1.55, 180),
        P("lecture_blackboards", -2.8, B + 0.1), P("school_bell", -4.85, 3.0, 90, 1.5), P("projector_screen", -2.8, B + 0.4, 0, 2.95),
        *row("bookcase", [1.4, 2.6, 3.8], B + 0.25), P("library_ladder", 2.6, B + 0.45), P("book_truck", 2.0, -0.6, 30),
        P("reading_table", 2.8, 1.8), P("globe", 2.8, 1.8, 0, 0.75), P("wall_clock", 0.2, B + 0.09, 0, 2.5), P("flip_chart", -0.6, -2.4, 20),
    ]),
    dict(name="bakery", w=8, yaw=180, door=1.02, floor="tile", things=[
        P("door_glass", 0, F), P("deck_oven", -2.6, B + 0.6), P("dough_mixer", -0.9, B + 0.5), *row("kitchen_base", [0.5, 1.1, 1.7], B + 0.4),
        P("shop_scales", 0.6, B + 0.4, 0, TOP["worktop"]), P("stand_mixer", 1.6, B + 0.4, 0, TOP["worktop"]),
        P("kitchen_swing_doors", 3.0, -1.6), P("bar_counter", -0.6, 1.3), P("cash_register", -1.1, 1.3, 180, TOP["bar"]),
        P("display_turntable", -0.1, 1.3, 0, TOP["bar"]), P("drinks_fridge_single", 3.4, 1.4, 270), P("wall_clock", 2.5, B + 0.09, 0, 2.4),
    ]),
]

# Sites: a point in the world, a facing, and a list measured from it. A playground is one of these, not a prefab.
SITES = [
    dict(name="north pavement", x=0, z=-(HALF + 1.6), yaw=0, things=[
        P("vending_machine", -21, -1.0), P("kiddie_ride_car", -26.5, -0.6), P("kiddie_ride_rocket", -25.2, -0.6),
        P("newspaper_box", -17.5, -0.8), P("litter_bin", -16.5, -0.8), P("sandwich_board", -11.5, -0.4), P("barber_pole", -9.2, -1.32, 0, 1.6),
        P("awning", -13.5, -1.3, 0, 2.7), P("patio_umbrella", -15.6, 0.3), P("picnic_table", -15.6, 0.3),
        P("park_bench", -3, -0.8), P("bike_rack", 3, -0.6), P("mailbox", 13.5, 0.6), P("street_lamp", -18, 1.2), P("street_lamp", 6, 1.2),
        P("hydrant", 11, 1.1), P("mailboxes", 24.5, -1.0),
    ]),
    dict(name="south pavement", x=0, z=HALF + 1.6, yaw=180, things=[
        P("park_bench", 14, -0.8), P("litter_bin", 12.5, -0.8), P("street_lamp", 20, 1.2), P("street_lamp", -4, 1.2),
        P("awning", -20, -1.3, 0, 2.7), P("sandwich_board", -18.5, -0.4), P("bike_rack", 2, -0.6),
    ]),
    dict(name="street end", x=-29, z=0, yaw=0, things=[
        P("parking_barrier", 0, -2.0, 90), *row("rising_bollard", [0.0], 1.0), P("rising_bollard", 0, 2.0), P("rising_bollard", 0, 3.0),
        P("parked_sedan", 6, 2.6, 90), P("parked_pickup", 13, -2.6, 270),
    ]),
    dict(name="playground", x=60, z=18, yaw=0, things=[
        P("playground_swing", -4, -4), P("playground_seesaw", 3.5, -4.5), P("playground_roundabout", -4.5, 3),
        P("playground_slide", 3.5, 2.5), P("park_bench", 0, -8.5), P("park_bench", -8.6, 0, 90), P("litter_bin", 1.8, -8.5),
        P("drinking_fountain", -8.5, -6), P("park_sculpture", 8, -8), P("oak", 8.5, 7.5), P("oak", -8.5, 8), P("boulder", 7, 7),
        P("garden_gate", -10, -9.5, 90), P("picnic_table", 0, 8), P("bbq", 2.2, 8.4), P("porch_swing", 8.2, 0.5, 270),
        P("rotary_dryer", -1.5, -0.5), P("weathervane", 9.5, -4.5), P("wind_chime", -9.2, 4.5),
    ]),
    dict(name="square", x=60, z=-18, yaw=0, things=[
        P("fountain_plaza", 0, 0), P("flagpole", -8, 7), *row("park_bench", [-4, 4], 5.5, 180), P("escalator_up", -7, -4.5, 90),
        P("revolving_door", 7.5, 5), P("lift_doors", 5, 9.6, 180), P("garage_door", 10.2, 1.5, 270), P("roller_shutter", 10.2, -4.5, 270), P("ballot_box", 4, -4), P("street_lamp", -9, -8), P("street_lamp", 9, 8),
        # a wall of windows and screens along the square's far edge, each standing free
        # a row of entrance doors by kind of building, along the near edge
        *[P(d, -9.5 + i * 2.1, 10.4, 180) for i, d in enumerate(["front_door_red", "front_door_green", "front_door_wood", "back_door", "fire_exit", "shop_door", "apartment_entrance", "office_entrance", "school_entrance", "civic_entrance"])],
        P("sash_window", -8, -9, 0, 0.9), P("casement_window", -6, -9, 0, 0.9), P("shoji_screens", -3, -9), P("cinema_seats", 3.5, -8.5),
        P("lecture_seats", 7.5, -8.5), P("dj_deck", 0.5, 8.5, 180), P("mirror_ball", 0.5, 8.5, 0, 3.0), P("clothes_rail_round", -4.5, 8.5),
    ]),
]

GROUNDS = [dict(x=60, z=18, w=22, d=22, kind="grass"), dict(x=60, z=-18, w=22, d=22, kind="paving")]


def prefab_names():
    text = open(os.path.join(ROOT, "assets/scripts/prefabs.lua")).read()
    return set(re.findall(r"^  ([a-z_0-9]+) = \{ model", text, re.M))


def place(cx, cz, yaw, x, z):
    """A point in a room's or a site's own space, in the world (a turn about +y takes +x toward -z)."""
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return cx + x * c + z * s, cz - x * s + z * c


def build(busy):
    known = prefab_names()
    # lay the rooms along each side, from the west end
    cursor = {0: -28.0, 180: -28.0}
    rooms = []
    for r in ROOMS:
        r = dict(r)
        r["x"] = cursor[r["yaw"]] + r["w"] / 2
        r["z"] = -(LINE + DEPTH / 2) if r["yaw"] == 0 else LINE + DEPTH / 2
        r["d"] = DEPTH
        cursor[r["yaw"]] += r["w"]
        rooms.append(r)
    east = max(cursor.values())
    entities = [
        {"id": 1, "name": "ground", "shape": "box", "position": [18, -0.2, 0], "size": [160, 0.4, 110],
         "material": {"albedo": [0.42, 0.44, 0.4], "metallic": 0.0, "roughness": 1.0, "opacity": 1.0, "emission": [0, 0, 0], "checkerboard": False},
         "physics": {"motion": "static", "friction": 0.9, "restitution": 0.0}},
        {"id": 2, "name": "the place", "shape": "script", "file": "prefab_town.lua", "seed": 1,
         "opts": {"rooms": [{k: r[k] for k in ("name", "x", "z", "w", "d", "yaw", "door", "floor")} for r in rooms],
                  "street": {"x0": -32, "x1": 52, "half": HALF, "walk": WALK}, "grounds": GROUNDS}},
    ]
    count, missing = 0, []
    for group in rooms + SITES:
        for n, (prefab, x, z, face, y) in enumerate(group["things"]):
            if prefab not in known:
                missing.append("%s: %s" % (group["name"], prefab))
                continue
            name = prefab + "_in_use" if busy and prefab + "_in_use" in known else prefab
            wx, wz = place(group["x"], group["z"], group["yaw"], x, z)
            ent = {"id": len(entities) + 1, "name": "%s / %s %d" % (group["name"], prefab, n + 1), "shape": "prefab", "prefab": name,
                   "position": [round(wx, 3), round(y + 0.05 if group in rooms else y, 3), round(wz, 3)]}
            turn = (group["yaw"] + face) % 360
            if turn:
                ent["orientation"] = {"axis": [0, 1, 0], "angleDeg": turn}
            entities.append(ent)
            count += 1
    if missing:
        sys.exit("prefab_town: no such prefab: " + "; ".join(missing))
    name = "prefab_town_busy" if busy else "prefab_town"
    level = {
        "version": 1, "watch_scripts": True, "name": name,
        "_note": "PREFAB TOWN (#191): %d prefabs stood in %d rooms, along a street, in a park and on a square. Written by "
                 "tools/prefab_town.py from the tables in it: edit those and run it again, not this file. %s"
                 % (count, len(rooms), "Every piece that has an `_in_use` prefab is that one: all moving parts working at once, as a load test."
                    if busy else "Every piece is still."),
        "player": {"position": [-24, 1.2, 0], "collider": {"shape": "capsule", "radius": 0.3, "halfHeight": 0.6}, "friction": 0.5},
        "environment": {"skyColor": [0.55, 0.68, 0.86]},
        "lighting": {"ambientMultiplier": 0.7, "exposure": 0.7,
                     "sun": {"castsShadow": True, "color": [1, 0.98, 0.94], "direction": [0.4, 0.72, -0.32], "intensity": 6.0}},
        "entities": entities,
    }
    path = os.path.join(ROOT, "assets/levels", name + ".json")
    with open(path, "w") as out:
        json.dump(level, out, indent=1)
        out.write("\n")
    print("%s: %d prefabs, %d rooms, %d sites, street to x = %.0f" % (os.path.relpath(path, ROOT), count, len(rooms), len(SITES), east))


if __name__ == "__main__":
    build(False)
    build(True)
