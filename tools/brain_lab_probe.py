#!/usr/bin/env python3
"""brain_lab_probe: the resident with the agent brain (ADR-0441, ADR-0442) watched with nobody at the keyboard.

Launches the viewer on brain_lab, waits for the city, sends `goto <marker>` and polls `brain?` four times a second
until the resident stands at the marker (or the timeout). At each moment worth seeing it takes the follow camera's
screenshot and prints the `brain?` line: at home beside the parked car, in the car, the car sliding out of its space,
stopped on the drive (a light or a stop line), sliding into its space, walking to the marker, standing at it. The
pictures go to ~/Pictures/Raytracer/brain/. While nothing moves (the two minutes of getting in, the three of getting
out, which are travel.lua's own) the sim is run faster (--wait-speed). Stops the viewer it started, by its own process
(never by name: the owner runs `viewer` too).

A picture taken from behind a car cannot say where its nose is against a line painted on the road (the ground beyond
the roof is hidden, and the ground seen level with the rear window is metres ahead of it). So where that matters the
probe also lets the camera go (`brain follow off`), holds the sim still, and takes the same moment FROM ABOVE and from
the side: the person at a marker, the car at each stop on the drive, the car in its space.

    python3 tools/brain_lab_probe.py                 # home to work
    python3 tools/brain_lab_probe.py --to shop --trait has_board
    python3 tools/brain_lab_probe.py --suffix _v2    # the pictures' names end in _v2
    python3 tools/brain_lab_probe.py --scenario crowd --level assets/levels/brain_crowd.json   # the crowd's busy period
"""
import argparse
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtctl  # noqa: E402

ABOVE_HEIGHT = 15.0   # metres above the ground the camera looks down from
SIDE_AWAY = 7.5       # metres to the side of the body the side view is taken from
SIDE_HEIGHT = 1.7     # ...and its height, about a standing person's eyes
STOPS_SHOT = 3        # how many stops on the drive are taken from above


def wait_for_file(path, timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return True
        time.sleep(0.25)
    return False


def shoot(sock, shots, name, t0, line):
    path = os.path.join(shots, name + ".png")
    if os.path.exists(path):
        os.remove(path)
    rtctl.send(sock, f"shot {path}")
    ok = wait_for_file(path)
    print(f"  {time.time() - t0:6.1f} s  shot {path}" + ("" if ok else " (did not land)"))
    print(f"           brain? {line}")


def state_of(line):
    """The state's name from a `brain?` line: the part after the plan and the leg, before ` since`."""
    m = re.search(r"\| (\w+) since [\d.]+ s", line)
    return m.group(1) if m else ""


def body_at(line):
    """Where the person is (in the car: the car's middle), from a `brain?` line: plan x and z."""
    m = re.search(r"is at (-?[\d.]+) (-?[\d.]+)", line)
    return (float(m.group(1)), float(m.group(2))) if m else None


def from_above_and_the_side(sock, shots, name, t0, line, above=True, side=True, who="ana", at=None):
    """The same moment with the camera let go: (above) straight down from ABOVE_HEIGHT, and (side) level from SIDE_AWAY
    to the side. The sim is held still meanwhile; afterwards the camera follows the person again. (A marker is on a
    sidewalk under the street's trees, which hide it from above: there only the side is taken.)"""
    at = at or body_at(line)
    if not at:
        print("           (no position in the brain? line: not taken from above)")
        return
    x, z = at
    rtctl.send(sock, "sim pause")
    rtctl.send(sock, "brain follow off")
    if above:
        rtctl.send(sock, f"camera {x} {ABOVE_HEIGHT} {z} -89 0")
        time.sleep(0.8)
        shoot(sock, shots, name + "_from_above", t0, line)
        # what the view's centre (straight down on the body) hits: a car's kinematic box shows as a body at its roof
        print("           aim?", rtctl.send(sock, "aim?"))
    if side:
        # yaw 0 looks along -z, so standing SIDE_AWAY toward +z of the body and looking at it
        rtctl.send(sock, f"camera {x} {SIDE_HEIGHT} {z + SIDE_AWAY} -6 0")
        time.sleep(0.8)
        shoot(sock, shots, name + "_from_the_side", t0, line)
    rtctl.send(sock, f"brain follow {who}")   # (`brain follow` alone moves on to the next resident)
    rtctl.send(sock, "sim resume")
    time.sleep(0.5)


def wait_until(sock, who, test, timeout, every=0.2):
    """Polls `brain? <who>` until test(line) is true; the line, or None when `timeout` real seconds pass first."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = rtctl.send(sock, f"brain? {who}", timeout=30)
        if test(line):
            return line
        time.sleep(every)
    return None


def car_speed(line):
    m = re.search(r"([\d.]+) m/s", line)
    return float(m.group(1)) if m else -1.0


def multi(sock, shots, t0, suffix, wait_speed):
    """SEVERAL RESIDENTS (ADR-0443): the contended household car, then the player taking the car from its driver.

    Ana and ben share one car. Both are sent far away a second apart; ben reaches the car first and takes it; ana finds
    it gone at the kerb and walks. While ben waits at a stop on his drive, the PLAYER is put beside the car (`teleport`)
    and takes it with the real key (`tap E`: the interaction broker's "take the car", CityVehicleSystem's promotion,
    CitySim::playerTakesCar). Ben is put out on the kerb side and walks on. Each key moment is taken from above and from
    the side, the names beginning `multi_`."""
    def named(name):
        return f"multi_{name}{suffix}"

    def both(name, line, who, at=None):
        shoot(sock, shots, named(name), t0, line)
        from_above_and_the_side(sock, shots, named(name), t0, line, who=who, at=at)

    print(rtctl.send(sock, "brain day all off"))
    print(rtctl.send(sock, "residents?"))
    print(rtctl.send(sock, "brain follow ana"))
    time.sleep(3.0)
    both("0_ana_and_ben_at_home", rtctl.send(sock, "brain? ana"), "ana")
    # the other household, at `west`: three people on their own spots, their car beside them
    print(rtctl.send(sock, "brain follow cat"))
    time.sleep(2.0)
    both("0_the_trio_at_west", rtctl.send(sock, "brain? cat"), "cat")
    print(rtctl.send(sock, "brain follow ana"))
    time.sleep(1.0)
    print(rtctl.send(sock, "goto ana work"))
    time.sleep(1.0)
    print(rtctl.send(sock, "goto ben ramp"))
    # BEN HAS THE CAR, ANA AT THE KERB FINDS IT GONE
    line = wait_until(sock, "ana", lambda l: "car -- the car is in use" in l, 60)
    if not line:
        print("  ana never found the car gone"); return False
    print("  ana:", line)
    ben = rtctl.send(sock, "brain? ben")
    print("  ben:", ben)
    both("1_ben_has_the_car_ana_finds_it_gone", ben, "ana", at=body_at(ben))
    time.sleep(4.0)
    shoot(sock, shots, named("2_ana_walks_on"), t0, rtctl.send(sock, "brain? ana"))
    # ben sits in the car for the way's two minutes: run them fast, then watch him pull out
    print(rtctl.send(sock, "brain follow ben"))
    rtctl.send(sock, f"sim speed {wait_speed}")
    line = wait_until(sock, "ben", lambda l: state_of(l) == "driving", 120, every=0.1)
    rtctl.send(sock, "sim speed 1")
    line = wait_until(sock, "ben", lambda l: "sliding out of its space" in l or "on the road" in l, 30, every=0.1)
    if line:
        both("3_ben_pulls_out", line, "ben")
    # THE PLAYER TAKES THE CAR at ben's first stop on the drive (a stop line or a red light)
    moving = wait_until(sock, "ben", lambda l: "on the road" in l and car_speed(l) > 4.0, 120)
    took = False
    for attempt in range(6):
        line = wait_until(sock, "ben", lambda l: "on the road" in l and 0.0 <= car_speed(l) < 0.05, 240, every=0.1)
        if not line:
            break
        x, z = body_at(line)
        if attempt == 0:
            both("4_ben_stopped_on_the_drive", line, "ben")
            line = rtctl.send(sock, "brain? ben")
            if not (0.0 <= car_speed(line) < 0.05):
                continue   # it moved off while the pictures were taken: the next stop
            x, z = body_at(line)
        # the player beside the car (2.5 m from its middle), then the key
        print(rtctl.send(sock, f"teleport {x + 2.5} {z}"))
        time.sleep(0.8)
        # the car offers each of its seats; the one that takes it is the driver's ("take the car"). Tab through them.
        for _ in range(8):
            focus = rtctl.send(sock, "interact?")
            if "take the car" in focus:
                break
            rtctl.send(sock, "interact next")
            time.sleep(0.25)
        print(focus)
        if "take the car" not in focus:
            print("  (the driver's seat was not offered here; the next stop)")
            wait_until(sock, "ben", lambda l: car_speed(l) > 2.0, 120)
            continue
        print(rtctl.send(sock, "tap E"))
        time.sleep(1.2)
        line = rtctl.send(sock, "brain? ben")
        if "taken by the player" in line:
            took = True
            break
        print("  (the car was not taken; the next stop)")
        wait_until(sock, "ben", lambda l: car_speed(l) > 2.0, 120)
    if not took:
        print("  THE PLAYER COULD NOT TAKE THE CAR through the key: see the log")
        return False
    print("  taken:", line)
    both("5_the_player_took_the_car_ben_on_the_kerb", line, "ben", at=(x, z))
    time.sleep(5.0)
    line = rtctl.send(sock, "brain? ben")
    both("6_ben_walks_on", line, "ben")
    print(rtctl.send(sock, "vehicle?"))
    print(rtctl.send(sock, "residents?"))
    return True


# WHERE TO LOOK ON brain_crowd at the busy period (ADR-0446): from the level test's look list
# (RT_CROWD_LOG=11 RT_CROWD_MINUTES=11 RT_TEST_FILTER=level_brain_crowd build-viewer/level_tests), the places with the
# most cars near them at minute 11 (the only minute of the busy period with a car near a ramp's merge). x, z of the
# plan; the height the camera looks down from.
CROWD_LOOKS = [
    ("signalled_junction_j0", -268.85, -264.57, 70.0),
    ("stop_signed_junction_j11", -270.0, 60.0, 70.0),
    ("ramp_merge_d0_0_a_off", -567.93, 5.20, 80.0),
]
CROWD_BUSY_MINUTE = 11.0  # in the test's busy period (59-74 cars on the road over minutes 6-11; 62 at minute 11)
CROWD_STANDING_SHOTS = 6  # cars found standing on the road, each taken from above
STANDING_HEIGHT = 40.0    # ...from this high
CHASE_BACK = 14.0         # metres behind the chased car the camera stands
CHASE_UP = 6.0            # ...and above the ground


def crowd(sock, shots, t0, suffix, wait_speed):
    """THE CROWD AT ITS BUSY PERIOD (brain_crowd, ADR-0446): the day run fast to the busiest minute, then the sim held
    still and three places taken straight down from above (a signalled junction, a stop-signed one, a ramp's merge),
    then one resident driving taken from the follow camera behind their car. The pictures' names begin `crowd_`."""
    def named(name):
        return f"crowd_{name}{suffix}"

    print(rtctl.send(sock, "residents?"))
    # the lab's clock to the busy period, fast (the day starts when the level does)
    # (the sim's own clock, from `driving?`'s tick count at 60 a second: the speed asked for is not always reached)
    rtctl.send(sock, f"sim speed {wait_speed}")
    while time.time() - t0 < 1200:
        m = re.search(r"over (\d+) ticks", rtctl.send(sock, "driving?"))
        ticks = int(m.group(1)) if m else 0
        if ticks >= (CROWD_BUSY_MINUTE * 60.0 - 6.0) * 60:
            break
        rtctl.send(sock, f"sim speed {wait_speed if ticks < (CROWD_BUSY_MINUTE * 60.0 - 30.0) * 60 else 1}")
        time.sleep(0.5)
    rtctl.send(sock, "sim speed 1")
    print(rtctl.send(sock, "driving?"))
    rtctl.send(sock, "sim pause")
    rtctl.send(sock, "brain follow off")
    for name, x, z, height in CROWD_LOOKS:
        rtctl.send(sock, f"camera {x} {height} {z} -89 0")
        time.sleep(1.0)
        shoot(sock, shots, named(name + "_from_above"), t0, rtctl.send(sock, "waiting?"))
    # CARS STANDING ON THE ROAD (a red light, a stop sign, the car ahead): the first few residents found so, from above
    # (the viewer's day is not the test's to the tick, so a junction picked from the test's list may be empty)
    standing = 0
    for i in range(350):
        if standing >= CROWD_STANDING_SHOTS:
            break
        line = rtctl.send(sock, f"brain? r{i}", timeout=30)
        if state_of(line) == "driving" and "on the road" in line and 0.0 <= car_speed(line) < 0.05:
            at = body_at(line)
            if not at:
                continue
            standing += 1
            rtctl.send(sock, f"camera {at[0]} {STANDING_HEIGHT} {at[1]} -89 0")
            time.sleep(1.0)
            shoot(sock, shots, named(f"standing_{standing}_r{i}_from_above"), t0, line)
    rtctl.send(sock, "sim resume")
    # ONE CAR CHASED: the first resident found driving on the road
    for i in range(200):
        line = rtctl.send(sock, f"brain? r{i}", timeout=30)
        if state_of(line) == "driving" and "on the road" in line and car_speed(line) > 3.0:
            # (after `brain follow off` and `camera`, `brain follow <who>` did not bring the chase camera back on this
            # run: the camera is put behind the car by hand instead, along the way it went in the last half second)
            print(rtctl.send(sock, f"brain follow r{i}"))
            for shot in range(2):
                p0 = body_at(rtctl.send(sock, f"brain? r{i}"))
                time.sleep(0.5)
                line = rtctl.send(sock, f"brain? r{i}")
                p1 = body_at(line)
                if not p0 or not p1 or math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 0.5:
                    time.sleep(2.0)
                    continue
                fx, fz = p1[0] - p0[0], p1[1] - p0[1]
                n = math.hypot(fx, fz)
                fx, fz = fx / n, fz / n
                rtctl.send(sock, "sim pause")
                yaw = math.degrees(math.atan2(fx, -fz))
                rtctl.send(sock, f"camera {p1[0] - CHASE_BACK * fx} {CHASE_UP} {p1[1] - CHASE_BACK * fz} -14 {yaw}")
                time.sleep(1.0)
                shoot(sock, shots, named(f"chase_r{i}_{shot}"), t0, line)
                rtctl.send(sock, "sim resume")
                time.sleep(4.0)
            print(rtctl.send(sock, "driving?"))
            return True
    print("  nobody was driving on the road to chase")
    return False


def near_cars(line):
    """The physics cars of a `near?` line: [(x, y, z, speed)]."""
    return [(float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)))
            for m in re.finditer(r"car \d+ at (-?[\d.]+) (-?[\d.]+) (-?[\d.]+), (-?[\d.]+) m/s", line)]


def chase(sock, shots, t0, name, x, y, z, heading):
    """The camera CHASE_BACK behind a car at (x, z) facing `heading` (unit x, z), CHASE_UP above, the sim held."""
    fx, fz = heading
    rtctl.send(sock, "sim pause")
    rtctl.send(sock, f"camera {x - CHASE_BACK * fx} {y + CHASE_UP} {z - CHASE_BACK * fz} -14 {math.degrees(math.atan2(fx, -fz))}")
    time.sleep(1.0)
    shoot(sock, shots, name, t0, rtctl.send(sock, "near?"))
    rtctl.send(sock, "sim resume")


def near(sock, shots, t0, suffix, wait_speed):
    """THE NEAR TIER'S TEST BED (ADR-0447) on brain_lab: `near route` puts a physics car on the test's route, standing
    until its light turns red (the sim run fast meanwhile); it is taken from behind as it nears the stop line, standing at
    it, and through the turn. Then `near queue`: a lane car with a physics car behind it, taken from above standing in
    the queue at the red. The pictures' names begin `near_`."""
    def named(name):
        return f"near_{name}{suffix}"

    print(rtctl.send(sock, "brain day all off"))
    print(rtctl.send(sock, "brain follow off"))
    staged = rtctl.send(sock, "near route")
    print(staged)
    m = re.search(r"round connector (\d+)", staged)
    connector = int(m.group(1)) if m else -1
    rtctl.send(sock, f"sim speed {wait_speed}")
    line = wait_until_near(sock, lambda cars: len(cars) >= 1, 400)
    rtctl.send(sock, "sim speed 1")
    if not line:
        print("  the physics car never set out"); return False
    taken = set()
    heading = None
    last = None
    onTurnSince = None
    while time.time() - t0 < 1500 and "turn" not in taken:
        line = rtctl.send(sock, "near?")
        cars = near_cars(line)
        if not cars or "arrived" in line:
            print("  (it arrived before the turn was taken)")
            break
        x, y, z, speed = cars[0]
        pm = re.search(r"path (\d+)", line)
        path = int(pm.group(1)) if pm else -1
        if last and math.hypot(x - last[0], z - last[1]) > 0.05:
            d = (x - last[0], z - last[1])
            n = math.hypot(*d)
            heading = (d[0] / n, d[1] / n)
        last = (x, z)
        if heading and "approach" not in taken and speed > 4:
            taken.add("approach")
            chase(sock, shots, t0, named("1_nearing_the_line"), x, y, z, heading)
        if speed < 0.05 and heading and "approach" in taken and "stopped" not in taken:
            taken.add("stopped")
            chase(sock, shots, t0, named("2_at_the_stop_line"), x, y, z, heading)
            rtctl.send(sock, f"sim speed {wait_speed}")   # (the red is long: run it fast until it moves)
        if speed > 0.5 and "stopped" in taken and "fast_off" not in taken:
            taken.add("fast_off")
            rtctl.send(sock, "sim speed 1")
        # through the turn: on the junction's connector, a moment after it entered it
        if path == connector and onTurnSince is None:
            onTurnSince = time.time()
        if onTurnSince is not None and time.time() - onTurnSince > 0.8 and heading:
            taken.add("turn")
            chase(sock, shots, t0, named("3_through_the_turn"), x, y, z, heading)
        time.sleep(0.05)
    print(rtctl.send(sock, "near?"))
    # THE MIXED QUEUE
    print(rtctl.send(sock, "near queue"))
    rtctl.send(sock, f"sim speed {wait_speed}")
    wait_until_near(sock, lambda cars: len(cars) >= 1, 400)
    rtctl.send(sock, "sim speed 1")
    moving = False
    for _ in range(600):
        cars = near_cars(rtctl.send(sock, "near?"))
        if cars and cars[0][3] > 1.0:
            moving = True
        if cars and moving and cars[0][3] < 0.05:
            x, y, z, _ = cars[0]
            rtctl.send(sock, "sim pause")
            rtctl.send(sock, f"camera {x} {y + 30} {z} -89 0")
            time.sleep(1.0)
            shoot(sock, shots, named("4_the_mixed_queue_from_above"), t0, rtctl.send(sock, "near?"))
            rtctl.send(sock, "sim resume")
            break
        time.sleep(0.1)
    print(rtctl.send(sock, "near?"))
    return "turn" in taken


NEAR_CROWD_MINUTE = 9.0      # the crowd test's busy minutes (near cars most from minute 9 on)
NEAR_CROWD_AT = (-250.0, 6.0)  # beside the signalled junction at -270, 0 the crowd's cars cross


def heading_of(sock, pick):
    """A near car moving faster than 4 m/s, and its heading from two looks half a second apart: (car, (fx, fz))."""
    first = near_cars(rtctl.send(sock, "near?"))
    time.sleep(0.5)
    second = near_cars(rtctl.send(sock, "near?"))
    for a in second:
        if a[3] < pick:
            continue
        b = min(first, key=lambda c: math.hypot(c[0] - a[0], c[2] - a[2]), default=None)
        if not b:
            continue
        d = math.hypot(a[0] - b[0], a[2] - b[2])
        if 0.3 < d < 15.0:
            return a, ((a[0] - b[0]) / d, (a[2] - b[2]) / d)
    return None, None


def things(sock, shots, t0, suffix, wait_speed, log_path):
    """WHAT IS IN A DRIVER'S WAY (ADR-0451) on brain_crowd at its busy minute: the player stands in the lane ahead of a
    moving near car (it stops short and sounds its horn: the viewer's log says so), the same from above as it goes round
    him, and a lane car stopped for a person crossing. Pictures `things_*`."""
    def named(name):
        return f"things_{name}{suffix}"

    print(rtctl.send(sock, "near marks on"))
    print(rtctl.send(sock, "brain labels off"))
    print(rtctl.send(sock, "brain follow off"))
    print(rtctl.send(sock, "detach off"))
    rtctl.send(sock, f"sim speed {wait_speed}")
    while time.time() - t0 < 900:
        m = re.search(r"over (\d+) ticks", rtctl.send(sock, "driving?"))
        if m and int(m.group(1)) >= NEAR_CROWD_MINUTE * 60 * 60:
            break
        time.sleep(0.5)
    rtctl.send(sock, "sim speed 1")
    ok = True
    # 1. THE PLAYER IN THE LANE, 30 m ahead of a moving near car
    car, fwd = None, None
    for attempt in range(40):
        car, fwd = heading_of(sock, 4.0)
        if car:
            break
        time.sleep(0.5)
    if not car:
        print("  no moving near car")
        return False
    fx, fz = fwd
    px, pz = car[0] + 30 * fx, car[2] + 30 * fz
    m = re.search(r"car (\d+) at %.2f" % car[0], rtctl.send(sock, "near?"))
    if m:
        a = re.findall(r"-?[\d.]+", rtctl.send(sock, f"near ahead {m.group(1)} 28"))
        if len(a) >= 2:
            px, pz = float(a[0]), float(a[1])
    print(rtctl.send(sock, f"teleport {px} {pz}"))
    stopped = None
    for i in range(40):
        time.sleep(0.25)
        for c in near_cars(rtctl.send(sock, "near?")):
            if math.hypot(c[0] - px, c[2] - pz) < 12 and c[3] < 0.2:
                stopped = c
        if stopped:
            break
    print("  stopped short:", stopped)
    rtctl.send(sock, "sim pause")
    sx, sz = (stopped[0] + px) / 2 if stopped else px, (stopped[2] + pz) / 2 if stopped else pz
    # from the side: 9 m off the line to the left, a little above, looking across at the two
    cx, cz = sx + 9 * fz, sz - 9 * fx
    yaw = math.degrees(math.atan2(sx - cx, -(sz - cz)))
    rtctl.send(sock, f"camera {cx} 3 {cz} -8 {yaw}")
    time.sleep(1.0)
    shoot(sock, shots, named("1_stopped_short_of_the_player"), t0, rtctl.send(sock, "things?"))
    rtctl.send(sock, "sim resume")
    print(rtctl.send(sock, "detach off"))
    time.sleep(0.5)
    with open(log_path, "r", errors="replace") as f:
        horns = [l.strip() for l in f if "[near] horn" in l]
    print("  horns in the log:", len(horns), horns[-3:])
    # 2. GOING ROUND HIM, from above
    before = re.search(r"swerves (\d+)", rtctl.send(sock, "things?"))
    before = int(before.group(1)) if before else 0
    swerved = False
    for i in range(60):
        time.sleep(0.25)
        m = re.search(r"swerves (\d+)", rtctl.send(sock, "things?"))
        if m and int(m.group(1)) > before:
            swerved = True
            time.sleep(1.2)
            break
    print("  swerved:", swerved)
    rtctl.send(sock, "sim pause")
    rtctl.send(sock, f"camera {px} 22 {pz} -89 0")
    time.sleep(1.0)
    shoot(sock, shots, named("2_going_round_the_player_from_above"), t0, rtctl.send(sock, "things?"))
    rtctl.send(sock, "sim resume")
    print(rtctl.send(sock, "detach off"))
    # 3. A LANE CAR STOPPED FOR A PERSON CROSSING: the player out of the road first
    print(rtctl.send(sock, f"teleport {px + 40 * fz} {pz - 40 * fx}"))
    found = None
    for i in range(240):
        line = rtctl.send(sock, "things?")
        for m in re.finditer(r"lane car (\d+) at (-?[\d.]+) (-?[\d.]+), ([\d.]+) m/s, a thing ([\d.]+) m ahead, moving", line):
            if float(m.group(4)) < 0.1 and float(m.group(5)) < 8:
                found = (float(m.group(2)), float(m.group(3)))
        if found:
            break
        time.sleep(0.25)
    print("  a lane car stopped for someone moving in its way:", found)
    if found:
        rtctl.send(sock, "sim pause")
        rtctl.send(sock, f"camera {found[0]} 18 {found[1]} -89 0")
        time.sleep(1.0)
        shoot(sock, shots, named("3_a_lane_car_stopped_for_a_person_crossing"), t0, rtctl.send(sock, "things?"))
        rtctl.send(sock, "sim resume")
    end = rtctl.send(sock, "near?")
    print(end)
    m = re.search(r"contacts (\d+)", end)
    print("CONTACTS AT THE END:", m.group(1) if m else "?")
    return ok and bool(stopped) and swerved and bool(found)


def near_crowd(sock, shots, t0, suffix, wait_speed):
    """THE NEAR TIER IN THE CROWD (ADR-0448) on brain_crowd: `near marks on` (a cyan post over each physics car), the day
    run fast to the busy minute, the player put beside a busy junction and walked (the game's own chase camera), then
    the same place from above; then the nearest physics car taken with E, driven, stopped, left, and the car after it is
    given back, from above. Pictures `nearcrowd_*`."""
    def named(name):
        return f"nearcrowd_{name}{suffix}"

    print(rtctl.send(sock, "near marks on"))
    print(rtctl.send(sock, "brain labels off"))   # (#299: the places' and residents' names off the pictures)
    print(rtctl.send(sock, "brain follow off"))   # (the brain's spectate camera off r0: the camera is the player's)
    print(rtctl.send(sock, "detach off"))
    rtctl.send(sock, f"sim speed {wait_speed}")
    while time.time() - t0 < 900:
        m = re.search(r"over (\d+) ticks", rtctl.send(sock, "driving?"))
        if m and int(m.group(1)) >= NEAR_CROWD_MINUTE * 60 * 60:
            break
        time.sleep(0.5)
    rtctl.send(sock, "sim speed 1")
    x, z = NEAR_CROWD_AT
    yaw = 0.0
    # (behind a physics car that is moving, if one is, facing the way it goes: the player walks after it down the
    # street, the game's chase camera looking along the street with the car ahead. Its heading from two looks.)
    first = near_cars(rtctl.send(sock, "near?"))
    time.sleep(0.5)
    second = near_cars(rtctl.send(sock, "near?"))
    best = None
    for a in second:
        if a[3] < 2.0:
            continue
        b = min(first, key=lambda c: math.hypot(c[0] - a[0], c[2] - a[2]), default=None)
        if b and 0.3 < math.hypot(a[0] - b[0], a[2] - b[2]) < 15.0:
            best = (a, b)
            break
    if best:
        a, b = best
        d = math.hypot(a[0] - b[0], a[2] - b[2])
        fx, fz = (a[0] - b[0]) / d, (a[2] - b[2]) / d
        # 12 m behind it, to the kerb side (the left of the heading, cars drive on the left: (fz, -fx))
        x, z = a[0] - 12.0 * fx + 3.0 * fz, a[2] - 12.0 * fz - 3.0 * fx
        yaw = math.degrees(math.atan2(fx, -fz))
        print(rtctl.send(sock, f"teleport {x} 2 {z} 0 {yaw}"))
    else:
        print(rtctl.send(sock, f"teleport {x} {z}"))
    time.sleep(0.6)
    rtctl.send(sock, "hold W")
    time.sleep(1.2)
    print(rtctl.send(sock, "near?"))
    shoot(sock, shots, named("1_walking_among_physics_cars"), t0, rtctl.send(sock, "near?"))
    rtctl.send(sock, "release W")
    cam = re.findall(r"-?[\d.]+", rtctl.send(sock, "where?"))
    cx, cy, cz = float(cam[0]), float(cam[1]), float(cam[2])
    rtctl.send(sock, "sim pause")
    rtctl.send(sock, f"camera {cx} 70 {cz} -89 0")
    time.sleep(1.0)
    shoot(sock, shots, named("2_near_and_middle_from_above"), t0, rtctl.send(sock, "near?"))
    # (ADR-0450: close over a near car that is moving, if one is, to see its line beside the parked cars)
    moving = [c for c in near_cars(rtctl.send(sock, "near?")) if c[3] > 2.0]
    if moving:
        mx, my, mz, _ = moving[0]
        rtctl.send(sock, f"camera {mx} {my + 24} {mz} -89 0")
        time.sleep(1.0)
        shoot(sock, shots, named("2b_a_moving_near_car_from_above"), t0, rtctl.send(sock, "near?"))
    rtctl.send(sock, "sim resume")
    print(rtctl.send(sock, "detach off"))   # (the camera back on the player)
    # TAKE THE NEAREST PHYSICS CAR: beside it, E
    cars = near_cars(rtctl.send(sock, "near?"))
    if not cars:
        print("  no physics car to take"); return False
    cars.sort(key=lambda c: math.hypot(c[0] - cx, c[2] - cz))
    took = False
    for attempt in range(8):
        cars = near_cars(rtctl.send(sock, "near?"))
        if not cars:
            break
        cars.sort(key=lambda c: math.hypot(c[0] - cx, c[2] - cz))
        car = cars[0]
        rtctl.send(sock, f"teleport {car[0] + 1.5} {car[2]}")
        time.sleep(0.6)
        # (the broker offered only the hood here, its "get in" out of view: VehicleSystem's own key instead, the
        # nearest car within its reach)
        print(rtctl.send(sock, "near key"))
        time.sleep(1.0)
        if re.search(r"taken [1-9]", rtctl.send(sock, "near?")):
            took = True
            break
    print(rtctl.send(sock, "vehicle?"))
    rtctl.send(sock, "hold W")
    time.sleep(2.5)
    shoot(sock, shots, named("3_driving_a_taken_car"), t0, rtctl.send(sock, "vehicle?"))
    rtctl.send(sock, "release W")
    rtctl.send(sock, "hold Space")
    time.sleep(3.0)
    rtctl.send(sock, "release Space")
    print(rtctl.send(sock, "near key"))
    time.sleep(3.0)
    cam = re.findall(r"-?[\d.]+", rtctl.send(sock, "where?"))
    rtctl.send(sock, "sim pause")
    rtctl.send(sock, f"camera {float(cam[0])} 25 {float(cam[2])} -89 0")
    time.sleep(1.0)
    shoot(sock, shots, named("4_the_car_given_back"), t0, rtctl.send(sock, "near?"))
    rtctl.send(sock, "sim resume")
    end = rtctl.send(sock, "near?")
    print(end)
    m = re.search(r"contacts (\d+)", end)
    print("CONTACTS AT THE END:", m.group(1) if m else "?")
    return took


def wait_until_near(sock, test, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = rtctl.send(sock, "near?", timeout=30)
        if test(near_cars(line)):
            return line
        time.sleep(0.25)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", default="build-viewer/viewer")
    ap.add_argument("--level", default="assets/levels/brain_lab.json")
    ap.add_argument("--to", default="work", help="the marker to send the resident to")
    ap.add_argument("--trait", default="", help="a trait to give them first (has_board)")
    ap.add_argument("--wait-speed", type=float, default=8.0, help="sim speed while they sit in the parked car")
    ap.add_argument("--timeout", type=float, default=900.0, help="real seconds the trip may take")
    ap.add_argument("--shots", default=os.path.expanduser("~/Pictures/Raytracer/brain"))
    ap.add_argument("--log", default="out/zz_brain3/probe_viewer.log")
    ap.add_argument("--scenario", default="trip", choices=["trip", "multi", "crowd", "near", "nearcrowd", "things"],
                    help="trip: one resident's trip (--to); multi: the contended household car, then the player takes "
                         "it; crowd: brain_crowd at its busy period (give --level assets/levels/brain_crowd.json); "
                         "near: the near tier's test bed (ADR-0447): a physics car at the stop line and through the turn, "
                         "then the mixed queue")
    ap.add_argument("--tag", default="", help="a prefix for the pictures' names")
    ap.add_argument("--suffix", default="", help="an ending for the pictures' names (_v2), so an earlier set is kept")
    args = ap.parse_args()
    os.makedirs(args.shots, exist_ok=True)
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    t0 = time.time()
    proc, sock = rtctl.launch(args.viewer, args.level, log=open(args.log, "wb"))
    print("viewer pid", proc.pid, "log", args.log)
    done = False
    tag = args.tag or args.to

    def named(name):
        return f"{tag}_{name}{args.suffix}"

    try:
        rtctl.send(sock, "camera?", timeout=1500)
        while True:
            line = rtctl.send(sock, "brain?", timeout=60)
            if line.startswith("ok") or time.time() - t0 > 1500:
                break
            time.sleep(1.0)
        print(f"loaded in {time.time() - t0:.0f} s: {line}")
        print(rtctl.send(sock, "markers?"))
        if args.scenario == "things":
            done = things(sock, args.shots, t0, args.suffix, args.wait_speed, args.log)
            print("\nprobe", "OK" if done else "FAILED", f"({time.time() - t0:.0f} s in all)")
            return 0 if done else 1
        if args.scenario in ("multi", "crowd", "near", "nearcrowd"):
            run = {"multi": multi, "crowd": crowd, "near": near, "nearcrowd": near_crowd}[args.scenario]
            done = run(sock, args.shots, t0, args.suffix, args.wait_speed)
            print("\nprobe", "OK" if done else "FAILED", f"({time.time() - t0:.0f} s in all)")
            return 0 if done else 1
        # one resident's trip: ana, the first; everybody's day table off, so the other five stand at home
        print(rtctl.send(sock, "brain day all off"))
        print(rtctl.send(sock, "brain follow ana"))
        time.sleep(3.0)
        line = rtctl.send(sock, "brain?")
        shoot(sock, args.shots, named("0_at_home"), t0, line)
        from_above_and_the_side(sock, args.shots, named("0_at_home"), t0, line, above=False)
        if args.trait:
            print(rtctl.send(sock, f"brain trait {args.trait} on"))
        print(rtctl.send(sock, f"goto {args.to}"))
        taken = set()
        speed = 1.0
        entered = {}
        last = ""
        stops = 0
        was_moving = False
        while time.time() - t0 < args.timeout:
            time.sleep(0.25)
            line = rtctl.send(sock, "brain?", timeout=30)
            state = state_of(line)
            now = time.time()
            if state != last:
                print(f"  {now - t0:6.1f} s  {last or '-'} -> {state}")
                entered[state] = entered.get(state, 0) + 1
                last = state
                since = now
            sliding_out = "sliding out of its space" in line
            sliding_in = "sliding into its space" in line
            # run the sim faster only while the person sits in the car in its space
            want = args.wait_speed if (state in ("pulling_out", "parking") and "is standing" in line) else 1.0
            if want != speed:
                rtctl.send(sock, f"sim speed {want}")
                speed = want
            car = re.search(r"([\d.]+) m/s", line)
            car_speed = float(car.group(1)) if car else -1.0

            def once(key, name, above=False, side=False):
                if key in taken:
                    return
                taken.add(key)
                shoot(sock, args.shots, named(name), t0, line)
                if above or side:
                    from_above_and_the_side(sock, args.shots, named(name), t0, line, above, side)

            if state in ("walking", "on_small_wheels") and entered.get(state, 0) == 1 and now - since > 1.5:
                once("walk1", "1_setting_out")
            if state == "pulling_out" and now - since > 1.0:
                once("in_car", "2_in_the_car", above=True, side=True)
            if sliding_out:
                once("out", "3_pulling_out")
            on_road = state == "driving" and "on the road" in line
            if on_road and car_speed > 6.0:
                once("moving", "4_driving")
            if on_road and car_speed > 1.0:
                was_moving = True
            if on_road and was_moving and 0.0 <= car_speed < 0.05 and stops < STOPS_SHOT:
                # each stop on the drive (a stop line, a red light): from behind, then from above
                was_moving = False
                stops += 1
                once(f"stopped{stops}", f"5_stopped_on_the_drive_{stops}", above=True)
            if sliding_in:
                once("in", "6_sliding_in")
            if state == "parking" and "is standing" in line:
                once("parked", "7_in_its_space", above=True, side=True)
            if state == "walking" and entered.get("parking", 0) >= 1 and now - since > 1.0:
                once("walk2", "8_walking_to_the_marker")
            if state == "standing" and "plan done" in line:
                time.sleep(1.0)
                once("there", "9_standing_at_the_marker", side=True)
                done = True
                break
        print("final:", rtctl.send(sock, "brain?"))
    finally:
        rtctl.stop(proc)
    print("\nprobe", "OK" if done else "FAILED", f"({time.time() - t0:.0f} s in all)")
    return 0 if done else 1


if __name__ == "__main__":
    sys.exit(main())
