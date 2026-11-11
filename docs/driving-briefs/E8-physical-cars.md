# E8 — Physics bodies steering along the path

**Read first:** design 2, 12 (near ↔ physical), 3.1 (the player). `src/apps/citysim/city_physics.h`,
`src/engine/vehicle_steering.h`. ADR-0087 (vehicle handling). Brief A2's list of writers.

## The goal

The dozen cars nearest the player are real physics bodies, so a bump feels real. While a car is physical, the body is
the truth and its place on the road is read from the body. There is no second copy of the car.

## What is there now

The old model keeps a planned "ghost" car and a physics body trailing it on a 5 m tether, with a `bodyLag` figure so
followers can find the real body. The ghost and the body disagreeing is the cause of several old bugs, including a
possessed driver freezing. Under the new model there is no ghost.

## Decisions already made

**Reading the body back.** Each tick, the body's position is laid onto its route: search a short way either side of
last tick's place for the nearest point on the paths ahead and behind. That gives which path, how far along, how far
to the side, and the speed along the path. These go into the car's ordinary state, so every other driver sees a
physical car exactly as it sees any car.

**The driver still decides** from that state, by the same rules. Only how the answer is carried out differs:

- *Speed:* throttle and brake toward the acceleration `act` asks for.
- *Steering:* aim at a point on the path some way ahead, and steer along the circle that passes through it (the
  plan's "pure pursuit", with the simple two-axle model of a car to turn that circle into a wheel angle). The
  distance ahead grows with speed, between a floor and a ceiling. All three numbers are data.

**Larger margins.** A body takes time to respond, so it overshoots a little. The rule set's `detail` table has a
margin for physical cars added to stop points and gaps.

**Becoming physical:** a body is made at the car's place, direction and speed. Not during a lane change's first
half, a reversing move, or inside a junction.

**Becoming kinematic again:** only when the body is within a small distance of its path, pointing within a few
degrees of it, and its speed matches. Otherwise it stays physical, even if far from the player.

**A wreck:** a body off its path by more than a distance for more than a time, or upside down, or stopped by damage.
The trip ends with a "crashed" event; the body stays as a path obstacle (design 11). Clearing it is the rule set's
timeout for now (section 18, answer 3).

**The player takes a car:** its trip ends; the body is the player's; drivers see it as an obstacle and a guessed
announcer.

## Traps

- **Slow speeds.** Aiming at a point ahead goes wrong when nearly stopped: the wheel swings about. Keep the floor on
  the distance ahead, and below a crawl speed hold the last wheel angle.
- **Tight turns.** Aiming ahead cuts corners. The distance ahead must shorten on a connector, by its radius.
- **Laying the body onto the wrong path.** Where two paths lie close (side-by-side lanes, a junction), search only
  the paths on the car's route plus the lanes beside it. A body that has ended up in the lane beside is treated as
  having changed lane, and the route is worked out again if needed.
- **Hills and bridges:** search by height as well as plan position.
- **The old tether and ghost** stay for the old model until E9. Under the new model none of that code runs; don't
  share helpers with it.
- **Probing:** use the existing drive probe harness for tuning, with fixed seeds.

## Done when

1. A physical car follows a straight, a curve and a junction turn within a small sideways error (record the worst).
2. It stops for a red light with its front no more than the physical margin past where a kinematic car would.
3. Handing over each way while driving in traffic: no jump in position or speed that a following car reacts to.
4. A physical car shoved sideways by the player's car drives back to its lane and carries on, or becomes a wreck; it
   never snaps back.
5. Followers keep their gap from a physical car as from any other (no `bodyLag`).
6. Ten minutes in the lab with the nearest twelve physical: the measurements table still passes.

## Show Glenn

Stand at a lab junction and watch the nearest cars. Then nudge one with the player's car and watch it recover.
