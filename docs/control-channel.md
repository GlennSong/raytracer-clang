# Viewer Control Channel & MCP

The viewer answers a local socket so an agent (or you, with `nc`) can frame
cameras, take screenshots, toggle overlays, drive the sim clock, and reload
the level in a RUNNING instance — no relaunch loop, no `settings.json`
patching (ADR-0078). `tools/viewer-mcp.py` fronts the socket as an MCP server
so Claude Code sessions get these as first-class tools.

| Question | Tool | Where it works |
|---|---|---|
| "Show me this spot" | `screenshot` (camera + shot in one) | viewer, edit & play |
| "What's running?" | `viewer_status` | any live viewer |
| "What did the planner do?" | `planner_stats` (COVER, rejections) | launched instances (log captured) |
| "Toggle the plan overlay" | `overlay` | citysim layers: play mode only |
| "Freeze traffic for a shot" | `sim` (`pause` / `resume` / `step`) | viewer |
| "Regrow the city" | `reload` | viewer |
| "Make it golden hour" | `daynight` (an hour, `hold`, `run`) | viewer |
| "Is the sun low yet?" | `sun_q` (elevation/intensity probe) | viewer |
| "Crank SSAO / grade / shadows" | `render_params` (set + apply live) | viewer |
| "Any settings knob" | `set` / `get`, `render save` | viewer |
| "Show me the AO buffer" | `view` (+ wireframe) | viewer |
| "Where's the frame time going?" | `ledger` summary / CSV capture | viewer |
| "Give me an avatar" | `possess` car/walker + `drive_to`/`walk_to` | play mode, city levels |

## The socket

Every viewer listens at `/tmp/raytracer-viewer-<pid>.sock` (user-only perms;
`RT_CONTROL=0` disables; newest client wins; unlinked on clean shutdown, and
stale files from a killed viewer are cleaned by the MCP server's discovery).
The protocol is one command line in, one reply line out (`ok ...` / `err ...`).

**Every verb is in one table** (ADR-0253): `commands?` lists them all with their usage, and `commands? json` adds a
one-line summary and the owner of each, for tools. The list below is the original set, with notes; the table is
the truth.


```
ping
info
camera <x> <y> <z> <pitchDeg> <yawDeg>    # stages + detaches the freecam
camera?
shot <path.png>                            # arms; the file lands next frame
overlay <ui|hud|debug|lamplights|master|agents|cones|nav|plan> <on|off>
clouds [on|off] [coverage <0-1>] [density <v>] [scale <v>] [wind <m/s>]
                                  # the cloud deck live; a knob not named is
                                  # left alone (weather other than off wins)
teleport open                     # unfold the Debug window's Teleport section
jitter [agent|-1] [frames]        # record how evenly a vehicle is drawn (-1:
                                  # the one you ride, else the nearest moving);
                                  # jitter? answers when the frames are in
sim <pause|resume|step|speed> [value]
reload
set <key> <value...>              # any Settings key (generic escape hatch)
get <key>
daynight <hour0-24>|hold|run      # one-shots DayNightSystem consumes; work
                                  # while the sim clock is paused. LOCKSTEP:
                                  # an hour jump re-opens the city at that
                                  # hour (population re-placed from its
                                  # schedules); hold holds the city's clock
                                  # too (traffic keeps moving)
daynight minutes <n>              # loop length in REAL minutes (default 30;
                                  # 0 freezes the sun); the citysim's clock
                                  # follows the cycle's hour and rate
daynight day <1-365>              # the calendar day: season + moon phase
daynight terrain on|off           # terrain horizon shadows (mountains) on/off
daynight hud on|off               # the sky HUD (compass + chart). Shows with the
                                  # debug overlay — but stage it (and `camera`)
                                  # BEFORE `overlay debug on`: the overlay is a
                                  # pushed state that pauses the game state's
                                  # update beneath it, which is what consumes
                                  # every one-shot here
daynight chart <path.svg>         # the day's sky chart: sun + moon arcs with
                                  # hour ticks, rise/set marks, both bodies
                                  # now (street-map orientation)
daynight moon <age-days>|auto     # hold the moon's age (0 new, 7.4 first
                                  # quarter, 14.8 full, 22.1 last quarter) or
                                  # let the calendar drive it
teleport <x y z [pitch yaw] | x z> # move the PLAYER (camera when none) to a
                                  # pose; "x z" stands on the ground. The
                                  # Debug > Teleport panel's paste format.
teleport?                         # the last teleport's result
clip <text> / clip?               # set / read the debug UI's clipboard (is the
                                  # OS clipboard bridge alive?)
person first|third|toggle         # on-foot camera (the V key, guard-free);
                                  # persisted as playerThirdPerson
where?                            # the viewpoint as "x y z pitch yaw"
bundle?                           # the level bundle (ADR-0084): last hit/miss with timings, and the root
citymap <path.svg> [layers|all]   # the layered city map from the running level
                                  # (roads,curbs,sidewalks,gaps,nav,furniture,
                                  # objects,blocks,lots,buildings,districts,
                                  # places,conflicts,legend); every layer
                                  # click-toggles; conflicts = the sidewalk
                                  # band inside a carriageway, logged as a census
daynight?                         # the clock as numbers: hour, dayMinutes,
                                  # today's sunrise/sunset, daylight fraction,
                                  # sunY, and the citysim's own hour (sim=)
                                  # — equal when a cycle is staged
weather <clear|fair|overcast|storm|auto|off>
                                  # sky states over the volumetric deck
                                  # (weather_cycle.h): eased in like a front
                                  # (~2 min), sun dims with the deck; auto =
                                  # seeded neighbor walk every 4 in-world
                                  # hours; off returns the knobs to the panel
weather?                          # current state ("storm (auto)" / "off")
sun?                              # sunY / intensity / dark — numeric probe
fog <density> [heightFalloff] [r g b]
                                  # live atmosphere tuning on the level's
                                  # lighting (like `sun`, nothing persists —
                                  # bake keepers into the level JSON); on
                                  # scattering-sky levels the haze fades toward
                                  # the real sky color (sunset/night correct)
fog?                              # enabled / density / heightFalloff / color
render <apply|save>               # push ssao./ssr./shadow./bloom./tonemap.op/
                                  # grade./hud.show settings into the renderer
                                  # (the visionOS panel's static mapping);
                                  # save also persists settings.json
view <0-8> [wire0-2]              # debug views (AO/SSR/depth/...) + wireframe
ledger <start <csv>|stop|summary> # ADR-0077 frame ledger, remotely
possess <car|walker> [x z]        # ADR-0079 avatar: spawn an AI sedan on the
                                  # nearest lane / commandeer a pedestrian;
                                  # the chase camera follows it
drive_to <x> <z>                  # route + drive there (pursuit + sensing)
walk_to <x> <z>                   # sidewalk route (walker)
possess?                          # state: driving/walking/arrived/stuck/
                                  # no-route + pos/speed/remaining/lat/lead
possess_stop | release            # brake and hold / detach brain + camera
```

Possession is PLAY-mode only (edit mode has no nav graph or physics) and
performs transient play acts only — nothing it spawns carries a SourceSpec or
can reach the level document (ADR-0079). The possessed car ignores traffic
signals (v1); sensing keeps it from rear-ending traffic. `stuck` cycles with
a brake-hold/retry — persistent stuck is the director's cue to `drive_to`
somewhere reachable or `release`.

Try it by hand against a running viewer:

```bash
printf 'info\n' | nc -U /tmp/raytracer-viewer-*.sock
```

Threading (ADR-0072 staging): the socket thread only buffers lines;
`Application::runFrame` applies them top-of-frame on the main thread. Effects
land within a frame; `shot` files land a frame later — poll for the file.

## Verbs from Lua

A gameplay script (a ScriptBehaviour) can add verbs to the table and run any:

```lua
function M:start(e)
  control.command{name = "greet", usage = "greet [who]", summary = "Say hello.",
                  run = function(args) return "hello " .. (args[1] or "you") end}
  log(control.run("where?"))
end
```

The verb is listed by `commands?` (owner `script:<entity>`) and reaches the
MCP server's tools at its next relist. `run` gets the words after the verb and
returns the reply; text without `ok`/`err` is sent as `ok <text>`, `nil` as
`ok`, and a Lua error as `err <verb>: <message>`. The verb goes when the
script's entity does.

## Who answers a verb

A command verb (`teleport`, `possess`, `daynight 18`) parses its words into a
typed request and publishes it on the EventBus; the system that does the job
hears it. When this state has no such system (the editor has no player, a
level without a city has no possession), the verb says so:
`err teleport: nothing here takes it (no system for it in this state)`.

A question (`teleport?`, `traffic?`, `possess?`) is listed by the engine and
answered by the system that owns the answer, while that system runs; until
then it replies `err <verb>: nothing answers it here (it needs ...)`. The
city's costly island-wide counts run only while one of the city's own
questions has been asked in the last minute.

The same requests are named events, so `publish TeleportRequest {...}`, a Lua
`events.publish(...)` and `teleport ...` all reach the same system.

Settings holds preferences only. `set`/`get` still reach any key, but no system
watches a key for a command any more: the old idioms (`set clouds.apply 1`,
`set teleport.open 1`, `set jitter.request ...`) are the verbs above, and the
editor's Play Here is an argument to the play state.

## Named events

The EventBus's described types (ADR-0253) are reachable from the socket and
from Lua by name, their fields as JSON:

```
events?                                     # ok 1 events: PlaySound {clip, pitch, position, range, spatial, volume}
publish PlaySound {"clip": "bell", "volume": 0.3}
watch PlaySound                             # then, as they happen:
event PlaySound {"clip":"bell","pitch":1.0,...}
unwatch
```

A watch belongs to its connection: the next client to take the socket ends
it. Lines go out without blocking the frame; a client that stops reading loses
them. In Lua: `events.publish(name, fields)` (delivered at the frame's queued
dispatch), `local id = events.on(name, function(fields) ... end)`,
`events.off(id)`; a script's subscriptions go with its entity.

## The MCP server

`tools/viewer-mcp.py` — standard library only, registered in the repo-root
`.mcp.json`. Claude Code spawns it automatically at session start (first use
asks for approval once).

Its tools are the viewer's command table (ADR-0253): one tool per verb, read
from the newest live viewer's `commands? json`, or from
`web/engine/commands.json` when no viewer runs (a test holds that copy equal
to the built-in table). A question's `?` becomes `_q` (`camera?` is
`camera_q`, marked read-only); every verb tool takes `args`, the rest of the
line as its usage shows it. After `launch_viewer`, `viewer_status` or `reload`
the table is read again, and when it changed the client is told to list again
(`notifications/tools/list_changed`), so a newer build's verbs or a script's
appear without a session restart.

Written by hand are only the composites, which do more than send one line:
`viewer_status`, `launch_viewer`, `screenshot` (frames a pose, waits for the
PNG), `teleport` (stages, then reads `teleport?`), `planner_stats` (the log
scrape), `render_params` (several `set`s and `render apply`), and
`viewer_command` (a raw protocol line).

Instances: every tool takes an optional `instance` (socket basename
substring); default is the newest live viewer — which is how an agent attaches
to the instance a human already has open. `launch_viewer` captures the
viewer's stdout to `/tmp/raytracer-viewer-<pid>.log`, which is what feeds
`planner_stats`; hand-launched viewers still answer every tool except the log
scrape.

## The loop

1. `launch_viewer("assets/levels/metropolis_sky.json", "edit")` — or skip, and
   attach to the viewer that's already open.
2. `screenshot(x, y, z, pitch, yaw)` — frame and look, seconds per glance.
   `overlay("ui", false)` first if the editor panel is in the way.
3. `planner_stats()` — the `[citylots]` COVER line and `[roadgraph]` counts
   without log spelunking.
4. Edit the recipe, `reload()`, look again.

Camera conventions match the shot scripts: pitch −89 is straight down, yaw 0
looks toward −Z, `yaw = atan2(dx, −dz)`. In play mode the staged camera
detaches the freecam (it wins over the player pin); press F in-game to
re-attach.

## Limits (by charter — ADR-0078)

Read-and-look only: no world edits (the EditorBridge owns those), no input
injection, no entity queries. citysim overlays need play mode. Windows has no
socket backend. One client is served at a time; the newest connection wins.
