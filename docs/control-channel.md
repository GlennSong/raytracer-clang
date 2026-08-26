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
| "Freeze traffic for a shot" | `sim pause` / `resume` / `step` | viewer |
| "Regrow the city" | `reload` | viewer |
| "Make it golden hour" | `time_of_day` (set hour / hold the cycle) | viewer |
| "Is the sun low yet?" | `sun` (elevation/intensity probe) | viewer |
| "Crank SSAO / grade / shadows" | `render_params` (set + apply live) | viewer |
| "Any settings knob" | `settings_kv` get/set/save | viewer |
| "Show me the AO buffer" | `debug_view` (+ wireframe) | viewer |
| "Where's the frame time going?" | `ledger` summary / CSV capture | viewer |
| "Give me an avatar" | `possess` car/walker + `drive_to`/`walk_to` | play mode, city levels |

## The socket

Every viewer listens at `/tmp/raytracer-viewer-<pid>.sock` (user-only perms;
`RT_CONTROL=0` disables; newest client wins; unlinked on clean shutdown, and
stale files from a killed viewer are cleaned by the MCP server's discovery).
The protocol is one command line in, one reply line out (`ok ...` / `err ...`):

```
ping
info
camera <x> <y> <z> <pitchDeg> <yawDeg>    # stages + detaches the freecam
camera?
shot <path.png>                            # arms; the file lands next frame
overlay <ui|hud|debug|master|agents|cones|nav|plan> <on|off>
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

## The MCP server

`tools/viewer-mcp.py` — standard library only, registered in the repo-root
`.mcp.json`. Claude Code spawns it automatically at session start (first use
asks for approval once). Tools: `viewer_status`, `launch_viewer`,
`set_camera`, `get_camera`, `screenshot`, `overlay`, `sim`, `reload`,
`planner_stats`, `time_of_day` (hour / hold / `day_minutes`), `clock`
(`daynight?`), `sun`, `settings_kv`, `render_params`,
`debug_view`, `ledger`, and `viewer_command` (raw protocol line — the escape
hatch that makes engine verbs usable before the shim relearns them at the
next session restart).

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
