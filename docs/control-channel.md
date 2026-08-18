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
```

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
`planner_stats`.

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
