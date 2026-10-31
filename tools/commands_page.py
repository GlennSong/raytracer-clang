#!/usr/bin/env python3
"""Build web/engine/commands.html, the site's list of control-channel verbs (ADR-0253).

The list comes from web/engine/commands.json, which a test holds equal to the command table
(tests/test_control_channel.cpp, control_site_command_list_matches_the_table). After the table changes:

  RT_WRITE_COMMANDS_JSON=1 RT_TEST_FILTER=control_site ./build-viewer/run_tests
  python3 tools/commands_page.py --pin <commit>

--pin names the commit the code excerpt and the "how it gets there" column are read from (default HEAD).
How each verb reaches its system is read from its handler in src/engine/application_commands.cpp:
a verb that publishes a typed request sends it to the system that does the job, a question with the engine's
`unanswered` default is answered by the system that owns it, one that still writes a Settings key is a mailbox
and one that returns a Settings string an answer board; anything else acts directly (ADR-0253).
"""
import argparse
import html
import json
import os
import re
import subprocess

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GH = "https://github.com/GlennSong/raytracer-clang/blob"
SRC = "src/engine/application_commands.cpp"

FAMILIES = [
    ("Connection", "Is it there, what is it, what can it do.", ["ping", "info", "commands?"]),
    ("Camera and pictures", "Where the eye is, what it shows, and saving frames.",
     ["camera", "camera?", "where?", "detach", "person", "shot", "shots", "overlay", "view"]),
    ("Time, sky and weather", "The sim clock, the day/night cycle, the sun, fog and clouds.",
     ["sim", "daynight", "daynight?", "sun", "sun?", "fog", "fog?", "clouds", "weather", "weather?"]),
    ("The player", "Moving, pressing keys, getting into things.",
     ["teleport", "teleport?", "tap", "hold", "move?", "ragdoll", "interact", "interact?", "vehicle_spawn",
      "vehicle?", "ride", "train_board", "elevator", "elevator?"]),
    ("Possession", "Take over one of the city's people or cars and direct them (ADR-0079, ADR-0091).",
     ["possess", "possess?", "agent?", "look?", "walk_to", "drive_to", "direct", "board", "alight", "enter", "exit",
      "possess_stop", "release"]),
    ("Asking the city", "The living city's counts and diagnostics, and finding things in it.",
     ["walkers?", "flicker?", "speeders?", "activities?", "knock", "knock?", "traffic?", "parking?", "depots?",
      "trains?", "opensigns?", "jitter", "jitter?", "who?", "pin", "unpin", "pin?", "findplace", "findplace?"]),
    ("Asking the ground and the buildings", "What stands where, and maps of it.",
     ["lot?", "towers?", "recipes?", "ground?", "citymap"]),
    ("Events", "The bus's named events (ADR-0253): list them, publish one, stream them as they happen.",
     ["events?", "publish", "watch", "unwatch"]),
    ("Engine knobs and health", "Settings, the renderer's knobs, memory, frame time, sound, keys, the bake.",
     ["set", "get", "render", "reload", "mem?", "ledger", "audio?", "keys?", "bundle?", "clip", "clip?"]),
]

KIND_TEXT = {
    "direct": ("acts directly", "The handler does the work itself: the renderer, the clock, the world."),
    "request": ("typed request", "The handler publishes a typed request on the EventBus; the system that does the job hears it."),
    "answered": ("a system answers", "The system that owns the answer binds this question while it runs."),
    "mailbox": ("settings mailbox", "The handler writes a Settings key; a system finds it next frame and clears it."),
    "board": ("answer board", "The handler returns a string a system keeps rewriting into Settings."),
}


def row_id(name):
    """A link target per verb: `teleport` and `teleport?` must differ (the question gets -q)."""
    return "v-" + re.sub(r"[^a-z0-9_]", "", name.replace("?", "_q"))


def git_show(pin, path):
    return subprocess.run(["git", "show", f"{pin}:{path}"], cwd=REPO, check=True, capture_output=True,
                          text=True).stdout


def handler_bodies(src):
    """name -> handler text, from each commandTable.add("name" (shared handlers: the lambda above them)."""
    bodies = {}
    starts = [(m.start(), m.group(1)) for m in re.finditer(r'commandTable\.add\("([^"]+)"', src)]
    for i, (at, name) in enumerate(starts):
        end = starts[i + 1][0] if i + 1 < len(starts) else len(src)
        text = src[at:end]
        if re.match(r'commandTable\.add\("[^"]+", "(?:[^"\\]|\\.)*",\s*"(?:[^"\\]|\\.)*", h\);', text):
            block_start = src.rfind("const ControlCommands::Handler h", 0, at)
            text = src[block_start:at]
        else:
            close = text.find("\n    });")
            text = text[:close] if close >= 0 else text
        bodies[name] = text
    return bodies


# Verbs whose job IS Settings (the generic knob and the render keys) are not the string bus.
SETTINGS_VERBS = {"set", "get", "render"}


def kind_of(body):
    if "unanswered(" in body:
        return "answered"
    if "publishRequest(" in body or "eventBus.publish(" in body:
        return "request"
    if re.search(r'settingsStore\.set(String|Double|Bool)\(', body):
        return "mailbox"
    # an answer board: a string some system keeps rewriting (`levelPath` is a real setting, not a board)
    if any(k != "levelPath" for k in re.findall(r'settingsStore\.getString\("([^"]+)"', body)):
        return "board"
    return "direct"


def excerpt(src, name):
    """The registration of one verb: its line range and text."""
    lines = src.split("\n")
    a = next((i for i, l in enumerate(lines) if l.startswith(f'    commandTable.add("{name}"')), None)
    if a is None:
        raise SystemExit(f"commands_page: no `{name}` registration in {SRC} at the pin, for the excerpt")
    b = next(i for i in range(a, len(lines)) if lines[i] == "    });")
    return a + 1, b + 1, "\n".join(lines[a:b + 1])


def code_html(text):
    text = "\n".join(l[4:] if l.startswith("    ") else l for l in text.split("\n"))
    out = []
    for m in re.finditer(r'(//[^\n]*)|("(?:[^"\\\n]|\\.)*")|([^/"]+|/|")', text):
        piece = html.escape(m.group(0), quote=False)
        out.append(f'<span class="c">{piece}</span>' if m.group(1) else
                   f'<span class="s">{piece}</span>' if m.group(2) else piece)
    return "".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pin", default="HEAD")
    args = ap.parse_args()
    pin = subprocess.run(["git", "rev-parse", args.pin], cwd=REPO, check=True, capture_output=True,
                         text=True).stdout.strip()
    cmds = json.load(open(os.path.join(REPO, "web/engine/commands.json")))
    by_name = {c["name"]: c for c in cmds}
    src = git_show(pin, SRC)
    bodies = handler_bodies(src)
    # The list and the code must be the same version: the "how it gets there" column reads each verb's handler
    # at the pin. (Verbs a system or script registers elsewhere are owned by someone other than the engine.)
    engine_verbs = {n for n, c in by_name.items() if c["owner"] == "engine"}
    if engine_verbs != set(bodies):
        missing = sorted(engine_verbs - set(bodies)); extra = sorted(set(bodies) - engine_verbs)
        raise SystemExit(f"commands_page: commands.json and {SRC} at {pin[:8]} disagree "
                         f"(not in the code: {missing}; not in the list: {extra}). Regenerate commands.json "
                         f"and pin the commit that has the code.")
    kinds = {n: "direct" if n in SETTINGS_VERBS or by_name[n]["owner"] != "engine" else kind_of(bodies.get(n, ""))
             for n in by_name}

    placed = set()
    families = []
    for title, blurb, names in FAMILIES:
        present = [n for n in names if n in by_name]
        placed.update(present)
        families.append((title, blurb, present))
    rest = sorted(n for n in by_name if n not in placed)
    if rest:
        families.append(("Other", "Verbs not yet sorted into a family here (a new one, or one a script added).", rest))

    counts = {k: sum(1 for n in by_name if kinds[n] == k) for k in KIND_TEXT}
    questions = sum(1 for n in by_name if n.endswith("?"))
    a, b, ex = excerpt(src, "teleport")
    short = pin[:8]

    rows = []
    for title, blurb, names in families:
        anchor = re.sub(r'[^a-z]+', '-', title.lower()).strip('-')
        rows.append(f'  <h2 id="{anchor}">{html.escape(title)} <span class="count">{len(names)}</span></h2>')
        rows.append(f'  <p>{html.escape(blurb)}</p>')
        rows.append('  <table class="verbs">')
        rows.append('    <tr><th>Usage</th><th>What it does</th><th>How it gets there today</th></tr>')
        for n in names:
            c = by_name[n]
            k = kinds[n]
            rows.append(f'    <tr id="{row_id(n)}"><td><code>{html.escape(c["usage"])}</code></td>'
                        f'<td>{html.escape(c["summary"])}</td>'
                        f'<td><span class="kind {k}">{KIND_TEXT[k][0]}</span></td></tr>')
        rows.append('  </table>')

    toc = " · ".join(
        f'<a href="#{re.sub(r"[^a-z]+", "-", t.lower()).strip("-")}">{html.escape(t)}</a>' for t, _, _ in families)

    page = f'''<!DOCTYPE html>
<!--
  GENERATED by tools/commands_page.py from web/engine/commands.json (the command table, ADR-0253).
  Do not edit by hand: change the verb in src/engine/application_commands.cpp, then regenerate.
-->
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Control Verbs — How the Engine Works</title>
  <link rel="stylesheet" href="engine.css">
  <style>
    .count {{ font-size: 14px; color: var(--dim); font-weight: 400; margin-left: 6px; }}
    table.verbs td:first-child {{ white-space: nowrap; }}
    table.verbs td:first-child code {{ white-space: pre-wrap; }}
    .kind {{ display: inline-block; font-size: 12px; padding: 1px 8px; border-radius: 9px; white-space: nowrap;
             border: 1px solid var(--line2); }}
    .kind.direct {{ color: #8fd8c8; border-color: #2f6a5c; }}
    .kind.request {{ color: #9cc4ff; border-color: #2f4a6a; }}
    .kind.answered {{ color: #c8b4ff; border-color: #4a3d6a; }}
    .kind.mailbox {{ color: #e6c07b; border-color: #6a5a2f; }}
    .kind.board {{ color: #ff9fb2; border-color: #6a2f3d; }}
    .tally {{ display: flex; gap: 10px; flex-wrap: wrap; margin: 14px 0; }}
    .tally div {{ background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 10px 14px;
                  flex: 1 1 150px; }}
    .tally b {{ display: block; font-size: 24px; color: var(--text); }}
    .tally span {{ font-size: 13px; color: var(--muted); }}
  </style>
</head>
<body>
<div class="wrap">
  <div class="nav"><a href="index.html">← How the engine works</a> · <a href="spine.html">the spine</a> · <a href="../index.html">scenes</a></div>
  <h1>Control verbs</h1>
  <p class="lede">A running viewer listens on a local socket, and anything that can write a line of text can drive
    it: a probe script, Claude through the MCP server, or you with <code>nc</code>. These are all {len(cmds)} things
    it can be told, straight from the engine's command table, where every verb is registered once with its usage
    and what it does.</p>

  <div class="tally">
    <div><b>{len(cmds)}</b><span>verbs in all</span></div>
    <div><b>{questions}</b><span>questions (ending in <code>?</code>)</span></div>
    <div><b>{counts["direct"]}</b><span>act directly</span></div>
    <div><b>{counts["request"]}</b><span>send a typed request</span></div>
    <div><b>{counts["answered"]}</b><span>answered by their system</span></div>
  </div>

  <h2>How a line gets in</h2>
  <div class="diagram">
    <svg viewBox="0 0 860 210" role="img" aria-label="A client writes a line to the socket; the socket thread buffers it; on the main thread the front door parses it and the command table runs the verb's handler, which acts directly or publishes a typed request the system that does the job hears; the reply goes back as one line">
      <defs><marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M0,0 L10,5 L0,10 z" fill="#6f8fb0"/></marker></defs>
      <g font-family="system-ui, sans-serif" font-size="13" text-anchor="middle">
        <rect x="10" y="20" width="150" height="74" rx="9" fill="#131a24" stroke="#2c3d50"/>
        <text x="85" y="44" fill="#cfe4ff">A client</text>
        <text x="85" y="62" fill="#93a6ba" font-size="12">probe · MCP · nc</text>
        <text x="85" y="80" fill="#93a6ba" font-size="12">"teleport 20 600"</text>
        <rect x="200" y="20" width="140" height="74" rx="9" fill="#131a24" stroke="#2c3d50"/>
        <text x="270" y="44" fill="#cfe4ff">Socket thread</text>
        <text x="270" y="62" fill="#93a6ba" font-size="12">only buffers</text>
        <text x="270" y="80" fill="#93a6ba" font-size="12">the line</text>
        <rect x="380" y="20" width="140" height="74" rx="9" fill="#131a24" stroke="#2c3d50"/>
        <text x="450" y="44" fill="#cfe4ff">Front door</text>
        <text x="450" y="62" fill="#93a6ba" font-size="12">main thread,</text>
        <text x="450" y="80" fill="#93a6ba" font-size="12">top of the frame</text>
        <rect x="560" y="20" width="140" height="74" rx="9" fill="#131a24" stroke="#3b82f6"/>
        <text x="630" y="44" fill="#cfe4ff">Command table</text>
        <text x="630" y="62" fill="#93a6ba" font-size="12">one handler</text>
        <text x="630" y="80" fill="#93a6ba" font-size="12">per name</text>
        <rect x="740" y="20" width="110" height="74" rx="9" fill="#131a24" stroke="#2c3d50"/>
        <text x="795" y="52" fill="#cfe4ff">Reply</text>
        <text x="795" y="70" fill="#93a6ba" font-size="12">"ok ..." / "err ..."</text>
        <line x1="160" y1="57" x2="196" y2="57" stroke="#6f8fb0" stroke-width="1.6" marker-end="url(#ar)"/>
        <line x1="340" y1="57" x2="376" y2="57" stroke="#6f8fb0" stroke-width="1.6" marker-end="url(#ar)"/>
        <line x1="520" y1="57" x2="556" y2="57" stroke="#6f8fb0" stroke-width="1.6" marker-end="url(#ar)"/>
        <line x1="700" y1="57" x2="736" y2="57" stroke="#6f8fb0" stroke-width="1.6" marker-end="url(#ar)"/>
        <rect x="430" y="130" width="190" height="60" rx="9" fill="#131a24" stroke="#2f6a5c"/>
        <text x="525" y="156" fill="#8fd8c8">acts directly</text>
        <text x="525" y="174" fill="#93a6ba" font-size="12">renderer · clock · world</text>
        <rect x="640" y="130" width="210" height="60" rx="9" fill="#131a24" stroke="#2f4a6a"/>
        <text x="745" y="156" fill="#9cc4ff">a typed request</text>
        <text x="745" y="174" fill="#93a6ba" font-size="12">on the bus, to its system</text>
        <line x1="610" y1="96" x2="560" y2="126" stroke="#6f8fb0" stroke-width="1.4" marker-end="url(#ar)"/>
        <line x1="650" y1="96" x2="710" y2="126" stroke="#6f8fb0" stroke-width="1.4" marker-end="url(#ar)"/>
      </g>
    </svg>
  </div>
  <p class="cap">One line in, one line out. The socket thread never touches the engine; every verb runs on the main
    thread, where the world, the renderer and Settings are safe to touch (ADR-0072's staging rule).</p>

  <h2>What a verb looks like</h2>
  <div class="code">
    <div class="hd"><span>The teleport verb: its name, usage and summary, then the handler</span>
      <a href="{GH}/{pin}/{SRC}#L{a}-L{b}">{SRC}</a></div>
<pre>{code_html(ex)}</pre>
  </div>
  <p>Registering a name a second time is refused and logged, so one verb can never have two handlers. That is how
    <code>ride</code> went wrong in the old if-chain: it was handled twice and the second handler never ran.</p>

  <h2>Try it</h2>
  <div class="code">
    <div class="hd"><span>From a terminal, with a viewer running</span></div>
<pre>printf 'commands?\\n' | nc -U /tmp/raytracer-viewer-*.sock
printf 'teleport 20 600\\n' | nc -U /tmp/raytracer-viewer-*.sock
printf 'teleport?\\n' | nc -U /tmp/raytracer-viewer-*.sock</pre>
  </div>
  <p>A Claude session in the repo gets the same verbs as tools through <code>tools/viewer-mcp.py</code>
    (<a href="{GH}/main/docs/control-channel.md">docs/control-channel.md</a>).</p>

  <h2>Where it is going</h2>
  <p>The last column shows how each verb reaches the system that does the work. Most used to go through the
    <b>Settings string bus</b>: the verb wrote a named Settings key that a system checked every frame, or read a
    string a system kept rewriting. ADR-0253 retired it, with the text on the wire unchanged. Now a command
    <span class="kind direct">acts directly</span> or sends a <span class="kind request">typed request</span> on the
    EventBus, which the system that does the job hears (with no such system in this state, the verb says so), and a
    question is <span class="kind answered">answered by its system</span> while that system runs. The same requests
    are named events: <code>events?</code> lists them, <code>publish</code> sends one, <code>watch</code> streams
    them as they happen, and Lua scripts can send and hear them too, or add verbs of their own. The MCP server
    builds its tools from this same list.</p>

  <p>Families: {toc}</p>

{chr(10).join(rows)}

  <h2>Where to look</h2>
  <ul class="files">
    <li><a href="{GH}/main/src/engine/control/control_commands.h">src/engine/control/control_commands.h</a> — the command table</li>
    <li><a href="{GH}/main/src/engine/application_commands.cpp">src/engine/application_commands.cpp</a> — the built-in verbs</li>
    <li><a href="{GH}/main/src/engine/control/control_channel.h">src/engine/control/control_channel.h</a> — the socket and its threading</li>
    <li><a href="{GH}/main/tools/viewer-mcp.py">tools/viewer-mcp.py</a> — the MCP server over the socket</li>
    <li><a href="{GH}/main/tests/test_control_channel.cpp">tests/test_control_channel.cpp</a> — the table's tests, and the check that this list matches it</li>
  </ul>

  <footer>Generated by <code>tools/commands_page.py</code> from the command table at commit <code>{short}</code>.
    Drawn from ADR-0078 and ADR-0253 (<a href="{GH}/main/docs/decisions.md">docs/decisions.md</a>).</footer>
</div>
</body>
</html>
'''
    open(os.path.join(REPO, "web/engine/commands.html"), "w").write(page)
    print(f"web/engine/commands.html: {len(cmds)} verbs, {counts}, pinned {short}")


if __name__ == "__main__":
    main()
