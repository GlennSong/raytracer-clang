# How the engine works — authoring the pages

`web/engine/` is a section of the GitHub Pages site (https://glennsong.github.io/raytracer-clang/engine/)
that explains how the engine's systems work, at a high to mid level, for Glenn's own education and as
notes for whoever works on the code next. One page per system, linked from `index.html`.

## What a page is for

A reader who knows programming but not this code should come away knowing:
- what the system does and why it exists (the problem it solves, in plain words);
- the shape of it: the stages data goes through, the main types, who calls whom;
- the key data structures (real code, excerpted) and the few functions that do the real work;
- how it got this way: the dead ends and the measured turning points (from the ADRs and commits);
- where to look: the files, the ADRs, the tests, the lab levels, the debug switches.

High to mid level: explain the idea, then show the real code that carries it. Not an API reference.

## Writing

- Plain, clear English. Define a term the first time it appears ("a CDLOD terrain: the ground drawn as
  one grid whose detail falls off with distance"). Short paragraphs. Say why, not only what.
- Accurate: every claim checked against the code or an ADR. Numbers come from ADRs, commit messages or
  tests, with where they came from. No invented dates or measurements. Use they/them for anyone whose
  pronouns are unknown.
- History goes in `.note.history` boxes or a "How it got here" section: what was tried, what was
  measured, why it changed (ADR numbers link to `docs/decisions.md` on GitHub).

## Page parts (see `_template.html`)

- **Header:** `.nav` (back to the section index and the site), `h1`, `.lede` (two or three sentences).
- **Diagrams:** inline SVG in a `.diagram` div, then a `.cap` caption. At least two per page: a flow
  (the pipeline) and a structure (how the main types relate), more if they help. Colours: boxes
  `fill="#131a24" stroke="#2c3d50"`, highlighted box `stroke="#3b82f6"`, text `fill="#cfe4ff"`
  (secondary `#93a6ba`), arrows `stroke="#6f8fb0"` with a marker, accent `#8ec5ff`, warm accent
  `#e6c07b`. `font-family="system-ui, sans-serif"`, 12-14px. Give it a `viewBox`; width scales.
  Keep text inside its boxes (measure: ~7 px per character at 13 px).
- **Screenshots:** `figure.shot` with `img` + `figcaption`; `.shots` for a row. Images live in
  `img/<page>/` as WebP, at most 1280 px wide, quality ~78, each under ~250 KB:
  `magick in.png -resize '1280x>' -quality 78 img/<page>/name.webp`. Sources: `~/Pictures/Raytracer`
  (Glenn's screenshots and the dated probe folders). Always `alt` text.
- **Code excerpts:** `.code` with a `.hd` header and a `pre`. Code is copied verbatim from the file at
  the pinned commit (elide with `// ...`), HTML-escaped (`&lt;` `&gt;` `&amp;`). Comments may be
  wrapped in `<span class="c">`; keywords/types/strings/numbers spans (`k t s n`) are optional.
  The header links the exact lines at the pinned commit:
  `https://github.com/GlennSong/raytracer-clang/blob/<SHA>/<path>#L<a>-L<b>`
  (check the line numbers with `git show <SHA>:<path> | sed -n 'a,bp'`).
- **Key files:** a `ul.files` list near the end, linking `blob/main/<path>` (the current file).
- **Footer:** the commit the excerpts were taken from, and the ADRs the page draws on.

Pinned commit for the first edition: `47d587f31769b795527e22ef848a82f6401ad112` (main, 2026-10-26).
When a page is revised, re-pin it to the commit it was checked against.

## Mechanics

- Self-contained apart from `engine.css`: no external scripts, fonts or images.
- Must read well at phone width (390 px): no horizontal page scroll; diagrams and code scroll inside
  their boxes. Check with headless Firefox:
  `firefox --headless --no-remote --profile "$(mktemp -d)" --window-size=390,3000 --screenshot out.png file://$PWD/web/engine/<page>.html`
  (and at 1280 wide), then look at the PNGs.
- `web/engine/` is copied whole into the site by CMake (`viewer_web` POST_BUILD) and the
  deploy workflow; a new page needs only a card in `index.html`.
