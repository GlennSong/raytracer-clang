#!/usr/bin/env python3
"""File a field report (the game's F9, engine/field_report.h) as a GitHub issue.

    python3 tools/file_report.py <report folder> [--dry-run] [--again]

The report folder holds shot.png, report.json and note.md. The issue is the note, the teleport line and the
facts (note.md), titled from report.json, labelled `field-report`. GitHub's API cannot attach an image to an
issue, so the shot is committed to the `issue-shots` branch (shots/<id>.png) with git plumbing -- a private
index, no checkout, the working tree and its uncommitted files untouched -- pushed, and linked from the body.
The repo is public: so is the shot.

The issue's URL is written to <folder>/issue.txt; a folder that has one is not filed twice (--again does).
The game runs this in the background when "File a GitHub issue" is ticked; run it by hand for a report saved
without it.
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRANCH = "issue-shots"
LABEL = "field-report"


def run(args, env=None, check=True, input=None):
    r = subprocess.run(args, cwd=REPO, env=env, input=input, capture_output=True, text=True)
    if check and r.returncode != 0:
        raise SystemExit(f"{' '.join(args)} failed: {r.stderr.strip() or r.stdout.strip()}")
    return r.stdout.strip()


def push_shot(shot, report_id, title, dry):
    """Commit shot to BRANCH as shots/<id>.png on top of the remote branch; return its raw URL."""
    run(["git", "fetch", "-q", "origin", BRANCH], check=False)
    parent = run(["git", "rev-parse", "--verify", "-q", f"refs/remotes/origin/{BRANCH}"], check=False)
    with tempfile.TemporaryDirectory() as tmp:
        env = dict(os.environ, GIT_INDEX_FILE=os.path.join(tmp, "index"))
        if parent:
            run(["git", "read-tree", parent], env=env)
        blob = run(["git", "hash-object", "-w", shot])
        path = f"shots/{report_id}.png"
        run(["git", "update-index", "--add", "--cacheinfo", f"100644,{blob},{path}"], env=env)
        tree = run(["git", "write-tree"], env=env)
    cmd = ["git", "commit-tree", tree, "-m", f"Field report {report_id}: {title}"]
    if parent:
        cmd[3:3] = ["-p", parent]
    commit = run(cmd)
    slug = run(["gh", "repo", "view", "--json", "nameWithOwner", "-q", ".nameWithOwner"])
    if dry:
        print(f"[dry run] would push {commit} to {BRANCH} ({path})")
    else:
        run(["git", "push", "-q", "origin", f"{commit}:refs/heads/{BRANCH}"])
    return f"https://github.com/{slug}/blob/{BRANCH}/{path}?raw=true"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    dry = "--dry-run" in sys.argv
    again = "--again" in sys.argv
    if len(args) != 1:
        raise SystemExit(__doc__)
    folder = os.path.abspath(args[0])
    issue_txt = os.path.join(folder, "issue.txt")
    if os.path.exists(issue_txt) and not again and not dry:
        print(f"already filed: {open(issue_txt).read().strip()} (--again files it once more)")
        return
    with open(os.path.join(folder, "report.json")) as f:
        report = json.load(f)
    with open(os.path.join(folder, "note.md")) as f:
        body = f.read()
    if body.startswith("# "):   # the title line: the issue has its own
        body = body.split("\n", 1)[1].lstrip("\n")
    title = report.get("title") or f"Field report {report.get('id', '')}"

    shot = os.path.join(folder, "shot.png")
    if os.path.exists(shot) and os.path.getsize(shot) > 0:
        url = push_shot(shot, report.get("id") or os.path.basename(folder), title, dry)
        body = f"![screenshot]({url})\n\n" + body

    if dry:
        print(f"[dry run] title: {title}\n{body}")
        return
    run(["gh", "label", "create", LABEL, "--color", "5319e7", "--force",
         "--description", "Filed from the game's F9 field report"], check=False)
    issue = run(["gh", "issue", "create", "--title", title, "--label", LABEL, "--body-file", "-"], input=body)
    with open(issue_txt, "w") as f:
        f.write(issue + "\n")
    print(issue)


if __name__ == "__main__":
    main()
