#!/usr/bin/env python3
"""Generate proto/roadlab/rl_paint.wgsl from proto/roadlab/rl_paint.h.

rl_paint.h is written in a subset that compiles as C++, MSL and GLSL unchanged.
WGSL is the one backend that cannot take it verbatim: `float x` is `x : f32`,
`T f(a)` is `fn f(a) -> T`, struct members are comma-separated, and a pointer
parameter needs an address space. So WGSL gets generated instead of included.

This is a translator for exactly the subset that file uses, and its most
important property is that it REFUSES anything else. A translator that guesses
would emit WGSL that compiles and shades the road differently from the CPU
reference, which is the one failure mode the whole shared-source exercise exists
to prevent. Every unrecognised line raises.

  tools/roadlab-wgsl.py            # write proto/roadlab/rl_paint.wgsl
  tools/roadlab-wgsl.py --check    # exit 1 if the checked-in file is stale

The generated file carries a digest of its input, and tests.cpp recomputes it,
so a header edit that nobody regenerated fails the suite rather than shipping.
"""

import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "proto", "roadlab", "rl_paint.h")
DST = os.path.join(ROOT, "proto", "roadlab", "rl_paint.wgsl")

# The two functions that are mapped rather than translated. Their C bodies use
# the ternary that the rest of the file is written to avoid; in WGSL they are
# builtins. Keeping the mapping to exactly two is what lets everything above
# them be mechanical.
PRIMITIVES = {
    "rlMax": ("fn rlMax(a : f32, b : f32) -> f32 { return max(a, b); }"),
    "rlMin": ("fn rlMin(a : f32, b : f32) -> f32 { return min(a, b); }"),
}

# C names with a different spelling in WGSL. `floor` and `sqrt` carry over.
BUILTINS = {"fabs": "abs"}

TYPES = {"float": "f32", "int": "i32", "void": "void"}


class Refusal(Exception):
    """The header contains something this translator will not guess at.

    Carries its position so a refusal raised deep in an expression can be
    re-pointed at the file line that contains it — an error that says "line 0"
    is an error someone has to go hunting for."""

    def __init__(self, lineno, line, why):
        self.lineno = lineno
        self.line = line
        self.why = why
        super().__init__(
            "rl_paint.h:%s refuses translation (%s)\n    %s\n"
            "Either write it in the subset the generator handles, or teach\n"
            "tools/roadlab-wgsl.py the construct — do not hand-edit the WGSL."
            % (lineno if lineno else "?", why, line.strip())
        )

    def at(self, lineno, line):
        """Re-raise pointing at the real file line, keeping the reason."""
        if self.lineno:
            return self
        return Refusal(lineno, line, self.why)


def digest(data):
    """FNV-1a, 64-bit, over the header's bytes.

    Not a cryptographic choice — nobody is attacking a build stamp. It is here
    because tests.cpp has to recompute it, roadlab depends on nothing but the
    STL, and sixty lines of SHA-256 in a test is a worse trade than eight lines
    of FNV. Bytes rather than text so the em-dashes in the comments hash the
    same on both sides."""
    h = 0xCBF29CE484222325
    for byte in data:
        h = ((h ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return "%016x" % h


def split_comment(line):
    """Peel a trailing // comment off, ignoring one inside a string (there are
    none in this file, but a silent mangling would be worse than an assert)."""
    if '"' in line:
        raise Refusal(0, line, "string literals are not part of the subset")
    i = line.find("//")
    if i < 0:
        return line, ""
    return line[:i], line[i:]


FLOAT_LITERAL = re.compile(r"(\d*\.?\d+(?:[eE][-+]?\d+)?)f\b")
IDENT = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\b")


def translate_expr(expr, ptr_params):
    """Expression text, C to WGSL. Textual because the subset is small enough
    that it can be: no casts, no compound literals, no ternaries."""
    if "?" in expr:
        raise Refusal(0, expr, "ternary — use rlMax/rlMin or an if")
    if "&" in expr.replace("&&", ""):
        raise Refusal(0, expr, "address-of — out-parameters are not in the subset")
    out = FLOAT_LITERAL.sub(r"\1", expr)

    def sub_ident(m):
        name = m.group(1)
        if name in BUILTINS:
            return BUILTINS[name]
        if name in ptr_params:
            # `bounds[i]` is a dereference in WGSL. The pointer is to the whole
            # fixed-size array, which is the one pointer form core WGSL takes as
            # a parameter without an extension.
            return "(*" + name + ")"
        return name

    return IDENT.sub(sub_ident, out)


def translate_type(ctype, lineno, line):
    ctype = ctype.replace("struct ", "").strip()
    if ctype in TYPES:
        return TYPES[ctype]
    if re.fullmatch(r"Rl[A-Za-z]+", ctype):
        return ctype
    raise Refusal(lineno, line, "unknown type %r" % ctype)


PARAM = re.compile(
    r"^\s*(?:(RL_BUF|const)\s+)?(?:(struct)\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*"
    r"(\*?)\s*([A-Za-z_][A-Za-z0-9_]*)\s*$"
)


def translate_params(text, lineno, line):
    """C parameter list to WGSL, reporting which names became pointers."""
    params, ptrs = [], set()
    if text.strip():
        for raw in text.split(","):
            m = PARAM.match(raw)
            if not m:
                raise Refusal(lineno, line, "parameter %r" % raw.strip())
            _qual, _st, ctype, star, name = m.groups()
            wtype = translate_type(ctype, lineno, line)
            if star:
                # A flat array of records. Length is RL_MAX_BOUNDS because a GPU
                # record has to be fixed-size; `count` says how many are live.
                params.append(
                    "%s : ptr<function, array<%s, RL_MAX_BOUNDS>>" % (name, wtype)
                )
                ptrs.add(name)
            else:
                params.append("%s : %s" % (name, wtype))
    return params, ptrs


DEFINE = re.compile(r"^#define\s+(RL_[A-Z_0-9]+)\s+(-?\d+)\s*$")
STRUCT_OPEN = re.compile(r"^struct\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{\s*$")
FIELD = re.compile(r"^\s*(float|int)\s+([A-Za-z_][A-Za-z0-9_]*)\s*;\s*$")
FN_OPEN = re.compile(
    r"^RL_FN\s+(?:(struct)\s+)?([A-Za-z_][A-Za-z0-9_]*)\s+"
    r"([A-Za-z_][A-Za-z0-9_]*)\s*\((.*)$"
)
DECL = re.compile(
    r"^(?:(struct)\s+)?(float|int|Rl[A-Za-z]+)\s+([A-Za-z_][A-Za-z0-9_]*)"
    r"\s*(?:=\s*(.+?))?\s*;\s*$"
)
FOR = re.compile(
    r"^for\s*\(\s*int\s+([A-Za-z_]\w*)\s*=\s*(.+?)\s*;\s*(.+?)\s*;\s*\+\+(\w+)\s*\)\s*\{\s*$"
)
ASSIGN = re.compile(r"^([A-Za-z_][\w.]*)\s*=\s*(.+?)\s*;\s*$")
COMPOUND = re.compile(r"^([A-Za-z_][\w.]*)\s*([-+*/])=\s*(.+?)\s*;\s*$")
IF_HEAD = re.compile(r"^(\}\s*else\s+)?if\s*\(")
ELSE_OPEN = re.compile(r"^\}\s*else\s*\{\s*$")
RETURN = re.compile(r"^return\s*(.*?)\s*;\s*$")


def split_condition(text, lineno, raw):
    """`if (a > -(b + c)) rest` -> ('a > -(b + c)', 'rest'). A regex cannot do
    this: the condition contains parentheses of its own, and a non-greedy match
    stops at the first one, silently truncating the test."""
    open_at = text.index("(")
    depth = 0
    for k in range(open_at, len(text)):
        if text[k] == "(":
            depth += 1
        elif text[k] == ")":
            depth -= 1
            if depth == 0:
                return text[open_at + 1:k].strip(), text[k + 1:].strip()
    raise Refusal(lineno, raw, "unbalanced condition")


def reassigned(body_lines, name):
    """Does anything below reassign this local? WGSL wants `let` for the ones
    that do not, and rejects assignment to them — so getting this wrong is a
    compile error rather than a wrong road, which is the right way round."""
    pat = re.compile(r"^\s*" + re.escape(name) + r"\s*(\.\w+\s*)?=[^=]")
    return any(pat.match(l) for l in body_lines)


def generate(src_text):
    lines = src_text.split("\n")
    out = []
    consts, structs, fns = [], [], []
    seen_primitives = set()

    i = 0
    n = len(lines)
    pending_comment = []

    def flush_comment(target, indent=""):
        for c in pending_comment:
            target.append(indent + c if c else "")
        del pending_comment[:]

    while i < n:
        raw = lines[i]
        line = raw.strip()
        lineno = i + 1
        i += 1

        if re.match(r"^#\s*(ifndef|ifdef|else|endif)\b", line) or \
           re.match(r"^#\s+define\b", line):
            # Include guard and the C/GLSL linkage macros. WGSL needs none —
            # and neither does it need the comments explaining them, which would
            # otherwise drift down and attach to whatever came next.
            del pending_comment[:]
            continue
        if not line:
            if pending_comment:
                pending_comment.append("")
            continue
        if line.startswith("//"):
            pending_comment.append(line)
            continue

        m = DEFINE.match(line)
        if m:
            flush_comment(consts)
            consts.append("const %s : i32 = %s;" % (m.group(1), m.group(2)))
            continue
        if line.startswith("#define"):
            # RL_BUF / RL_FN — C and GLSL plumbing with no WGSL counterpart.
            del pending_comment[:]
            continue

        m = STRUCT_OPEN.match(line)
        if m:
            flush_comment(structs)
            structs.append("struct %s {" % m.group(1))
            while i < n:
                fl, fc = split_comment(lines[i])
                i += 1
                if fl.strip() == "};":
                    structs.append("}")
                    structs.append("")
                    break
                fm = FIELD.match(fl)
                if not fm:
                    raise Refusal(i, fl, "struct field")
                structs.append(
                    "  %-9s : %s,%s"
                    % (fm.group(2), TYPES[fm.group(1)], ("  " + fc) if fc else "")
                )
            continue

        m = FN_OPEN.match(line)
        if m:
            _st, ret, name, rest = m.groups()
            if name in PRIMITIVES:
                seen_primitives.add(name)
                flush_comment(fns)
                fns.append(PRIMITIVES[name])
                continue
            # Gather the whole signature, which may wrap.
            sig = rest
            while "{" not in sig:
                if i >= n:
                    raise Refusal(lineno, line, "unterminated signature")
                sig += " " + lines[i].strip()
                i += 1
            head, _, tail = sig.partition("{")
            head = head.rstrip()
            if not head.endswith(")"):
                raise Refusal(lineno, line, "signature does not close")
            params, ptrs = translate_params(head[:-1], lineno, line)
            wret = translate_type(ret, lineno, line)
            flush_comment(fns)
            arrow = "" if wret == "void" else " -> %s" % wret
            fns.append("fn %s(%s)%s {" % (name, ", ".join(params), arrow))

            # One-liner: `... { return expr; }`
            if tail.strip().endswith("}"):
                inner = tail.strip()[:-1].strip()
                rm = RETURN.match(inner)
                if not rm:
                    raise Refusal(lineno, line, "one-line body that is not a return")
                fns.append("  return %s;" % translate_expr(rm.group(1), ptrs))
                fns.append("}")
                fns.append("")
                continue

            body = []
            body_start = i + 1   # file line of body[0], for refusal messages
            depth = 1
            while i < n:
                bl = lines[i]
                i += 1
                depth += bl.count("{") - bl.count("}")
                if depth == 0:
                    break
                body.append(bl)
            fns.extend(translate_body(body, ptrs, body_start))
            fns.append("}")
            fns.append("")
            continue

        raise Refusal(lineno, line, "top-level construct")

    missing = set(PRIMITIVES) - seen_primitives
    if missing:
        raise Refusal(0, ", ".join(sorted(missing)),
                      "primitive missing from the header — it was renamed or removed")

    out.extend(consts)
    out.append("")
    out.extend(structs)
    out.append("")
    out.extend(fns)
    return out


def translate_body(body, ptrs, base_line):
    """A function body, statement by statement. Depth tracking is by brace count
    because the subset has no braces inside expressions."""
    res = []
    for idx, raw in enumerate(body):
        try:
            translate_statement(res, body, idx, raw, ptrs)
        except Refusal as e:
            raise e.at(base_line + idx, raw)
    return res


def translate_statement(res, body, idx, raw, ptrs):
    code, comment = split_comment(raw)
    stripped = code.strip()
    indent = "  " * (1 + (len(raw) - len(raw.lstrip())) // 4 - 1)
    lineno = 0   # filled in by the caller, which knows the file offset

    def emit(text):
        res.append((indent + text + ("  " + comment if comment else "")).rstrip())

    if not stripped:
        res.append(comment.rjust(len(comment) + len(indent)) if comment else "")
        return

    m = FOR.match(stripped)
    if m:
        var, init, cond, inc = m.groups()
        if inc != var:
            raise Refusal(lineno, raw, "loop increments something else")
        emit("for (var %s : i32 = %s; %s; %s = %s + 1) {"
             % (var, translate_expr(init, ptrs), translate_expr(cond, ptrs), var, var))
        return

    m = IF_HEAD.match(stripped)
    if m:
        prefix = "} else " if m.group(1) else ""
        cond, rest = split_condition(stripped, lineno, raw)
        cond = translate_expr(cond, ptrs)
        if rest == "{":
            emit("%sif (%s) {" % (prefix, cond))
            return
        # Braceless single-statement if. WGSL requires the braces.
        if rest == "continue;":
            inner = "continue;"
        else:
            am, rm = ASSIGN.match(rest), RETURN.match(rest)
            if am:
                inner = "%s = %s;" % (am.group(1), translate_expr(am.group(2), ptrs))
            elif rm:
                inner = "return %s;" % translate_expr(rm.group(1), ptrs)
            else:
                raise Refusal(lineno, raw, "braceless if body")
        emit("%sif (%s) { %s }" % (prefix, cond, inner))
        return

    if ELSE_OPEN.match(stripped):
        emit("} else {")
        return
    if stripped == "}":
        emit("}")
        return
    if stripped == "continue;":
        emit("continue;")
        return

    m = RETURN.match(stripped)
    if m:
        emit("return %s;" % translate_expr(m.group(1), ptrs) if m.group(1)
             else "return;")
        return

    # Several field assignments can share a line in the C (`c.r = ..; c.g = ..;`).
    if stripped.count(";") > 1 and DECL.match(stripped) is None:
        parts = [p.strip() + ";" for p in stripped.split(";") if p.strip()]
        emit(" ".join(
            "%s = %s;" % (ASSIGN.match(p).group(1),
                          translate_expr(ASSIGN.match(p).group(2), ptrs))
            if ASSIGN.match(p) else _refuse(lineno, raw)
            for p in parts))
        return

    m = DECL.match(stripped)
    if m:
        _st, ctype, name, init = m.groups()
        wtype = translate_type(ctype, lineno, raw)
        if init is None:
            emit("var %s : %s;" % (name, wtype))
        elif reassigned(body[idx + 1:], name):
            emit("var %s = %s;" % (name, translate_expr(init, ptrs)))
        else:
            emit("let %s = %s;" % (name, translate_expr(init, ptrs)))
        return

    m = COMPOUND.match(stripped)
    if m:
        emit("%s %s= %s;" % (m.group(1), m.group(2),
                             translate_expr(m.group(3), ptrs)))
        return

    m = ASSIGN.match(stripped)
    if m:
        emit("%s = %s;" % (m.group(1), translate_expr(m.group(2), ptrs)))
        return

    raise Refusal(lineno, raw, "statement")


def _refuse(lineno, raw):
    raise Refusal(lineno, raw, "multi-statement line")


def build():
    with open(SRC, "rb") as f:
        src_bytes = f.read()
    src_text = src_bytes.decode("utf-8")
    body = generate(src_text)
    header = [
        "// GENERATED by tools/roadlab-wgsl.py from proto/roadlab/rl_paint.h.",
        "// Do not edit: regenerate. The digest below is checked by roadlab's test",
        "// suite, so a header edit that nobody regenerated fails the build rather",
        "// than shipping a shader that shades the road differently from the CPU.",
        "//",
        "// rl_paint.h digest: %s" % digest(src_bytes),
        "//",
        "// Sampler contract (paint_texture.h): the caller fills a",
        "// var<function> array<RlBoundary, RL_MAX_BOUNDS> from the profile and style",
        "// textures, then passes &that. Core WGSL takes a function-address-space",
        "// pointer parameter, which is why the array is fixed-size and local.",
        "",
    ]
    text = "\n".join(header + body).rstrip() + "\n"
    return re.sub(r"\n{3,}", "\n\n", text)


SAMPLER = os.path.join(ROOT, "proto", "roadlab", "rl_paint_sampler.wgsl")


def validate():
    """Run the generated WGSL through a real WGSL front-end.

    naga is the one wgpu and Firefox use, so "it validates" means something. It
    is not vendored — `cargo install naga-cli` — so this degrades to a stated
    skip rather than a silent pass when it is absent.

    Two subjects, because they check different things. rl_paint.wgsl alone
    type-checks the evaluator. Concatenated with rl_paint_sampler.wgsl it also
    checks the contract in paint_texture.h: the bindings, the texel arithmetic,
    and passing a pointer-to-array into the evaluator from a fragment entry
    point. And the SPIR-V backend is asked for output rather than just a parse,
    because lowering enforces things the front-end lets through."""
    import subprocess
    import tempfile

    naga = None
    for candidate in ("naga", os.path.expanduser("~/.cargo/bin/naga")):
        try:
            v = subprocess.run([candidate, "--version"], capture_output=True, text=True)
            if v.returncode == 0:
                naga = candidate
                version = v.stdout.strip()
                break
        except OSError:
            continue
    if naga is None:
        print("naga not found — skipping WGSL validation "
              "(`cargo install naga-cli` to enable it)")
        return 0

    with open(DST) as f:
        evaluator = f.read()
    with open(SAMPLER) as f:
        sampler = f.read()

    failed = False
    with tempfile.TemporaryDirectory() as tmp:
        subjects = [
            ("rl_paint.wgsl", evaluator),
            ("rl_paint.wgsl + rl_paint_sampler.wgsl", evaluator + sampler),
        ]
        for name, text in subjects:
            path = os.path.join(tmp, "subject.wgsl")
            with open(path, "w") as f:
                f.write(text)
            steps = [("validate", [naga, path])]
            if "@fragment" in text:
                # Only a module with an entry point can be lowered.
                steps.append(("spirv", [naga, path, os.path.join(tmp, "out.spv")]))
                steps.append(("msl", [naga, path, os.path.join(tmp, "out.metal")]))
            for what, cmd in steps:
                r = subprocess.run(cmd, capture_output=True, text=True)
                if r.returncode != 0:
                    failed = True
                    sys.stderr.write("%s: %s FAILED\n%s%s\n"
                                     % (name, what, r.stdout, r.stderr))
                else:
                    print("%s: %s ok" % (name, what))
    print("validated with naga %s" % version)
    return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if the checked-in WGSL is stale")
    ap.add_argument("--validate", action="store_true",
                    help="run the WGSL through naga, if it is installed")
    args = ap.parse_args()
    try:
        text = build()
    except Refusal as e:
        sys.stderr.write(str(e) + "\n")
        return 2
    if args.check:
        existing = open(DST).read() if os.path.exists(DST) else ""
        if existing != text:
            sys.stderr.write(
                "proto/roadlab/rl_paint.wgsl is stale — run tools/roadlab-wgsl.py\n")
            return 1
        print("rl_paint.wgsl is up to date")
    else:
        with open(DST, "w") as f:
            f.write(text)
        print("wrote %s (%d lines)" % (os.path.relpath(DST, ROOT), text.count("\n")))
    if args.validate:
        return validate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
