#!/usr/bin/env python3
"""Make a macOS .app self-contained: copy in every non-system library it loads, recursively.

macdeployqt bundles Qt, but not the rest — Python's own extension modules link Homebrew's
OpenSSL, mpdecimal, xz and so on, and those link further libraries in turn. This walks
every Mach-O file in the bundle, copies each dependency that lives outside the system into
Contents/Frameworks, rewrites the reference to point there, and repeats until no file in
the bundle refers to anything outside it but the system. It then signs everything ad hoc,
inside out, since editing a load command invalidates a signature and Apple Silicon will not
run unsigned code.

    macos_bundle_deps.py <path/to/App.app>

Exits non-zero, listing them, if any outside reference survives — a DMG that only works on
the Mac that built it is the failure this exists to prevent.
"""
import os
import shutil
import subprocess
import sys

SYSTEM_PREFIXES = ("/System/", "/usr/lib/")


def run(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def is_macho(path):
    if os.path.islink(path) or not os.path.isfile(path):
        return False
    with open(path, "rb") as f:
        magic = f.read(4)
    return magic in (b"\xcf\xfa\xed\xfe", b"\xca\xfe\xba\xbe", b"\xfe\xed\xfa\xcf", b"\xbe\xba\xfe\xca")


def deps(path):
    lines = run("otool", "-L", path).splitlines()[1:]
    out = [l.strip().split(" (")[0] for l in lines if l.strip()]
    own = run("otool", "-D", path).splitlines()[1:]
    own = own[0].strip() if own else None
    return [d for d in out if d != own], own


def rpaths(path):
    out, lines, i = [], run("otool", "-l", path).splitlines(), 0
    while i < len(lines):
        if "LC_RPATH" in lines[i]:
            for j in range(i, min(i + 4, len(lines))):
                if lines[j].strip().startswith("path "):
                    out.append(lines[j].strip().split()[1])
        i += 1
    return out


def resolve(dep, referrer, search):
    """Where an @rpath / @loader_path / absolute reference actually points on this Mac."""
    if dep.startswith("@loader_path/"):
        return os.path.realpath(os.path.join(os.path.dirname(referrer), dep[len("@loader_path/"):]))
    if dep.startswith("@executable_path/"):
        return None  # already bundle-relative
    if dep.startswith("@rpath/"):
        name = dep[len("@rpath/"):]
        for base in rpaths(referrer) + search:
            base = base.replace("@loader_path", os.path.dirname(referrer))
            candidate = os.path.join(base, name)
            if os.path.exists(candidate):
                return os.path.realpath(candidate)
        return None
    return os.path.realpath(dep) if os.path.exists(dep) else None


def main(app):
    app = os.path.abspath(app)
    frameworks = os.path.join(app, "Contents", "Frameworks")
    os.makedirs(frameworks, exist_ok=True)
    search = [frameworks] + [p for p in os.environ.get("BUNDLE_LIBPATH", "").split(":") if p]
    copied = {}  # realpath outside -> name inside Frameworks

    changed = True
    while changed:
        changed = False
        files = [os.path.join(r, f) for r, _, fs in os.walk(app) for f in fs]
        for path in filter(is_macho, files):
            refs, _ = deps(path)
            for dep in refs:
                if dep.startswith(SYSTEM_PREFIXES) or dep.startswith("@executable_path/"):
                    continue
                target = resolve(dep, path, search)
                if target is None or target.startswith(app + "/"):
                    # already inside the bundle: point at it bundle-relatively if it is not
                    if target and dep.startswith("/"):
                        rel = os.path.relpath(target, os.path.join(app, "Contents", "MacOS"))
                        run("install_name_tool", "-change", dep, "@executable_path/" + rel, path)
                        changed = True
                    continue
                if target.startswith(SYSTEM_PREFIXES):
                    continue
                name = copied.get(target)
                if name is None:
                    name = os.path.basename(target)
                    dest = os.path.join(frameworks, name)
                    if not os.path.exists(dest):
                        shutil.copy2(target, dest)
                        os.chmod(dest, 0o755)
                        run("install_name_tool", "-id", "@executable_path/../Frameworks/" + name, dest)
                    copied[target] = name
                    changed = True
                run("install_name_tool", "-change", dep, "@executable_path/../Frameworks/" + name, path)
                changed = True

    # anything still pointing outside the bundle is a library this Mac has and others do not
    bad = []
    files = [os.path.join(r, f) for r, _, fs in os.walk(app) for f in fs]
    for path in filter(is_macho, files):
        for dep in deps(path)[0]:
            if dep.startswith(SYSTEM_PREFIXES) or dep.startswith("@executable_path/"):
                continue
            target = resolve(dep, path, [frameworks])
            if target is None or not target.startswith(app + "/"):
                bad.append(f"{os.path.relpath(path, app)} -> {dep}")
    if bad:
        print("references outside the bundle remain:", *bad, sep="\n  ")
        return 1

    # sign inside out: every loose Mach-O, then frameworks, then the app
    machos = sorted((p for p in files if is_macho(p)), key=lambda p: -p.count("/"))
    for path in machos:
        run("codesign", "--force", "--sign", "-", "--timestamp=none", path)
    for root, dirs, _ in os.walk(frameworks):
        for d in dirs:
            if d.endswith(".framework"):
                run("codesign", "--force", "--sign", "-", "--timestamp=none", os.path.join(root, d))
    run("codesign", "--force", "--sign", "-", "--timestamp=none", app)
    print(f"bundled {len(copied)} librar{'y' if len(copied) == 1 else 'ies'}; nothing outside the bundle is referenced")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
