#!/usr/bin/env python3
"""Check what a change needs: formatting, then tests and clippy for the crates
it touches and every workspace crate that depends on them.

Changed files differ from the merge base with --base (default origin/main),
committed or not, or are untracked. A file in a crate's directory selects that
crate. Root Markdown, LICENSE, .github/, licenses/ and scripts/ select nothing;
any other root file (Cargo.toml, Cargo.lock, rust-toolchain.toml, ...) selects
the whole workspace. Dependents follow normal, build and dev dependencies,
optional ones included.

Features are unified across the selected crates only, so run
`cargo test --workspace --locked` before a release as well.
"""
import argparse
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
QUIET_DIRS = {".github", "licenses", "scripts"}
QUIET_FILES = {"LICENSE"}


def output(*args):
    return subprocess.run(args, cwd=ROOT, check=True, text=True, encoding="utf-8",
                          capture_output=True).stdout


def changed_files(base):
    merge_base = output("git", "merge-base", base, "HEAD").strip()
    tracked = output("git", "diff", "--name-only", merge_base).splitlines()
    untracked = output("git", "ls-files", "--others", "--exclude-standard").splitlines()
    return sorted(set(tracked) | set(untracked))


def workspace():
    """Each member's directory (relative, POSIX) and its workspace dependents."""
    metadata = json.loads(output("cargo", "metadata", "--format-version", "1", "--no-deps"))
    dirs, dependents = {}, {}
    for package in metadata["packages"]:
        manifest = Path(package["manifest_path"]).resolve()
        dirs[package["name"]] = manifest.parent.relative_to(ROOT).as_posix()
        dependents.setdefault(package["name"], set())
    for package in metadata["packages"]:
        for dependency in package["dependencies"]:
            if dependency["name"] in dirs and dependency["name"] != package["name"]:
                dependents[dependency["name"]].add(package["name"])
    return dirs, dependents


def select(files, dirs, dependents):
    """The crates to check, the ones changed directly, and the root files that
    select the whole workspace."""
    changed, everything = set(), []
    for file in files:
        path = PurePosixPath(file)
        crate = next((name for name, d in dirs.items() if path.is_relative_to(d)), None)
        if crate:
            changed.add(crate)
        elif path.parts[0] in QUIET_DIRS:
            continue
        elif len(path.parts) == 1 and (path.suffix == ".md" or file in QUIET_FILES):
            continue
        else:
            everything.append(file)
    if everything:
        return set(dirs), changed, everything
    selected, pending = set(changed), list(changed)
    while pending:
        for dependent in dependents[pending.pop()]:
            if dependent not in selected:
                selected.add(dependent)
                pending.append(dependent)
    return selected, changed, everything


def run(args):
    print("+", " ".join(args), flush=True)
    return subprocess.run(args, cwd=ROOT).returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", default="origin/main",
                        help="revision to compare with (default: origin/main)")
    parser.add_argument("--list", action="store_true",
                        help="print the selection and run nothing")
    parser.add_argument("cargo_args", nargs="*",
                        help="after --, extra arguments for cargo test and clippy, "
                             "e.g. --features roplat_behavior/roplat")
    args = parser.parse_args()

    dirs, dependents = workspace()
    selected, changed, everything = select(changed_files(args.base), dirs, dependents)
    if everything:
        print("whole workspace, for:", ", ".join(everything))
    else:
        print("changed:", ", ".join(sorted(changed)) or "none")
        print("dependents:", ", ".join(sorted(selected - changed)) or "none")
    if args.list:
        return 0

    failed = []
    if run(["cargo", "fmt", "--all", "--", "--check"]):
        failed.append("fmt")
    if selected:
        packages = (["--workspace"] if selected == set(dirs)
                    else [arg for name in sorted(selected) for arg in ("-p", name)])
        if run(["cargo", "test", "--locked", *packages, *args.cargo_args]):
            failed.append("test")
        if run(["cargo", "clippy", "--locked", "--all-targets", *packages,
                *args.cargo_args, "--", "-D", "warnings"]):
            failed.append("clippy")
    print("failed:", ", ".join(failed) if failed else "nothing")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
