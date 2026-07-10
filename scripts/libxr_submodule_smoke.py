#!/usr/bin/env python3
"""Smoke-test LibXR submodule update policy with local Git repositories."""

from __future__ import annotations

import contextlib
import io
import os
import subprocess
import sys
import tempfile

from pathlib import Path
from typing import Optional, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

from libxr.ConfigCubemxProject import add_libxr  # noqa: E402


def git(*args: str, cwd: Optional[Path] = None) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=cwd,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def commit_file(repo: Path, name: str, text: str) -> str:
    (repo / name).write_text(text, encoding="utf-8")
    git("add", name, cwd=repo)
    git("commit", "-m", text, cwd=repo)
    return git("rev-parse", "HEAD", cwd=repo)


def create_libxr_remote(root: Path) -> Tuple[Path, str, str, str, str]:
    source = root / "libxr-source"
    source.mkdir()
    git("init", "-b", "master", cwd=source)
    git("config", "user.name", "Smoke Test", cwd=source)
    git("config", "user.email", "smoke@example.com", cwd=source)
    old_commit = commit_file(source, "version.txt", "old")
    default_commit = commit_file(source, "version.txt", "default")
    newer_commit = commit_file(source, "version.txt", "newer")
    git("checkout", "-b", "divergent", old_commit, cwd=source)
    divergent_commit = commit_file(source, "divergent.txt", "divergent")
    git("checkout", "master", cwd=source)

    remote = root / "libxr.git"
    git("clone", "--bare", str(source), str(remote), cwd=root)
    return remote, old_commit, default_commit, newer_commit, divergent_commit


def create_project(
    root: Path,
    name: str,
    remote: Path,
    recorded_commit: str,
    checkout_commit: str,
) -> Tuple[Path, Path]:
    project = root / name
    project.mkdir()
    git("init", "-b", "master", cwd=project)
    git("config", "user.name", "Smoke Test", cwd=project)
    git("config", "user.email", "smoke@example.com", cwd=project)
    git("-c", "protocol.file.allow=always", "submodule", "add", str(remote),
        "Middlewares/Third_Party/LibXR", cwd=project)
    checkout = project / "Middlewares" / "Third_Party" / "LibXR"
    git("checkout", recorded_commit, cwd=checkout)
    git("add", ".gitmodules", "Middlewares/Third_Party/LibXR", cwd=project)
    git("commit", "-m", "add LibXR", cwd=project)
    git("checkout", checkout_commit, cwd=checkout)
    return project, checkout


@contextlib.contextmanager
def allow_file_protocol():
    old_value = os.environ.get("GIT_ALLOW_PROTOCOL")
    os.environ["GIT_ALLOW_PROTOCOL"] = "file"
    try:
        yield
    finally:
        if old_value is None:
            os.environ.pop("GIT_ALLOW_PROTOCOL", None)
        else:
            os.environ["GIT_ALLOW_PROTOCOL"] = old_value


def assert_head(checkout: Path, expected: str, scenario: str) -> None:
    actual = git("rev-parse", "HEAD", cwd=checkout)
    if actual != expected:
        raise AssertionError(f"{scenario}: expected {expected}, got {actual}")


def run_policy_checks(root: Path) -> None:
    remote, old_commit, default_commit, newer_commit, divergent_commit = (
        create_libxr_remote(root)
    )

    project, checkout = create_project(
        root, "stale-checkout", remote,
        recorded_commit=default_commit, checkout_commit=old_commit
    )
    add_libxr(project, default_libxr_commit=default_commit)
    assert_head(checkout, default_commit, "stale checkout with current gitlink")

    project, checkout = create_project(
        root, "old-gitlink", remote,
        recorded_commit=old_commit, checkout_commit=old_commit
    )
    add_libxr(project, default_libxr_commit=default_commit)
    assert_head(checkout, default_commit, "old checkout with old gitlink")

    project, checkout = create_project(
        root, "current", remote,
        recorded_commit=default_commit, checkout_commit=default_commit
    )
    add_libxr(project, default_libxr_commit=default_commit)
    assert_head(checkout, default_commit, "current checkout")

    project, checkout = create_project(
        root, "newer", remote,
        recorded_commit=default_commit, checkout_commit=newer_commit
    )
    add_libxr(project, default_libxr_commit=default_commit)
    assert_head(checkout, newer_commit, "newer checkout")

    project, checkout = create_project(
        root, "divergent", remote,
        recorded_commit=default_commit, checkout_commit=divergent_commit
    )
    add_libxr(project, default_libxr_commit=default_commit)
    assert_head(checkout, divergent_commit, "divergent checkout")

    project, checkout = create_project(
        root, "dirty", remote,
        recorded_commit=old_commit, checkout_commit=old_commit
    )
    (checkout / "version.txt").write_text("local changes", encoding="utf-8")
    add_libxr(project, default_libxr_commit=default_commit)
    assert_head(checkout, old_commit, "dirty old checkout")
    if git("status", "--porcelain", cwd=checkout) == "":
        raise AssertionError("dirty old checkout: local changes were lost")

    project, checkout = create_project(
        root, "explicit", remote,
        recorded_commit=old_commit, checkout_commit=newer_commit
    )
    add_libxr(
        project, libxr_commit=old_commit, default_libxr_commit=default_commit
    )
    assert_head(checkout, old_commit, "explicit commit")


def main() -> int:
    with allow_file_protocol(), tempfile.TemporaryDirectory(
        prefix="libxr_submodule_smoke_"
    ) as tmp:
        with contextlib.redirect_stdout(io.StringIO()):
            run_policy_checks(Path(tmp))
    print("LibXR submodule policy smoke checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
