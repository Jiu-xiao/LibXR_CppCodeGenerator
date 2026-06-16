#!/usr/bin/env python

import logging
import os
import subprocess
import shlex
import shutil
import sys

import argparse

from libxr.CubeMXGenerator import generate_cubemx_project

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

DEFAULT_MIRRORS = [
    "https://gitee.com/jiu-xiao/libxr",
]


def is_git_repo(path):
    try:
        result = subprocess.run(
            ["git", "-C", path, "rev-parse", "--is-inside-work-tree"],
            capture_output=True,
            text=True,
            check=True
        )
        return result.stdout.strip() == "true"
    except subprocess.CalledProcessError:
        return False


def is_git_worktree_root(path):
    try:
        result = subprocess.run(
            ["git", "-C", path, "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            check=True
        )
        return os.path.realpath(result.stdout.strip()) == os.path.realpath(path)
    except subprocess.CalledProcessError:
        return False


def get_git_dir(path):
    result = subprocess.run(
        ["git", "-C", path, "rev-parse", "--git-dir"],
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        logging.warning(f"Failed to resolve git dir for {path}: {result.stderr.strip()}")
        return ""
    git_dir = result.stdout.strip()
    if not os.path.isabs(git_dir):
        git_dir = os.path.normpath(os.path.join(path, git_dir))
    return git_dir


def is_git_clean(path):
    """Check if the Git repo at `path` has no uncommitted changes."""
    result = subprocess.run(
        ["git", "-C", path, "status", "--porcelain"],
        capture_output=True,
        text=True
    )
    return result.returncode == 0 and result.stdout.strip() == ""


def _fmt_cmd(cmd):
    if isinstance(cmd, (list, tuple)):
        return " ".join(shlex.quote(str(x)) for x in cmd)
    return str(cmd)


def run_command(cmd, ignore_error=False):
    """Run a command. Accepts either a list/tuple (preferred, shell=False) or a string (shell=True)."""
    if isinstance(cmd, (list, tuple)):
        result = subprocess.run(cmd, capture_output=True, text=True)
    else:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        logging.info(f"[OK] {_fmt_cmd(cmd)}")
        return result.stdout
    if ignore_error:
        logging.warning(f"[IGNORED FAILURE] {_fmt_cmd(cmd)}\n{result.stderr}")
        return result.stdout
    logging.error(f"[FAILED] {_fmt_cmd(cmd)}\n{result.stderr}")
    sys.exit(1)


def find_ioc_file(directory):
    """Search for a .ioc file in the specified directory."""
    for file in os.listdir(directory):
        if file.endswith(".ioc"):
            return os.path.join(directory, file)
    return None


def pick_git_base(default_base="https://github.com", mirrors=None, timeout=5.0):
    """
    Select the fastest accessible Git source among the default and mirrors.
    Returns either a base URL or a full repository URL.
    - default_base: e.g. https://github.com
    - mirrors: a list of base URLs or full repo URLs
    """
    import time

    def is_repo_url(s: str) -> bool:
        return s.endswith(".git") or s.rstrip("/").split("/")[-1].lower() == "libxr"

    def to_probe_url(base_or_repo: str) -> str:
        if is_repo_url(base_or_repo):
            return base_or_repo
        return f"{base_or_repo.rstrip('/')}/Jiu-Xiao/libxr.git"

    candidates = [default_base] + [m.strip() for m in (mirrors or []) if m.strip()]
    scores = []
    for item in candidates:
        url = to_probe_url(item)
        start = time.time()
        try:
            r = subprocess.run(
                ["git", "ls-remote", "-h", url],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=timeout
            )
            if r.returncode == 0:
                scores.append((time.time() - start, item))
        except subprocess.TimeoutExpired:
            pass
    return min(scores)[1] if scores else default_base


def make_repo_url(base_or_repo: str, owner="Jiu-Xiao", repo="libxr"):
    # If a full repository URL is provided (.git or ends with repo name), return it as-is
    if base_or_repo.endswith(".git") or base_or_repo.rstrip("/").split("/")[-1].lower() == repo.lower():
        return base_or_repo
    return f"{base_or_repo.rstrip('/')}/{owner}/{repo}.git"


def create_gitignore_file(project_dir):
    gitignore_path = os.path.join(project_dir, ".gitignore")
    if not os.path.exists(gitignore_path):
        logging.info("Creating .gitignore file...")
        with open(gitignore_path, "w", encoding="utf-8", newline="\n") as gitignore_file:
            gitignore_file.write("""build/**
.history/**
.cache/**
.config.yaml
CMakeFiles/**
""")


def get_git_head(path):
    result = subprocess.run(
        ["git", "-C", path, "rev-parse", "HEAD"],
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


def get_submodule_gitlink(project_dir, rel_path):
    result = subprocess.run(
        ["git", "-C", project_dir, "ls-tree", "HEAD", "--", rel_path],
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        return ""
    parts = result.stdout.strip().split()
    if len(parts) >= 3 and parts[0] == "160000" and parts[1] == "commit":
        return parts[2]
    return ""


def is_commit_ancestor(repo_path, older_commit, newer_commit):
    if not older_commit or not newer_commit:
        return False
    result = subprocess.run(
        [
            "git", "-C", repo_path, "merge-base", "--is-ancestor",
            older_commit, newer_commit
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    return result.returncode == 0


def remove_path(path):
    if os.path.islink(path) or os.path.isfile(path):
        os.remove(path)
    elif os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)


def repair_submodule_checkout(project_dir, rel_path, checkout_path):
    logging.warning("LibXR submodule checkout is invalid; recreating it from registered metadata.")
    run_command(
        ["git", "-C", project_dir, "submodule", "deinit", "-f", "--", rel_path],
        ignore_error=True
    )

    git_dir = get_git_dir(project_dir)
    if git_dir:
        module_git_dir = os.path.join(git_dir, "modules", *rel_path.split("/"))
        if os.path.exists(module_git_dir):
            logging.info(f"Removing stale LibXR submodule gitdir: {module_git_dir}")
            remove_path(module_git_dir)

    if os.path.exists(checkout_path) or os.path.islink(checkout_path):
        logging.info(f"Removing stale LibXR submodule worktree: {checkout_path}")
        remove_path(checkout_path)


def add_libxr(project_dir, libxr_commit=None, git_base="https://github.com",
              default_libxr_commit=None):
    sub_rel_path_posix = "Middlewares/Third_Party/LibXR"
    libxr_path = os.path.join(project_dir, "Middlewares", "Third_Party", "LibXR")

    midware_path = os.path.join(project_dir, "Middlewares")
    third_party_path = os.path.join(midware_path, "Third_Party")

    def has_registered_submodule(repo_root, rel_path):
        if os.path.exists(os.path.join(repo_root, ".gitmodules")):
            result = subprocess.run(
                [
                    "git", "-C", repo_root, "config", "-f", ".gitmodules",
                    "--get-regexp", r"^submodule\..*\.path$"
                ],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                for line in result.stdout.splitlines():
                    parts = line.split(None, 1)
                    if len(parts) == 2 and parts[1].strip() == rel_path:
                        return True

        result = subprocess.run(
            ["git", "-C", repo_root, "ls-files", "--stage", "--", rel_path],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            return False

        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[0] == "160000":
                return True
        return False

    if not os.path.exists(midware_path):
        logging.info("Creating Middleware folder...")
        os.makedirs(midware_path)
    if not os.path.exists(third_party_path):
        logging.info("Creating Third Party folder...")
        os.makedirs(third_party_path)

    if not is_git_repo(project_dir):
        logging.warning(f"{project_dir} is not a Git repository. Initializing...")
        run_command(["git", "init", project_dir])

    registered = has_registered_submodule(project_dir, sub_rel_path_posix)
    checkout_path_present = os.path.lexists(libxr_path)
    existing_checkout = checkout_path_present and is_git_worktree_root(libxr_path)
    added_submodule = False

    if registered:
        if existing_checkout:
            run_command(
                ["git", "-C", project_dir, "submodule", "sync", "--", sub_rel_path_posix],
                ignore_error=False
            )
            logging.info("LibXR submodule already exists; preserving current checkout.")
        else:
            if checkout_path_present:
                repair_submodule_checkout(project_dir, sub_rel_path_posix, libxr_path)
            run_command(
                ["git", "-C", project_dir, "submodule", "sync", "--", sub_rel_path_posix],
                ignore_error=False
            )
            run_command(
                [
                    "git", "-C", project_dir, "submodule", "update",
                    "--init", "--recursive", "--", sub_rel_path_posix
                ],
                ignore_error=False
            )
    else:
        logging.info("LibXR submodule not registered yet; skipping preemptive update.")

    repo_url = make_repo_url(git_base, "Jiu-Xiao", "libxr")
    if not registered:
        logging.info(f"Adding LibXR as submodule from {repo_url} ...")
        run_command(
            ["git", "-C", project_dir, "submodule", "add", repo_url, sub_rel_path_posix]
        )
        logging.info("LibXR submodule added and initialized.")
        added_submodule = True
    else:
        logging.info("LibXR submodule already registered.")

    if os.path.exists(libxr_path):
        logging.info("LibXR submodule path exists.")
        current_commit = get_git_head(libxr_path)
        project_commit = get_submodule_gitlink(project_dir, sub_rel_path_posix)
        dirty = not is_git_clean(libxr_path)
        fetched = False
        target_commit = ""

        if libxr_commit:
            target_commit = libxr_commit
            logging.info(f"Checking out LibXR to requested commit {target_commit}")
        elif added_submodule and default_libxr_commit:
            target_commit = default_libxr_commit
            logging.info(f"Initializing new LibXR submodule to default commit {target_commit}")
        elif dirty:
            logging.warning("LibXR submodule has local changes; keeping current checkout.")
        elif (project_commit and default_libxr_commit and project_commit != default_libxr_commit
              and current_commit in (project_commit, default_libxr_commit)):
            run_command(["git", "-C", libxr_path, "fetch", "origin"], ignore_error=True)
            fetched = True

            if current_commit == project_commit and is_commit_ancestor(
                libxr_path, project_commit, default_libxr_commit
            ):
                target_commit = default_libxr_commit
                logging.info(f"Updating LibXR from older project commit to default {target_commit}")
            elif current_commit == default_libxr_commit and not is_commit_ancestor(
                libxr_path, project_commit, default_libxr_commit
            ):
                target_commit = project_commit
                logging.info(f"Restoring LibXR to project submodule commit {target_commit}")

        if target_commit:
            if not fetched:
                run_command(["git", "-C", libxr_path, "fetch", "origin"], ignore_error=True)
            run_command(["git", "-C", libxr_path, "checkout", target_commit])
        elif not dirty:
            logging.info("No LibXR commit requested; keeping existing submodule checkout.")


def create_user_directory(project_dir):
    """Ensure the User directory exists."""
    user_path = os.path.join(project_dir, "User")
    if not os.path.exists(user_path):
        os.makedirs(user_path)
    return user_path


def process_ioc_file(project_dir, yaml_output):
    """Parse the .ioc file and generate YAML configuration."""
    logging.info("Parsing .ioc file...")
    run_command(f"xr_parse_ioc -d {project_dir} -o {yaml_output}")


def generate_cpp_code(yaml_output, cpp_output, xrobot_enable=False):
    """Generate C++ code from YAML configuration, with optional XRobot support."""
    logging.info("Generating C++ code...")
    cmd = f"xr_gen_code_stm32 -i {yaml_output} -o {cpp_output}"
    if xrobot_enable:
        cmd += " --xrobot"
    run_command(cmd)


def generate_cmake_file(project_dir):
    """Generate CMakeLists.txt for STM32 project with selected compiler."""
    run_command(f"xr_stm32_cmake {project_dir}")


def _friendly_path_name(path: str) -> str:
    """
    Return a human-friendly name for a path.
    If path is '.', show the current folder name instead of '.'.
    Falls back to absolute path for root-like cases.
    """
    abs_path = os.path.abspath(path)
    base = os.path.basename(abs_path.rstrip(os.sep))
    return base or abs_path


def ensure_valid_cubemx_project(path: str):
    """
    Exit if `path` is not a typical STM32CubeMX project (must contain Core/).
    Display a friendly name instead of '.' when logging.
    """
    display_name = _friendly_path_name(path)
    core_dir = os.path.join(path, "Core")
    if not os.path.isdir(core_dir):
        logging.error(f"{display_name} is not a valid STM32CubeMX project: missing Core/ directory")
        sys.exit(1)


def main():
    from libxr.PackageInfo import LibXRPackageInfo

    LibXRPackageInfo.check_and_print()

    parser = argparse.ArgumentParser(description="Automate STM32CubeMX project setup")
    parser.add_argument("-d", "--directory", required=True, help="STM32CubeMX project directory")
    parser.add_argument("-t", "--terminal", default="", help="Optional terminal device source")
    parser.add_argument("--xrobot", action="store_true", help="Support XRobot")
    parser.add_argument("--commit", default="", help="Specify locked LibXR commit hash")
    parser.add_argument("--git-source", default="auto",
                        help="Git source base URL or full repo URL, or 'auto'/'github' (default: auto)")
    parser.add_argument("--git-mirrors", default="",
                        help="Comma-separated mirror base/repo URLs (will be tried when --git-source=auto)")
    parser.add_argument("--cubemx-generate", action="store_true",
                        help="Run STM32CubeMX script-mode generation before parsing the .ioc")
    parser.add_argument("--cubemx-cmd", default="",
                        help="STM32CubeMX executable path used with --cubemx-generate")
    parser.add_argument("--java-cmd", default="",
                        help="Java executable path used with --cubemx-generate")
    parser.add_argument("--cubemx-launch-mode", choices=["auto", "direct", "java"], default="auto",
                        help="CubeMX launch mode for --cubemx-generate (default: auto)")
    parser.add_argument("--cubemx-generate-code-dir", default="",
                        help="Use 'generate code <dir>' instead of 'project generate' when running CubeMX")
    parser.add_argument("--cubemx-expect-path", action="append", default=[],
                        help="Path that must exist after CubeMX generation; can be passed multiple times")
    parser.add_argument("--cubemx-log-dir", default="",
                        help="Directory for CubeMX command/script/stdout/stderr logs")
    parser.add_argument("--cubemx-script-path", default="",
                        help="Explicit path for the generated CubeMX script file")
    parser.add_argument("--cubemx-keep-script", action="store_true",
                        help="Keep the generated CubeMX script in the project directory")
    parser.add_argument("--cubemx-silent", action="store_true",
                        help="Pass -s to STM32CubeMX during --cubemx-generate")
    parser.add_argument("--cubemx-auto-confirm", action="store_true",
                        help="Attempt to auto-confirm migration/license/download dialogs during CubeMX generation")
    parser.add_argument("--cubemx-timeout", type=int, default=1200,
                        help="CubeMX generation timeout in seconds (default: 1200)")

    args = parser.parse_args()

    project_dir = args.directory.rstrip("/")
    terminal_source = args.terminal
    xrobot_enable = bool(args.xrobot)

    libxr_commit = args.commit.strip()
    default_libxr_commit = ""
    if not libxr_commit:
        try:
            sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))
            from libxr.libxr_version import LibXRInfo
            default_libxr_commit = LibXRInfo.COMMIT
        except Exception as e:
            logging.info(f"No lock commit found in src/libxr/libxr_version.py: {e}")
            default_libxr_commit = ""

    if libxr_commit:
        logging.info(f"Requested LibXR commit: {libxr_commit}")
    elif default_libxr_commit:
        logging.info(f"Default LibXR commit: {default_libxr_commit}")

    if not os.path.isdir(project_dir):
        logging.error(f"Directory {_friendly_path_name(project_dir)} does not exist")
        sys.exit(1)

    # Validate STM32CubeMX project structure (must have Core/ directory)
    ensure_valid_cubemx_project(project_dir)

    # Select Git source (auto benchmarks default and mirrors)
    env_mirrors = os.environ.get("XR_GIT_MIRRORS", "")
    cli_mirrors = [m for m in args.git_mirrors.split(",") if m.strip()]
    all_mirrors = DEFAULT_MIRRORS + \
                  [m.strip() for m in (env_mirrors.split(",") if env_mirrors else []) if m.strip()] + \
                  cli_mirrors

    if args.git_source == "auto":
        git_base = pick_git_base(default_base="https://github.com", mirrors=all_mirrors, timeout=5.0)
    elif args.git_source == "github":
        git_base = "https://github.com"
    else:
        git_base = args.git_source
    logging.info(f"Selected Git base/repo: {git_base}")

    # Add Git submodule if necessary
    add_libxr(
        project_dir,
        libxr_commit if libxr_commit else None,
        git_base=git_base,
        default_libxr_commit=default_libxr_commit if default_libxr_commit else None
    )

    # Find .ioc file
    ioc_file = find_ioc_file(project_dir)
    if not ioc_file:
        logging.error("No .ioc file found")
        sys.exit(1)

    logging.info(f"Found .ioc file: {ioc_file}")

    if args.cubemx_generate:
        logging.info("Running STM32CubeMX generation before codegen...")
        generate_cubemx_project(
            project_dir=project_dir,
            ioc_file=ioc_file,
            cubemx_cmd=args.cubemx_cmd,
            java_cmd=args.java_cmd,
            launch_mode=args.cubemx_launch_mode,
            generate_code_dir=args.cubemx_generate_code_dir,
            expect_paths=args.cubemx_expect_path,
            log_dir=args.cubemx_log_dir,
            script_path=args.cubemx_script_path,
            keep_script=args.cubemx_keep_script,
            silent=args.cubemx_silent,
            auto_confirm=args.cubemx_auto_confirm,
            timeout=args.cubemx_timeout,
        )

    create_gitignore_file(project_dir)

    # Create user directory
    user_path = create_user_directory(project_dir)

    # Define paths
    yaml_output = os.path.join(project_dir, ".config.yaml")
    cpp_output = os.path.join(user_path, "app_main.cpp")

    # Process .ioc file
    process_ioc_file(project_dir, yaml_output)

    # Generate C++ code
    generate_cpp_code(yaml_output, cpp_output, xrobot_enable)

    # Generate CMakeLists.txt with selected compiler
    generate_cmake_file(project_dir)

    # Handle optional terminal source
    if terminal_source:
        logging.info("Modifying terminal device source...")

    logging.info("[Pass] All tasks completed successfully!")


if __name__ == "__main__":
    main()
