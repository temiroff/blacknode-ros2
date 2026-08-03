"""Run ``ros2`` CLI commands through the best available backend.

Native ``ros2`` on PATH (Linux, WSL, or a sourced Windows install) is
preferred. Otherwise a long-lived Docker helper container (image
``ros:jazzy`` by default) is started on demand and commands run via
``docker exec`` inside it. Everything node-facing returns structured results
instead of raising, so graphs stay viewable and editable on machines with no
ROS at all.

Environment overrides:

- ``BLACKNODE_ROS2_IMAGE``      Docker image (default ``ros:jazzy``)
- ``BLACKNODE_ROS2_CONTAINER``  helper container name (default ``blacknode-ros2``)
"""
from __future__ import annotations

import os
import json
import hashlib
import re
import signal
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any

from blacknode import console

IMAGE = os.environ.get("BLACKNODE_ROS2_IMAGE", "ros:jazzy")
CONTAINER = os.environ.get("BLACKNODE_ROS2_CONTAINER", "blacknode-ros2")
STREAM_PORT_RANGE = os.environ.get("BLACKNODE_ROS2_STREAM_PORT_RANGE", "39000-39049")
_CONTAINER_STREAM_SCRIPT = "/tmp/blacknode_ros2_image_stream_server.py"
_CONTAINER_SNAPSHOT_SCRIPT = "/tmp/blacknode_ros2_image_snapshot.py"
_CONTAINER_TOPIC_RELAY_SCRIPT = "/tmp/blacknode_ros2_topic_relay.py"
_CONTAINER_TOPIC_SUBSCRIBER_SCRIPT = "/tmp/blacknode_ros2_topic_subscriber.py"
_CONTAINER_PYTHON_NODE_PREFIX = "/tmp/blacknode_ros2_python_node_"
_CONTAINER_WORKSPACE_PREFIX = "/tmp/blacknode_ros2_workspace_"

_NO_BACKEND_HELP = (
    "ROS 2 is not available: no `ros2` on PATH and Docker is not installed. "
    "Install ROS 2 natively, or install Docker Desktop — Blacknode starts it "
    f"automatically and pulls `{IMAGE}` on first use."
)
_DOCKER_UNREACHABLE_HELP = (
    "Docker CLI is installed but its daemon is not reachable. Start Docker "
    "Desktop (or dockerd) manually and retry."
)

_cached_backend: dict[str, str] | None = None
_backend_detection_lock = threading.Lock()
_detached: list[subprocess.Popen] = []
_managed_detached: dict[str, subprocess.Popen] = {}
_managed_docker_patterns: dict[str, str] = {}
_topic_subscribers: dict[str, dict[str, Any]] = {}
_python_nodes: dict[str, dict[str, Any]] = {}
_streams: dict[str, dict[str, Any]] = {}


def runtime_status() -> dict[str, Any]:
    """Return Blacknode-started ROS runtime helpers still known to this process."""
    live_streams: list[dict[str, Any]] = []
    for stream_id, item in list(_streams.items()):
        proc = item.get("proc")
        running = bool(proc is not None and proc.poll() is None)
        if not running:
            _streams.pop(stream_id, None)
            continue
        live_streams.append({
            "stream_id": stream_id,
            "url": item.get("url", ""),
            "snapshot_url": item.get("snapshot_url", ""),
            "topic": item.get("topic", ""),
            "message_type": item.get("message_type", ""),
        })

    live_runs: list[dict[str, Any]] = []
    for run_id, proc in list(_managed_detached.items()):
        if proc.poll() is None:
            live_runs.append({"run_id": run_id, "pid": proc.pid})
        else:
            _managed_detached.pop(run_id, None)
    known_native_runs = {str(item.get("run_id") or "") for item in live_runs}
    live_runs.extend(
        {"run_id": run_id, "backend": "docker"}
        for run_id in sorted(_managed_docker_patterns)
        if run_id not in known_native_runs
    )

    node_outputs: list[dict[str, Any]] = []
    for run_id, item in list(_topic_subscribers.items()):
        snapshot = _topic_subscriber_snapshot(item)
        running = bool(snapshot.get("running"))
        messages = list(snapshot.get("messages") or [])
        errors = list(item.get("errors") or [])
        latest = snapshot.get("latest") or {}
        report = (
            f"subscribing as /{item.get('node_name', '')} on {item.get('topic', '')}; "
            f"received {int(snapshot.get('received') or 0)} message(s)"
            if running
            else f"subscriber stopped after {int(snapshot.get('received') or 0)} message(s)"
        )
        if errors and not messages:
            report = f"subscriber error: {errors[-1]}"
        if item.get("public_node_type") == "ROS2":
            outputs = ros2_topic_outputs(snapshot, report=report)
        else:
            outputs = {
                "running": running,
                "latest": latest,
                "messages": messages,
                "received": int(snapshot.get("received") or 0),
                "backend": item.get("backend", ""),
                "report": report,
            }
        node_outputs.append({
            "node_type": item.get("public_node_type", "ROS2TopicSubscriber"),
            "run_id": run_id,
            "outputs": outputs,
        })
    node_outputs.extend(_python_node_runtime_outputs())

    live_detached = [proc for proc in _detached if proc.poll() is None]
    _detached[:] = live_detached
    try:
        from blacknode.pkg.blacknode_skills.follow_person.follow_runtime import continuous_follow_runtime_status
        from blacknode.pkg.blacknode_skills.follow_person.leader_follower_runtime import leader_follower_runtime_status
        continuous_follows = continuous_follow_runtime_status()
        leader_followers = leader_follower_runtime_status()
    except Exception:
        continuous_follows = []
        leader_followers = []
    try:
        from blacknode.pkg.blacknode_motion.policy.policy_runtime import (
            runtime_status as policy_runtime_status,
        )
        policy_runs = policy_runtime_status()
    except Exception:
        policy_runs = []
    return {
        "ok": True,
        "backend": _passive_backend(),
        "streams": live_streams,
        "managed_runs": live_runs,
        "node_outputs": node_outputs,
        "detached_count": len(live_detached),
        "continuous_follows": continuous_follows,
        "leader_followers": leader_followers,
        "policy_runs": policy_runs,
        "active": bool(live_streams or live_runs or live_detached or continuous_follows or leader_followers or policy_runs),
    }


def stop_runtime_services() -> dict[str, Any]:
    """Stop all ROS helpers this Blacknode process started for live workflows."""
    status_before = runtime_status()
    stream_result = stop_image_stream("")
    try:
        from blacknode.pkg.blacknode_skills.follow_person.follow_runtime import stop_continuous_follow_services
        from blacknode.pkg.blacknode_skills.follow_person.leader_follower_runtime import stop_leader_follower_services
        follow_result = stop_continuous_follow_services()
        leader_follower_result = stop_leader_follower_services()
    except ModuleNotFoundError:
        follow_result = {"ok": True, "stopped": 0, "error": ""}
        leader_follower_result = {"ok": True, "stopped": 0, "error": ""}
    except Exception as exc:
        follow_result = {"ok": False, "stopped": 0, "error": str(exc)}
        leader_follower_result = {"ok": False, "stopped": 0, "error": str(exc)}
    try:
        from blacknode.pkg.blacknode_motion.policy.policy_runtime import stop_policy_services
        policy_result = stop_policy_services()
    except ModuleNotFoundError:
        policy_result = {"ok": True, "stopped": 0, "error": ""}
    except Exception as exc:
        policy_result = {"ok": False, "stopped": 0, "error": str(exc)}

    managed_stopped = 0
    managed_errors: list[str] = []
    for run_id in sorted(set(_managed_detached) | set(_managed_docker_patterns)):
        result = stop_ros2_managed(run_id)
        if result.get("ok"):
            managed_stopped += int(result.get("stopped") or 0)
        else:
            managed_errors.append(str(result.get("error") or f"could not stop {run_id}"))

    detached_stopped = 0
    detached_errors: list[str] = []
    if _detached:
        result = stop_detached(pattern="ros2")
        if result.get("ok"):
            detached_stopped += int(result.get("stopped") or 0)
        else:
            detached_errors.append(str(result.get("error") or "could not stop detached ROS 2 process"))

    errors = managed_errors + detached_errors
    if not follow_result.get("ok"):
        errors.append(str(follow_result.get("error") or "could not stop continuous visual follow"))
    if not leader_follower_result.get("ok"):
        errors.append(str(leader_follower_result.get("error") or "could not stop leader-follower control"))
    if not policy_result.get("ok"):
        errors.append(str(policy_result.get("error") or "could not stop policy runtime"))
    stopped = {
        "streams": int(stream_result.get("stopped") or 0),
        "managed_runs": managed_stopped,
        "detached": detached_stopped,
        "continuous_follows": int(follow_result.get("stopped") or 0),
        "leader_followers": int(leader_follower_result.get("stopped") or 0),
        "policy_runs": int(policy_result.get("stopped") or 0),
    }
    return {
        "ok": not errors,
        "backend": _passive_backend(),
        "active_before": status_before,
        "stopped": stopped,
        "errors": errors,
        "report": (
            f"stopped {stopped['streams']} stream(s), "
            f"{stopped['managed_runs']} ROS 2 run process(es), "
            f"{stopped['detached']} detached ROS 2 process(es), "
            f"{stopped['continuous_follows']} continuous visual-follow loop(s), "
            f"{stopped['leader_followers']} leader-follower controller(s)"
            f", {stopped['policy_runs']} policy runtime(s)"
        ),
    }


def detect_backend(refresh: bool = False) -> dict[str, str]:
    """{"backend": "native"|"docker"|"none", "detail": ...} — cached after first call.

    Docker is launched on demand: if the CLI is present but its daemon isn't
    answering, :func:`ensure_docker_desktop` starts Docker Desktop and waits
    for it, so templates work without the user starting Docker themselves.
    """
    global _cached_backend
    if _cached_backend is not None and not refresh:
        return _cached_backend
    # Backend discovery can launch Docker Desktop and wait for its engine.
    # Serialize it so two explicit ROS actions cannot start or probe Docker at
    # the same time. Passive editor status never enters this function.
    with _backend_detection_lock:
        if _cached_backend is not None and not refresh:
            return _cached_backend
        native = shutil.which("ros2")
        if native:
            distro = os.environ.get("ROS_DISTRO", "")
            _cached_backend = {"backend": "native", "detail": f"{native}" + (f" ({distro})" if distro else "")}
        elif shutil.which("docker"):
            if not _docker_ok():
                launch_error = ensure_docker_desktop()
                if launch_error:
                    _cached_backend = {"backend": "none", "detail": launch_error}
                    return _cached_backend
            if _docker_ok():
                _cached_backend = {"backend": "docker", "detail": f"image {IMAGE}, container {CONTAINER}"}
            else:
                _cached_backend = {"backend": "none", "detail": _DOCKER_UNREACHABLE_HELP}
        else:
            _cached_backend = {"backend": "none", "detail": _NO_BACKEND_HELP}
    return _cached_backend


def _passive_backend() -> str:
    """Report known runtime state without probing or starting Docker."""
    if _cached_backend is not None:
        return _cached_backend["backend"]
    return "native" if shutil.which("ros2") else "none"


def _docker_ok() -> bool:
    try:
        return _run(["docker", "version", "--format", "{{.Server.Version}}"], 10).returncode == 0
    except Exception:
        return False


def _docker_desktop_executable() -> Path | None:
    candidates = [
        Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "Docker/Docker/Docker Desktop.exe",
        Path(os.environ.get("LOCALAPPDATA", "")) / "Docker/Docker Desktop.exe",
    ]
    return next((path for path in candidates if path.is_file()), None)


def ensure_docker_desktop(timeout: float = 90.0) -> str | None:
    """Launch Docker Desktop if it's installed but not answering, and wait for it.

    Windows-only (Blacknode's primary target here); a no-op elsewhere, or when
    a daemon is already up. Returns an error string describing what to do
    manually, or None once the daemon is reachable.
    """
    if _docker_ok():
        return None
    if sys.platform != "win32":
        return None
    executable = _docker_desktop_executable()
    if executable is None:
        return None
    try:
        subprocess.Popen([str(executable)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as exc:
        return f"could not launch Docker Desktop: {exc}"
    deadline = time.monotonic() + max(10.0, timeout)
    while time.monotonic() < deadline:
        if _docker_ok():
            return None
        time.sleep(2.0)
    return f"Docker Desktop was started but did not become ready within {timeout:g}s; open it manually and retry"


def _run(cmd: list[str], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def ensure_container() -> str | None:
    """Start the helper container if needed. Returns an error string or None."""
    try:
        check = _run(["docker", "ps", "-q", "--filter", f"name=^{CONTAINER}$"], 15)
    except subprocess.TimeoutExpired:
        return "docker ps timed out"
    if check.returncode != 0:
        return check.stderr.strip() or "docker ps failed"
    if check.stdout.strip() and _container_has_stream_ports():
        return None
    _run(["docker", "rm", "-f", CONTAINER], 30)  # clear a stopped leftover
    start = _run([
        "docker",
        "run",
        "-d",
        "--name",
        CONTAINER,
        *_docker_stream_port_args(),
        IMAGE,
        "sleep",
        "infinity",
    ], 180)
    if start.returncode != 0:
        return start.stderr.strip() or f"could not start {IMAGE} (docker pull {IMAGE} first?)"
    return None


def stop_container() -> None:
    if shutil.which("docker"):
        _run(["docker", "rm", "-f", CONTAINER], 30)


def _container_shell(args: list[str], timeout: float) -> str:
    # ros images export ROS_DISTRO; `timeout` bounds commands like `topic echo`
    # that would otherwise wait forever when nothing publishes.
    return (
        "source /opt/ros/$ROS_DISTRO/setup.bash && "
        f"timeout {max(1, int(timeout))}s ros2 {shlex.join(args)}"
    )


def _stream_port_bounds() -> tuple[int, int]:
    text = str(STREAM_PORT_RANGE or "").strip()
    if "-" in text:
        start_s, end_s = text.split("-", 1)
    else:
        start_s = end_s = text
    try:
        start = int(start_s)
        end = int(end_s)
    except ValueError:
        start, end = 39000, 39049
    start = max(1024, min(65535, start))
    end = max(start, min(65535, end))
    return start, end


def _docker_stream_port_args() -> list[str]:
    start, end = _stream_port_bounds()
    return ["-p", f"127.0.0.1:{start}-{end}:{start}-{end}/tcp"]


def _container_has_stream_ports() -> bool:
    start, _end = _stream_port_bounds()
    inspect = _run(["docker", "inspect", "-f", "{{json .NetworkSettings.Ports}}", CONTAINER], 15)
    return inspect.returncode == 0 and f'"{start}/tcp"' in (inspect.stdout or "")


def _copy_to_container(host_path: Path, container_path: str) -> str | None:
    if not host_path.exists():
        return f"helper not found: {host_path}"
    copied = _run(["docker", "cp", str(host_path), f"{CONTAINER}:{container_path}"], 30)
    if copied.returncode != 0:
        return copied.stderr.strip() or f"could not copy {host_path.name} into {CONTAINER}"
    return None


def resolve_workspace_path(workspace_path: str) -> tuple[Path | None, str]:
    """Resolve an editor workspace-relative ROS 2 workspace directory."""
    value = str(workspace_path or "").strip()
    if not value:
        return None, "workspace_path is required"

    path = Path(value).expanduser()
    if not path.is_absolute():
        relative = path
        bases = [Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents]
        candidates: list[Path] = []
        seen: set[Path] = set()
        for base in bases:
            candidate = (base / relative).resolve()
            if candidate in seen:
                continue
            seen.add(candidate)
            candidates.append(candidate)
        path = next((candidate for candidate in candidates if candidate.is_dir()), candidates[0])
    else:
        path = path.resolve()

    if not path.is_dir():
        return None, f"ROS 2 workspace not found: {path}"
    source_dir = path / "src"
    if not source_dir.is_dir():
        return None, f"ROS 2 workspace must contain a src directory: {source_dir}"
    if not any(source_dir.rglob("package.xml")):
        return None, f"no ROS 2 package.xml found below: {source_dir}"
    return path, ""


def _container_workspace_path(workspace: Path) -> str:
    """Return a stable, shell-safe container path for one host workspace."""
    digest = hashlib.sha256(str(workspace).encode("utf-8")).hexdigest()[:12]
    name = re.sub(r"[^A-Za-z0-9_]+", "_", workspace.name).strip("_")[:32] or "workspace"
    return f"{_CONTAINER_WORKSPACE_PREFIX}{name}_{digest}"


def _native_workspace_setup(workspace: Path) -> Path | None:
    candidates = (
        workspace / "install" / "setup.bat",
        workspace / "install" / "local_setup.bat",
        workspace / "install" / "setup.bash",
        workspace / "install" / "local_setup.bash",
    )
    return next((path for path in candidates if path.is_file()), None)


def _ensure_container_colcon() -> str | None:
    check = _run(
        ["docker", "exec", CONTAINER, "bash", "-lc", "command -v colcon"],
        15,
    )
    if check.returncode == 0:
        return None
    install = _run(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            (
                "apt-get update && DEBIAN_FRONTEND=noninteractive "
                "apt-get install -y python3-colcon-common-extensions"
            ),
        ],
        300,
    )
    if install.returncode == 0:
        return None
    return (
        install.stderr.strip()
        or install.stdout.strip()
        or "could not install colcon in the ROS 2 helper container"
    )


def build_ros2_workspace(
    workspace_path: str,
    *,
    packages_select: list[str] | None = None,
    timeout: float = 300.0,
) -> dict[str, Any]:
    """Build a local colcon workspace for the active native or Docker backend."""
    workspace, error = resolve_workspace_path(workspace_path)
    backend = detect_backend()["backend"]
    if workspace is None:
        return {
            "ok": False,
            "backend": backend,
            "workspace_path": workspace_path,
            "setup_path": "",
            "stdout": "",
            "stderr": "",
            "error": error,
        }
    if backend == "none":
        return {
            "ok": False,
            "backend": backend,
            "workspace_path": str(workspace),
            "setup_path": "",
            "stdout": "",
            "stderr": "",
            "error": _NO_BACKEND_HELP,
        }

    package_names = [str(name).strip() for name in (packages_select or []) if str(name).strip()]
    invalid = [name for name in package_names if not re.fullmatch(r"[a-z][a-z0-9_]*", name)]
    if invalid:
        return {
            "ok": False,
            "backend": backend,
            "workspace_path": str(workspace),
            "setup_path": "",
            "stdout": "",
            "stderr": "",
            "error": f"invalid ROS 2 package name: {invalid[0]}",
        }
    build_args = ["colcon", "build", "--symlink-install"]
    if package_names:
        build_args.extend(["--packages-select", *package_names])

    try:
        if backend == "docker":
            container_error = ensure_container() or _ensure_container_colcon()
            if container_error:
                return {
                    "ok": False,
                    "backend": backend,
                    "workspace_path": str(workspace),
                    "setup_path": "",
                    "stdout": "",
                    "stderr": "",
                    "error": container_error,
                }
            container_workspace = _container_workspace_path(workspace)
            prepared = _run(
                [
                    "docker", "exec", CONTAINER, "bash", "-lc",
                    f"rm -rf -- {container_workspace} && mkdir -p -- {container_workspace}",
                ],
                30,
            )
            if prepared.returncode != 0:
                message = prepared.stderr.strip() or "could not prepare ROS 2 container workspace"
                return {
                    "ok": False,
                    "backend": backend,
                    "workspace_path": str(workspace),
                    "setup_path": "",
                    "stdout": prepared.stdout.strip(),
                    "stderr": prepared.stderr.strip(),
                    "error": message,
                }
            copy_error = _copy_to_container(workspace / "src", container_workspace)
            if copy_error:
                return {
                    "ok": False,
                    "backend": backend,
                    "workspace_path": str(workspace),
                    "setup_path": "",
                    "stdout": "",
                    "stderr": copy_error,
                    "error": copy_error,
                }
            shell = (
                "source /opt/ros/$ROS_DISTRO/setup.bash && "
                f"cd {container_workspace} && {shlex.join(build_args)}"
            )
            proc = _run(
                ["docker", "exec", CONTAINER, "bash", "-lc", shell],
                max(30.0, float(timeout)),
            )
            setup_path = f"{container_workspace}/install/setup.bash"
        else:
            proc = subprocess.run(
                build_args,
                cwd=workspace,
                capture_output=True,
                text=True,
                timeout=max(30.0, float(timeout)),
            )
            setup = _native_workspace_setup(workspace)
            setup_path = str(setup) if setup else str(workspace / "install" / "setup.bash")
    except subprocess.TimeoutExpired:
        return {
            "ok": False,
            "backend": backend,
            "workspace_path": str(workspace),
            "setup_path": "",
            "stdout": "",
            "stderr": "",
            "error": f"colcon build timed out after {max(30.0, float(timeout)):g}s",
        }
    except Exception as exc:
        return {
            "ok": False,
            "backend": backend,
            "workspace_path": str(workspace),
            "setup_path": "",
            "stdout": "",
            "stderr": "",
            "error": f"{type(exc).__name__}: {exc}",
        }

    result = {
        "ok": proc.returncode == 0,
        "backend": backend,
        "workspace_path": str(workspace),
        "setup_path": setup_path,
        "stdout": proc.stdout.strip(),
        "stderr": proc.stderr.strip(),
    }
    if proc.returncode != 0:
        result["error"] = result["stderr"] or result["stdout"] or f"colcon exited with code {proc.returncode}"
    return result


def _ensure_container_stream_deps() -> str | None:
    check = _run([
        "docker",
        "exec",
        CONTAINER,
        "bash",
        "-lc",
        "python3 - <<'PY'\nimport rclpy, sensor_msgs, numpy, PIL\nPY",
    ], 30)
    if check.returncode == 0:
        return None

    install = _run([
        "docker",
        "exec",
        CONTAINER,
        "bash",
        "-lc",
        "apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y python3-numpy python3-pil",
    ], 240)
    if install.returncode != 0:
        return install.stderr.strip() or install.stdout.strip() or "could not install python3-pil in ROS 2 helper container"
    return None


def _container_pkg_prefix_ok(package: str) -> bool:
    check = _run(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            f"source /opt/ros/$ROS_DISTRO/setup.bash && ros2 pkg prefix {shlex.quote(package)}",
        ],
        15,
    )
    return check.returncode == 0


def _ensure_container_package(package: str) -> str | None:
    """Make sure a ROS 2 package resolves in the helper container, installing it via apt if not.

    Returns an error string if the package still can't be found after trying
    to install it, or None once ``ros2 pkg prefix <package>`` succeeds.
    """
    if _container_pkg_prefix_ok(package):
        return None
    apt_name = f"ros-$ROS_DISTRO-{package.replace('_', '-')}"
    install = _run(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            f"apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y {apt_name}",
        ],
        240,
    )
    if install.returncode != 0:
        detail = install.stderr.strip() or install.stdout.strip() or "apt-get install failed"
        return (
            f"ROS 2 package '{package}' is not installed in the {IMAGE} helper container, "
            f"and installing it automatically failed: {detail}"
        )
    if not _container_pkg_prefix_ok(package):
        return (
            f"installed {apt_name} but ROS 2 still can't find package '{package}' "
            "(the apt package name may not match; install the correct one manually)"
        )
    return None


def run_ros2(args: list[str], timeout: float = 15.0) -> dict[str, Any]:
    """Run ``ros2 <args>``; returns {ok, stdout, stderr, backend, error?, timed_out?}."""
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "stdout": "", "stderr": "", "backend": backend, "error": _NO_BACKEND_HELP}
    # Logged before it runs, so a command that blocks is visible while it blocks
    # rather than only once it returns.
    logged = console.record("ros2 " + " ".join(args), backend=backend, source="ros2")
    try:
        if backend == "native":
            # Suppressed because this call reports itself above, with duration
            # and output the bare spawn record cannot carry.
            with console.suppress():
                proc = _run(["ros2", *args], timeout)
            timed_out = False
        else:
            err = ensure_container()
            if err:
                return {"ok": False, "stdout": "", "stderr": err, "backend": backend, "error": err}
            with console.suppress():
                proc = _run(
                    ["docker", "exec", CONTAINER, "bash", "-lc", _container_shell(args, timeout)],
                    timeout + 15,
                )
            timed_out = proc.returncode == 124  # GNU timeout exit code
    except subprocess.TimeoutExpired:
        message = f"`ros2 {' '.join(args)}` timed out after {timeout:g}s"
        logged.finish(False, error=message)
        return {
            "ok": False, "stdout": "", "stderr": "", "backend": backend,
            "error": message, "timed_out": True,
        }
    result: dict[str, Any] = {
        "ok": proc.returncode == 0,
        "stdout": proc.stdout.strip(),
        "stderr": proc.stderr.strip(),
        "backend": backend,
    }
    if timed_out:
        result["timed_out"] = True
        result["error"] = f"`ros2 {' '.join(args)}` timed out after {timeout:g}s"
    elif not result["ok"]:
        result["error"] = result["stderr"] or f"ros2 exited with code {proc.returncode}"
    logged.finish(
        bool(result["ok"]),
        stdout=result["stdout"],
        stderr=result["stderr"],
        error=str(result.get("error") or ""),
        exit_code=proc.returncode,
    )
    return result


def run_ros2_detached(args: list[str]) -> dict[str, Any]:
    """Start ``ros2 <args>`` in the background."""
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    try:
        if backend == "native":
            proc = subprocess.Popen(
                ["ros2", *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
            )
            _detached.append(proc)
        else:
            err = ensure_container()
            if err:
                return {"ok": False, "backend": backend, "error": err}
            # `ros2 launch <package> ...` fails silently under `docker exec -d`
            # when the package is missing, so install it first exactly like the
            # `ros2 run` path does.
            if len(args) >= 2 and args[0] == "launch":
                package_error = _ensure_container_package(args[1])
                if package_error:
                    return {"ok": False, "backend": backend, "error": package_error}
            shell = f"source /opt/ros/$ROS_DISTRO/setup.bash && exec ros2 {shlex.join(args)}"
            proc = _run(["docker", "exec", "-d", CONTAINER, "bash", "-lc", shell], 30)
            if proc.returncode != 0:
                return {"ok": False, "backend": backend, "error": proc.stderr.strip() or "docker exec failed"}
    except Exception as exc:  # never break the graph
        return {"ok": False, "backend": backend, "error": str(exc)}
    return {"ok": True, "backend": backend}


def stop_detached(pattern: str = "ros2 topic pub") -> dict[str, Any]:
    """Stop background publishers started by :func:`run_ros2_detached`."""
    backend = detect_backend()["backend"]
    stopped = 0
    if backend == "native":
        for proc in _detached:
            if _terminate_process(proc):
                stopped += 1
        _detached.clear()
        return {"ok": True, "backend": backend, "stopped": stopped}
    if backend == "docker":
        proc = _run(["docker", "exec", CONTAINER, "pkill", "-f", pattern], 15)
        # pkill: 0 = killed something, 1 = nothing matched — both fine
        if proc.returncode in (0, 1):
            return {"ok": True, "backend": backend, "stopped": 1 if proc.returncode == 0 else 0}
        return {"ok": False, "backend": backend, "error": proc.stderr.strip() or "pkill failed"}
    return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}


def run_ros2_managed(
    key: str,
    args: list[str],
    *,
    workspace_path: str = "",
) -> dict[str, Any]:
    """Start one named background ``ros2 <args>`` process, optionally in an overlay."""
    stop_ros2_managed(key, pattern=f"ros2 {shlex.join(args)}")
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    workspace: Path | None = None
    if str(workspace_path or "").strip():
        workspace, workspace_error = resolve_workspace_path(workspace_path)
        if workspace is None:
            return {"ok": False, "backend": backend, "error": workspace_error}
    try:
        if backend == "native":
            if workspace is None:
                command = ["ros2", *args]
            else:
                setup = _native_workspace_setup(workspace)
                if setup is None:
                    return {
                        "ok": False,
                        "backend": backend,
                        "error": (
                            f"workspace is not built: {workspace}. "
                            "Run ROS2WorkspaceBuild first."
                        ),
                    }
                ros_command = subprocess.list2cmdline(["ros2", *args])
                if setup.suffix.lower() == ".bat":
                    command = [
                        "cmd.exe", "/d", "/s", "/c",
                        f'call "{setup}" && {ros_command}',
                    ]
                else:
                    command = [
                        "bash", "-lc",
                        f"source {shlex.quote(str(setup))} && exec ros2 {shlex.join(args)}",
                    ]
            proc = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            _managed_detached[key] = proc
            return {"ok": True, "backend": backend, "pid": proc.pid}
        err = ensure_container()
        if err:
            return {"ok": False, "backend": backend, "error": err}
        # `docker exec -d` only confirms the shell was dispatched, not that
        # `ros2 run <package> <executable>` actually started inside it -- a
        # missing package fails silently there, producing a false "started"
        # report followed by a confusing downstream timeout. Verify (and, if
        # missing, install) the package first so a missing package either
        # gets fixed automatically or fails immediately with an actionable
        # message, instead of surfacing as an opaque "no active publisher"
        # from whatever's downstream of this run.
        overlay_setup = ""
        if workspace is not None:
            container_workspace = _container_workspace_path(workspace)
            overlay_setup = f"{container_workspace}/install/setup.bash"
            setup_check = _run(
                ["docker", "exec", CONTAINER, "test", "-f", overlay_setup],
                15,
            )
            if setup_check.returncode != 0:
                return {
                    "ok": False,
                    "backend": backend,
                    "error": (
                        f"workspace is not built in the ROS helper container: {workspace}. "
                        "Run ROS2WorkspaceBuild first."
                    ),
                }
        if len(args) >= 2 and args[0] in {"run", "launch"} and not overlay_setup:
            package = args[1]
            package_error = _ensure_container_package(package)
            if package_error:
                return {"ok": False, "backend": backend, "error": package_error}
        source_overlay = f"source {overlay_setup} && " if overlay_setup else ""
        if len(args) >= 2 and args[0] in {"run", "launch"} and overlay_setup:
            package = args[1]
            package_check = _run(
                [
                    "docker", "exec", CONTAINER, "bash", "-lc",
                    (
                        "source /opt/ros/$ROS_DISTRO/setup.bash && "
                        f"source {overlay_setup} && ros2 pkg prefix {shlex.quote(package)}"
                    ),
                ],
                30,
            )
            if package_check.returncode != 0:
                return {
                    "ok": False,
                    "backend": backend,
                    "error": (
                        package_check.stderr.strip()
                        or f"ROS 2 package '{package}' is not available in workspace {workspace}"
                    ),
                }
        shell = (
            "source /opt/ros/$ROS_DISTRO/setup.bash && "
            f"{source_overlay}exec ros2 {shlex.join(args)}"
        )
        proc = _run(["docker", "exec", "-d", CONTAINER, "bash", "-lc", shell], 30)
        if proc.returncode != 0:
            return {"ok": False, "backend": backend, "error": proc.stderr.strip() or "docker exec failed"}
        _managed_docker_patterns[key] = re.escape(
            "ros2 " + shlex.join(args)
        )
        return {"ok": True, "backend": backend}
    except Exception as exc:
        return {"ok": False, "backend": backend, "error": str(exc)}


def ros2_managed_status(key: str) -> dict[str, Any]:
    """Return the current state of one named ROS 2 process."""
    clean_key = str(key or "").strip()
    if not clean_key:
        return {
            "ok": False,
            "running": False,
            "backend": _passive_backend(),
            "error": "managed ROS 2 service ID is required",
        }
    proc = _managed_detached.get(clean_key)
    if proc is not None:
        return {
            "ok": True,
            "running": proc.poll() is None,
            "backend": "native",
            "pid": proc.pid,
            "exit_code": proc.poll(),
        }
    pattern = _managed_docker_patterns.get(clean_key, "")
    if pattern:
        backend = detect_backend()["backend"]
        if backend != "docker":
            return {
                "ok": True,
                "running": False,
                "backend": backend,
                "error": "the Docker-backed ROS 2 service is no longer reachable",
            }
        result = _run(
            ["docker", "exec", CONTAINER, "pgrep", "-f", pattern],
            15,
        )
        return {
            "ok": result.returncode in (0, 1),
            "running": result.returncode == 0,
            "backend": "docker",
            "error": (
                ""
                if result.returncode in (0, 1)
                else result.stderr.strip() or "could not inspect Docker process"
            ),
        }
    return {
        "ok": True,
        "running": False,
        "backend": _passive_backend(),
    }


def _safe_python_run_id(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_-]+", "_", str(value or "").strip()).strip("_")[:64]


def _python_node_source(
    *,
    source_mode: str,
    script_path: str,
    code: str,
    run_id: str,
) -> tuple[Path | None, str]:
    if source_mode == "file":
        path = Path(script_path).expanduser()
        if not path.is_absolute():
            relative = path
            bases = [Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents]
            candidates: list[Path] = []
            seen: set[Path] = set()
            for base in bases:
                candidate = (base / relative).resolve()
                if candidate in seen:
                    continue
                seen.add(candidate)
                candidates.append(candidate)
            path = next((candidate for candidate in candidates if candidate.is_file()), candidates[0])
        else:
            path = path.resolve()
        if not path.is_file():
            return None, f"Python script not found: {path}"
        if path.suffix.lower() != ".py":
            return None, "ROS 2 Python node files must use the .py extension"
        try:
            source = path.read_text(encoding="utf-8")
        except Exception as exc:
            return None, f"could not read Python script: {type(exc).__name__}: {exc}"
    elif source_mode == "inline":
        source = str(code or "")
        if not source.strip():
            return None, "inline Python code is empty"
        directory = Path(tempfile.gettempdir()) / "blacknode-ros2-python-nodes"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{run_id}.py"
        try:
            path.write_text(source, encoding="utf-8")
        except Exception as exc:
            return None, f"could not prepare inline Python code: {type(exc).__name__}: {exc}"
    else:
        return None, f"source_mode must be file or inline, got {source_mode!r}"

    try:
        compile(source, str(path), "exec")
    except SyntaxError as exc:
        location = f"line {exc.lineno}" if exc.lineno else "unknown line"
        return None, f"Python syntax error at {location}: {exc.msg}"
    return path, ""


def start_ros2_python_node(
    *,
    run_id: str,
    source_mode: str,
    script_path: str,
    code: str,
    arguments: list[str],
) -> dict[str, Any]:
    """Start a standalone ``rclpy`` script as one managed ROS 2 process."""
    clean_id = _safe_python_run_id(run_id)
    if not clean_id:
        return {
            "ok": False,
            "backend": _passive_backend(),
            "error": "run_id must contain a letter, number, underscore, or hyphen",
        }
    source, source_error = _python_node_source(
        source_mode=source_mode,
        script_path=script_path,
        code=code,
        run_id=clean_id,
    )
    if source is None:
        return {"ok": False, "backend": _passive_backend(), "error": source_error}
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}

    stop_ros2_python_node(clean_id)
    display_source = str(source if source_mode == "file" else "inline code")
    try:
        if backend == "docker":
            error = ensure_container()
            container_script = f"{_CONTAINER_PYTHON_NODE_PREFIX}{clean_id}.py"
            if not error:
                error = _copy_to_container(source, container_script)
            if error:
                return {"ok": False, "backend": backend, "error": error}
            checked = _run(
                [
                    "docker", "exec", CONTAINER, "bash", "-lc",
                    (
                        "source /opt/ros/$ROS_DISTRO/setup.bash && "
                        f"python3 -m py_compile {shlex.quote(container_script)}"
                    ),
                ],
                30,
            )
            if checked.returncode != 0:
                return {
                    "ok": False,
                    "backend": backend,
                    "error": checked.stderr.strip() or "Python syntax validation failed in ROS Docker",
                }
            container_log = f"{_CONTAINER_PYTHON_NODE_PREFIX}{clean_id}.log"
            command_prefix = f"python3 {container_script}"
            shell = (
                "source /opt/ros/$ROS_DISTRO/setup.bash && "
                f"exec {command_prefix} {shlex.join(arguments)} "
                f"> {shlex.quote(container_log)} 2>&1"
            )
            started = _run(
                ["docker", "exec", "-d", CONTAINER, "bash", "-lc", shell],
                30,
            )
            if started.returncode != 0:
                return {
                    "ok": False,
                    "backend": backend,
                    "error": started.stderr.strip() or "docker exec failed",
                }
            _managed_docker_patterns[clean_id] = re.escape(command_prefix)
            log_location = container_log
        else:
            log_dir = Path(tempfile.gettempdir()) / "blacknode-ros2-python-nodes"
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"{clean_id}.log"
            with log_path.open("w", encoding="utf-8") as log:
                proc = subprocess.Popen(
                    [sys.executable, str(source), *arguments],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            _managed_detached[clean_id] = proc
            log_location = str(log_path)
    except Exception as exc:
        return {"ok": False, "backend": backend, "error": f"{type(exc).__name__}: {exc}"}

    _python_nodes[clean_id] = {
        "backend": backend,
        "run_id": clean_id,
        "script": display_source,
        "log_location": log_location,
        "logs": [],
        "running": True,
        "last_log_poll": 0.0,
    }

    time.sleep(0.35)
    status = ros2_managed_status(clean_id)
    if not status.get("running"):
        error = "ROS 2 Python node exited during startup"
        if backend == "docker":
            log_result = _run(
                [
                    "docker", "exec", CONTAINER, "bash", "-lc",
                    f"tail -n 20 {_CONTAINER_PYTHON_NODE_PREFIX}{clean_id}.log 2>/dev/null || true",
                ],
                15,
            )
            if log_result.stdout.strip():
                error += f": {log_result.stdout.strip()}"
        else:
            log_path = Path(tempfile.gettempdir()) / "blacknode-ros2-python-nodes" / f"{clean_id}.log"
            if log_path.is_file():
                detail = log_path.read_text(encoding="utf-8", errors="replace").strip()
                if detail:
                    error += f": {detail}"
        stop_ros2_python_node(clean_id)
        return {"ok": False, "backend": backend, "error": error}
    return {
        "ok": True,
        "running": True,
        "backend": backend,
        "run_id": clean_id,
        "script": display_source,
    }


def stop_ros2_python_node(run_id: str) -> dict[str, Any]:
    """Stop one managed standalone Python node, including after editor reloads."""
    clean_id = _safe_python_run_id(run_id)
    if not clean_id:
        return {
            "ok": False,
            "backend": _passive_backend(),
            "stopped": 0,
            "error": "run_id is required",
        }
    pattern = ""
    if detect_backend()["backend"] == "docker":
        pattern = re.escape(f"python3 {_CONTAINER_PYTHON_NODE_PREFIX}{clean_id}.py")
    result = stop_ros2_managed(clean_id, pattern=pattern)
    item = _python_nodes.get(clean_id)
    if item is not None:
        item["running"] = False
        item["last_log_poll"] = 0.0
        _refresh_python_node_record(item)
        result["logs"] = list(item.get("logs") or [])
    return result


def _refresh_python_node_record(item: dict[str, Any]) -> None:
    now = time.monotonic()
    if now - float(item.get("last_log_poll") or 0.0) < 0.75:
        return
    item["last_log_poll"] = now
    backend = str(item.get("backend") or "")
    location = str(item.get("log_location") or "")
    if backend == "docker" and location:
        result = _run(
            [
                "docker", "exec", CONTAINER, "bash", "-lc",
                f"tail -n 50 {shlex.quote(location)} 2>/dev/null || true",
            ],
            10,
        )
        if result.returncode == 0:
            item["logs"] = [line for line in result.stdout.splitlines() if line.strip()]
    elif location:
        path = Path(location)
        if path.is_file():
            try:
                item["logs"] = [
                    line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()[-50:]
                    if line.strip()
                ]
            except Exception:
                pass


def _python_node_runtime_outputs() -> list[dict[str, Any]]:
    outputs: list[dict[str, Any]] = []
    for run_id, item in list(_python_nodes.items()):
        running = bool(item.get("running"))
        if running:
            status = ros2_managed_status(run_id)
            running = bool(status.get("running"))
            item["running"] = running
        _refresh_python_node_record(item)
        logs = list(item.get("logs") or [])
        script = str(item.get("script") or "")
        backend = str(item.get("backend") or "")
        outputs.append({
            "node_type": "ROS2PythonNode",
            "run_id": run_id,
            "outputs": {
                "running": running,
                "run_id": run_id,
                "backend": backend,
                "script": script,
                "logs": logs,
                "report": (
                    f"ROS 2 Python node {run_id} running from {script} via {backend}"
                    if running
                    else f"ROS 2 Python node {run_id} stopped"
                ),
            },
        })
    return outputs


def inspect_topic_interfaces(
    expectations: list[dict[str, Any]],
) -> dict[str, Any]:
    """Inspect expected topic types and publisher endpoints in one ROS graph."""
    normalized: list[dict[str, Any]] = []
    for item in expectations:
        if not isinstance(item, dict):
            continue
        topic = str(item.get("topic") or "").strip()
        if not topic:
            continue
        normalized.append({
            "name": str(item.get("name") or topic).strip() or topic,
            "topic": topic,
            "message_type": str(item.get("message_type") or "").strip(),
            "required": bool(item.get("required", True)),
        })
    if not normalized:
        return {
            "ok": False,
            "ready": False,
            "backend": _passive_backend(),
            "interfaces": [],
            "error": "at least one expected ROS 2 topic is required",
        }

    listing = run_ros2(["topic", "list", "-t"], timeout=15)
    if not listing.get("ok"):
        return {
            "ok": False,
            "ready": False,
            "backend": listing.get("backend", _passive_backend()),
            "interfaces": [],
            "error": str(
                listing.get("error")
                or listing.get("stderr")
                or "could not list ROS 2 topics"
            ),
        }
    discovered: dict[str, str] = {}
    for raw_line in str(listing.get("stdout") or "").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        match = re.match(r"^(\S+)\s+\[([^\]]+)\]\s*$", line)
        if match:
            discovered[match.group(1)] = match.group(2).strip()
        else:
            discovered[line.split()[0]] = ""

    interfaces: list[dict[str, Any]] = []
    for expected in normalized:
        topic = expected["topic"]
        actual_type = discovered.get(topic, "")
        present = topic in discovered
        type_matches = bool(
            present
            and (
                not expected["message_type"]
                or not actual_type
                or actual_type == expected["message_type"]
            )
        )
        publisher_count: int | None = None
        detail_error = ""
        if present and type_matches:
            detail = run_ros2(["topic", "info", topic, "-v"], timeout=15)
            if detail.get("ok"):
                count_match = re.search(
                    r"Publisher count:\s*(\d+)",
                    str(detail.get("stdout") or ""),
                    re.IGNORECASE,
                )
                if count_match:
                    publisher_count = int(count_match.group(1))
            else:
                detail_error = str(
                    detail.get("error")
                    or detail.get("stderr")
                    or "could not inspect topic endpoints"
                )
        publishing = bool(
            present
            and type_matches
            and publisher_count != 0
        )
        if not present:
            status = "missing"
        elif not type_matches:
            status = "type_mismatch"
        elif publisher_count == 0:
            status = "no_publisher"
        elif publisher_count is None:
            status = "present"
        else:
            status = "publishing"
        interfaces.append({
            **expected,
            "actual_message_type": actual_type,
            "present": present,
            "type_matches": type_matches,
            "publisher_count": publisher_count,
            "publishing": publishing,
            "status": status,
            "error": detail_error,
        })

    blocking = [
        item
        for item in interfaces
        if item["required"] and not item["publishing"]
    ]
    return {
        "ok": True,
        "ready": not blocking,
        "backend": listing.get("backend", _passive_backend()),
        "interfaces": interfaces,
        "missing": [item["topic"] for item in blocking],
    }


def wait_for_topic_interfaces(
    expectations: list[dict[str, Any]],
    *,
    timeout: float,
    interval: float = 0.5,
) -> dict[str, Any]:
    """Wait for required topic publishers while preserving structured status."""
    deadline = time.monotonic() + max(0.0, float(timeout))
    result = inspect_topic_interfaces(expectations)
    while (
        result.get("ok")
        and not result.get("ready")
        and time.monotonic() < deadline
    ):
        time.sleep(max(0.05, float(interval)))
        result = inspect_topic_interfaces(expectations)
    return result


def stop_ros2_managed(key: str, pattern: str = "") -> dict[str, Any]:
    """Stop one named background process."""
    backend = detect_backend()["backend"]
    stopped = 0
    proc = _managed_detached.pop(key, None)
    docker_pattern = _managed_docker_patterns.pop(key, "")
    pattern = pattern or docker_pattern
    if proc is not None and _terminate_process(proc):
        stopped += 1
    if backend == "native" and pattern and shutil.which("pkill"):
        result = _run(["pkill", "-f", pattern], 15)
        if result.returncode not in (0, 1):
            return {"ok": False, "backend": backend, "stopped": stopped, "error": result.stderr.strip() or "pkill failed"}
        stopped += 1 if result.returncode == 0 else 0
    if backend == "docker" and pattern:
        result = _run(["docker", "exec", CONTAINER, "pkill", "-f", pattern], 15)
        if result.returncode not in (0, 1):
            return {"ok": False, "backend": backend, "stopped": stopped, "error": result.stderr.strip() or "pkill failed"}
        stopped += 1 if result.returncode == 0 else 0
    return {"ok": True, "backend": backend, "stopped": stopped}


def _topic_subscriber_script() -> Path:
    return Path(__file__).resolve().parents[1] / "scripts" / "ros2_topic_subscriber.py"


def _read_topic_subscriber_output(key: str, stream: Any, *, error: bool = False) -> None:
    """Drain one subscriber pipe without ever blocking the runtime status path."""
    try:
        for raw_line in iter(stream.readline, ""):
            line = str(raw_line or "").strip()
            if not line:
                continue
            item = _topic_subscribers.get(key)
            if item is None:
                break
            if error:
                item["errors"].append(line)
                continue
            try:
                decoded = json.loads(line)
                message = decoded.get("message") if isinstance(decoded, dict) else None
            except json.JSONDecodeError:
                item["errors"].append(line)
                continue
            if isinstance(message, dict):
                item["messages"].append(message)
                item["received"] = int(item.get("received") or 0) + 1
                item["last_message_time_ns"] = time.time_ns()
                item["last_message_monotonic"] = time.monotonic()
    finally:
        try:
            stream.close()
        except Exception:
            pass


def start_topic_subscriber(
    *,
    topic: str,
    message_type: str,
    node_name: str,
    history: int = 10,
    max_messages: int = 0,
    public_node_type: str = "ROS2TopicSubscriber",
    stale_after_seconds: float = 2.0,
) -> dict[str, Any]:
    """Start one named subscriber and retain a bounded structured message history."""
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    script = _topic_subscriber_script()
    if not script.exists():
        return {
            "ok": False,
            "backend": backend,
            "error": f"topic subscriber helper not found: {script}",
        }
    interface = run_ros2(["interface", "show", message_type], timeout=15)
    if not interface.get("ok"):
        return {
            "ok": False,
            "backend": backend,
            "error": (
                f"ROS 2 message type '{message_type}' is unavailable: "
                f"{interface.get('error') or interface.get('stderr') or 'interface lookup failed'}"
            ),
        }

    key = f"topic-subscriber:{topic}"
    stop_topic_subscriber(topic)
    helper_args = [
        "--node-name", node_name,
        "--topic", topic,
        "--message-type", message_type,
        "--max-messages", str(max(0, int(max_messages))),
    ]
    command: list[str]
    if backend == "docker":
        error = ensure_container() or _copy_to_container(
            script,
            _CONTAINER_TOPIC_SUBSCRIBER_SCRIPT,
        )
        if error:
            return {"ok": False, "backend": backend, "error": error}
        shell = (
            "source /opt/ros/$ROS_DISTRO/setup.bash && "
            f"exec python3 {_CONTAINER_TOPIC_SUBSCRIBER_SCRIPT} {shlex.join(helper_args)}"
        )
        command = ["docker", "exec", CONTAINER, "bash", "-lc", shell]
    else:
        command = [sys.executable, str(script), *helper_args]

    try:
        proc = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            start_new_session=True,
        )
    except Exception as exc:
        return {"ok": False, "backend": backend, "error": f"{type(exc).__name__}: {exc}"}

    _managed_detached[key] = proc
    if backend == "docker":
        _managed_docker_patterns[key] = (
            r"ros2_topic_subscriber\.py .*--topic " + re.escape(topic)
        )
    _topic_subscribers[key] = {
        "proc": proc,
        "backend": backend,
        "topic": topic,
        "message_type": message_type,
        "node_name": node_name,
        "messages": deque(maxlen=max(1, min(100, int(history)))),
        "errors": deque(maxlen=20),
        "received": 0,
        "public_node_type": public_node_type,
        "stale_after_seconds": max(0.05, float(stale_after_seconds)),
        "last_message_time_ns": 0,
        "last_message_monotonic": 0.0,
    }
    assert proc.stdout is not None
    assert proc.stderr is not None
    threading.Thread(
        target=_read_topic_subscriber_output,
        args=(key, proc.stdout),
        daemon=True,
    ).start()
    threading.Thread(
        target=_read_topic_subscriber_output,
        args=(key, proc.stderr),
        kwargs={"error": True},
        daemon=True,
    ).start()

    # Fail fast when imports, node creation, or the interface are invalid.
    time.sleep(0.25)
    if proc.poll() is not None and max_messages <= 0:
        errors = list(_topic_subscribers.get(key, {}).get("errors") or [])
        _managed_detached.pop(key, None)
        _managed_docker_patterns.pop(key, None)
        return {
            "ok": False,
            "backend": backend,
            "error": errors[-1] if errors else "topic subscriber exited during startup",
        }
    return {"ok": True, "backend": backend, "run_id": key}


def _topic_subscriber_snapshot(item: dict[str, Any]) -> dict[str, Any]:
    proc = item.get("proc")
    messages = list(item.get("messages") or [])
    errors = list(item.get("errors") or [])
    last_monotonic = float(item.get("last_message_monotonic") or 0.0)
    age_seconds = max(0.0, time.monotonic() - last_monotonic) if last_monotonic else None
    stale_after_seconds = max(0.05, float(item.get("stale_after_seconds") or 2.0))
    running = bool(proc is not None and proc.poll() is None)
    return {
        "ok": not (errors and not messages and not running),
        "running": running,
        "backend": item.get("backend", ""),
        "topic": item.get("topic", ""),
        "message_type": item.get("message_type", ""),
        "node_name": item.get("node_name", ""),
        "service_id": f"topic-subscriber:{item.get('topic', '')}",
        "messages": messages,
        "latest": messages[-1] if messages else {},
        "received": int(item.get("received") or 0),
        "last_message_time_ns": int(item.get("last_message_time_ns") or 0),
        "age_seconds": age_seconds,
        "stale_after_seconds": stale_after_seconds,
        "source_fresh": bool(messages and age_seconds is not None and age_seconds <= stale_after_seconds),
        "error": errors[-1] if errors else "",
    }


def ros2_topic_outputs(status: dict[str, Any], *, report: str = "") -> dict[str, Any]:
    """Normalize a managed subscriber snapshot for the generic ROS2 node."""
    running = bool(status.get("running"))
    source_fresh = bool(status.get("source_fresh"))
    error = str(status.get("error") or "")
    backend = str(status.get("backend") or _passive_backend())
    explicit_state = str(status.get("state") or "").strip().lower()
    if explicit_state in {"error", "ready", "stale", "waiting", "stopped", "unavailable"}:
        state = explicit_state
    elif backend == "none":
        state = "unavailable"
        error = error or _NO_BACKEND_HELP
    elif error:
        state = "error"
    elif source_fresh:
        state = "ready"
    elif running and status.get("received"):
        state = "stale"
    elif running:
        state = "waiting"
    else:
        state = "stopped"
    topic = str(status.get("topic") or "")
    message_type = str(status.get("message_type") or "")
    service_id = str(status.get("service_id") or f"topic-subscriber:{topic}")
    stream = {
        "kind": "blacknode.message-stream",
        "schema_version": 1,
        "stream_id": service_id,
        "protocol": "ros2",
        "state": state,
        "managed": True,
        "topic": topic,
        "message_type": message_type,
        "backend": backend,
    }
    health = {
        "kind": "blacknode.stream-status",
        "schema_version": 1,
        "stream_id": service_id,
        "state": state,
        "available": backend != "none",
        "worker_alive": running,
        "source_fresh": source_fresh,
        "received": int(status.get("received") or 0),
        "last_message_time_ns": int(status.get("last_message_time_ns") or 0),
        "age_seconds": status.get("age_seconds"),
        "stale_after_seconds": float(status.get("stale_after_seconds") or 2.0),
        "error": error,
    }
    return {
        "running": running,
        "message": status.get("latest") or {},
        "messages": list(status.get("messages") or []),
        "stream": stream,
        "status": health,
        "received": int(status.get("received") or 0),
        "backend": backend,
        "report": report or f"ROS2 {state}: {topic or '(topic not set)'}",
    }


def topic_subscriber_status(topic: str) -> dict[str, Any]:
    key = f"topic-subscriber:{topic}"
    item = _topic_subscribers.get(key)
    if item is None:
        return {
            "ok": True,
            "running": False,
            "backend": _passive_backend(),
            "messages": [],
            "received": 0,
            "topic": topic,
            "message_type": "",
            "service_id": key,
            "last_message_time_ns": 0,
            "age_seconds": None,
            "stale_after_seconds": 2.0,
            "source_fresh": False,
            "error": "",
        }
    return _topic_subscriber_snapshot(item)


def stop_topic_subscriber(topic: str) -> dict[str, Any]:
    key = f"topic-subscriber:{topic}"
    item = _topic_subscribers.get(key)
    backend = detect_backend()["backend"]
    pattern = ""
    if backend == "docker":
        pattern = r"ros2_topic_subscriber\.py .*--topic " + re.escape(topic)
    result = stop_ros2_managed(key, pattern=pattern)
    if item is not None:
        result.update(_topic_subscriber_snapshot(item))
        result["running"] = False
    _topic_subscribers.pop(key, None)
    return result


def run_topic_subscriber_once(
    *,
    topic: str,
    message_type: str,
    node_name: str,
    timeout: float,
    public_node_type: str = "ROS2TopicSubscriber",
    stale_after_seconds: float = 2.0,
) -> dict[str, Any]:
    started = start_topic_subscriber(
        topic=topic,
        message_type=message_type,
        node_name=node_name,
        history=1,
        max_messages=1,
        public_node_type=public_node_type,
        stale_after_seconds=stale_after_seconds,
    )
    if not started.get("ok"):
        return {**started, "running": False, "messages": [], "received": 0}
    deadline = time.monotonic() + max(0.1, float(timeout))
    status = topic_subscriber_status(topic)
    while status.get("running") and not status.get("messages") and time.monotonic() < deadline:
        time.sleep(0.05)
        status = topic_subscriber_status(topic)
    stopped = stop_topic_subscriber(topic)
    messages = list(status.get("messages") or stopped.get("messages") or [])
    return {
        "ok": bool(messages),
        "running": False,
        "backend": started.get("backend", ""),
        "messages": messages,
        "latest": messages[-1] if messages else {},
        "received": len(messages),
        "topic": topic,
        "message_type": message_type,
        "service_id": f"topic-subscriber:{topic}",
        "last_message_time_ns": int(status.get("last_message_time_ns") or 0),
        "age_seconds": status.get("age_seconds"),
        "stale_after_seconds": max(0.05, float(stale_after_seconds)),
        "source_fresh": bool(messages),
        "error": "" if messages else f"no message received from {topic} within {timeout:g}s",
    }


def _topic_relay_script() -> Path:
    return Path(__file__).resolve().parents[1] / "scripts" / "ros2_topic_relay.py"


def start_topic_relay(
    *,
    run_id: str,
    source_topic: str,
    destination_topic: str,
    message_type: str,
    qos: str,
    queue_depth: int,
) -> dict[str, Any]:
    """Start one managed, type-preserving ROS 2 data-topic relay."""
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    script = _topic_relay_script()
    if not script.exists():
        return {
            "ok": False,
            "backend": backend,
            "error": f"topic relay helper not found: {script}",
        }

    interface = run_ros2(["interface", "show", message_type], timeout=15)
    if not interface.get("ok"):
        return {
            "ok": False,
            "backend": backend,
            "error": (
                f"ROS 2 message type '{message_type}' is unavailable: "
                f"{interface.get('error') or interface.get('stderr') or 'interface lookup failed'}"
            ),
        }

    key = f"topic-relay:{run_id}"
    stop_topic_relay(run_id)
    helper_args = [
        "--run-id", run_id,
        "--source-topic", source_topic,
        "--destination-topic", destination_topic,
        "--message-type", message_type,
        "--qos", qos,
        "--queue-depth", str(max(1, int(queue_depth))),
    ]
    if backend == "docker":
        error = ensure_container() or _copy_to_container(
            script,
            _CONTAINER_TOPIC_RELAY_SCRIPT,
        )
        if error:
            return {"ok": False, "backend": backend, "error": error}
        shell = (
            "source /opt/ros/$ROS_DISTRO/setup.bash && "
            f"exec python3 {_CONTAINER_TOPIC_RELAY_SCRIPT} {shlex.join(helper_args)}"
        )
        started = _run(
            ["docker", "exec", "-d", CONTAINER, "bash", "-lc", shell],
            30,
        )
        if started.returncode != 0:
            return {
                "ok": False,
                "backend": backend,
                "error": started.stderr.strip() or "docker exec failed",
            }
        _managed_docker_patterns[key] = (
            r"ros2_topic_relay\.py .*--run-id "
            + re.escape(run_id)
        )
    else:
        try:
            process = subprocess.Popen(
                [sys.executable, str(script), *helper_args],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            return {
                "ok": False,
                "backend": backend,
                "error": f"{type(exc).__name__}: {exc}",
            }
        _managed_detached[key] = process

    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        process = _managed_detached.get(key)
        if process is not None and process.poll() is not None:
            _managed_detached.pop(key, None)
            return {
                "ok": False,
                "backend": backend,
                "error": "topic relay helper exited before its publisher appeared",
            }
        topics = run_ros2(["topic", "list"], timeout=10)
        if topics.get("ok") and destination_topic in topics.get("stdout", "").split():
            return {
                "ok": True,
                "backend": backend,
                "run_id": run_id,
                "source_topic": source_topic,
                "destination_topic": destination_topic,
                "message_type": message_type,
            }
        time.sleep(0.25)
    stop_topic_relay(run_id)
    return {
        "ok": False,
        "backend": backend,
        "error": (
            f"topic relay started, but {destination_topic} did not become "
            "discoverable within 15 seconds"
        ),
    }


def stop_topic_relay(run_id: str) -> dict[str, Any]:
    clean_run_id = str(run_id or "default").strip() or "default"
    return stop_ros2_managed(
        f"topic-relay:{clean_run_id}",
        pattern=f"ros2_topic_relay.py .*--run-id {clean_run_id}",
    )


def _free_port(host: str) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        return int(sock.getsockname()[1])


def _free_docker_stream_port(preferred: int = 0) -> tuple[int, str]:
    start, end = _stream_port_bounds()
    used = {
        int(item.get("port") or 0)
        for item in _streams.values()
        if item.get("backend") == "docker" and item.get("proc") is not None and item["proc"].poll() is None
    }
    if preferred > 0:
        if preferred < start or preferred > end:
            return 0, f"Docker CameraROS2Subscribe port must be within published range {start}-{end}; set port=0 to auto-pick"
        return preferred, "" if preferred not in used else f"port {preferred} is already in use by another CameraROS2Subscribe"
    for port in range(start, end + 1):
        if port not in used:
            return port, ""
    return 0, f"no free Docker CameraROS2Subscribe port in range {start}-{end}"


def _port_open(host: str, port: int, timeout: float = 0.15) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _stream_http_ready(host: str, port: int, timeout: float = 0.6) -> bool:
    """True once the MJPEG server actually answers HTTP on this port.

    A bare TCP connect is not enough on the Docker backend: ``docker run -p``
    publishes the port through a proxy that accepts connections immediately,
    long before the server inside the container is listening. Reporting the
    stream ready on TCP alone means the editor loads its <img> against a port
    that resets the connection, and a broken <img> is never retried -- the
    preview stays blank even though the stream comes up moments later.
    """
    try:
        with urllib.request.urlopen(
            f"http://{host}:{port}/health.json", timeout=timeout
        ) as response:
            return 200 <= int(getattr(response, "status", 200)) < 300
    except Exception:
        return False


def _terminate_process(proc: subprocess.Popen) -> bool:
    if proc.poll() is not None:
        return False
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            proc.kill()
    return True


def _stream_script() -> Path:
    return Path(__file__).resolve().parents[1] / "scripts" / "ros2_image_stream_server.py"


def probe_web_video(url: str, timeout: float = 10.0) -> tuple[bool, str]:
    """Check a robot's web_video_server MJPEG URL actually delivers a stream.

    Returns (ok, detail). Runs from the Blacknode process over plain HTTP, so
    it does not need a local ROS graph or the Docker helper container.
    """
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            status = int(getattr(response, "status", 200))
            if not 200 <= status < 300:
                return False, f"robot answered HTTP {status}"
            content_type = str(response.headers.get("Content-Type", ""))
            if "multipart" not in content_type.lower():
                return False, f"expected an MJPEG stream but got Content-Type '{content_type or 'unknown'}'"
            # web_video_server answers 200 for an unknown topic and then never
            # sends a frame, so require actual bytes before calling it live.
            if not response.read(64):
                return False, "connected but the robot sent no video data (is that topic publishing?)"
            return True, content_type
    except urllib.error.HTTPError as exc:
        return False, f"robot answered HTTP {exc.code}"
    except urllib.error.URLError as exc:
        return False, f"cannot reach the robot ({exc.reason})"
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"


def _host_camera_script() -> Path:
    return Path(__file__).resolve().parents[1] / "scripts" / "ros2_host_camera_publisher.py"


def container_reachable_url(url: str) -> str:
    """Rewrite a host-loopback URL so the Docker helper container can reach it.

    Inside the container ``127.0.0.1`` is the container itself, so a stream the
    host is serving on loopback is invisible. Docker Desktop publishes the host
    as ``host.docker.internal``.
    """
    for loopback in ("127.0.0.1", "localhost", "[::1]"):
        if f"//{loopback}:" in url or url.endswith(f"//{loopback}"):
            return url.replace(f"//{loopback}", "//host.docker.internal", 1)
    return url


def start_host_camera_publisher(
    *,
    run_id: str,
    source_url: str,
    topic: str,
    frame_id: str,
    max_fps: float,
) -> dict[str, Any]:
    """Bridge a host MJPEG camera stream onto a ROS 2 image topic."""
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    script = _host_camera_script()
    if not script.exists():
        return {"ok": False, "backend": backend, "error": f"host camera helper not found: {script}"}

    stop_ros2_managed(run_id, pattern="ros2_host_camera_publisher.py")

    if backend == "docker":
        err = ensure_container() or _ensure_container_stream_deps()
        if err:
            return {"ok": False, "backend": backend, "error": err}
        container_script = "/tmp/blacknode_ros2_host_camera_publisher.py"
        err = _copy_to_container(script, container_script)
        if err:
            return {"ok": False, "backend": backend, "error": err}
        reachable = container_reachable_url(source_url)
        helper_args = [
            "--source-url", reachable,
            "--topic", topic,
            "--frame-id", frame_id,
            "--max-fps", str(max_fps),
        ]
        shell = (
            "source /opt/ros/$ROS_DISTRO/setup.bash && "
            f"exec python3 {container_script} {shlex.join(helper_args)}"
        )
        started = _run(["docker", "exec", "-d", CONTAINER, "bash", "-lc", shell], 30)
        if started.returncode != 0:
            return {"ok": False, "backend": backend, "error": started.stderr.strip() or "docker exec failed"}
        return {"ok": True, "backend": backend, "source_url": reachable}

    args = [
        sys.executable, str(script),
        "--source-url", source_url,
        "--topic", topic,
        "--frame-id", frame_id,
        "--max-fps", str(max_fps),
    ]
    try:
        proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    except Exception as exc:
        return {"ok": False, "backend": backend, "error": f"{type(exc).__name__}: {exc}"}
    _managed_detached[run_id] = proc
    return {"ok": True, "backend": backend, "source_url": source_url}


def stop_host_camera_publisher(run_id: str) -> dict[str, Any]:
    return stop_ros2_managed(run_id, pattern="ros2_host_camera_publisher.py")


def _snapshot_script() -> Path:
    return Path(__file__).resolve().parents[1] / "scripts" / "ros2_image_snapshot.py"


def capture_image_snapshot(
    *,
    topic: str,
    message_type: str,
    timeout: float,
    output_format: str,
    jpeg_quality: int,
) -> dict[str, Any]:
    """Capture one image message through rclpy and return a data URL."""
    backend = detect_backend()["backend"]
    script = _snapshot_script()
    if not script.exists():
        return {"ok": False, "backend": backend, "error": f"snapshot helper not found: {script}"}
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}

    helper_args = [
        "--topic",
        topic,
        "--message-type",
        message_type,
        "--timeout",
        str(timeout),
        "--output-format",
        output_format,
        "--jpeg-quality",
        str(jpeg_quality),
    ]
    if backend == "docker":
        err = ensure_container() or _ensure_container_stream_deps() or _copy_to_container(script, _CONTAINER_SNAPSHOT_SCRIPT)
        if err:
            return {"ok": False, "backend": backend, "error": err}
        shell = (
            "source /opt/ros/$ROS_DISTRO/setup.bash && "
            f"timeout {max(1, int(timeout) + 5)}s python3 {_CONTAINER_SNAPSHOT_SCRIPT} {shlex.join(helper_args)}"
        )
        run_args = ["docker", "exec", CONTAINER, "bash", "-lc", shell]
    else:
        run_args = [
            sys.executable,
            str(script),
            *helper_args,
        ]
    try:
        proc = _run(run_args, max(1.0, float(timeout)) + 15.0)
    except subprocess.TimeoutExpired:
        return {"ok": False, "backend": backend, "error": f"snapshot helper timed out after {timeout:g}s"}
    try:
        payload = json.loads((proc.stdout or "").strip() or "{}")
    except Exception:
        payload = {"ok": False, "error": proc.stderr.strip() or "snapshot helper did not return JSON"}
    payload["backend"] = backend
    if proc.returncode != 0 and payload.get("ok") is not True:
        payload.setdefault("error", proc.stderr.strip() or f"snapshot helper exited with code {proc.returncode}")
    return payload


def start_image_stream(
    *,
    stream_id: str,
    topic: str,
    message_type: str,
    host: str,
    port: int,
    max_fps: float,
    max_width: int,
    jpeg_quality: int,
) -> dict[str, Any]:
    """Start a ROS image-topic MJPEG bridge. Returns URL/report data."""
    backend = detect_backend()["backend"]
    if backend == "docker":
        return _start_docker_image_stream(
            stream_id=stream_id,
            topic=topic,
            message_type=message_type,
            host=host,
            port=port,
            max_fps=max_fps,
            max_width=max_width,
            jpeg_quality=jpeg_quality,
        )
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    script = _stream_script()
    if not script.exists():
        return {"ok": False, "backend": backend, "error": f"stream helper not found: {script}"}

    existing = _streams.get(stream_id)
    if (
        existing
        and existing.get("proc") is not None
        and existing["proc"].poll() is None
        and existing.get("topic") == topic
        and existing.get("message_type") == message_type
    ):
        # Subscribing to a ROS topic happens once at process start (rclpy has
        # no cheap way to hot-swap it); when the topic/message_type this
        # subscriber cares about hasn't changed, reuse the running bridge
        # instead of tearing down and reconnecting the camera just because a
        # downstream node (e.g. a CUDA filter reading this stream) recooked.
        return {
            "ok": True,
            "backend": backend,
            "stream_id": stream_id,
            "stream_url": existing.get("url", ""),
            "snapshot_url": existing.get("snapshot_url", ""),
            "health_url": existing.get("health_url", ""),
        }

    stop_image_stream(stream_id)
    selected_port = int(port) if int(port) > 0 else _free_port(host)
    args = [
        sys.executable,
        str(script),
        "--topic",
        topic,
        "--message-type",
        message_type,
        "--host",
        host,
        "--port",
        str(selected_port),
        "--max-fps",
        str(max_fps),
        "--max-width",
        str(max_width),
        "--jpeg-quality",
        str(jpeg_quality),
    ]
    try:
        proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    except Exception as exc:
        return {"ok": False, "backend": backend, "error": f"{type(exc).__name__}: {exc}"}
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return {"ok": False, "backend": backend, "error": "stream helper exited before opening its HTTP port"}
        if _stream_http_ready(host, selected_port):
            break
        time.sleep(0.1)
    else:
        _terminate_process(proc)
        return {"ok": False, "backend": backend, "error": f"stream helper did not answer HTTP on http://{host}:{selected_port}"}
    url = f"http://{host}:{selected_port}/stream.mjpg"
    _streams[stream_id] = {
        "proc": proc,
        "url": url,
        "snapshot_url": f"http://{host}:{selected_port}/snapshot.jpg",
        "health_url": f"http://{host}:{selected_port}/health.json",
        "topic": topic,
        "message_type": message_type,
    }
    return {
        "ok": True,
        "backend": backend,
        "stream_id": stream_id,
        "stream_url": url,
        "snapshot_url": _streams[stream_id]["snapshot_url"],
        "health_url": _streams[stream_id]["health_url"],
        "port": selected_port,
    }


def _start_docker_image_stream(
    *,
    stream_id: str,
    topic: str,
    message_type: str,
    host: str,
    port: int,
    max_fps: float,
    max_width: int,
    jpeg_quality: int,
) -> dict[str, Any]:
    err = ensure_container() or _ensure_container_stream_deps() or _copy_to_container(_stream_script(), _CONTAINER_STREAM_SCRIPT)
    if err:
        return {"ok": False, "backend": "docker", "error": err}

    existing = _streams.get(stream_id)
    if (
        existing
        and existing.get("backend") == "docker"
        and existing.get("proc") is not None
        and existing["proc"].poll() is None
        and existing.get("topic") == topic
        and existing.get("message_type") == message_type
    ):
        return {
            "ok": True,
            "backend": "docker",
            "stream_id": stream_id,
            "stream_url": existing.get("url", ""),
            "snapshot_url": existing.get("snapshot_url", ""),
            "health_url": existing.get("health_url", ""),
        }

    stop_image_stream(stream_id)
    selected_port, port_error = _free_docker_stream_port(int(port) if int(port) > 0 else 0)
    if port_error:
        return {"ok": False, "backend": "docker", "error": port_error}

    helper_args = [
        "--topic",
        topic,
        "--message-type",
        message_type,
        "--host",
        "0.0.0.0",
        "--port",
        str(selected_port),
        "--max-fps",
        str(max_fps),
        "--max-width",
        str(max_width),
        "--jpeg-quality",
        str(jpeg_quality),
    ]
    marker = f"{_CONTAINER_STREAM_SCRIPT} --topic {topic}"
    shell = (
        "source /opt/ros/$ROS_DISTRO/setup.bash && "
        f"exec python3 {_CONTAINER_STREAM_SCRIPT} {shlex.join(helper_args)}"
    )
    try:
        proc = subprocess.Popen(
            ["docker", "exec", CONTAINER, "bash", "-lc", shell],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except Exception as exc:
        return {"ok": False, "backend": "docker", "error": f"{type(exc).__name__}: {exc}"}

    # Docker publishes the port through a proxy that accepts TCP immediately,
    # so wait for a real HTTP answer rather than a bare connect -- otherwise
    # the editor loads its <img> before the server is serving and shows a
    # permanently blank preview.
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return {"ok": False, "backend": "docker", "error": "stream helper exited before opening its HTTP port"}
        if _stream_http_ready("127.0.0.1", selected_port):
            break
        time.sleep(0.1)
    else:
        _terminate_process(proc)
        _run(["docker", "exec", CONTAINER, "pkill", "-f", marker], 15)
        return {"ok": False, "backend": "docker", "error": f"stream helper did not answer HTTP on http://127.0.0.1:{selected_port}"}

    public_host = "127.0.0.1" if host in {"", "0.0.0.0"} else host
    url = f"http://{public_host}:{selected_port}/stream.mjpg"
    _streams[stream_id] = {
        "backend": "docker",
        "proc": proc,
        "url": url,
        "snapshot_url": f"http://{public_host}:{selected_port}/snapshot.jpg",
        "health_url": f"http://{public_host}:{selected_port}/health.json",
        "topic": topic,
        "message_type": message_type,
        "marker": marker,
        "port": selected_port,
    }
    return {
        "ok": True,
        "backend": "docker",
        "stream_id": stream_id,
        "stream_url": url,
        "snapshot_url": _streams[stream_id]["snapshot_url"],
        "health_url": _streams[stream_id]["health_url"],
        "port": selected_port,
    }


def stop_image_stream(stream_id: str = "") -> dict[str, Any]:
    """Stop one image stream by id, or all streams when stream_id is empty."""
    ids = [stream_id] if stream_id else list(_streams)
    stopped = 0
    for sid in ids:
        item = _streams.pop(sid, None)
        if not item:
            continue
        killed = False
        if item.get("backend") == "docker" and item.get("marker"):
            result = _run(["docker", "exec", CONTAINER, "pkill", "-f", str(item["marker"])], 15)
            killed = result.returncode in (0, 1)
        if _terminate_process(item["proc"]) or killed:
            stopped += 1
    return {"ok": True, "backend": _passive_backend(), "stopped": stopped}
