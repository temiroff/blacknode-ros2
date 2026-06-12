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
import shlex
import shutil
import subprocess
from typing import Any

IMAGE = os.environ.get("BLACKNODE_ROS2_IMAGE", "ros:jazzy")
CONTAINER = os.environ.get("BLACKNODE_ROS2_CONTAINER", "blacknode-ros2")

_NO_BACKEND_HELP = (
    "ROS 2 is not available: no `ros2` on PATH and no running Docker daemon. "
    "Either install ROS 2 natively, or start Docker and pull the image with "
    f"`docker pull {IMAGE}` (blacknode packages setup blacknode-ros2 does this)."
)

_cached_backend: dict[str, str] | None = None
_detached: list[subprocess.Popen] = []


def detect_backend(refresh: bool = False) -> dict[str, str]:
    """{"backend": "native"|"docker"|"none", "detail": ...} — cached after first call."""
    global _cached_backend
    if _cached_backend is not None and not refresh:
        return _cached_backend
    native = shutil.which("ros2")
    if native:
        distro = os.environ.get("ROS_DISTRO", "")
        _cached_backend = {"backend": "native", "detail": f"{native}" + (f" ({distro})" if distro else "")}
    elif shutil.which("docker") and _docker_ok():
        _cached_backend = {"backend": "docker", "detail": f"image {IMAGE}, container {CONTAINER}"}
    else:
        _cached_backend = {"backend": "none", "detail": _NO_BACKEND_HELP}
    return _cached_backend


def _docker_ok() -> bool:
    try:
        return _run(["docker", "version", "--format", "{{.Server.Version}}"], 10).returncode == 0
    except Exception:
        return False


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
    if check.stdout.strip():
        return None
    _run(["docker", "rm", "-f", CONTAINER], 30)  # clear a stopped leftover
    start = _run(["docker", "run", "-d", "--name", CONTAINER, IMAGE, "sleep", "infinity"], 180)
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


def run_ros2(args: list[str], timeout: float = 15.0) -> dict[str, Any]:
    """Run ``ros2 <args>``; returns {ok, stdout, stderr, backend, error?, timed_out?}."""
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "stdout": "", "stderr": "", "backend": backend, "error": _NO_BACKEND_HELP}
    try:
        if backend == "native":
            proc = _run(["ros2", *args], timeout)
            timed_out = False
        else:
            err = ensure_container()
            if err:
                return {"ok": False, "stdout": "", "stderr": err, "backend": backend, "error": err}
            proc = _run(
                ["docker", "exec", CONTAINER, "bash", "-lc", _container_shell(args, timeout)],
                timeout + 15,
            )
            timed_out = proc.returncode == 124  # GNU timeout exit code
    except subprocess.TimeoutExpired:
        return {
            "ok": False, "stdout": "", "stderr": "", "backend": backend,
            "error": f"`ros2 {' '.join(args)}` timed out after {timeout:g}s", "timed_out": True,
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
    return result


def run_ros2_detached(args: list[str]) -> dict[str, Any]:
    """Start ``ros2 <args>`` in the background (e.g. a demo publisher)."""
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    try:
        if backend == "native":
            proc = subprocess.Popen(
                ["ros2", *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            _detached.append(proc)
        else:
            err = ensure_container()
            if err:
                return {"ok": False, "backend": backend, "error": err}
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
            if proc.poll() is None:
                proc.terminate()
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
