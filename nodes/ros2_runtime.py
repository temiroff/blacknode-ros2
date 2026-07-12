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
import signal
import shlex
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
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
_managed_detached: dict[str, subprocess.Popen] = {}
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

    live_detached = [proc for proc in _detached if proc.poll() is None]
    _detached[:] = live_detached
    return {
        "ok": True,
        "backend": detect_backend()["backend"],
        "streams": live_streams,
        "managed_runs": live_runs,
        "detached_count": len(live_detached),
        "active": bool(live_streams or live_runs or live_detached),
    }


def stop_runtime_services() -> dict[str, Any]:
    """Stop all ROS helpers this Blacknode process started for live workflows."""
    status_before = runtime_status()
    stream_result = stop_image_stream("")

    managed_stopped = 0
    managed_errors: list[str] = []
    for run_id in list(_managed_detached):
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
    stopped = {
        "streams": int(stream_result.get("stopped") or 0),
        "managed_runs": managed_stopped,
        "detached": detached_stopped,
    }
    return {
        "ok": not errors,
        "backend": detect_backend()["backend"],
        "active_before": status_before,
        "stopped": stopped,
        "errors": errors,
        "report": (
            f"stopped {stopped['streams']} stream(s), "
            f"{stopped['managed_runs']} ROS 2 run process(es), "
            f"{stopped['detached']} detached ROS 2 process(es)"
        ),
    }


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
                ["ros2", *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
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
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                except Exception:
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


def run_ros2_managed(key: str, args: list[str]) -> dict[str, Any]:
    """Start one named background ``ros2 <args>`` process, replacing any old one."""
    stop_ros2_managed(key, pattern=f"ros2 {shlex.join(args)}")
    backend = detect_backend()["backend"]
    if backend == "none":
        return {"ok": False, "backend": backend, "error": _NO_BACKEND_HELP}
    try:
        if backend == "native":
            proc = subprocess.Popen(
                ["ros2", *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
            )
            _managed_detached[key] = proc
            return {"ok": True, "backend": backend, "pid": proc.pid}
        err = ensure_container()
        if err:
            return {"ok": False, "backend": backend, "error": err}
        shell = f"source /opt/ros/$ROS_DISTRO/setup.bash && exec ros2 {shlex.join(args)}"
        proc = _run(["docker", "exec", "-d", CONTAINER, "bash", "-lc", shell], 30)
        if proc.returncode != 0:
            return {"ok": False, "backend": backend, "error": proc.stderr.strip() or "docker exec failed"}
        return {"ok": True, "backend": backend}
    except Exception as exc:
        return {"ok": False, "backend": backend, "error": str(exc)}


def stop_ros2_managed(key: str, pattern: str = "") -> dict[str, Any]:
    """Stop one named background process."""
    backend = detect_backend()["backend"]
    stopped = 0
    proc = _managed_detached.pop(key, None)
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


def _free_port(host: str) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        return int(sock.getsockname()[1])


def _port_open(host: str, port: int, timeout: float = 0.15) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
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
    """Capture one image message through native rclpy and return a data URL."""
    backend = detect_backend()["backend"]
    if backend != "native":
        return {
            "ok": False,
            "backend": backend,
            "error": "image snapshots require native ROS 2 with rclpy; start Blacknode from a sourced ROS shell.",
        }
    script = _snapshot_script()
    if not script.exists():
        return {"ok": False, "backend": backend, "error": f"snapshot helper not found: {script}"}
    args = [
        sys.executable,
        str(script),
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
    try:
        proc = _run(args, max(1.0, float(timeout)) + 5.0)
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
    """Start a native ROS image-topic MJPEG bridge. Returns URL/report data."""
    backend = detect_backend()["backend"]
    if backend != "native":
        return {
            "ok": False,
            "backend": backend,
            "error": "ROS2ImageStream requires native ROS 2 with rclpy; start Blacknode from a sourced ROS shell.",
        }
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
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return {"ok": False, "backend": backend, "error": "stream helper exited before opening its HTTP port"}
        if _port_open(host, selected_port):
            break
        time.sleep(0.05)
    else:
        _terminate_process(proc)
        return {"ok": False, "backend": backend, "error": f"stream helper did not open http://{host}:{selected_port}"}
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


def stop_image_stream(stream_id: str = "") -> dict[str, Any]:
    """Stop one image stream by id, or all streams when stream_id is empty."""
    ids = [stream_id] if stream_id else list(_streams)
    stopped = 0
    for sid in ids:
        item = _streams.pop(sid, None)
        if not item:
            continue
        if _terminate_process(item["proc"]):
            stopped += 1
    return {"ok": True, "backend": detect_backend()["backend"], "stopped": stopped}
