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
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from blacknode import console

IMAGE = os.environ.get("BLACKNODE_ROS2_IMAGE", "ros:jazzy")
CONTAINER = os.environ.get("BLACKNODE_ROS2_CONTAINER", "blacknode-ros2")
STREAM_PORT_RANGE = os.environ.get("BLACKNODE_ROS2_STREAM_PORT_RANGE", "39000-39049")
_CONTAINER_STREAM_SCRIPT = "/tmp/blacknode_ros2_image_stream_server.py"
_CONTAINER_SNAPSHOT_SCRIPT = "/tmp/blacknode_ros2_image_snapshot.py"

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
    try:
        from blacknode.pkg.blacknode_skills.follow_person.follow_runtime import continuous_follow_runtime_status
        from blacknode.pkg.blacknode_skills.follow_person.leader_follower_runtime import leader_follower_runtime_status
        continuous_follows = continuous_follow_runtime_status()
        leader_followers = leader_follower_runtime_status()
    except Exception:
        continuous_follows = []
        leader_followers = []
    try:
        from blacknode.pkg.blacknode_controllers.policy.policy_runtime import (
            runtime_status as policy_runtime_status,
        )
        policy_runs = policy_runtime_status()
    except Exception:
        policy_runs = []
    return {
        "ok": True,
        "backend": detect_backend()["backend"],
        "streams": live_streams,
        "managed_runs": live_runs,
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
        from blacknode.pkg.blacknode_controllers.policy.policy_runtime import stop_policy_services
        policy_result = stop_policy_services()
    except ModuleNotFoundError:
        policy_result = {"ok": True, "stopped": 0, "error": ""}
    except Exception as exc:
        policy_result = {"ok": False, "stopped": 0, "error": str(exc)}

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
        "backend": detect_backend()["backend"],
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
        # `docker exec -d` only confirms the shell was dispatched, not that
        # `ros2 run <package> <executable>` actually started inside it -- a
        # missing package fails silently there, producing a false "started"
        # report followed by a confusing downstream timeout. Verify (and, if
        # missing, install) the package first so a missing package either
        # gets fixed automatically or fails immediately with an actionable
        # message, instead of surfacing as an opaque "no active publisher"
        # from whatever's downstream of this run.
        if len(args) >= 2 and args[0] == "run":
            package = args[1]
            package_error = _ensure_container_package(package)
            if package_error:
                return {"ok": False, "backend": backend, "error": package_error}
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
    return {"ok": True, "backend": detect_backend()["backend"], "stopped": stopped}
