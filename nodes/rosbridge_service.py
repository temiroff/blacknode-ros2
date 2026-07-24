"""Manage a local rosbridge container for one-click workflows."""
from __future__ import annotations

import os
import platform
import shutil
import socket
import subprocess
import time
from pathlib import Path

from blacknode.node import Bool, Enum, Float, Int, Text

from ._implementation import implementation_node as node

_IMAGE = "blacknode-rosbridge:jazzy"
_CONTAINER = "blacknode-rosbridge"
_DOCKERFILE = """FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-rosbridge-server && rm -rf /var/lib/apt/lists/*
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090"]
"""


def _port_open(host: str, port: int, timeout: float = 0.4) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _run(command: list[str], timeout: float, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False, **kwargs)


def _docker_ready() -> bool:
    try:
        return _run(["docker", "info"], 8).returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _start_docker_desktop(timeout: float) -> None:
    """Ensure Docker is ready, launching Docker Desktop where supported.

    The historical name is retained because extension tests and callers may
    patch this helper.
    """
    if _docker_ready():
        return
    system = platform.system()
    if system == "Linux":
        if shutil.which("docker") is None:
            raise RuntimeError(
                "Docker Engine is not installed. In blacknode-runtime run "
                "./service.sh docker (or ./setup-docker.sh), then restart the runtime."
            )
        raise RuntimeError(
            "Docker Engine is installed but the daemon or socket is unavailable. "
            "In blacknode-runtime run ./service.sh docker, then restart the runtime."
        )
    if system != "Windows":
        raise RuntimeError(
            "Docker is unavailable. Start a Docker-compatible daemon and confirm "
            "'docker info' works for the Blacknode Runtime user."
        )
    candidates = [
        Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "Docker/Docker/Docker Desktop.exe",
        Path(os.environ.get("LOCALAPPDATA", "")) / "Docker/Docker Desktop.exe",
    ]
    executable = next((path for path in candidates if path.is_file()), None)
    if executable is None:
        raise RuntimeError("Docker is unavailable. Install Docker Desktop; the template will start it automatically after that.")
    subprocess.Popen([str(executable)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + max(10.0, timeout)
    while time.monotonic() < deadline:
        if _docker_ready():
            return
        time.sleep(2.0)
    raise RuntimeError("Docker Desktop did not become ready before the timeout")


def _container_name(port: int) -> str:
    return _CONTAINER if port == 9090 else f"{_CONTAINER}-{port}"


def ensure_local_rosbridge(
    host: str,
    port: int,
    timeout: float,
    *,
    expose_lan: bool = False,
) -> str:
    if host.lower() not in {"127.0.0.1", "localhost", "::1"}:
        raise RuntimeError("automatic rosbridge management only supports localhost")
    if _port_open(host, port):
        return f"rosbridge ready at ws://{host}:{port} (already running)"

    _start_docker_desktop(timeout)
    container = _container_name(port)
    publish_host = "0.0.0.0" if expose_lan else "127.0.0.1"
    image = _run(["docker", "image", "inspect", _IMAGE], 15)
    if image.returncode != 0:
        build_timeout = max(600.0, timeout)
        try:
            built = _run(
                ["docker", "build", "-t", _IMAGE, "-"],
                build_timeout,
                input=_DOCKERFILE,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(
                f"rosbridge image build exceeded {build_timeout:.0f} seconds; "
                "check the device network and Docker logs, then retry"
            ) from exc
        if built.returncode != 0:
            raise RuntimeError(f"could not build rosbridge image: {(built.stderr or built.stdout).strip()}")

    exists = _run(["docker", "container", "inspect", container], 15).returncode == 0
    if exists:
        started = _run(["docker", "start", container], 30)
    else:
        started = _run(
            [
                "docker", "run", "-d", "--name", container,
                "--restart", "unless-stopped",
                "-p", f"{publish_host}:{port}:9090",
                _IMAGE,
            ],
            45,
        )
    if started.returncode != 0:
        raise RuntimeError(f"could not start rosbridge container: {(started.stderr or started.stdout).strip()}")

    deadline = time.monotonic() + max(10.0, timeout)
    while time.monotonic() < deadline:
        if _port_open(host, port):
            visibility = "LAN-exposed" if expose_lan else "local-only"
            return (
                f"rosbridge ready at ws://{host}:{port} "
                f"(Docker container {container}, {visibility})"
            )
        time.sleep(0.5)
    logs = _run(["docker", "logs", "--tail", "20", container], 15)
    raise RuntimeError(f"rosbridge did not open port {port}: {(logs.stderr or logs.stdout).strip()}")


@node(
    name="ROS2RosbridgeServer", component="rosbridge",
    category="ROS 2",
    hidden=True,
    description="Ensure a local rosbridge Docker service is running for one-click workflows.",
    inputs={
        "action": Enum(["ensure", "check", "stop"], default="ensure"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "expose_lan": Bool(default=False),
        "timeout": Float(default=180.0),
    },
    outputs={"ready": Bool, "report": Text},
)
def ros2_rosbridge_server(ctx: dict) -> dict:
    action = str(ctx.get("action") or "ensure")
    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)
    expose_lan = bool(ctx.get("expose_lan", False))
    timeout = float(ctx.get("timeout") or 180.0)
    if action == "check":
        ready = _port_open(host, port)
        return {"ready": ready, "report": f"rosbridge {'ready' if ready else 'not reachable'} at ws://{host}:{port}"}
    if action == "stop":
        try:
            result = _run(["docker", "stop", _container_name(port)], 30)
            ok = result.returncode == 0
            return {"ready": False, "report": "rosbridge stopped" if ok else (result.stderr or result.stdout).strip()}
        except FileNotFoundError:
            return {"ready": False, "report": "Docker CLI not found"}
    try:
        report = ensure_local_rosbridge(
            host,
            port,
            timeout,
            expose_lan=expose_lan,
        )
        return {"ready": True, "report": report}
    except Exception as exc:  # noqa: BLE001 - node reports actionable runtime failures
        return {"ready": False, "report": f"rosbridge startup FAILED: {type(exc).__name__}: {exc}"}
