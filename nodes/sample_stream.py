"""Local HTTP transport for latest-value robot samples."""
from __future__ import annotations

import json
import threading
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import quote, unquote, urlparse

_lock = threading.RLock()
_providers: dict[str, Callable[[], dict[str, Any]]] = {}
_server: ThreadingHTTPServer | None = None
_thread: threading.Thread | None = None


def _handler_type():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: Any) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            prefix = "/samples/"
            if not path.startswith(prefix) or not path.endswith(".json"):
                self.send_error(404, "sample stream not found")
                return
            stream_id = unquote(path[len(prefix) : -len(".json")])
            with _lock:
                provider = _providers.get(stream_id)
            if provider is None:
                self.send_error(404, "sample stream is not active")
                return
            try:
                body = json.dumps(provider(), separators=(",", ":"), allow_nan=False).encode("utf-8")
            except Exception as exc:  # noqa: BLE001
                self.send_error(503, f"sample unavailable: {exc}")
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def _ensure_server() -> ThreadingHTTPServer:
    global _server, _thread
    with _lock:
        if _server is not None:
            return _server
        server = ThreadingHTTPServer(("127.0.0.1", 0), _handler_type())
        thread = threading.Thread(
            target=server.serve_forever,
            kwargs={"poll_interval": 0.2},
            name="blacknode-ros2-sample-stream",
            daemon=True,
        )
        _server = server
        _thread = thread
        thread.start()
        return server


def register(stream_id: str, provider: Callable[[], dict[str, Any]]) -> dict[str, Any]:
    normalized = str(stream_id or "").strip()
    if not normalized:
        raise ValueError("stream_id is required")
    server = _ensure_server()
    with _lock:
        _providers[normalized] = provider
    host, port = server.server_address
    return {
        "kind": "blacknode.sample-stream",
        "schema_version": 1,
        "stream_id": normalized,
        "url": f"http://{host}:{port}/samples/{quote(normalized, safe='')}.json",
        "media_type": "application/json",
        "mode": "latest",
        "clock": "unix_ns",
    }


def unregister(stream_id: str) -> None:
    with _lock:
        _providers.pop(str(stream_id or "").strip(), None)


def snapshot(stream_id: str) -> dict[str, Any]:
    with _lock:
        provider = _providers.get(str(stream_id or "").strip())
    return dict(provider() if provider is not None else {})
