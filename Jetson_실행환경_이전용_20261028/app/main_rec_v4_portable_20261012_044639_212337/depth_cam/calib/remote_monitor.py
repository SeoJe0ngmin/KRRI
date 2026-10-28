"""Optional loopback viewer. Network clients never own the vehicle lifetime."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue
import secrets
import threading
import time
from urllib.parse import urlsplit

import cv2


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    request_queue_size = 8

    def __init__(self, address, handler):
        self.slots = threading.BoundedSemaphore(8)
        super().__init__(address, handler)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class RemoteMonitor:
    """A bounded JPEG encoder and localhost HTTP server with explicit controls.

    The caller publishes rendered frames and polls ``read_key``. Publishing,
    client disconnects and encoding failures never raise into vehicle control.
    Binding errors from ``start`` raise before the caller starts the vehicle.
    """

    def __init__(self, port=8768, fps=5.0, *, demo_mode=False):
        self.port = int(port)
        if not 0 <= self.port <= 65535 or not 0 < fps <= 20:
            raise ValueError("Invalid monitor port or FPS")
        self.period = 1.0 / fps
        self.demo_mode = bool(demo_mode)
        self.token = secrets.token_urlsafe(32)
        self._frames = queue.Queue(maxsize=1)
        self._keys = queue.Queue(maxsize=16)
        self._stop_key = threading.Event()
        self._closing = threading.Event()
        self._lock = threading.Lock()
        self._jpeg = None
        self._next_publish = 0.0
        self._server = self._http_thread = self._encoder = None
        self._stats = dict(published=0, encoded=0, dropped=0, encode_errors=0,
                           last_error=None)

    @property
    def url(self):
        return f"http://127.0.0.1:{self.port}/"

    @property
    def telemetry(self):
        with self._lock:
            return dict(self._stats, has_frame=self._jpeg is not None,
                        demo_mode=self.demo_mode)

    @staticmethod
    def _valid_host(host):
        if not host or any(c in host for c in "\\/@#? \t\r\n"):
            return False
        try:
            parsed = urlsplit("http://" + host)
            return (parsed.hostname in {"127.0.0.1", "localhost"}
                    and parsed.port is not None and 0 < parsed.port <= 65535
                    and not parsed.path)
        except ValueError:
            return False

    def start(self):
        if self._server is not None:
            return self
        if self._closing.is_set():
            raise RuntimeError("Closed monitor cannot restart")
        monitor = self

        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(2.0)

            def log_message(self, *args):
                pass

            def _reply(self, code, body=b"", content_type="text/plain; charset=utf-8"):
                try:
                    self.send_response(code)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.send_header("X-Frame-Options", "DENY")
                    self.send_header("Referrer-Policy", "no-referrer")
                    self.end_headers()
                    self.wfile.write(body)
                except (OSError, TimeoutError):
                    # Closing a viewer or a failed tunnel is not a stop request.
                    pass

            def _host_allowed(self):
                if not monitor._valid_host(self.headers.get("Host")):
                    self._reply(403)
                    return False
                return True

            def do_GET(self):
                if not self._host_allowed():
                    return
                path = urlsplit(self.path).path
                if path == "/session":
                    self._reply(200, json.dumps(dict(token=monitor.token,
                        demo_mode=monitor.demo_mode)).encode(), "application/json")
                elif path == "/frame.jpg":
                    with monitor._lock:
                        jpeg = monitor._jpeg
                    self._reply(200 if jpeg is not None else 503,
                                jpeg or b"Waiting for a frame", "image/jpeg")
                elif path == "/status":
                    self._reply(200, json.dumps(monitor.telemetry).encode(),
                                "application/json")
                else:
                    self._reply(404)

            def do_POST(self):
                if not self._host_allowed():
                    return
                origin = self.headers.get("Origin")
                if origin is not None and origin != "http://" + self.headers["Host"]:
                    self._reply(403)
                    return
                supplied = self.headers.get("X-Lifter-Token", "")
                if not hmac.compare_digest(supplied.encode("utf-8"),
                                           monitor.token.encode("ascii")):
                    self._reply(403)
                    return
                if self.path != "/key":
                    self._reply(404)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 1024 or self.headers.get("Transfer-Encoding"):
                        raise ValueError("Invalid body length")
                    if self.headers.get_content_type() != "application/json":
                        raise ValueError("JSON required")
                    key = json.loads(self.rfile.read(length)).get("key")
                    code = {"stop": 27, "step": 32, "record": ord("r")}[key]
                    if monitor.demo_mode and key != "stop":
                        self._reply(409)
                        return
                except (ValueError, KeyError, TypeError, AttributeError, OSError):
                    self._reply(400)
                    return
                if code == 27:
                    monitor._stop_key.set()
                else:
                    try:
                        monitor._keys.put_nowait(code)
                    except queue.Full:
                        self._reply(429)
                        return
                self._reply(204)

        self._server = _Server(("127.0.0.1", self.port), Handler)
        self.port = self._server.server_address[1]
        self._encoder = threading.Thread(target=self._encode, name="MonitorJPEG", daemon=True)
        self._http_thread = threading.Thread(
            target=self._server.serve_forever, kwargs={"poll_interval": 0.1},
            name="MonitorHTTP", daemon=True,
        )
        try:
            self._encoder.start()
            self._http_thread.start()
        except BaseException:
            self.close()
            raise
        return self

    def publish(self, image):
        try:
            now = time.monotonic()
            if self._closing.is_set() or now < self._next_publish:
                return False
            self._next_publish = now + self.period
            frame = image.copy()
            try:
                self._frames.put_nowait(frame)
            except queue.Full:
                try:
                    self._frames.get_nowait()
                except queue.Empty:
                    pass
                self._frames.put_nowait(frame)
                with self._lock:
                    self._stats["dropped"] += 1
            with self._lock:
                self._stats["published"] += 1
            return True
        except Exception as exc:
            self._encoder_error(exc)
            return False

    def _encoder_error(self, exc):
        with self._lock:
            self._stats["encode_errors"] += 1
            self._stats["last_error"] = f"{type(exc).__name__}: {exc}"

    def _encode(self):
        while not self._closing.is_set():
            try:
                image = self._frames.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                ok, encoded = cv2.imencode(".jpg", image,
                                           [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                jpeg = encoded.tobytes()
                with self._lock:
                    self._jpeg = jpeg
                    self._stats["encoded"] += 1
            except Exception as exc:
                self._encoder_error(exc)

    def read_key(self):
        if self._stop_key.is_set():
            self._stop_key.clear()
            return 27
        try:
            return self._keys.get_nowait()
        except queue.Empty:
            return -1

    def close(self):
        self._closing.set()
        if self._server is not None:
            if self._http_thread is not None and self._http_thread.is_alive():
                self._server.shutdown()
            self._server.server_close()
        for thread in (self._http_thread, self._encoder):
            if thread is not None and thread.ident is not None:
                thread.join(timeout=1.0)
        self._server = None
