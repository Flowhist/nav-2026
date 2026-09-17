"""HTTP limits, request validation and revisioned configuration persistence."""

import functools
import base64
import binascii
import hashlib
import math
import os
import secrets
import tempfile
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path

import yaml


class RequestError(ValueError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def access_password():
    if os.environ.get("FINAV_WEB_AUTH") != "1":
        return None
    password = os.environ.get("FINAV_WEB_PASSWORD")
    if password:
        return password
    directory = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "finav"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "web-access.key"
    try:
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError:
        password = path.read_text(encoding="utf-8").strip()
    else:
        password = secrets.token_urlsafe(24)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(password + "\n")
    if not password:
        raise RuntimeError("Web access password is empty")
    return password


def authorized(header, password):
    if password is None:
        return True
    try:
        kind, encoded = (header or "").split(" ", 1)
        if kind.lower() != "basic":
            return False
        supplied = base64.b64decode(encoded, validate=True)
        return secrets.compare_digest(supplied, ("finav:" + password).encode("utf-8"))
    except (ValueError, binascii.Error):
        return False


def request_errors(method):
    @functools.wraps(method)
    def wrapped(self):
        try:
            if not self._authorize():
                return
            method(self)
        except RequestError as exc:
            self.close_connection = True
            self._json(exc.status, {"ok": False, "error": str(exc)})
        except (ValueError, TypeError, KeyError, UnicodeError) as exc:
            self.close_connection = True
            self._json(400, {"ok": False, "error": str(exc)})
        except (OSError, RuntimeError) as exc:
            self.close_connection = True
            self._json(503, {"ok": False, "error": str(exc)})
    return wrapped


def finite_fields(body, *names):
    result = {}
    for name in names:
        value = body.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RequestError(400, f"{name} must be a number")
        try:
            value = float(value)
        except OverflowError as exc:
            raise RequestError(400, f"{name} is out of range") from exc
        if not math.isfinite(value):
            raise RequestError(400, f"{name} must be finite")
        result[name] = value
    return result


def query_integer(query, name, default, minimum, maximum):
    value = int(query.get(name, [default])[0])
    if not minimum <= value <= maximum:
        raise RequestError(400, f"{name} is out of range")
    return value


class LimitedHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, handler, max_connections=32):
        self._slots = threading.BoundedSemaphore(max_connections)
        self.stream_slots = threading.BoundedSemaphore(8)
        self._clients = set()
        self._clients_lock = threading.Lock()
        super().__init__(address, handler)

    def process_request(self, request, client_address):
        if not self._slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        request.settimeout(10.0)
        with self._clients_lock:
            self._clients.add(request)
        try:
            super().process_request(request, client_address)
        except Exception:
            with self._clients_lock:
                self._clients.discard(request)
            self._slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            with self._clients_lock:
                self._clients.discard(request)
            self._slots.release()

    def close_clients(self):
        with self._clients_lock:
            clients = list(self._clients)
        for client in clients:
            self.shutdown_request(client)


def content_revision(content):
    return hashlib.sha256(content.encode("utf-8")).hexdigest()


def atomic_write(path, content):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        if path.exists():
            os.fchmod(fd, path.stat().st_mode & 0o777)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class ConfigStore:
    def __init__(self):
        self._lock = threading.Lock()

    def read(self, path):
        with self._lock:
            content = path.read_text(encoding="utf-8")
            return {"content": content, "revision": content_revision(content)}

    def save(self, path, body):
        content = body.get("content")
        revision = body.get("expected_revision")
        if not isinstance(content, str) or not isinstance(revision, str):
            raise RequestError(400, "content and expected_revision are required")
        try:
            document = yaml.safe_load(content)
        except yaml.YAMLError as exc:
            raise RequestError(422, f"Invalid YAML: {exc}") from exc
        if not isinstance(document, dict) or not document:
            raise RequestError(422, "Configuration must be a non-empty mapping")
        for value in document.values():
            if isinstance(value, dict) and "ros__parameters" in value:
                if not isinstance(value["ros__parameters"], dict):
                    raise RequestError(422, "ros__parameters must be a mapping")
        with self._lock:
            if content_revision(path.read_text(encoding="utf-8")) != revision:
                raise RequestError(409, "Configuration changed; reload before saving")
            atomic_write(path, content)
        return content_revision(content)
