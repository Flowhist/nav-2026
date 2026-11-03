"""Bounded, acknowledged WebSocket snapshots on the existing HTTP listener."""

import json
import math
import select
import socket
import struct
import threading
import time
import uuid
import zlib
from urllib.parse import urlsplit

from websockets.datastructures import Headers
from websockets.extensions.permessage_deflate import ServerPerMessageDeflateFactory
from websockets.frames import OP_BINARY, OP_CONT, OP_TEXT
from websockets.http11 import Request
from websockets.protocol import OPEN
from websockets.server import ServerProtocol

from web_support import RequestError


PROTOCOL = "finav.realtime.v1"
MAX_PACKET = 32 * 1024 * 1024
ACK_TIMEOUT = 10.0
TOPIC_LIMITS = {"scene": 15, "perception": 10, "status": 2}


def perception_snapshot(app):
    payload = app.state.perception.snapshot()
    payload["service"] = (app.runtime.perception_status() if hasattr(app.runtime, "perception_status")
                          else {"available": False, "running": False,
                                "error": "Observation requires user services"})
    return payload


class PacketEncoder:
    """Share the latest compressed map across clients, never retain old versions."""

    def __init__(self):
        self._lock = threading.Lock()
        self._map_source = None
        self._map_compressed = None
        self._points_source = None
        self._points_packed = b""

    def encode(self, envelope):
        attachments, blocks, offset = [], [], 0

        def attach(path, raw, encoding):
            nonlocal offset
            if len(raw) > MAX_PACKET:
                raise ValueError("Realtime attachment exceeds limit")
            decoded_size = len(raw)
            if encoding == "int8":
                with self._lock:
                    if raw is not self._map_source:
                        self._map_compressed = zlib.compress(raw, 1)
                        self._map_source = raw
                    raw = self._map_compressed
                compression = "deflate"
            else:
                compression = "none"
            attachments.append(dict(path=path, offset=offset, length=len(raw),
                                    decoded_length=decoded_size, encoding=encoding,
                                    compression=compression))
            blocks.append(raw)
            offset += len(raw)

        payload = dict(envelope)
        if "scene" in payload:
            scene = payload["scene"] = dict(payload["scene"])
            for key, field, encoding in (("map", "data_bytes", "int8"),
                                          ("scan", "ranges_bytes", "uint16-le")):
                source = scene.get(key)
                if source is not None and field in source:
                    item = scene[key] = dict(source)
                    raw = item.pop(field)
                    item["encoding"] = encoding
                    attach(["scene", key, "data" if key == "map" else "ranges"], raw, encoding)
        if "perception" in payload:
            sample = payload["perception"] = dict(payload["perception"])
            points = sample.pop("points")
            with self._lock:
                if points is not self._points_source:
                    self._points_packed = b"".join(struct.pack("<ff", x, y) for x, y in points)
                    self._points_source = points
                raw = self._points_packed
            attach(["perception", "points"], raw, "float32-xy-le")
        payload["attachments"] = attachments
        header = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode()
        packet = b"FNV1" + struct.pack("<I", len(header)) + header + b"".join(blocks)
        if len(packet) > MAX_PACKET:
            raise ValueError("Realtime packet exceeds limit")
        return packet


def validate_subscription(message):
    if not isinstance(message, dict) or message.get("type") != "subscribe":
        raise ValueError("Expected subscribe")
    topics = message.get("topics")
    if not isinstance(topics, dict) or not topics or set(topics) - TOPIC_LIMITS.keys():
        raise ValueError("Unknown or empty topics")
    rates = {}
    for topic, hz in topics.items():
        if isinstance(hz, bool) or not isinstance(hz, (float, int)) or not math.isfinite(hz) or hz <= 0:
            raise ValueError("Topic rate must be positive and finite")
        rates[topic] = min(float(hz), TOPIC_LIMITS[topic])
    mode = message.get("mode", "mapping")
    if mode not in ("mapping", "navigation"):
        raise ValueError("Invalid scene mode")
    if "scene" in rates and mode == "mapping":
        rates["scene"] = min(rates["scene"], 10)
    return rates, mode


class RealtimeSession:
    def __init__(self, app, encoder):
        self.app, self.encoder = app, encoder
        self.rates, self.due = {}, {}
        self.mode = None
        self.versions = (-1, -1, -1)
        self.seq = 0
        self.pending = None
        self.pending_at = 0
        self.last_ack_ms = None
        self._scan = None

    def receive(self, message):
        if not isinstance(message, dict):
            raise ValueError("Expected object")
        if message.get("type") == "ack":
            if type(message.get("seq")) is not int or message["seq"] != self.pending:
                raise ValueError("Unexpected acknowledgement")
            self.last_ack_ms = round((time.monotonic() - self.pending_at) * 1000, 1)
            self.pending = None
            return
        if self.rates:
            raise ValueError("Open a new connection to replace subscriptions")
        self.rates, self.mode = validate_subscription(message)
        self.due = dict.fromkeys(self.rates, 0.0)
        if "scene" in self.rates:
            self.app.state.add_live_viewer(self.mode, self.rates["scene"])

    def next_packet(self, now):
        if self.pending is not None:
            if now - self.pending_at > ACK_TIMEOUT:
                raise TimeoutError("Realtime acknowledgement timed out")
            return None
        ready = [topic for topic in self.rates if now >= self.due[topic]]
        if not ready:
            return None
        payload = dict(protocol=1, session_id=self.app.state.session_id,
                       seq=self.seq + 1, server_sent_at=time.time())
        for topic in ready:
            if topic == "scene":
                scene = self.app.state.snapshot_scene(*self.versions)
                self.versions = tuple(scene[key] for key in
                                      ("map_version", "planning_map_version", "plan_version"))
                scan = scene.get("scan")
                if scan is self._scan:
                    scene.pop("scan", None)
                self._scan = scan
                scene["stream"] = dict(mode=self.mode, target_hz=self.rates[topic],
                                       server_sent_at=payload["server_sent_at"])
                payload[topic] = scene
            elif topic == "perception":
                payload[topic] = perception_snapshot(self.app)
            else:
                payload[topic] = self.app.state.snapshot()
            # Never catch up on missed ticks: the next packet reads current state.
            self.due[topic] = now + 1.0 / self.rates[topic]
        packet = self.encoder.encode(payload)
        self.seq += 1
        self.pending, self.pending_at = self.seq, now
        return packet

    def close(self):
        if "scene" in self.rates:
            self.app.state.remove_live_viewer(self.rates["scene"])
        self.rates = {}


def serve_realtime(handler, app, encoder):
    """Authentication has already run in the HTTP handler's request wrapper."""
    hosts = handler.headers.get_all("Host", [])
    if len(hosts) != 1 or handler.request_version != "HTTP/1.1":
        raise RequestError(400, "A single Host and HTTP/1.1 are required")
    origin = handler.headers.get("Origin")
    if origin:
        parsed = urlsplit(origin)
        if parsed.scheme not in ("http", "https") or parsed.netloc.lower() != hosts[0].lower():
            raise RequestError(403, "Cross-origin realtime access is not allowed")
    if handler.headers.get("Transfer-Encoding") or handler.headers.get("Content-Length", "0") != "0":
        raise RequestError(400, "WebSocket upgrade must not have a request body")
    if not handler.server.stream_slots.acquire(blocking=False):
        raise RequestError(503, "Too many live streams")
    session = RealtimeSession(app, encoder)
    client_id = uuid.uuid4().hex
    protocol = ServerProtocol(max_size=8192, subprotocols=[PROTOCOL], extensions=[
        ServerPerMessageDeflateFactory(server_max_window_bits=12, client_max_window_bits=12,
                                      compress_settings={"level": 1, "memLevel": 5})])
    connection = handler.connection
    handler.close_connection = True

    def flush():
        sent = 0
        for chunk in protocol.data_to_send():
            if chunk:
                connection.sendall(chunk)
                sent += len(chunk)
        return sent

    try:
        request = Request(handler.path, Headers(handler.headers.items()))
        protocol.receive_data(request.serialize())
        requests = protocol.events_received()
        if not requests:
            flush()
            return
        response = protocol.accept(requests[0])
        protocol.send_response(response)
        flush()
        if response.status_code != 101:
            return
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        connection.settimeout(2.0)
        started = stats_at = time.monotonic()
        sent_bytes = sent_frames = 0
        fragments = bytearray()
        while app._running and protocol.state is OPEN:
            now = time.monotonic()
            if not session.rates and now - started > 5:
                raise TimeoutError("Subscription timed out")
            packet = session.next_packet(now)
            if packet is not None:
                protocol.send_binary(packet)
                sent_bytes += flush()
                sent_frames += 1
            if now - stats_at >= 1:
                app.state.update_realtime_client(client_id, {
                    "topics": session.rates, "bytes_per_sec": round(sent_bytes / (now - stats_at)),
                    "stream_hz": round(sent_frames / (now - stats_at), 1),
                    "ack_ms": session.last_ack_ms, "pending": session.pending is not None,
                })
                stats_at, sent_bytes, sent_frames = now, 0, 0
            wait = .25
            if session.rates and session.pending is None:
                wait = min(wait, max(0.0, min(session.due.values()) - time.monotonic()))
            readable, _, _ = select.select([connection], [], [], wait)
            if not readable:
                continue
            chunk = connection.recv(8192)
            if not chunk:
                break
            protocol.receive_data(chunk)
            events = protocol.events_received()
            flush()
            if protocol.state is not OPEN:
                break
            for event in events:
                if event.opcode is OP_BINARY:
                    raise ValueError("Client messages must be JSON text")
                if event.opcode in (OP_TEXT, OP_CONT):
                    fragments.extend(event.data)
                    if len(fragments) > 8192:
                        raise ValueError("Client message exceeds limit")
                    if event.fin:
                        session.receive(json.loads(fragments.decode("utf-8")))
                        fragments.clear()
    except (ValueError, TypeError, KeyError, UnicodeError, OverflowError):
        if protocol.state is OPEN:
            protocol.send_close(1008, "Invalid realtime message or payload")
            try:
                flush()
            except OSError:
                pass
    except (OSError, TimeoutError):
        pass
    finally:
        session.close()
        app.state.update_realtime_client(client_id, None)
        handler.server.stream_slots.release()
