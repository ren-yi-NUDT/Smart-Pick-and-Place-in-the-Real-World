"""Common TCP framing used by the robot services.

The canonical wire format is a 4-byte unsigned big-endian payload length
followed by exactly that many bytes. JSON services put UTF-8 JSON in the
payload; binary services can put any bytes there. ``JsonStreamReader`` keeps
servers compatible with the old raw-JSON clients during migration.
"""

import json
import socket
import struct


FRAME_HEADER = struct.Struct(">I")
MAX_FRAME_SIZE = 64 * 1024 * 1024
JSON_FRAME_PROTOCOL = "json-frame-v1"


def recv_exact(sock: socket.socket, size: int) -> bytes:
    """Read exactly *size* bytes or raise ``ConnectionError`` on EOF."""
    if size < 0:
        raise ValueError("size must be non-negative")
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("TCP peer closed the connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_frame(sock: socket.socket, payload: bytes) -> None:
    """Send one canonical length-prefixed frame."""
    if len(payload) > MAX_FRAME_SIZE:
        raise ValueError(f"frame exceeds {MAX_FRAME_SIZE} bytes")
    sock.sendall(FRAME_HEADER.pack(len(payload)))
    sock.sendall(payload)


def recv_frame(sock: socket.socket) -> bytes:
    """Receive one canonical length-prefixed frame."""
    size = FRAME_HEADER.unpack(recv_exact(sock, FRAME_HEADER.size))[0]
    if size > MAX_FRAME_SIZE:
        raise ValueError(f"frame exceeds {MAX_FRAME_SIZE} bytes")
    return recv_exact(sock, size)


def send_json_frame(sock: socket.socket, data: dict) -> None:
    send_frame(sock, json.dumps(data, ensure_ascii=False).encode("utf-8"))


def recv_json_frame(sock: socket.socket) -> dict:
    return json.loads(recv_frame(sock).decode("utf-8"))


def _decode_one_json(buffer: bytes):
    """Return ``(value, consumed_bytes)`` or ``None`` if JSON is incomplete."""
    try:
        text = buffer.decode("utf-8")
    except UnicodeDecodeError:
        return None
    decoder = json.JSONDecoder()
    try:
        value, end = decoder.raw_decode(text.lstrip())
    except json.JSONDecodeError:
        return None
    leading = len(text) - len(text.lstrip())
    return value, len(text[:leading + end].encode("utf-8"))


def recv_json_compat(sock: socket.socket) -> dict:
    """Receive a framed JSON response, accepting one legacy raw JSON reply.

    Raw JSON starts with ``{``/``[``/a quote, whose first four bytes cannot be
    mistaken for a reasonable frame length under ``MAX_FRAME_SIZE``. This
    makes the compatibility path deterministic without a second read-ahead
    protocol.
    """
    prefix = recv_exact(sock, FRAME_HEADER.size)
    size = FRAME_HEADER.unpack(prefix)[0]
    if size <= MAX_FRAME_SIZE:
        payload = recv_exact(sock, size)
        try:
            return json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            # A legacy raw response may begin with a byte sequence that looks
            # like a small integer. Fall through and decode it as raw JSON.
            prefix += payload

    buffer = prefix
    while True:
        decoded = _decode_one_json(buffer)
        if decoded is not None:
            value, _ = decoded
            return value
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("TCP peer closed before a JSON response")
        buffer += chunk


def send_json_compat(sock: socket.socket, data: dict, mode: str) -> None:
    """Reply using the peer's negotiated mode (``framed`` or ``raw``)."""
    if mode == "framed":
        send_json_frame(sock, data)
    elif mode == "raw":
        sock.sendall(json.dumps(data, ensure_ascii=False).encode("utf-8") + b"\n")
    else:
        raise ValueError(f"unknown TCP mode: {mode}")


class JsonStreamReader:
    """Read framed JSON and legacy raw JSON on a persistent connection.

    The first message selects the connection mode. Subsequent messages use
    that mode, while raw mode also preserves bytes belonging to the next JSON
    object if TCP coalesces writes.
    """

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.mode = None
        self._buffer = b""

    def read(self):
        if self.mode == "framed":
            return recv_json_frame(self.sock)
        if self.mode == "raw":
            return self._read_raw()

        prefix = recv_exact(self.sock, FRAME_HEADER.size)
        size = FRAME_HEADER.unpack(prefix)[0]
        if 0 < size <= MAX_FRAME_SIZE:
            payload = recv_exact(self.sock, size)
            try:
                value = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.mode = "raw"
                self._buffer = prefix + payload
                return self._read_raw()
            self.mode = "framed"
            return value

        self.mode = "raw"
        self._buffer = prefix
        return self._read_raw()

    def _read_raw(self):
        while True:
            decoded = _decode_one_json(self._buffer)
            if decoded is not None:
                value, consumed = decoded
                self._buffer = self._buffer[consumed:]
                return value
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("TCP peer closed before a JSON request")
            self._buffer += chunk
