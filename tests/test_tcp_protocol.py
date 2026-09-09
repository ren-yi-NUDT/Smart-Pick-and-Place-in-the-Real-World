import json
import socket

from core.tcp_protocol import (
    JsonStreamReader,
    recv_frame,
    recv_json_compat,
    send_frame,
    send_json_compat,
    send_json_frame,
)


def test_json_frame_round_trip_handles_coalesced_frames():
    left, right = socket.socketpair()
    try:
        send_json_frame(left, {"id": 1})
        send_json_frame(left, {"id": 2})
        reader = JsonStreamReader(right)
        assert reader.read() == {"id": 1}
        assert reader.mode == "framed"
        assert reader.read() == {"id": 2}
    finally:
        left.close()
        right.close()


def test_json_stream_reader_accepts_legacy_raw_json_stream():
    left, right = socket.socketpair()
    try:
        left.sendall(
            json.dumps({"id": 1}).encode("utf-8")
            + json.dumps({"id": 2}).encode("utf-8")
        )
        reader = JsonStreamReader(right)
        assert reader.read() == {"id": 1}
        assert reader.mode == "raw"
        assert reader.read() == {"id": 2}
    finally:
        left.close()
        right.close()


def test_response_reader_accepts_framed_and_raw_json():
    for mode in ("framed", "raw"):
        left, right = socket.socketpair()
        try:
            send_json_compat(left, {"value": True}, mode)
            assert recv_json_compat(right) == {"value": True}
        finally:
            left.close()
            right.close()


def test_binary_frame_preserves_arbitrary_payload():
    left, right = socket.socketpair()
    try:
        payload = b"header\n\x00\xffimage-bytes"
        send_frame(left, payload)
        assert recv_frame(right) == payload
    finally:
        left.close()
        right.close()
