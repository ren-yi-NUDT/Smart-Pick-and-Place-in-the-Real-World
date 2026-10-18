#!/usr/bin/env python3
"""Azure capture, detection, recording, and read-only local HTTP service.

The process owns the only AzureCamera instance. Consumers fetch a JSON
snapshot and, when they provide matching provenance, the exact BGR frame used
for that detection pass as a PNG.
"""
import argparse
import json
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

HOST = "127.0.0.1"
PORT = 8792
PING_INTERVAL = 2.0
PING_COUNT = 2
GRACE = 3
UPDATE_INTERVAL = 0.05
FRAME_HISTORY = 8


class _CameraPacket:
    """Adapter retaining the packet consumed by VisionOwner.update()."""

    def __init__(self, camera):
        self.camera = camera
        self.last = None

    def get_snapshot(self, *args, **kwargs):
        packet = self.camera.get_snapshot(*args, **kwargs)
        self.last = packet
        return packet


class VisionService:
    """Own camera/detector state and expose immutable, same-frame results."""

    def __init__(self, camera, owner, max_age=2.0):
        self.camera = camera
        self.owner = owner
        self.max_age = max_age
        self._packet = None
        self._history = deque(maxlen=FRAME_HISTORY)
        self._snapshot = None
        self._lock = threading.Lock()

    def update(self):
        snapshot = self.owner.update()
        packet = getattr(self.owner.camera, "last", None)
        if packet is None or packet.get("session_id") != snapshot["session_id"] \
                or int(packet.get("frame_id", -1)) != snapshot["frame_id"]:
            raise RuntimeError("owner did not publish the consumed frame")
        with self._lock:
            self._snapshot = dict(snapshot, objects=[dict(o) for o in snapshot["objects"]])
            packet_copy = {
                "session_id": packet["session_id"],
                "frame_id": int(packet["frame_id"]),
                "captured_at": float(packet["captured_at"]),
                "bgr": packet["bgr"].copy(),
            }
            if self._history and self._history[-1]["session_id"] != packet_copy["session_id"]:
                self._history.clear()
            self._history.append(packet_copy)
            self._packet = packet_copy
        return snapshot

    def snapshot(self):
        from core.azure_vision import validate_snapshot
        with self._lock:
            if self._snapshot is None:
                raise RuntimeError("no Azure vision snapshot")
            snapshot = dict(self._snapshot, objects=[dict(o) for o in self._snapshot["objects"]])
        validate_snapshot(snapshot, max_age=self.max_age)
        return snapshot

    def frame(self, session_id, frame_id):
        with self._lock:
            packet = next((item for item in self._history
                           if item["session_id"] == session_id and item["frame_id"] == frame_id), None)
            if packet is None:
                raise LookupError("frame provenance mismatch")
            if time.monotonic() - packet["captured_at"] > self.max_age:
                raise LookupError("frame expired")
            return packet["bgr"].copy(), packet["session_id"], packet["frame_id"]


class _Handler(BaseHTTPRequestHandler):
    server_version = "AzureVision/1"

    def do_GET(self):  # noqa: N802
        service = self.server.service
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/snapshot":
                snapshot = service.snapshot()
                payload = json.dumps(snapshot, separators=(",", ":")).encode()
                self._send(200, payload, "application/json", snapshot)
                return
            if parsed.path == "/frame":
                query = parse_qs(parsed.query)
                session_id = query.get("session_id", [""])[0]
                frame_text = query.get("frame_id", [""])[0]
                if not session_id or not frame_text:
                    raise ValueError("session_id and frame_id are required")
                frame, sid, fid = service.frame(session_id, int(frame_text))
                import cv2
                ok, encoded = cv2.imencode(".png", frame)
                if not ok:
                    raise RuntimeError("could not encode frame")
                self._send(200, encoded.tobytes(), "image/png",
                           {"session_id": sid, "frame_id": fid})
                return
            self._send_error(404, "not found")
        except (ValueError, LookupError) as exc:
            self._send_error(404 if isinstance(exc, LookupError) else 400, str(exc))
        except Exception as exc:
            self._send_error(503, str(exc))

    def _send(self, status, body, content_type, provenance):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if isinstance(provenance, dict) and "session_id" in provenance:
            self.send_header("X-Session-ID", str(provenance["session_id"]))
            self.send_header("X-Frame-ID", str(provenance["frame_id"]))
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, status, message):
        self._send(status, json.dumps({"error": message}).encode(), "application/json", {})

    def log_message(self, *_args):
        return


def make_server(service, host=HOST, port=PORT):
    if host != HOST:
        raise ValueError("vision service must bind to loopback")
    server = ThreadingHTTPServer((host, port), _Handler)
    server.service = service
    return server


def arm_alive(arm_ip):
    try:
        result = subprocess.run(
            ["ping", "-c", str(PING_COUNT), "-W", "1", arm_ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def _make_runtime():
    from core.azure_camera import AzureCamera
    from core.azure_vision import VisionOwner
    from core.config import DEFAULT_YOLO_MODEL, YOLO_CONF_GRASP
    from core.scene import SCAN_VOCAB
    from core.perception import Perception

    camera = AzureCamera(record=True)
    packet_camera = _CameraPacket(camera)
    perception = Perception(yolo_model_path=DEFAULT_YOLO_MODEL)
    detector = lambda rgb: perception.detect_objects(rgb, SCAN_VOCAB, conf=YOLO_CONF_GRASP)
    owner = VisionOwner(packet_camera, detector)
    return camera, VisionService(camera, owner)


def _update_loop(server, stopped):
    while not stopped.is_set():
        service = server.service
        if service is not None:
            try:
                service.update()
            except Exception as exc:
                print(f"[azure_recorder] vision update failed: {exc}")
        stopped.wait(UPDATE_INTERVAL)


def run(arm_ip, host=HOST, port=PORT):
    camera = None
    service = None
    server = None
    fail_streak = 0
    try:
        # The endpoint exists even while hardware is offline; it never opens it.
        server = make_server(None, host, port)
        stopped = threading.Event()
        updater = threading.Thread(target=_update_loop, args=(server, stopped), daemon=True)
        updater.start()
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"[azure_recorder] vision service listening on http://{host}:{port}")
        while True:
            alive = arm_alive(arm_ip)
            if alive and camera is None:
                try:
                    camera, service = _make_runtime()
                    camera.start()
                    server.service = service
                    fail_streak = 0
                    print("[azure_recorder] arm up -> start recording and vision")
                except Exception as exc:
                    print(f"[azure_recorder] camera start failed: {exc}")
                    if camera is not None:
                        try:
                            camera.stop()
                        except Exception:
                            pass
                    camera = service = None
            elif not alive and camera is not None:
                fail_streak += 1
                if fail_streak >= GRACE:
                    print("[azure_recorder] arm down -> stop recording")
                    try:
                        camera.stop()
                    finally:
                        camera = service = None
                        server.service = None
                        fail_streak = 0
            else:
                fail_streak = 0
            time.sleep(PING_INTERVAL)
    finally:
        stopped.set()
        if server is not None:
            server.shutdown()
            server.server_close()
        if camera is not None:
            camera.stop()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm-ip", default=None)
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args(argv)
    if args.arm_ip is None:
        with (ROOT / "robot_config.json").open() as stream:
            args.arm_ip = json.load(stream)["arm_ip"]
    run(args.arm_ip, args.host, args.port)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
