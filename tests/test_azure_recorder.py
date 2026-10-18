"""Offline owner/HTTP/lifecycle checks; never open Azure or call model APIs."""
import importlib.util
import json
import sys
import threading
import time
import types
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen

import cv2
import numpy as np
import pytest

from tools import azure_recorder as recorder


class Camera:
    def __init__(self):
        self.frame_id = 1
        self.session_id = "session-a"
        self.image = np.full((8, 12, 3), [10, 20, 30], dtype=np.uint8)

    def get_snapshot(self):
        packet = dict(session_id=self.session_id, frame_id=self.frame_id,
                      captured_at=time.monotonic(), bgr=self.image.copy())
        self.frame_id += 1
        return packet


class Owner:
    def __init__(self, camera):
        self.camera = camera

    def update(self):
        packet = self.camera.get_snapshot()
        return {k: packet[k] for k in ("session_id", "frame_id", "captured_at")} | {
            "width": 12, "height": 8, "objects": []}


@pytest.fixture
def service(monkeypatch):
    # Inject only the shared validation dependency: no hardware/model imports.
    module = types.ModuleType("core.azure_vision")

    def validate(snapshot, max_age):
        if time.monotonic() - snapshot["captured_at"] > max_age:
            raise ValueError("stale snapshot")
        return snapshot

    module.validate_snapshot = validate
    monkeypatch.setitem(sys.modules, "core.azure_vision", module)
    camera = Camera()
    return recorder.VisionService(camera, Owner(recorder._CameraPacket(camera)))


def test_retains_exact_frame_after_new_capture_and_rejects_missing_or_expired(service):
    first = service.update()
    service.camera.image[:] = [40, 50, 60]
    service.update()
    frame, sid, fid = service.frame(first["session_id"], first["frame_id"])
    assert (sid, fid) == ("session-a", 1)
    assert frame[0, 0].tolist() == [10, 20, 30]
    frame[:] = 0
    assert service.frame(sid, fid)[0][0, 0].tolist() == [10, 20, 30]
    with pytest.raises(LookupError):
        service.frame("different-session", fid)
    with pytest.raises(LookupError):
        service.frame(sid, 999)
    service.max_age = -1
    with pytest.raises(LookupError, match="expired"):
        service.frame(sid, fid)
    with pytest.raises(ValueError, match="stale"):
        service.snapshot()


def test_frame_store_is_bounded_and_session_restart_invalidates_old_frames(service):
    first = service.update()
    for _ in range(recorder.FRAME_HISTORY):
        service.update()
    with pytest.raises(LookupError):
        service.frame(first["session_id"], first["frame_id"])
    previous = service.snapshot()
    service.camera.session_id = "session-b"
    service.update()
    with pytest.raises(LookupError):
        service.frame(previous["session_id"], previous["frame_id"])


def test_http_snapshot_png_provenance_and_read_only_routes(service):
    expected = service.update()
    server = recorder.make_server(service, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(base + "/snapshot") as response:
            assert json.load(response) == expected
            assert response.headers["X-Session-ID"] == expected["session_id"]
            assert response.headers["X-Frame-ID"] == str(expected["frame_id"])
            assert response.headers["Cache-Control"] == "no-store"
        query = f"?session_id={expected['session_id']}&frame_id={expected['frame_id']}"
        with urlopen(base + "/frame" + query) as response:
            image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
            assert np.array_equal(image, service.camera.image)
            assert response.headers["Content-Type"] == "image/png"
        for route, code in [("/frame", 400), ("/frame?session_id=x&frame_id=no", 400),
                            ("/frame?session_id=x&frame_id=1", 404), ("/move", 404)]:
            with pytest.raises(HTTPError) as error:
                urlopen(base + route)
            assert error.value.code == code
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_public_bind_is_rejected(service):
    with pytest.raises(ValueError, match="loopback"):
        recorder.make_server(service, host="0.0.0.0", port=0)
