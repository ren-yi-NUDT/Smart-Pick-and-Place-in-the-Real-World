"""Azure Kinect: 抓帧线程 + MKV 录像 + 实时帧共享（供 YOLO/VLM 使用）"""
import queue
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from pyk4a import Config, PyK4A, PyK4ARecord, ColorResolution, FPS

RECORDINGS_DIR = Path(__file__).resolve().parent.parent / "recordings"

# 录像上限：任一超限即关闭文件并停止录像（防止单文件撑爆磁盘/内存）
MAX_RECORD_SECONDS = 1800        # 30 分钟
MAX_RECORD_BYTES = 10 * 1024**3  # 10 GB


class AzureCamera:
    def __init__(self, record: bool = False, recordings_dir: Path = RECORDINGS_DIR):
        self._config = Config(
            color_resolution=ColorResolution.RES_1080P,
            camera_fps=FPS.FPS_30,
        )
        # thread_safe=False: pyk4a C 层全局锁会和主线程死锁（GIL），单取帧线程下安全
        self._device = PyK4A(self._config, thread_safe=False)
        self._record = record
        self._recordings_dir = recordings_dir
        self._recorder = None
        self._record_queue = queue.Queue(maxsize=120)  # ~4s 缓冲，满则丢帧
        self._writer_thread = None
        self._latest_frame = None       # BGR ndarray, RGB去alpha
        self._latest_ts = 0.0
        self._latest_frame_id = 0
        self.session_id = uuid.uuid4().hex
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = None
        self.frame_count = 0
        self.record_path = None
        self._record_start_ts = 0.0
        self._record_stopped = False

    def _record_over_limit(self) -> bool:
        if self._record_stopped or self._recorder is None:
            return False
        if time.time() - self._record_start_ts > MAX_RECORD_SECONDS:
            return True
        try:
            if self.record_path and self.record_path.exists() \
                    and self.record_path.stat().st_size > MAX_RECORD_BYTES:
                return True
        except OSError:
            pass
        return False

    def _stop_record(self):
        print(f"[azure] record limit hit, closing {self.record_path}")
        self._record_stopped = True
        try:
            self._record_queue.join()  # 排空剩余帧
        except Exception:
            pass
        try:
            self._recorder.flush()
            self._recorder.close()
        except Exception as e:
            print(f"[azure] recorder close error: {e}")
        self._recorder = None

    def start(self):
        self._device.start()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=5)
        if self._writer_thread:
            self._record_queue.join()  # 排空剩余帧
            self._writer_thread.join(timeout=10)
            self._writer_thread = None
        if self._recorder:
            self._recorder.flush()
            self._recorder.close()
            self._recorder = None
        self._device.stop()

    def get_snapshot(self, max_age: float = 0.5):
        """Return one atomic frame packet for downstream vision consumers."""
        with self._lock:
            if self._latest_frame is None or time.monotonic() - self._latest_ts > max_age:
                raise RuntimeError("no fresh frame from Azure Kinect")
            return {"session_id": self.session_id, "frame_id": self._latest_frame_id,
                    "captured_at": self._latest_ts, "bgr": self._latest_frame.copy()}

    def get_frame(self, max_age: float = 0.5) -> np.ndarray:
        """取最新一帧 BGR。stale 超过 max_age 秒抛 RuntimeError。"""
        with self._lock:
            if self._latest_frame is None or time.monotonic() - self._latest_ts > max_age:
                raise RuntimeError("no fresh frame from Azure Kinect")
            return self._latest_frame.copy()

    def _loop(self):
        if self._record:
            self._open_record()
            self._writer_thread = threading.Thread(
                target=self._write_loop, daemon=True)
            self._writer_thread.start()
        while not self._stop_evt.is_set():
            try:
                cap = self._device.get_capture(timeout=1000)
                if cap.color is None or cap.color.size == 0:
                    continue
                if self._recorder:
                    if self._record_over_limit():
                        self._stop_record()
                        continue
                    try:
                        self._record_queue.put_nowait(cap)  # 写盘异步化，满则丢帧不卡取帧
                    except queue.Full:
                        self.frame_count += 1
                        pass
            except Exception as e:
                print(f"[azure] capture error: {e}")
                continue
            frame = cap.color[:, :, :3]
            with self._lock:
                self._latest_frame = frame
                self._latest_ts = time.monotonic()
                self._latest_frame_id += 1
            self.frame_count += 1
        self._stop_evt.set()  # 通知写盘线程收尾

    def _write_loop(self):
        # ponytail: 丢帧容忍 —— 队列满直接丢，保实时优先于完整性
        while not (self._stop_evt.is_set() and self._record_queue.empty()):
            try:
                cap = self._record_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if self._recorder is not None:
                    self._recorder.write_capture(cap)
            except Exception as e:
                print(f"[azure] record write error: {e}")
            self._record_queue.task_done()

    def _open_record(self):
        self._recordings_dir.mkdir(parents=True, exist_ok=True)
        self.record_path = self._recordings_dir / (
            f"azure_{datetime.now():%Y%m%d_%H%M%S}.mkv")
        self._recorder = PyK4ARecord(path=self.record_path,
                                     config=self._config,
                                     device=self._device)
        self._recorder.create()
        self._recorder.write_header()
        self._record_start_ts = time.time()
