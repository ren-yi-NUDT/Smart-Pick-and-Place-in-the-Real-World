"""Dual-arm synchronization primitives for coordinating parallel arm execution."""

import threading


class DualArmSync:
    """Thread-safe synchronization for two arms using barriers, events, and shared state."""

    def __init__(self):
        self._barriers = {}
        self._events = {}
        self._state = {}
        self._lock = threading.Lock()

    def barrier(self, name: str):
        self._barriers.setdefault(name, threading.Barrier(2)).wait()

    def notify(self, name: str):
        self._events.setdefault(name, threading.Event()).set()

    def wait(self, name: str, timeout: float = None) -> bool:
        return self._events.setdefault(name, threading.Event()).wait(timeout)

    def set_state(self, key: str, value):
        with self._lock:
            self._state[key] = value

    def get_state(self, key: str, default=None):
        with self._lock:
            return self._state.get(key, default)
