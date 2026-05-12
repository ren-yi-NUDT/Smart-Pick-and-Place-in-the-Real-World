"""
ROS2 lifecycle singleton for the Igrape backend.

Ensures rclpy is initialized exactly once and provides a shared Node
that all Igrape adapters use. A background thread spins the node so
that TF2 lookups, subscriber callbacks, and publisher latching work
without the caller needing to call rclpy.spin().

Also ensures Igrape's ROS2 workspace is on sys.path so that
bodyctrl_msgs and other custom messages can be imported.
"""

import os
import sys
import threading

_IGRAPE_ROOT = "/home/zz/Code/IgrapeRobot3/IgrapeRobot3-task_planner_v3.0"
_INSTALL_PATH = os.path.join(_IGRAPE_ROOT, "install")


def _ensure_igrape_on_path():
    if _INSTALL_PATH not in sys.path:
        sys.path.insert(0, _INSTALL_PATH)
    # Also add the python lib path for the workspace
    for subdir in os.listdir(_INSTALL_PATH) if os.path.isdir(_INSTALL_PATH) else []:
        lib_path = os.path.join(_INSTALL_PATH, subdir, "lib", "python3.10", "site-packages")
        if os.path.isdir(lib_path) and lib_path not in sys.path:
            sys.path.insert(0, lib_path)


class ROS2Context:
    """Singleton that owns the rclpy lifecycle and a shared Node."""

    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        _ensure_igrape_on_path()
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        from rclpy.node import Node
        self.node = Node('skill_db_igrape_adapter')
        self._spinner_thread = threading.Thread(target=self._spin, daemon=True)
        self._spinner_thread.start()

    def _spin(self):
        import rclpy
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    @classmethod
    def get(cls) -> "ROS2Context":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def shutdown(self):
        import rclpy
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()
