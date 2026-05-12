"""
Igrape-bot3 perception adapter.

Currently delegates to the existing Perception class (YOLO-World + AnyGrasp).
Future: can be replaced with the remote grasp WebSocket service
(ws://192.168.3.11:8775) for Igrape-specific perception.
"""

from core.perception import Perception
from core.abc import BasePerception


class IgrapePerception(Perception):
    """Perception adapter for Igrape. Currently uses the same Perception backend."""
    pass
