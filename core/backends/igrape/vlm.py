"""
Igrape-bot3 VLM adapter.

Currently delegates to the existing VLMClient (GLM-4.5V).
"""

from core.vlm import VLMClient
from core.abc import BaseVLM


class IgrapeVLM(VLMClient):
    """VLM adapter for Igrape. Currently uses the same GLM-4.5V backend."""
    pass
