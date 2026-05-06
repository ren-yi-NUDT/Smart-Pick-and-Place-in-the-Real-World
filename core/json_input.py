"""
JSON input parser for stdin-based command input.

Migrated from ``json_input.py``.

Input format (one JSON object per line via stdin):
    {"object": "orange", "container": "pink plate"}
    {"object": "apple,fruit", "container": "bowl", "direction": "left"}

Usage:
    from core.json_input import JsonInputParser
    parser = JsonInputParser()
    cmd = parser.get_command()
"""

import json
import sys
from typing import Any, Dict, Optional


class JsonInputParser:
    """Read JSON commands from stdin, one per line."""

    def __init__(self):
        pass

    def read_from_stdin(self) -> Optional[Dict[str, Any]]:
        """Read a single line of JSON from stdin.

        Returns ``None`` on EOF or empty line.
        """
        try:
            line = sys.stdin.readline()
            if not line:
                return None
            line = line.strip()
            if not line:
                return None
            return json.loads(line)
        except json.JSONDecodeError as e:
            print(f"JSON parse error: {e}")
            return None
        except Exception as e:
            print(f"Input read error: {e}")
            return None

    def parse(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract object, container, and direction fields from raw JSON.

        Returns
        -------
        dict with keys ``"object"``, ``"container"``, ``"direction"``, ``"original"``.
        """
        return {
            "object": data.get("object"),
            "container": data.get("container"),
            "direction": data.get("direction"),
            "original": data,
        }

    def get_command(self) -> Optional[Dict[str, Any]]:
        """Read and parse one command from stdin (convenience wrapper)."""
        data = self.read_from_stdin()
        if data is None:
            return None
        return self.parse(data)
