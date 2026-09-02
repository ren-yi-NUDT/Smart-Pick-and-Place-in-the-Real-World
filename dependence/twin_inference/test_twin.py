#!/usr/bin/env python3
"""Manual Twin trajectory-generation smoke test.

This file intentionally keeps the historical ``test_twin.py`` name because
it is used by existing operators.  It must remain safe to import: pytest
discovers files named ``test_*.py`` during collection, and importing this
module must never connect to a live Twin service or send a robot request.

Run manually with::

    python dependence/twin_inference/test_twin.py --side left --port 8020

The request uses the current project protocol through :class:`TwinClient`:
raw JSON on send and a four-byte length-prefixed JSON response.
"""

import argparse
import os
import sys


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from core.twin_client import TwinClient  # noqa: E402


TARGET_POSE = [
    -0.29824502615690757,
    -0.1048591177826923,
    0.1558604879639906,
    0.7429832323953516,
    -0.3406782645617565,
    -0.5639482092149282,
    -0.11779920949573683,
]
CURRENT_JS_RAD = [
    -0.021013764194011724,
    -0.15385077356330015,
    1.0034596001416198,
    -0.05811946409141117,
    1.986167235477027,
    -0.00020943951023931956,
]


def build_smoke_config(side="left"):
    """Build one safe, deterministic trajectory-generation request."""
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return {
        "target_pose": [TARGET_POSE],
        "current_js": list(CURRENT_JS_RAD),
        "struct": f"{side}_arm",
    }


def run_smoke_test(host="127.0.0.1", port=8020, side="left"):
    """Send one explicit Twin smoke request and return its response.

    No connection is made until this function is called.  The socket is
    always closed, including when Twin reports an error.
    """
    client = TwinClient(host=host, port=int(port))
    if not client.connect():
        raise ConnectionError(f"unable to connect to Twin at {host}:{port}")
    try:
        response = client.generate_trajectory2(build_smoke_config(side))
        if not isinstance(response, dict):
            raise RuntimeError(f"Twin returned a non-object response: {response!r}")
        if "value" not in response:
            raise RuntimeError(f"Twin response has no 'value' field: {response!r}")
        return response
    finally:
        client.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.getenv("TWIN_TEST_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("TWIN_TEST_PORT", "8020")))
    parser.add_argument("--side", choices=("left", "right"), default="left")
    args = parser.parse_args(argv)

    response = run_smoke_test(args.host, args.port, args.side)
    print(f"Twin smoke test passed: value={response.get('value')}")
    info = response.get("info")
    if isinstance(info, dict) and isinstance(info.get("trajectory"), list):
        print(f"trajectory waypoints: {len(info['trajectory'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
