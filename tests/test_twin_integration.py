"""Explicit live-Twin integration test.

The test is skipped by default so normal pytest runs never connect to a live
robotics service.  Enable it deliberately with::

    RUN_TWIN_INTEGRATION=1 pytest -m integration -q tests/test_twin_integration.py
"""

import os

import pytest

from dependence.twin_inference.test_twin import run_smoke_test


RUN_LIVE_TWIN = os.getenv("RUN_TWIN_INTEGRATION", "").lower() in {
    "1", "true", "yes", "on"
}


@pytest.mark.integration
@pytest.mark.skipif(
    not RUN_LIVE_TWIN,
    reason="live Twin integration disabled; set RUN_TWIN_INTEGRATION=1",
)
def test_live_twin_trajectory_generation2():
    """Verify the current Twin protocol against an explicitly running service."""
    host = os.getenv("TWIN_TEST_HOST", "127.0.0.1")
    port = int(os.getenv("TWIN_TEST_PORT", "8020"))
    side = os.getenv("TWIN_TEST_SIDE", "left")

    response = run_smoke_test(host=host, port=port, side=side)

    assert response.get("value") is True, response
    info = response.get("info", {})
    assert isinstance(info, dict), response
    assert isinstance(info.get("trajectory"), list), response
    assert info["trajectory"], response
