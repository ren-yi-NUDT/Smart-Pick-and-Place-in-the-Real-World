#!/usr/bin/env python3
"""
Test script for hand verification, critic logic, and grasp/destination
verification.

Run without arguments to test all scenarios in mock mode (no hardware).
Use --real to test against the actual arm/hand servers.
Use --scenario to pick specific test cases.

Usage:
    python test_critic.py                    # all tests, mock mode
    python test_critic.py --scenario grasp   # only grasp verification tests
    python test_critic.py --scenario dest    # only destination tests
    python test_critic.py --scenario goal    # only goal verification tests
    python test_critic.py --scenario edge    # only edge-case tests
    python test_critic.py --real             # against real hardware
    python test_critic.py --list             # list available scenarios
"""

import sys
import os
import json
import time
import argparse
from unittest.mock import MagicMock, patch
import numpy as np

# ---------------------------------------------------------------------------
# Constants (mirrors core/config.py)
# ---------------------------------------------------------------------------
HAND_CLOSE = [0, 0, 0, 460, 0, 0]
HAND_OPEN = [1000, 1000, 1000, 1000, 1000, 0]

# ---------------------------------------------------------------------------
# Color helpers
# ---------------------------------------------------------------------------
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"
BOLD = "\033[1m"


def ok(msg):
    print(f"  {GREEN}[PASS]{RESET} {msg}")


def fail(msg):
    print(f"  {RED}[FAIL]{RESET} {msg}")


def info(msg):
    print(f"  {CYAN}[INFO]{RESET} {msg}")


def section(title):
    print(f"\n{BOLD}{'='*60}{RESET}")
    print(f"{BOLD}  {title}{RESET}")
    print(f"{BOLD}{'='*60}{RESET}")


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------
class TestStats:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []

    def check(self, condition, name):
        if condition:
            self.passed += 1
            ok(name)
        else:
            self.failed += 1
            fail(name)
            self.errors.append(name)

    def summary(self) -> bool:
        total = self.passed + self.failed
        print(f"\n{BOLD}Results: {self.passed}/{total} passed{RESET}")
        if self.errors:
            print(f"{RED}Failed tests:{RESET}")
            for e in self.errors:
                print(f"  - {e}")
        return self.failed == 0


# ===================================================================
# HandClient simulation tests (mock mode)
# ===================================================================
def test_hand_logic(stats: TestStats):
    """Test HandClient methods with simulated responses."""
    section("HandClient Logic (mock)")

    from core.hand import HandClient

    # -- helper to create a mock hand with given state response --
    def make_hand(motor_values):
        h = HandClient.__new__(HandClient)
        h.sock = MagicMock()
        h._hand_config = {"close": list(HAND_CLOSE), "open": list(HAND_OPEN)}
        h._send_cmd = MagicMock(return_value={"value": motor_values})
        return h

    # --- is_grasping ---
    # Fully closed hand → NOT grasping
    h = make_hand(HAND_CLOSE)
    stats.check(not h.is_grasping(), "is_grasping: fully closed → False")

    # Open hand → grasping (deviates from close)
    h = make_hand(HAND_OPEN)
    stats.check(h.is_grasping(), "is_grasping: fully open → True (deviation > 20)")

    # Slightly open (holding thin object like paper) → grasping
    h = make_hand([10, 10, 10, 470, 10, 0])
    stats.check(h.is_grasping(), "is_grasping: slight deviation → True")

    # Empty response → NOT grasping
    h = make_hand([])
    stats.check(not h.is_grasping(), "is_grasping: empty response → False")

    # Partially closed (5 fingers, missing 6th) → False
    h = make_hand([100, 100, 100, 100, 100])
    stats.check(not h.is_grasping(), "is_grasping: short response → False")

    # --- is_fully_open ---
    h = make_hand(HAND_OPEN)
    stats.check(h.is_fully_open(), "is_fully_open: fully open → True")

    h = make_hand(HAND_CLOSE)
    stats.check(not h.is_fully_open(), "is_fully_open: fully closed → False")

    h = make_hand([990, 990, 990, 990, 990, 0])
    stats.check(h.is_fully_open(), "is_fully_open: near-open → True (within 50)")

    # --- get_finger_deviation ---
    h = make_hand(HAND_CLOSE)
    dev = h.get_finger_deviation()
    stats.check(dev == 0.0, f"finger_deviation: fully closed = 0.0 (got {dev:.1f})")

    h = make_hand(HAND_OPEN)
    dev = h.get_finger_deviation()
    expected = sum(abs(np.array(HAND_OPEN) - np.array(HAND_CLOSE)))
    stats.check(abs(dev - expected) < 1.0,
                f"finger_deviation: fully open ≈ {expected} (got {dev:.1f})")

    h = make_hand([0, 0, 0, 500, 0, 0])  # only index deviates by 40
    dev = h.get_finger_deviation()
    stats.check(dev == 40.0, f"finger_deviation: single finger offset = 40 (got {dev:.1f})")


# ===================================================================
# Critic verify_grasp tests
# ===================================================================
def test_verify_grasp(stats: TestStats):
    """Test WorldModelCritic.verify_grasp with various evidence payloads."""
    section("Critic verify_grasp")

    from core.world_memory import WorldMemory
    from core.world_model_critic import WorldModelCritic

    def make_critic(obj="peach", container="person"):
        m = WorldMemory("test_mem", {"object": obj, "container": container}, root="/tmp/test_critic_memory")
        return WorldModelCritic(m), m

    # --- Scenario 1: Successful grasp with real evidence ---
    critic, mem = make_critic()
    result = critic.verify_grasp({
        "success": True,           # trajectory OK
        "hand_closed": True,       # close command sent
        "finger_deviation": 150.0, # fingers deviated → object held
        "object": "peach",
    })
    stats.check(result["success"], "grasp(evidence): trajectory+deviation → True")
    stats.check(result["observed"]["evidence_holding"], "grasp(evidence): evidence_holding flag True")
    stats.check(mem.data["robot_state"]["holding"] == "peach",
                f"grasp(evidence): holding set to 'peach' (got {mem.data['robot_state']['holding']})")

    # --- Scenario 2: THE RUBBER STAMP — trajectory OK but no hand evidence ---
    critic, mem = make_critic()
    result = critic.verify_grasp({
        "success": True,
        "hand_closed": False,
        "finger_deviation": 0.0,
        "object": "peach",
    })
    stats.check(not result["success"],
                "grasp(rubber-stamp): trajectory OK, no evidence → REJECTED")
    stats.check(not result["observed"]["evidence_holding"],
                "grasp(rubber-stamp): evidence_holding flag False")
    stats.check(mem.data["robot_state"]["holding"] is None,
                "grasp(rubber-stamp): holding remains None (not set)")
    stats.check(len(mem.data["failure_records"]) > 0,
                "grasp(rubber-stamp): failure recorded")

    # --- Scenario 3: Trajectory OK, hand closed, but zero deviation ---
    # This is the key case: arm moved to position, hand sent "close" cmd,
    # but fingers hit nothing → fully closed state
    critic, mem = make_critic()
    result = critic.verify_grasp({
        "success": True,
        "hand_closed": True,
        "finger_deviation": 0.0,  # fingers closed fully → nothing between them
        "object": "peach",
    })
    stats.check(not result["success"],
                "grasp(missed): hand closed but zero deviation → REJECTED")
    stats.check(not result["observed"]["evidence_holding"],
                "grasp(missed): evidence_holding False (deviation 0)")
    stats.check(len(mem.data["failure_records"]) > 0,
                "grasp(missed): failure recorded")

    # --- Scenario 4: Trajectory OK, deviation just above threshold ---
    critic, mem = make_critic()
    result = critic.verify_grasp({
        "success": True,
        "hand_closed": True,
        "finger_deviation": 6.0,  # thin object, barely above threshold
        "object": "peach",
    })
    stats.check(result["success"],
                "grasp(thin_object): deviation 6.0 > threshold 5.0 → ACCEPTED")
    stats.check(mem.data["robot_state"]["holding"] == "peach",
                "grasp(thin_object): holding set correctly")

    # --- Scenario 5: Trajectory OK, deviation at exactly threshold ---
    critic, mem = make_critic()
    result = critic.verify_grasp({
        "success": True,
        "hand_closed": True,
        "finger_deviation": 5.0,  # exactly at threshold
        "object": "peach",
    })
    stats.check(not result["success"],
                "grasp(boundary): deviation 5.0 NOT > 5.0 → REJECTED")

    # --- Scenario 6: Trajectory FAILED — regardless of hand evidence ---
    critic, mem = make_critic()
    result = critic.verify_grasp({
        "success": False,
        "hand_closed": False,
        "finger_deviation": 0.0,
        "object": "peach",
    })
    stats.check(not result["success"],
                "grasp(failed_traj): trajectory failed → REJECTED")
    stats.check(mem.data["robot_state"]["holding"] is None,
                "grasp(failed_traj): holding not set")

    # --- Scenario 7: Missing fields (empty dict) ---
    critic, mem = make_critic()
    result = critic.verify_grasp({})
    stats.check(not result["success"],
                "grasp(empty_dict): all defaults → REJECTED")

    # --- Scenario 8: None input ---
    critic, mem = make_critic()
    result = critic.verify_grasp(None)
    stats.check(not result["success"],
                "grasp(None): None input → REJECTED")


# ===================================================================
# Critic verify_destination_action tests
# ===================================================================
def test_verify_destination(stats: TestStats):
    """Test WorldModelCritic.verify_destination_action."""
    section("Critic verify_destination_action")

    from core.world_memory import WorldMemory
    from core.world_model_critic import WorldModelCritic

    def make_critic(mode_container="person"):
        m = WorldMemory("test_mem", {"object": "peach", "container": mode_container}, root="/tmp/test_critic_memory")
        m.data["robot_state"]["holding"] = "peach"  # was holding before place
        return WorldModelCritic(m), m

    # --- handover mode: hand opened → success ---
    critic, mem = make_critic("person")
    result = critic.verify_destination_action({
        "success": True,
        "hand_opened": True,
        "type": "handover",
        "container": "person",
    })
    stats.check(result["success"], "dest(handover): hand opened → True")
    stats.check(mem.data["robot_state"]["holding"] is None,
                "dest(handover): holding cleared to None")
    stats.check(len(result["observed"].get("evidence_released", False) and [True] or []) == 1,
                "dest(handover): evidence_released True")  # just check it exists

    # --- handover mode: hand NOT opened → failure ---
    critic, mem = make_critic("person")
    result = critic.verify_destination_action({
        "success": True,
        "hand_opened": False,
        "type": "handover",
        "container": "person",
    })
    stats.check(not result["success"],
                "dest(handover): hand NOT opened → REJECTED")

    # --- trash mode: hand opened → success ---
    critic, mem = make_critic("trash")
    result = critic.verify_destination_action({
        "success": True,
        "hand_opened": True,
        "type": "trash",
        "container": "trash",
    })
    stats.check(result["success"], "dest(trash): hand opened → True")
    stats.check(mem.data["robot_state"]["holding"] is None,
                "dest(trash): holding cleared")

    # --- desk_place mode: hand opened → success ---
    critic, mem = make_critic("desk")
    result = critic.verify_destination_action({
        "success": True,
        "hand_opened": True,
        "type": "desk_place",
        "container": "desk",
    })
    stats.check(result["success"], "dest(desk_place): hand opened → True")

    # --- normal_place mode: trajectory + hand opened → success ---
    critic, mem = make_critic("green bowl")
    result = critic.verify_destination_action({
        "success": True,
        "hand_opened": True,
        "type": "place",
        "container": "green bowl",
    })
    stats.check(result["success"], "dest(place): trajectory OK + hand opened → True")

    # --- normal_place mode: trajectory OK but hand NOT opened → failure ---
    critic, mem = make_critic("green bowl")
    result = critic.verify_destination_action({
        "success": True,
        "hand_opened": False,
        "type": "place",
        "container": "green bowl",
    })
    stats.check(not result["success"],
                "dest(place): trajectory OK, hand NOT opened → REJECTED")

    # --- normal_place mode: trajectory FAILED, hand opened → failure ---
    critic, mem = make_critic("green bowl")
    result = critic.verify_destination_action({
        "success": False,
        "hand_opened": True,
        "type": "place",
        "container": "green bowl",
    })
    stats.check(not result["success"],
                "dest(place): trajectory failed, hand opened → REJECTED")

    # --- Empty dict ---
    critic, mem = make_critic()
    result = critic.verify_destination_action({})
    stats.check(not result["success"],
                "dest(empty_dict): all defaults → REJECTED")


# ===================================================================
# Critic verify_goal tests
# ===================================================================
def test_verify_goal(stats: TestStats):
    """Test WorldModelCritic.verify_goal end-to-end."""
    section("Critic verify_goal")

    from core.world_memory import WorldMemory
    from core.world_model_critic import WorldModelCritic

    def make_critic():
        m = WorldMemory("test_mem", {"object": "peach", "container": "person"}, root="/tmp/test_critic_memory")
        return WorldModelCritic(m), m

    # --- Full success: both verifications passed with evidence ---
    critic, mem = make_critic()
    goal = critic.verify_goal({
        "object": "peach",
        "container": "person",
        "mode": "handover",
        "grasp_result": {"success": True, "hand_closed": True, "finger_deviation": 150.0},
        "grasp_verification": {"success": True, "observed": {"evidence_holding": True}},
        "destination_result": {"success": True, "hand_opened": True},
        "destination_verification": {"success": True, "observed": {"evidence_released": True}},
        "resolved_failures": 0,
    })
    stats.check(goal["success"], "goal(full_success): all conditions met → True")
    stats.check(goal["conditions"]["grasp_success"], "goal: grasp_success True")
    stats.check(goal["conditions"]["destination_action_success"], "goal: dest_success True")
    stats.check(goal["conditions"]["robot_not_holding"], "goal: robot_not_holding True")
    stats.check(goal["conditions"]["no_unresolved_failures"], "goal: no_unresolved_failures True")

    # --- Grasp failed → goal fails ---
    critic, mem = make_critic()
    goal = critic.verify_goal({
        "object": "peach",
        "container": "person",
        "mode": "handover",
        "grasp_result": {"success": True, "hand_closed": False, "finger_deviation": 0.0},
        "grasp_verification": {"success": False},
        "destination_result": {},
        "destination_verification": {"success": False},
        "resolved_failures": 0,
    })
    stats.check(not goal["success"], "goal(grasp_failed): grasp_verification False → goal fails")

    # --- Destination failed → goal fails ---
    critic, mem = make_critic()
    goal = critic.verify_goal({
        "object": "peach",
        "container": "person",
        "mode": "handover",
        "grasp_result": {"success": True, "hand_closed": True, "finger_deviation": 150.0},
        "grasp_verification": {"success": True},
        "destination_result": {"success": True, "hand_opened": False},
        "destination_verification": {"success": False},
        "resolved_failures": 0,
    })
    stats.check(not goal["success"], "goal(dest_failed): dest_verification False → goal fails")

    # --- Unresolved failures → goal fails ---
    critic, mem = make_critic()
    mem.record_failure({"stage": "after_grasp", "reason": "prior failure"})
    goal = critic.verify_goal({
        "object": "peach",
        "container": "person",
        "mode": "handover",
        "grasp_result": {"success": True, "hand_closed": True, "finger_deviation": 150.0},
        "grasp_verification": {"success": True},
        "destination_result": {"success": True, "hand_opened": True},
        "destination_verification": {"success": True},
        "resolved_failures": 0,  # not resolved
    })
    stats.check(not goal["success"], "goal(unresolved_failures): failure not resolved → goal fails")
    stats.check(goal["unresolved_failures"] > 0,
                f"goal: unresolved_failures={goal['unresolved_failures']}")

    # --- Failures resolved → goal succeeds ---
    critic, mem = make_critic()
    mem.record_failure({"stage": "after_grasp", "reason": "prior failure"})
    goal = critic.verify_goal({
        "object": "peach",
        "container": "person",
        "mode": "handover",
        "grasp_result": {"success": True, "hand_closed": True, "finger_deviation": 150.0},
        "grasp_verification": {"success": True},
        "destination_result": {"success": True, "hand_opened": True},
        "destination_verification": {"success": True},
        "resolved_failures": 1,  # resolved!
    })
    stats.check(goal["success"], "goal(resolved_failures): failure resolved → goal passes")


# ===================================================================
# Safe release / recovery tests
# ===================================================================
def test_safe_release(stats: TestStats):
    """Test the Skill.safe_release() recovery method."""
    section("Skill.safe_release (recovery)")

    from unittest.mock import MagicMock, call, patch
    from skills.base import Skill

    # Skill is abstract — create a minimal concrete subclass for testing
    class _TestSkill(Skill):
        def run(self, **kwargs):
            pass

    skill = _TestSkill()
    skill._arm = MagicMock()
    skill._hand = MagicMock()

    # --- Normal recovery: arm works, hand opens ---
    skill._arm.reset_mock()
    skill._hand.reset_mock()
    skill.safe_release(safe_pose="grasp1")
    stats.check(skill._arm.move_to_named_pose.called,
                "safe_release: arm.move_to_named_pose called")
    stats.check(skill._hand.open.called,
                "safe_release: hand.open() called")
    # Hand must be called even if arm succeeds
    call_order = [c[0] for c in skill._hand.method_calls]
    stats.check("open" in str(call_order),
                "safe_release: hand.open is in method calls")

    # --- Recovery when arm fails: hand still opens ---
    skill._arm.reset_mock()
    skill._hand.reset_mock()
    skill._arm.move_to_named_pose.side_effect = RuntimeError("arm disconnected")
    skill.safe_release(safe_pose="grasp1")
    stats.check(skill._hand.open.called,
                "safe_release(arm_fail): hand.open() still called despite arm error")

    # --- Recovery when everything fails: no unhandled exception ---
    skill._arm.reset_mock()
    skill._hand.reset_mock()
    skill._arm.move_to_named_pose.side_effect = RuntimeError("arm down")
    skill._hand.open.side_effect = RuntimeError("hand down")
    try:
        skill.safe_release(safe_pose="grasp1")
        stats.check(True, "safe_release(double_fail): no exception propagated")
    except Exception:
        stats.check(False, "safe_release(double_fail): exception should not propagate")


# ===================================================================
# Result-structure / retry-tracking tests
# ===================================================================
def test_retry_tracking(stats: TestStats):
    """Test that retry/recovery fields are tracked in result dicts."""
    section("Retry & Recovery Tracking")

    # --- verify_goal counts recoveries as resolved_failures ---
    from core.world_memory import WorldMemory
    from core.world_model_critic import WorldModelCritic

    m = WorldMemory("test_mem", {"object": "peach", "container": "person"}, root="/tmp/test_critic_memory")
    critic = WorldModelCritic(m)

    # Simulate: 1st grasp missed (recorded as failure), 2nd succeeded
    # This mimics the retry loop: grasp_attempts=2, dest_attempts=1
    m.record_failure({"stage": "after_grasp", "reason": "first grasp missed"})

    goal = critic.verify_goal({
        "object": "peach",
        "container": "person",
        "mode": "handover",
        "grasp_result": {"success": True, "hand_closed": True, "finger_deviation": 150.0},
        "grasp_verification": {"success": True},
        "destination_result": {"success": True, "hand_opened": True},
        "destination_verification": {"success": True},
        "resolved_failures": 1,  # recovery resolved the failed attempt
    })
    stats.check(goal["success"],
                "retry: 1 failure + 1 resolved → goal passes")
    stats.check(goal["failure_count"] == 1,
                f"retry: failure_count=1 (got {goal['failure_count']})")
    stats.check(goal["unresolved_failures"] == 0,
                f"retry: unresolved=0 (got {goal['unresolved_failures']})")

    # --- Multiple failures, not all resolved → goal fails ---
    m2 = WorldMemory("test_mem2", {"object": "peach", "container": "plate"}, root="/tmp/test_critic_memory")
    critic2 = WorldModelCritic(m2)
    m2.record_failure({"stage": "after_grasp", "reason": "grasp miss 1"})
    m2.record_failure({"stage": "after_grasp", "reason": "grasp miss 2"})

    goal2 = critic2.verify_goal({
        "object": "peach",
        "container": "plate",
        "mode": "normal_place",
        "grasp_result": {"success": True, "hand_closed": True, "finger_deviation": 120.0},
        "grasp_verification": {"success": True},
        "destination_result": {"success": True, "hand_opened": True},
        "destination_verification": {"success": True},
        "resolved_failures": 1,  # only 1 of 2 resolved
    })
    stats.check(not goal2["success"],
                "retry: 2 failures + 1 resolved → goal fails")
    stats.check(goal2["unresolved_failures"] == 1,
                f"retry: unresolved=1 (got {goal2['unresolved_failures']})")

    # --- No failures at all → clean pass ---
    m3 = WorldMemory("test_mem3", {"object": "peach", "container": "plate"}, root="/tmp/test_critic_memory")
    critic3 = WorldModelCritic(m3)
    goal3 = critic3.verify_goal({
        "object": "peach",
        "container": "plate",
        "mode": "normal_place",
        "grasp_result": {"success": True, "hand_closed": True, "finger_deviation": 200.0},
        "grasp_verification": {"success": True},
        "destination_result": {"success": True, "hand_opened": True},
        "destination_verification": {"success": True},
        "resolved_failures": 0,
    })
    stats.check(goal3["success"], "retry: zero failures → clean pass")
    stats.check(goal3["failure_count"] == 0,
                f"retry: failure_count=0 (got {goal3['failure_count']})")


# ===================================================================
# fetch_from_user recovery tests
# ===================================================================
def test_fetch_from_user_recovery(stats: TestStats):
    """Test that fetch_from_user calls safe_release on all failure paths."""
    section("fetch_from_user Recovery")

    from unittest.mock import MagicMock, patch, PropertyMock
    import sys

    # Mock the entire hardware stack to avoid network/ROS imports
    mock_config = MagicMock()
    mock_config.get_pose.return_value = None  # all poses missing → triggers failure paths
    mock_config.robot_config = {}
    mock_config.default_traj_js = {"grasp1": {}, "grasp2": {}, "grasp3": {}, "grasp4": {}}

    with patch.dict(sys.modules, {
        'skills.base': MagicMock(),
        'core.transforms': MagicMock(),
        'core.config': MagicMock(Config=mock_config),
        'numpy': MagicMock(),
        'scipy.spatial.transform': MagicMock(),
        'termcolor': MagicMock(),
    }):
        # We test the structural property: that safe_release is called
        # before every `return False` in the failure paths.
        # This is verified by code review (see below) — we validate the
        # file structure here.

        import inspect
        import os

        repo_root = os.path.dirname(os.path.abspath(__file__))
        fetch_path = os.path.join(repo_root, "skills", "fetch_from_user.py")

        with open(fetch_path) as f:
            source = f.read()

        lines = source.split("\n")

        # Find the run() method body range so we only check return False
        # lines inside run() — not in helper methods like _execute_placement().
        run_start = None
        run_end = None
        for i, line in enumerate(lines):
            if line.strip().startswith("def run("):
                run_start = i
            elif run_start is not None and line.startswith("    def "):
                run_end = i
                break

        # Collect return False line numbers inside run() only
        return_false_in_run = []
        for i in range(run_start or 0, run_end or len(lines)):
            if lines[i].strip() == "return False":
                return_false_in_run.append(i + 1)  # 1-based

        # Count safe_release calls across the whole file
        safe_release_count = source.count("safe_release")
        stats.check(safe_release_count == 4,
                    f"fetch_from_user: 4 safe_release calls (got {safe_release_count})")

        # Verify that each return False AFTER grasp inside run()
        # has safe_release before it
        grasp_check_line = None
        for i, line in enumerate(lines):
            if "check_grasping_object" in line:
                grasp_check_line = i + 1
                break

        stats.check(grasp_check_line is not None,
                    "fetch_from_user: found check_grasping_object in source")

        if grasp_check_line:
            post_grasp_failures = 0
            post_grasp_with_recovery = 0
            for line_num in return_false_in_run:
                if line_num > grasp_check_line:
                    post_grasp_failures += 1
                    start = max(0, line_num - 15)
                    preceding = "\n".join(lines[start:line_num])
                    if "safe_release" in preceding:
                        post_grasp_with_recovery += 1
            stats.check(post_grasp_failures == post_grasp_with_recovery,
                        f"fetch_from_user: all {post_grasp_failures} post-grasp "
                        f"return False in run() have safe_release ({post_grasp_with_recovery} verified)")
            stats.check(post_grasp_failures == 4,
                        f"fetch_from_user: 4 post-grasp return False in run() (got {post_grasp_failures})")


# ===================================================================
# Scenarios command
# ===================================================================
SCENARIOS = {
    "hand":     (test_hand_logic,             "HandClient motor-state logic (mock)"),
    "grasp":    (test_verify_grasp,            "Critic verify_grasp (mock)"),
    "dest":     (test_verify_destination,      "Critic verify_destination_action (mock)"),
    "goal":     (test_verify_goal,             "Critic verify_goal end-to-end (mock)"),
    "recovery": (test_safe_release,            "Skill.safe_release recovery method (mock)"),
    "retry":    (test_retry_tracking,          "Retry/recovery result tracking (mock)"),
    "fetch":    (test_fetch_from_user_recovery,"fetch_from_user safe_release on failures"),
    "edge":     (None,                         "Combined edge-case tests (hand+grasp+dest)"),
    "all":      (None,                         "Run all test scenarios"),
}


def main():
    parser = argparse.ArgumentParser(description="Test critic and verification logic")
    parser.add_argument("--scenario", "-s", choices=list(SCENARIOS.keys()),
                        default="all", help="Which scenario to run")
    parser.add_argument("--list", "-l", action="store_true",
                        help="List available scenarios")
    parser.add_argument("--real", action="store_true",
                        help="Run against real hardware (requires arm/hand servers)")
    args = parser.parse_args()

    if args.list:
        print(f"\n{BOLD}Available scenarios:{RESET}\n")
        for name, (fn, desc) in SCENARIOS.items():
            print(f"  {CYAN}{name:<10}{RESET} {desc}")
        print()
        return 0

    if args.real:
        print(f"{RED}Real-hardware mode not implemented. Use mock mode.{RESET}")
        return 1

    stats = TestStats()

    if args.scenario == "all":
        test_hand_logic(stats)
        test_verify_grasp(stats)
        test_verify_destination(stats)
        test_verify_goal(stats)
        test_safe_release(stats)
        test_retry_tracking(stats)
        test_fetch_from_user_recovery(stats)
    elif args.scenario == "edge":
        # Edge cases: run hand + grasp + recovery
        test_hand_logic(stats)
        test_verify_grasp(stats)
        test_safe_release(stats)
    else:
        fn, desc = SCENARIOS[args.scenario]
        print(f"\n{CYAN}Running: {desc}{RESET}")
        fn(stats)

    # Clean up test memory files
    import shutil
    shutil.rmtree("/tmp/test_critic_memory", ignore_errors=True)

    all_ok = stats.summary()
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
