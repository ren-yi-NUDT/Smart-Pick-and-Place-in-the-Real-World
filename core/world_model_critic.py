#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
World-model critic for task-scoped world memory.

This module checks whether the next robot action is allowed according to
the current world memory state, and records critic / verification results.

It does not connect to hardware.
"""

import time

class WorldModelCritic:
    """
    Critic working on a WorldMemory object.

    Main responsibilities:
    1. Check preconditions before grasp.
    2. Verify grasp result and update robot holding state.
    3. Check preconditions before destination action.
    4. Verify destination action result and update memory.
    5. Verify final task goal.
    """

    def __init__(self, memory):
        self.memory = memory

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _record_and_return_critic(self, record):
        record.setdefault("time", time.time())
        self.memory.record_critic(record)
        return record

    def _record_and_return_verification(self, record):
        record.setdefault("time", time.time())
        self.memory.record_verification(record)
        return record

    def _target_object(self):
        return self.memory.command.get("object")

    def _target_container(self):
        return self.memory.command.get("container")

    def _mode(self):
        return self.memory.data.get("mode", "normal_place")

    def _holding(self):
        return self.memory.data.get("robot_state", {}).get("holding")

    def _safe_int(self, value, default=0):
        """
        Convert value to int safely.
        """
        try:
            return int(value)
        except Exception:
            return default

    def _count_failures_from_memory(self, memory):
        """
        Count failures and recovered failures from world memory.

        Important:
        - failure_records is treated as the primary source of failures.
        - recovery actions are treated as resolved failures.
        - Do NOT double-count failed actions/events/history if failure_records
          already exists, otherwise retry success may still fail final goal check.

        Returns:
            tuple: (failure_count, resolved_failures)
        """
        if not isinstance(memory, dict):
            return 0, 0

        # ------------------------------------------------------------
        # 1. Count failures
        # ------------------------------------------------------------
        failure_records = memory.get("failure_records", [])

        if isinstance(failure_records, list):
            failure_count = len(failure_records)
        else:
            failure_count = 0

        # Backward compatibility:
        # If old memory has no failure_records, derive failures from
        # actions/events/history result.success == False.
        if failure_count == 0:
            for key in ["actions", "events", "history"]:
                records = memory.get(key)

                if not isinstance(records, list):
                    continue

                for record in records:
                    if not isinstance(record, dict):
                        continue

                    # Do not count recovery records as failures.
                    if record.get("type") == "recovery":
                        continue

                    result = record.get("result", {})

                    if isinstance(result, dict) and result.get("success") is False:
                        failure_count += 1

        # ------------------------------------------------------------
        # 2. Count resolved failures from recovery records
        # ------------------------------------------------------------
        resolved_failures = 0

        for key in ["actions", "events", "history"]:
            records = memory.get(key)

            if not isinstance(records, list):
                continue

            for record in records:
                if not isinstance(record, dict):
                    continue

                if record.get("type") != "recovery":
                    continue

                result = record.get("result", {})

                if not isinstance(result, dict):
                    continue

                resolved_failures += self._safe_int(
                    result.get("resolved_failures", 0),
                    default=0,
                )

        # Do not allow resolved_failures to exceed failure_count.
        resolved_failures = min(resolved_failures, failure_count)

        return failure_count, resolved_failures

    def _has_unresolved_failure(self):
        return len(self.memory.data.get("failure_records", [])) > 0

    # ------------------------------------------------------------------
    # Before grasp
    # ------------------------------------------------------------------

    def before_grasp(self):
        """
        Check whether grasp is allowed.

        Required:
        - command.object exists
        - robot is not already holding something

        It does not require object_index to contain the target object yet,
        because perception may happen inside the grasp skill.
        """
        target = self._target_object()
        holding = self._holding()

        missing = []
        risks = []

        if not target:
            missing.append("missing target object in command")

        if holding is not None:
            risks.append("robot already holding object")

        approved = len(missing) == 0 and len(risks) == 0

        record = {
            "stage": "before_grasp",
            "approved": approved,
            "target_object": target,
            "robot_state": {
                "holding": holding,
            },
            "missing": missing,
            "risks": risks,
            "next_action": "run_grasp" if approved else "stop_or_reobserve",
        }

        return self._record_and_return_critic(record)

    # ------------------------------------------------------------------
    # Verify grasp
    # ------------------------------------------------------------------

    def verify_grasp(self, grasp_result):
        """
        Verify grasp result with real-world evidence.

        Required evidence in grasp_result:
            - "hand_closed": True (hand close command was sent)
            - "finger_deviation": float > 0 (sum of finger deviations from
              fully-closed position — proves something is between the fingers)
            - "object": str (target object name)

        Without real evidence, verification is REJECTED even if
        grasp_result["success"] is True.

        If verified, robot_state.holding is set to the object name.
        """
        grasp_result = grasp_result or {}

        target = self._target_object()
        object_id = (
            grasp_result.get("object_id")
            or grasp_result.get("object")
            or target
        )

        hand_closed = bool(grasp_result.get("hand_closed", False))
        finger_deviation = float(grasp_result.get("finger_deviation", 0.0))
        trajectory_ok = bool(grasp_result.get("success", False))

        # Real verification: hand must have closed AND fingers must
        # deviate from the fully-closed position (object between fingers).
        # A deviation < 5 means fingers are essentially fully closed —
        # nothing was grasped.
        GRASP_DEVIATION_THRESHOLD = 5.0
        evidence_holding = hand_closed and (finger_deviation > GRASP_DEVIATION_THRESHOLD)

        success = trajectory_ok and evidence_holding

        if trajectory_ok and not evidence_holding:
            # Trajectory executed but hand didn't close on anything —
            # this is the exact rubber-stamp case we must catch.
            reason = (
                f"hand_closed={hand_closed}, deviation={finger_deviation:.1f}"
                if hand_closed
                else "hand close command not confirmed"
            )
            self.memory.record_failure({
                "stage": "after_grasp",
                "reason": f"grasp trajectory executed but no object held: {reason}",
                "target_object": target,
                "grasp_result": grasp_result,
            })

        holding_after = object_id if success else None

        self.memory.add_observation({
            "source": "world_model_critic.verify_grasp",
            "robot_state": {
                "holding": holding_after,
            },
        })

        record = {
            "stage": "after_grasp",
            "success": success,
            "expected": {
                "robot_holding_target": True,
                "min_finger_deviation": GRASP_DEVIATION_THRESHOLD,
            },
            "observed": {
                "hand_closed": hand_closed,
                "finger_deviation": finger_deviation,
                "trajectory_ok": trajectory_ok,
                "evidence_holding": evidence_holding,
                "grasp_result": grasp_result,
            },
            "target_object": target,
            "verified_object": object_id if success else None,
            "memory_update_policy": "set_holding" if success else "do_not_set_holding",
            "next_action": "critic_before_destination" if success else "retry_or_reobserve",
        }

        self._record_and_return_verification(record)

        if not success:
            self.memory.record_failure({
                "stage": "after_grasp",
                "reason": "grasp not verified by real-world evidence",
                "target_object": target,
                "grasp_result": grasp_result,
            })

        return record

    # ------------------------------------------------------------------
    # Before destination action
    # ------------------------------------------------------------------

    def before_destination_action(self):
        """
        Check whether place / handover / trash / desk_place is allowed.

        Required:
        - robot must be holding something
        - for normal_place, command.container should exist

        Mode mapping is produced by core.world_memory.infer_task_mode().
        """
        mode = self._mode()
        holding = self._holding()
        container = self._target_container()

        missing = []
        risks = []

        if holding is None:
            missing.append("robot is not holding object")

        if mode == "normal_place":
            if not container:
                missing.append("missing destination container in command")

        elif mode == "handover":
            pass

        elif mode == "trash":
            pass

        elif mode == "desk_place":
            pass

        else:
            risks.append(f"unknown destination mode: {mode}")

        approved = len(missing) == 0 and len(risks) == 0

        record = {
            "stage": f"before_{mode}",
            "approved": approved,
            "mode": mode,
            "target_container": container,
            "robot_state": {
                "holding": holding,
            },
            "missing": missing,
            "risks": risks,
            "next_action": self._destination_skill_name(mode) if approved else "stop_or_recover",
        }

        return self._record_and_return_critic(record)

    def _destination_skill_name(self, mode):
        if mode == "handover":
            return "handover"
        if mode == "trash":
            return "trash"
        if mode == "desk_place":
            return "desk_place"
        return "place"

    # ------------------------------------------------------------------
    # Verify destination action
    # ------------------------------------------------------------------

    def verify_destination_action(self, action_result):
        """
        Verify destination action with real-world evidence.

        Required evidence in action_result:
            - "hand_opened": True (hand open command was executed)
            - For normal_place mode: trajectory must have succeeded

        If verified, robot_state.holding is cleared to None.
        """
        action_result = action_result or {}

        trajectory_ok = bool(action_result.get("success"))
        hand_opened = bool(action_result.get("hand_opened", False))
        mode = self._mode()
        holding_before = self._holding()
        container = self._target_container()

        # For all modes, hand must have opened to release the object
        evidence_released = hand_opened

        # For normal_place, also require trajectory success
        if mode == "normal_place":
            success = trajectory_ok and evidence_released
        else:
            success = evidence_released

        if not success and trajectory_ok and not evidence_released:
            self.memory.record_failure({
                "stage": "after_destination_action",
                "reason": "destination trajectory executed but hand open not confirmed",
                "mode": mode,
                "holding_before": holding_before,
                "target_container": container,
                "action_result": action_result,
            })

        if success:
            self.memory.add_observation({
                "source": "world_model_critic.verify_destination_action",
                "robot_state": {
                    "holding": None,
                },
                "relations": self._success_relations(
                    mode=mode,
                    obj=holding_before,
                    container=container,
                ),
            })

        holding_after = self._holding()

        record = {
            "stage": "after_destination_action",
            "success": success,
            "mode": mode,
            "expected": {
                "robot_released_object": True,
                "hand_opened": True,
            },
            "observed": {
                "hand_opened": hand_opened,
                "trajectory_ok": trajectory_ok,
                "evidence_released": evidence_released,
                "holding_before": holding_before,
                "holding_after": holding_after,
                "action_result": action_result,
            },
            "memory_update_policy": "clear_holding" if success else "do_not_clear_holding",
            "next_action": "verify_goal" if success else "replan_or_safe_recover",
        }

        self._record_and_return_verification(record)

        if not success:
            self.memory.record_failure({
                "stage": "after_destination_action",
                "reason": "destination action not verified by real-world evidence",
                "mode": mode,
                "holding_before": holding_before,
                "target_container": container,
                "action_result": action_result,
            })

        return record

    def _success_relations(self, mode, obj, container):
        """
        Add a simple symbolic relation after successful destination action.
        """
        if obj is None:
            return []

        if mode == "normal_place":
            relation = "in_or_on"
            target = container or "unknown_container"
        elif mode == "handover":
            relation = "handed_to"
            target = "person"
        elif mode == "trash":
            relation = "inside"
            target = "trash"
        elif mode == "desk_place":
            relation = "on"
            target = "desk"
        else:
            relation = "at"
            target = container or "unknown_destination"

        return [
            {
                "source": obj,
                "relation": relation,
                "target": target,
                "active": True,
            }
        ]

    # ------------------------------------------------------------------
    # Final goal verification
    # ------------------------------------------------------------------
    def verify_goal(self, payload=None):
        """
        Verify final pick-and-place goal.

        Goal success requires:
        - robot is not holding the object
        - destination action succeeded
        - no unresolved failures remain after retry recovery
        """
        if payload is None:
            payload = {}

        if not isinstance(payload, dict):
            payload = {}

        payload_memory = payload.get("memory", {})
        if not isinstance(payload_memory, dict):
            payload_memory = {}

        # Count failures from BOTH self.memory (recorded by critic during
        # verify_grasp / verify_destination_action) AND payload["memory"]
        # (passed by caller for external context).
        own_failure_count, own_resolved = self._count_failures_from_memory(
            self.memory.data
        )
        payload_failure_count, payload_resolved = self._count_failures_from_memory(
            payload_memory
        )

        failure_count = own_failure_count + payload_failure_count

        payload_resolved_failures = self._safe_int(
            payload.get("resolved_failures", 0),
            default=0,
        )

        resolved_failures = max(
            payload_resolved_failures,
            own_resolved + payload_resolved,
        )

        resolved_failures = min(resolved_failures, failure_count)
        unresolved_failures = max(0, failure_count - resolved_failures)

        destination_verification = payload.get("destination_verification", {})
        grasp_verification = payload.get("grasp_verification", {})

        destination_success = False
        grasp_success = False

        if isinstance(destination_verification, dict):
            destination_success = bool(destination_verification.get("success", False))

        if isinstance(grasp_verification, dict):
            grasp_success = bool(grasp_verification.get("success", False))

        # Fallback to raw results ONLY when the verification dict has no
        # "success" key at all (meaning verification was never performed).
        # If verification was performed and returned False, do NOT override
        # with raw result — raw result is untrusted without evidence.
        if isinstance(destination_verification, dict) and "success" not in destination_verification:
            destination_result = payload.get("destination_result", {})
            if isinstance(destination_result, dict):
                destination_success = bool(destination_result.get("success", False))

        if isinstance(grasp_verification, dict) and "success" not in grasp_verification:
            grasp_result = payload.get("grasp_result", {})
            if isinstance(grasp_result, dict):
                grasp_success = bool(grasp_result.get("success", False))

        # robot_not_holding is derived from destination verification evidence
        # (hand_opened + trajectory_ok), NOT from the raw destination_success
        # boolean. When both verifications passed with real evidence,
        # the robot has truly released the object.
        robot_not_holding = destination_success

        conditions = {
            "grasp_success": grasp_success,
            "destination_action_success": destination_success,
            "robot_not_holding": robot_not_holding,
            "no_unresolved_failures": unresolved_failures == 0,
        }

        success = all(conditions.values())

        return {
            "success": success,
            "failure_count": failure_count,
            "resolved_failures": resolved_failures,
            "unresolved_failures": unresolved_failures,
            "conditions": conditions,
            "object": payload.get("object"),
            "container": payload.get("container"),
            "mode": payload.get("mode"),
        }