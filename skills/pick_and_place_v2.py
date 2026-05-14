#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
High-level pick-and-place skill.

This file is intentionally a high-level orchestrator only.

Pipeline:

    input json
    -> setup world memory
    -> critic.before_grasp
    -> grasp / mock grasp
    -> critic.verify_grasp
    -> critic.before_destination_action
    -> place / handover / trash / desk_place / mock destination
    -> critic.verify_destination_action
    -> critic.verify_goal

No low-level robot, camera, perception, or motion logic should live here.
Those belong to:
    skills/grasp.py
    skills/place.py
    skills/handover.py
    skills/trash.py
    skills/desk_place.py

Mock mode:
    JSON:
        {"mock": true}
        {"dry_run": true}
        {"no_hardware": true}

    Environment:
        SKILL_MOCK=1
        SKILL_DRY_RUN=1
"""

import os

from termcolor import cprint

from skills.base import Skill, register_skill
from core.world_memory import setup_world_memory
from core.world_model_critic import WorldModelCritic

@register_skill("pick_and_place_v2")
class PickAndPlaceV2Skill(Skill):
    """
    High-level pick-and-place orchestrator.

    Example real mode:

        echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place

    Example mock mode:

        echo '{"object":"orange","container":"green bowl","mock":true}' | python run_skill.py pick_and_place

    Example modes:

        echo '{"object":"orange","container":"green bowl","mock":true}' | python run_skill.py pick_and_place
        echo '{"object":"bottle","container":"person","mock":true}' | python run_skill.py pick_and_place
        echo '{"object":"wrapper","container":"trash","mock":true}' | python run_skill.py pick_and_place
        echo '{"object":"cup","container":"desk","mock":true}' | python run_skill.py pick_and_place
    """

    def run(self, **kwargs):
        """
        Execute pick-and-place task.

        Expected input:
            {
                "object": "orange",
                "container": "green bowl",
                "mock": true
            }

        Returns:
            dict
        """

        # ------------------------------------------------------------------
        # 1. Read input
        # ------------------------------------------------------------------
        json_data = self._read_command(kwargs)

        if json_data is None:
            cprint("未收到有效的JSON输入", "red")
            return {
                "success": False,
                "skill": "pick_and_place",
                "stage": "input",
                "reason": "invalid_json",
            }

        cprint(
            f"=================== 1. Get JSON input: {json_data} ===================",
            "cyan",
        )

        obj = json_data.get("object")
        container = json_data.get("container")

        if obj is None or container is None:
            cprint("JSON输入缺少必需字段 object 或 container", "red")
            return {
                "success": False,
                "skill": "pick_and_place",
                "stage": "input",
                "reason": "missing_object_or_container",
                "input": json_data,
            }

        cprint(
            f"=================== 2. Parse input: Grasp {obj} and place it in/on/to {container} ===================",
            "cyan",
        )

        # ------------------------------------------------------------------
        # 2. Mock mode
        # ------------------------------------------------------------------
        mock_mode = self._is_mock_mode(json_data)
        self._mock_mode = mock_mode
        
        # Retry config.
        # max_xxx_retries means extra retries after the first attempt.
        self._max_grasp_retries = self._get_int_config(
            json_data,
            key="max_grasp_retries",
            default=0,
            minimum=0,
        )
        self._max_destination_retries = self._get_int_config(
            json_data,
            key="max_destination_retries",
            default=0,
            minimum=0,
        )
        
        # Mock success config.
        # Supports:
        #   "mock_grasp_success": true
        #   "mock_grasp_success": false
        #   "mock_grasp_success": [false, true]
        self._mock_grasp_success_sequence = self._get_bool_sequence_config(
            json_data,
            key="mock_grasp_success",
            default=None,
        )
        self._mock_destination_success_sequence = self._get_bool_sequence_config(
            json_data,
            key="mock_destination_success",
            default=None,
        )
        
        self._mock_grasp_success = self._get_bool_config(
            json_data,
            key="mock_grasp_success",
            default=True,
        )
        self._mock_destination_success = self._get_bool_config(
            json_data,
            key="mock_destination_success",
            default=True,
        )
        # ------------------------------------------------------------------
        # 3. Setup world memory
        # ------------------------------------------------------------------
        command = {
            "object": obj,
            "container": container,
            "source": json_data.get("source", "workspace"),
            "mock": mock_mode,
        }

        memory = setup_world_memory(command)
        critic = WorldModelCritic(memory)

        cprint(
            f"=================== 3. World memory initialized: {memory.memory_id} ===================",
            "cyan",
        )

        if mock_mode:
            cprint(
                "M=================== MOCK mode enabled: no hardware/simulation will be used ===================",
                "yellow",
            )

        # ------------------------------------------------------------------
        # 4. Critic before grasp
        # ------------------------------------------------------------------
        before_grasp = critic.before_grasp()

        if not before_grasp.get("approved", False):
            cprint("C=================== Critic blocked grasp ===================", "red")
            return {
                "success": False,
                "skill": "pick_and_place",
                "stage": "before_grasp",
                "memory_id": memory.memory_id,
                "mode": memory.data.get("mode"),
                "object": obj,
                "container": container,
                "critic": before_grasp,
                "memory_path": str(memory.path),
                "event_path": str(memory.event_path),
                "current_path": str(memory.current_path),
            }

        cprint(
            "C=================== Critic approved grasp ===================",
            "green",
        )

        # ------------------------------------------------------------------
        # 5. Run grasp with retry
        # ------------------------------------------------------------------
        grasp_attempts = []
        grasp_result = None
        grasp_verification = None
        
        grasp_failed_attempts = 0
        resolved_grasp_failures = 0
        
        max_grasp_attempts = 1 + max(0, getattr(self, "_max_grasp_retries", 0))
        
        for grasp_attempt in range(1, max_grasp_attempts + 1):
            self._current_grasp_attempt = grasp_attempt
        
            cprint(
                f"G=================== Grasp attempt {grasp_attempt}/{max_grasp_attempts} ===================",
                "cyan",
            )
        
            grasp_result = self._run_grasp(obj)
        
            memory.record_action({
                "type": "grasp",
                "attempt": grasp_attempt,
                "max_attempts": max_grasp_attempts,
                "input": {
                    "object": obj,
                    "mock": mock_mode,
                },
                "result": self._normalize_raw_result(grasp_result),
            })
        
            grasp_verification = critic.verify_grasp({
                "success": self._is_success(grasp_result),
                "skill": "grasp",
                "object_id": obj,
                "attempt": grasp_attempt,
                "max_attempts": max_grasp_attempts,
                "raw_result": self._normalize_raw_result(grasp_result),
            })
        
            grasp_attempt_record = {
                "attempt": grasp_attempt,
                "success": bool(grasp_verification.get("success", False)),
                "result": self._normalize_raw_result(grasp_result),
                "verification": grasp_verification,
            }
            grasp_attempts.append(grasp_attempt_record)
        
            if grasp_verification.get("success", False):
                cprint(
                    f"G=================== Grasp succeeded on attempt {grasp_attempt}/{max_grasp_attempts} ===================",
                    "green",
                )
                break
        
            if grasp_attempt < max_grasp_attempts:
                cprint(
                    f"R=================== Grasp failed, retrying {grasp_attempt + 1}/{max_grasp_attempts} ===================",
                    "yellow",
                )
            else:
                cprint(
                    f"G=================== Grasp failed after {max_grasp_attempts} attempt(s) ===================",
                    "red",
                )
        
        # After retry loop: if still failed, return failure.
        if not grasp_verification or not grasp_verification.get("success", False):
            return {
                "success": False,
                "skill": "pick_and_place",
                "stage": "grasp_failed",
                "memory_id": memory.memory_id,
                "mode": memory.data.get("mode"),
                "object": obj,
                "container": container,
                "mock": mock_mode,
                "max_grasp_attempts": max_grasp_attempts,
                "grasp_attempts": grasp_attempts,
                "grasp_failed_attempts": len(grasp_attempts),
                "resolved_grasp_failures": 0,
                "resolved_failures": 0,
                "grasp_result": self._normalize_raw_result(grasp_result),
                "grasp_verification": grasp_verification,
                "memory_path": str(memory.path),
                "event_path": str(memory.event_path),
                "current_path": str(memory.current_path),
            }
        
        # ------------------------------------------------------------------
        # 5.1 Record grasp recovery if retry eventually succeeded
        # ------------------------------------------------------------------
        grasp_failed_attempts = sum(
            1 for x in grasp_attempts
            if not x.get("success", False)
        )
        
        if grasp_failed_attempts > 0:
            resolved_grasp_failures = grasp_failed_attempts
        
            self._record_recovery(
                memory=memory,
                stage="grasp",
                failed_attempts=grasp_failed_attempts,
                succeeded_attempt=getattr(
                    self,
                    "_current_grasp_attempt",
                    len(grasp_attempts),
                ),
            )
        else:
            resolved_grasp_failures = 0
        
        cprint(
            "G=================== Successfully completed and verified grasp ===================",
            "green",
        )

        # ------------------------------------------------------------------
        # 6. Critic before destination action
        # ------------------------------------------------------------------
        before_dest = critic.before_destination_action()

        if not before_dest.get("approved", False):
            cprint("C=================== Critic blocked destination action ===================", "red")
            return {
                "success": False,
                "skill": "pick_and_place",
                "stage": "before_destination",
                "memory_id": memory.memory_id,
                "mode": memory.data.get("mode"),
                "object": obj,
                "container": container,
                "mock": mock_mode,
                "critic": before_dest,
                "memory_path": str(memory.path),
                "event_path": str(memory.event_path),
                "current_path": str(memory.current_path),
            }

        cprint(
            f"C=================== Critic approved destination action: {before_dest.get('next_action')} ===================",
            "green",
        )

# ------------------------------------------------------------------
        # 7. Run destination skill with retry
        # ------------------------------------------------------------------
        mode = memory.data.get("mode", "normal_place")

        destination_attempts = []
        dest_skill_name = self._destination_skill_name(mode)
        dest_result = None
        dest_verification = None

        destination_failed_attempts = 0
        resolved_destination_failures = 0

        max_destination_attempts = 1 + max(
            0,
            getattr(self, "_max_destination_retries", 0),
        )

        for destination_attempt in range(1, max_destination_attempts + 1):
            self._current_destination_attempt = destination_attempt

            cprint(
                f"{self._mode_prefix(mode)}=================== Destination attempt {destination_attempt}/{max_destination_attempts}: {dest_skill_name} ===================",
                "cyan",
            )

            dest_skill_name, dest_result = self._run_destination_skill(
                mode=mode,
                container=container,
            )

            memory.record_action({
                "type": dest_skill_name,
                "attempt": destination_attempt,
                "max_attempts": max_destination_attempts,
                "input": {
                    "container": container,
                    "mode": mode,
                    "mock": mock_mode,
                },
                "result": self._normalize_raw_result(dest_result),
            })

            dest_verification = critic.verify_destination_action({
                "success": self._is_success(dest_result),
                "skill": dest_skill_name,
                "container": container,
                "attempt": destination_attempt,
                "max_attempts": max_destination_attempts,
                "raw_result": self._normalize_raw_result(dest_result),
            })

            destination_attempt_record = {
                "attempt": destination_attempt,
                "success": bool(dest_verification.get("success", False)),
                "skill": dest_skill_name,
                "result": self._normalize_raw_result(dest_result),
                "verification": dest_verification,
            }
            destination_attempts.append(destination_attempt_record)

            if dest_verification.get("success", False):
                cprint(
                    f"{self._mode_prefix(mode)}=================== Destination succeeded on attempt {destination_attempt}/{max_destination_attempts}: {dest_skill_name} ===================",
                    "green",
                )
                break

            if destination_attempt < max_destination_attempts:
                cprint(
                    f"R=================== Destination failed, retrying {destination_attempt + 1}/{max_destination_attempts} ===================",
                    "yellow",
                )
            else:
                cprint(
                    f"{self._mode_prefix(mode)}=================== Destination failed after {max_destination_attempts} attempt(s): {dest_skill_name} ===================",
                    "red",
                )

        # After retry loop: if still failed, return failure.
        if not dest_verification or not dest_verification.get("success", False):
            return {
                "success": False,
                "skill": "pick_and_place",
                "stage": "destination_failed",
                "memory_id": memory.memory_id,
                "mode": mode,
                "object": obj,
                "container": container,
                "mock": mock_mode,
                "destination_skill": dest_skill_name,
                "max_destination_attempts": max_destination_attempts,
                "destination_attempts": destination_attempts,
                "destination_failed_attempts": sum(
                    1 for x in destination_attempts
                    if not x.get("success", False)
                ),
                "resolved_destination_failures": 0,
                "destination_result": self._normalize_raw_result(dest_result),
                "destination_verification": dest_verification,
                "resolved_failures": resolved_grasp_failures,
                "memory_path": str(memory.path),
                "event_path": str(memory.event_path),
                "current_path": str(memory.current_path),
            }

        # ------------------------------------------------------------------
        # 7.1 Record destination recovery if retry eventually succeeded
        # ------------------------------------------------------------------
        destination_failed_attempts = sum(
            1 for x in destination_attempts
            if not x.get("success", False)
        )

        if destination_failed_attempts > 0:
            resolved_destination_failures = destination_failed_attempts

            self._record_recovery(
                memory=memory,
                stage=mode,
                failed_attempts=destination_failed_attempts,
                succeeded_attempt=getattr(
                    self,
                    "_current_destination_attempt",
                    len(destination_attempts),
                ),
            )
        else:
            resolved_destination_failures = 0

        cprint(
            f"{self._mode_prefix(mode)}=================== Successfully completed and verified destination task: {dest_skill_name} ===================",
            "green",
        )

        # ------------------------------------------------------------------
        # 8. Final goal verification
        # ------------------------------------------------------------------
        resolved_failures = resolved_grasp_failures + resolved_destination_failures
        
        goal_payload = dict(memory.data)
        goal_payload.update({
            "object": obj,
            "container": container,
            "mode": mode,
            "mock": mock_mode,
            "memory": memory.data,
        
            "grasp_result": self._normalize_raw_result(grasp_result),
            "grasp_verification": grasp_verification,
            "grasp_attempts": grasp_attempts,
            "grasp_failed_attempts": grasp_failed_attempts,
            "resolved_grasp_failures": resolved_grasp_failures,
        
            "destination_skill": dest_skill_name,
            "destination_result": self._normalize_raw_result(dest_result),
            "destination_verification": dest_verification,
            "destination_attempts": destination_attempts,
            "destination_failed_attempts": destination_failed_attempts,
            "resolved_destination_failures": resolved_destination_failures,
        
            "resolved_failures": resolved_failures,
        })
        
        goal = critic.verify_goal(goal_payload)

        resolved_failures = resolved_grasp_failures + resolved_destination_failures
        
        result = {
            "success": bool(goal.get("success")),
            "skill": "pick_and_place",
            "stage": "completed" if goal.get("success") else "goal_not_verified",
            "memory_id": memory.memory_id,
            "mode": mode,
            "object": obj,
            "container": container,
            "mock": mock_mode,
        
            "max_grasp_attempts": max_grasp_attempts,
            "grasp_attempts": grasp_attempts,
            "grasp_failed_attempts": grasp_failed_attempts,
            "resolved_grasp_failures": resolved_grasp_failures,
            "grasp_result": self._normalize_raw_result(grasp_result),
            "grasp_verification": grasp_verification,
        
            "destination_skill": dest_skill_name,
            "max_destination_attempts": max_destination_attempts,
            "destination_attempts": destination_attempts,
            "destination_failed_attempts": destination_failed_attempts,
            "resolved_destination_failures": resolved_destination_failures,
            "destination_result": self._normalize_raw_result(dest_result),
            "destination_verification": dest_verification,
        
            "resolved_failures": resolved_failures,
            "goal": goal,
        
            "memory_path": str(memory.path),
            "event_path": str(memory.event_path),
            "current_path": str(memory.current_path),
        }

        if result["success"]:
            cprint(
                "P=================== Successfully completed the full pick-and-place task ===================",
                "green",
            )
        else:
            cprint(
                "P=================== Full pick-and-place task finished but goal was not verified ===================",
                "red",
            )

        return result

    # ------------------------------------------------------------------
    # Input
    # ------------------------------------------------------------------
    def _read_command(self, kwargs):
        """
        Prefer kwargs from run_skill.py stdin.
        Fallback to self.json_parser.get_command() for compatibility.
        """
        if kwargs:
            return dict(kwargs)

        try:
            return self.json_parser.get_command()
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Mock
    # ------------------------------------------------------------------
    def _is_mock_mode(self, json_data):
        """
        Enable hardware-free mock mode.

        Supported ways:
            JSON:
                {"mock": true}
                {"dry_run": true}
                {"no_hardware": true}

            Environment:
                SKILL_MOCK=1
                SKILL_DRY_RUN=1
        """
        if json_data is None:
            json_data = {}

        if bool(json_data.get("mock", False)):
            return True

        if bool(json_data.get("dry_run", False)):
            return True

        if bool(json_data.get("no_hardware", False)):
            return True

        if os.environ.get("SKILL_MOCK", "").lower() in ["1", "true", "yes", "y"]:
            return True

        if os.environ.get("SKILL_DRY_RUN", "").lower() in ["1", "true", "yes", "y"]:
            return True

        return False

    # ------------------------------------------------------------------
    # Grasp
    # ------------------------------------------------------------------
    def _run_grasp(self, obj):
        """
        Run grasp.

        In mock mode:
            no hardware or simulation is touched.

        In real mode:
            calls skills.grasp.GraspSkill.
        """
        cprint(
            f"G=================== Running grasp skill for object: {obj} ===================",
            "cyan",
        )

        if getattr(self, "_mock_mode", False):
            mock_success = self._get_mock_success_for_attempt(
                kind="grasp",
                default=True,
            )
        
            if mock_success:
                cprint(
                    f"M=================== MOCK grasp success: {obj} ===================",
                    "yellow",
                )
                return {
                    "success": True,
                    "skill": "grasp",
                    "mock": True,
                    "attempt": getattr(self, "_current_grasp_attempt", 1),
                    "object_id": obj,
                    "message": f"mock grasped {obj}",
                }
        
            cprint(
                f"M=================== MOCK grasp failure: {obj} ===================",
                "yellow",
            )
            return {
                "success": False,
                "skill": "grasp",
                "mock": True,
                "attempt": getattr(self, "_current_grasp_attempt", 1),
                "object_id": obj,
                "error_type": "MockGraspFailure",
                "error": f"mock failed to grasp {obj}",
            }
            
        try:
            from skills.grasp import GraspSkill

            skill = GraspSkill(
                config_path=self.config_path,
                save_path=self.save_path,
            )

            result = skill.run(object=obj)
            return self._wrap_skill_result("grasp", result)

        except Exception as e:
            cprint(
                f"G=================== Grasp skill exception: {type(e).__name__}: {e} ===================",
                "red",
            )
            return {
                "success": False,
                "skill": "grasp",
                "mock": False,
                "error_type": type(e).__name__,
                "error": str(e),
            }

    # ------------------------------------------------------------------
    # Destination
    # ------------------------------------------------------------------
    def _run_destination_skill(self, mode, container):
        """
        Dispatch destination skill according to world memory mode.

        Modes:
            normal_place -> PlaceSkill
            handover     -> HandoverSkill
            trash        -> TrashSkill
            desk_place   -> DeskPlaceSkill

        In mock mode:
            no hardware or simulation is touched.
        """
        skill_name = self._destination_skill_name(mode)

        if getattr(self, "_mock_mode", False):
            mock_success = self._get_mock_success_for_attempt(
                kind="destination",
                default=True,
            )
            if mock_success:
                cprint(
                    f"M=================== MOCK destination success: {skill_name}, container={container} ===================",
                    "yellow",
                )
                return skill_name, {
                    "success": True,
                    "skill": skill_name,
                    "mock": True,
                    "attempt": getattr(self, "_current_destination_attempt", 1),
                    "mode": mode,
                    "container": container,
                    "message": f"mock destination action {skill_name} completed",
                }
        
            cprint(
                f"M=================== MOCK destination failure: {skill_name}, container={container} ===================",
                "yellow",
            )
            return skill_name, {
                "success": False,
                "skill": skill_name,
                "mock": True,
                "attempt": getattr(self, "_current_destination_attempt", 1),
                "mode": mode,
                "container": container,
                "error_type": "MockDestinationFailure",
                "error": f"mock destination action {skill_name} failed",
            }

        try:
            if mode == "handover":
                cprint(
                    "H=================== Handover mode detected: delivering to person ===================",
                    "cyan",
                )

                from skills.handover import HandoverSkill

                skill = HandoverSkill(
                    config_path=self.config_path,
                    save_path=self.save_path,
                )

                result = skill.run()
                return "handover", self._wrap_skill_result("handover", result)

            if mode == "trash":
                cprint(
                    "T=================== Trash mode detected: throwing to trash ===================",
                    "cyan",
                )

                from skills.trash import TrashSkill

                skill = TrashSkill(
                    config_path=self.config_path,
                    save_path=self.save_path,
                )

                result = skill.run()
                return "trash", self._wrap_skill_result("trash", result)

            if mode == "desk_place":
                cprint(
                    "D=================== Desk placement mode detected: placing on desk ===================",
                    "cyan",
                )

                from skills.desk_place import DeskPlaceSkill

                skill = DeskPlaceSkill(
                    config_path=self.config_path,
                    save_path=self.save_path,
                )

                result = skill.run()
                return "desk_place", self._wrap_skill_result("desk_place", result)

            cprint(
                f"P=================== Normal placement mode detected: placing into/on {container} ===================",
                "cyan",
            )

            from skills.place import PlaceSkill

            skill = PlaceSkill(
                config_path=self.config_path,
                save_path=self.save_path,
            )

            result = skill.run(container=container)
            return "place", self._wrap_skill_result("place", result)

        except Exception as e:
            cprint(
                f"{self._mode_prefix(mode)}=================== Destination skill exception: {type(e).__name__}: {e} ===================",
                "red",
            )
            return skill_name, {
                "success": False,
                "skill": skill_name,
                "mock": False,
                "error_type": type(e).__name__,
                "error": str(e),
            }

    # ------------------------------------------------------------------
    # Result helpers
    # ------------------------------------------------------------------
    def _destination_skill_name(self, mode):
        """
        Convert world-memory mode to destination skill name.
        """
        if mode == "handover":
            return "handover"

        if mode == "trash":
            return "trash"

        if mode == "desk_place":
            return "desk_place"

        return "place"

    def _wrap_skill_result(self, skill_name, result):
        """
        Wrap bool/dict atomic skill result into a structured dict.
        """
        if isinstance(result, dict):
            result.setdefault("skill", skill_name)
            result.setdefault("success", bool(result.get("success", False)))
            return result

        return {
            "success": bool(result),
            "skill": skill_name,
            "raw": result,
        }

    def _normalize_raw_result(self, result):
        """
        Make result JSON serializable and consistent.

        This avoids AttributeError and also avoids json.dumps failures
        when a low-level skill returns a non-serializable object.
        """
        if isinstance(result, dict):
            return result

        if isinstance(result, list):
            return [self._normalize_raw_result(x) for x in result]

        if isinstance(result, tuple):
            return [self._normalize_raw_result(x) for x in result]

        if isinstance(result, (str, int, float, bool)) or result is None:
            return result

        return str(result)

    def _is_success(self, result):
        """
        Normalize skill result into bool.
        """
        if isinstance(result, dict):
            return bool(result.get("success", False))

        return bool(result)

    def _mode_prefix(self, mode):
        """
        Pretty log prefix.
        """
        if mode == "handover":
            return "H"

        if mode == "trash":
            return "T"

        if mode == "desk_place":
            return "D"

        return "P"
    def _get_bool_config(self, json_data, key, default=False):
        """
        Read bool config from JSON.
    
        Supports:
            true / false
            "true" / "false"
            "1" / "0"
            "yes" / "no"
            "y" / "n"
    
        If value is list, return default.
        Lists are handled by _get_bool_sequence_config().
        """
        if json_data is None:
            return default
    
        if key not in json_data:
            return default
    
        value = json_data.get(key)
    
        if isinstance(value, list):
            return default
    
        return self._to_bool(value, default=default)
    def _get_int_config(self, json_data, key, default=0, minimum=None, maximum=None):
        """
        Read int config from JSON.
    
        Example:
            {"max_grasp_retries": 2}
            {"max_destination_retries": "2"}
        """
        if json_data is None:
            return default
    
        if key not in json_data:
            return default
    
        value = json_data.get(key)
    
        try:
            value = int(value)
        except Exception:
            return default
    
        if minimum is not None:
            value = max(minimum, value)
    
        if maximum is not None:
            value = min(maximum, value)
    
        return value
    
    def _get_bool_sequence_config(self, json_data, key, default=None):
        """
        Read a bool sequence from JSON.
    
        Supports:
            {"mock_grasp_success": [false, true]}
            {"mock_destination_success": [false, false, true]}
    
        If the value is not a list, return default.
        """
        if json_data is None:
            return default
    
        if key not in json_data:
            return default
    
        value = json_data.get(key)
    
        if not isinstance(value, list):
            return default
    
        return [self._to_bool(x, default=True) for x in value]
    
    def _to_bool(self, value, default=False):
        """
        Convert common bool-like values to bool.
        """
        if isinstance(value, bool):
            return value
    
        if isinstance(value, str):
            text = value.strip().lower()
    
            if text in ["1", "true", "yes", "y", "on"]:
                return True
    
            if text in ["0", "false", "no", "n", "off"]:
                return False
    
            return default
    
        if value is None:
            return default
    
        return bool(value)
    
    def _get_mock_success_for_attempt(self, kind, default=True):
        """
        Return mock success/failure for current attempt.
    
        kind:
            "grasp"
            "destination"
    
        If sequence exists:
            attempt 1 uses sequence[0]
            attempt 2 uses sequence[1]
            ...
            attempts beyond sequence length reuse the last value.
    
        If sequence does not exist:
            use fixed bool config.
        """
        if kind == "grasp":
            sequence = getattr(self, "_mock_grasp_success_sequence", None)
            fixed_value = getattr(self, "_mock_grasp_success", default)
            attempt = getattr(self, "_current_grasp_attempt", 1)
    
        elif kind == "destination":
            sequence = getattr(self, "_mock_destination_success_sequence", None)
            fixed_value = getattr(self, "_mock_destination_success", default)
            attempt = getattr(self, "_current_destination_attempt", 1)
    
        else:
            return default
    
        if sequence:
            index = max(0, attempt - 1)
            index = min(index, len(sequence) - 1)
            return bool(sequence[index])
    
        return bool(fixed_value)
    def _record_recovery(self, memory, stage, failed_attempts, succeeded_attempt):
        """
        Record that previous failed attempts were recovered by a later successful retry.
    
        This is important because the final goal checker may reject a task if it sees
        unresolved failures in world memory. A retry success should resolve previous
        failures in the same stage.
        """
        failed_attempts = int(failed_attempts or 0)
    
        if failed_attempts <= 0:
            return None
    
        recovery_event = {
            "type": "recovery",
            "input": {
                "stage": stage,
                "failed_attempts": failed_attempts,
                "succeeded_attempt": succeeded_attempt,
            },
            "result": {
                "success": True,
                "stage": stage,
                "resolved_failures": failed_attempts,
                "recovered_by_attempt": succeeded_attempt,
                "message": (
                    f"Recovered {failed_attempts} failed attempt(s) "
                    f"in stage '{stage}' by attempt {succeeded_attempt}"
                ),
            },
        }
    
        memory.record_action(recovery_event)
        return recovery_event