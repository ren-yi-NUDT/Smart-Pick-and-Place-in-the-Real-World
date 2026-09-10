#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""水果由左臂到右臂递人。

桌面整理一键通技能。

一条命令完成 CMLLR 标准桌面整理流程：
  水果由左臂到右臂递人 → 纸团直扔 → 抽屉支线（红甜椒出抽递人 / 蔬菜入抽）→ 擦桌子。

设计要点（2026-09-09 与 CMLLR 对齐的提速方案 #5）：
  * 侦察驱动：开局对左相机画面跑一次 YOLO 全词表检测，分类成
    水果/蔬菜/纸团三张计划表；后续抓取只遍历"计划里真实存在"的词，
    不做盲词扫描（提速核心之一）。
  * 水果由左臂到右臂递人：左臂抓取后经纯位姿双臂交接，再由右臂使用
    ``recorded_poses/right.json`` 中最新版 ``handover_pose`` 递给用户。
  * 纸团连发：纸团逐个 ``检测 → 抓取 → 验爪 → throw_to_trash_pose``，
    每次投掷后重新检测桌面，确认无纸团才进入下一阶段。
  * 身份校验门：纸团咬合深度应落在松软物区间（默认 [150, 235]）；
    若 pos < 150 说明夹到的可能是甜椒/水果等致密物——**拒绝投掷**，
    保持夹持并中止全流程交人工处理（防"甜椒进垃圾桶"事故复发）。
  * 水果/蔬菜空爪门：验爪 pos > 200 或 object_detected=False 视为
    假抓取，立即中止全流程（宁停勿错）。

抽屉支线：右臂开抽屉 → 尝试视觉抓取红甜椒并直接递人（仅右臂，遵守
抽屉取物规则）→ 左臂抓蔬菜 → 双臂交接 → drawer_1_placement 入抽 →
关抽屉。红甜椒不在/识别失败时报告并跳过，不阻塞主流程。

擦桌子：直接复用 ``wipe_table`` 技能（固定抓取后验爪），本技能
不做任何海绵相关的前置动作。

CLI 示例：
    python3 run_skill.py desk_cleanup --plan-only 语义见 {"plan_only": true}
    echo '{"plan_only": true}' | python3 run_skill.py desk_cleanup   # 只侦察+打印计划，不动臂
    echo '{}' | python run_skill.py desk_cleanup                     # 完整流程
    echo '{"wipe": false}' | python run_skill.py desk_cleanup        # 不擦桌
    echo '{"take_red_pepper": false}' | python run_skill.py desk_cleanup  # 跳过抽屉红甜椒
"""

import time

from termcolor import cprint

from core.skill_runtime import SkillResult
from skills.base import Skill, register_skill


@register_skill("desk_cleanup")
class DeskCleanupSkill(Skill):
    """侦察分类 → 水果左交右递人 → 纸团直扔 → 抽屉支线 → 擦桌子。"""

    # 侦察词表（类别 → 候选词）。实际执行只遍历侦察命中的词。
    FRUIT_WORDS = ("apple", "orange", "lemon", "tomato", "banana",
                   "pear", "peach", "kiwi", "mango", "grape")
    VEG_WORDS = ("green pepper", "bell pepper", "cucumber",
                 "eggplant", "carrot")
    PAPER_WORDS = ("crumpled paper",)

    RED_PEPPER_WORDS = ("red pepper", "bell pepper")

    # 安全校验阈值（软合复检咬合深度 pos，0-255）
    EMPTY_GRASP_POS = 200      # 水果/蔬菜：> 视为空爪/假抓取
    PAPER_POS_MIN = 150        # 纸团身份下限；低于此值怀疑夹到致密物
    PAPER_POS_MAX = 225        # 纸团上限：>225 接近空合（实测空合≈227-230）

    RECON_CONF = 0.2           # 侦察置信度（与离线探针一致）
    IOU_DEDUPE = 0.5           # 多词同框去重阈值

    def validate_inputs(self, **kwargs):
        max_fruits = int(kwargs.get("max_fruits", 6))
        max_papers = int(kwargs.get("max_papers", 5))
        max_veg = int(kwargs.get("max_veg", 4))
        if min(max_fruits, max_papers, max_veg) < 1:
            raise ValueError("max_fruits/max_papers/max_veg 必须 >= 1")

    # ------------------------------------------------------------------
    # 侦察与分类
    # ------------------------------------------------------------------

    @staticmethod
    def _iou(a, b):
        ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
        ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
        area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0

    def _recon(self):
        """左相机一帧 + 全词表 YOLO 检测，输出分类计划表。

        返回 dict: {"fruits": [(word, conf)...], "vegs": [...],
                    "papers": int, "image_path": str}
        """
        cprint("[desk_cleanup] 阶段1: 左相机侦察 + YOLO 分类", "cyan")
        vocabulary = list(self.FRUIT_WORDS + self.VEG_WORDS + self.PAPER_WORDS)
        rgb, _depth = self.get_camera_obs("left")
        detections = self.perception.detect_objects(
            rgb, vocabulary, conf=self.RECON_CONF
        )

        # 保存侦察帧，方便事后追溯
        image_path = ""
        try:
            import cv2
            import time
            image_path = self._save_recon_image(rgb)
            cprint(f"[desk_cleanup] 侦察帧已保存: {image_path}", "dim")
        except Exception:
            pass

        # 整理检测: [[x1,y1,x2,y2,conf,cls], ...]
        items = []
        for x1, y1, x2, y2, conf, cls in detections:
            cls = int(cls)
            if cls >= len(vocabulary):
                continue
            items.append({
                "word": vocabulary[cls],
                "box": (float(x1), float(y1), float(x2), float(y2)),
                "conf": float(conf),
            })

        # 多词同框去重：IoU 超阈值保留置信度更高者
        items.sort(key=lambda d: d["conf"], reverse=True)
        kept = []
        for item in items:
            if all(self._iou(item["box"], k["box"]) < self.IOU_DEDUPE
                   for k in kept):
                kept.append(item)

        fruits = [(d["word"], d["conf"]) for d in kept
                  if d["word"] in self.FRUIT_WORDS]
        vegs = [(d["word"], d["conf"]) for d in kept
                if d["word"] in self.VEG_WORDS]
        papers = sum(1 for d in kept if d["word"] in self.PAPER_WORDS)

        plan = {
            "fruits": fruits,
            "vegs": vegs,
            "papers": papers,
            "image_path": image_path,
        }
        cprint(
            "[desk_cleanup] 计划: 水果={} 蔬菜={} 纸团≈{}".format(
                fruits or "无", vegs or "无", papers or "0(可能漏检)",
            ),
            "yellow",
        )
        return plan

    def _save_recon_image(self, rgb):
        import cv2
        import os
        import time
        path = os.path.join(
            self.save_path,
            "desk_cleanup_recon_%s.jpg" % time.strftime("%H%M%S"),
        )
        cv2.imwrite(path, rgb)
        return path

    # ------------------------------------------------------------------
    # 验爪辅助
    # ------------------------------------------------------------------

    def _grip_position(self, side="left", passive=True):
        """防滑验爪读数（全流程统一，16:41 CMLLR 指令），
        返回 (object_detected, position)。

        默认纯被动 get_state()：只读当前咬合位置，不碰爪、
        不改保持力（软合验爪会把保持力降到 0，光滑物有滑脱风险）。
        判断规则：空合 = 机械限位 pos≈227-230 / gOBJ=at requested；
        pos<227 视为有物。软合模式保留作备用来路（不推荐）。
        """
        gripper = self.gripper_for(side)
        if passive:
            import re
            state = gripper.get_state()
            info = state.get("info", "") if isinstance(state, dict) else ""
            match = re.search(r"pos=(\d+)/255", info)
            pos = int(match.group(1)) if match else 255
            detected = pos < 227 and "at requested" not in info
            return detected, pos
        response = gripper.close(force=20, soft=True)
        if isinstance(response, dict):
            return (
                bool(response.get("object_detected", False)),
                int(response.get("position", 255)),
            )
        return False, 255

    # ------------------------------------------------------------------
    # VLM 照片复检（跳过/收尾前的二次确认，16:53 CMLLR 指令）
    # ------------------------------------------------------------------

    def _vlm_present(self, question, side="left"):
        """拍一帧照片让 VLM 回答“有/没有”问题。

        返回 True=VLM 说有；False=没有 或 VLM 出错（按不在处理，
        不阻塞流程）。帧会存档到 save_path 供事后追溯。
        """
        try:
            from core.vlm import VLMClient
            if getattr(self, "_vlm_client", None) is None:
                self._vlm_client = VLMClient()
            rgb, _depth = self.get_camera_obs(side)
            try:
                import cv2
                import os
                import time
                path = os.path.join(
                    self.save_path,
                    "vlm_check_%s.jpg" % time.strftime("%H%M%S"),
                )
                cv2.imwrite(path, rgb)
            except Exception:
                pass
            prompt = (
                "这张照片是机械臂视角拍摄的桌面/抽屉。"
                f"请只回答两个字：'有' 或 '没有'。问题：{question}"
            )
            answer = self._vlm_client.analyze(
                rgb, prompt=prompt, max_tokens=8,
            )
            if not answer:
                cprint("[desk_cleanup] VLM 复检无响应（按不在处理）", "yellow")
                return False
            text = str(answer).strip()
            cprint(f"[desk_cleanup] VLM 复检: {question} → {text}", "cyan")
            if "没有" in text or "不在" in text:
                return False
            if "有" in text:
                return True
            return False
        except Exception as exc:
            cprint(f"[desk_cleanup] VLM 复检出错（按不在处理）: {exc}", "yellow")
            return False

    # ------------------------------------------------------------------
    # 各类别执行
    # ------------------------------------------------------------------

    def _run_fruits(self, fruit_words, max_fruits):
        """水果：左臂抓 → 双臂位姿交接(1倍速) → 验右爪 → 右臂命名位姿递人。"""
        delivered = 0
        for word, _conf in fruit_words[:max_fruits]:
            cprint(f"[desk_cleanup] 水果: 抓取 {word}（左臂）", "cyan")
            if not self.visual_grasp(
                word, side="left", use_vlm_grounding=False,
            ):
                # 跳过前 VLM 复检照片：真不在才跳过
                if not self._vlm_present(
                    f"桌上是否还有{word}（水果）？"
                ):
                    cprint(
                        f"[desk_cleanup] VLM 确认 {word} 不在桌上，跳过",
                        "yellow",
                    )
                    continue
                cprint(
                    f"[desk_cleanup] VLM 仍见 {word}，启用 grounding 重试一次",
                    "yellow",
                )
                if not self.visual_grasp(
                    word, side="left", use_vlm_grounding=True,
                ):
                    cprint(f"[desk_cleanup] 水果 {word} 重试仍失败，跳过", "yellow")
                    continue

            detected, pos = self._grip_position("left")
            if not detected or pos > self.EMPTY_GRASP_POS:
                return delivered, False, (
                    f"水果 {word} 验爪异常（detected={detected}, pos={pos}），"
                    "按空爪/假抓取中止全流程"
                )

            # 双臂交接只复现命名位姿，不回放轨迹；水果按 1 倍速执行。
            if not self.handover_pipeline.run(
                mode="dual", speed=1, require_confirmation=False,
                direction="left_to_right",
            ):
                return delivered, False, (
                    f"水果 {word} 双臂交接失败，中止全流程"
                    "（物体状态未知，请人工接管）"
                )

            # 交接后必验右爪（原版 SOP：拦截空爪，不空递）
            r_detected, r_pos = self._grip_position("right")
            if not r_detected or r_pos > self.EMPTY_GRASP_POS:
                return delivered, False, (
                    f"水果 {word} 交接后右爪验爪异常"
                    f"（detected={r_detected}, pos={r_pos}），"
                    "不空递，中止全流程"
                )

            # 动态读取 right.json 的最新版 handover_pose，不使用轨迹文件。
            if not self._deliver_right_direct():
                return delivered, False, (
                    f"水果 {word} 已交接但右臂递送失败，中止全流程"
                    "（物体仍在右爪，请人工接管）"
                )
            delivered += 1
            cprint(f"[desk_cleanup] 水果由左臂到右臂递人完成 {delivered} 件", "green")
        return delivered, True, ""

    def _deliver_right_direct(self, speed=15, release_wait=2.0, home_speed=30):
        """右臂直接平滑递送：handover_pose → 开爪 → 停 → home。

        不经 Skill.control_arm 的相邻转换检查，避免
        desk_front → home 快速绕行（光滑物在绕行段滑脱的实锤场景）。
        与 14:42 手动 pose_execute play 序列语义一致。
        """
        from skills.pose_execute import _load_poses
        side = "right"
        arm = self.context.arm(side)

        def _joint_dict(name):
            pose = _load_poses(side).get(name)
            if not pose:
                cprint(f"[desk_cleanup] recorded_poses 中没有 {name}({side})", "red")
                return None
            return {
                f"J{i + 1}": float(v)
                for i, v in enumerate(pose["joint_angles_deg"])
            }

        handover = _joint_dict("handover_pose")
        if handover is None:
            return False
        if not arm.move_to_named_pose(handover, speed=speed):
            return False
        self.context.gripper(side).open(speed=10)
        time.sleep(max(0.0, float(release_wait)))
        home = _joint_dict("home")
        if home is None:
            return False
        return arm.move_to_named_pose(home, speed=home_speed)

    def _paper_count_on_desk(self):
        """重新拍摄桌面并返回当前检出的纸团数量。"""
        rgb, _depth = self.get_camera_obs("left")
        return len(self.perception.detect_objects(
            rgb, list(self.PAPER_WORDS), conf=self.RECON_CONF,
        ))

    def _run_papers(self, _initial_paper_count, max_papers):
        """逐个投掷纸团，复检确认桌面无纸团后才结束。"""
        thrown = 0
        while True:
            remaining = self._paper_count_on_desk()
            vlm_override = False
            if remaining == 0:
                # 收尾前 VLM 复检照片：确认真没有纸团才进下一阶段
                if not self._vlm_present("桌面上有没有揉皱的纸团？"):
                    cprint(
                        "[desk_cleanup] 纸团复检: YOLO 与 VLM 均确认桌面无纸团",
                        "cyan",
                    )
                    return thrown, True, ""
                cprint(
                    "[desk_cleanup] YOLO 未检出但 VLM 复检仍有纸团，"
                    "启用 VLM grounding 抓取",
                    "yellow",
                )
                vlm_override = True
            if thrown >= max_papers:
                return thrown, False, (
                    f"已投掷 {thrown} 个纸团并达到 max_papers={max_papers}，"
                    f"桌面仍检出 {remaining} 个纸团；停止进入下一阶段"
                )

            cprint(f"[desk_cleanup] 纸团复检: 桌面仍有约 {remaining or 'VLM'} 个", "cyan")
            if not self.visual_grasp(
                "crumpled paper", side="left",
                use_vlm_grounding=vlm_override,
            ):
                return thrown, False, (
                    "桌面仍检出纸团但抓取失败；停止进入下一阶段"
                )

            detected, pos = self._grip_position("left")
            if not detected:
                return thrown, False, (
                    f"纸团验爪异常（detected=False, pos={pos}），"
                    "中止全流程（物体仍在左爪，请人工接管）"
                )
            # 单一窗口门：pos<150 致密物（甜椒/水果），pos>225 空合
            if pos < self.PAPER_POS_MIN or pos > self.PAPER_POS_MAX:
                # 身份存疑：夹到的东西密度不像纸团——宁可不扔！
                return thrown, False, (
                    f"身份校验失败：标称纸团但咬合 pos={pos} 超出纸团区间 "
                    f"[{self.PAPER_POS_MIN}, {self.PAPER_POS_MAX}]，"
                    "疑似甜椒/水果等致密物或空爪。已保持夹持并中止，"
                    "请人工确认处理"
                )

            if not self.place_pipeline.place_at_named_pose(
                "throw_to_trash_pose", side="left", release_wait=1.5,
            ):
                return thrown, False, "纸团投掷动作失败，中止全流程"
            thrown += 1
            cprint(f"[desk_cleanup] 纸团已投掷 {thrown} 个", "green")

    def _run_drawer(self, veg_words, max_veg, take_red_pepper):
        """抽屉支线：开抽 → 红甜椒出抽递人（右臂）→ 蔬菜入抽（左臂链路）→ 关抽。"""
        red_pepper = "skipped"
        veg_in = 0

        cprint("[desk_cleanup] 阶段4: 右臂开抽屉", "cyan")
        if not self.drawer_pipeline.open():
            return red_pepper, veg_in, False, "抽屉打开失败，中止全流程"

        if take_red_pepper:
            cprint("[desk_cleanup] 抽屉: 尝试取出红甜椒（仅右臂视觉，防滑验爪）", "cyan")
            # 防滑配方（16:38 CMLLR 指令恢复）：hold_after_grasp 实夹
            # force=80 停在抓取位，被动读数验爪（不碰爪不掉力），
            # 直接平滑递送不经 home 绕行
            grasp_ok = self.visual_grasp(
                self.RED_PEPPER_WORDS[0], side="right",
                use_vlm_grounding=False, hold_after_grasp=True,
            )
            if not grasp_ok:
                # 跳过前 VLM 复检抽屉照片：真不在才跳过
                if self._vlm_present(
                    "抽屉里是否有红甜椒（红色甜椒）？", side="right",
                ):
                    cprint(
                        "[desk_cleanup] YOLO 未识别但 VLM 仍见红甜椒，"
                        "启用 grounding 重试一次",
                        "yellow",
                    )
                    grasp_ok = self.visual_grasp(
                        self.RED_PEPPER_WORDS[0], side="right",
                        use_vlm_grounding=True, hold_after_grasp=True,
                    )
            if grasp_ok:
                # 被动验爪：纯 get_state 读数，保持力不动
                detected, pos = self._grip_position("right", passive=True)
                if detected and pos <= self.EMPTY_GRASP_POS:
                    # 直接平滑递送（不经 home 绕行，防滑脱实锤场景）
                    if self._deliver_right_direct():
                        red_pepper = "delivered"
                        cprint("[desk_cleanup] 红甜椒已出抽递人", "green")
                    else:
                        return red_pepper, veg_in, False, (
                            "红甜椒已抓取但右臂递送失败，中止全流程"
                            "（物体仍在右爪，请人工接管）"
                        )
                else:
                    return red_pepper, veg_in, False, (
                        f"红甜椒验爪异常（detected={detected}, pos={pos}），"
                        "中止全流程"
                    )
            else:
                red_pepper = "not_found"
                cprint(
                    "[desk_cleanup] 抽屉内未识别到红甜椒（VLM 复检一致），跳过",
                    "yellow",
                )

        for word, _conf in veg_words[:max_veg]:
            cprint(f"[desk_cleanup] 蔬菜: 抓取 {word}", "cyan")
            if not self.visual_grasp(
                word, side="left", use_vlm_grounding=False,
            ):
                # 跳过前 VLM 复检照片：真不在才跳过
                if not self._vlm_present(f"桌上是否还有{word}？"):
                    cprint(
                        f"[desk_cleanup] VLM 确认 {word} 不在桌上，跳过",
                        "yellow",
                    )
                    continue
                cprint(
                    f"[desk_cleanup] VLM 仍见 {word}，启用 grounding 重试一次",
                    "yellow",
                )
                if not self.visual_grasp(
                    word, side="left", use_vlm_grounding=True,
                ):
                    cprint(f"[desk_cleanup] 蔬菜 {word} 重试仍失败，跳过", "yellow")
                    continue

            detected, pos = self._grip_position("left")
            if not detected or pos > self.EMPTY_GRASP_POS:
                return red_pepper, veg_in, False, (
                    f"蔬菜 {word} 验爪异常（detected={detected}, pos={pos}），"
                    "中止全流程（抽屉保持打开，物体在左爪，请人工接管）"
                )

            if not self.handover_pipeline.run(
                mode="dual", speed=1, require_confirmation=False,
                direction="left_to_right",
            ):
                return red_pepper, veg_in, False, (
                    "蔬菜双臂交接失败，中止全流程"
                    "（抽屉保持打开，请人工接管）"
                )
            if not self.place_pipeline.place_at_named_pose(
                "drawer_1_placement", side="right", release_wait=1.5,
            ):
                return red_pepper, veg_in, False, (
                    "蔬菜入抽放置失败，中止全流程"
                    "（抽屉保持打开，请人工接管）"
                )
            veg_in += 1
            cprint(f"[desk_cleanup] 蔬菜已入抽 {veg_in} 件", "green")

        cprint("[desk_cleanup] 阶段4: 右臂关抽屉", "cyan")
        if not self.drawer_pipeline.close():
            return red_pepper, veg_in, False, "抽屉关闭失败，请人工检查"
        return red_pepper, veg_in, True, ""

    def _run_wipe(self):
        """擦桌子：复用 wipe_table 技能（固定抓取后验爪），不新建多余动作。"""
        cprint("[desk_cleanup] 阶段5: 擦桌子（复用 wipe_table）", "cyan")
        from skills.wipe_table import WipeTableSkill
        result = WipeTableSkill(
            config_path=self.config_path, save_path=self.save_path,
        ).run()
        return bool(result)

    # ------------------------------------------------------------------
    # 主编排
    # ------------------------------------------------------------------

    def execute(self, **kwargs):
        plan_only = bool(kwargs.get("plan_only", False))
        max_fruits = int(kwargs.get("max_fruits", 6))
        max_papers = int(kwargs.get("max_papers", 5))
        max_veg = int(kwargs.get("max_veg", 4))
        take_red_pepper = bool(kwargs.get("take_red_pepper", True))
        do_wipe = bool(kwargs.get("wipe", True))

        if plan_only:
            cprint("[desk_cleanup] PLAN-ONLY：只侦察打印计划，不动臂", "yellow")
            plan = self._recon()
            return SkillResult(
                ok=True,
                code="PLAN",
                message="侦察计划已生成（未执行任何动作）",
                data={"plan": plan},
            )

        summary = {
            "fruits_delivered": 0,
            "papers_thrown": 0,
            "veg_stored": 0,
            "red_pepper": "skipped",
            "wipe": False,
        }

        # 阶段0: 双臂回 home（相机不被遮挡）
        cprint("[desk_cleanup] 阶段0: 双臂回 home", "cyan")
        for side in ("left", "right"):
            self.control_arm(pose_type="home", speed=30, side=side)

        # 阶段1: 侦察分类
        plan = self._recon()

        # 阶段2: 水果由左臂到右臂递人
        if plan["fruits"]:
            delivered, ok, err = self._run_fruits(plan["fruits"], max_fruits)
            summary["fruits_delivered"] = delivered
            if not ok:
                return self._fail(err, summary)

        # 阶段3: 纸团直扔
        thrown, ok, err = self._run_papers(plan["papers"], max_papers)
        summary["papers_thrown"] = thrown
        if not ok:
            return self._fail(err, summary)

        # 阶段4: 抽屉支线
        red_pepper, veg_in, ok, err = self._run_drawer(
            plan["vegs"], max_veg, take_red_pepper,
        )
        summary["red_pepper"] = red_pepper
        summary["veg_stored"] = veg_in
        if not ok:
            return self._fail(err, summary)

        # 阶段5: 擦桌子
        if do_wipe:
            summary["wipe"] = self._run_wipe()
            if not summary["wipe"]:
                return self._fail(
                    "擦桌子失败（常见原因：海绵不在固定位），"
                    "其余整理步骤已完成", summary,
                )

        summary["plan"] = plan
        return SkillResult(
            ok=True,
            code="SUCCESS",
            message=(
                f"桌面整理完成：水果左交右递人{summary['fruits_delivered']}、"
                f"纸团{summary['papers_thrown']}、"
                f"红甜椒={summary['red_pepper']}、"
                f"蔬菜入抽{summary['veg_stored']}、"
                f"擦桌={'完成' if summary['wipe'] else '跳过'}"
            ),
            data=summary,
        )

    @staticmethod
    def _fail(message, summary):
        cprint(f"[desk_cleanup] 中止: {message}", "red")
        return SkillResult(
            ok=False,
            code="ABORTED",
            message=message,
            recoverable=True,
            data={"summary": summary},
        )
