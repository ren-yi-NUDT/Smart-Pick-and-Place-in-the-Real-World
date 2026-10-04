import copy
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core import scene as scene_map
from core.config import SCENE_MAP_TTL_S, YOLO_CONF_GRASP, YOLO_CONF_CONTAINER
from core.transforms import graspcam2pixel, self_rotation_np, rpy_to_vector, pixel_to_camera_point2

# soft close 探测到物体后服务端是以 force=0 持握的, 提臂前补一次实夹的力度。
GRIPPER_HOLD_FORCE = 40

# 放容器时的末端朝向备选。只给一个写死的四元数不够用: 实测同一个盘子中心点,
# 默认朝向 IK 残差 66mm (够不着), 换成 place2 的末端朝向 0.1mm 精确命中 —— 卡住
# 的是姿态不是位置。这三个是机械臂自己录制的"放下物体"位姿的末端朝向。
# 重新生成: 用孪生跑一次 trajectory_generation2 (target_pose 给任意可达点, current_js
# 给该录制位姿), 返回的 trajectory_ee[0][3:] 就是它的末端四元数。
PLACEMENT_ORN_FALLBACKS = [
    [0.0656, -0.7928, 0.1117, 0.5956],  # place2
    [0.2197, -0.7229, 0.0792, 0.6503],  # place1
    [0.0094, -0.8820, 0.0341, 0.4698],  # desk_pose_3
]


@register_skill("pick_and_place")
class PickAndPlaceSkill(Skill):
    """Main pick-and-place pipeline: detect → grasp → place."""

    def run(self, **kwargs):
        if kwargs.get("object") and kwargs.get("container"):
            json_data = kwargs
        else:
            json_data = self.json_parser.get_command()
        if json_data is None:
            cprint("未收到有效的JSON输入", "red")
            return False

        cprint(f"=================== 1. Get JSON input: {json_data} ===================", "cyan")

        obj, container = json_data.get("object"), json_data.get("container")
        if obj is None or container is None:
            cprint("JSON输入缺少必需字段 (object 或 container)", "red")
            return False

        # 先查地图再动手: 没有地图就没有"东西在哪"的知识, 靠现场碰运气就是以前那个
        # 逐位姿瞎找。过期地图比没有更糟 (照着一张过期的坐标去够会够到空处), 所以这里
        # 直接停, 不退回四位姿循环 —— 退回就等于把"该重扫了"这个信号吃掉。
        scene = scene_map.load_fresh(SCENE_MAP_TTL_S)
        if scene is None:
            cprint(f"[map] 没有可用的场景地图 (没扫过 / 文件坏了 / 超过 {SCENE_MAP_TTL_S}s 过期)。"
                   f"请先跑 look_around。", "red")
            return False

        cprint(f"=================== 2. Parse input: Grasp {obj} and place it in the {container} ===================", "cyan")
        self.control_arm(pose_type="grasp1", speed=30)
        self.control_hand(cmd_type="close")

        # ---- Grasp phase ----
        check = False
        poses, from_map = self._poses_for(scene, obj, "物体")
        for key in poses:
            self.control_arm(pose_type=key, speed=30)
            self.rgb, self.depth = self.get_camera_obs()
            cprint(f"G=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")

            # raw_depth: 相机测不到的物体 (D455 最小距离 ~286mm, 相机离桌面只有 ~30cm)
            # 补洞后是一片桌面, AnyGrasp 会去抓物体前面的桌子。给它原始深度才能把物体
            # 顶回来, 见 core/camera.py:dome_fill_box。
            anygrasp_pose = self.perception.detect_grasps(
                self.rgb, self.depth, class_name=obj,
                raw_depth=getattr(self.camera, "raw_depth", None),
            )
            cprint(f"G=================== 4. Generate the grasping pose and save it in file: {self.save_path}/result.json ===================", "cyan")
            self.save_current_transformation()
            if not anygrasp_pose:
                cprint(f"[grasp] {key} 上 AnyGrasp 没给出任何抓取候选", "yellow")
                continue

            filtering_grasping_pose, detected = self._filtering_pose(anygrasp_pose, class_name=obj, image=self.rgb)
            # 地图说这个位姿看得见目标, 现场却什么都没检出 —— 东西被挪了, 或者地图过期了。
            # 停下问人, 不换下一个位姿 (那是"重复移动", 而且会把地图错了这件事悄悄吃掉)。
            if from_map and not detected:
                self._stop_map_mismatch(obj, key, scene)
                return False
            if not filtering_grasping_pose:
                continue

            grasping_pose_world = self._transform_anygrasp_pose(filtering_grasping_pose, _visualization=False)
            if not len(grasping_pose_world):
                continue

            # 只试分数最高的那一个。以前按候选逐个试, 一个抓不到就再试十几个 —— 现场看到的
            # 是"夹爪在果子旁边反复开合", 而失败的候选本身也没被修好过。抓不到就认, 往下走。
            #
            # "往下走"指的是换下一个**观察位姿** (换视角才换得到抓取几何), 不是换候选。
            # 地图命中时 poses 只有一个, 所以这里等于"抓不到就结束"。
            cprint(f"=================== 只试最高分的一个候选 ===================", "yellow")
            check = self._execute_grasping_twin_js_2(grasping_pose_world[0], idx=key)
            if check:
                break

        if not check:
            cprint("G=================== Grasping task failed ===================", "red")
            return False

        cprint("G=================== 5. Successfully completed the grasping task ===================", "green")

        # ---- Placement phase ----
        if container.lower() == "person":
            cprint("H=================== Handover mode detected: delivering to person ===================", "cyan")
            pose_1st = self.config.get_pose("get_ready_to_handover_1st")
            pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
            if self.config.get_pose("handover_pose") is None:
                cprint("H=================== Handover task failed: handover_pose not found ===================", "red")
                return False
            if pose_1st is not None:
                self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
            if pose_2nd is not None:
                self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
            self.control_arm(pose_type="handover_pose", speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("H=================== 5. Successfully completed the handover task ===================", "green")
            return True

        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            cprint("T=================== Trash mode detected: throwing to trash ===================", "cyan")
            throw_pose = self.config.get_pose("throw_to_trash_pose")
            if throw_pose is None:
                cprint("T=================== Trash task failed: throw_to_trash_pose not found ===================", "red")
                return False
            joint_angles = [throw_pose[f"J{i}"] for i in range(1, 7)]
            self.control_arm(trajectory=np.array([joint_angles]), speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("T=================== 5. Successfully completed the trash task ===================", "green")
            return True

        if container.lower() in ["desk", "桌子", "table"]:
            cprint("D=================== Desk placement mode detected: placing on desk ===================", "cyan")
            selected = random.choice(["desk_pose_1", "desk_pose_2", "desk_pose_3"])
            desk_pose = self.config.get_pose(selected)
            if desk_pose is None:
                cprint(f"D=================== Desk task failed: {selected} not found ===================", "red")
                return False
            joint_angles = [desk_pose[f"J{i}"] for i in range(1, 7)]
            self.control_arm(trajectory=np.array([joint_angles]), speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("D=================== 5. Successfully completed the desk placement task ===================", "green")
            return True

        # Normal vision-based placement
        check = False
        poses, from_map = self._poses_for(scene, container, "容器")
        for key in poses:
            current_js = self.config.get_pose(key)
            self.control_arm(pose_type=key, speed=30)
            self.rgb, self.depth = self.get_camera_obs()
            cprint(f"P=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")
            self.save_current_transformation()
            cprint("P=================== 4. Generate the placing pose ===================", "cyan")

            if from_map:
                # 地图说这个位姿看得见容器, 现场必须也认得出 —— 认不出就停下问人。
                # 位置直接用地图记的世界坐标: 现场那个框可能被画面切掉 (_well_framed 会
                # 丢掉它, 实测 grasp3 看蓝盘子只框到 21px 高的一条), 地图坐标反而更稳。
                if not self._map_still_holds(scene, container, self.rgb):
                    self._stop_map_mismatch(container, key, scene)
                    return False
                placing_pos_world = self._placing_position_from_scene(container, scene)
            else:
                placing_pos_world = self._get_placing_position(class_name=container, image=self.rgb)

            if placing_pos_world is None or not len(placing_pos_world):
                continue
            check = self._execute_placement(placing_pos_world, current_js=current_js)
            if check:
                break

        self.control_arm(pose_type="grasp1", speed=30)
        # 四个观察位姿都放不下就是失败 —— 以前这里无条件报成功, 于是"所有位姿都被孪生
        # 拒了"也打印 Successfully completed, 现场看起来像放好了其实夹爪一直夹着东西。
        if not check:
            cprint("P=================== Placement task failed ===================", "red")
            return False
        cprint("P=================== 5. Successfully completed the placement task ===================", "green")
        return True

    # ------------------------------------------------------------------
    # Grasp helpers (from planner.py)
    # ------------------------------------------------------------------
    def _filtering_pose(self, anygrasp_pose, class_name="", image=None):
        """返回 ``(抓取位姿列表, 有没有检出目标)``。

        第二个返回值不能省: "什么都没检出"和"检出了但框里没有抓取点"是两回事。
        前者说明地图和现场对不上 (该停下问人), 后者只是这个候选不行。
        """
        # YOLO-World 在这张桌面上对单件蔬果只给 0.05 左右的分数 (框是准的, 分低),
        # 用 0.2 会把目标整个漏掉, 后面所有候选都被判"没有物体"。
        # 阈值见 core/config.py:YOLO_CONF_GRASP (实测目标分数低到 0.012, 高了直接漏检)。
        # 放宽后仍要求框内必须落进 AnyGrasp 候选点, 空框自然被丢掉。
        detections = self.perception.detect_objects(image, class_name, conf=YOLO_CONF_GRASP)

        grasp_points, grasp_pose_cam = graspcam2pixel(anygrasp_pose)
        valid_indices = set()
        final_grasps = []
        valid_boxes = []
        ans = False

        if len(detections):
            det = detections[0][:4]
            x1, y1, x2, y2 = det
            valid_boxes.append((x1, y1, x2, y2))
            for i, grasp_p in enumerate(grasp_points):
                if grasp_p[0] > x1 - 20 and grasp_p[0] < x2 + 20 and \
                   grasp_p[1] > y1 - 20 and grasp_p[1] < y2 + 20:
                    valid_indices.add(i)

            if len(valid_indices):
                sorted_indices = sorted(list(valid_indices))
                class_names = [cls for cls in class_name.split(',')]
                for i in sorted_indices:
                    g_pose = grasp_pose_cam[i]
                    final_grasps.append(g_pose)
                cprint(
                    f"*********** Class name {class_names} ****************** "
                    f"Grasp pose number: {len(final_grasps)} ******************", "green")
                ans = True
            else:
                cprint(f"Found objects ({class_name}) but NO grasp points inside them.", "yellow")
        else:
            cprint(f"No object detected for class: {class_name}", "yellow")

        return (final_grasps if ans else []), bool(len(detections))

    # ------------------------------------------------------------------
    # 场景地图: 决定去哪个观察位姿
    # ------------------------------------------------------------------
    def _poses_for(self, scene, target, what):
        """该去哪些观察位姿看 ``target``。返回 ``(位姿列表, 是不是地图选的)``。

        地图里有它 → **只去最先看见它的那个** (``seen_from[0]``, 走路最少)。
        地图里没有 → 回退到全部观察位姿 (保底, 但把话说清楚)。
        """
        all_poses = [k for k in self.config.default_traj_js if "grasp" in k]
        color, name = scene_map.parse_target(target)
        hit = scene_map.find(scene, name, color)
        if hit is None:
            cprint(f"[map] 地图里没有{what} '{target}', 回退到逐个观察位姿", "yellow")
            return all_poses, False
        poses = [p for p in hit["seen_from"] if p in all_poses]
        if not poses:
            cprint(f"[map] {what} '{target}' 的记录没带观察位姿, 回退到逐个观察位姿", "yellow")
            return all_poses, False
        cprint(f"[map] {what} '{target}' 命中 {hit['color'] or '?'} {hit['name']} "
               f"(扫描于 {scene.get('scanned_at')}), 只去 {poses[0]}", "yellow")
        return poses[:1], True

    def _stop_map_mismatch(self, target, pose, scene):
        """地图说看得见、现场看不见 —— 停下问人, 不猜、不回退、不换下一个位姿。

        回退到四位姿循环正是这次要消灭的"重复移动", 而且它会把"地图错了"这个信号悄悄
        吃掉, 下次还是错的。
        """
        cprint(f"[map] 地图说 '{target}' 在 {pose} 看得见 (扫描于 {scene.get('scanned_at')}), "
               f"但现场在 {pose} 没检出它。是东西被挪了, 还是地图过期了? "
               f"停下, 请你确认。", "red")

    def _map_still_holds(self, scene, target, rgb):
        """现场这一帧还认得出地图里那件东西吗。

        用**建图同一套检测**核对 (见 core/scene.py:detect_scene) —— 换成容器的
        detect_container 会多一层"框被切掉就丢掉"的过滤, 于是画面边缘的框会被
        误报成"东西被挪了"。
        """
        color, name = scene_map.parse_target(target)
        for got_name, got_color, _box, _conf in scene_map.detect_scene(self.perception, rgb):
            if got_name == name and (color is None or got_color == color):
                return True
        return False

    def _transform_anygrasp_pose(self, anygrasp_pose, _visualization=True, return_labels=False):
        try:
            eval_score = {}
            for id, data in enumerate(anygrasp_pose[:]):
                grasp_translation = data['trans']
                grasp_rotation_matrix = data['rotation_matrix']

                grasp_transformation_matrix = np.eye(4, 4)
                grasp_transformation_matrix[:3, :3] = grasp_rotation_matrix
                grasp_transformation_matrix[:3, 3] = grasp_translation

                self_pose_matrix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                self_pose_matrix = self_rotation_np(self_pose_matrix)

                transformed_pose_camera = grasp_transformation_matrix @ self_pose_matrix
                transformed_pose_world = self.T_base_to_cam @ transformed_pose_camera
                transformed_pose_world = self._transform_x_axis(transformed_pose_world)

                eval_score[id] = {}
                eval_score[id]["score"] = data["score"]
                eval_score[id]["transformed_pose_world"] = transformed_pose_world
                eval_score[id]["original_pose"] = anygrasp_pose[id]
                if return_labels:
                    eval_score[id]["label"] = data["label"]

            final_grasp_pose_data = sorted(eval_score.items(), key=lambda x: x[1]["score"], reverse=True)
            return [final_grasp_pose_data[i][1]["transformed_pose_world"] for i in range(len(final_grasp_pose_data))]
        except Exception:
            return []

    def _transform_x_axis(self, transformed_pose_world):
        r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler('xyz', degrees=False)
        x_axis_rotated = rpy_to_vector(r, p, y, axis=[1, 0, 0])
        y_axis_world = np.array([0, 1, 0])
        cos_theta = np.dot(x_axis_rotated, y_axis_world) / (
            np.linalg.norm(x_axis_rotated) * np.linalg.norm(y_axis_world)
        )
        if cos_theta > 0:
            transformed_pose_world = transformed_pose_world @ np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        return transformed_pose_world

    def _execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None):
        """Exact replica of planner.py execute_grasping_twin_js_2."""
        self.control_hand(cmd_type="open")
        basic_mat = np.eye(4)
        basic_mat[2, 3] -= 0.02
        preparasion_grasping_pos = grasping_pose_world_hand @ basic_mat
        preparasion_grasping_pos[2, 3] += 0.02
        preparasion_grasping_pos = preparasion_grasping_pos @ self.T_hand_effector_to_arm_endlink
        preparasion_grasping_pos_position = preparasion_grasping_pos[:3, 3]
        preparasion_grasping_pos_orientation = R.from_matrix(preparasion_grasping_pos[:3, :3]).as_quat()

        basic_mat = np.eye(4)
        execution_grasping_pos = grasping_pose_world_hand @ basic_mat
        execution_grasping_pos[2, 3] = max(execution_grasping_pos[2, 3], 0.042)
        execution_grasping_pos = execution_grasping_pos @ self.T_hand_effector_to_arm_endlink
        execution_grasping_pos_position = execution_grasping_pos[:3, 3]
        execution_grasping_pos_orientation = R.from_matrix(execution_grasping_pos[:3, :3]).as_quat()

        default_traj_js_rad = [data / 180 * np.pi for data in self.config.default_traj_js[idx].values()]
        cnfg = {
            "target_pose": [
                [preparasion_grasping_pos_position[0], preparasion_grasping_pos_position[1], preparasion_grasping_pos_position[2],
                 preparasion_grasping_pos_orientation[0], preparasion_grasping_pos_orientation[1], preparasion_grasping_pos_orientation[2], preparasion_grasping_pos_orientation[3]],
                [execution_grasping_pos_position[0], execution_grasping_pos_position[1], execution_grasping_pos_position[2],
                 execution_grasping_pos_orientation[0], execution_grasping_pos_orientation[1], execution_grasping_pos_orientation[2], execution_grasping_pos_orientation[3]],
            ],
            "current_js": default_traj_js_rad,
            "struct": "rm65",
        }
        rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
        state = rsp["value"]

        if state:
            trajectory_grasping = rsp["info"]["trajectory"]
            trajectory_grasping = np.array(copy.deepcopy(trajectory_grasping)) / np.pi * 180
            self.control_arm(trajectory=trajectory_grasping, speed=20)
            cprint("=============== Reach grasping pose =============")
            self.control_hand(cmd_type="close")
            cprint("=============== Close hand =============")
            time.sleep(0.5)

            # 孪生"能生成轨迹"只说明这个位姿够得着, 不代表夹爪真夹住了东西。
            # 以前这里直接 return True, 于是候选循环永远停在第一个可达候选上 ——
            # 橙子那次就是第一个候选斜着进刀把果子推开了, 照样算"抓取成功"。
            # check_grasping_object 走夹爪的 soft close, 用 gOBJ 判定合拢时中间有没有东西。
            held = self.check_grasping_object()
            if held:
                # soft close 探测完是以 force=0 持握的, 光滑一点的物体一提就滑, 所以
                # 要补一次实夹再提臂。但必须用 hold() 停在探测到的开度上 —— 用 close()
                # 会被当成"合到底", 从探到的开度一路挤到机械限位, 近球形的果子直接被
                # 挤出去 (现场: "夹爪一端先碰到果子, 然后往里收, 结果没夹起来")。
                self.hand.hold(force=GRIPPER_HOLD_FORCE)

            self.control_arm(pose_type=idx, speed=30)
            cprint("=============== Reach post grasping pose =============")
            if not held:
                cprint("=============== 夹爪空合, 该候选没夹住, 试下一个 =============", "yellow")
                self.control_hand(cmd_type="open")
            return held
        else:
            cprint("********************* The preparasion pose is not reachable !! *********************")
            return False

    # ------------------------------------------------------------------
    # Placement helpers (from planner.py)
    # ------------------------------------------------------------------
    def _get_placing_position(self, class_name=None, image=None):
        detections = self.perception.detect_container(image, class_name, conf=YOLO_CONF_CONTAINER)
        try:
            if len(detections):
                det = detections[0][:4]
                if det[1] >= 400 and det[3] <= 480 and len(detections) > 1:
                    det = detections[1][:4]
                x1, y1, x2, y2 = [int(coord) for coord in det]

                H, W = self.depth.shape
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(W, x2)
                y2 = min(H, y2)

                depth_sub_image_mm = self.depth[y1:y2, x1:x2]
                valid_depths_mm = depth_sub_image_mm[depth_sub_image_mm > 0]

                if len(valid_depths_mm) > 0:
                    mean_depth_mm = np.median(valid_depths_mm)
                else:
                    print("Warning: No valid depth values found in the bounding box.")
                    return []

                mean_depth_m = mean_depth_mm * 1e-3

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_cam_point = pixel_to_camera_point2(np.array([center_x, center_y]).reshape(-1, 2), mean_depth_m)
                center_cam_point = center_cam_point.flatten()
                placing_pos_world = self._transform_pose_to_world(center_cam_point)
                return placing_pos_world
        except Exception:
            pass
        return []

    def _placing_position_from_scene(self, container, scene):
        """容器在世界里的位置 —— 直接取场景地图记的那个坐标。

        臂在 grasp1 看不见蓝盘子, 在 grasp4 看不见粉盘子 —— 用户说"放蓝盘子里"时
        臂恰好背对着就会失败, 尽管它 10 分钟前才看过。地图就是为这个存的, 而且它给的
        是世界坐标, 不受当前视角影响 (现场框被画面切掉时反而更准)。

        地图是快照, 所以调用方必须先确认现场还看得见它 (见 _map_still_holds)。
        """
        if not scene:
            return None
        color, name = scene_map.parse_target(container)
        obj = scene_map.find(scene, name, color)
        if obj is None:
            return None
        cprint(f"P=========== 实时没看到 {container}, 查场景地图: 命中 "
               f"{obj['color'] or '?'} {obj['name']} (扫描于 {scene.get('scanned_at')}) ===========", "yellow")
        T = np.eye(4)
        T[:3, :3] = R.from_quat(PLACEMENT_ORN_FALLBACKS[0]).as_matrix()
        T[:3, 3] = obj["world"]
        return T

    def _transform_pose_to_world(self, pose_cam_point):
        placing_translation = pose_cam_point.flatten()
        T_cam_point = np.eye(4, 4)
        T_cam_point[:3, :3] = R.from_quat([-0.210, 0.016, -0.056, 0.976]).as_matrix()
        T_cam_point[:3, 3] = placing_translation
        T_world_point = self.T_base_to_cam @ T_cam_point
        T_world_pose = T_world_point.copy()
        return T_world_pose

    def _execute_placement(self, placement_pos_world, current_js=None):
        base = placement_pos_world.copy()
        base[2, 3] += 0.15

        # 必须用"机械臂此刻真正所在的观察位姿"当轨迹起点。写死 grasp1 时, 如果放置是在
        # grasp2/3/4 上找到容器, 轨迹第一个路点就是 grasp1 —— 真机会从 grasp4 直接甩到
        # grasp1, 中间扫过桌面。调用方在 line 128 刚把臂移到 key, 所以起点就是 key。
        if current_js is None:
            current_js = self.config.default_traj_js["grasp1"]
        default_js_rad = [v / 180 * np.pi for v in current_js.values()]

        # 容器自身朝向优先 (原来的行为), 不行再换备选朝向。位置随朝向变 ——
        # T_hand_effector_to_arm_endlink 是纯平移, 但平移方向跟着朝向转, 所以每个
        # 候选都得重新算一遍 pos。
        orns = [base[:3, :3]] + [R.from_quat(q).as_matrix() for q in PLACEMENT_ORN_FALLBACKS]
        for i, rot in enumerate(orns):
            T = base.copy()
            T[:3, :3] = rot
            T = T @ self.T_hand_effector_to_arm_endlink
            pos, orn = T[:3, 3], R.from_matrix(T[:3, :3]).as_quat()
            cnfg = {
                "target_pose": [[pos[0], pos[1], pos[2], orn[0], orn[1], orn[2], orn[3]]],
                "current_js": default_js_rad,
                "struct": "rm65",
            }
            rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
            if not rsp["value"]:
                continue
            if i:
                cprint(f"=============== 默认放置朝向不可达, 换第 {i} 个备选朝向 =============", "yellow")
            traj = np.array(copy.deepcopy(rsp["info"]["trajectory"])) / np.pi * 180
            self.control_arm(trajectory=traj, speed=20)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            return True
        return False
