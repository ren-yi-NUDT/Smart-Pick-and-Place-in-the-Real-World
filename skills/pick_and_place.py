import copy
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core import scene as scene_map
from core.config import YOLO_CONF_GRASP, YOLO_CONF_CONTAINER
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

        cprint(f"=================== 2. Parse input: Grasp {obj} and place it in the {container} ===================", "cyan")
        self.control_arm(pose_type="grasp1", speed=30)
        self.control_hand(cmd_type="close")

        # ---- Grasp phase ----
        check = False
        for key, value in self.config.default_traj_js.items():
            if "grasp" not in key:
                continue
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
                continue

            filtering_grasping_pose = self._filtering_pose(anygrasp_pose, class_name=obj, image=self.rgb)
            if not filtering_grasping_pose:
                continue

            grasping_pose_world = self._transform_anygrasp_pose(filtering_grasping_pose, _visualization=False)
            if not len(grasping_pose_world):
                continue

            for i in range(len(grasping_pose_world)):
                cprint(f"=================== Checking pose: {i+1} / {len(grasping_pose_world)} ===================", "yellow")
                check = self._execute_grasping_twin_js_2(grasping_pose_world[i], idx=key)
                if check:
                    break

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
        last_js = None
        for key, value in self.config.default_traj_js.items():
            if "grasp" not in key:
                continue
            last_js = value
            self.control_arm(pose_type=key, speed=30)
            self.rgb, self.depth = self.get_camera_obs()
            cprint(f"P=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")
            self.save_current_transformation()
            placing_pos_world = self._get_placing_position(class_name=container, image=self.rgb)
            cprint("P=================== 4. Generate the placing pose ===================", "cyan")

            if not len(placing_pos_world):
                continue
            check = self._execute_placement(placing_pos_world, current_js=value)
            if check:
                break

        # 四个观察位姿都没看见容器, 再查一次地图。地图命中不依赖臂的当前朝向,
        # 所以放在循环外试一次就够。
        if not check:
            from_scene = self._placing_position_from_scene(container)
            if from_scene is not None:
                check = self._execute_placement(from_scene, current_js=last_js)

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

        return final_grasps if ans else []

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

    def _placing_position_from_scene(self, container):
        """实时检测没找到容器时, 回退到 look_around 存的场景地图。

        臂在 grasp1 看不见蓝盘子, 在 grasp4 看不见粉盘子 —— 用户说"放蓝盘子里"时
        臂恰好背对着就会失败, 尽管它 10 分钟前才看过。地图就是为这个存的。

        地图是快照: 东西被人挪过就对不上了, 所以命中时把扫描时间打出来, 让现场
        知道这个坐标有多新。
        """
        scene = scene_map.load()
        if not scene:
            return None
        words = container.split()
        color = words[0] if len(words) > 1 else None
        obj = scene_map.find(scene, words[-1], color)
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
