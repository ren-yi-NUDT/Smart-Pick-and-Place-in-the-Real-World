import copy
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core import scene as scene_map
from core.config import SCENE_MAP_TTL_S, YOLO_CONF_CONTAINER
from core.transforms import pixel_to_camera_point2

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


# 放置时"接近轴朝下"的末端朝向沿"背离容器"方向倾倒这几档 (角度单位: 度)。
#
# 腕部比指尖远 172mm, 竖直朝下就等于把腕部往外多推 172mm, 容器一远就够不着。
# 朝背离容器一侧倾倒, 腕部就沿容器方向收回底座这边。实测蓝盘子 (xy 半径 0.587,
# 落点 z=0.18): 竖直 delta 0.165 不可达, 倒 45° 后 delta 0.0001 精确命中。
#
# 角度只能挨个试, 不能单调扫 —— 孪生的 IK 分叉是离散的, 差一点就掉到另一支上:
# 朝下的姿势里 0/-15/-30 被拒而 -45/-60 可行, 换成抓取位姿那个朝向基准又变成
# 只有 -30 可行。所以两族基准都留着, 角度全列上。
PLACEMENT_TILTS = (-15, -30, -45, -60)


def _placement_orn_candidates(base_rot, container_xy):
    """放置末端朝向备选: 容器自身朝向 + "接近轴朝下"沿背离容器方向倾倒的几档。

    末端绕**自身接近轴**的偏航角不影响结果 —— 实测 0/90/180/270 四组 delta 完全
    一样, 所以朝下的基准姿势随便取一个。
    """
    cands = [base_rot]
    perp = np.array([-container_xy[1], container_xy[0], 0.0])
    norm = np.linalg.norm(perp)
    if norm < 1e-6:                     # 容器就在底座正上方, 没有"背离方向"可言
        return cands
    perp /= norm
    down = R.from_euler("x", 180, degrees=True).as_matrix()      # 接近轴朝下
    cands += [R.from_rotvec(perp * np.radians(t)).as_matrix() @ down for t in PLACEMENT_TILTS]
    return cands


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
        if not self._grasp_object(obj, scene):
            return False

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
            self.control_arm(pose_type="grasp1", speed=40)
            cprint("H=================== 5. Successfully completed the handover task ===================", "green")
            return True

        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            cprint("T=================== Trash mode detected: throwing to trash ===================", "cyan")
            # 贴合原项目 place_at_named_pose: 录制位姿直接裸发关节角。
            if self.config.get_pose("throw_to_trash_pose") is None:
                cprint("T=================== Trash task failed: throw_to_trash_pose not found ===================", "red")
                return False
            self.control_arm(pose_type="throw_to_trash_pose", speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=40)
            cprint("T=================== 5. Successfully completed the trash task ===================", "green")
            return True

        if container.lower() in ["desk", "桌子", "table"]:
            cprint("D=================== Desk placement mode detected: placing on desk ===================", "cyan")
            # 贴合原项目: 三个桌位姿随机挑一个, 裸发录制关节角。
            name = random.choice(["desk_pose_1", "desk_pose_2", "desk_pose_3"])
            if self.config.get_pose(name) is None:
                cprint(f"D=================== Desk task failed: {name} not found ===================", "red")
                return False
            self.control_arm(pose_type=name, speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=40)
            cprint("D=================== 5. Successfully completed the desk placement task ===================", "green")
            return True

        # Normal vision-based placement
        check = False
        poses, from_map = self._poses_for(scene, container, "容器")
        for key in poses:
            current_js = self.config.get_pose(key)
            self.control_arm(pose_type=key, speed=40)
            self.rgb, self.depth = self.get_camera_obs()
            cprint(f"P=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")
            self.save_current_transformation()
            cprint("P=================== 4. Generate the placing pose ===================", "cyan")

            if from_map:
                # 位置直接用地图记的世界坐标 —— 现场那个框可能被画面切掉 (_well_framed
                # 会丢掉它, 实测 grasp3 看蓝盘子只框到 21px 高的一条), 地图坐标反而更稳。
                #
                # 这里**不能**再核对"现场还认得出容器吗"。相机在腕上, 手里一有东西它就
                # 正堵在镜头前: 2026-10-06 抓完桃子到 grasp4, log/rgb.png 整个画面就是
                # 那颗桃子, 蓝盘子一点都看不见。物体越大越堵, 也就是**正常情况**必然触发,
                # 而触发时臂正夹着东西悬在半空 —— 最不该停的时刻。而且它想防的场景 (容器
                # 被挪走了) 同样被挡住, 根本防不住。地图过期由 SCENE_MAP_TTL_S 兜。
                placing_pos_world = self._placing_position_from_scene(container, scene)
            else:
                placing_pos_world = self._get_placing_position(class_name=container, image=self.rgb)

            if placing_pos_world is None or not len(placing_pos_world):
                continue
            check = self._execute_placement(placing_pos_world, current_js=current_js)
            if check:
                break

        self.control_arm(pose_type="grasp1", speed=40)
        # 四个观察位姿都放不下就是失败 —— 以前这里无条件报成功, 于是"所有位姿都被孪生
        # 拒了"也打印 Successfully completed, 现场看起来像放好了其实夹爪一直夹着东西。
        if not check:
            cprint("P=================== Placement task failed ===================", "red")
            return False
        cprint("P=================== 5. Successfully completed the placement task ===================", "green")
        return True

    def _grasp_object(self, obj, scene):
        """两个 CLI 入口共用抓取阶段，不建立额外硬件连接。

        抓取逻辑复刻自原项目 core/grasp_pipeline.py (visual_grasp): 候选打分
        (AnyGrasp 分 / 孪生可达 / 宽度 / 长度 / 接近角) → 三档预抓取高度重试 →
        generation2 规划 → soft close 探测 + hold 补夹 → 倒走孪生轨迹回撤 →
        提升验证。``scene`` 参数不再用于选观察位 (原版按配置的 grasp1-4 逐个走),
        地图留给放置阶段用。
        """
        return self.grasp_pipeline.visual_grasp(obj)

    # ------------------------------------------------------------------
    # 场景地图: 决定去哪个观察位姿
    # ------------------------------------------------------------------
    def _poses_for(self, scene, target, what):
        """该去哪些观察位姿看 ``target``。返回 ``(位姿列表, 是不是地图选的)``。

        地图里有它 → 去**看见过它的那些**位姿, 按 seen_from 的顺序 (最先看见的在前)。
        地图里没有 → 回退到全部观察位姿 (保底, 但把话说清楚)。

        为什么不是一个: ``seen_from`` 记的是"从这儿**看得见**它", 而看得见不等于
        **抓得着**。2026-10-06 桃子实测: 地图命中的是 grasp2, YOLO 在 grasp2 上确实
        检出了桃子 (框 (495,289)-(598,395), conf 0.85, 画面对), 但 AnyGrasp 在那一帧
        只给出 9 个候选, **全部落在中间的黄瓜上**, 桃子一个都没有 —— 桃子当时贴在画面
        右沿、掠射角, 点云根本立不起来。位姿换到 grasp3 立刻就有 15 个候选落在桃子上。
        这种事只有走过去才知道, 所以按顺序试, 第一个能出候选就停 (常态, 不多走一步)。
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
               f"(扫描于 {scene.get('scanned_at')}), 依次去 {poses}", "yellow")
        return poses, True

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

        地图是快照, 但**放的时候没法现场核对**: 手里夹着东西, 相机在腕上, 镜头被
        堵死。所以这里直接信地图, 过期由 SCENE_MAP_TTL_S 兜 (理由详见调用处)。
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
        # 容器自身朝向优先 (原来的行为), 不行再换备选朝向, 最后才试倾倒档 —— 倾倒档
        # 是给"容器离底座太远"兜底的, 近处第一个朝向就成了, 一次都不会多问。
        orns = [base[:3, :3]] + [R.from_quat(q).as_matrix() for q in PLACEMENT_ORN_FALLBACKS]
        orns += _placement_orn_candidates(base[:3, :3], base[:2, 3])[1:]
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
