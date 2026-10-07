import copy
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core import scene as scene_map
from core.config import SCENE_MAP_TTL_S, TABLE_Z_M, YOLO_CONF_GRASP, YOLO_CONF_CONTAINER
from core.transforms import graspcam2pixel, self_rotation_np, rpy_to_vector, pixel_to_camera_point2

# soft close 探测到物体后服务端是以 force=0 持握的, 提臂前补一次实夹的力度。
GRIPPER_HOLD_FORCE = 40

# 一个观察位姿上试几个抓取候选。定死 1 —— 夹不到就换观察位姿, 不遍历候选。
# 用户 2026-10-06 / 2026-10-07: "一次抓不到直接继续, 不要再跑候选"。
MAX_GRASP_CANDIDATES = 1

# "差不多一样准"的容差: 手爪点离物体框中心的像素距离在最小值的 1.5 倍以内, 就算一样准。
# 在这个集合里再挑最竖直的 —— **抓准优先于抓正**。用户 2026-10-07: "都抓的偏前"。
# 实测两张候选表 (火龙果 grasp3) 上 1.0~3.0 取哪个值结果都一样, 所以不必精调。
CENTER_TOL = 1.5

# 预抓取位姿相对抓取点的抬法, 从高到低试: 每项是 (沿接近轴后退, 世界Z再抬)。
#
# 第一档是原来那一档 (两笔在竖直下爪时叠成 4cm), 是给"从观察位姿飞过来"留的余量,
# 不能动。但它在够不太着的地方会要命: link_6 比指尖高 172mm, 预抓取抬多高, 腕部半径
# 就多出去多少。2026-10-06 桃子 (指尖半径 0.439m): 抬 4cm 的预抓取位姿腕部半径
# 0.637m, 孪生判不可达 (差 15.8mm); 同一个候选的执行位姿腕部半径 0.609m, 残差 0.3mm。
# 差的不是抓取点, 就是这 4cm。
#
# 后面几档是纯计算的重试 (臂不动, 只是再问一次孪生), 所以矮一档没有代价。
PRE_GRASP_OFFSETS = [(0.02, 0.02), (0.005, 0.005)]

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


def _grasp_height(contact_z):
    """指尖该落在哪个高度: **接触点到桌面高度的一半**。

    AnyGrasp 给的是可见表面上的接触点。相机装在腕上从上往下看, 那基本就是物体的
    **顶面**, 于是手指在物体的**上方**合拢 —— 2026-10-07 火龙果 (高 8.6cm) 现场就是
    "夹爪空合": 接触点在果子顶 z=0.092, 而果子高度中点是 0.052, 差了 40mm。

    取一半正好落在中点上, 也就是物体的最宽处 (手指从两侧夹住的地方):
      - 圆的果子: 接触点在顶, 一半 = 高度中点;
      - 躺着的东西: 黄瓜直径 4.4cm, 接触点在顶面, 一半 = 轴心高度。
    两者都不是凑出来的, 是"顶面高度的一半 = 最宽处"。

    只会往下调, 不会往上 —— 接触点永远在物体表面, 而表面不会低于最宽处。
    """
    return (contact_z + TABLE_Z_M) / 2.0


def _pre_grasp_pose(grasp_world, along, up, T_hand_effector_to_arm_endlink):
    """抓取点 -> 预抓取位姿 (末端 link_6 系): 沿接近轴后退 ``along``, 再在世界 Z 上抬 ``up``。"""
    basic_mat = np.eye(4)
    basic_mat[2, 3] -= along
    pre = grasp_world @ basic_mat
    pre[2, 3] += up
    return pre @ T_hand_effector_to_arm_endlink


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
        self.control_arm(pose_type="grasp1", speed=30)
        # 观察时必须张着夹爪。夹爪就长在相机视场里 (画面下沿那两个黑块就是手指),
        # 合拢着拍, 两根手指正好挡在物体前面: 2026-10-06 抓香蕉, 合着拍时 YOLO 的
        # 置信度从 0.45 跳到 0.88, AnyGrasp 最高分候选跑到离香蕉 55mm 的地方 (手指
        # 本身), 臂照那个点压下去, 空合一把。张着拍, 最高分候选回到香蕉上 (19mm)。
        # 这里原来是 close, 是双臂 planner 留下的"起始状态"习惯, 没有任何东西依赖它。
        self.control_hand(cmd_type="open")

        # ---- Grasp phase ----
        check = False
        self._obs_ee_world = None      # 观察位姿的末端笛卡尔位姿, 抓取那步顺带填
        self._obs_js = None            # 观察位姿的关节角 (弧度), 同上
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

            filtering_grasping_pose, detected, offsets_px = self._filtering_pose(
                anygrasp_pose, class_name=obj, image=self.rgb)
            # 地图说看得见、现场什么都认不出 —— 东西被挪了, 或者地图过期了。停下问人,
            # 不猜不换位姿 (换了就把"地图错了"这个信号悄悄吃掉)。
            # 只在**最后一个**位姿上判: seen_from 可能有好几个位姿, 前面那个视角差一点、
            # 认不出目标很正常, 那不算地图错了。
            if from_map and not detected and key == poses[-1]:
                self._stop_map_mismatch(obj, key, scene)
                return False
            if not filtering_grasping_pose:
                continue

            grasping_pose_world = self._transform_anygrasp_pose(filtering_grasping_pose, _visualization=False)
            if not len(grasping_pose_world):
                continue

            # 候选顺序: 最竖直的排前面, 但**够得着**优先 —— 孪生规划不过去的直接跳过,
            # 连试都不试 (见 _pick_reachable)。仍然是每个观察位姿只真抓一次, 夹不到就换
            # 下一个位姿, **不遍历候选**: 用户 2026-10-06 / 2026-10-07 "一次抓不到直接
            # 继续, 不要再跑候选" —— 现场看到的是夹爪在果子旁边反复开合。
            candidate = self._pick_reachable(grasping_pose_world, key, offsets_px)
            off_of = {}
            if len(offsets_px) == len(grasping_pose_world):
                off_of = {id(p): o for p, o in zip(grasping_pose_world, offsets_px)}
            if candidate is None:
                cprint(f"[grasp] {key} 上 {len(grasping_pose_world)} 个候选孪生一个都规划不过去, "
                       f"换下一个观察位姿", "yellow")
                continue
            cprint(f"=================== 候选: 倾角 {self._tilt_deg(candidate):.1f}° 离框中心 "
                   f"{off_of.get(id(candidate), float('nan')):.0f}px "
                   f"(共 {len(grasping_pose_world)} 个候选) ===================", "yellow")
            check = self._execute_grasping_twin_js_2(candidate, idx=key)
            # 夹住了就收工。少了这句, 外层会继续去下一个观察位姿 —— 手里已经攥着果子
            # 了, 那边当然"检不出目标", 于是报"东西被挪了"停下来问人。2026-10-06 抓
            # 黄瓜 (seen_from=[grasp2,grasp3]) 就是这么假失败的。
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
            names = ["desk_pose_1", "desk_pose_2", "desk_pose_3"]
            random.shuffle(names)
            selected, traj = None, None
            for name in names:
                traj = self._desk_transfer(name)
                if traj is not None:
                    selected = name
                    break
            if traj is None:
                cprint("D=== 三个桌位姿孪生都规划不过去, 放弃 —— 不裸发关节角硬走 ===", "red")
                return False
            cprint(f"D=== 去 {selected}: 孪生校验过的转移路径, {len(traj)} 个路点 ===", "cyan")
            self.control_arm(trajectory=traj, speed=15)
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
        offsets_px = []          # 手爪点离框中心多少像素, 和 final_grasps 一一对应
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
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                for i in sorted_indices:
                    g_pose = grasp_pose_cam[i]
                    final_grasps.append(g_pose)
                    offsets_px.append(float(np.hypot(grasp_points[i][0] - cx,
                                                     grasp_points[i][1] - cy)))
                cprint(
                    f"*********** Class name {class_names} ****************** "
                    f"Grasp pose number: {len(final_grasps)} ******************", "green")
                ans = True
            else:
                cprint(f"Found objects ({class_name}) but NO grasp points inside them.", "yellow")
        else:
            cprint(f"No object detected for class: {class_name}", "yellow")

        return (final_grasps if ans else []), bool(len(detections)), offsets_px

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

    def _stop_map_mismatch(self, target, pose, scene):
        """地图说看得见、现场看不见 —— 停下问人, 不猜、不回退、不换下一个位姿。

        回退到四位姿循环正是这次要消灭的"重复移动", 而且它会把"地图错了"这个信号悄悄
        吃掉, 下次还是错的。
        """
        cprint(f"[map] 地图说 '{target}' 在 {pose} 看得见 (扫描于 {scene.get('scanned_at')}), "
               f"但现场在 {pose} 没检出它。是东西被挪了, 还是地图过期了? "
               f"停下, 请你确认。", "red")

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

    @staticmethod
    def _tilt_deg(pose_world):
        """这个抓取位姿的接近轴偏离竖直多少度。0° = 正着往下扎。"""
        return float(np.degrees(np.arccos(np.clip(abs(pose_world[2, 2]), -1.0, 1.0))))

    def _desk_transfer(self, name):
        """从此刻的位姿去桌位姿 ``name`` 的关节轨迹, **孪生校验过**; 不行返回 None。

        桌面模式原来只发一个裸关节角 (``control_arm(trajectory=[joints])``), 臂自己
        在关节空间插值, 中间会不会扫过桌面没人知道。用户 2026-10-05: "上次重录后
        机械臂在位姿间转移时夹爪撞到桌面" —— 就是这条路。

        **为什么倒着问孪生**: 孪生的 ``target_pose`` 是笛卡尔的, 而桌位姿在 config 里
        存的是关节角。正着问 (此刻 -> 桌位姿) 得先有桌位姿的末端笛卡尔坐标, 那要 FK,
        孪生没这个接口 (它的 ``trajectory_ee[0]`` 只给 current_js 的 FK)。倒着问只要
        **此刻观察位姿**的末端坐标 —— 那个抓取那一步已经从 ``trajectory_ee[0]`` 里
        顺带拿到了。孪生给回来的关节轨迹倒过来执行, 就是"此刻 -> 桌位姿", 而且每个
        路点都是它自己检查过无碰撞的。

        拿不到观察位姿的末端坐标 (没走抓取那步 / 孪生没给) 就返回 None —— 宁可不放,
        不猜。
        """
        obs_ee = getattr(self, "_obs_ee_world", None)
        obs_js = getattr(self, "_obs_js", None)
        desk = self.config.get_pose(name)
        if obs_ee is None or obs_js is None or desk is None:
            return None
        desk_js = [desk[f"J{i}"] / 180 * np.pi for i in range(1, 7)]

        # 第一次: 从桌位姿问回观察位姿。这一问既是"够不够得着 + 会不会撞"的判据,
        # 又顺手把桌位姿的末端笛卡尔位姿换出来了 —— trajectory_ee[0] 就是 current_js
        # 的 FK, 而孪生没有单独的 FK 接口。
        back = self._twin_plan(desk_js, obs_ee)
        if back is None:
            return None

        # 第二次: 从臂**真正在的**关节角出发规划过去。不能拿第一次的轨迹倒过来用 ——
        # 那条轨迹的起点是孪生对观察位姿的 IK 解, 跟 config 里录的差 J4 12°、J6 10°
        # (同一个末端位姿, 不同的关节解), 倒着执行臂会先在原地拧一下手腕。
        fwd = self._twin_plan(obs_js, back["trajectory_ee"][0])
        if fwd is None:
            return None
        return np.array(fwd["trajectory"]) / np.pi * 180

    def _twin_plan(self, current_js, target_pose):
        """问孪生要一条 current_js -> target_pose 的轨迹。规划不出来返回 None。"""
        cnfg = {"target_pose": [list(target_pose)], "current_js": list(current_js),
                "struct": "rm65"}
        rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference",
                                             "type": "trajectory_generation2", "cnfg": cnfg})
        return rsp["info"] if rsp["value"] else None

    def _plan_grasp(self, grasping_pose_world_hand, idx):
        """问孪生: 这个候选够得着吗。返回孪生的响应 (value / trajectory / trajectory_ee)。

        **只问, 不动机器人。** 挑候选时拿它把"规划不过去"的候选筛掉, 于是每个观察位姿
        上仍然只有一次真实抓取 —— 用户 2026-10-06: "一次抓不到直接继续, 不要再跑候选"。
        筛掉的是根本够不着的, 不是"试了没夹住"的。
        """
        # 高度先压到物体的最宽处 (见 _grasp_height), 再算末端目标。预抓取也用同一个
        # 压过的点 —— 否则预抓取停在果子顶上 4cm, 执行点却在果子中间, 是斜着扎下去的。
        pose = grasping_pose_world_hand.copy()
        pose[2, 3] = max(_grasp_height(pose[2, 3]), 0.042)
        execution_grasping_pos = pose @ self.T_hand_effector_to_arm_endlink
        exec_pos = execution_grasping_pos[:3, 3]
        exec_orn = R.from_matrix(execution_grasping_pos[:3, :3]).as_quat()

        default_traj_js_rad = [data / 180 * np.pi for data in self.config.default_traj_js[idx].values()]
        rsp = None
        for along, up in PRE_GRASP_OFFSETS:
            pre = _pre_grasp_pose(pose, along, up, self.T_hand_effector_to_arm_endlink)
            pre_pos = pre[:3, 3]
            pre_orn = R.from_matrix(pre[:3, :3]).as_quat()
            cnfg = {
                "target_pose": [
                    [pre_pos[0], pre_pos[1], pre_pos[2],
                     pre_orn[0], pre_orn[1], pre_orn[2], pre_orn[3]],
                    [exec_pos[0], exec_pos[1], exec_pos[2],
                     exec_orn[0], exec_orn[1], exec_orn[2], exec_orn[3]],
                ],
                "current_js": default_traj_js_rad,
                "struct": "rm65",
            }
            rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
            if rsp["value"]:
                if (along, up) != PRE_GRASP_OFFSETS[0]:
                    cprint(f"=================== 预抓取抬 {along + up:.3f}m 才够得着 "
                           f"(默认 0.04m) ===================", "yellow")
                break
        return rsp

    def _pick_reachable(self, poses_world, idx, offsets_px=None):
        """挑一个**够得着**的候选: 先要**抓准** (手爪点落在物体上), 再要**抓正** (下爪竖直)。

        只在"差不多一样准"的那批里挑 (容差见 CENTER_TOL), 那批一个都够不着才放宽到全部。
        抓准优先于抓正: 用户 2026-10-07 "都抓的偏前" —— 只看倾角会挑中离物体中心
        89px 的候选 (实测火龙果 grasp3: 最竖直的那个离框中心 89px, 而最准的只有 12px)。

        原来只挑最竖直的那个, 于是 r≈0.45 的火龙果上选中的是倾角 3°
        的候选 —— 工具竖直意味着 link_6 还要沿工具轴往外 172mm, 那个位置机械臂够不到,
        孪生报 delta_xyz=24mm 不可达, 一次抓取都没发生就"失败"了; 同一个果子上倾角
        24~49° 的候选孪生全都能规划 (2026-10-07 实测 7/10 可达)。竖直不是目的, 够得着
        才是 —— 把孪生的判决提到**挑选**阶段, 而不是挑完了才发现够不着。
        """
        pairs = list(zip(poses_world, offsets_px or [0.0] * len(poses_world)))
        for tol in (CENTER_TOL, float("inf")):
            best = min(o for _, o in pairs)
            near = [p for p, o in pairs if o <= best * tol]
            for pose in sorted(near, key=lambda T: -abs(T[2, 2])):
                if self._plan_grasp(pose, idx)["value"]:
                    return pose
        return None

    def _execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None):
        """Exact replica of planner.py execute_grasping_twin_js_2."""
        # 张不开就别往下走: 合拢的手指按抓取位姿压下去, 果子被推开, 后面 close 又是一次
        # 空合, 日志里只剩一句 "no object detected", 看着像对不准。control_hand("open")
        # 会读回实际开度, 张不开时挂 opened=False。
        if self.control_hand(cmd_type="open").get("opened") is False:
            cprint("=================== 夹爪张不开, 放弃这次抓取 ===================", "red")
            return False
        default_traj_js_rad = [data / 180 * np.pi for data in self.config.default_traj_js[idx].values()]
        rsp = self._plan_grasp(grasping_pose_world_hand, idx)
        state = rsp["value"]

        if state:
            # 孪生的 trajectory_ee[0] 就是 current_js (= 观察位姿) 的 FK。白捡的,
            # 桌面模式校验转移路径要用 (见 _desk_transfer)。
            self._obs_ee_world = list(rsp["info"]["trajectory_ee"][0])
            self._obs_js = default_traj_js_rad
            trajectory_grasping = rsp["info"]["trajectory"]
            trajectory_grasping = np.array(copy.deepcopy(trajectory_grasping)) / np.pi * 180
            # 20 -> 30: 用户 2026-10-07 "抓的速度也慢了点"。跟观察位姿之间的移动同一个档。
            self.control_arm(trajectory=trajectory_grasping, speed=30)
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
