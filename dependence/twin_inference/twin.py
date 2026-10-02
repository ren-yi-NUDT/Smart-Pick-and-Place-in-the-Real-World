#!/home/zz/anaconda3/envs/anygrasp/bin/python3.9

import os, sys
sys.path.append(os.path.dirname(__file__))
import rospy
import numpy as np
import json
import time
import pybullet as p
import pybullet_data
import threading
import copy
import socket, struct

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from termcolor import cprint

from robot import ErdaijiRobot
import tqdm
from sim_world import World
from utils import save_dict_to_json, slerp, draw_axis, calcul_transformation_matrix
# from simulation_bringup.srv import StringRequest, StringRequestResponse
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
# pynput 只给 __main__ 里的"按 ` 退出"用, 放在 __main__ 里导入,
# 免得 socket 服务/单元测试因为缺这个包就起不来。

SIMULATION_STEP_DELAY = 1 / 240.
ROUND_NUMBER = 3

def visualize_pose(pos, orn, d = 1):
    T = calcul_transformation_matrix(pos, orn)
    draw_axis(T, duration=d)


def insert_lift_waypoints(target_pos_list, target_orn_list, curr_pos, min_z_list,
                          z_min_threshold=0.10, z_lift_margin=0.02):
    """
    给"要从别处横向过去、又贴桌面"的目标点补一个抬升路点。

    位姿本身不用改: 路点放在目标点正上方, 抬升量按该目标点实测的最低 link
    (min_z_list, 世界坐标系, 桌面=0) 反推, 保证横向那一段整条都在
    z_min_threshold 之上。纯垂直下探(与上一个点 XY 相同)不补 —— 抓取/放桌面
    本来就要贴到桌面, 补了反而够不着。

    target_pos_list 是机器人坐标系, min_z_list 是世界坐标系; 抬升是纯 Z 平移,
    机器人基座没有横滚/俯仰, 所以这个增量在两个坐标系里等价。

    返回 (pos_list, orn_list, is_lift_list), 三者等长。
    """
    pos_out, orn_out, lift_out = [], [], []
    ref_pos = np.array(curr_pos, dtype=float)
    for pos_i, orn_i, min_z in zip(target_pos_list, target_orn_list, min_z_list):
        tgt = np.array(pos_i, dtype=float)
        lateral = np.linalg.norm(tgt[:2] - ref_pos[:2]) > 1e-3
        if min_z < z_min_threshold and lateral:
            rise = max(0.0, z_min_threshold - min_z) + z_lift_margin
            lift = [tgt[0], tgt[1], tgt[2] + rise]
            pos_out.append(lift)
            orn_out.append(list(orn_i))
            lift_out.append(True)
            ref_pos = np.array(lift, dtype=float)
        pos_out.append(list(pos_i))
        orn_out.append(list(orn_i))
        lift_out.append(False)
        ref_pos = tgt
    return pos_out, orn_out, lift_out

SUPPORTED_TWIN_SRV_TYPE = ["reachability_check", "collision_check", "IK_calculation","trajectory_generation" , "trajectory_generation2"]

class TwinTest2(World):
    def __init__(self, vis=True):
        self.vis = vis
        self.camera_refresh_freq = 50
        urdf_dir = os.path.join(os.path.dirname(__file__), "../smart_pick_and_place_ws/src/rm_description/urdf/SingleArm")
        self.robot_path = os.path.join(urdf_dir, "rm65.urdf")
        self.robot_config_path = os.path.join(urdf_dir, "robot_config.json")
        self.robot = ErdaijiRobot((0.0, 0.0, 0.0), (0, 0, 0), robot_path = self.robot_path, config_path = self.robot_config_path, fixed_robot = True, vis=self.vis)        

        self.srv_name = "twin_inference"
        self.server_ip = '0.0.0.0'
        self.server_port = 8020
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(5)

        self.socket_thread = threading.Thread(target=self.socket_server_worker)
        self.socket_thread.daemon = True 
        self.socket_thread.start()

        self.setup()

    def socket_server_worker(self):
        """Socket 监听循环"""
        while not rospy.is_shutdown():
            try:
                self.server_socket.settimeout(1.0)
                try:
                    conn, addr = self.server_socket.accept()
                    self.handle_client(conn)
                except socket.timeout:
                    continue
            except Exception as e:
                print(f"Socket Server Error: {e}")
                rospy.sleep(1)

    def handle_client(self, conn):
        """处理单个客户端请求"""
        with conn:
            while not rospy.is_shutdown():
                try:
                    data = conn.recv(1024)
                    if not data: break
                    
                    msg_str = data.decode('utf-8')
                    data_bytes = self.twin_callback(msg_str).encode('utf-8')
                    length_prefix = struct.pack('>I', len(data_bytes))
                    conn.sendall(length_prefix)
                    conn.sendall(data_bytes)
                    # conn.sendall(response)
                except Exception as e:
                    print(f"Client handler error: {e}")
                    break

    def twin_callback(self, req):
        data = json.loads(req)
        srv_name = data["srv"] if "srv" in data else None
        srv_type = data["type"] if "type" in data else None
        srv_config = data["cnfg"] if "cnfg" in data else None
        if srv_type not in SUPPORTED_TWIN_SRV_TYPE or srv_config is None or srv_name!= self.srv_name:
            cprint(f"WRONG REQUEST TO TWIN INFERENCER, req:{data}\n", "red")
            return_dict = {}
            return_dict["srv"] = self.srv_name
            return_dict["value"] = 0
            return_dict["info"] = {}
            return_dict_str = json.dumps(return_dict)
            return return_dict_str
        
        try:
            struct_name = srv_config["struct"]
            target_pose = srv_config["target_pose"]
            curr_js = srv_config["current_js"]
            cprint(f"struct_name::{struct_name}")
            cprint(f"target_pose::{target_pose}")
            cprint(f"curr_js::{curr_js}")
        except:
            cprint(f"NEED MORE CONFIG INFORMATION FOR TWIN INFERENCER, req:{data}\n")
            return_dict = {}
            return_dict["srv"] = self.srv_name
            return_dict["value"] = 0
            return_dict["info"] = {}
            return_dict_str = json.dumps(return_dict)
            return return_dict_str
        
        if struct_name != "rm65" or len(curr_js) != 6:
            return json.dumps({"srv": self.srv_name, "value": 0, "info": {}})

        if srv_type == "reachability_check":
            check, info = self.reachability_check(srv_config)
        elif srv_type == "collision_check":
            check, info = self.reachability_check(srv_config)
        elif srv_type == "IK_calculation":
            check, info = self.reachability_check(srv_config)
        elif srv_type == "trajectory_generation":
            check, traj, traj_ee, infos = self.linear_traj_generation(srv_config)
            info = {}
            info["trajectory"] = traj
            info["trajectory_ee"] = traj_ee
            info["infos"] = infos
        elif srv_type == "trajectory_generation2":
            check, traj, traj_ee, infos = self.linear_traj_generation2(srv_config)
            info = {}
            info["trajectory"] = traj
            info["trajectory_ee"] = traj_ee
            info["infos"] = infos
        else:
            cprint(f"TWIN INFERENCER DOES NOT  SUPPORT CURRENT TYPE REQUEST: {data}\n")
            check = False
            info = {}
        
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()    
            
        return_dict = {}
        return_dict["srv"] = srv_name
        return_dict["value"] = int(check)
        return_dict["info"] = info
        return_dict_str = json.dumps(return_dict)
        return return_dict_str
    
    def reachability_check(self, config):
    
        struct_name = config["struct"]
        target_pose = config["target_pose"]
        curr_js = config["current_js"]
        if struct_name != "rm65":
            rospy.logwarn(f"TWIN INFERENCER 'reachaibility' DOES NOT  SUPPORT CURRENT STRUCT: {struct_name}")
            return 0, {}
        
        # reset robot to target pose
        pos, orn = target_pose[:3], target_pose[3:]
        if self.vis:
            visualize_pose(pos, orn)

        arm = self.robot.robot_structs[struct_name]
        arm.reset_by_ee_pose_self(pos, orn, js = curr_js)

        # 同 linear_traj_generation2: 检查与取关节角都在 step 之前 (step 会把臂甩离 IK 解)
        target_js = arm.get_joint_pose()
        is_reach, delta_xyz, delta_rpy = arm.check_reach(pos, orn, xyz_threshold =0.01, rpy_threshold = 0.01)

        self.step()
        
        is_collide, collide_id = self.robot.check_collision()
        
        # update info
        check = is_reach and (not is_collide)
        info ={}
        info["is_reached"] = int(is_reach)
        info["delta_xyz"] = delta_xyz
        info["delta_rpy"] = delta_rpy
        info["target_js"] = list(target_js)
        info["is_collided"] = int(is_collide)
        info["collide_body"] = collide_id
        
        print("_________________________________________\n")
        rospy.loginfo(f"REACHABILITY CHECK RESULT:")
        for k, v in info.items():
            rospy.loginfo(f" {k}:  {v}")
        print("_________________________________________\n")
        return check,info
    
    
    def linear_traj_generation(self, config):
                
        struct_name = config["struct"]
        target_pose = config["target_pose"]
        curr_js = config["current_js"]
        interval_threshold = config["interval_threshold"] if "interval_threshold" in config else 0.05
        if struct_name != "rm65":
            rospy.logwarn(f"TWIN INFERENCER 'trajectory generation' DOES NOT  SUPPORT CURRENT STRUCT: {struct_name}")
            return 0, {}
        
        # get target
        target_pos, target_orn = target_pose[:3], target_pose[3:]
        if self.vis:
            visualize_pose(target_pos, target_orn)
                    
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()
        
        # get curr
        curr_pos, curr_orn = self.robot.robot_structs[struct_name].get_end_link_pose_self()
        # curr_R = R.from_quat(curr_orn)        
        
        delta_xyz = np.linalg.norm(target_pos - curr_pos)
        loop_num  = max(int(np.floor(delta_xyz/interval_threshold)), 1)
        intermediate_js = copy.copy(curr_js)
        trajectory  = []
        trajectory.append(curr_js)
        infos = []
        intermediate_orn_list = slerp(curr_orn, target_orn, loop_num+1)
        trajectory_ee = []
        
        for l in range(loop_num):
             # get and visualize intermediate pose
            intermediate_pos= curr_pos + (target_pos - curr_pos)*((l+1)/loop_num)
            intermediate_orn = intermediate_orn_list[l + 1]#R.slerp((l+1)/20, curr_R, target_R).as_quat() 
            if self.vis:
                visualize_pose(intermediate_pos, intermediate_orn, d = 100)
            
            # reset robot to interval pose
            arm = self.robot.robot_structs[struct_name]
            prev_js = np.array(intermediate_js, dtype=float)
            arm.reset_by_ee_pose_self(intermediate_pos, intermediate_orn, js = intermediate_js)

            # 同 linear_traj_generation2: 先读构型再 step (trajectory 里存的是 intermediate_js)
            intermediate_js = arm.get_joint_pose()

            # 同 linear_traj_generation2: 相邻路点换腕分支 -> 真机会扫过桌面, 直接拒发
            jump = float(np.max(np.abs(intermediate_js - prev_js)))
            if jump > np.radians(90):
                rospy.logwarn(f"BRANCH_JUMP {np.degrees(jump):.1f}deg at {intermediate_pos} "
                              f"CANNOT GENERATE TRAJECTORY")
                return 0, trajectory, trajectory_ee, infos
            is_reach, delta_xyz, delta_rpy = arm.check_reach(intermediate_pos, intermediate_orn
                                                             , xyz_threshold =0.01, rpy_threshold = 0.01)

            is_collide, collide_id = self.robot.check_collision()

            self.step()
            
            trajectory.append(list(intermediate_js))
            trajectory_ee.append(list(intermediate_pos) + list(intermediate_orn))

            info = {}
            info["intermediate_pose"] = list(intermediate_pos) + list(intermediate_orn)
            info["is_reached"] = int(is_reach)
            info["delta_xyz"] = delta_xyz
            info["delta_rpy"] = delta_rpy
            info["is_collided"] = int(is_collide)
            info["collide_body"] = collide_id
            infos.append(info)
            if is_collide:
                rospy.logwarn(f"COLLISION AT pos:{intermediate_pos}, {intermediate_orn}, CANOT GENERATE TRAJECTORY FOR TARGET:{target_pose}")
                return 0, trajectory, trajectory_ee, infos

        rospy.loginfo(f"TRAJECTORY GENERATED FOR CURENT POSE:{[curr_pos, curr_orn]} TO TARGET POSE:{[target_pos, target_orn]}")
        return 1, trajectory, trajectory_ee, infos
    
    def check_arm_joints_z_height(self, struct_name, z_min_threshold=0.10):
        """
        检查机械臂所有 link 的 Z 高度是否低于阈值

        两个曾经的坑 (2026-09-30 修):
          1. 原来只遍历 Arm struct 的 link_names (= link_1..link_6), 夹爪那 10 个 link
             一个都不查 —— 而夹爪比最低臂杆还低 8~13cm, 盲区正好是夹爪长度。
             实测 grasp2: 臂 link 最低 0.105 刚好过 0.10 的线, 夹爪最低 0.000 贴桌面,
             Z 检查报"安全"而夹爪已经坐在桌上。
          2. 原来用 getLinkState()[0] (关节原点), 不是几何最低点, 本身就低估。
             改用 getAABB 的最低角点。

        Args:
            struct_name: 机械臂结构名称 (rm65)
            z_min_threshold: Z高度最小阈值，默认0.10m

        Returns:
            is_safe: 是否安全 (所有 link 几何最低点 > z_min_threshold)
            unsafe_links: 低于阈值的 link 列表 [[link_name, z_value], ...]
            min_z: 最低的Z值
        """
        body = self.robot.id
        unsafe_links = []
        min_z = float('inf')

        for link_id in range(p.getNumJoints(body)):
            link_name = p.getJointInfo(body, link_id)[12].decode()
            # 跳过机器人底座本体; 夹爪的 gripper_base 要看 (它下面就是手指)
            if link_name in ("base_link", "robot_base_link", "arm_end_link"):
                continue

            link_z = p.getAABB(body, link_id)[0][2]  # 几何最低点, 不是关节原点
            min_z = min(min_z, link_z)

            if link_z < z_min_threshold:
                unsafe_links.append([link_name, round(link_z, 4)])

        is_safe = len(unsafe_links) == 0
        return is_safe, unsafe_links, round(min_z, 4)

    def linear_traj_generation2(self, config):

        struct_name = config["struct"]
        target_pose_list = config["target_pose"]
        curr_js = config["current_js"]
        interval_threshold = config["interval_threshold"] if "interval_threshold" in config else 0.05
        loose_constraint = config["loose_constraint"] if "loose_constraint" in config else 0
        z_min_threshold = config["z_min_threshold"] if "z_min_threshold" in config else 0.10  # 横向转移的安全高度
        # 硬检查阈值: 低于它才算真的插进桌面。抓取/放桌面本来就要贴到桌面,
        # 所以目标点和轨迹点的检查用这个 (0 = 桌面平面), 而不是 z_min_threshold。
        z_floor_threshold = config["z_floor_threshold"] if "z_floor_threshold" in config else 0.0
        z_lift_margin = config["z_lift_margin"] if "z_lift_margin" in config else 0.02
        if struct_name != "rm65":
            rospy.logwarn(f"TWIN INFERENCER 'trajectory generation' DOES NOT  SUPPORT CURRENT STRUCT: {struct_name}")
            return 0, {}
        
        # get target
        target_pos_list = []
        target_orn_list = []
        for target_pose in target_pose_list:
            target_pos, target_orn = target_pose[:3], target_pose[3:]
            target_pos_list.append(target_pos)
            target_orn_list.append(target_orn)
            if self.vis:
                visualize_pose(target_pos, target_orn)
                    
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()
        # reset robot to target_poses
        infos = []
        reaches = []
        collides = []
        z_safes = []
        js_fake = copy.deepcopy(curr_js)
        for target_pos, target_orn in zip(target_pos_list, target_orn_list):
            arm = self.robot.robot_structs[struct_name]
            arm.reset_by_ee_pose_self(target_pos, target_orn, js = js_fake)

            # 所有检查都必须在 step 之前做。move_joint 用的是 POSITION_CONTROL 的默认
            # positionGain=0.1, 撑不住重力: 实测一步 stepSimulation (4ms) 就能让 J4 走
            # 0.42rad (正好 maxvel*dt), 末端偏离 IK 解 40mm。在 step 之后再读 check_reach,
            # 读到的是"被重力甩开之后的位姿", 于是明明 IK 命中 0.1mm 的抓取候选全判不可达。
            # 可达性问的是"IK 解这个构型能不能到位", 就该在解上问, 不该问重力把臂拖到哪了。
            # z 高度检查同理 —— 漂移是往下塌的, 会把本来贴桌面的夹爪报成"插进桌面"。
            js_fake = arm.get_joint_pose()
            is_reach, delta_xyz, delta_rpy = arm.check_reach(target_pos, target_orn
                                                             , xyz_threshold =0.01, rpy_threshold = 0.01)
            is_collide, collide_id = self.robot.check_collision()
            # 检查关节Z高度是否安全
            is_z_safe, unsafe_links, min_z = self.check_arm_joints_z_height(struct_name, z_floor_threshold)

            self.step()
            info = {}
            info["is_reached"] = int(is_reach)
            info["delta_xyz"] = delta_xyz
            info["delta_rpy"] = delta_rpy
            info["is_collided"] = int(is_collide)
            info["collide_body"] = collide_id
            info["is_z_safe"] = int(is_z_safe)
            info["unsafe_links"] = unsafe_links
            info["min_z"] = min_z
            info["js_fake"] = list(js_fake)
            info["ee_fake"] = target_pos + target_orn
            if self.vis:
                print(info)
                print("-"*20)
            infos.append(info)
            reaches.append(int(is_reach))
            collides.append(is_collide)
            z_safes.append(is_z_safe)

            # Z高度不安全则直接失败
            if not is_z_safe:
                rospy.logwarn(f"Z_HEIGHT_UNSAFE: Links {unsafe_links} below {z_floor_threshold}m at target pose {target_pos}")
                return 0, [], [], infos

        if loose_constraint:
            if not reaches[-1]: # or any(is_collide == 1 for is_collide in collides):
                return 0, [], [], infos
        else:
            if any(is_reach == 0 for is_reach in reaches): # or any(is_collide == 1 for is_collide in collides):
                return 0, [], [], infos
        
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()
        
        # get curr
        curr_pos, curr_orn = self.robot.robot_structs[struct_name].get_end_link_pose_self()

        # 低目标点(贴桌面的放桌面位姿、低观察位姿)如果要从别处横向过去, 先补一个
        # "抬到安全高度 -> 平移 -> 垂直下探" 的路点。位姿本身不用改, 横向那一段
        # 整条都在 z_min_threshold 之上。抬升量按该目标点实测的最低 link 反推,
        # 所以不依赖任何魔数。纯垂直下探(与上一个点 XY 相同)不补 —— 抓取要贴桌面。
        # ponytail: 抬升路点不单独做可达性预检, 交给下面主循环的逐步检查兜底 ——
        # 真不可达时 check_collision 会返回 0, 上层看到 value=0 不会执行任何动作。
        target_pos_list, target_orn_list, _ = insert_lift_waypoints(
            target_pos_list, target_orn_list, curr_pos,
            [info["min_z"] for info in infos], z_min_threshold, z_lift_margin)

        trajectory  = []
        trajectory_ee = []
        infos = []
        trajectory.append(curr_js)
        trajectory_ee.append(list(curr_pos)+list(curr_orn))
        
        fake_pos = copy.copy(curr_pos)
        fake_orn = copy.copy(curr_orn)
        fake_js = copy.copy(curr_js)
        
        for target_pos, target_orn in zip(target_pos_list, target_orn_list):
            # generate traj from fake pose to target
            target_pos_arr, target_orn_arr = np.array(target_pos), np.array(target_orn)
            delta_xyz = np.linalg.norm(target_pos_arr - fake_pos)
            loop_num  = max(int(np.floor(delta_xyz/interval_threshold)), 1)
            
            intermediate_pos_list = [fake_pos + (target_pos_arr - fake_pos)*((l+1)/loop_num) for l in range(loop_num)]
            intermediate_orn_list = slerp(fake_orn, target_orn_arr, loop_num+1)[1:]
            intermediate_js = copy.copy(fake_js)
            
            for l in range(loop_num):
                # get and visualize intermediate pose
                intermediate_pos = intermediate_pos_list[l]
                intermediate_orn = intermediate_orn_list[l]
                if self.vis:
                    visualize_pose(intermediate_pos, intermediate_orn, d = 10)
                    time.sleep(0.1)
                
                # reset robot to interval pose
                arm = self.robot.robot_structs[struct_name]
                prev_js = np.array(intermediate_js, dtype=float)
                arm.reset_by_ee_pose_self(intermediate_pos, intermediate_orn, js = intermediate_js)

                # 同 381 行: 先读构型再 step。这里更要紧 —— trajectory 里存的是
                # intermediate_js, 而这条轨迹是要发给真机的。在 step 之后取, 取到的是
                # 仿真里被甩开 0.4rad 的关节角, 等于把错的目标点发给机械臂。
                intermediate_js = arm.get_joint_pose()

                # 分支一致性。enforce_joint_limits 只保证"这个点本身合法"; 如果相邻
                # 路点换了腕部分支 (J4 一次跳 ~180°), 真机照着走会让整条臂扫过桌面。
                # 相邻路点间距恒为 interval_threshold (~50mm), 这个尺度下 90° 的关节
                # 跳变只可能来自换分支。判不可用 -> 上层去试下一个观察位姿, 而不是
                # 发一条会把夹爪甩飞的轨迹。
                jump = float(np.max(np.abs(intermediate_js - prev_js)))
                if jump > np.radians(90):
                    rospy.logwarn(f"BRANCH_JUMP {np.degrees(jump):.1f}deg at {intermediate_pos} "
                                  f"(prev={np.degrees(prev_js).round(1)} now={np.degrees(intermediate_js).round(1)}) "
                                  f"CANNOT GENERATE TRAJECTORY")
                    return 0, trajectory, trajectory_ee, infos
                is_reach, delta_xyz, delta_rpy = arm.check_reach(intermediate_pos, intermediate_orn
                                                                 , xyz_threshold =0.01, rpy_threshold = 0.01)
                is_collide, collide_id = self.robot.check_collision()
                # 检查关节Z高度
                is_z_safe, unsafe_links, min_z = self.check_arm_joints_z_height(struct_name, z_floor_threshold)

                self.step()


                trajectory.append(list(intermediate_js))
                trajectory_ee.append(list(intermediate_pos) + list(intermediate_orn))

                info = {}
                info["intermediate_pose"] = list(intermediate_pos) + list(intermediate_orn)
                info["is_reached"] = int(is_reach)
                info["delta_xyz"] = delta_xyz
                info["delta_rpy"] = delta_rpy
                info["is_collided"] = int(is_collide)
                info["collide_body"] = collide_id
                info["is_z_safe"] = int(is_z_safe)
                info["unsafe_links"] = unsafe_links
                info["min_z"] = min_z
                if self.vis:
                    print(info)
                    print("-"*20)
                infos.append(info)

                print(info)

                if is_collide:
                    rospy.logwarn(f"COLLISION AT pos:{intermediate_pos}, {intermediate_orn}, CANOT GENERATE TRAJECTORY FOR TARGET:{target_pose}")
                    return 0, trajectory, trajectory_ee, infos

                if not is_z_safe:
                    rospy.logwarn(f"Z_HEIGHT_UNSAFE: Links {unsafe_links} below {z_floor_threshold}m at pos:{intermediate_pos}")
                    return 0, trajectory, trajectory_ee, infos
            
            fake_pos = np.array(target_pos)
            fake_orn = np.array(target_orn)
            fake_js = copy.copy(intermediate_js)
            

        rospy.loginfo(f"TRAJECTORY GENERATED FOR CURENT POSE:{[curr_pos, curr_orn]} TO TARGET POSE:{[target_pos, target_orn]}")
        return 1, trajectory, trajectory_ee, infos

    def linear_traj_generation3(self, config):
        struct_name = config["struct"]
        target_pose_list = config["target_pose"]
        curr_js = config["current_js"]
        interval_threshold = config["interval_threshold"] if "interval_threshold" in config else 0.05
        loose_constraint = config["loose_constraint"] if "loose_constraint" in config else 0
        
        if struct_name != "rm65":
            rospy.logwarn(f"TWIN INFERENCER 'trajectory generation' DOES NOT SUPPORT CURRENT STRUCT: {struct_name}")
            return 0, {}, [], [] # 修正返回格式以匹配下文

        # --- 准备工作 ---
        target_pos_list = []
        target_orn_list = []
        for target_pose in target_pose_list:
            target_pos, target_orn = target_pose[:3], target_pose[3:]
            target_pos_list.append(target_pos)
            target_orn_list.append(target_orn)
            if self.vis:
                visualize_pose(target_pos, target_orn)

        # 获取结构体对象
        arm_struct = self.robot.robot_structs[struct_name]

        # ------------------------------------------------------------------
        # 阶段 1: 关键点预检查 (Pre-check & Anchor Generation)
        # 目的：确保目标点绝对可达，并计算出"无碰撞且合理的"最终关节角度
        # ------------------------------------------------------------------
        
        # 先重置回当前状态
        arm_struct.reset_by_joint_states(curr_js)
        arm_struct.move_joint(curr_js)
        self.step()

        infos = []
        pre_check_reaches = []
        pre_check_js_solutions = [] # 存储预检查计算出的目标关节角
        
        # 使用虚拟变量进行模拟，不影响物理显示的连贯性（如果不需要实时渲染中间过程）
        js_fake = copy.deepcopy(curr_js) 
        
        for target_pos, target_orn in zip(target_pos_list, target_orn_list):
            # 尝试求解 IK
            # 注意：这里我们使用更严格的 solve_robust_ik
            success, solved_js = self.robot.robot_structs[struct_name].solve_robust_ik(target_pos, target_orn, seed_js=js_fake)
            
            # 将模拟机器人设置到解的位置进行碰撞检测
            if success:
                arm_struct.reset_by_joint_states(solved_js)
                js_fake = solved_js # 更新下一个点的种子

            # 验证 Reach (双重确认 IK 误差)。必须在 step 之前 —— step 会把臂甩离 IK 解
            is_reach, delta_xyz, delta_rpy = arm_struct.check_reach(target_pos, target_orn, xyz_threshold=0.015, rpy_threshold=0.05)

            if success:
                self.step() # 更新物理引擎状态 (碰撞检测用)
            # 验证 Collision
            is_collide, collide_id = self.robot.check_collision()
            
            # 综合判定
            # 如果 IK 求解报告成功，但实际物理误差大或者碰撞，则视为不可行
            final_reach_status = 1 if (success and is_reach and not is_collide) else 0

            info = {
                "is_reached": int(final_reach_status),
                "delta_xyz": delta_xyz,
                "delta_rpy": delta_rpy,
                "is_collided": int(is_collide),
                "collide_body": collide_id,
                "js_fake": list(js_fake) if success else [],
                "ee_fake": list(target_pos) + list(target_orn)
            }
            if self.vis:
                print(f"[Pre-check] Reach:{final_reach_status}, Collide:{is_collide}, Err:{delta_xyz:.4f}")

            infos.append(info)
            pre_check_reaches.append(final_reach_status)
            pre_check_js_solutions.append(solved_js if success else None)

        # 宽松约束与严格约束判断
        if loose_constraint:
            if not pre_check_reaches[-1]: # 只要最后一个点不可达就失败
                 rospy.logwarn("Target Unreachable in Loose Mode.")
                 return 0, [], [], infos
        else:
            if any(r == 0 for r in pre_check_reaches): # 任何中间关键点不可达就失败
                rospy.logwarn("Target Unreachable in Strict Mode.")
                return 0, [], [], infos

        # ------------------------------------------------------------------
        # 阶段 2: 分段线性轨迹生成 (Interpolation)
        # 目的：生成平滑轨迹，并利用阶段 1 的结果作为引导
        # ------------------------------------------------------------------
        
        # 重置回起始点
        arm_struct.reset_by_joint_states(curr_js)
        arm_struct.move_joint(curr_js)
        self.step()
        
        curr_pos, curr_orn = arm_struct.get_end_link_pose_self()        
        
        trajectory_js = [curr_js]
        trajectory_ee = [list(curr_pos) + list(curr_orn)]
        
        fake_pos = np.array(curr_pos)
        fake_orn = np.array(curr_orn)
        fake_js = copy.deepcopy(curr_js)
        
        for idx, (target_pos, target_orn) in enumerate(zip(target_pos_list, target_orn_list)):
            target_pos_arr = np.array(target_pos)
            target_orn_arr = np.array(target_orn)
            
            dist = np.linalg.norm(target_pos_arr - fake_pos)
            
            # 动态步长计算：保证最小步数，防止极短距离导致的跳变
            min_steps = 5
            calc_steps = int(np.floor(dist / interval_threshold))
            loop_num = max(calc_steps, min_steps)

            # 插值
            interp_pos_list = [fake_pos + (target_pos_arr - fake_pos) * ((i+1)/loop_num) for i in range(loop_num)]
            # Slerp 插值需注意：slerp通常返回包括起点的列表，这里取 [1:]
            interp_orn_list = slerp(fake_orn, target_orn_arr, loop_num + 1)[1:]
            
            last_valid_js = copy.deepcopy(fake_js)
            
            for l in range(loop_num):
                t_pos = interp_pos_list[l]
                t_orn = interp_orn_list[l]

                if self.vis:
                    visualize_pose(t_pos, t_orn, d=0.05) # d 缩小，显示更精细

                # --- 核心改进：鲁棒 IK 求解 ---
                # 使用上一步的解 last_valid_js 作为 seed
                success, solved_js = self.solve_robust_ik(struct_name, t_pos, t_orn, seed_js=last_valid_js)
                
                if not success:
                    # 尝试降级策略：如果只是中间点微小误差，可以容忍吗？
                    # 严格模式下，为了保证一定是直线，如果不成功则中断
                    rospy.logerr(f"Traj Gen Failed: IK stuck at step {l}/{loop_num} of segment {idx}")
                    return 0, trajectory_js, trajectory_ee, infos
                
                # --- 核心改进：关节连续性检查 (Anti-Flip) ---
                # 检查最大关节变化量，防止机械臂"翻转"
                js_diff = np.max(np.abs(np.array(solved_js) - np.array(last_valid_js)))
                if js_diff > 1.5: # 阈值根据实际机器人设定，通常相邻帧不应超过 1.5 rad
                    rospy.logerr(f"Traj Gen Failed: Joint Flip detected (diff: {js_diff:.2f})")
                    return 0, trajectory_js, trajectory_ee, infos

                # 模拟执行
                arm_struct.reset_by_joint_states(solved_js)
                self.step()

                # 记录
                last_valid_js = solved_js
                trajectory_js.append(list(solved_js))
                trajectory_ee.append(list(t_pos) + list(t_orn))

            # 更新段间状态
            fake_pos = target_pos_arr
            fake_orn = target_orn_arr
            # 关键：这里可以选择是否强行对齐到 Pre-check 计算出的完美解
            # 为了平滑，我们通常继续使用插值的最后结果，但如果误差累积，可以使用 pre_check_js_solutions[idx] 进行修正
            fake_js = last_valid_js 
            
        rospy.loginfo(f"TRAJECTORY GENERATED SAFELY with {len(trajectory_js)} steps.")
        return 1, trajectory_js, trajectory_ee, infos

    def test_client(self):
        
        data = {
            "srv":self.srv_name,
            "type":"trajectory_generation2",
            "cnfg":{
                "target_pose":[
                    [0.6130359164338649, -0.26390629817473416, 0.9572628420278162, 0.5423872870142016, 0.7882424147151591, 0.21071606926629194, 0.20022153900705178],
                    [0.6130359164338649, -0.18390629817473416, 0.9572628420278162, 0.5423872870142016, 0.7882424147151591, 0.21071606926629194, 0.20022153900705178]        
                               ],
                "current_js":[-1.204, -8.815, 57.494, -3.33, 113.799, -0.012],
                "struct":"rm65"
            }
        }
        req = json.dumps(data)
        rsp = self.node_list.client_service_request(self.srv_name, StringRequest, req)
        rsp = json.loads(rsp.response)
        import pprint
        pprint.pprint(rsp)
        return True
    def start_test(self):
        self.test_thread = threading.Thread(target=self.test_client)
        self.test_thread.start()
        return self.test_thread



def on_press(key):
    global kill_program
    try:
        if key.char == '`':  # Check if 's' is pressed
            print("PROGRAM KILLED BY HUMAN")
            kill_program = True  # Set flag to stop the loop
    except AttributeError:
        pass




if __name__ == '__main__':
    from pynput import keyboard

    world = TwinTest2(vis=True)
    world.reset()

    key_board_listener = keyboard.Listener(on_press=on_press)
    key_board_listener.start()
    
    kill_program = False
    count = 0
    while not kill_program:
        count+= 1
        robot_state = world.step()
         
        
        # print(count)
        # if count == 50:
        #     world.start_test()
        
        # if count == 100:
        #     time.sleep(5)
        #     break